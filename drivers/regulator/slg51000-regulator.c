// SPDX-License-Identifier: GPL-2.0+
//
// SLG51000 High PSRR, Multi-Output Regulators
// Copyright (C) 2019  Dialog Semiconductor
//
// Author: Eric Jeong <eric.jeong.opensource@diasemi.com>

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include "slg51000-regulator.h"

#define SLG51000_SCTL_EVT               7
#define SLG51000_MAX_EVT_REGISTER       8
#define SLG51000_LDOHP_LV_MIN           1200000
#define SLG51000_LDOHP_HV_MIN           2400000

enum slg51000_regulators {
	SLG51000_REGULATOR_LDO1 = 0,
	SLG51000_REGULATOR_LDO2,
	SLG51000_REGULATOR_LDO3,
	SLG51000_REGULATOR_LDO4,
	SLG51000_REGULATOR_LDO5,
	SLG51000_REGULATOR_LDO6,
	SLG51000_REGULATOR_LDO7,
	SLG51000_MAX_REGULATORS,
};

/*
 * GPIO1..4 output control (vendor pinctrl-slg51000.c): direction bit 7 of
 * IO_GPIOn_CONF, level in MUXARRAY_INPUT_SEL_16..19, both only writable in
 * software test mode (0x111a..0x111d = 0x45 0x53 0x54 0x4d, left with
 * SYSCTL_TEST_EN = 0).
 */
#define SLG51000_SYSCTL_TEST_EN			0x1119
#define SLG51000_TEST_EN_OFF			0x00
#define SLG51000_SW_TEST_MODE_1			0x111a
#define SLG51000_SW_TEST_MODE_4			0x111d
#define SLG51000_GPIO_DIR_OUT			BIT(7)
#define SLG51000_GPIO_CTRL(n)			(SLG51000_MUXARRAY_INPUT_SEL_0 + 16 + (n))
#define SLG51000_NUM_GPIOS			4

struct slg51000 {
	struct gpio_chip gc;
	struct device *dev;
	struct regmap *regmap;
	struct regulator_desc *rdesc[SLG51000_MAX_REGULATORS];
	struct regulator_dev *rdev[SLG51000_MAX_REGULATORS];
	struct gpio_desc *cs_gpiod;
	/*
	 * Camera-PMIC master enables driven in a strict order at probe
	 * (see slg51000_i2c_probe). bb/buck power the internal boost/buck
	 * that feed the LV LDOs (ldo5/6/7); pu triggers the power-up
	 * sequencer and must be asserted LAST, after cs is Ready.
	 */
	struct gpio_desc *bb_gpiod;
	struct gpio_desc *buck_gpiod;
	struct gpio_desc *pu_gpiod;
	int chip_irq;
};

struct slg51000_evt_sta {
	unsigned int ereg;
	unsigned int sreg;
};

static const struct slg51000_evt_sta es_reg[SLG51000_MAX_EVT_REGISTER] = {
	{SLG51000_LDO1_EVENT, SLG51000_LDO1_STATUS},
	{SLG51000_LDO2_EVENT, SLG51000_LDO2_STATUS},
	{SLG51000_LDO3_EVENT, SLG51000_LDO3_STATUS},
	{SLG51000_LDO4_EVENT, SLG51000_LDO4_STATUS},
	{SLG51000_LDO5_EVENT, SLG51000_LDO5_STATUS},
	{SLG51000_LDO6_EVENT, SLG51000_LDO6_STATUS},
	{SLG51000_LDO7_EVENT, SLG51000_LDO7_STATUS},
	{SLG51000_SYSCTL_EVENT, SLG51000_SYSCTL_STATUS},
};

static const struct regmap_range slg51000_writeable_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_TEST_EN, SLG51000_SW_TEST_MODE_4),
	regmap_reg_range(SLG51000_IO_GPIO1_CONF, SLG51000_IO_GPIO4_CONF),
	regmap_reg_range(SLG51000_GPIO_CTRL(0), SLG51000_GPIO_CTRL(3)),
	regmap_reg_range(SLG51000_SYSCTL_MATRIX_CONF_A,
			 SLG51000_SYSCTL_MATRIX_CONF_A),
	regmap_reg_range(SLG51000_LDO1_VSEL, SLG51000_LDO1_VSEL),
	regmap_reg_range(SLG51000_LDO1_MINV, SLG51000_LDO1_MAXV),
	regmap_reg_range(SLG51000_LDO1_IRQ_MASK, SLG51000_LDO1_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO2_VSEL, SLG51000_LDO2_VSEL),
	regmap_reg_range(SLG51000_LDO2_MINV, SLG51000_LDO2_MAXV),
	regmap_reg_range(SLG51000_LDO2_IRQ_MASK, SLG51000_LDO2_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO3_VSEL, SLG51000_LDO3_VSEL),
	regmap_reg_range(SLG51000_LDO3_MINV, SLG51000_LDO3_MAXV),
	regmap_reg_range(SLG51000_LDO3_IRQ_MASK, SLG51000_LDO3_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO4_VSEL, SLG51000_LDO4_VSEL),
	regmap_reg_range(SLG51000_LDO4_MINV, SLG51000_LDO4_MAXV),
	regmap_reg_range(SLG51000_LDO4_IRQ_MASK, SLG51000_LDO4_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO5_VSEL, SLG51000_LDO5_VSEL),
	regmap_reg_range(SLG51000_LDO5_MINV, SLG51000_LDO5_MAXV),
	regmap_reg_range(SLG51000_LDO5_IRQ_MASK, SLG51000_LDO5_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO6_VSEL, SLG51000_LDO6_VSEL),
	regmap_reg_range(SLG51000_LDO6_MINV, SLG51000_LDO6_MAXV),
	regmap_reg_range(SLG51000_LDO6_IRQ_MASK, SLG51000_LDO6_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO7_VSEL, SLG51000_LDO7_VSEL),
	regmap_reg_range(SLG51000_LDO7_MINV, SLG51000_LDO7_MAXV),
	regmap_reg_range(SLG51000_LDO7_IRQ_MASK, SLG51000_LDO7_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_IRQ_MASK, SLG51000_OTP_IRQ_MASK),
};

static const struct regmap_range slg51000_readable_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_TEST_EN, SLG51000_SW_TEST_MODE_4),
	regmap_reg_range(SLG51000_SYSCTL_PATN_ID_B0,
			 SLG51000_SYSCTL_PATN_ID_B2),
	regmap_reg_range(SLG51000_SYSCTL_SYS_CONF_A,
			 SLG51000_SYSCTL_SYS_CONF_A),
	regmap_reg_range(SLG51000_SYSCTL_SYS_CONF_D,
			 SLG51000_SYSCTL_MATRIX_CONF_B),
	regmap_reg_range(SLG51000_SYSCTL_REFGEN_CONF_C,
			 SLG51000_SYSCTL_UVLO_CONF_A),
	regmap_reg_range(SLG51000_SYSCTL_FAULT_LOG1, SLG51000_SYSCTL_IRQ_MASK),
	regmap_reg_range(SLG51000_IO_GPIO1_CONF, SLG51000_IO_GPIO_STATUS),
	regmap_reg_range(SLG51000_LUTARRAY_LUT_VAL_0,
			 SLG51000_LUTARRAY_LUT_VAL_11),
	regmap_reg_range(SLG51000_MUXARRAY_INPUT_SEL_0,
			 SLG51000_MUXARRAY_INPUT_SEL_63),
	regmap_reg_range(SLG51000_PWRSEQ_RESOURCE_EN_0,
			 SLG51000_PWRSEQ_INPUT_SENSE_CONF_B),
	regmap_reg_range(SLG51000_LDO1_VSEL, SLG51000_LDO1_VSEL),
	regmap_reg_range(SLG51000_LDO1_MINV, SLG51000_LDO1_MAXV),
	regmap_reg_range(SLG51000_LDO1_MISC1, SLG51000_LDO1_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO1_EVENT, SLG51000_LDO1_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO2_VSEL, SLG51000_LDO2_VSEL),
	regmap_reg_range(SLG51000_LDO2_MINV, SLG51000_LDO2_MAXV),
	regmap_reg_range(SLG51000_LDO2_MISC1, SLG51000_LDO2_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO2_EVENT, SLG51000_LDO2_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO3_VSEL, SLG51000_LDO3_VSEL),
	regmap_reg_range(SLG51000_LDO3_MINV, SLG51000_LDO3_MAXV),
	regmap_reg_range(SLG51000_LDO3_CONF1, SLG51000_LDO3_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO3_EVENT, SLG51000_LDO3_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO4_VSEL, SLG51000_LDO4_VSEL),
	regmap_reg_range(SLG51000_LDO4_MINV, SLG51000_LDO4_MAXV),
	regmap_reg_range(SLG51000_LDO4_CONF1, SLG51000_LDO4_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO4_EVENT, SLG51000_LDO4_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO5_VSEL, SLG51000_LDO5_VSEL),
	regmap_reg_range(SLG51000_LDO5_MINV, SLG51000_LDO5_MAXV),
	regmap_reg_range(SLG51000_LDO5_TRIM2, SLG51000_LDO5_TRIM2),
	regmap_reg_range(SLG51000_LDO5_CONF1, SLG51000_LDO5_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO5_EVENT, SLG51000_LDO5_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO6_VSEL, SLG51000_LDO6_VSEL),
	regmap_reg_range(SLG51000_LDO6_MINV, SLG51000_LDO6_MAXV),
	regmap_reg_range(SLG51000_LDO6_TRIM2, SLG51000_LDO6_TRIM2),
	regmap_reg_range(SLG51000_LDO6_CONF1, SLG51000_LDO6_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO6_EVENT, SLG51000_LDO6_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO7_VSEL, SLG51000_LDO7_VSEL),
	regmap_reg_range(SLG51000_LDO7_MINV, SLG51000_LDO7_MAXV),
	regmap_reg_range(SLG51000_LDO7_CONF1, SLG51000_LDO7_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO7_EVENT, SLG51000_LDO7_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_EVENT, SLG51000_OTP_EVENT),
	regmap_reg_range(SLG51000_OTP_IRQ_MASK, SLG51000_OTP_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_LOCK_OTP_PROG, SLG51000_OTP_LOCK_CTRL),
	regmap_reg_range(SLG51000_LOCK_GLOBAL_LOCK_CTRL1,
			 SLG51000_LOCK_GLOBAL_LOCK_CTRL1),
};

static const struct regmap_range slg51000_volatile_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_TEST_EN, SLG51000_SW_TEST_MODE_4),
	regmap_reg_range(SLG51000_IO_GPIO1_CONF, SLG51000_IO_GPIO4_CONF),
	regmap_reg_range(SLG51000_GPIO_CTRL(0), SLG51000_GPIO_CTRL(3)),
	regmap_reg_range(SLG51000_SYSCTL_FAULT_LOG1, SLG51000_SYSCTL_STATUS),
	regmap_reg_range(SLG51000_IO_GPIO_STATUS, SLG51000_IO_GPIO_STATUS),
	regmap_reg_range(SLG51000_LDO1_EVENT, SLG51000_LDO1_STATUS),
	regmap_reg_range(SLG51000_LDO2_EVENT, SLG51000_LDO2_STATUS),
	regmap_reg_range(SLG51000_LDO3_EVENT, SLG51000_LDO3_STATUS),
	regmap_reg_range(SLG51000_LDO4_EVENT, SLG51000_LDO4_STATUS),
	regmap_reg_range(SLG51000_LDO5_EVENT, SLG51000_LDO5_STATUS),
	regmap_reg_range(SLG51000_LDO6_EVENT, SLG51000_LDO6_STATUS),
	regmap_reg_range(SLG51000_LDO7_EVENT, SLG51000_LDO7_STATUS),
	regmap_reg_range(SLG51000_OTP_EVENT, SLG51000_OTP_EVENT),
};

static const struct regmap_access_table slg51000_writeable_table = {
	.yes_ranges	= slg51000_writeable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_writeable_ranges),
};

static const struct regmap_access_table slg51000_readable_table = {
	.yes_ranges	= slg51000_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_readable_ranges),
};

static const struct regmap_access_table slg51000_volatile_table = {
	.yes_ranges	= slg51000_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_volatile_ranges),
};

static const struct regmap_config slg51000_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x8000,
	.wr_table = &slg51000_writeable_table,
	.rd_table = &slg51000_readable_table,
	.volatile_table = &slg51000_volatile_table,
};

static const struct regulator_ops slg51000_regl_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

static const struct regulator_ops slg51000_switch_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static int slg51000_of_parse_cb(struct device_node *np,
				const struct regulator_desc *desc,
				struct regulator_config *config)
{
	struct gpio_desc *ena_gpiod;

	ena_gpiod = fwnode_gpiod_get_index(of_fwnode_handle(np), "enable", 0,
					   GPIOD_OUT_LOW |
						GPIOD_FLAGS_BIT_NONEXCLUSIVE,
					   "gpio-en-ldo");
	if (!IS_ERR(ena_gpiod))
		config->ena_gpiod = ena_gpiod;

	return 0;
}

#define SLG51000_REGL_DESC(_id, _name, _s_name, _min, _step) \
	[SLG51000_REGULATOR_##_id] = {                             \
		.name = #_name,                                    \
		.supply_name = _s_name,				   \
		.id = SLG51000_REGULATOR_##_id,                    \
		.of_match = of_match_ptr(#_name),                  \
		.of_parse_cb = slg51000_of_parse_cb,               \
		.ops = &slg51000_regl_ops,                         \
		.regulators_node = of_match_ptr("regulators"),     \
		.n_voltages = 256,                                 \
		.min_uV = _min,                                    \
		.uV_step = _step,                                  \
		.linear_min_sel = 0,                               \
		.vsel_mask = SLG51000_VSEL_MASK,                   \
		.vsel_reg = SLG51000_##_id##_VSEL,                 \
		.enable_reg = SLG51000_SYSCTL_MATRIX_CONF_A,       \
		.enable_mask = BIT(SLG51000_REGULATOR_##_id),      \
		.type = REGULATOR_VOLTAGE,                         \
		.owner = THIS_MODULE,                              \
	}

static struct regulator_desc regls_desc[SLG51000_MAX_REGULATORS] = {
	SLG51000_REGL_DESC(LDO1, ldo1, NULL,   2400000,  5000),
	SLG51000_REGL_DESC(LDO2, ldo2, NULL,   2400000,  5000),
	SLG51000_REGL_DESC(LDO3, ldo3, "vin3", 1200000, 10000),
	SLG51000_REGL_DESC(LDO4, ldo4, "vin4", 1200000, 10000),
	SLG51000_REGL_DESC(LDO5, ldo5, "vin5",  400000,  5000),
	SLG51000_REGL_DESC(LDO6, ldo6, "vin6",  400000,  5000),
	SLG51000_REGL_DESC(LDO7, ldo7, "vin7", 1200000, 10000),
};

static int slg51000_regulator_init(struct slg51000 *chip)
{
	struct regulator_config config = { };
	struct regulator_desc *rdesc;
	unsigned int reg, val;
	u8 vsel_range[2];
	int id, ret = 0;
	const unsigned int min_regs[SLG51000_MAX_REGULATORS] = {
		SLG51000_LDO1_MINV, SLG51000_LDO2_MINV, SLG51000_LDO3_MINV,
		SLG51000_LDO4_MINV, SLG51000_LDO5_MINV, SLG51000_LDO6_MINV,
		SLG51000_LDO7_MINV,
	};

	for (id = 0; id < SLG51000_MAX_REGULATORS; id++) {
		chip->rdesc[id] = &regls_desc[id];
		rdesc = chip->rdesc[id];
		config.regmap = chip->regmap;
		config.dev = chip->dev;
		config.driver_data = chip;

		ret = regmap_bulk_read(chip->regmap, min_regs[id],
				       vsel_range, 2);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to read the MIN register\n");
			return ret;
		}

		switch (id) {
		case SLG51000_REGULATOR_LDO1:
		case SLG51000_REGULATOR_LDO2:
			if (id == SLG51000_REGULATOR_LDO1)
				reg = SLG51000_LDO1_MISC1;
			else
				reg = SLG51000_LDO2_MISC1;

			ret = regmap_read(chip->regmap, reg, &val);
			if (ret < 0) {
				dev_err(chip->dev,
					"Failed to read voltage range of ldo%d\n",
					id + 1);
				return ret;
			}

			rdesc->linear_min_sel = vsel_range[0];
			rdesc->n_voltages = vsel_range[1] + 1;
			if (val & SLG51000_SEL_VRANGE_MASK)
				rdesc->min_uV = SLG51000_LDOHP_HV_MIN
						+ (vsel_range[0]
						   * rdesc->uV_step);
			else
				rdesc->min_uV = SLG51000_LDOHP_LV_MIN
						+ (vsel_range[0]
						   * rdesc->uV_step);
			break;

		case SLG51000_REGULATOR_LDO5:
		case SLG51000_REGULATOR_LDO6:
			if (id == SLG51000_REGULATOR_LDO5)
				reg = SLG51000_LDO5_TRIM2;
			else
				reg = SLG51000_LDO6_TRIM2;

			ret = regmap_read(chip->regmap, reg, &val);
			if (ret < 0) {
				dev_err(chip->dev,
					"Failed to read LDO mode register\n");
				return ret;
			}

			if (val & SLG51000_SEL_BYP_MODE_MASK) {
				rdesc->ops = &slg51000_switch_ops;
				rdesc->n_voltages = 0;
				rdesc->min_uV = 0;
				rdesc->uV_step = 0;
				rdesc->linear_min_sel = 0;
				break;
			}
			fallthrough;	/* to the check below */

		default:
			rdesc->linear_min_sel = vsel_range[0];
			rdesc->n_voltages = vsel_range[1] + 1;
			rdesc->min_uV = rdesc->min_uV
					+ (vsel_range[0] * rdesc->uV_step);
			break;
		}

		chip->rdev[id] = devm_regulator_register(chip->dev, rdesc,
							 &config);
		if (IS_ERR(chip->rdev[id])) {
			ret = PTR_ERR(chip->rdev[id]);
			dev_err(chip->dev,
				"Failed to register regulator(%s):%d\n",
				chip->rdesc[id]->name, ret);
			return ret;
		}
	}

	return 0;
}

static irqreturn_t slg51000_irq_handler(int irq, void *data)
{
	struct slg51000 *chip = data;
	struct regmap *regmap = chip->regmap;
	enum { R0 = 0, R1, R2, REG_MAX };
	u8 evt[SLG51000_MAX_EVT_REGISTER][REG_MAX];
	int ret, i, handled = IRQ_NONE;
	unsigned int evt_otp, mask_otp;

	/* Read event[R0], status[R1] and mask[R2] register */
	for (i = 0; i < SLG51000_MAX_EVT_REGISTER; i++) {
		ret = regmap_bulk_read(regmap, es_reg[i].ereg, evt[i], REG_MAX);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to read event registers(%d)\n", ret);
			return IRQ_NONE;
		}
	}

	ret = regmap_read(regmap, SLG51000_OTP_EVENT, &evt_otp);
	if (ret < 0) {
		dev_err(chip->dev,
			"Failed to read otp event registers(%d)\n", ret);
		return IRQ_NONE;
	}

	ret = regmap_read(regmap, SLG51000_OTP_IRQ_MASK, &mask_otp);
	if (ret < 0) {
		dev_err(chip->dev,
			"Failed to read otp mask register(%d)\n", ret);
		return IRQ_NONE;
	}

	if ((evt_otp & SLG51000_EVT_CRC_MASK) &&
	    !(mask_otp & SLG51000_IRQ_CRC_MASK)) {
		dev_info(chip->dev,
			 "OTP has been read or OTP crc is not zero\n");
		handled = IRQ_HANDLED;
	}

	for (i = 0; i < SLG51000_MAX_REGULATORS; i++) {
		if (!(evt[i][R2] & SLG51000_IRQ_ILIM_FLAG_MASK) &&
		    (evt[i][R0] & SLG51000_EVT_ILIM_FLAG_MASK)) {
			regulator_notifier_call_chain(chip->rdev[i],
					    REGULATOR_EVENT_OVER_CURRENT, NULL);

			if (evt[i][R1] & SLG51000_STA_ILIM_FLAG_MASK)
				dev_warn(chip->dev,
					 "Over-current limit(ldo%d)\n", i + 1);
			handled = IRQ_HANDLED;
		}
	}

	if (!(evt[SLG51000_SCTL_EVT][R2] & SLG51000_IRQ_HIGH_TEMP_WARN_MASK) &&
	    (evt[SLG51000_SCTL_EVT][R0] & SLG51000_EVT_HIGH_TEMP_WARN_MASK)) {
		for (i = 0; i < SLG51000_MAX_REGULATORS; i++) {
			if (!(evt[i][R1] & SLG51000_STA_ILIM_FLAG_MASK) &&
			    (evt[i][R1] & SLG51000_STA_VOUT_OK_FLAG_MASK)) {
				regulator_notifier_call_chain(chip->rdev[i],
					       REGULATOR_EVENT_OVER_TEMP, NULL);
			}
		}
		handled = IRQ_HANDLED;
		if (evt[SLG51000_SCTL_EVT][R1] &
		    SLG51000_STA_HIGH_TEMP_WARN_MASK)
			dev_warn(chip->dev, "High temperature warning!\n");
	}

	return handled;
}

static void slg51000_clear_fault_log(struct slg51000 *chip)
{
	unsigned int val = 0;
	int ret = 0;

	ret = regmap_read(chip->regmap, SLG51000_SYSCTL_FAULT_LOG1, &val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read Fault log register\n");
		return;
	}

	if (val & SLG51000_FLT_OVER_TEMP_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_OVER_TEMP\n");
	if (val & SLG51000_FLT_POWER_SEQ_CRASH_REQ_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_POWER_SEQ_CRASH_REQ\n");
	if (val & SLG51000_FLT_RST_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_RST\n");
	if (val & SLG51000_FLT_POR_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_POR\n");
}

static int slg51000_sw_test_mode(struct regmap *map, bool on)
{
	static const u8 on_vals[] = { 0x45, 0x53, 0x54, 0x4d };

	if (!on)
		return regmap_write(map, SLG51000_SYSCTL_TEST_EN,
				    SLG51000_TEST_EN_OFF);

	return regmap_bulk_write(map, SLG51000_SW_TEST_MODE_1, on_vals,
				 ARRAY_SIZE(on_vals));
}

static int slg51000_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct slg51000 *chip = gpiochip_get_data(gc);
	unsigned int val;
	int ret;

	ret = regmap_read(chip->regmap, SLG51000_GPIO_CTRL(offset), &val);
	if (ret)
		return ret;

	return val & 1;
}

static int slg51000_gpio_set(struct gpio_chip *gc, unsigned int offset,
			     int value)
{
	struct slg51000 *chip = gpiochip_get_data(gc);
	int ret, err;

	ret = slg51000_sw_test_mode(chip->regmap, true);
	if (ret)
		return ret;

	ret = regmap_write(chip->regmap, SLG51000_GPIO_CTRL(offset), !!value);

	err = slg51000_sw_test_mode(chip->regmap, false);
	if (ret || err)
		dev_err(chip->dev, "gpio%u set %d failed (%d/%d)\n", offset + 1,
			!!value, ret, err);

	return ret ? ret : err;
}

static int slg51000_gpio_direction_output(struct gpio_chip *gc,
					  unsigned int offset, int value)
{
	struct slg51000 *chip = gpiochip_get_data(gc);
	int ret, err;

	ret = slg51000_sw_test_mode(chip->regmap, true);
	if (ret)
		return ret;

	ret = regmap_update_bits(chip->regmap, SLG51000_IO_GPIO1_CONF + offset,
				 SLG51000_GPIO_DIR_OUT, SLG51000_GPIO_DIR_OUT);
	if (!ret)
		ret = regmap_write(chip->regmap, SLG51000_GPIO_CTRL(offset),
				   !!value);

	err = slg51000_sw_test_mode(chip->regmap, false);
	if (ret || err)
		dev_err(chip->dev, "gpio%u direction out failed (%d/%d)\n",
			offset + 1, ret, err);

	return ret ? ret : err;
}

static int slg51000_gpio_get_direction(struct gpio_chip *gc,
				       unsigned int offset)
{
	struct slg51000 *chip = gpiochip_get_data(gc);
	unsigned int val;
	int ret;

	ret = regmap_read(chip->regmap, SLG51000_IO_GPIO1_CONF + offset, &val);
	if (ret)
		return ret;

	return (val & SLG51000_GPIO_DIR_OUT) ? GPIO_LINE_DIRECTION_OUT :
					       GPIO_LINE_DIRECTION_IN;
}

/* GPIO1..4 as outputs (camera module enables); only with "gpio-controller". */
static int slg51000_gpio_init(struct slg51000 *chip)
{
	if (!device_property_present(chip->dev, "gpio-controller"))
		return 0;

	chip->gc.label = "slg51000";
	chip->gc.parent = chip->dev;
	chip->gc.owner = THIS_MODULE;
	chip->gc.base = -1;
	chip->gc.ngpio = SLG51000_NUM_GPIOS;
	chip->gc.can_sleep = true;
	chip->gc.get = slg51000_gpio_get;
	chip->gc.set = slg51000_gpio_set;
	chip->gc.direction_output = slg51000_gpio_direction_output;
	chip->gc.get_direction = slg51000_gpio_get_direction;

	return devm_gpiochip_add_data(chip->dev, &chip->gc, chip);
}

static int slg51000_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct slg51000 *chip;
	struct gpio_desc *cs_gpiod, *bb_gpiod, *buck_gpiod, *pu_gpiod;
	int error, ret;

	chip = devm_kzalloc(dev, sizeof(struct slg51000), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	/*
	 * SLG51000 camera-PMIC power-up sequence, matching the vendor
	 * slg51000_power_on(): bb -> 2ms -> buck -> 2ms -> cs -> 10ms
	 * (datasheet CS-HIGH-to-Ready) -> pu -> 1ms. The internal boost
	 * (bb) and buck must be up before cs, and pu (which triggers the
	 * power-up sequencer) must come LAST. Asserting these lines all at
	 * once via DT gpio-hogs (before cs, at DT-parse) leaves the buck/
	 * buck-boost-fed LDOs (ldo5/6/7 = camera dvdd/dovdd) un-armed, so
	 * they never reach VOUT_OK and the sensors NACK on I2C (-EIO).
	 * All lines are requested LOW and driven high here in order.
	 */
	bb_gpiod = devm_gpiod_get_optional(dev, "dlg,bb",
					   GPIOD_OUT_LOW |
						GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(bb_gpiod))
		return PTR_ERR(bb_gpiod);

	buck_gpiod = devm_gpiod_get_optional(dev, "dlg,buck",
					     GPIOD_OUT_LOW |
						GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(buck_gpiod))
		return PTR_ERR(buck_gpiod);

	cs_gpiod = devm_gpiod_get_optional(dev, "dlg,cs",
					   GPIOD_OUT_LOW |
						GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(cs_gpiod))
		return PTR_ERR(cs_gpiod);

	pu_gpiod = devm_gpiod_get_optional(dev, "dlg,pu",
					   GPIOD_OUT_LOW |
						GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(pu_gpiod))
		return PTR_ERR(pu_gpiod);

	if (bb_gpiod) {
		chip->bb_gpiod = bb_gpiod;
		gpiod_set_value_cansleep(bb_gpiod, 1);
		usleep_range(2000, 2020);
	}

	if (buck_gpiod) {
		chip->buck_gpiod = buck_gpiod;
		gpiod_set_value_cansleep(buck_gpiod, 1);
		usleep_range(2000, 2020);
	}

	if (cs_gpiod) {
		dev_info(dev, "Found chip selector property\n");
		chip->cs_gpiod = cs_gpiod;
		gpiod_set_value_cansleep(cs_gpiod, 1);
		/* CS HIGH -> Ready is ~10ms per datasheet. */
		usleep_range(10000, 11000);
	} else {
		usleep_range(10000, 11000);
	}

	if (pu_gpiod) {
		chip->pu_gpiod = pu_gpiod;
		gpiod_set_value_cansleep(pu_gpiod, 1);
		usleep_range(1000, 1020);
	}

	i2c_set_clientdata(client, chip);
	chip->chip_irq = client->irq;
	chip->dev = dev;
	chip->regmap = devm_regmap_init_i2c(client, &slg51000_regmap_config);
	if (IS_ERR(chip->regmap)) {
		error = PTR_ERR(chip->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n",
			error);
		return error;
	}

	ret = slg51000_regulator_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to init regulator(%d)\n", ret);
		return ret;
	}

	ret = slg51000_gpio_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to init gpio(%d)\n", ret);
		return ret;
	}

	slg51000_clear_fault_log(chip);

	if (chip->chip_irq) {
		ret = devm_request_threaded_irq(dev, chip->chip_irq, NULL,
						slg51000_irq_handler,
						(IRQF_TRIGGER_HIGH |
						IRQF_ONESHOT),
						"slg51000-irq", chip);
		if (ret != 0) {
			dev_err(dev, "Failed to request IRQ: %d\n",
				chip->chip_irq);
			return ret;
		}
	} else {
		dev_info(dev, "No IRQ configured\n");
	}

	return ret;
}

static const struct i2c_device_id slg51000_i2c_id[] = {
	{ "slg51000" },
	{}
};
MODULE_DEVICE_TABLE(i2c, slg51000_i2c_id);

/*
 * DT-instantiated i2c devices are auto-loaded via their OF modalias
 * (of:...C<compatible>), so an of_match_table is required for the module
 * to load automatically. Without it the driver only matches the i2c
 * id_table and has to be modprobed by hand.
 */
static const struct of_device_id slg51000_dt_match[] = {
	{ .compatible = "dlg,slg51000" },
	{}
};
MODULE_DEVICE_TABLE(of, slg51000_dt_match);

static struct i2c_driver slg51000_regulator_driver = {
	.driver = {
		.name = "slg51000-regulator",
		.of_match_table = slg51000_dt_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = slg51000_i2c_probe,
	.id_table = slg51000_i2c_id,
};

module_i2c_driver(slg51000_regulator_driver);

MODULE_AUTHOR("Eric Jeong <eric.jeong.opensource@diasemi.com>");
MODULE_DESCRIPTION("SLG51000 regulator driver");
MODULE_LICENSE("GPL");

