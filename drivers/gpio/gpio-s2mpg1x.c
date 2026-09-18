// SPDX-License-Identifier: GPL-2.0
/*
 * GPIO driver for the Samsung S2MPG10 / S2MPG11 PMICs on Google Tensor (gs101).
 *
 * The two PMICs each expose six GPIOs behind a block of seven control
 * registers (input / output / output-enable / pull-down / pull-up /
 * drive-strength / remote), one bit per line. The parent sec-pmic MFD owns the
 * I2C regmap; this driver is a thin gpio-regmap child bound to the
 * "samsung,s2mpgNN-gpio" cell that MFD_CELL_OF() registers.
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/gpio/regmap.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg11.h>

#define S2MPG1X_GPIO_NGPIO	6

struct s2mpg1x_gpio_variant {
	unsigned int ctrl_base;		/* CTRL1: input data */
};

/* CTRL1 input, CTRL2 output, CTRL3 output-enable -- contiguous. */
static const struct s2mpg1x_gpio_variant s2mpg10_variant = {
	.ctrl_base = S2MPG10_PMIC_GPIO_CTRL1,
};

static const struct s2mpg1x_gpio_variant s2mpg11_variant = {
	.ctrl_base = S2MPG11_PMIC_GPIO_CTRL1,
};

static int s2mpg1x_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct s2mpg1x_gpio_variant *variant;
	struct sec_pmic_dev *sec_pmic;
	struct gpio_regmap_config config = {};

	variant = device_get_match_data(dev);
	if (!variant)
		return -EINVAL;

	sec_pmic = dev_get_drvdata(dev->parent);
	if (!sec_pmic || !sec_pmic->regmap_pmic)
		return -EPROBE_DEFER;

	config.parent = dev;
	config.regmap = sec_pmic->regmap_pmic;
	config.fwnode = dev_fwnode(dev);
	config.ngpio = S2MPG1X_GPIO_NGPIO;
	config.ngpio_per_reg = S2MPG1X_GPIO_NGPIO;
	config.reg_dat_base = variant->ctrl_base;		/* CTRL1 */
	config.reg_set_base = variant->ctrl_base + 1;		/* CTRL2 */
	config.reg_dir_out_base = variant->ctrl_base + 2;	/* CTRL3 */

	return PTR_ERR_OR_ZERO(devm_gpio_regmap_register(dev, &config));
}

static const struct of_device_id s2mpg1x_gpio_of_match[] = {
	{ .compatible = "samsung,s2mpg10-gpio", .data = &s2mpg10_variant },
	{ .compatible = "samsung,s2mpg11-gpio", .data = &s2mpg11_variant },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpg1x_gpio_of_match);

static struct platform_driver s2mpg1x_gpio_driver = {
	.driver = {
		.name = "s2mpg1x-gpio",
		.of_match_table = s2mpg1x_gpio_of_match,
	},
	.probe = s2mpg1x_gpio_probe,
};
module_platform_driver(s2mpg1x_gpio_driver);

MODULE_DESCRIPTION("Samsung S2MPG10/S2MPG11 PMIC GPIO driver");
MODULE_LICENSE("GPL");
