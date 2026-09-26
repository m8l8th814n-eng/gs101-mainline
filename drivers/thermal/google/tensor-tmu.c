// SPDX-License-Identifier: GPL-2.0
/* Google Tensor (gs101/gs201/zuma) TMU thermal sensor driver. */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>

struct tensor_tmu_soc {
	unsigned int triminfo0;
	unsigned int current_temp1_0;
	unsigned int temp_mask;
	unsigned int nr_sensors;
};

static const struct tensor_tmu_soc gs101_soc = {
	.triminfo0		= 0x0010,
	.current_temp1_0	= 0x0084,
	.temp_mask		= 0x1ff,
	.nr_sensors		= 16,
};

#define TMU_TRIMINFO(soc, p)		((p) * 0x4 + (soc)->triminfo0)
#define TMU_CURRENT_TEMP(soc, p)	(((p) / 2) * 0x4 + (soc)->current_temp1_0)

#define TMU_CAL_T1	25
#define TMU_CAL_T2	85

struct tensor_tmu {
	struct device			*dev;
	void __iomem			*base;
	const struct tensor_tmu_soc	*soc;
	u16				cal25[16];
	u16				cal85[16];
};

struct tensor_tmu_sensor {
	struct tensor_tmu	*tmu;
	int			id;
};

static int tensor_code_to_temp(struct tensor_tmu *tmu, int p, u16 code)
{
	int c25 = tmu->cal25[p];
	int c85 = tmu->cal85[p];

	if (c85 <= c25)
		return -EINVAL;

	return ((int)code - c25) * (TMU_CAL_T2 - TMU_CAL_T1) / (c85 - c25)
	       + TMU_CAL_T1;
}

static int tensor_tmu_get_temp(struct thermal_zone_device *tz, int *temp)
{
	const struct tensor_tmu_sensor *s = thermal_zone_device_priv(tz);
	struct tensor_tmu *tmu = s->tmu;
	int p = s->id;
	u32 word;
	u16 code;
	int t;

	word = readl(tmu->base + TMU_CURRENT_TEMP(tmu->soc, p));
	code = (p & 1) ? (word >> 16) : (word & 0xffff);
	code &= tmu->soc->temp_mask;

	t = tensor_code_to_temp(tmu, p, code);
	if (t < 0)
		return t;

	*temp = t * 1000;
	return 0;
}

static const struct thermal_zone_device_ops tensor_tmu_ops = {
	.get_temp = tensor_tmu_get_temp,
};

static void tensor_tmu_read_cal(struct tensor_tmu *tmu, int p)
{
	u32 trim = readl(tmu->base + TMU_TRIMINFO(tmu->soc, p));

	tmu->cal25[p] = trim & tmu->soc->temp_mask;
	tmu->cal85[p] = (trim >> 9) & tmu->soc->temp_mask;
}

static int tensor_tmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tensor_tmu *tmu;
	int i, registered = 0;

	tmu = devm_kzalloc(dev, sizeof(*tmu), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;

	tmu->dev = dev;
	tmu->soc = of_device_get_match_data(dev);
	if (!tmu->soc)
		return -EINVAL;

	tmu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tmu->base))
		return PTR_ERR(tmu->base);

	platform_set_drvdata(pdev, tmu);

	for (i = 0; i < tmu->soc->nr_sensors; i++) {
		struct tensor_tmu_sensor *s;
		struct thermal_zone_device *tz;

		s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
		if (!s)
			return -ENOMEM;
		s->tmu = tmu;
		s->id = i;

		tensor_tmu_read_cal(tmu, i);

		tz = devm_thermal_of_zone_register(dev, i, s, &tensor_tmu_ops);
		if (IS_ERR(tz)) {
			if (PTR_ERR(tz) == -ENODEV)
				continue;
			return dev_err_probe(dev, PTR_ERR(tz),
					     "failed to register sensor %d\n", i);
		}
		registered++;
	}

	if (!registered)
		return dev_err_probe(dev, -ENODEV,
				     "no thermal zones referenced this TMU\n");

	dev_info(dev, "Tensor TMU: %d sensor(s) registered\n", registered);
	return 0;
}

static const struct of_device_id tensor_tmu_of_match[] = {
	{ .compatible = "google,gs101-tmu", .data = &gs101_soc },
	{ }
};
MODULE_DEVICE_TABLE(of, tensor_tmu_of_match);

static struct platform_driver tensor_tmu_driver = {
	.driver = {
		.name = "tensor-tmu",
		.of_match_table = tensor_tmu_of_match,
	},
	.probe = tensor_tmu_probe,
};
module_platform_driver(tensor_tmu_driver);

MODULE_DESCRIPTION("Google Tensor TMU thermal sensor driver");
MODULE_LICENSE("GPL");
