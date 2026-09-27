// SPDX-License-Identifier: GPL-2.0
/* Minimal BCM4389 BT power/wake control (replaces the hogged BT_REG_ON). */

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

struct bt_regon {
	struct gpio_desc *reg_on;	/* BT_REG_ON  (shutdown) */
	struct gpio_desc *dev_wake;	/* BT device-wakeup      */
	struct gpio_desc *host_wake;	/* BT host-wakeup (input) */
};

static ssize_t reg_on_show(struct device *dev, struct device_attribute *a, char *buf)
{
	struct bt_regon *bt = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", gpiod_get_value_cansleep(bt->reg_on));
}

static ssize_t reg_on_store(struct device *dev, struct device_attribute *a,
			    const char *buf, size_t len)
{
	struct bt_regon *bt = dev_get_drvdata(dev);
	bool on;

	if (kstrtobool(buf, &on))
		return -EINVAL;
	gpiod_set_value_cansleep(bt->reg_on, on);
	return len;
}
static DEVICE_ATTR_RW(reg_on);

static ssize_t dev_wake_show(struct device *dev, struct device_attribute *a, char *buf)
{
	struct bt_regon *bt = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", gpiod_get_value_cansleep(bt->dev_wake));
}

static ssize_t dev_wake_store(struct device *dev, struct device_attribute *a,
			      const char *buf, size_t len)
{
	struct bt_regon *bt = dev_get_drvdata(dev);
	bool on;

	if (kstrtobool(buf, &on))
		return -EINVAL;
	gpiod_set_value_cansleep(bt->dev_wake, on);
	return len;
}
static DEVICE_ATTR_RW(dev_wake);

static ssize_t host_wake_show(struct device *dev, struct device_attribute *a, char *buf)
{
	struct bt_regon *bt = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", gpiod_get_value_cansleep(bt->host_wake));
}
static DEVICE_ATTR_RO(host_wake);

static struct attribute *bt_regon_attrs[] = {
	&dev_attr_reg_on.attr,
	&dev_attr_dev_wake.attr,
	&dev_attr_host_wake.attr,
	NULL,
};
ATTRIBUTE_GROUPS(bt_regon);

static int bt_regon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bt_regon *bt;

	bt = devm_kzalloc(dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;

	/* start powered off so userspace can do a clean power-cycle */
	bt->reg_on = devm_gpiod_get(dev, "shutdown", GPIOD_OUT_LOW);
	if (IS_ERR(bt->reg_on))
		return dev_err_probe(dev, PTR_ERR(bt->reg_on), "shutdown gpio\n");

	bt->dev_wake = devm_gpiod_get_optional(dev, "device-wakeup", GPIOD_OUT_LOW);
	if (IS_ERR(bt->dev_wake))
		return dev_err_probe(dev, PTR_ERR(bt->dev_wake), "device-wakeup gpio\n");

	bt->host_wake = devm_gpiod_get_optional(dev, "host-wakeup", GPIOD_IN);
	if (IS_ERR(bt->host_wake))
		return dev_err_probe(dev, PTR_ERR(bt->host_wake), "host-wakeup gpio\n");

	dev_set_drvdata(dev, bt);
	dev_info(dev, "bt-regon ready (reg_on off; control via sysfs)\n");
	return 0;
}

static const struct of_device_id bt_regon_of_match[] = {
	{ .compatible = "goog,nitrous" },
	{ }
};
MODULE_DEVICE_TABLE(of, bt_regon_of_match);

static struct platform_driver bt_regon_driver = {
	.driver = {
		.name = "bt-regon",
		.of_match_table = bt_regon_of_match,
		.dev_groups = bt_regon_groups,
	},
	.probe = bt_regon_probe,
};
module_platform_driver(bt_regon_driver);

MODULE_DESCRIPTION("Minimal BCM4389 BT_REG_ON / wake control");
MODULE_LICENSE("GPL");
