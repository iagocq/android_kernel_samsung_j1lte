/*
 * Marvell 88PM886 ONKEY driver
 *
 * Copyright (C) 2014 Marvell International Ltd.
 * Yi Zhang <yizhang@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/mfd/88pm886.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of.h>
#ifdef CONFIG_MACH_PXA_SAMSUNG
#include <linux/sec-common.h>
#endif

//#define PM886_ONKEY_STS1					(0x1 << 0)

#define PM886_GPIO0_HW_RST1_N				(0x6 << 1)
#define PM886_GPIO0_HW_RST2_N				(0x7 << 1)

#define PM886_GPIO1_HW_RST1_N				(0x6 << 5)
#define PM886_GPIO1_HW_RST2_N				(0x7 << 5)

#define PM886_GPIO2_HW_RST1					(0x6 << 1)
#define PM886_GPIO2_HW_RST2					(0x7 << 1)

#define PM886_HWRST_DB_MSK					(0x1 << 7)
#define PM886_HWRST_DB_2S					(0x0 << 7)
#define PM886_HWRST_DB_7S					(0x1 << 7)

#define PM886_LONKEY_PRESS_TIME_MASK		(0xf0)

#ifdef CONFIG_88PM886_SMART_RESET
#define PM886_LONG_KEY_DELAY				(16)	/* 1 .. 16 seconds */
#define PM886_LONKEY_PRESS_TIME             ((PM886_LONG_KEY_DELAY-1) << 4)
#define PM886_LONKEY_RESTOUTN_PULSE_MASK    (0x3)

#define PM886_LONG_ONKEY_EN1				(1 << 0)
#define PM886_LONG_ONKEY_EN1_DIS			(0 << 0)
#define PM886_LONG_ONKEY_EN2				(1 << 1)
#define PM886_LONG_ONKEY_EN2_DIS			(0 << 1)

static struct workqueue_struct *longkey_tmr_workqueue;
#define CHECK_LONGKEY_TIMER 12

struct pm886_onkey_info {
	struct input_dev *idev;
	struct pm886_chip *pm886;
	struct regmap *map;
	int irq;
	int gpio_number;
	struct work_struct tmr_work;
	struct timer_list longkey_timeout_tmr;
	struct timer_list *p_longkey_timeout_tmr;
	spinlock_t lock;
#ifdef CONFIG_MACH_PXA_SAMSUNG
	struct device *sec_power_key;
	bool state;
#endif
};
#else
#define PM886_LONG_KEY_DELAY				(8)	/* 1 .. 16 seconds */
#define PM886_LONKEY_PRESS_TIME             ((PM886_LONG_KEY_DELAY-1) << 4)
#define PM886_LONKEY_RESTOUTN_PULSE_MASK    (0x3)

#define PM886_LONG_ONKEY_EN1				(0 << 1)
#define PM886_LONG_ONKEY_EN2				(1 << 1)

struct pm886_onkey_info {
	struct input_dev *idev;
	struct pm886_chip *pm886;
	struct regmap *map;
	int irq;
	int gpio_number;
#ifdef CONFIG_MACH_PXA_SAMSUNG
	struct device *sec_power_key;
	bool state;
#endif
};
#endif

static int pm886_config_gpio(struct pm886_onkey_info *info)
{
	if (!info || !info->map) {
		pr_err("%s: No chip information!\n", __func__);
		return -ENODEV;
	}

	/* choose HW_RST1_N for GPIO, only toggle RESETOUTN */
	switch (info->gpio_number) {
	case 0:
		regmap_update_bits(info->map, PM886_GPIO_CTRL1,
				   PM886_GPIO0_MODE_MSK, PM886_GPIO0_HW_RST1_N);
		break;
	case 1:
		regmap_update_bits(info->map, PM886_GPIO_CTRL1,
				   PM886_GPIO1_MODE_MSK, PM886_GPIO1_HW_RST1_N);
		break;
	case 2:
		regmap_update_bits(info->map, PM886_GPIO_CTRL2,
				   PM886_GPIO2_MODE_MSK, PM886_GPIO2_HW_RST1);
		break;
	default:
		dev_err(info->idev->dev.parent, "use the wrong GPIO, exit 0\n");
		return 0;
	}
	/* 0xe2: set debounce period of ONKEY as 7s, when used with GPIO */
	regmap_update_bits(info->map, PM886_AON_CTRL2,
			   PM886_HWRST_DB_MSK , PM886_HWRST_DB_7S);

	/* AON CONTROL : 0xE3 */
	/* set duration of RESETOUTN pulse as 1s */
	regmap_update_bits(info->map, PM886_AON_CTRL3,
		PM886_LONKEY_RESTOUTN_PULSE_MASK, 1);
	
	/* set debounce period of ONKEY as 8s */
	regmap_update_bits(info->map, PM886_AON_CTRL3,
		PM886_LONKEY_PRESS_TIME_MASK,
		PM886_LONKEY_PRESS_TIME);

#ifdef CONFIG_88PM886_SMART_RESET
	/* 0xe4: disable LONG_ONKEY_DETECT1, onkey reset system */
	regmap_update_bits(info->map, PM886_AON_CTRL4,
		PM886_LONG_ONKEY_EN1, PM886_LONG_ONKEY_EN1_DIS);

	/* 0xe4: enable LONG_ONKEY_DETECT2, onkey reset system */
	regmap_update_bits(info->map, PM886_AON_CTRL4,
		PM886_LONG_ONKEY_EN2, PM886_LONG_ONKEY_EN2_DIS);
#else
	/* 0xe4: enable LONG_ONKEY_DETECT2, onkey reset system */
	regmap_update_bits(info->map, PM886_AON_CTRL4,
		PM886_LONG_ONKEY_EN2, PM886_LONG_ONKEY_EN2);
#endif
	return 0;
}

#ifdef CONFIG_88PM886_SMART_RESET
static void longkey_timeout_handler(unsigned long data)
{
	struct pm886_onkey_info *info = (struct pm886_onkey_info *)data;

	info->p_longkey_timeout_tmr = NULL;
	queue_work(longkey_tmr_workqueue, &info->tmr_work);
}

static void longkey_timer_start(u16 sec, struct pm886_onkey_info *info)
{
	unsigned long flags;

	spin_lock_irqsave(&info->lock,flags);
	if (info->p_longkey_timeout_tmr != NULL)
		del_timer_sync(info->p_longkey_timeout_tmr);

	info->p_longkey_timeout_tmr = NULL;
	init_timer(&(info->longkey_timeout_tmr));
	info->longkey_timeout_tmr.data = (unsigned long)(info);
	info->longkey_timeout_tmr.function = longkey_timeout_handler;
	info->longkey_timeout_tmr.expires = jiffies + (sec * HZ);
	info->p_longkey_timeout_tmr = &info->longkey_timeout_tmr;
	add_timer(&info->longkey_timeout_tmr);
	spin_unlock_irqrestore(&info->lock, flags);
}

static void longkey_timer_stop(struct pm886_onkey_info *info)
{
	unsigned long flags;

	spin_lock_irqsave(&info->lock,flags);

	if (info->p_longkey_timeout_tmr)
		del_timer_sync(info->p_longkey_timeout_tmr);

	info->p_longkey_timeout_tmr = NULL;
	spin_unlock_irqrestore(&info->lock, flags);
}

static void longkey_tmr_work(struct work_struct *work)
{
	int ret = 0;
	unsigned int val;
	struct pm886_onkey_info *info =
		container_of(work, struct pm886_onkey_info, tmr_work);

	info->p_longkey_timeout_tmr = NULL;

	/* reset the LONGKEY reset time */
	regmap_update_bits(info->map, PM886_MISC_CONFIG1,
					PM886_LONKEY_RST, PM886_LONKEY_RST);

	ret = regmap_read(info->map, PM886_STATUS1, &val);
	if (ret < 0) {
		dev_info(info->idev->dev.parent, "failed to read status: %d\n", ret);
	}
	val &= PM886_ONKEY_STS1;

	val ? longkey_timer_start(CHECK_LONGKEY_TIMER, info) : longkey_timer_stop(info);
}
#endif

static irqreturn_t pm886_onkey_handler(int irq, void *data)
{
	struct pm886_onkey_info *info = data;
	int ret = 0;
	unsigned int val;

	/* reset the LONGKEY reset time */
	regmap_update_bits(info->map, PM886_MISC_CONFIG1,
			   PM886_LONKEY_RST, PM886_LONKEY_RST);

	ret = regmap_read(info->map, PM886_STATUS1, &val);
	if (ret < 0) {
		dev_err(info->idev->dev.parent,
			"failed to read status: %d\n", ret);
		return IRQ_NONE;
	}
	val &= PM886_ONKEY_STS1;

	input_report_key(info->idev, KEY_POWER, val);

#ifdef CONFIG_MACH_PXA_SAMSUNG
	info->state = val;
#endif
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
	pr_info("[KEY] %d key %s\n", KEY_POWER, val ? "Press" : "Release");
#else
	pr_info("[KEY] key %s\n", val ? "Press" : "Release");
#endif

#ifdef CONFIG_88PM886_SMART_RESET
	val ? longkey_timer_start(CHECK_LONGKEY_TIMER, info) : longkey_timer_stop(info);
#endif

	input_sync(info->idev);

	return IRQ_HANDLED;
}

#ifdef CONFIG_MACH_PXA_SAMSUNG
static ssize_t show_key_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pm886_onkey_info *info = dev_get_drvdata(dev);
	bool state = info->state;

	return sprintf(buf, state ? "PRESS" : "RELEASE");
}

static ssize_t store_key(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct pm886_onkey_info *info = dev_get_drvdata(dev);
	unsigned int val;

	if (!strncmp(buf, "1", 1))
		val = 1;
	else if (!strncmp(buf, "0", 1))
		val = 0;
	else
		return count;

	input_report_key(info->idev, KEY_POWER, val);
	info->state = val;

	pr_info("[KEY] %d key %s by SW\n", KEY_POWER, val ? "Press" : "Release");

	input_sync(info->idev);

	return count;
}

static DEVICE_ATTR(sec_power_key_pressed,
	S_IRUGO | S_IWUSR | S_IWGRP, show_key_status, store_key);

static struct attribute *power_key_attributes[] = {
	&dev_attr_sec_power_key_pressed.attr,
	NULL,
};

static struct attribute_group power_key_attr_group = {
	.attrs = power_key_attributes,
};
#endif

static SIMPLE_DEV_PM_OPS(pm886_onkey_pm_ops, NULL, NULL);

static int pm886_onkey_dt_init(struct device_node *np,
			       struct pm886_onkey_info *info)
{
	int ret;
	if (!info) {
		pr_err("%s: No chip information!\n", __func__);
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "marvell,pm886-onkey-gpio-number",
				   &info->gpio_number);
	if (ret < 0) {
		dev_warn(info->idev->dev.parent, "No GPIO for long onkey.\n");
		return 0;
	}

	return 0;
}

static int pm886_onkey_probe(struct platform_device *pdev)
{

	struct pm886_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct device_node *node = pdev->dev.of_node;
	struct pm886_onkey_info *info;
	int err;

	info = devm_kzalloc(&pdev->dev, sizeof(struct pm886_onkey_info),
			    GFP_KERNEL);
	if (!info || !chip)
		return -ENOMEM;
	info->pm886 = chip;

#ifdef CONFIG_88PM886_SMART_RESET
	info->p_longkey_timeout_tmr = NULL;
#endif

	/* give the gpio number as a default value */
	info->gpio_number = -1;
	err = pm886_onkey_dt_init(node, info);
	if (err < 0)
		return -ENODEV;

	info->irq = platform_get_irq(pdev, 0);
	if (info->irq < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		err = -EINVAL;
		goto out;
	}

	info->map = info->pm886->base_regmap;
	if (!info->map) {
		dev_err(&pdev->dev, "No regmap handler!\n");
		err = -EINVAL;
		goto out;
	}

	info->idev = input_allocate_device();
	if (!info->idev) {
		dev_err(&pdev->dev, "Failed to allocate input dev\n");
		err = -ENOMEM;
		goto out;
	}

	info->idev->name = "88pm886_on";
	info->idev->phys = "88pm886_on/input0";
	info->idev->id.bustype = BUS_I2C;
	info->idev->dev.parent = &pdev->dev;
	info->idev->evbit[0] = BIT_MASK(EV_KEY);
	__set_bit(KEY_POWER, info->idev->keybit);

#ifdef CONFIG_88PM886_SMART_RESET
	spin_lock_init(&info->lock);
	INIT_WORK(&info->tmr_work, longkey_tmr_work);
	longkey_tmr_workqueue =
		create_singlethread_workqueue("longkey_tmr_workqueue");
#endif

	err = devm_request_threaded_irq(&pdev->dev, info->irq, pm886_onkey_handler,
					pm886_onkey_handler,
					IRQF_ONESHOT | IRQF_NO_SUSPEND,
					"onkey", info);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq, err);
		goto out_register;
	}

	err = input_register_device(info->idev);
	if (err) {
		dev_err(&pdev->dev, "Can't register input device: %d\n", err);
		goto out_register;
	}

	platform_set_drvdata(pdev, info);

	device_init_wakeup(&pdev->dev, 1);

	err = pm886_config_gpio(info);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't configure gpio: %d\n", err);
		goto out_register;
	}

#ifdef CONFIG_MACH_PXA_SAMSUNG
	info->sec_power_key = device_create(sec_class, NULL,
		MKDEV(0, 0), info, "sec_power_key");

	if (IS_ERR(info->sec_power_key)) {
		err = PTR_ERR(info->sec_power_key);
		dev_err(&pdev->dev,
			"Failed to create device for sec power key(%d)\n", err);
	}

	err = sysfs_create_group(&info->sec_power_key->kobj,
					&power_key_attr_group);
	if (err)
		dev_err(&pdev->dev,
			"Failed to create sysfs for sec power key(%d)\n", err);
#endif

	return 0;

out_register:
	input_free_device(info->idev);
out:
	return err;
}

static int pm886_onkey_remove(struct platform_device *pdev)
{
	struct pm886_onkey_info *info = platform_get_drvdata(pdev);

#ifdef CONFIG_MACH_PXA_SAMSUNG
	sysfs_remove_group(&info->sec_power_key->kobj, &power_key_attr_group);
	device_destroy(sec_class, MKDEV(0, 0));
#endif

	device_init_wakeup(&pdev->dev, 0);
	devm_free_irq(&pdev->dev, info->irq, info);

	input_unregister_device(info->idev);

	devm_kfree(&pdev->dev, info);

	return 0;
}

static const struct of_device_id pm886_onkey_dt_match[] = {
	{ .compatible = "marvell,88pm886-onkey", },
	{ },
};
MODULE_DEVICE_TABLE(of, pm886_onkey_dt_match);

static struct platform_driver pm886_onkey_driver = {
	.driver = {
		.name = "88pm886-onkey",
		.owner = THIS_MODULE,
		.pm = &pm886_onkey_pm_ops,
		.of_match_table = of_match_ptr(pm886_onkey_dt_match),
	},
	.probe = pm886_onkey_probe,
	.remove = pm886_onkey_remove,
};
module_platform_driver(pm886_onkey_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Marvell 88PM886 ONKEY driver");
MODULE_AUTHOR("Yi Zhang <yizhang@marvell.com>");
MODULE_ALIAS("platform:88pm886-onkey");
