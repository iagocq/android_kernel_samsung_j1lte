/*
 * Samsung Mobile SystemS/W R&D 2 Group.
 *
 * drivers/samsung/sec-factoryled.c
 *
 * Drivers for samsung factory LED.
 *
 * Copyright (C) 2014, Samsung Electronics.
 *
 * This program is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_data/gen-panel.h>
#include <linux/extcon/sm5504-muic.h>

static void secfactory_led_handler(struct work_struct *work);
struct delayed_work wq;
int bootcheck;
int flash_en;
extern unsigned int battery_soc;
unsigned long delay;

extern void secfactory_flash_control(int en);

/*sys fs*/
struct class *secfactory_led_class;
EXPORT_SYMBOL(secfactory_led_class);

struct device *secfactory_led;
EXPORT_SYMBOL(secfactory_led);

static ssize_t secfactory_bootcheck_file_write(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size);

static DEVICE_ATTR(secfactory_bootcheck, 0664 , NULL, secfactory_bootcheck_file_write);

static struct attribute *secfactory_led_attributes[] = {
		&dev_attr_secfactory_bootcheck.attr,
		NULL,
};

static struct attribute_group secfactory_led_attr_group = {
		.attrs = secfactory_led_attributes,
};

static int atoi(const char *str)
{
	int result = 0;
	int count = 0;
	if (str == NULL)
		return -EIO;
	while (str[count] != 0	/* NULL */
		&& str[count] >= '0' && str[count] <= '9')	{
		result = result * 10 + str[count] - '0';
		++count;
	}
	return result;
}

static ssize_t secfactory_bootcheck_file_write(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	pr_info("[secfactory_led] booting complete value = %s\n", buf);

	bootcheck = atoi(buf);

	pr_info("[secfactory_led] bootcheck value = %d\n", bootcheck);

	return size;
}

static void secfactory_led_handler(struct work_struct *work)
{
	if(bootcheck){
		if(battery_soc > 21)
			secfactory_flash_control(2);
		else {
			if(flash_en) {
				secfactory_flash_control(0);
				flash_en = 0;
			} 
			else {
				secfactory_flash_control(2);
				flash_en = 1;
			}
			delay = msecs_to_jiffies(800);
			schedule_delayed_work(&wq, delay);
		}
	}
	else {
		if(flash_en) {
			secfactory_flash_control(0);
			flash_en = 0;
		} 
		else {
			secfactory_flash_control(1);
			flash_en = 1;
		}
		if(battery_soc > 21)
			delay = msecs_to_jiffies(300);
		else
			delay = msecs_to_jiffies(800);
		schedule_delayed_work(&wq, delay);
	}
}

static int secfactory_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct class *secfactory_led_class;
	struct device *secfactory_led;

	bootcheck = 0;
	flash_en = 0;
	
	if(battery_soc > 21)
		delay = msecs_to_jiffies(300);
	else
		delay = msecs_to_jiffies(800);

	if ((!lcd_connected())&&(!get_jig_state())) {
		INIT_DELAYED_WORK(&wq, secfactory_led_handler);
		schedule_delayed_work(&wq, delay);
	}

	pr_info("[secfactory_led] %s has been created!!!\n", __func__);

	secfactory_led_class = class_create(THIS_MODULE, "secfactory_led");
	if (IS_ERR(secfactory_led_class)) {
		ret = PTR_ERR(secfactory_led_class);
		pr_err("Failed to create class(secfactory_led_all)");
		goto class_create_fail;
	}

	secfactory_led= device_create(secfactory_led_class,
				NULL, 0, NULL, "secfactory_led_all");
	if (IS_ERR(secfactory_led)) {
		ret = PTR_ERR(secfactory_led);
		pr_err("Failed to create device(secfactory_led_all)");
		goto device_create_fail;
	}

	ret = sysfs_create_group(&secfactory_led->kobj,
			&secfactory_led_attr_group);
	if (ret) {
		pr_err("Failed to create sysfs group");
		goto create_group_fail;
	}

	return 0;

create_group_fail:
	device_destroy(secfactory_led_class, secfactory_led);
device_create_fail:
	class_destroy(secfactory_led_class);
class_create_fail:
	if (ret)
		pr_err(" (err = %d)!\n", ret);
	return ret;
}

static int secfactory_led_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_device secfactory_led_device = {
	.name	= "secfactory_led_driver",
	.id		= -1,
};

static struct platform_device *secfactory_led_devices[] __initdata = {
		&secfactory_led_device,
};


static struct platform_driver secfactory_led_driver = {
	.probe = secfactory_led_probe,
	.remove = secfactory_led_remove,
	.driver = {
		.name = "secfactory_led_driver",
		.owner = THIS_MODULE,
	},
};

static int __init secfactory_led_init(void)
{
	int ret;
	ret = platform_driver_register(&secfactory_led_driver);
	if (ret) {
		pr_info("[secfactory_led] secfactory_led_init has been failed!!!\n");
		return ret;
	}
	platform_add_devices(secfactory_led_devices, ARRAY_SIZE(secfactory_led_devices));

	pr_info("[secfactory_led] secfactory_led_init has been initialized!!!\n");

	return 0;
}

static void __exit secfactory_led_exit(void)
{
	platform_driver_unregister(&secfactory_led_driver);
}

module_init(secfactory_led_init);
module_exit(secfactory_led_exit);

MODULE_AUTHOR("inbum.choi@samsung.com");
MODULE_DESCRIPTION("Samsung Factory LED driver");
MODULE_LICENSE("GPL");
