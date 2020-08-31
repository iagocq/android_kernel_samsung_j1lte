/*
 * drivers/video/mmp/panel/mipi_hx8369b.c
 * active panel using DSI interface to do init
 *
 * Copyright (C) 2013 Marvell Technology Group Ltd.
 * Authors: Yonghai Huang <huangyh@marvell.com>
 *		Xiaolong Ye <yexl@marvell.com>
 *          Guoqing Li <ligq@marvell.com>
 *          Lisa Du <cldu@marvell.com>
 *          Zhou Zhu <zzhu3@marvell.com>
 *          Jing Xiang <jxiang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/device.h>
#include <linux/lcd.h>
#include <video/mmp_disp.h>
#include <video/mipi_display.h>
#include "panel_hx8369b_param.h"

/*FIXME*/
static int dsi_status = 1;

struct hx8369b_plat_data {
	struct mmp_panel *panel;
	void (*plat_onoff)(int status);
	void (*plat_set_backlight)(struct mmp_panel *panel, int level);
	struct backlight_device *bl;
};

static DEFINE_SPINLOCK(bl_ctrl_lock);
#define DATA_START	(40)
#define LOW_BIT_L	(40)
#define LOW_BIT_H	(10)
#define HIGH_BIT_L	(10)
#define HIGH_BIT_H	(40)
#define END_DATA_L	(10)
#define END_DATA_H	(400)
#define MAX_BRI		(0xff)
#define MIN_BRI		(0x0)
#define GPIO_BL_CTRL		(97)

static void hx8369b_set_brightness(struct mmp_panel *panel, int level)
{
	char i;
	static int is_init = 0;
	unsigned char brightness;
	int gpio_bl = GPIO_BL_CTRL;
	unsigned long flags;

	if (!dsi_status)
		return;

	if (level == 0) {
		gpio_set_value(gpio_bl, 0);
		udelay(200);
		is_init = 0;
		return;
	}

	if (!is_init) {
		spin_lock_irqsave(&bl_ctrl_lock, flags);
		printk("backlight init\n");
		gpio_set_value(gpio_bl, 1);
		udelay(200);
		gpio_set_value(gpio_bl, 0);
		udelay(300);
		gpio_set_value(gpio_bl, 1);
		udelay(400);
		spin_unlock_irqrestore(&bl_ctrl_lock, flags);

		is_init = 1;
	}

	/*
	 * KTD 2801
	*/
	if (level > MAX_BRI || level < MIN_BRI)
		brightness = MAX_BRI;
	else
		brightness = (unsigned char)level;


	printk("set_brightness : level(%d)\n", brightness);

	spin_lock_irqsave(&bl_ctrl_lock, flags);
	gpio_set_value(gpio_bl, 1);
	udelay(DATA_START);

	for (i = 0; i < 8; i++) {
		if (brightness & (0x80 >> i)) {
			gpio_set_value(gpio_bl, 0);
			udelay(HIGH_BIT_L);
			gpio_set_value(gpio_bl, 1);
			udelay(HIGH_BIT_H);
		} else {
			gpio_set_value(gpio_bl, 0);
			udelay(LOW_BIT_L);
			gpio_set_value(gpio_bl, 1);
			udelay(LOW_BIT_H);
		}
	}

	gpio_set_value(gpio_bl, 0);
	udelay(END_DATA_L);
	gpio_set_value(gpio_bl, 1);
	udelay(END_DATA_H);
	spin_unlock_irqrestore(&bl_ctrl_lock, flags);

}

static void hx8369b_panel_on(struct mmp_panel *panel, int status)
{
	if (status) {
		mmp_panel_dsi_ulps_set_on(panel, 0);
		mmp_panel_dsi_tx_cmd_array(panel, hx8369b_display_on_cmds,
			ARRAY_SIZE(hx8369b_display_on_cmds));
	} else {
		mmp_panel_dsi_tx_cmd_array(panel, hx8369b_display_off_cmds,
			ARRAY_SIZE(hx8369b_display_off_cmds));
		mmp_panel_dsi_ulps_set_on(panel, 1);
	}
}

#ifdef CONFIG_OF
static void hx8369b_panel_power(struct mmp_panel *panel, int skip_on, int on)
{
	static struct regulator  *lcd_avdd;
	int lcd_rst_n, ret = 0;

	lcd_rst_n = of_get_named_gpio(panel->dev->of_node, "rst_gpio", 0);
	if (lcd_rst_n < 0) {
		pr_err("%s: of_get_named_gpio failed\n", __func__);
		return;
	}

	if (gpio_request(lcd_rst_n, "lcd reset gpio")) {
		pr_err("gpio %d request failed\n", lcd_rst_n);
		return;
	}

	if (panel->is_avdd && (!lcd_avdd)) {
		lcd_avdd = regulator_get(panel->dev, "avdd");
		if (IS_ERR(lcd_avdd)) {
			pr_err("%s regulator get error!\n", __func__);
			ret = -EIO;
			goto regu_lcd_iovdd;
		}
	}
	if (on) {
		if (panel->is_avdd && lcd_avdd) {
			regulator_set_voltage(lcd_avdd, 2800000, 2800000);
			ret = regulator_enable(lcd_avdd);
			if (ret < 0)
				goto regu_lcd_avdd;
		}


		usleep_range(10000, 12000);
		if (!skip_on) {
			gpio_direction_output(lcd_rst_n, 0);
			usleep_range(10000, 12000);
			gpio_direction_output(lcd_rst_n, 1);
			usleep_range(10000, 12000);
		}
	} else {
		/* set panel reset */
		gpio_direction_output(lcd_rst_n, 0);

		if (panel->is_avdd && lcd_avdd)
			regulator_disable(lcd_avdd);
	}
regu_lcd_avdd:
	regulator_put(lcd_avdd);

regu_lcd_iovdd:
	gpio_free(lcd_rst_n);

	lcd_avdd = NULL;

}
#else
static void hx8369b_panel_power(struct mmp_panel *panel, int skip_on, int on) {}
#endif

void j1_set_brightness(struct mmp_panel *panel)
{
	struct hx8369b_plat_data *plat = panel->plat_data;
	panel->set_brightness(panel, plat->bl->props.brightness);
}

static void hx8369b_set_status(struct mmp_panel *panel, int status)
{
	struct hx8369b_plat_data *plat = panel->plat_data;
	int skip_on = (status == MMP_ON_REDUCED);

	if (status_is_on(status)) {
		/* power on */
		if (plat->plat_onoff)
			plat->plat_onoff(1);
		else
			hx8369b_panel_power(panel, skip_on, 1);
		if (!skip_on)
			hx8369b_panel_on(panel, 1);
		dsi_status = 1;
	} else if (status_is_off(status)) {
		hx8369b_panel_on(panel, 0);
		/* power off */
		if (plat->plat_onoff)
			plat->plat_onoff(0);
		else
			hx8369b_panel_power(panel, 0, 0);
		dsi_status = 0;
	} else
		dev_warn(panel->dev, "set status %s not supported\n",
					status_name(status));
}

static struct mmp_mode mmp_modes_hx8369b[] = {
	[0] = {
		.pixclock_freq = 34890240,//69264000,
		.refresh = 60,
		.xres = 480,
		.yres = 800,
		.real_xres = 480,
		.real_yres = 800,
		.hsync_len = 60,
		.left_margin = 60,
		.right_margin = 100,
		.vsync_len = 4,
		.upper_margin = 12,
		.lower_margin = 10,
		.invert_pixclock = 0,
		.pix_fmt_out = PIXFMT_BGR888PACK,
		.hsync_invert = 0,
		.vsync_invert = 0,
		.height = 110,
		.width = 62,
	},
};

static int hx8369b_get_modelist(struct mmp_panel *panel,
		struct mmp_mode **modelist)
{
	*modelist = mmp_modes_hx8369b;
	return 1;
}

static struct mmp_panel panel_hx8369b = {
	.name = "hx8369b",
	.panel_type = PANELTYPE_DSI_VIDEO,
	.is_iovdd = 0,
	.is_avdd = 0,
	.get_modelist = hx8369b_get_modelist,
	.set_status = hx8369b_set_status,
	.set_brightness = hx8369b_set_brightness,
};

static int hx8369b_bl_update_status(struct backlight_device *bl)
{
	struct hx8369b_plat_data *data = dev_get_drvdata(&bl->dev);
	struct mmp_panel *panel = data->panel;
	int level;

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	/* If there is backlight function of board, use it */
	if (data && data->plat_set_backlight) {
		data->plat_set_backlight(panel, level);
		return 0;
	}

	if (panel && panel->set_brightness)
		panel->set_brightness(panel, level);

	return 0;
}

static int hx8369b_bl_get_brightness(struct backlight_device *bl)
{
	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		return bl->props.brightness;

	return 0;
}

static const struct backlight_ops hx8369b_bl_ops = {
	.get_brightness = hx8369b_bl_get_brightness,
	.update_status  = hx8369b_bl_update_status,
};

static ssize_t lcd_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "BOE_HX8369B\n");
}
static DEVICE_ATTR(lcd_type, S_IRUGO | S_IXOTH, lcd_type_show, NULL);

struct class *lcd_class;
struct device *lcd_class_node_dev;

static int hx8369b_probe(struct platform_device *pdev)
{
	struct mmp_mach_panel_info *mi;
	struct hx8369b_plat_data *plat_data;
	struct device_node *np = pdev->dev.of_node;
	const char *path_name;
	struct backlight_properties props;
	struct backlight_device *bl;
	int ret;
	int gpio_bl;

	printk("hx8369b_probe\n");

	plat_data = kzalloc(sizeof(*plat_data), GFP_KERNEL);
	if (!plat_data)
		return -ENOMEM;

	lcd_class = class_create(THIS_MODULE, "lcd");
	
	if (IS_ERR(lcd_class)) {
		pr_err("failed to create lcd_class\n");
	}

	lcd_class_node_dev = device_create(lcd_class, NULL, 0, NULL, "panel");

	if (IS_ERR(lcd_class_node_dev)) {
		pr_err("Failed to create device(panel)!\n");
	}

	device_create_file(lcd_class_node_dev, &dev_attr_lcd_type);


	if (IS_ENABLED(CONFIG_OF)) {
		ret = of_property_read_string(np, "marvell,path-name",
				&path_name);
		if (ret < 0) {
			kfree(plat_data);
			return ret;
		}
		panel_hx8369b.plat_path_name = path_name;

		if (of_find_property(np, "iovdd-supply", NULL))
			panel_hx8369b.is_iovdd = 1;

		if (of_find_property(np, "avdd-supply", NULL))
			panel_hx8369b.is_avdd = 1;
	} else {
		/* get configs from platform data */
		mi = pdev->dev.platform_data;
		if (mi == NULL) {
			dev_err(&pdev->dev, "no platform data defined\n");
			kfree(plat_data);
			return -EINVAL;
		}
		plat_data->plat_onoff = mi->plat_set_onoff;
		panel_hx8369b.plat_path_name = mi->plat_path_name;
		plat_data->plat_set_backlight = mi->plat_set_backlight;
	}

	plat_data->panel = &panel_hx8369b;
	panel_hx8369b.plat_data = plat_data;
	panel_hx8369b.dev = &pdev->dev;
	mmp_register_panel(&panel_hx8369b);

	/*
	 * if no panel or plat associate backlight control,
	 * don't register backlight device here.
	 */
	if (!panel_hx8369b.set_brightness && !plat_data->plat_set_backlight)
		return 0;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = 255;
	props.type = BACKLIGHT_RAW;

	bl = backlight_device_register("panel", &pdev->dev, plat_data,
			&hx8369b_bl_ops, &props);
	if (IS_ERR(bl)) {
		ret = PTR_ERR(bl);
		dev_err(&pdev->dev, "failed to register lcd-backlight\n");
		return ret;
	}

	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.brightness = 40;

	/*FIXME*/
	plat_data->bl = bl;

	if (gpio_request(GPIO_BL_CTRL, "lcd backlight")) {
		pr_err("gpio %d request failed\n", gpio_bl);
		return;
	}

	printk("hx8369b_probe done\n");

	return 0;
}

static int hx8369b_remove(struct platform_device *dev)
{
	mmp_unregister_panel(&panel_hx8369b);
	kfree(panel_hx8369b.plat_data);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_hx8369b_dt_match[] = {
	{ .compatible = "marvell,mmp-hx8369b" },
	{},
};
#endif

static struct platform_driver hx8369b_driver = {
	.driver		= {
		.name	= "mmp-hx8369b",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mmp_hx8369b_dt_match),
	},
	.probe		= hx8369b_probe,
	.remove		= hx8369b_remove,
};

static int hx8369b_init(void)
{
	return platform_driver_register(&hx8369b_driver);
}
static void hx8369b_exit(void)
{
	platform_driver_unregister(&hx8369b_driver);
}
module_init(hx8369b_init);
module_exit(hx8369b_exit);

MODULE_AUTHOR("Yonghai Huang <huangyh@marvell.com>");
MODULE_DESCRIPTION("Panel driver for MIPI panel HX8369b");
MODULE_LICENSE("GPL");
