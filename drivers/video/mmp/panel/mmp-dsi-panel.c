/* drivers/video/mmp/panel/mmp-dsi-panel.c
 *
 * Copyright (C) 2013 Samsung Electronics Co, Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/lcd.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <video/mmp_disp.h>
#include <video/mmp_esd.h>
#include <video/mipi_display.h>
#include <linux/platform_data/gen-panel.h>

static struct mmp_panel mmp_dsi_panel;

static inline struct lcd *panel_to_lcd(const struct mmp_panel *panel)
{
	return (struct lcd *)panel->plat_data;
}
static inline struct mmp_panel *lcd_to_panel(const struct lcd *lcd)
{
	return (struct mmp_panel *)lcd->pdata;
}

static void gpmode_to_mmpmode(struct mmp_mode *mmpmode,
		struct gen_panel_mode *gpmode)
{
	mmpmode->refresh = gpmode->refresh;
	mmpmode->xres = gpmode->xres;
	mmpmode->yres = gpmode->yres;
	mmpmode->real_xres = gpmode->real_xres;
	mmpmode->real_yres = gpmode->real_yres;

	mmpmode->left_margin = gpmode->left_margin;
	mmpmode->right_margin = gpmode->right_margin;
	mmpmode->upper_margin = gpmode->upper_margin;
	mmpmode->lower_margin = gpmode->lower_margin;
	mmpmode->hsync_len = gpmode->hsync_len;
	mmpmode->vsync_len = gpmode->vsync_len;

	mmpmode->pixclock_freq = gpmode->refresh *
		(gpmode->xres + gpmode->right_margin +
		 gpmode->left_margin + gpmode->hsync_len) *
		(gpmode->yres + gpmode->upper_margin +
		 gpmode->lower_margin + gpmode->vsync_len);

	mmpmode->height = gpmode->height;
	mmpmode->width = gpmode->width;
}

static inline int mmp_panel_tx_cmd_array(const struct lcd *lcd,
		const void *cmds, const int count)
{
	struct mmp_panel *panel = lcd_to_panel(lcd);
	int ret, retry = 3;

	/* TODO: Retry send cmds */
	do {
		ret = mmp_panel_dsi_tx_cmd_array(panel,
				(struct mmp_dsi_cmd_desc *)cmds, count);
	} while (retry-- && ret < 0);

	return ret;
}

static inline int mmp_panel_rx_cmd_array(const struct lcd *lcd,
		u8 *buf, const void *cmds, const int count)
{
	struct mmp_panel *panel = lcd_to_panel(lcd);
	struct mmp_dsi_buf dbuf;
	int ret;

	ret = mmp_panel_dsi_rx_cmd_array(panel, &dbuf,
			(struct mmp_dsi_cmd_desc *)cmds, count);

	memcpy(buf, dbuf.data, dbuf.length);

	return dbuf.length;
}

int mmp_panel_parse_dt(const struct device_node *np)
{
	struct mmp_panel *panel = &mmp_dsi_panel;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);
	struct mmp_dsi *dsi = mmp_path_to_dsi(path);

	pr_info("%s enter\n", __func__);
	if (!np) {
		pr_info("%s, device_node is null\n", __func__);
		return -EINVAL;
	}

	if (!dsi) {
		pr_info("%s, dsi is null\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_bool(np, "gen-panel-master-mode")) {
		pr_info("%s, master mode\n", __func__);
		dsi->setting.master_mode = 1;
	} else {
		pr_info("%s, slave mode\n", __func__);
		dsi->setting.master_mode = 0;
	}
	if (of_property_read_bool(np, "gen-panel-hfp-en")) {
		pr_info("%s, hfp_en\n", __func__);
		dsi->setting.hfp_en = 1;
	}
	if (of_property_read_bool(np, "gen-panel-hbp-en")) {
		pr_info("%s, hbp_en\n", __func__);
		dsi->setting.hbp_en = 1;
	}

	return 0;
}

static const struct gen_panel_ops gp_ops = {
	.tx_cmds = mmp_panel_tx_cmd_array,
	.rx_cmds = mmp_panel_rx_cmd_array,
#if CONFIG_OF
	.parse_dt = mmp_panel_parse_dt,
#endif
};

static void mmp_panel_set_status(struct mmp_panel *panel, int status)
{
	struct lcd *lcd = panel_to_lcd(panel);

	gen_panel_set_status(lcd, status);

	return;
}

static void mmp_panel_start(struct mmp_panel *panel, int status)
{
	struct lcd *lcd = panel_to_lcd(panel);

	gen_panel_start(lcd, status);

	return;
}

static void mmp_panel_esd_onoff(struct mmp_panel *panel, int status)
{
	struct lcd *lcd = panel_to_lcd(panel);

	if (!lcd->esd_en)
		return;

	if (status)
		esd_start(&panel->esd);
	else
		esd_stop(&panel->esd);
}

static int mmp_panel_get_status(struct mmp_panel *panel)
{
	return ESD_STATUS_OK;
}

static void mmp_panel_esd_recover(struct mmp_panel *panel)
{
	struct mmp_path *path =
		mmp_get_path(panel->plat_path_name);

	esd_panel_recover(path);
	pr_info("%s done\n", __func__);
}

static struct mmp_mode mmp_panel_modes[] = {
	[0] = {
		.refresh = 60,
		.invert_pixclock = 0,
		.pix_fmt_out = PIXFMT_BGR888PACK,
		.hsync_invert = 0,
		.vsync_invert = 0,
	},
};

static int mmp_panel_get_modelist(struct mmp_panel *panel,
		struct mmp_mode **modelist)
{
	*modelist = mmp_panel_modes;
	return 1;
}

static struct mmp_panel mmp_dsi_panel = {
	.name = "dummy panel",
	.panel_type = PANELTYPE_DSI_VIDEO,
	.get_modelist = mmp_panel_get_modelist,
	.set_status = mmp_panel_set_status,
	.panel_start = mmp_panel_start,
	.panel_esd_recover = mmp_panel_esd_recover,
	.get_status = mmp_panel_get_status,
	.esd_set_onoff = mmp_panel_esd_onoff,
};

static int mmp_panel_probe(struct platform_device *pdev)
{
	struct lcd *lcd;
	struct mmp_panel *panel = &mmp_dsi_panel;
	int ret;

	pr_info("called %s\n", __func__);

	lcd = kzalloc(sizeof(*lcd), GFP_KERNEL);
	if (unlikely(!lcd))
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF)) {
		/* 0. Parse : path-name*/
		ret = of_property_read_string(pdev->dev.of_node,
				"marvell,path-name", &panel->plat_path_name);
		if (unlikely(ret)) {
			dev_err(&pdev->dev, "fail to parse path-name\n");
			goto err_parse_path_name;
		}
	}

	lcd->pdata = (void *)panel;
	lcd->ops = &gp_ops;

	ret = gen_panel_probe(pdev->dev.of_node, lcd);
	if (unlikely(ret))
		goto err_gen_panel_probe;

	gpmode_to_mmpmode(&mmp_panel_modes[0], &lcd->mode);

	if (lcd->esd_en) {
		panel->esd_type = lcd->esd_type;
		panel->esd_gpio = lcd->esd_gpio;
		esd_init(panel);
	}
	panel->plat_data = lcd;
	panel->dev = &pdev->dev;
	panel->name = lcd->panel_name;
	mmp_register_panel(panel);

	return 0;

err_parse_path_name:
err_gen_panel_probe:
	pr_info("%s dts parsing error\n", __func__);
	kfree(lcd);
	return ret;
}

static int mmp_panel_remove(struct platform_device *pdev)
{
	struct lcd *lcd = dev_get_drvdata(&pdev->dev);
	struct mmp_panel *panel = lcd_to_panel(lcd);

	gen_panel_remove(lcd);
	mmp_unregister_panel(panel);

	return 0;
}

void mmp_dsi_panel_shutdown(struct platform_device *pdev)
{
	struct mmp_panel *panel = &mmp_dsi_panel;
	mmp_panel_set_status(panel, MMP_OFF);
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_panel_dt_match[] = {
	{ .compatible = "marvell,mmp-dsi-panel" },
	{},
};
#endif
static struct platform_driver mmp_panel_driver = {
	.driver     = {
		.name   = "mmp-dsi-panel",
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(mmp_panel_dt_match),
#endif
	},

	.probe      = mmp_panel_probe,
	.remove     = mmp_panel_remove,
	.shutdown	= mmp_dsi_panel_shutdown,
};

static int mmp_panel_module_init(void)
{
	return platform_driver_register(&mmp_panel_driver);
}
static void mmp_panel_module_exit(void)
{
	platform_driver_unregister(&mmp_panel_driver);
}
module_init(mmp_panel_module_init);
module_exit(mmp_panel_module_exit);

MODULE_DESCRIPTION("MMP DSI PANEL DRIVER");
MODULE_LICENSE("GPL");
