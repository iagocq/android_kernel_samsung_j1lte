/*
 * sr130pc20m Camera Driver
 *
 * Copyright (c) 2013 Marvell Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/platform_data/camera-mmp.h>
#include <linux/v4l2-mediabus.h>
#include "sr130pc20m.h"
#include "sr130pc20m_regs.h"

MODULE_DESCRIPTION("SR130PC20M Camera Driver");
MODULE_LICENSE("GPL");

//Tunning Binary define

static int debug;
module_param(debug, int, 0644);

static const struct sr130pc20m_datafmt sr130pc20m_colour_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
#if 0	
	{V4L2_MBUS_FMT_VYUY8_2X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_YVYU8_2X8, V4L2_COLORSPACE_JPEG},
#endif
};

static const struct v4l2_queryctrl sr130pc20m_controls[] = {
	{
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "brightness",
	}
};

static int video_mode;
static struct sr130pc20m_regval *regs_resolution;
static char *regs_resolution_name;
static struct sr130pc20m_regval *regs_display_setting;

static const struct v4l2_ctrl_ops sr130pc20m_ctrl_ops;
static struct v4l2_ctrl_config sr130pc20m_ctrl_video_mode_cfg = {
	.ops = &sr130pc20m_ctrl_ops,
	.id = V4L2_CID_PRIVATE_SR130PC20M_VIDEO_MODE,
	.name = "video mode",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = VIDEO_TO_NORMAL,
	.max = VIDEO_TO_CALL,
	.step = 1,
	.def = VIDEO_TO_NORMAL,
};

static struct sr130pc20m_regval *regs_res_all[] = {
	[SR130PC20M_FMT_VGA] = regs_res_vga,
	[SR130PC20M_FMT_1_3M] = regs_res_1_3m,
	[SR130PC20M_FMT_176X144] = sr130pc20m_176_144_size_regs,
	[SR130PC20M_FMT_352X288] = sr130pc20m_352_288_size_regs,
	[SR130PC20M_FMT_320X240] = sr130pc20m_320_240_size_regs,
	[SR130PC20M_FMT_528X432] = sr130pc20m_528_432_size_regs,
};

static struct sr130pc20m *to_sr130pc20m(const struct i2c_client
						 *client)
{
	return container_of(i2c_get_clientdata(client),
				struct sr130pc20m, subdev);
}

static int sr130pc20m_i2c_read(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 1,
			.buf    = (u8 *)&reg,
		},
		{
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 1,
			.buf    = val,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading register 0x%x!\n", reg);
		return ret;
	}
	v4l2_dbg(3, debug, client, "i2c_read: 0x%X : 0x%x\n", reg, val);
	
	return 0;
}

static int sr130pc20m_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct i2c_msg msg;
	struct sr130pc20m_regval buf;

	buf.addr = reg;
	buf.val = val;

	msg.addr    = client->addr;
	msg.flags   = 0;
	msg.len     = 2;
	msg.buf     = (u8 *)&buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x!\n", reg);
		return ret;
	}
	v4l2_dbg(3, debug, client, "i2c_read: 0x%X : 0x%x\n", reg, val);

	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sr130pc20m_write(struct i2c_client *c, u8 addr, u8 val)
{
	return sr130pc20m_i2c_write(c, addr, val);
}
#endif

/* The command register read, assumes Command_Rd_addH = 0x7000. */
static int sr130pc20m_read(struct i2c_client *client, u8 addr, u8 *val)
{
	return sr130pc20m_i2c_read(client, addr, val);
}

static int sr130pc20m_write_raw_array(struct v4l2_subdev *sd,
				  const struct sr130pc20m_regval *msg, char *name)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	while (1) {
		if(msg->addr == SR130PC20M_TERM && msg->val == 0)
			break;
		else if(msg->addr == SR130PC20M_TERM && msg->val != 0)		
		{
			msleep(msg->val*10);
			msg++;
			continue;
		}
		
		ret = sr130pc20m_i2c_write(client, msg->addr, msg->val);
		if (ret < 0)
			break;
		/* Assume that msg->addr is always less than 0xfffc */
		msg++;
	}
	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sr130pc20m_g_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return sr130pc20m_read(client, (u8) reg->reg,
				   (u8 *)&(reg->val));
}

static int sr130pc20m_s_register(struct v4l2_subdev *sd,
				 const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return sr130pc20m_write(client, (u8) reg->reg,
				(u8)reg->val);
}
#endif

static int sr130pc20m_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;
    printk("sr130 set stream: %d\n", enable);
	if (enable)
		ret = sr130pc20m_write_raw_array(sd, regs_start_stream, "regs_start_stream");
	else
		ret = sr130pc20m_write_raw_array(sd, regs_stop_stream, "regs_stop_stream");
	return ret;
}

static int sr130pc20m_enum_fmt(struct v4l2_subdev *sd,
		unsigned int index,
		enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(sr130pc20m_colour_fmts))
		return -EINVAL;
	*code = sr130pc20m_colour_fmts[index].code;
	return 0;
}

static inline enum sr130pc20m_res_support find_res_code(int width, int height)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(sr130pc20m_resolutions); i++) {
		if (sr130pc20m_resolutions[i].width == width &&
			sr130pc20m_resolutions[i].height == height)
			return sr130pc20m_resolutions[i].res_code;
	}
	return -EINVAL;
}

static struct sr130pc20m_regval *sr130pc20m_get_res_regs(int width, int height)
{
	enum sr130pc20m_res_support res_code;

	res_code = find_res_code(width, height);
	if (res_code == -EINVAL)
		return NULL;
	printk("get res_regs w:h = %d:%d, and res_code = %d\n\r", width, height, res_code);
	return regs_res_all[res_code];
}

static int sr130pc20m_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sr130pc20m *sr130pc20m = to_sr130pc20m(client);

	switch (mf->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
#if 0	
	case V4L2_MBUS_FMT_VYUY8_2X8:
	case V4L2_MBUS_FMT_YVYU8_2X8:
	case V4L2_MBUS_FMT_YUYV8_2X8:
#endif
		/* for sr130pc20m, only in normal preview status, it need to get res.*/
		regs_resolution = sr130pc20m_get_res_regs(mf->width, mf->height);
		if (!regs_resolution) {
			/* set default resolution */
			dev_warn(&client->dev,
					"sr130pc20m: res not support, set to VGA\n");
			mf->width = SR130PC20M_OUT_WIDTH_DEF;
			mf->height = SR130PC20M_OUT_HEIGHT_DEF;
			regs_resolution =
				sr130pc20m_get_res_regs(mf->width, mf->height);
		}
		sr130pc20m->rect.width = mf->width;
		sr130pc20m->rect.height = mf->height;
		/* colorspace should be set in host driver */
		mf->colorspace = V4L2_COLORSPACE_JPEG;
#if 0		
		if (mf->code == V4L2_MBUS_FMT_UYVY8_2X8)
			regs_display_setting = regs_display_uyvy_setting;
		else if (mf->code == V4L2_MBUS_FMT_VYUY8_2X8)
			regs_display_setting = regs_display_vyuy_setting;
		else if (mf->code == V4L2_MBUS_FMT_YUYV8_2X8)
			regs_display_setting = regs_display_yuyv_setting;
		else
			regs_display_setting = regs_display_yvyu_setting;
#endif
		break;
	default:
		/* This should never happen */
		mf->code = V4L2_MBUS_FMT_UYVY8_2X8;
		return -EINVAL;
	}
	return 0;
}

/*
 * each time before start streaming,
 * this function must be called
 */
static int sr130pc20m_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	int ret = 0;
	/*
	 * for sr130pc20m sensor output uyvy, if change, pls open this
	 * ret |= sr130pc20m_write_raw_array(sd, regs_display_setting);
	 */
	//if normal to video mode, it need to set FPS before size setting
	//other, if return from recording, need to reinit sensor
	if(video_mode == NORMAL_TO_VIDEO) {
		ret = sr130pc20m_write_raw_array(sd, sr130pc20m_fps_25_regs, "sr130pc20m_fps_25_regs");
	} else if(video_mode == VIDEO_TO_NORMAL) {
		ret = sr130pc20m_write_raw_array(sd, regs_start_setting, "regs_start_setting");
	}

	ret = sr130pc20m_write_raw_array(sd, regs_resolution, regs_resolution_name);
	/* restore video_mode to normal preview status */
	video_mode = NORMAL_STATUS;
	return ret;
}

static int sr130pc20m_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sr130pc20m *sr130pc20m = to_sr130pc20m(client);

	mf->width		= sr130pc20m->rect.width;
	mf->height		= sr130pc20m->rect.height;
	/* all belows are fixed */
	mf->code		= V4L2_MBUS_FMT_UYVY8_2X8;
	mf->field		= V4L2_FIELD_NONE;
	mf->colorspace		= V4L2_COLORSPACE_JPEG;
	return 0;
}

static int sr130pc20m_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	return 0;
}

static int sr130pc20m_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *crop)
{
	return 0;
}

static int sr130pc20m_enum_fsizes(struct v4l2_subdev *sd,
				struct v4l2_frmsizeenum *fsize)
{
	if (!fsize)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	if (fsize->index >= ARRAY_SIZE(sr130pc20m_resolutions))
		return -EINVAL;

	fsize->discrete.height = sr130pc20m_resolutions[fsize->index].height;
	fsize->discrete.width = sr130pc20m_resolutions[fsize->index].width;
	return 0;
}

static int sr130pc20m_set_video_mode(int value)
{
	video_mode = value;
	return 0;
}

static int sr130pc20m_init(struct v4l2_subdev *sd, u32 plat)
{
	int ret;

	ret = sr130pc20m_write_raw_array(sd, regs_start_setting, "regs_start_setting");
	return ret;
}

static int sr130pc20m_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_1_LANE;
	return 0;
}

static int sr130pc20m_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_PRIVATE_SR130PC20M_VIDEO_MODE:
		ret = sr130pc20m_set_video_mode(ctrl->val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}
static const struct v4l2_ctrl_ops sr130pc20m_ctrl_ops = {
	.s_ctrl = sr130pc20m_s_ctrl,
};

static struct v4l2_subdev_core_ops sr130pc20m_subdev_core_ops = {
	.init		= sr130pc20m_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= sr130pc20m_g_register,
	.s_register	= sr130pc20m_s_register,
#endif
};

static struct v4l2_subdev_video_ops sr130pc20m_subdev_video_ops = {
	.s_stream = sr130pc20m_s_stream,
	.g_mbus_fmt = sr130pc20m_g_fmt,
	.s_mbus_fmt = sr130pc20m_s_fmt,
	.try_mbus_fmt = sr130pc20m_try_fmt,
	.enum_framesizes = sr130pc20m_enum_fsizes,
	.enum_mbus_fmt = sr130pc20m_enum_fmt,
	.g_crop = sr130pc20m_g_crop,
	.s_crop = sr130pc20m_s_crop,
	.g_mbus_config = sr130pc20m_g_mbus_config,
};

static struct v4l2_subdev_ops sr130pc20m_subdev_ops = {
	.core = &sr130pc20m_subdev_core_ops,
	.video = &sr130pc20m_subdev_video_ops,
};

static int sr130pc20m_detect(struct i2c_client *client)
{
	int ret;
	u8 val = 0;

	/* revision */
	ret = sr130pc20m_read(client, 0x04, &val);
	printk("!!!!!!!!!!!!%x!!!!!!\n",val);
/*	
	 if (ret)
		return ret;
	if (val != 0xB4)
		return -ENODEV;
*/
	return 0;
}

static int sr130pc20m_probe(struct i2c_client *client,
				const struct i2c_device_id *did)
{
	struct sr130pc20m *sr130pc20m;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	ret = sr130pc20m_detect(client);
	if (ret)
		return ret;
	dev_info(&adapter->dev, "cam: SR130PC20M detect!\n");

	sr130pc20m = kzalloc(sizeof(struct sr130pc20m), GFP_KERNEL);
	if (!sr130pc20m)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&sr130pc20m->subdev, client, &sr130pc20m_subdev_ops);
	v4l2_ctrl_handler_init(&sr130pc20m->hdl, 1);
	v4l2_ctrl_new_custom(&sr130pc20m->hdl, &sr130pc20m_ctrl_video_mode_cfg, NULL);
	sr130pc20m->subdev.ctrl_handler = &sr130pc20m->hdl;
	if (sr130pc20m->hdl.error)
		return sr130pc20m->hdl.error;

	sr130pc20m->rect.left = 0;
	sr130pc20m->rect.top = 0;
	sr130pc20m->rect.width = SR130PC20M_OUT_WIDTH_DEF;
	sr130pc20m->rect.height = SR130PC20M_OUT_HEIGHT_DEF;
	sr130pc20m->pixfmt = V4L2_PIX_FMT_UYVY;
	return ret;
}

static int sr130pc20m_remove(struct i2c_client *client)
{
	struct sr130pc20m *sr130pc20m = to_sr130pc20m(client);

	v4l2_ctrl_handler_free(&sr130pc20m->hdl);
	kfree(sr130pc20m);
	return 0;
}

static const struct i2c_device_id sr130pc20m_idtable[] = {
	{"sr130pc20m", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sr130pc20m_idtable);

static struct i2c_driver sr130pc20m_driver = {
	.driver = {
		   .name = "sr130pc20m",
		   },
	.probe = sr130pc20m_probe,
	.remove = sr130pc20m_remove,
	.id_table = sr130pc20m_idtable,
};

static int __init sr130pc20m_mod_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&sr130pc20m_driver);
	return ret;
}

static void __exit sr130pc20m_mod_exit(void)
{
	i2c_del_driver(&sr130pc20m_driver);
}

module_init(sr130pc20m_mod_init);
module_exit(sr130pc20m_mod_exit);
