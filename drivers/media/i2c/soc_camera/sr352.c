/*
 * sr352 Camera Driver
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
#include "sr352.h"
#include "sr352_regs.h"

MODULE_DESCRIPTION("SR352 Camera Driver");
MODULE_LICENSE("GPL");

static int debug;
module_param(debug, int, 0644);

static const struct sr352_datafmt sr352_colour_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
#if 0
	{V4L2_MBUS_FMT_VYUY8_2X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_YVYU8_2X8, V4L2_COLORSPACE_JPEG},
#endif
};

bool need_reinit = false;
bool record_use_preview = false;
static const struct v4l2_ctrl_ops sr352_ctrl_ops;
static int sensor_mode;
static struct sr352_regval *regs_resolution;
static struct sr352_regval *regs_resolution_setting_before;
static struct sr352_regval *regs_resolution_setting_after;

static struct v4l2_ctrl_config sr352_ctrl_sensor_mode_cfg = {
	.ops = &sr352_ctrl_ops,
	.id = V4L2_CID_PRIVATE_SR352_SENSOR_MODE,
	.name = "video mode",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = VIDEO_TO_NORMAL,
	.max = NORMAL_TO_CAPTURE,
	.step = 1,
	.def = VIDEO_TO_NORMAL,
};

static struct sr352_resolution regs_res_recording[] = {
	{SR352_FMT_176X144,	regs_res_preview_176x144},
	{SR352_FMT_320X240,	regs_res_preview_320x240},
	{SR352_FMT_352X288,	regs_res_preview_352x288},
	{SR352_FMT_VGA,		regs_res_preview_vga},
	{SR352_FMT_720X480,	regs_res_preview_720x480},
	{SR352_FMT_800X480,	regs_res_preview_800x480},
	{SR352_FMT_704X576,	regs_res_preview_704x576},
	{SR352_FMT_800X600,	regs_res_preview_800x600},
	{SR352_FMT_HD,		regs_res_recording_HD},
	{SR352_FMT_1024X576,	regs_res_preview_1024x576},
	{SR352_FMT_1024X768,	regs_res_preview_1024x768},
	{SR352_FMT_END, NULL},
};

static struct sr352_resolution regs_res_capture[] = {
	{SR352_FMT_VGA,			regs_res_capture_vga}, 
	{SR352_FMT_800X480,		regs_res_capture_800x480}, 
	{SR352_FMT_960X720,		regs_res_capture_960x720}, 
	{SR352_FMT_1280X720,		regs_res_capture_1280x720},
	{SR352_FMT_1280X960,		regs_res_capture_1280x960},
	{SR352_FMT_1600X960,		regs_res_capture_1600x960},
	{SR352_FMT_1600X1200,	regs_res_capture_1600x1200},
	{SR352_FMT_2048X1152,	regs_res_capture_2048x1152},
	{SR352_FMT_2048X1536,	regs_res_capture_2048x1536},
	{SR352_FMT_END, NULL},
};

static struct sr352_resolution regs_res_preview[] = {
	{SR352_FMT_VGA,		regs_res_preview_vga_enter},
	{SR352_FMT_720X480,	regs_res_preview_720x480_enter},
	{SR352_FMT_800X480,	regs_res_preview_800x480_enter},
	{SR352_FMT_704X576,	regs_res_preview_704x576_enter},
	{SR352_FMT_1024X576,	regs_res_preview_1024x576_enter},
	{SR352_FMT_1024X640,	regs_res_preview_1024x640_enter},
	{SR352_FMT_1024X768,	regs_res_preview_1024x768_enter},
	{SR352_FMT_END, NULL},
};

static struct sr352 *to_sr352(const struct i2c_client
						 *client)
{
	return container_of(i2c_get_clientdata(client),
				struct sr352, subdev);
}

static int sr352_i2c_read(struct i2c_client *client, u8 reg, u8 *val)
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
	v4l2_dbg(3, debug, client, "i2c_read: 0x%X : 0x%x\n", reg, *val);

	return 0;
}

static int sr352_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct i2c_msg msg;
	struct sr352_regval buf;

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
	v4l2_dbg(3, debug, client, "i2c_write: 0x%0X : 0x%0x\n", reg, val);

	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sr352_write(struct i2c_client *c, u8 addr, u8 val)
{
	return sr352_i2c_write(c, addr, val);
}
#endif

/* The command register read, assumes Command_Rd_addH = 0x7000. */
static int sr352_read(struct i2c_client *client, u8 addr, u8 *val)
{
	return sr352_i2c_read(client, addr, val);
}

static int sr352_write_raw_array(struct v4l2_subdev *sd,
				  const struct sr352_regval *msg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	while (! ((msg->addr == SR352_TERM) && (msg->val == SR352_TERM))) {
		if( msg->addr == SR352_TERM ) {
			msleep(msg->val * 10);
			msg++;
			continue;
		}
		ret = sr352_i2c_write(client, msg->addr, msg->val);
		if (ret < 0)
			break;
		/* Assume that msg->addr is always less than 0xfffc */
		msg++;
	}

	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sr352_g_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return sr352_read(client, (u8) reg->reg,
				   (u8 *)&(reg->val));
}

static int sr352_s_register(struct v4l2_subdev *sd,
				 const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return sr352_write(client, (u8) reg->reg,
				(u8)reg->val);
}
#endif

static int sr352_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;

	if (enable)
		ret = sr352_write_raw_array(sd, regs_start_stream);
	else
		ret = sr352_write_raw_array(sd, regs_stop_stream);
	return ret;
}

static int sr352_enum_fmt(struct v4l2_subdev *sd,
		unsigned int index,
		enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(sr352_colour_fmts))
		return -EINVAL;
	*code = sr352_colour_fmts[index].code;
	return 0;
}

static inline enum sr352_res_support find_res_code(int width, int height)
{
	int i;
	printk("aibing debug find_res_code w:h %d:%d\n\r", width, height);
	for (i = 0; i < ARRAY_SIZE(sr352_resolutions); i++) {
		if (sr352_resolutions[i].width == width &&
			sr352_resolutions[i].height == height)
			return sr352_resolutions[i].res_code;
	}
	return -EINVAL;
}

static struct sr352_regval *sr352_res_get_support(struct sr352_resolution *sr352_res, int res_code)
{
	while(sr352_res->regs_resolution) {
		if(sr352_res->res_code == res_code) {
			return sr352_res->regs_resolution;
		}
		sr352_res++;
	}
	return NULL;
}

static struct sr352_regval *sr352_get_res_regs(int width, int height)
{
	enum sr352_res_support res_code;
	struct sr352_regval *regs_res_tmp;

	res_code = find_res_code(width, height);
	if (res_code == -EINVAL)
		return NULL;
	printk("aibing debug get_res_regs sensor_mode: %d  res_code: %d \n", sensor_mode, res_code);
	switch(sensor_mode) {
	case NORMAL_TO_VIDEO:
		regs_res_tmp = sr352_res_get_support(regs_res_recording, res_code);
		if(regs_res_tmp) {
			if(res_code == SR352_FMT_HD) {
				printk("aibing debug HD recording mode \n");
				//return from HD recording mode need to reinit sensor, as vender suggest
				need_reinit = true;
				//HD mode recording, need to add 50HZ setting before HD setting
				regs_resolution_setting_before = regs_res_recording_50Hz_HD;
			} else {
				//other mode recording, need to add 50HZ setting after size setting
				regs_resolution_setting_after = regs_res_recording_50Hz_30fps;
			}
			return regs_res_tmp;
		}
		//sometimes, we use preview setting to do recording if it's not support in
		//recording table
		//We just to need to modify sensor_mode to check with Normal status switch
		sensor_mode = NORMAL_STATUS;
		return NULL;
	case NORMAL_TO_CAPTURE:
		//sometimes, we use preview setting to do capture if it's not support in
		//capture table
		regs_res_tmp = sr352_res_get_support(regs_res_capture, res_code);
		if(regs_res_tmp) {
			return regs_res_tmp;
		}
		//if not support in capture table, set sensor_mode to normal, and try to
		//get the resolution from preview
		sensor_mode = NORMAL_STATUS;
		return NULL;
	case VIDEO_TO_NORMAL:
		//As in HD mode recording, it need to reinit sensor, not use such setting
		if(!need_reinit) {
			regs_resolution_setting_before = regs_res_recording_50Hz_modeoff;
		}
		//already VIDEO_TO_NORMAL, so not record mode, keep this flag to false
		return sr352_res_get_support(regs_res_preview, res_code);
	case CAPTURE_TO_NORMAL:
		return sr352_res_get_support(regs_res_preview, res_code);
	case NORMAL_STATUS:
		regs_res_tmp = sr352_res_get_support(regs_res_preview, res_code);
		if(regs_res_tmp) {
			return regs_res_tmp;
		}
		//if normal state, but do not has the resolution for such size
		//use recording table to try.
		sensor_mode = NORMAL_TO_VIDEO;
		return NULL;
	default:
		return NULL;
	}
}

static int sr352_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sr352 *sr352 = to_sr352(client);

	switch (mf->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
	case V4L2_MBUS_FMT_YVYU8_2X8:
	case V4L2_MBUS_FMT_YUYV8_2X8:
		/* for sr352, only in normal preview status, it need to get res.*/
		printk("aibing debug try_fmt w:h = %d:%d \n", mf->width, mf->height);
		regs_resolution = sr352_get_res_regs(mf->width, mf->height);
		if (!regs_resolution) {
			/* set default resolution */
			dev_warn(&client->dev,
					"aibing debug sr352: res not support, set to VGA\n");
			//such scenario, modify the sensor mode already, and try again.
			regs_resolution =
				sr352_get_res_regs(mf->width, mf->height);
		}
		sr352->rect.width = mf->width;
		sr352->rect.height = mf->height;
		/* colorspace should be set in host driver */
		mf->colorspace = V4L2_COLORSPACE_JPEG;
#if 0
		if (mf->code == V4L2_MBUS_FMT_UYVY8_2X8)
			regs_display_setting = regs_display_uyvy_setting;
		else if (mf->code == V4L2_MBUS_FMT_VYUY8_2X8) 
			regs_display_setting = regs_display_vyuy_setting; 
		else if (mf->code == V4L2_MBUS_FMT_YUYV8_2X8) 
			regs_display_setting = regs_display_yuyv_setting; 
		else regs_display_setting = regs_display_yvyu_setting;
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
static int sr352_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	int ret = 0;

	/*
	 * for sr352 sensor output uyvy, if change, pls open this
	 * ret |= sr352_write_raw_array(sd, regs_display_setting);
	 */
	if(need_reinit) {
		need_reinit = false;		
		ret = sr352_write_raw_array(sd, regs_start_setting);
		if(ret < 0) {
			goto Fail;
		}
		ret = sr352_write_raw_array(sd, regs_flicker_50HZ_setting);
		if(ret < 0) {
			goto Fail;
		}
	}

	if(regs_resolution_setting_before) {
		printk("aibing debug write regs_resolution_setting_before\n");
		ret = sr352_write_raw_array(sd, regs_resolution_setting_before);
		if(ret < 0) {
			goto Fail;
		}
	}
	if(regs_resolution) {
		printk("aibing debug write regs_resolution\n");
		ret = sr352_write_raw_array(sd, regs_resolution);
		if(ret < 0) {
			goto Fail;
		}
	}
	if(regs_resolution_setting_after) {
		printk("aibing debug write regs_resolution_setting_after \n");
		sr352_write_raw_array(sd, regs_resolution_setting_after);
		if(ret < 0) {
			goto Fail;
		}
	}
Fail:
	regs_resolution_setting_after = NULL;
	regs_resolution_setting_before = NULL;
	regs_resolution = NULL; 
	/* restore sensor_mode to normal preview status */
	sensor_mode = NORMAL_STATUS;
	return ret;
}

static int sr352_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sr352 *sr352 = to_sr352(client);

	mf->width		= sr352->rect.width;
	mf->height		= sr352->rect.height;
	/* all belows are fixed */
	mf->code		= V4L2_MBUS_FMT_UYVY8_2X8;
	mf->field		= V4L2_FIELD_NONE;
	mf->colorspace		= V4L2_COLORSPACE_JPEG;
	return 0;
}

static int sr352_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	return 0;
}

static int sr352_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *crop)
{
	return 0;
}

static int sr352_enum_fsizes(struct v4l2_subdev *sd,
				struct v4l2_frmsizeenum *fsize)
{
	if (!fsize)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	if (fsize->index >= ARRAY_SIZE(sr352_resolutions))
		return -EINVAL;

	fsize->discrete.height = sr352_resolutions[fsize->index].height;
	fsize->discrete.width = sr352_resolutions[fsize->index].width;
	return 0;
}

static int sr352_set_sensor_mode(int value)
{
	printk("aibing debug set sensormode: %d\n", value);
	sensor_mode = value;
	return 0;
}

static int sr352_init(struct v4l2_subdev *sd, u32 plat)
{
	int ret;
	ret = sr352_write_raw_array(sd, regs_start_setting);
	ret = sr352_write_raw_array(sd, regs_flicker_50HZ_setting);
	sensor_mode = NORMAL_STATUS;
	return ret;
}

static int sr352_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_1_LANE;
	return 0;
}

static int sr352_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_PRIVATE_SR352_SENSOR_MODE:
		ret = sr352_set_sensor_mode(ctrl->val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}
static const struct v4l2_ctrl_ops sr352_ctrl_ops = {
	.s_ctrl = sr352_s_ctrl,
};

static struct v4l2_subdev_core_ops sr352_subdev_core_ops = {
	.init		= sr352_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= sr352_g_register,
	.s_register	= sr352_s_register,
#endif
};

static struct v4l2_subdev_video_ops sr352_subdev_video_ops = {
	.s_stream = sr352_s_stream,
	.g_mbus_fmt = sr352_g_fmt,
	.s_mbus_fmt = sr352_s_fmt,
	.try_mbus_fmt = sr352_try_fmt,
	.enum_framesizes = sr352_enum_fsizes,
	.enum_mbus_fmt = sr352_enum_fmt,
	.g_crop = sr352_g_crop,
	.s_crop = sr352_s_crop,
	.g_mbus_config = sr352_g_mbus_config,
};

static struct v4l2_subdev_ops sr352_subdev_ops = {
	.core = &sr352_subdev_core_ops,
	.video = &sr352_subdev_video_ops,
};

static int sr352_detect(struct i2c_client *client)
{
	int ret;
	u8 val;

	/* revision */
	ret = sr352_read(client, 0x04, &val);
	if (ret)
		return ret;
	if (val != 0xC2)
		return -ENODEV;
	return 0;
}

static int sr352_probe(struct i2c_client *client,
				const struct i2c_device_id *did)
{
	struct sr352 *sr352;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	ret = sr352_detect(client);
	if (ret)
		return ret;
	dev_info(&adapter->dev, "cam: SR352 detect!\n");

	sr352 = kzalloc(sizeof(struct sr352), GFP_KERNEL);
	if (!sr352)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&sr352->subdev, client, &sr352_subdev_ops);
	v4l2_ctrl_handler_init(&sr352->hdl, 1);
	v4l2_ctrl_new_custom(&sr352->hdl, &sr352_ctrl_sensor_mode_cfg, NULL);
	sr352->subdev.ctrl_handler = &sr352->hdl;
	if (sr352->hdl.error)
		return sr352->hdl.error;

	sr352->rect.left = 0;
	sr352->rect.top = 0;
	sr352->rect.width = SR352_OUT_WIDTH_DEF;
	sr352->rect.height = SR352_OUT_HEIGHT_DEF;
	sr352->pixfmt = V4L2_PIX_FMT_UYVY;
	return ret;
}

static int sr352_remove(struct i2c_client *client)
{
	struct sr352 *sr352 = to_sr352(client);

	v4l2_ctrl_handler_free(&sr352->hdl);
	kfree(sr352);
	return 0;
}

static const struct i2c_device_id sr352_idtable[] = {
	{"sr352", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sr352_idtable);

static struct i2c_driver sr352_driver = {
	.driver = {
		   .name = "sr352",
		   },
	.probe = sr352_probe,
	.remove = sr352_remove,
	.id_table = sr352_idtable,
};

static int __init sr352_mod_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&sr352_driver);
	return ret;
}

static void __exit sr352_mod_exit(void)
{
	i2c_del_driver(&sr352_driver);
}

module_init(sr352_mod_init);
module_exit(sr352_mod_exit);
