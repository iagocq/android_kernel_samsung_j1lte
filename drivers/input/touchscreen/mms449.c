/*
 * MELFAS MMS400 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * This module is tested on the Google AOSP (Nexus) platforms.
 *
 * Board Type : Maguro (Google Galaxy Nexus) - Android 4.3 with Kernel 3.0
 * DeviceTree Type : Hammerhead (Google Nexus 5) - Android 5.0 with Kernel 3.4
 *
 */

#include "melfas_mms400.h"

static int mms_i2c_read_next(struct mms_ts_info *info, char *read_buf, int start_idx, unsigned int read_len);
static int mms_disable_esd_alert(struct mms_ts_info *info);
static int mms_suspend(struct device *dev);
static int mms_resume(struct device *dev);

int mms_i2c_read(struct mms_ts_info *info, char *write_buf, unsigned int write_len,
						char *read_buf, unsigned int read_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;

	struct i2c_msg msg[] = {
		{
			.addr = info->client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = write_len,
		}, {
			.addr = info->client->addr,
			.flags = I2C_M_RD,
			.buf = read_buf,
			.len = read_len,
		},
	};

	while (retry--) {
		res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));
		if (res == ARRAY_SIZE(msg))
			return 0;

		dev_err(&info->client->dev, "i2c_transter fail ret:%d, try:%d\n",
							res, I2C_RETRY_COUNT - retry);
	}

	/* i2c read fail */
	mms_reboot(info);
	return res;
}

static int mms_i2c_read_next(struct mms_ts_info *info, char *read_buf,
						int start_idx, unsigned int read_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;
	u8 rbuf[read_len];

	while (retry--) {
		res = i2c_master_recv(info->client, rbuf, read_len);
		if (res == read_len) {
			memcpy(&read_buf[start_idx], rbuf, read_len);
			return 0;
		}
		dev_err(&info->client->dev, "i2c_master_recv fail ret:%d, try:%d\n",
						res, I2C_RETRY_COUNT - retry);
	}

	/* i2c read fail */
	mms_reboot(info);
	return -1;
}

int mms_i2c_write(struct mms_ts_info *info, char *write_buf, unsigned int write_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;

	while (retry--) {
		res = i2c_master_send(info->client, write_buf, write_len);
		if (res == write_len)
			return 0;

		dev_err(&info->client->dev, "i2c_master_send fail ret:%d, try:%d\n",
				res, I2C_RETRY_COUNT - retry);
	}

	/* i2c write fail */
	mms_reboot(info);

	return -1;
}

void mms_enable(struct mms_ts_info *info)
{

	mutex_lock(&info->lock);

	if (info->enabled) {
		dev_err(&info->client->dev, "%s: device already enabled\n", __func__);
		return;
	}

	mms_set_power(info->client, true);

	enable_irq(info->client->irq);
	info->enabled = true;

	mutex_unlock(&info->lock);

	if (info->disable_esd)
		mms_disable_esd_alert(info);

	return;
}

void mms_disable(struct mms_ts_info *info)
{
	if (!info->enabled) {
		dev_err(&info->client->dev, "%s: device already disabled\n", __func__);
		return;
	}

	mutex_lock(&info->lock);

	info->enabled = false;
	disable_irq(info->client->irq);

	mms_set_power(info->client, false);

	mutex_unlock(&info->lock);

	return;
}

void mms_reboot(struct mms_ts_info *info)
{
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

	i2c_lock_adapter(adapter);

	dev_info(&info->client->dev, "%s: reboot IC power\n", __func__);
	mms_set_power(info->client, false);
	mms_set_power(info->client, true);

	i2c_unlock_adapter(adapter);
}

int mms_get_fw_version(struct mms_ts_info *info, u8 *ver_buf)
{
	u8 rbuf[8];
	u8 wbuf[2];
	int i;

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_VERSION_BOOT;

	if (mms_i2c_read(info, wbuf, 2, rbuf, 8)) {
		dev_err(&info->client->dev, "%s: i2c read fail!\n", __func__);
		return -1;
	}

	for (i = 0; i < MMS_FW_MAX_SECT_NUM; i++) {
		ver_buf[i * 2] = rbuf[i * 2 + 1];
		ver_buf[i * 2 + 1] = rbuf[i * 2];
	}

	return 0;
}

int mms_get_fw_version_u16(struct mms_ts_info *info, u16 *ver_buf_u16)
{
	u8 rbuf[8];
	int i;

	if (mms_get_fw_version(info, rbuf)) {
		dev_err(&info->client->dev, "%s: fw version read failed\n", __func__);
		return -1;
	}

	for (i = 0; i < MMS_FW_MAX_SECT_NUM; i++)
		ver_buf_u16[i] = (rbuf[i * 2] << 8) | rbuf[1 + i * 2];

	return 0;
}

void mms_clear_input(struct mms_ts_info *info)
{
	int i;

	for (i = 0; i < MAX_FINGER_NUM ; i++) {
		info->finger_state[i] = false;
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev,
					MT_TOOL_FINGER, false);
		input_report_key(info->input_dev, BTN_TOUCH, 0);
	}

	input_sync(info->input_dev);

	return;
}

static int mms_disable_esd_alert(struct mms_ts_info *info)
{
	u8 wbuf[4];
	u8 rbuf[4];

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_DISABLE_ESD_ALERT;
	wbuf[2] = 1;

	if (mms_i2c_write(info, wbuf, 3)) {
		dev_err(&info->client->dev, "%s: i2c_write fail\n", __func__);
		return -1;
	}

	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		dev_err(&info->client->dev, "%s: i2c_read fail\n", __func__);
		return -1;
	}

	if (rbuf[0] != 1) {
		dev_dbg(&info->client->dev, "%s: rbuf data error\n", __func__);
		return -1;
	}

	return 0;
}

static void mms_alert_handler_esd(struct mms_ts_info *info, u8 *rbuf)
{
	u8 frame_cnt = rbuf[2];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
	dev_dbg(&info->client->dev, "%s - frame_cnt[%d]\n", __func__, frame_cnt);

	if (!frame_cnt) {
		/* sensor crack, not ESD */
		info->esd_cnt++;
		dev_dbg(&info->client->dev, "%s - esd_cnt[%d]\n", __func__, info->esd_cnt);

		if (info->disable_esd)
			mms_disable_esd_alert(info);

		else if (info->esd_cnt > ESD_COUNT_FOR_DISABLE) {
			/* Disable ESD alert */
			if (!mms_disable_esd_alert(info))
				info->disable_esd = true;
		} else
			mms_reboot(info);
	} else {
		/* ESD detected Reset chip */
		mms_reboot(info);
		info->esd_cnt = 0;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

static void mms_input_event_handler(struct mms_ts_info *info, u8 sz, u8 *buf)
{
	struct i2c_client *client = info->client;
	int i;

	for (i = 1; i < sz; i += info->event_size) {
		u8 *tmp = &buf[i];
		int id = (tmp[0] & 0xf) - 1;
		int x = tmp[2] | ((tmp[1] & 0xf) << 8);
		int y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
		int touch_major = tmp[4];
		int pressure = tmp[5];

		if (((tmp[0] & MIP_EVENT_INPUT_SCREEN) == 0) && (info->pdata->key_size > 0)) {
			/* Touchkey Event */
			int key = tmp[0] & 0xf;
			int key_state = (tmp[0] & MIP_EVENT_INPUT_PRESS) ? 1 : 0;
			int key_code = 0;

			if (key <= info->pdata->key_size && key > 0)
				key_code = info->pdata->key_codes[key - 1];
			else {
				dev_err(&client->dev, "Unknown key code [%d]\n", key);
				continue;
			}

			input_report_key(info->input_dev, key_code, key_state);
			dev_info(&client->dev,
				"Key : ID[%d] Code[%d] State[%d]\n",
							key, key_code, key_state);

		} else {
			/* Touchscreen Event */
			if ((tmp[0] & MIP_EVENT_INPUT_PRESS) == 0) {
				/* Release */
				input_mt_slot(info->input_dev, id);
				input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
				dev_info(&client->dev, "Touch : ID[%d] Release\n", id);
				info->finger_state[id] = false;
				input_sync(info->input_dev);
				continue;
			}

			/* Press or Move */
			input_mt_slot(info->input_dev, id);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(info->input_dev, ABS_MT_PRESSURE, pressure);
			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, touch_major);

			dev_dbg(&client->dev,
				"Touch : ID[%d] X[%d] Y[%d] P[%d] M[%d]\n",
						id, x, y, pressure, touch_major);

			if (!info->finger_state[id]) {
				info->finger_state[id] = true;
				dev_info(&client->dev, "Touch : ID[%d] Press\n", id);
			}
		}
		input_sync(info->input_dev);
	}

	return;
}

/* RESET_ON_EVENT_ERROR initialized to zero */
#if RESET_ON_EVENT_ERROR
static void reset_event_error(struct mms_ts_info *info)
{
	mms_disable(info);
	mms_clear_input(info);
	mms_enable(info);
}
#else
static void reset_event_error(struct mms_ts_info *info) {}
#endif

static irqreturn_t mms_interrupt(int irq, void *dev_id)
{
	struct mms_ts_info *info = dev_id;
	struct i2c_client *client = info->client;
	u8 wbuf[8];
	u8 rbuf[256];
	unsigned int size;
	int event_size = info->event_size;
	u8 category;
	u8 alert_type;


	/* Read first packet */
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
	if (mms_i2c_read(info, wbuf, 2, rbuf, (1 + event_size))) {
		dev_err(&client->dev, "read packet info error\n");
		goto ERROR;
	}

	/* Check event */
	size = (rbuf[0] & 0x7F);

	category = ((rbuf[0] >> 7) & 0x1);
	if (!category) {
		/* Touch event */
		if (size > event_size) {
			/* Read next packet */
			if (mms_i2c_read_next(info, rbuf, (event_size + 1), (size - event_size))) {
				dev_err(&client->dev, "read next packet error\n");
				goto ERROR;
			}
		}

		info->esd_cnt = 0;
		mms_input_event_handler(info, size, rbuf);
	} else {
		/* Alert event */
		alert_type = rbuf[1];
		dev_dbg(&client->dev, "%s - alert type [%d]\n", __func__, alert_type);

		if (alert_type == MIP_ALERT_ESD) {
			/* ESD detection */
			mms_alert_handler_esd(info, rbuf);
		}  else {
			dev_err(&client->dev, "Unknown alert type [%d]\n", alert_type);
			goto ERROR;
		}
	}

	return IRQ_HANDLED;

ERROR:
	reset_event_error(info);
	return IRQ_HANDLED;
}

int mms_fw_update_from_kernel(struct mms_ts_info *info)
{
	const char *fw_name = INTERNAL_FW_PATH;
	const struct firmware *fw;
	int retries = 3;
	int ret = -1;

	request_firmware(&fw, fw_name, &info->client->dev);

	if (!fw) {
		dev_err(&info->client->dev, "request_firmware failed!\n");
		return -1;
	}

	do {
		ret = mms_flash_fw(info, fw->data, fw->size, false, true);
		if (!ret)
			break;
	} while (--retries);

	if (!retries) {
		dev_err(&info->client->dev, "failed to flash built-in firmware\n");
		goto END;
	}

END:
	release_firmware(fw);
	return ret;
}

int mms_fw_update_from_storage(struct mms_ts_info *info, bool force)
{
	struct file *fp;
	mm_segment_t old_fs;
	size_t fw_size, nread;
	int ret;
	unsigned char *fw_data;

	mutex_lock(&info->lock);
	disable_irq(info->client->irq);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(EXTERNAL_FW_PATH, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		dev_err(&info->client->dev, "%s: failed to open fw file\n", __func__);
		ret = -ENOENT;
		goto ERROR;
	}

	fw_size = fp->f_path.dentry->d_inode->i_size;
	if (fw_size == 0) {
		dev_err(&info->client->dev, "fw_size error!\n");
		ret = -EIO;
		goto ERROR1;
	}

	fw_data = kzalloc(fw_size, GFP_KERNEL);
	if (!fw_data) {
		dev_err(&info->client->dev, "failed to allocate fw_data!\n");
		ret = -ENOMEM;
		goto ERROR1;
	}

	nread = vfs_read(fp, (char __user *)fw_data, fw_size, &fp->f_pos);
	dev_dbg(&info->client->dev, "%s - path [%s] size [%zu]\n",
					__func__, EXTERNAL_FW_PATH, fw_size);

	if (nread != fw_size) {
		dev_err(&info->client->dev, "vfs_read error - size[%zu] read[%zu]\n",
									fw_size, nread);
		ret = -EIO;
	} else
		ret = mms_flash_fw(info, fw_data, fw_size, force, true);

	if (!ret)
		dev_info(&info->client->dev, "succeeded update external firmware\n");
	kfree(fw_data);
ERROR1:
	filp_close(fp, current->files);
ERROR:
	set_fs(old_fs);
	enable_irq(info->client->irq);
	mutex_unlock(&info->lock);
	if (ret)
		dev_err(&info->client->dev, "failed to update external firmware\n");

	return ret;
}

#define MMS_EVENT_SIZE	6

static void mms_init_config(struct mms_ts_info *info)
{
	u8 wbuf[8];
	u8 rbuf[64];

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;

	if (mms_i2c_read(info, wbuf, 2, rbuf, 16)) {
		dev_err(&info->client->dev, "%s: i2c read fail!\n", __func__);
		return;
	}

	memcpy(info->product_name, rbuf, 16);

	if (mms_get_fw_version(info, rbuf)) {
		dev_err(&info->client->dev, "%s: failed to get fw version!\n", __func__);
		return;
	}
	memcpy(info->fw_version, rbuf, 8);
	dev_info(&info->client->dev, "%s - F/W Version : %02X.%02X %02X.%02X %02X.%02X %02X.%02X\n",
					__func__, info->fw_version[0], info->fw_version[1],
					info->fw_version[2], info->fw_version[3], info->fw_version[4],
					info->fw_version[5], info->fw_version[6], info->fw_version[7]);

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_RESOLUTION_X;
	if (mms_i2c_read(info, wbuf, 2, rbuf, 7)) {
		dev_err(&info->client->dev, "%s: i2c read fail!\n", __func__);
		return;
	}

	info->max_x = (rbuf[0]) | (rbuf[1] << 8);
	info->max_y = (rbuf[2]) | (rbuf[3] << 8);

	info->node_x = rbuf[4];
	info->node_y = rbuf[5];
	info->node_key = rbuf[6];

	if (info->node_key > 0)
		info->tkey_enable = true;

	info->event_size = MMS_EVENT_SIZE;
}

void mms_set_power(struct i2c_client *client, bool enable)
{
	int ret;
	static struct regulator *tsp_vdd;

	tsp_vdd = regulator_get(&client->dev, "tsp_vdd");

	if (IS_ERR(tsp_vdd)) {
		dev_err(&client->dev, "regulator_get error!\n");
		return;
	}

	if (enable) {
		if (!regulator_is_enabled(tsp_vdd)) {
			regulator_set_voltage(tsp_vdd, 3300000, 3300000);
			ret = regulator_enable(tsp_vdd);
			if (ret) {
				dev_err(&client->dev, "power on error!!\n");
				return;
			}
			dev_info(&client->dev, "power on!\n");
		} else
			dev_err(&client->dev, "already power on!\n");
		mdelay(50);
	} else {
		if (regulator_is_enabled(tsp_vdd)) {
			ret = regulator_disable(tsp_vdd);
			if (ret) {
				dev_err(&client->dev, "power off error!\n");
				return;
			}
			dev_info(&client->dev, "power off!\n");
			mdelay(10);
		} else
			dev_err(&client->dev, "already power off!\n");
	}

	regulator_put(tsp_vdd);
}


#if defined(CONFIG_OF)
static int mms_parse_devicetree(struct device *dev, struct mms_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 val;
	int ret;
	struct property *prop;

	ret = of_property_read_u32(np, MMS_DEVICE_NAME",max_x", &val);
	if (ret) {
		dev_err(dev, "%s: failed to read max_x\n", __func__);
		pdata->max_x = 1024;
	} else
		pdata->max_x = val;

	ret = of_property_read_u32(np, MMS_DEVICE_NAME",max_y", &val);
	if (ret) {
		dev_err(dev, "%s: failed to read max_y\n", __func__);
		pdata->max_y = 600;
	} else
		pdata->max_y = val;

	ret = of_property_read_u32(np, MMS_DEVICE_NAME",x_num", &val);
	if (ret) {
		dev_err(dev, "%s: failed to read x channel no.\n", __func__);
		pdata->x_num = 0;
	} else
		pdata->x_num = val;

	ret = of_property_read_u32(np, MMS_DEVICE_NAME",y_num", &val);
	if (ret) {
		dev_err(dev, "%s: failed to read y channel no.\n", __func__);
		pdata->y_num = 0;
	} else
		pdata->y_num = val;

	prop = of_find_property(np, MMS_DEVICE_NAME",tsp-keycodes", NULL);
	if (prop && prop->value) {
		pdata->key_size = prop->length / sizeof(u32);
		ret = of_property_read_u32_array(np, MMS_DEVICE_NAME",tsp-keycodes",
							pdata->key_codes, pdata->key_size);
		if (ret) {
			dev_err(dev, "%s: failed to read key_code\n", __func__);
			return ret;
		}
	}

	pdata->gpio_intr = of_get_named_gpio(np, "irq-gpio", 0);
	if (pdata->gpio_intr < 0) {
		dev_err(dev, "%s: failed to get irq number\n", __func__);
		return -EINVAL;
	}

	ret = gpio_request(pdata->gpio_intr, "mms_ts_int");
	if (ret < 0) {
		dev_err(dev, "%s: gpio_request fail!\n", __func__);
		return -EINVAL;
	}
	gpio_direction_input(pdata->gpio_intr);

	return 0;
}
#else
static int mms_parse_devicetree(struct device *dev, struct mms_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int __init mms_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mms_ts_info *info;
	struct input_dev *input_dev;
	struct mms_platform_data *pdata;
	int ret = 0;
	int i;

	dev_dbg(&client->dev, "%s [START]\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s [ERROR] i2c_check_functionality\n", __func__);
		return -EIO;
	}

	if (IS_ENABLED(CONFIG_OF)) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct mms_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "failed to allocate pdata\n");
			return -ENOMEM;
		}

		ret = mms_parse_devicetree(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "failed to parse_dt\n");
			return ret;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "no platform data\n");
			return -EINVAL;
		}
	}

	info = kzalloc(sizeof(struct mms_ts_info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "failed to allocate info data!\n");
		return -ENOMEM;
	}

	info->pdata = pdata;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_input_alloc;
	}

	info->client = client;
	info->input_dev = input_dev;
	mutex_init(&info->lock);

	snprintf(info->phys, sizeof(info->phys), "%s/input0", dev_name(&client->dev));

	input_dev->name = "sec_touchscreen";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	/* Create device */
	input_set_drvdata(input_dev, info);
	i2c_set_clientdata(client, info);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register input device\n");
		ret = -EIO;
		goto err_input_register_device;
	}


	/* Power on */
	mms_set_power(info->client, true);

	/* Firmware update */
	ret = mms_fw_update_from_kernel(info);
	if (ret) {
		dev_err(&client->dev, "failed to firmware update\n");
		goto err_fw_update;
	}

	/* Initial config */
	mms_init_config(info);

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_LED, input_dev->evbit);
	for (i  = 0; i < info->pdata->key_size ; i++)
		set_bit(info->pdata->key_codes[i], input_dev->keybit);

	input_mt_init_slots(input_dev, MAX_FINGER_NUM, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
					0, info->pdata->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
					0, info->pdata->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
					0, INPUT_PRESSURE_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
					0, INPUT_TOUCH_MAJOR_MAX, 0, 0);

	client->irq = gpio_to_irq(info->pdata->gpio_intr);
	if (client->irq < 0) {
		dev_err(&client->dev, "failed to get gpio_to_irq\n");
		goto err_gpio_irq;
	}
	ret = request_threaded_irq(client->irq, NULL, mms_interrupt,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					MMS_DEVICE_NAME, info);
	if (ret) {
		dev_err(&client->dev, "failed to register irq hander!\n");
		goto err_req_irq;
	}

	disable_irq(client->irq);
	info->irq = client->irq;

#ifdef CONFIG_HAS_EARLYSUSPEND
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	info->early_suspend.suspend = mms_early_suspend;
	info->early_suspend.resume = mms_late_resume;
	register_early_suspend(&info->early_suspend);
#endif

	/* Enable device */
	mms_enable(info);

#if MMS_USE_DEV_MODE
	if (mms_dev_create(info)) {
		dev_err(&client->dev, "%s [ERROR] mms_dev_create\n", __func__);
		ret = -EAGAIN;
		goto err_dev_mode;
	}

	info->class = class_create(THIS_MODULE, MMS_DEVICE_NAME);
	device_create(info->class, NULL, info->mms_dev, NULL, MMS_DEVICE_NAME);
#endif

#if MMS_USE_TEST_MODE
	if (mms_sysfs_create(info)) {
		dev_err(&client->dev, "%s [ERROR] mms_sysfs_create\n", __func__);
		ret = -EAGAIN;
		goto err_test_mode;
	}
#endif

#if MMS_USE_CMD_MODE
	if (mms_sysfs_cmd_create(info)) {
		dev_err(&client->dev, "%s [ERROR] mms_sysfs_cmd_create\n", __func__);
		ret = -EAGAIN;
		goto err_cmd_mode;
	}
#endif

	dev_info(&client->dev, "MELFAS " CHIP_NAME " Touchscreen probe done!\n");
	return 0;

/*err_factory_init:*/
#if MMS_USE_CMD_MODE
err_cmd_mode:
#endif
#if MMS_USE_TEST_MODE
err_test_mode:
#endif
#if MMS_USE_DEV_MODE
err_dev_mode:
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&info->early_suspend);
#endif
	free_irq(client->irq, info);
err_req_irq:
err_gpio_irq:
err_fw_update:
	input_unregister_device(input_dev);
err_input_register_device:
	input_free_device(input_dev);
	input_dev = NULL;
err_input_alloc:
	kfree(info);

	dev_err(&client->dev, "MELFAS " CHIP_NAME " Touchscreen initialization failed.\n");
	return ret;
}

static int mms_remove(struct i2c_client *client)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);

	if (info->irq >= 0)
		free_irq(info->irq, info);

#if MMS_USE_CMD_MODE
	mms_sysfs_cmd_remove(info);
#endif

#if MMS_USE_TEST_MODE
	mms_sysfs_remove(info);
#endif

#if MMS_USE_DEV_MODE
	device_destroy(info->class, info->mms_dev);
	class_destroy(info->class);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&info->early_suspend);
#endif

	input_unregister_device(info->input_dev);

	kfree(info->fw_name);
	kfree(info);

	return 0;
}

#if defined(CONFIG_PM)
static int mms_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);

	mms_disable(info);
	mms_clear_input(info);

	return 0;
}

static int mms_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);

	if (!info->enabled)
		mms_enable(info);

	return 0;
}
#else
#define mms_suspend	NULL
#define mms_resume	NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mms_early_suspend(struct early_suspend *h)
{
	struct mms_ts_info *info = container_of(h, struct mms_ts_info, early_suspend);

	mms_suspend(&info->client->dev);
}

static void mms_late_resume(struct early_suspend *h)
{
	struct mms_ts_info *info = container_of(h, struct mms_ts_info, early_suspend);

	mms_resume(&info->client->dev);
}
#endif

const struct dev_pm_ops mms_pm_ops = {
	.suspend = mms_suspend,
	.resume = mms_resume,
};


#if defined(CONFIG_OF)
static const struct of_device_id mms_match_table[] = {
	{ .compatible = "melfas,"MMS_DEVICE_NAME,},
	{},
};
MODULE_DEVICE_TABLE(of, mms_match_table);
#else
#define mms_match_table	NULL
#endif

static const struct i2c_device_id mms_id[] = {
	{MMS_DEVICE_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, mms_id);

static struct i2c_driver mms_driver = {
	.id_table = mms_id,
	.probe = mms_probe,
	.remove = mms_remove,
	.driver = {
		.name = MMS_DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mms_match_table,
#ifndef CONFIG_HAS_EARLYSUSPEND
		.pm = &mms_pm_ops,
#endif
	},
};

static int __init mms_init(void)
{
	return i2c_add_driver(&mms_driver);
}

static void __exit mms_exit(void)
{
	i2c_del_driver(&mms_driver);
}

module_init(mms_init);
module_exit(mms_exit);

MODULE_DESCRIPTION("MELFAS MMS400 Touchscreen");
MODULE_VERSION("2014.11.27");
MODULE_AUTHOR("Jee SangWon <jeesw@melfas.com>");
MODULE_LICENSE("GPL");
