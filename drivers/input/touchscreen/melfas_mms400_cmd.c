/*
 * MELFAS MMS400 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * Command Functions (Optional)
 *
 */

#include "melfas_mms400.h"

#if MMS_USE_CMD_MODE

#define NAME_OF_UNKNOWN_CMD "not_support_cmd"

enum CMD_STATUS {
	CMD_STATUS_WAITING = 0,
	CMD_STATUS_RUNNING,
	CMD_STATUS_OK,
	CMD_STATUS_FAIL,
	CMD_STATUS_NONE,
};

enum {
	NORMAL = 0,
	EXT_FILE,
};

static void cmd_clear_result(struct mms_ts_info *info)
{
	char delim = ':';
	memset(info->cmd_result, 0x00, ARRAY_SIZE(info->cmd_result));
	memcpy(info->cmd_result, info->cmd, strnlen(info->cmd, CMD_LEN));
	strncat(info->cmd_result, &delim, 1);
}

static void cmd_set_result(struct mms_ts_info *info, char *buf, int len)
{
	strncat(info->cmd_result, buf, len);
}

static void cmd_fw_update(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	cmd_clear_result(info);

	switch (info->cmd_param[0]) {
	case NORMAL:
		if (!mms_fw_update_from_kernel(info))
			info->cmd_state = CMD_STATUS_OK;
		else
			info->cmd_state = CMD_STATUS_FAIL;
		break;
	case EXT_FILE:
		if (!mms_fw_update_from_storage(info, true))
			info->cmd_state = CMD_STATUS_OK;
		else
			info->cmd_state = CMD_STATUS_FAIL;
		break;
	default:
		info->cmd_state = CMD_STATUS_FAIL;
		break;
	}

	if (info->cmd_state == CMD_STATUS_OK)
		sprintf(buf, "%s", "OK");
	else
		sprintf(buf, "%s", "FAIL");

	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

/* FIXME: firmware V03 touch threshold */
#define TOUCH_THRESHOLD	25

static void cmd_get_threshold(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	cmd_clear_result(info);

	sprintf(buf, "%d\n", TOUCH_THRESHOLD);
	info->cmd_state = CMD_STATUS_OK;

	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));

}

static void cmd_get_fw_ver_bin(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	const char *fw_name = INTERNAL_FW_PATH;
	const struct firmware *fw;
	struct mms_bin_hdr *fw_hdr;
	struct mms_fw_img **img;
	u8 ver_file[MMS_FW_MAX_SECT_NUM * 2];
	int i;
	int offset = sizeof(struct mms_bin_hdr);

	cmd_clear_result(info);

	request_firmware(&fw, fw_name, &info->client->dev);

	if (!fw) {
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		goto EXIT;
	}

	fw_hdr = (struct mms_bin_hdr *)fw->data;
	img = kzalloc(sizeof(*img) * fw_hdr->section_num, GFP_KERNEL);
	if (!img) {
		dev_err(&info->client->dev, "%s: failed to alloc img!\n", __func__);
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		goto EXIT;
	}

	for (i = 0; i < fw_hdr->section_num; i++, offset += sizeof(struct mms_fw_img)) {
		img[i] = (struct mms_fw_img *)(fw->data + offset);
		ver_file[i * 2] = ((img[i]->version) >> 8) & 0xFF;
		ver_file[i * 2 + 1] = (img[i]->version) & 0xFF;
	}

	sprintf(buf, "ME%02X%02X%02X\n", ver_file[0], ver_file[3], ver_file[5]);
	info->cmd_state = CMD_STATUS_OK;

EXIT:
	release_firmware(fw);
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_get_fw_ver_ic(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };
	u8 rbuf[64];

	cmd_clear_result(info);

	if (mms_get_fw_version(info, rbuf)) {
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
	} else {
		sprintf(buf, "ME%02X%02X%02X\n", rbuf[0], rbuf[3], rbuf[5]);
		info->cmd_state = CMD_STATUS_OK;
	}

	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_get_chip_vendor(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	cmd_clear_result(info);

	sprintf(buf, "MELFAS");
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));

	info->cmd_state = CMD_STATUS_OK;
}

static void cmd_get_chip_name(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	cmd_clear_result(info);

	sprintf(buf, CHIP_NAME);
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));

	info->cmd_state = CMD_STATUS_OK;
}

static void cmd_get_x_num(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	cmd_clear_result(info);

	sprintf(buf, "%d", info->pdata->x_num);
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));

	info->cmd_state = CMD_STATUS_OK;
}

static void cmd_get_y_num(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	cmd_clear_result(info);

	sprintf(buf, "%d", info->pdata->y_num);
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));

	info->cmd_state = CMD_STATUS_OK;
}

static void cmd_get_max_x(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	cmd_clear_result(info);

	sprintf(buf, "%d", info->pdata->max_x);
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));

	info->cmd_state = CMD_STATUS_OK;
}

static void cmd_get_max_y(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	cmd_clear_result(info);

	sprintf(buf, "%d", info->pdata->max_y);
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));

	info->cmd_state = CMD_STATUS_OK;
}

static void cmd_module_off_master(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	cmd_clear_result(info);

	mms_set_power(info->client, false);

	sprintf(buf, "%s", "OK");
	info->cmd_state = CMD_STATUS_OK;

	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_module_on_master(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	cmd_clear_result(info);

	mms_set_power(info->client, true);

	sprintf(buf, "%s", "OK");
	info->cmd_state = CMD_STATUS_OK;

	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

#define MAX_VALUE	-999999
#define MIN_VALUE	999999

static void cmd_read_intensity(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };
	int i;
	int max = MAX_VALUE;
	int min = MIN_VALUE;

	cmd_clear_result(info);

	if (mms_get_image(info, MIP_IMG_TYPE_INTENSITY)) {
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		goto EXIT;
	}

	for (i = 0; i < (info->node_x * info->node_y); i++) {
		if (info->image_buf[i] > max)
			max = info->image_buf[i];
		if (info->image_buf[i] < min)
			min = info->image_buf[i];
	}

	sprintf(buf, "%d,%d", min, max);
	info->cmd_state = CMD_STATUS_OK;

EXIT:
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_get_intensity(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	int x = info->cmd_param[0];
	int y = info->cmd_param[1];
	int idx = 0;

	cmd_clear_result(info);

	if ((x < 0) || (x >= info->node_x) || (y < 0) || (y >= info->node_y)) {
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		goto EXIT;
	}

	idx = x + info->node_y * y;

	sprintf(buf, "%d", info->image_buf[idx]);
	info->cmd_state = CMD_STATUS_OK;

EXIT:
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_read_rawdata(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };
	int i;
	int min = MIN_VALUE;
	int max = MAX_VALUE;

	cmd_clear_result(info);

	if (mms_get_image(info, MIP_IMG_TYPE_RAWDATA)) {
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		goto EXIT;
	}

	for (i = 0; i < (info->node_x * info->node_y); i++) {
		if (info->image_buf[i] > max)
			max = info->image_buf[i];
		if (info->image_buf[i] < min)
			min = info->image_buf[i];
	}

	sprintf(buf, "%d,%d", min, max);
	info->cmd_state = CMD_STATUS_OK;

EXIT:
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_get_rawdata(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	int x = info->cmd_param[0];
	int y = info->cmd_param[1];
	int idx;

	cmd_clear_result(info);

	if ((x < 0) || (x >= info->node_x) || (y < 0) || (y >= info->node_y)) {
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		goto EXIT;
	}

	idx = x + y * info->node_y;

	sprintf(buf, "%d", info->image_buf[idx]);
	info->cmd_state = CMD_STATUS_OK;

EXIT:
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_run_test_cm_delta(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };
	int i;
	int min = MIN_VALUE;
	int max = MAX_VALUE;

	cmd_clear_result(info);

	if (mms_run_test(info, MIP_TEST_TYPE_CM_DELTA)) {
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		goto EXIT;
	}

	for (i = 0; i < (info->node_x * info->node_y); i++) {
		if (info->image_buf[i] > max)
			max = info->image_buf[i];
		if (info->image_buf[i] < min)
			min = info->image_buf[i];
	}

	sprintf(buf, "%d,%d", min, max);
	info->cmd_state = CMD_STATUS_OK;

EXIT:
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_get_cm_delta(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	int x = info->cmd_param[0];
	int y = info->cmd_param[1];
	int idx;

	cmd_clear_result(info);

	if ((x < 0) || (x >= info->node_x) || (y < 0) || (y >= info->node_y)) {
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		goto EXIT;
	}

	idx = x + y * info->node_y;

	sprintf(buf, "%d", info->image_buf[idx]);
	info->cmd_state = CMD_STATUS_OK;

EXIT:
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_run_test_cm_abs(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };
	int min = MIN_VALUE;
	int max = MAX_VALUE;
	int i;

	cmd_clear_result(info);

	if (mms_run_test(info, MIP_TEST_TYPE_CM_ABS)) {
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		goto EXIT;
	}

	for (i = 0; i < (info->node_x * info->node_y); i++) {
		if (info->image_buf[i] > max)
			max = info->image_buf[i];
		if (info->image_buf[i] < min)
			min = info->image_buf[i];
	}

	sprintf(buf, "%d,%d", min, max);
	info->cmd_state = CMD_STATUS_OK;

EXIT:
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_get_cm_abs(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };

	int x = info->cmd_param[0];
	int y = info->cmd_param[1];
	int idx;

	cmd_clear_result(info);

	if ((x < 0) || (x >= info->node_x) || (y < 0) || (y >= info->node_y)) {
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		goto EXIT;
	}

	idx = x + y * info->node_y;

	sprintf(buf, "%d", info->image_buf[idx]);
	info->cmd_state = CMD_STATUS_OK;

EXIT:
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_run_test_cm_jitter(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };
	int min = MIN_VALUE;
	int max = MAX_VALUE;
	int i;

	cmd_clear_result(info);

	if (mms_run_test(info, MIP_TEST_TYPE_CM_JITTER)) {
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		goto EXIT;
	}

	for (i = 0; i < (info->node_x * info->node_y); i++) {
		if (info->image_buf[i] > max)
			max = info->image_buf[i];
		if (info->image_buf[i] < min)
			min = info->image_buf[i];
	}

	sprintf(buf, "%d,%d", min, max);
	info->cmd_state = CMD_STATUS_OK;

EXIT:
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_get_cm_jitter(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[64] = { 0 };
	int x = info->cmd_param[0];
	int y = info->cmd_param[1];
	int idx;

	cmd_clear_result(info);

	if ((x < 0) || (x >= info->node_x) || (y < 0) || (y >= info->node_y)) {
		sprintf(buf, "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		goto EXIT;
	}

	idx = x + y * info->node_y;

	sprintf(buf, "%d", info->image_buf[idx]);
	info->cmd_state = CMD_STATUS_OK;

EXIT:
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));
}

static void cmd_unknown_cmd(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buf[16] = { 0 };

	cmd_clear_result(info);

	snprintf(buf, sizeof(buf), "%s", NAME_OF_UNKNOWN_CMD);
	cmd_set_result(info, buf, strnlen(buf, sizeof(buf)));

	info->cmd_state = CMD_STATUS_NONE;
}

#define MMS_CMD(name, func)	.cmd_name = name, .cmd_func = func

struct mms_cmd {
	struct list_head list;
	const char *cmd_name;
	void (*cmd_func) (void *device_data);
};

static struct mms_cmd mms_commands[] = {
	{MMS_CMD("fw_update", cmd_fw_update),},
	{MMS_CMD("get_fw_ver_bin", cmd_get_fw_ver_bin),},
	{MMS_CMD("get_fw_ver_ic", cmd_get_fw_ver_ic),},
	{MMS_CMD("get_chip_vendor", cmd_get_chip_vendor),},
	{MMS_CMD("get_chip_name", cmd_get_chip_name),},
	{MMS_CMD("get_x_num", cmd_get_x_num),},
	{MMS_CMD("get_y_num", cmd_get_y_num),},
	{MMS_CMD("get_max_x", cmd_get_max_x),},
	{MMS_CMD("get_max_y", cmd_get_max_y),},
	{MMS_CMD("get_threshold", cmd_get_threshold),},
	{MMS_CMD("module_off_master", cmd_module_off_master),},
	{MMS_CMD("module_on_master", cmd_module_on_master),},
	{MMS_CMD("run_intensity_read", cmd_read_intensity),},
	{MMS_CMD("get_intensity", cmd_get_intensity),},
	{MMS_CMD("run_rawdata_read", cmd_read_rawdata),},
	{MMS_CMD("get_rawdata", cmd_get_rawdata),},
	{MMS_CMD("run_inspection_read", cmd_run_test_cm_delta),},
	{MMS_CMD("get_inspection", cmd_get_cm_delta),},
	{MMS_CMD("run_cm_delta_read", cmd_run_test_cm_delta),},
	{MMS_CMD("get_cm_delta", cmd_get_cm_delta),},
	{MMS_CMD("run_cm_abs_read", cmd_run_test_cm_abs),},
	{MMS_CMD("get_cm_abs", cmd_get_cm_abs),},
	{MMS_CMD("run_cm_jitter_read", cmd_run_test_cm_jitter),},
	{MMS_CMD("get_cm_jitter", cmd_get_cm_jitter),},

	{MMS_CMD("get_config_ver", cmd_unknown_cmd),},
	{MMS_CMD("module_off_slave", cmd_unknown_cmd),},
	{MMS_CMD("module_on_slave", cmd_unknown_cmd),},
	{MMS_CMD(NAME_OF_UNKNOWN_CMD, cmd_unknown_cmd),},
};

static ssize_t mms_sys_cmd(struct device *dev, struct device_attribute *devattr,
							const char *buf, size_t count)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	char *cur, *start, *end;
	char cbuf[CMD_LEN] = { 0 };
	int len, i;
	struct mms_cmd *mms_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;

	if (!info) {
		dev_err(&info->client->dev, "%s [ERROR] mms_ts_info not found\n", __func__);
		goto ERROR;
	}

	if (!info->input_dev) {
		dev_err(&info->client->dev, "%s [ERROR] input_dev not found\n", __func__);
		goto ERROR;
	}

	mutex_lock(&info->lock);
	if (info->cmd_busy == true) {
		dev_err(&info->client->dev, "%s [ERROR] previous command is not ended\n", __func__);
		mutex_unlock(&info->lock);
		goto ERROR;
	}

	info->cmd_busy = true;
	mutex_unlock(&info->lock);

	info->cmd_state = 1;
	for (i = 0; i < ARRAY_SIZE(info->cmd_param); i++)
		info->cmd_param[i] = 0;

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;

	memset(info->cmd, 0x00, ARRAY_SIZE(info->cmd));
	memcpy(info->cmd, buf, len);
	cur = strchr(buf, (int)delim);
	if (cur)
		memcpy(cbuf, buf, cur - buf);
	else
		memcpy(cbuf, buf, len);

	dev_dbg(&info->client->dev, "%s - command [%s]\n", __func__, cbuf);

	list_for_each_entry(mms_cmd_ptr, &info->cmd_list_head, list) {
		if (!strncmp(cbuf, mms_cmd_ptr->cmd_name, CMD_LEN)) {
			cmd_found = true;
			break;
		}
	}

	if (!cmd_found) {
		list_for_each_entry(mms_cmd_ptr, &info->cmd_list_head, list) {
			if (!strncmp(NAME_OF_UNKNOWN_CMD, mms_cmd_ptr->cmd_name, CMD_LEN))
				break;
		}
	}

	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(cbuf, 0x00, ARRAY_SIZE(cbuf));

		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(cbuf, start, end - start);
				*(cbuf + strnlen(cbuf, ARRAY_SIZE(cbuf))) = '\0';
				if (kstrtoint(cbuf, 10, info->cmd_param + param_cnt) < 0)
					goto ERROR;
				start = cur + 1;
				memset(cbuf, 0x00, ARRAY_SIZE(cbuf));
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}

	mms_cmd_ptr->cmd_func(info);
ERROR:
	return count;
}
static DEVICE_ATTR(cmd, S_IWUSR | S_IWGRP, NULL, mms_sys_cmd);

static ssize_t mms_sys_cmd_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	char cbuf[32] = {0};


	if (info->cmd_state == CMD_STATUS_WAITING)
		snprintf(cbuf, sizeof(cbuf), "WAITING");
	else if (info->cmd_state == CMD_STATUS_RUNNING)
		snprintf(cbuf, sizeof(cbuf), "RUNNING");
	else if (info->cmd_state == CMD_STATUS_OK)
		snprintf(cbuf, sizeof(cbuf), "OK");
	else if (info->cmd_state == CMD_STATUS_FAIL)
		snprintf(cbuf, sizeof(cbuf), "FAIL");
	else if (info->cmd_state == CMD_STATUS_NONE)
		snprintf(cbuf, sizeof(cbuf), "NOT_APPLICABLE");

	return snprintf(buf, PAGE_SIZE, "%s\n", cbuf);
}
static DEVICE_ATTR(cmd_status, S_IRUGO, mms_sys_cmd_status, NULL);

static ssize_t mms_sys_cmd_result(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);

	mutex_lock(&info->lock);
	info->cmd_busy = false;
	mutex_unlock(&info->lock);

	info->cmd_state = CMD_STATUS_WAITING;

	return snprintf(buf, PAGE_SIZE, "%s\n", info->cmd_result);
}
static DEVICE_ATTR(cmd_result, S_IRUGO, mms_sys_cmd_result, NULL);

static ssize_t mms_sys_cmd_list(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	int i = 0;
	char buffer[info->cmd_buffer_size];
	char buffer_name[CMD_LEN];

	snprintf(buffer, 30, "== Command list ==\n");
	while (strncmp(mms_commands[i].cmd_name, NAME_OF_UNKNOWN_CMD, CMD_LEN) != 0) {
		snprintf(buffer_name, CMD_LEN, "%s\n", mms_commands[i].cmd_name);
		strcat(buffer, buffer_name);
		i++;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", buffer);
}
static DEVICE_ATTR(cmd_list, S_IRUGO, mms_sys_cmd_list, NULL);

static struct attribute *mms_cmd_attr[] = {
	&dev_attr_cmd.attr,
	&dev_attr_cmd_status.attr,
	&dev_attr_cmd_result.attr,
	&dev_attr_cmd_list.attr,
	NULL,
};

static const struct attribute_group mms_cmd_attr_group = {
	.attrs = mms_cmd_attr,
};

static ssize_t mms_sys_intensity_key(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	int key_offset = info->node_x * info->node_y;
	int key_idx = -1;

	if (!strcmp(attr->attr.name, "touchkey_dummy_btn1"))
		key_idx = 0;
	else if (!strcmp(attr->attr.name, "touchkey_recent"))
		key_idx = 1;
	else if (!strcmp(attr->attr.name, "touchkey_back"))
		key_idx = 2;
	else if (!strcmp(attr->attr.name, "touchkey_dummy_btn3"))
		key_idx = 3;
	else {
		dev_err(&info->client->dev, "%s [ERROR] Invalid attribute\n", __func__);
		goto ERROR;
	}

	if (mms_get_image(info, MIP_IMG_TYPE_INTENSITY)) {
		dev_err(&info->client->dev, "%s [ERROR] mms_get_image\n", __func__);
		return snprintf(buf, PAGE_SIZE, "%d", key_idx);
	}

	dev_dbg(&info->client->dev, "%s - %s [%d]\n", __func__,
			attr->attr.name, info->image_buf[key_offset + key_idx]);

	return snprintf(buf, PAGE_SIZE, "%d", info->image_buf[key_offset + key_idx]);

ERROR:
	return snprintf(buf, PAGE_SIZE, "NG");
}
static DEVICE_ATTR(touchkey_recent, S_IRUGO, mms_sys_intensity_key, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO, mms_sys_intensity_key, NULL);
static DEVICE_ATTR(touchkey_dummy_btn1, S_IRUGO, mms_sys_intensity_key, NULL);
static DEVICE_ATTR(touchkey_dummy_btn3, S_IRUGO, mms_sys_intensity_key, NULL);

/* FIXME: firmware V03 touch key threshold */
#define TOUCH_KEY_THRESHOLD	35

static ssize_t mms_sys_threshold_key(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);

	dev_dbg(&info->client->dev, "%s - threshold [%d]\n", __func__, TOUCH_KEY_THRESHOLD);

	return snprintf(buf, PAGE_SIZE, "%d", TOUCH_KEY_THRESHOLD);

}
static DEVICE_ATTR(touchkey_threshold, S_IRUGO, mms_sys_threshold_key, NULL);

static struct attribute *mms_key_attr[] = {
	&dev_attr_touchkey_recent.attr,
	&dev_attr_touchkey_back.attr,
	&dev_attr_touchkey_dummy_btn1.attr,
	&dev_attr_touchkey_dummy_btn3.attr,
	&dev_attr_touchkey_threshold.attr,
	NULL,
};

static const struct attribute_group mms_key_attr_group = {
	.attrs = mms_key_attr,
};

int mms_sysfs_cmd_create(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;
	int i;

	INIT_LIST_HEAD(&info->cmd_list_head);
	info->cmd_buffer_size = 0;

	for (i = 0; i < ARRAY_SIZE(mms_commands); i++) {
		list_add_tail(&mms_commands[i].list, &info->cmd_list_head);
		if (mms_commands[i].cmd_name)
			info->cmd_buffer_size += strlen(mms_commands[i].cmd_name) + 1;
	}

	info->cmd_busy = false;

	if (!info->print_buf) {
		info->print_buf = kzalloc(sizeof(u8) * 4096, GFP_KERNEL);
		if (!info->print_buf) {
			dev_err(&client->dev, "failed to alloc print_buf\n");
			return -ENOMEM;
		}
	}
	if (!info->image_buf) {
		info->image_buf = kzalloc(sizeof(int) * ((info->node_x * info->node_y) +
								info->node_key), GFP_KERNEL);
		if (!info->image_buf) {
			dev_err(&client->dev, "failed to alloc image_buf\n");
			return -ENOMEM;
		}
	}

	if (sysfs_create_group(&client->dev.kobj, &mms_cmd_attr_group)) {
		dev_err(&client->dev, "failed to create cmd_attr_group\n");
		return -EAGAIN;
	}

	info->cmd_dev = device_create(sec_class, NULL, 0, info, "tsp");
	if (sysfs_create_group(&info->cmd_dev->kobj, &mms_cmd_attr_group)) {
		dev_err(&client->dev, "failed to sysfs_create_group\n");
		return -EAGAIN;
	}

	if (info->pdata->key_size > 0) {
		if (sysfs_create_group(&client->dev.kobj, &mms_key_attr_group)) {
			dev_err(&client->dev, "%s [ERROR] sysfs_create_group\n", __func__);
			return -EAGAIN;
		}
		info->key_dev = device_create(sec_class, NULL, 0, info, "sec_touchkey");
		if (sysfs_create_group(&info->key_dev->kobj, &mms_key_attr_group)) {
			dev_err(&client->dev, "%s [ERROR] sysfs_create_group\n", __func__);
			return -EAGAIN;
		}
	}

	return 0;
}

void mms_sysfs_cmd_remove(struct mms_ts_info *info)
{
	sysfs_remove_group(&info->client->dev.kobj, &mms_cmd_attr_group);

	if (info->pdata->key_size > 0)
		sysfs_remove_group(&info->client->dev.kobj, &mms_key_attr_group);

	kfree(info->print_buf);
	kfree(info->image_buf);

	return;
}
#endif
