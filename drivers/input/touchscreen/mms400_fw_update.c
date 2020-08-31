/*
 * MELFAS MMS438/449/458 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * Firmware update functions
 *
 */

#include "melfas_mms400.h"

/* ISC Info */
#define ISC_PAGE_SIZE				128

/* ISC Command */
#define ISC_CMD_ERASE_ALL			{0xFB, 0x4A, 0x00, 0x15, 0x00, 0x00}
#define ISC_CMD_ERASE_PAGE			{0xFB, 0x4A, 0x00, 0x8F, 0x00, 0x00}
#define ISC_CMD_READ_PAGE			{0xFB, 0x4A, 0x00, 0xC2, 0x00, 0x00}
#define ISC_CMD_WRITE_PAGE			{0xFB, 0x4A, 0x00, 0xA5, 0x00, 0x00}
#define ISC_CMD_PROGRAM_PAGE		{0xFB, 0x4A, 0x00, 0x54, 0x00, 0x00}
#define ISC_CMD_READ_STATUS		{0xFB, 0x4A, 0x36, 0xC2, 0x00, 0x00}
#define ISC_CMD_EXIT				{0xFB, 0x4A, 0x00, 0x66, 0x00, 0x00}

/* ISC Status */
#define ISC_STATUS_BUSY				0x96
#define ISC_STATUS_DONE				0xAD

static int mms_isc_read_status(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;
	u8 cmd[6] =  ISC_CMD_READ_STATUS;
	u8 result = 0;
	int cnt = 100;
	int ret = 0;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = cmd,
			.len = 6,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = &result,
			.len = 1,
		},
	};

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	do {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
			dev_err(&info->client->dev, "%s: i2c_transfer fail\n", __func__);
			return -1;
		}

		if (result == ISC_STATUS_DONE)
			break;
		if (result != ISC_STATUS_BUSY)
			dev_err(&info->client->dev, "%s: wrong value [0x%02X]\n", __func__, result);
		msleep(20);	/* FIXME: msleep(1) */

	} while (--cnt);

	if (!cnt) {
		dev_err(&info->client->dev, "count overflow - cnt [%d] status [0x%02X]\n", cnt, result);
		ret = -1;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return ret;
}

static int mms_isc_erase_page(struct mms_ts_info *info, int offset)
{
	u8 write_buf[6] = ISC_CMD_ERASE_PAGE;

	struct i2c_msg msg[1] = {
		{
			.addr = info->client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = 6,
		},
	};

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	write_buf[4] = (u8)((offset >> 8) & 0xFF);
	write_buf[5] = (u8)((offset >> 0) & 0xFF);
	if (i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
		dev_err(&info->client->dev, "%s [ERROR] i2c_transfer\n", __func__);
		return -1;
	}

	if (mms_isc_read_status(info))
		return -1;

	dev_dbg(&info->client->dev, "%s [DONE] - Offset [0x%04X]\n", __func__, offset);

	return 0;
}

static int mms_isc_read_page(struct mms_ts_info *info, int offset, u8 *data)
{
	u8 write_buf[6] = ISC_CMD_READ_PAGE;

	struct i2c_msg msg[2] = {
		{
			.addr = info->client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = 6,
		}, {
			.addr = info->client->addr,
			.flags = I2C_M_RD,
			.buf = data,
			.len = ISC_PAGE_SIZE,
		},
	};

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	write_buf[4] = (u8)((offset >> 8) & 0xFF);
	write_buf[5] = (u8)((offset >> 0) & 0xFF);
	if (i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
		dev_err(&info->client->dev, "%s: i2c_transfer fail\n", __func__);
		return -1;
	}

	dev_dbg(&info->client->dev, "%s [DONE] - Offset [0x%04X]\n", __func__, offset);

	return 0;
}

static int mms_isc_program_page(struct mms_ts_info *info, int offset, const u8 *data, int length)
{
	u8 write_buf[134] = ISC_CMD_PROGRAM_PAGE;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if (length > 128) {
		dev_err(&info->client->dev, "%s: page length overflow\n", __func__);
		return -1;
	}

	write_buf[4] = (u8)((offset >> 8) & 0xFF);
	write_buf[5] = (u8)((offset >> 0) & 0xFF);

	memcpy(&write_buf[6], data, length);

	if (i2c_master_send(info->client, write_buf, length + 6) != (length + 6)) {
		dev_err(&info->client->dev, "%s: i2c_master_send\n", __func__);
		return -1;
	}

	if (mms_isc_read_status(info))
		return -1;

	dev_dbg(&info->client->dev, "%s: Offset[0x%04X] Length[%d]\n",
								__func__, offset, length);

	return 0;
}

static int mms_isc_exit(struct mms_ts_info *info)
{
	u8 write_buf[6] = ISC_CMD_EXIT;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if (i2c_master_send(info->client, write_buf, 6) != 6) {
		dev_err(&info->client->dev, "%s: i2c_master_send\n", __func__);
		return -1;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return 0;
}

int mms_flash_fw(struct mms_ts_info *info, const u8 *fw_data, size_t fw_size, bool force, bool section)
{

	struct mms_bin_hdr *fw_hdr;
	struct mms_fw_img **img;
	struct i2c_client *client = info->client;
	int i;
	int retries = 3;
	int ret = -1;
	int nStartAddr;
	int nWriteLength;
	int nLast;
	int nOffset;
	int nTransferLength;
	int size;
	u8 *data;
	u8 *cpydata;

	int offset = sizeof(struct mms_bin_hdr);

	bool update_flag = false;
	bool update_flags[MMS_FW_MAX_SECT_NUM] = {false, };

	u16 ver_chip[MMS_FW_MAX_SECT_NUM];
	u16 ver_file[MMS_FW_MAX_SECT_NUM];

	int offsetStart = 0;
	u8 initData[ISC_PAGE_SIZE];
	memset(initData, 0xFF, sizeof(initData));

	dev_dbg(&client->dev, "%s [START]\n", __func__);

	fw_hdr = (struct mms_bin_hdr *)fw_data;
	img = kzalloc(sizeof(*img) * fw_hdr->section_num, GFP_KERNEL);
	if (unlikely(!img))
		goto ERR0;

	if (unlikely(memcmp(CHIP_FW_CODE, &fw_hdr->tag[4], 4))) {
		dev_err(&client->dev, "F/W file is not for %s\n", CHIP_NAME);
		goto ERR1;
	}

	/* Check chip firmware version */
	while (retries--) {
		if (!mms_get_fw_version_u16(info, ver_chip))
			break;
		else
			mms_reboot(info);
	}

	if (unlikely(retries < 0)) {
		dev_dbg(&client->dev, "cannot read chip firmware version\n");
		memset(ver_chip, 0xFFFF, sizeof(ver_chip));
		dev_dbg(&client->dev, "Chip firmware version is set to [0xFFFF]\n");
	} else {
		dev_info(&client->dev, "Chip firmware version [0x%04X 0x%04X 0x%04X 0x%04X]\n",
						ver_chip[0], ver_chip[1], ver_chip[2], ver_chip[3]);
	}

	/* set update flag */
	dev_info(&client->dev, "Firmware file info : Sections[%d] Offset[0x%08X] Length[0x%08X]\n",
				fw_hdr->section_num, fw_hdr->binary_offset, fw_hdr->binary_length);

	for (i = 0; i < fw_hdr->section_num; i++, offset += sizeof(struct mms_fw_img)) {
		img[i] = (struct mms_fw_img *)(fw_data + offset);
		ver_file[i] = img[i]->version;
		dev_dbg(&client->dev,
		"Section[%d] Version[0x%04X] StartPage[%d] EndPage[%d] Offset[0x%08X] Length[0x%08X]\n",
		i, img[i]->version, img[i]->start_page, img[i]->end_page, img[i]->offset, img[i]->length);

		/* Compare section version */
		if (ver_chip[i] != ver_file[i]) {
			update_flag = true;
			update_flags[i] = true;
			dev_dbg(&client->dev,
			"Section [%d] is need to be updated. Chip[0x%04X] File[0x%04X]\n",
								i, ver_chip[i], ver_file[i]);
		}
	}

	/* Set force update flag */
	if (force) {
		update_flag = true;
		for (i = 0; i < MMS_FW_MAX_SECT_NUM; i++)
			update_flags[i] = true;
		dev_dbg(&client->dev, "%s - Force update\n", __func__);
	}

	/* Exit when up-to-date */
	if (!update_flag) {
		ret = 0;
		dev_info(&client->dev, "Chip firmware is already up-to-date\n");
		goto EXIT;
	}

	/* Set start addr offset */
	if (section) {
		if (update_flags[0])
			offsetStart = img[0]->start_page;
		else if (update_flags[1])
			offsetStart = img[1]->start_page;
		else if (update_flags[2])
			offsetStart = img[2]->start_page;
		else if (update_flags[3])
			offsetStart = img[3]->start_page;
	} else
		offsetStart = 0;

	offsetStart = offsetStart * 1024;

	/* Load firmware data */
	data = kzalloc(sizeof(u8) * fw_hdr->binary_length, GFP_KERNEL);
	if (unlikely(!data))
		goto ERR1;

	size = fw_hdr->binary_length;
	cpydata = kzalloc(ISC_PAGE_SIZE, GFP_KERNEL);
	if (unlikely(!cpydata))
		goto ERR2;

	/* Check firmware size */
	if (size % ISC_PAGE_SIZE != 0)
		size += (ISC_PAGE_SIZE - (size % ISC_PAGE_SIZE));

	nStartAddr = 0;
	nWriteLength = size;
	nLast = nStartAddr + nWriteLength;

	if (unlikely((nLast % 8) != 0)) {
		dev_err(&client->dev, "Firmware size mismatch\n");
		goto ERR3;
	} else
		memcpy(data, fw_data+fw_hdr->binary_offset, fw_hdr->binary_length);

	/* Set address */
	nOffset = nStartAddr + nWriteLength - ISC_PAGE_SIZE;
	nTransferLength = ISC_PAGE_SIZE;

	/* Erase first page */
	dev_info(&client->dev, "Erase first page : Offset[0x%04X]\n", offsetStart);
	ret = mms_isc_erase_page(info, offsetStart);
	if (unlikely(ret)) {
		dev_err(&client->dev, "clear first page failed\n");
		goto ERR3;
	}

	/* Flash firmware */
	dev_info(&client->dev, "Start Download : Offset Start[0x%04X] End[0x%04X]\n",
								nOffset, offsetStart);
	while (nOffset >= offsetStart) {
		dev_dbg(&client->dev, "Downloading : Offset[0x%04X]\n", nOffset);
		ret = mms_isc_program_page(info, nOffset, &data[nOffset], nTransferLength);
		if (unlikely(ret)) {
			dev_err(&client->dev, "isc_program_page error\n");
			goto ERR3;
		}

		/* Verify (read and compare) */
		ret = mms_isc_read_page(info, nOffset, cpydata);
		if (unlikely(ret)) {
			dev_err(&client->dev, "mms_isc_read_page error\n");
			goto ERR3;
		}

		ret = memcmp(&data[nOffset], cpydata, ISC_PAGE_SIZE);
		if (unlikely(ret)) {
#if MMS_HEX_DEBUG
			print_hex_dump(KERN_ERR, "Firmware Page Write : ",
					DUMP_PREFIX_OFFSET, 16, 1, data, ISC_PAGE_SIZE, false);
			print_hex_dump(KERN_ERR, "Firmware Page Read : ",
					DUMP_PREFIX_OFFSET, 16, 1, cpydata, ISC_PAGE_SIZE, false);
#endif
			dev_err(&client->dev, "verify page failed\n");
			goto ERR3;
		}
		nOffset -= nTransferLength;
	}

	ret = mms_isc_exit(info);
	if (unlikely(ret)) {
		dev_err(&client->dev, "mms_isc_exit failed\n");
		goto ERR3;
	}

	mms_reboot(info);

	/* Check chip firmware version */
	ret = mms_get_fw_version_u16(info, ver_chip);
	if (unlikely(ret)) {
		dev_err(&client->dev, "cannot read chip firmware version after flash\n");
		goto ERR3;
	} else {
		for (i = 0; i < fw_hdr->section_num; i++) {
			if (unlikely(ver_chip[i] != ver_file[i])) {
				dev_err(&client->dev, "version mismatch after flash.\n");
				dev_err(&client->dev, "Section[%d] : Chip[0x%04X] != File[0x%04X]\n",
									i, ver_chip[i], ver_file[i]);
				goto ERR3;
			}
		}
		dev_info(&client->dev, "New firmware version [0x%04X 0x%04X 0x%04X 0x%04X]\n",
						ver_chip[0], ver_chip[1], ver_chip[2], ver_chip[3]);
	}

ERR3:
	kfree(cpydata);
ERR2:
	kfree(data);
ERR1:
EXIT:
	kfree(img);
ERR0:
	dev_err(&client->dev, "Firmware update %s\n", ret ? "failed" : "success");

	return ret;
}

