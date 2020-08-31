/*
 * MELFAS MMS400 Touchscreen Driver
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/limits.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/gpio_event.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/input/melfas_mms.h>

//Chip info
#ifdef CONFIG_TOUCHSCREEN_MELFAS_MMS438
#define CHIP_MMS438
#define CHIP_NAME		"MMS438"
#define CHIP_FW_CODE	"M4H0"
#define FW_UPDATE_TYPE	"MMS438"
#endif
#ifdef CONFIG_TOUCHSCREEN_MELFAS_MMS449
#define CHIP_MMS449
#define CHIP_NAME		"MMS449"
#define CHIP_FW_CODE	"M4HP"
#define FW_UPDATE_TYPE	"MMS438"
#endif
#ifdef CONFIG_TOUCHSCREEN_MELFAS_MMS458
#define CHIP_MMS458
#define CHIP_NAME		"MMS458"
#define CHIP_FW_CODE	"M4HN"
#define FW_UPDATE_TYPE	"MMS438"
#endif
#ifdef CONFIG_TOUCHSCREEN_MELFAS_MMS492
#define CHIP_MMS492
#define CHIP_NAME		"MMS492"
#define CHIP_FW_CODE	"M4HL"
#define FW_UPDATE_TYPE	"MMS492"
#endif

//Config driver
#define MMS_USE_INPUT_OPEN_CLOSE	0	// 0 or 1
#define I2C_RETRY_COUNT			3	// 2~
#define RESET_ON_EVENT_ERROR		0	// 0 or 1
#define ESD_COUNT_FOR_DISABLE		7	// 7~

//Features
#define MMS_USE_CMD_MODE			1	// 0 or 1 (Optional - Sysfs command functions)
#define MMS_USE_NAP_MODE			0	// 0 or 1

#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
#define MMS_USE_TEST_MODE			0	/* 0 or 1 (Sysfs test functions) */
#define MMS_USE_DEV_MODE			0	/* 0 or 1 (Devmode for debugging) */
#else
#define MMS_USE_TEST_MODE			1
#define MMS_USE_DEV_MODE			1
#endif

//Input value
#define MAX_FINGER_NUM				10
#define INPUT_AREA_MIN 			0
#define INPUT_AREA_MAX 			255
#define INPUT_PRESSURE_MIN 		0
#define INPUT_PRESSURE_MAX 		255
#define INPUT_TOUCH_MAJOR_MIN 	0
#define INPUT_TOUCH_MAJOR_MAX 	255
#define INPUT_TOUCH_MINOR_MIN 	0
#define INPUT_TOUCH_MINOR_MAX 	255
#define INPUT_ANGLE_MIN 			0
#define INPUT_ANGLE_MAX 			255
#define INPUT_HOVER_MIN 			0
#define INPUT_HOVER_MAX 			255
#define INPUT_PALM_MIN 			0
#define INPUT_PALM_MAX 			1

//Firmware update
#define INTERNAL_FW_PATH	"melfas/melfas_mms400.fw"
#define EXTERNAL_FW_PATH	"/sdcard/melfas_mms400.mfsb"
#define MMS_USE_AUTO_FW_UPDATE		1	// 0 or 1
#define MMS_FW_MAX_SECT_NUM		4
#define MMS_HEX_DEBUG			0	// 0 or 1
#define MMS_FW_UPDATE_SECTION		1	// 0 or 1
#define MMS_EXT_FW_FORCE_UPDATE		1	//0 or 1

//Command mode
#define CMD_LEN 			32
#define CMD_RESULT_LEN 			512
#define CMD_PARAM_NUM 			8


/* register map */
//Address
#define MIP_R0_INFO							0x01
#define MIP_R1_INFO_PRODUCT_NAME			0x00
#define MIP_R1_INFO_RESOLUTION_X			0x10
#define MIP_R1_INFO_RESOLUTION_Y			0x12
#define MIP_R1_INFO_NODE_NUM_X			0x14
#define MIP_R1_INFO_NODE_NUM_Y			0x15
#define MIP_R1_INFO_KEY_NUM				0x16
#define MIP_R1_INFO_VERSION_BOOT			0x20
#define MIP_R1_INFO_VERSION_CORE			0x22
#define MIP_R1_INFO_VERSION_CUSTOM		0x24
#define MIP_R1_INFO_VERSION_PARAM			0x26
#define MIP_R1_INFO_SECT_BOOT_START		0x30
#define MIP_R1_INFO_SECT_BOOT_END			0x31
#define MIP_R1_INFO_SECT_CORE_START		0x32
#define MIP_R1_INFO_SECT_CORE_END			0x33
#define MIP_R1_INFO_SECT_CUSTOM_START	0x34
#define MIP_R1_INFO_SECT_CUSTOM_END		0x35
#define MIP_R1_INFO_SECT_PARAM_START		0x36
#define MIP_R1_INFO_SECT_PARAM_END		0x37
#define MIP_R1_INFO_BUILD_DATE				0x40
#define MIP_R1_INFO_BUILD_TIME				0x44
#define MIP_R1_INFO_CHECKSUM_PRECALC		0x48
#define MIP_R1_INFO_CHECKSUM_REALTIME	0x4A
#define MIP_R1_INFO_CHECKSUM_CALC			0x4C
#define MIP_R1_INFO_PROTOCOL_NAME		0x50
#define MIP_R1_INFO_PROTOCOL_VERSION		0x58
#define MIP_R1_INFO_IC_ID					0x70

#define MIP_R0_EVENT						0x02
#define MIP_R1_EVENT_SUPPORTED_FUNC		0x00
#define MIP_R1_EVENT_FORMAT				0x04
#define MIP_R1_EVENT_SIZE					0x06
#define MIP_R1_EVENT_PACKET_INFO			0x10
#define MIP_R1_EVENT_PACKET_DATA			0x11

#define MIP_R0_CTRL							0x06	
#define MIP_R1_CTRL_READY_STATUS			0x00
#define MIP_R1_CTRL_EVENT_READY			0x01
#define MIP_R1_CTRL_MODE					0x10
#define MIP_R1_CTRL_EVENT_TRIGGER_TYPE	0x11
#define MIP_R1_CTRL_RECALIBRATE			0x12
#define MIP_R1_CTRL_POWER_STATE			0x13
#define MIP_R1_CTRL_GESTURE_TYPE			0x14
#define MIP_R1_CTRL_DISABLE_ESD_ALERT		0x18
#define MIP_R1_CTRL_CHARGER_MODE			0x19
#define MIP_R1_CTRL_GLOVE_MODE			0x1A
#define MIP_R1_CTRL_WINDOW_MODE			0x1B
#define MIP_R1_CTRL_PALM_REJECTION		0x1C
#define MIP_R1_CTRL_DISABLE_EDGE_EXPAND	0x1D

#define MIP_R0_PARAM						0x08
#define MIP_R1_PARAM_BUFFER_ADDR			0x00
#define MIP_R1_PARAM_PROTOCOL			0x04
#define MIP_R1_PARAM_MODE				0x10

#define MIP_R0_TEST							0x0A
#define MIP_R1_TEST_BUF_ADDR				0x00
#define MIP_R1_TEST_PROTOCOL				0x02
#define MIP_R1_TEST_TYPE					0x10
#define MIP_R1_TEST_DATA_FORMAT			0x20
#define MIP_R1_TEST_ROW_NUM				0x20
#define MIP_R1_TEST_COL_NUM				0x21
#define MIP_R1_TEST_BUFFER_COL_NUM		0x22
#define MIP_R1_TEST_COL_AXIS				0x23
#define MIP_R1_TEST_KEY_NUM				0x24
#define MIP_R1_TEST_DATA_TYPE				0x25

#define MIP_R0_IMAGE						0x0C
#define MIP_R1_IMAGE_BUF_ADDR				0x00
#define MIP_R1_IMAGE_PROTOCOL_ID			0x04
#define MIP_R1_IMAGE_TYPE					0x10
#define MIP_R1_IMAGE_DATA_FORMAT			0x20
#define MIP_R1_IMAGE_ROW_NUM				0x20
#define MIP_R1_IMAGE_COL_NUM				0x21
#define MIP_R1_IMAGE_BUFFER_COL_NUM		0x22
#define MIP_R1_IMAGE_COL_AXIS				0x23
#define MIP_R1_IMAGE_KEY_NUM				0x24
#define MIP_R1_IMAGE_DATA_TYPE			0x25
#define MIP_R1_IMAGE_FINGER_NUM			0x30
#define MIP_R1_IMAGE_FINGER_AREA			0x31

#define MIP_R0_LOG							0x10
#define MIP_R1_LOG_TRIGGER					0x14

//Value
#define MIP_EVENT_INPUT_PRESS			0x80
#define MIP_EVENT_INPUT_SCREEN			0x40
#define MIP_EVENT_INPUT_HOVER			0x20
#define MIP_EVENT_INPUT_PALM			0x10
#define MIP_EVENT_INPUT_ID				0x0F

#define MIP_ALERT_ESD					1
#define MIP_ALERT_WAKEUP				2

#define MIP_CTRL_STATUS_NONE			0x05
#define MIP_CTRL_STATUS_READY			0xA0
#define MIP_CTRL_STATUS_LOG			0x77

#define MIP_CTRL_MODE_NORMAL			0		
#define MIP_CTRL_MODE_PARAM			1
#define MIP_CTRL_MODE_TEST_CM			2

#define MIP_TEST_TYPE_NONE				0
#define MIP_TEST_TYPE_CM_DELTA			1
#define MIP_TEST_TYPE_CM_ABS			2
#define MIP_TEST_TYPE_CM_JITTER			3
#define MIP_TEST_TYPE_SHORT			4

#define MIP_IMG_TYPE_NONE				0
#define MIP_IMG_TYPE_INTENSITY			1
#define MIP_IMG_TYPE_RAWDATA			2
#define MIP_IMG_TYPE_WAIT				255

#define MIP_TRIGGER_TYPE_NONE			0
#define MIP_TRIGGER_TYPE_INTR			1
#define MIP_TRIGGER_TYPE_REG			2

#define MIP_LOG_MODE_NONE			0
#define MIP_LOG_MODE_TRIG				1


#if MMS_USE_CALLBACK
struct mms_callbacks {
	void (*inform_charger) (struct mms_callbacks *, int);
};

extern struct mms_callbacks *mms_inform_callbacks;
#endif

extern struct class *sec_class;

/**
* Device info structure
*/
struct mms_ts_info {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[32];
	struct mms_platform_data *pdata;
	bool finger_state[MAX_FINGER_NUM];

	dev_t mms_dev;
	struct class *class;
	
	struct mutex lock;
	struct mutex lock_test;
	struct mutex lock_cmd;
	struct mutex lock_dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	
	int irq;
	bool enabled;
	bool init;	
	char *fw_name;
		
	u8 product_name[16];
	int max_x;
	int max_y;
	u8 node_x;
	u8 node_y;
	u8 node_key;
	u8 fw_version[8];
	u8 event_size;

	bool tkey_enable;
	
	u8 nap_mode;
	u8 glove_mode;
	u8 charger_mode;
	u8 cover_mode;
	
	u8 esd_cnt;
	bool disable_esd;

	u8 *print_buf;	
	int *image_buf;
	
	bool test_busy;
	bool cmd_busy;
	bool dev_busy;

#if MMS_USE_CMD_MODE
	dev_t cmd_dev_t;
	struct device *cmd_dev;
	struct class *cmd_class;
	struct list_head cmd_list_head;
	u8 cmd_state;
	char cmd[CMD_LEN];
	char cmd_result[CMD_RESULT_LEN];
	int cmd_param[CMD_PARAM_NUM];
	int cmd_buffer_size;
	
	struct device *key_dev;
#endif	

#if MMS_USE_DEV_MODE
	struct cdev cdev;
	u8 *dev_fs_buf;
#endif	

#if MMS_USE_CALLBACK
	void (*register_callback)(void *);
	struct mms_callbacks callbacks;
#endif

};

/**
* Firmware binary header info
*/
struct mms_bin_hdr {
	char	tag[8];
	u16	core_version;
	u16	section_num;
	u16	contains_full_binary;
	u16	reserved0;

	u32	binary_offset;
	u32	binary_length;

	u32	extention_offset;	
	u32	reserved1;
} __attribute__ ((packed));

/**
* Firmware image info
*/
struct mms_fw_img {
	u16	type;
	u16	version;

	u16	start_page;
	u16	end_page;

	u32	offset;
	u32	length;
} __attribute__ ((packed));

/**
* Declarations
*/
void mms_reboot(struct mms_ts_info *info);
void mms_enable(struct mms_ts_info *info);
void mms_disable(struct mms_ts_info *info);
void mms_clear_input(struct mms_ts_info *info);
int mms_i2c_read(struct mms_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len);
int mms_i2c_write(struct mms_ts_info *info, char *write_buf, unsigned int write_len);
int mms_get_ready_status(struct mms_ts_info *info);
int mms_get_fw_version(struct mms_ts_info *info, u8 *ver_buf);
int mms_get_fw_version_u16(struct mms_ts_info *info, u16 *ver_buf_u16);
int mms_fw_update_from_kernel(struct mms_ts_info *info);
int mms_fw_update_from_storage(struct mms_ts_info *info, bool force);
void mms_set_power(struct i2c_client *client, bool enable);

#if MMS_USE_CALLBACK
void mms_config_callback(struct mms_ts_info *info);
#endif

int mms_flash_fw(struct mms_ts_info *info, const u8 *fw_data, size_t fw_size, bool force, bool section);

#if MMS_USE_DEV_MODE
int mms_dev_create(struct mms_ts_info *info);
int mms_get_log(struct mms_ts_info *info);
#endif
int mms_run_test(struct mms_ts_info *info, u8 test_type);
int mms_get_image(struct mms_ts_info *info, u8 image_type);
#if MMS_USE_TEST_MODE
int mms_sysfs_create(struct mms_ts_info *info);
void mms_sysfs_remove(struct mms_ts_info *info);
static const struct attribute_group mms_test_attr_group;
#endif

#if MMS_USE_CMD_MODE
int mms_sysfs_cmd_create(struct mms_ts_info *info);
void mms_sysfs_cmd_remove(struct mms_ts_info *info);
static const struct attribute_group mms_cmd_attr_group;
#endif
