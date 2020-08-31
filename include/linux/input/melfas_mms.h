/*
 * MELFAS MMS Touchscreen Driver - Platform Data
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 * Default path : linux/input/melfas_mms.h
 *
 */

#ifndef _LINUX_MMS_TOUCH_H
#define _LINUX_MMS_TOUCH_H


#define MMS_USE_CALLBACK	0
#define MAX_TOUCH_KEY		6

#define MMS_DEVICE_NAME	"mms_ts"

struct mms_platform_data {
	unsigned int max_x;
	unsigned int max_y;
	unsigned int x_num;
	unsigned int y_num;
	int gpio_intr;
	int gpio_vdd_en;
	u32 key_codes[MAX_TOUCH_KEY];
	u8 key_size;
#if MMS_USE_CALLBACK
	void (*register_callback) (void *);
#endif
};

#endif

