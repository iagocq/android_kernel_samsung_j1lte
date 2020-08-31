/*
 * Marvell 88PM886 Interface
 *
 * Copyright (C) 2014 Marvell International Ltd.
 *  Yi Zhang <yizhang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_88PM886_H
#define __LINUX_MFD_88PM886_H

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/atomic.h>
#include <linux/reboot.h>
#include "88pm886-reg.h"
#include <linux/battery/sec_charger.h>

#define PM886_RTC_NAME		"88pm886-rtc"
#define PM886_ONKEY_NAME	"88pm886-onkey"
#define PM886_CHARGER_NAME	"88pm886-charger"
#define PM886_BATTERY_NAME	"88pm886-battery"
#define PM886_HEADSET_NAME	"88pm886-headset"
#define PM886_VBUS_NAME		"88pm886-vbus"
#define PM886_CFD_NAME		"88pm886-leds"
#define PM886_REGULATOR_NAME	"88pm886-regulator"
#define PM886_DVC_NAME		"88pm886-dvc"
#define PM886_RGB_NAME		"88pm886-rgb"
#define PM886_DEBUGFS_NAME	"88pm886-debugfs"
#define PM886_GPADC_NAME	"88pm886-gpadc"
#define PM886_HWMON_NAME	"88pm886-hwmon"

enum {
	PM886_A0 = 0x00,
	PM886_A1 = 0xa1,
};

/* TODO: use chip id is better */
enum pm88x_type {
	PM886 = 1,
};

enum {
	PM886_RGB_LED0,
	PM886_RGB_LED1,
	PM886_RGB_LED2,
};

/* FIXME: change according to spec */
enum pm886_reg_nums {
	PM886_BASE_PAGE_NUMS = 0xff,
	PM886_POWER_PAGE_NUMS = 0xff,
	PM886_GPADC_PAGE_NUMS = 0xff,
	PM886_BATTERY_PAGE_NUMS = 0xff,
	PM886_TEST_PAGE_NUMS = 0xff,
};

enum pm886_pages {
	PM886_BASE_PAGE = 0,
	PM886_POWER_PAGE,
	PM886_GPADC_PAGE,
	PM886_BATTERY_PAGE,
	PM886_TEST_PAGE = 7,
};

enum pm886_gpadc {
	PM886_NO_GPADC = -1,
	PM886_GPADC0 = 0,
	PM886_GPADC1,
	PM886_GPADC2,
	PM886_GPADC3,
};

/* Interrupt Number in 88PM886 */
enum pm886_irq_number {
	PM886_IRQ_ONKEY,	/* EN1b0 *//* 0 */
	PM886_IRQ_EXTON,	/* EN1b1 */
	PM886_IRQ_CHG_GOOD,	/* EN1b2 */
	PM886_IRQ_BAT_DET,	/* EN1b3 */
	PM886_IRQ_RTC,		/* EN1b4 */
	PM886_IRQ_CLASSD,	/* EN1b5 *//* 5 */
	PM886_IRQ_XO,		/* EN1b6 */
	PM886_IRQ_GPIO,		/* EN1b7 */

	PM886_IRQ_VBAT,		/* EN2b0 *//* 8 */
				/* EN2b1 */
	PM886_IRQ_VBUS,		/* EN2b2 */
	PM886_IRQ_ITEMP,	/* EN2b3 *//* 10 */
	PM886_IRQ_BUCK_PGOOD,	/* EN2b4 */
	PM886_IRQ_LDO_PGOOD,	/* EN2b5 */

	PM886_IRQ_GPADC0,	/* EN3b0 */
	PM886_IRQ_GPADC1,	/* EN3b1 */
	PM886_IRQ_GPADC2,	/* EN3b2 *//* 15 */
	PM886_IRQ_GPADC3,	/* EN3b3 */
	PM886_IRQ_MIC_DET,	/* EN3b4 */
	PM886_IRQ_HS_DET,	/* EN3b5 */
	PM886_IRQ_GND_DET,	/* EN3b6 */

	PM886_IRQ_CHG_FAIL,	/* EN4b0 *//* 20 */
	PM886_IRQ_CHG_DONE,	/* EN4b1 */
				/* EN4b2 */
	PM886_IRQ_CFD_FAIL,	/* EN4b3 */
	PM886_IRQ_OTG_FAIL,	/* EN4b4 */
	PM886_IRQ_CHG_ILIM,	/* EN4b5 *//* 25 */
				/* EN4b6 */
	PM886_IRQ_CC,		/* EN4b7 *//* 27 */

	PM886_MAX_IRQ,			   /* 28 */
};

enum {
	PM886_ID_BUCK1 = 0,
	PM886_ID_BUCK2,
	PM886_ID_BUCK3,
	PM886_ID_BUCK4,
	PM886_ID_BUCK5,

	PM886_ID_LDO1 = 5,
	PM886_ID_LDO2,
	PM886_ID_LDO3,
	PM886_ID_LDO4,
	PM886_ID_LDO5,
	PM886_ID_LDO6,
	PM886_ID_LDO7,
	PM886_ID_LDO8,
	PM886_ID_LDO9,
	PM886_ID_LDO10,
	PM886_ID_LDO11,
	PM886_ID_LDO12,
	PM886_ID_LDO13,
	PM886_ID_LDO14 = 18,
	PM886_ID_LDO15,
	PM886_ID_LDO16 = 20,

	PM886_ID_RG_MAX = 21,
};

enum {
	PM886_NO_LED = -1,
	PM886_FLASH_LED = 0,
	PM886_TORCH_LED,
};

struct pm886_led_pdata {
	unsigned int cf_en;
	unsigned int cf_txmsk;
	int gpio_en;
	unsigned int flash_timer;
	unsigned int cls_ov_set;
	unsigned int cls_uv_set;
	unsigned int cfd_bst_vset;
	unsigned int bst_uvvbat_set;
	unsigned int max_flash_current;
	unsigned int max_torch_current;
	unsigned int torch_force_max_current;
};

struct pm886_chip {
	struct i2c_client *client;
	struct device *dev;

	struct i2c_client *power_page;	/* chip client for power page */
	struct i2c_client *gpadc_page;	/* chip client for gpadc page */
	struct i2c_client *battery_page;/* chip client for battery page */
	struct i2c_client *test_page;	/* chip client for test page */

	struct regmap *base_regmap;
	struct regmap *power_regmap;
	struct regmap *gpadc_regmap;
	struct regmap *battery_regmap;
	struct regmap *test_regmap;
	struct regmap *codec_regmap;

	unsigned short power_page_addr;	/* power page I2C address */
	unsigned short gpadc_page_addr;	/* gpadc page I2C address */
	unsigned short battery_page_addr;/* battery page I2C address */
	unsigned short test_page_addr;	/* test page I2C address */

	unsigned int chip_id;
	long type;
	int irq;

	int irq_mode;
	struct regmap_irq_chip_data *irq_data;

	bool rtc_wakeup;
	u8 powerdown1;
	u8 powerdown2;
	u8 powerup;

	struct notifier_block reboot_notifier;
	struct notifier_block cb_nb;

	struct pm886_platform_data *pdata;
};

struct pm886_platform_data {
	unsigned int chgen;
	int irq_gpio;
	unsigned long chg_irq_attr;
	sec_battery_platform_data_t *battery_data;
};

struct regmap *get_companion(void);
struct regmap *get_codec_companion(void);

/* 1.367 mV/LSB */
#define PM886_VBAT_2_VALUE(v)		((v << 9) / 700)
#define PM886_VALUE_2_VBAT(val)		((val * 700) >> 9)

/* 0.342 mV/LSB */
#define PM886_GPADC_VOL_2_VALUE(v)	((v << 9) / 175)
#define PM886_GPADC_VALUE_2_VOL(val)	((val * 175) >> 9)

struct pm886_chip *pm886_init_chip(struct i2c_client *client);
int pm886_parse_dt(struct device_node *np, struct pm886_chip *chip);

int pm886_init_pages(struct pm886_chip *chip);
int pm886_post_init_chip(struct pm886_chip *chip);
void pm800_exit_pages(struct pm886_chip *chip);

int pm886_init_subdev(struct pm886_chip *chip);
long pm886_of_get_type(struct device *dev);
void pm886_dev_exit(struct pm886_chip *chip);

int pm886_irq_init(struct pm886_chip *chip);
int pm886_irq_exit(struct pm886_chip *chip);
int pm886_apply_patch(struct pm886_chip *chip);
int pm886_stepping_fixup(struct pm886_chip *chip);
int pm886_apply_bd_patch(struct pm886_chip *chip, struct device_node *np);

extern struct regmap_irq_chip pm886_irq_chip;
extern const struct of_device_id pm886_of_match[];

/* dvc external interface */
#ifdef CONFIG_MFD_88PM886
int pm886_dvc_set_volt(u8 level, int uv);
int pm886_dvc_get_volt(u8 level);
#else
static inline int pm886_dvc_set_volt(u8 level, int uv)
{
	return 0;
}
static inline int pm886_dvc_get_volt(u8 level)
{
	return 0;
}
#endif

struct pm886_chip *pm886_get_chip(void);
void pm886_set_chip(struct pm886_chip *chip);
void  pm886_power_off(void);
int pm886_reboot_notifier_callback(struct notifier_block *nb,
				   unsigned long code, void *cmd);

/* gpadc part */
int extern_pm886_gpadc_set_current_generator(int gpadc_number, int on);
int extern_pm886_gpadc_get_volt(int gpadc_number, int *volt);
int extern_pm886_gpadc_set_bias_current(int gpadc_number, int bias);

#endif /* __LINUX_MFD_88PM886_H */
