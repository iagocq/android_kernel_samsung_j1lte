/*
 * cyttsp5_platform.c
 * Cypress TrueTouch(TM) Standard Product V4 Platform Module.
 * For use with Cypress Txx4xx parts.
 * Supported parts include:
 * TMA4XX
 * TMA1036
 *
 * Copyright (C) 2013 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_platform.h>
#include "cyttsp5_regs.h"

#define GPIO_TSP_nINT_SECURE	17

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
#include "cyttsp5_firmware.h"

static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
#include "cyttsp5_params.h"
static struct touch_settings cyttsp5_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = &cyttsp5_sett_param_regs,
	.param_size = &cyttsp5_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};
#else
static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif


struct cyttsp5_loader_platform_data _cyttsp5_loader_platform_data = {
	.fw = &cyttsp5_firmware,
	.ttconfig = &cyttsp5_ttconfig,
	.flags = CY_LOADER_FLAG_CALIBRATE_AFTER_FW_UPGRADE,
};

int enabled = 0;

static int cyttsp5_hw_power(struct cyttsp5_core_platform_data *pdata, int on)
{
	struct regulator *regulator_3v3 = pdata->pwr_3p3;
	int pwr_1p8 = pdata->pwr_1p8;
	int ret;

	if (!regulator_3v3) {
		pr_err("%s : No vdd regulator_3v3\n", __func__);
		return -EPERM;
	}

	if(on) {
		ret = regulator_is_enabled(regulator_3v3);
		if (ret)
			pr_err("%s : Already enabled regulator_3v3\n", __func__);
		else
			ret = regulator_enable(regulator_3v3);


		ret = gpio_direction_output(pwr_1p8, 1);
				if (ret) {
					pr_err("[TSP]%s : unable to set_direciton for TSP_EN [%d] 1v8\n", __func__, pwr_1p8);
					return -EINVAL;
				}
	} else {
		ret = gpio_direction_output(pwr_1p8, 0);
		if (ret) {
			pr_err("[TSP]%s : unable to set_direciton for TSP_EN [%d] 1v8\n", __func__, pwr_1p8);
			return -EINVAL;
		}

		ret = regulator_is_enabled(regulator_3v3);
		if (ret)
			ret = regulator_disable(regulator_3v3);
		else
			pr_err("%s : Already disabled regulator_3v3\n", __func__);
	}
		pr_info("%s : %s\n", __func__,
		(on) ? "on" : "off");

	return ret;

}

int cyttsp5_xres(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	int rc;

	rc = cyttsp5_hw_power(pdata, 0);
	if (rc) {
		dev_err(dev, "%s: Fail power down HW\n", __func__);
		goto exit;
	}

	msleep(50);

	rc = cyttsp5_hw_power(pdata, 1);
	if (rc) {
		dev_err(dev, "%s: Fail power up HW\n", __func__);
		goto exit;
	}
	msleep(10);

exit:
	return rc;
}

int cyttsp5_init(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev)
{
	struct regulator *regulator_3v3;
	int pwr_1p8 = pdata->pwr_1p8;
	int ret;
	regulator_3v3 = regulator_get(dev, "v_tsp_3v3");
		if (IS_ERR(regulator_3v3)) {
			ret = PTR_ERR(regulator_3v3);
			dev_err(dev, "Failed to get vdd regulator_3v3 (%d)\n", ret);
		}

		gpio_request(pwr_1p8, "TSP_1p8_GPIO");
		ret = regulator_set_voltage(regulator_3v3, pdata->vdd_volt_3p3, pdata->vdd_volt_3p3);
		if (ret < 0) {
			dev_err(dev,
				"Failed to set vdd regulator_3v3 to %d, %d uV(%d)\n",
				pdata->vdd_volt_3p3, pdata->vdd_volt_3p3, ret);
			goto out;
		}

		dev_info(dev, "Set vdd regulator_1v8 to %d uV, regulator_3v3 to %d uV\n",
			pdata->vdd_volt_1p8, pdata->vdd_volt_3p3);

		pdata->pwr_3p3 = regulator_3v3;

	if (on) {
		return ret;
	} else {
		cyttsp5_hw_power(pdata, 0);
	}
out:
	return ret;
}

int cyttsp5_power(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	return cyttsp5_hw_power(pdata, on);
}

int cyttsp5_irq_stat(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}

#ifndef CONFIG_TOUCHSCREEN_CYTTSP5_DEVICETREE_SUPPORT // platform data of board file
#include <linux/input.h>
#include <linux/export.h>

#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_platform.h>

#define CYTTSP5_HID_DESC_REGISTER 1

#define CY_MAXX 720
#define CY_MAXY 1280
#define CY_MINX 0
#define CY_MINY 0

#define CY_ABS_MIN_X CY_MINX
#define CY_ABS_MIN_Y CY_MINY
#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MIN_P 0
#define CY_ABS_MAX_P 255
#define CY_ABS_MIN_W 0
#define CY_ABS_MAX_W 255

#define CY_ABS_MIN_T 0

#define CY_ABS_MAX_T 15

#define CY_IGNORE_VALUE 0xFFFF

static struct cyttsp5_core_platform_data _cyttsp5_core_platform_data = {
	.irq_gpio = GPIO_TSP_nINT_SECURE,
	.hid_desc_register = CYTTSP5_HID_DESC_REGISTER,
	.xres = cyttsp5_xres,
	.init = cyttsp5_init,
	.power = cyttsp5_power,
	.irq_stat = cyttsp5_irq_stat,
	.sett = {
		NULL,	/* Reserved */
		NULL,	/* Command Registers */
		NULL,	/* Touch Report */
		NULL,	/* Cypress Data Record */
		NULL,	/* Test Record */
		NULL,	/* Panel Configuration Record */
		NULL,	/* &cyttsp5_sett_param_regs, */
		NULL,	/* &cyttsp5_sett_param_size, */
		NULL,	/* Reserved */
		NULL,	/* Reserved */
		NULL,	/* Operational Configuration Record */
		NULL, /* &cyttsp5_sett_ddata, *//* Design Data Record */
		NULL, /* &cyttsp5_sett_mdata, *//* Manufacturing Data Record */
		NULL,	/* Config and Test Registers */
		NULL, /* &cyttsp5_sett_btn_keys, */	/* button-to-keycode table */
	},
	//.flags = CY_FLAG_CORE_POWEROFF_ON_SLEEP,
};

static const uint16_t cyttsp5_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0,
	ABS_MT_TOUCH_MINOR, 0, 255, 0, 0,
	ABS_MT_ORIENTATION, -128, 127, 0, 0,
	ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0,
};

struct touch_framework cyttsp5_framework = {
	.abs = (uint16_t *)&cyttsp5_abs[0],
	.size = ARRAY_SIZE(cyttsp5_abs),
	.enable_vkeys = 0,
};

static struct cyttsp5_mt_platform_data _cyttsp5_mt_platform_data = {
	.frmwrk = &cyttsp5_framework,
	.flags = CY_MT_FLAG_NONE,
	.inp_dev_name = "sec_touchscreen",
};

static struct cyttsp5_btn_platform_data _cyttsp5_btn_platform_data = {
	.inp_dev_name = "sec_touchkey",
};

struct cyttsp5_platform_data _cyttsp5_platform_data = {
	.core_pdata = &_cyttsp5_core_platform_data,
	.mt_pdata = &_cyttsp5_mt_platform_data,
	.loader_pdata = &_cyttsp5_loader_platform_data,
	.btn_pdata = &_cyttsp5_btn_platform_data,
};

EXPORT_SYMBOL(_cyttsp5_platform_data);

#endif //#if 1

