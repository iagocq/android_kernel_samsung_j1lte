/* arch/arm64/boot/dts/pxa1908-degasvelte-00-gpio.h
 *
 *	Copyright (C) 2014 Marvell Technology Group Ltd.
 *	Copyright (C) 2014 Samsung Electronics Co, Ltd.
 *
 *	Most of configurations are taken from J1-LTE
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 */

#ifndef __PXA1908_DEGASVELTE_00_GPIO_H__
#define __PXA1908_DEGASVELTE_00_GPIO_H__

#define GPIO_EAR_SW		13
#define GPIO_EAR_G_DET		14
#define GPIO_EAR_DET_3P5	15
#define GPIO_3M_CAM_SCL_1P8	53
#define GPIO_3M_CAM_SDA_1P8	54
#define GPIO_3M_CAM_NRST	67
#define GPIO_3M_CAM_STBY	68
#define GPIO_CAM_MCLK		77
#define GPIO_VT_CAM_NRST	69
#define GPIO_VT_CAM_NSTBY	70
#define GPIO_VT_CAM_SCL_1P8	79
#define GPIO_VT_CAM_SDA_1P8	80
#define GPIO_ND_ALE_SM_WEN	107
#define GPIO_GPS_IRQ		81
#define GPIO_GPS_RST		82
#define GPIO_GPS_CS_N		83
#define GPIO_GPS_SYNC_CLK	84
#define GPIO_GNSS_RF_SDIN	85
#define GPIO_VOL_UP		16
#define GPIO_VOL_DN		17
#define GPIO_HOME_KEY		18
#define GPIO_HW_REV0		33
#define GPIO_HW_REV1		34
#define GPIO_HW_REV2		35
#define GPIO_HW_REV3		36
#define GPIO_LCD_3V3_EN		5
#define GPIO_LCD_ESD_DET	6
#define GPIO_LCD_NRST		96
#define GPIO_LCD_BLIC_ON	97
#define GPIO_MUIC_SCL_1P8	29
#define GPIO_MUIC_SDA_1P8	30
#define GPIO_MUIC_INT		0
#define GPIO_UART_BOOT_ON	91
#define GPIO_PA_EN		9
#define GPIO_T_FLASH_DET_N	11
#define GPIO_WLAN_PDN		98
#define GPIO_SENSOR_SCL_1P8	51
#define GPIO_SENSOR_SDA_1P8	52
#define GPIO_ACC_INT_N		10
#define GPIO_CONDUCTION_DET	31
#define GPIO_RF_TOUCH_INT	32
#define GPIO_TSP_INT		72
#define GPIO_TSP_VENDOR_1	61
#define GPIO_TSP_VENDOR_2	60
#define GPIO_TSP_SCL_1P8	73
#define GPIO_TSP_SDA_1P8	74
#define GPIO_USB_ID_AP		19
#define GPIO_MOTOR_EN		20

#endif	/*__PXA1908_DEGASVELTE_00_GPIO_H__*/
