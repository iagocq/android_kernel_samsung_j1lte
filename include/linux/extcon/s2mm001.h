/*
 *
 * Copyright (c) Samsung Electronics Co, Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef _S2MM001_MUIC_H_
#define _S2MM001_MUIC_H_

#include <linux/pm_qos.h>
#include <linux/switch.h>
#include <linux/types.h>
#include <linux/extcon/muic-sm550x.h>


enum s2mm001_muic_reg {
	S2MM001_MUIC_REG_DEVID		= 0x01,
	S2MM001_MUIC_REG_CTRL1		= 0x02,
	S2MM001_MUIC_REG_INT1		= 0x03,
	S2MM001_MUIC_REG_INT2		= 0x04,
	S2MM001_MUIC_REG_INTMASK1	= 0x05,
	S2MM001_MUIC_REG_INTMASK2	= 0x06,
	S2MM001_MUIC_REG_ADC		= 0x07,
	S2MM001_MUIC_REG_TIMING1	= 0x08,
	S2MM001_MUIC_REG_TIMING2	= 0x09,
	S2MM001_MUIC_REG_DEV_T1		= 0x0a,
	S2MM001_MUIC_REG_DEV_T2		= 0x0b,
	S2MM001_MUIC_REG_BUTTON1	= 0X0c,
	S2MM001_MUIC_REG_BUTTON2	= 0X0d,
	S2MM001_MUIC_REG_MANSW1		= 0x13,
	S2MM001_MUIC_REG_MANSW2		= 0x14,
	S2MM001_MUIC_REG_DEV_T3		= 0x15,
	S2MM001_MUIC_REG_RESET		= 0x1B,
	S2MM001_MUIC_REG_TIMING3	= 0x20,
	S2MM001_MUIC_REG_OCP_SET	= 0x22,
	S2MM001_MUIC_REG_CTRL2		= 0x23,
	S2MM001_MUIC_REG_ADC_MODE	= 0x25,
	S2MM001_MUIC_REG_END,
};

/* S2MM001 Control register */
#define CTRL1_SWITCH_OPEN		(1 << 4)
#define CTRL1_RAW_DATA			(1 << 3)
#define CTRL1_MANUAL_SW			(1 << 2)
#define CTRL1_WAIT			(1 << 1)
#define CTRL1_INT_MASK			(1 << 0)
#define CTRL1_INIT			(CTRL1_SWITCH_OPEN | CTRL1_RAW_DATA |\
						CTRL1_MANUAL_SW | CTRL1_WAIT |\
						!CTRL1_INT_MASK)
#define CTRL1_DEF_INT			0x1F

/* S2MM001 Interrupt 1 register */
#define INT1_OVP_OCP			(1 << 7)
#define INT1_VBUS_OCP			(1 << 6)
#define INT1_VBUS_OVP			(1 << 5)
#define INT1_LKR			(1 << 4) /* Long Key Released */
#define INT1_LKP			(1 << 3) /* Long Key Pressed */
#define INT1_KP				(1 << 2) /* Key Pressed */
#define INT1_DETACH			(1 << 1)
#define INT1_ATTACH			(1 << 0)

#define INT1_MASK_INIT			(INT1_OVP_OCP | INT1_VBUS_OCP | \
					INT1_VBUS_OVP | INT1_LKR | INT1_LKP |\
					INT1_KP | !INT1_DETACH | !INT1_ATTACH)

/* S2MM001 Interrupt 2 register */
#define INT2_AV_CHARGE			(1 << 6)
#define INT2_MHDL			(1 << 5)
#define INT2_STUCK_RCV			(1 << 4)
#define INT2_STUCK			(1 << 3)
#define INT2_ADC_CHANGE			(1 << 2)
#define INT2_RSV_ATTACH			(1 << 1) /* Reseved Accessory */
#define INT2_VBUS_DET			(1 << 0)

#define INT2_MASK_INIT			(INT2_AV_CHARGE | INT2_MHDL | \
					INT2_STUCK_RCV | INT2_STUCK | \
					INT2_ADC_CHANGE | INT2_RSV_ATTACH | \
					!INT2_VBUS_DET)

/* S2MM001 Device Type 1 register */
#define DEV_TYPE1_USB_OTG		(1 << 7)
#define DEV_TYPE1_DEDICATED_CHG		(1 << 6)
#define DEV_TYPE1_CDP			(1 << 5)
#define DEV_TYPE1_CARKIT		(1 << 4)
#define DEV_TYPE1_UART			(1 << 3)
#define DEV_TYPE1_USB			(1 << 2)
#define DEV_TYPE1_AUDIO_2		(1 << 1)
#define DEV_TYPE1_AUDIO_1		(1 << 0)
#define DEV_TYPE1_USB_TYPES		(DEV_TYPE1_USB_OTG | \
						DEV_TYPE1_USB)
#define DEV_TYPE1_CHG_TYPES		(DEV_TYPE1_DEDICATED_CHG | \
						DEV_TYPE1_CDP)

/* S2MM001 Device Type 2 register */
#define DEV_TYPE2_AV			(1 << 6)
#define DEV_TYPE2_TTY			(1 << 5)
#define DEV_TYPE2_PPD			(1 << 4)
#define DEV_TYPE2_JIG_UART_OFF		(1 << 3)
#define DEV_TYPE2_JIG_UART_ON		(1 << 2)
#define DEV_TYPE2_JIG_USB_OFF		(1 << 1)
#define DEV_TYPE2_JIG_USB_ON		(1 << 0)
#define DEV_TYPE2_JIG_USB_TYPES		(DEV_TYPE2_JIG_USB_OFF | \
						DEV_TYPE2_JIG_USB_ON)
#define DEV_TYPE2_JIG_UART_TYPES	(DEV_TYPE2_JIG_UART_OFF | \
						DEV_TYPE2_JIG_UART_ON)
#define DEV_TYPE2_JIG_TYPES		(DEV_TYPE2_JIG_UART_TYPES | \
						DEV_TYPE2_JIG_USB_TYPES)

/* S2MM001 Device Type 3 register */
#define DEV_TYPE3_AV75			(1 << 7) /* A/V Cable 75 ohm */
#define DEV_TYPE3_U200_CHG		(1 << 6)
#define DEV_TYPE3_APPLE_CHG		(1 << 5)
#define DEV_TYPE3_AV_WITH_VBUS		(1 << 4)
#define DEV_TYPE3_NO_STD_CHG		(1 << 2)
#define DEV_TYPE3_VBUS_VALID		(1 << 1)
#define DEV_TYPE3_MHL			(1 << 0)
#define DEV_TYPE3_CHG_TYPE		(DEV_TYPE3_U200_CHG | \
						DEV_TYPE3_NO_STD_CHG | \
						DEV_TYPE3_APPLE_CHG)
/*
 * Manual Switch
 * D- [7:5] / D+ [4:2] / CHARGER[1] / OTGEN[0]
 * 000: Open all / 001: USB / 010: AUDIO / 011: UART / 100: V_AUDIO
 * 00: Vbus to Open / 01: Vbus to Charger / 10: Vbus to MIC / 11: Vbus to VBout
 */
#define MANUAL_SW1_DM_SHIFT		5
#define MANUAL_SW1_DP_SHIFT		2
#define MANUAL_SW1_VBUS_SHIFT		0
#define MANUAL_SW1_D_OPEN		(0x0)
#define MANUAL_SW1_D_USB		(0x1)
#define MANUAL_SW1_D_AUDIO		(0x2)
#define MANUAL_SW1_D_UART		(0x3)
#define MANUAL_SW1_V_OPEN		(0x0)
#define MANUAL_SW1_V_CHARGER		(0x2)
#define MANUAL_SW1_V_OTGEN		(0x1)

/* MANUAL SWITCH 1 */
#define CON_TO_USB	(MANUAL_SW1_D_USB << MANUAL_SW1_DM_SHIFT) | \
			(MANUAL_SW1_D_USB << MANUAL_SW1_DP_SHIFT) | \
			(MANUAL_SW1_V_CHARGER << MANUAL_SW1_VBUS_SHIFT)
#define CON_TO_UART	(MANUAL_SW1_D_UART << MANUAL_SW1_DM_SHIFT) | \
			(MANUAL_SW1_D_UART << MANUAL_SW1_DP_SHIFT) | \
			(MANUAL_SW1_V_CHARGER << MANUAL_SW1_VBUS_SHIFT)
#define CON_OPEN	(MANUAL_SW1_D_OPEN << MANUAL_SW1_DM_SHIFT) | \
			(MANUAL_SW1_D_OPEN << MANUAL_SW1_DP_SHIFT) | \
			(MANUAL_SW1_V_OPEN << MANUAL_SW1_VBUS_SHIFT)

#define RESET_BIT			(1 << 0)

#define I2C_RW_RETRY_MAX    3
#define I2C_RW_RETRY_DELAY  15

struct s2mm001_platform_data {
	void (*charger_cb) (u8 attached);
	u32 qos_val;
};

struct s2mm001_usbsw {
	struct i2c_client *client;
	struct s2mm001_platform_data *pdata;
	struct work_struct work;
	struct pm_qos_request qos_idle;
	struct switch_dev dock_dev;
	struct mutex mutex;
	u8 id;
	u8 dev1;
	u8 dev2;
	u8 dev3;
	u8 adc;
	u8 chg_type;
	u8 intr1;
	u8 intr2;
};

struct ic_vendor {
	u8 id;
	char *part_num;
};

#if 0
enum {
	CABLE_NONE_MUIC = 0,
	CABLE_SDP_MUIC,		//Max 500mA with Data
	CABLE_DCP_MUIC,		//Max 1.5A
	CABLE_CDP_MUIC,		//Max 1.5A with Data
	CABLE_OTG_MUIC,
	CABLE_CARKIT_T1_MUIC,	//Max 500mA with Data (Because of LG USB cable)
	CABLE_UART_MUIC,
	CABLE_JIG_UART_OFF_MUIC,
	CABLE_JIG_UART_OFF_VB_MUIC,
	CABLE_JIG_UART_ON_MUIC,
	CABLE_JIG_UART_ON_VB_MUIC,
	CABLE_JIG_USB_ON_MUIC,
	CABLE_JIG_USB_OFF_MUIC,
	CABLE_DESKTOP_DOCK_MUIC,
	CABLE_DESKTOP_DOCK_VB_MUIC,
	CABLE_UNKNOWN,
};

extern int usb_switch_register_notify(struct notifier_block *nb);
extern int usb_switch_unregister_notify(struct notifier_block *nb);
#endif

extern int get_jig_state(void);
extern void muic_attached_accessory_inquire(void);
extern int is_battery_connected(void);
#endif /* _S2MM001_MUIC_H_ */
