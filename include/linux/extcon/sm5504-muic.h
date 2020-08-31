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

#ifndef _SM5504_MUIC_H_
#define _SM5504_MUIC_H_

#include <linux/pm_qos.h>
#include <linux/switch.h>
#include <linux/types.h>
#include <linux/extcon/muic-sm550x.h>

/* REGISTER */
#define REG_DEVID		0x01
#define REG_CTRL		0x02
#define REG_INT1		0x03
#define REG_INT2		0x04
#define REG_INT1_MASK		0x05
#define REG_INT2_MASK		0x06
#define REG_ADC			0x07
#define REG_DEV_T1		0x0A
#define REG_DEV_T2		0x0B
#define REG_MANSW1		0x13
#define REG_MANSW2		0x14
#define REG_RESET		0x1B
#define REG_CHG_TYPE		0x24

/* CONTROL */
#define ADC_EN			(1 << 7)
#define USBCHDEN		(1 << 6)
#define CHGTYP			(1 << 5)
#define SWITCH_OPEN		(1 << 4)
#define MANUAL_SWITCH		(1 << 2)
#define MASK_INT		(1 << 0)
#define CTRL_INIT		(ADC_EN | USBCHDEN | CHGTYP | MANUAL_SWITCH)
#define RESET_DEFAULT		(ADC_EN | USBCHDEN | CHGTYP | MANUAL_SWITCH | MASK_INT)

/* INTERRUPT 1 */
#define ADC_CHG			(1 << 6)
#define CONNECT			(1 << 5)
#define OVP			(1 << 4)
#define DCD_OUT			(1 << 3)
#define CHGDET			(1 << 2)
#define DETACH			(1 << 1)
#define ATTACH			(1 << 0)

/* INTERRUPT 2 */
#define OVP_OCP			(1 << 7)
#define OCP			(1 << 6)
#define OCP_LATCH		(1 << 5)
#define OVP_FET			(1 << 4)
#define POR			(1 << 2)
#define UVLO			(1 << 1)
#define RID_CHARGER		(1 << 0)

/* INTMASK 1 */
#define ADC_CHG_M		(1 << 6)
#define CONNECT_M		(1 << 5)
#define OVP_M			(1 << 4)
#define DCD_OUT_M		(1 << 3)
#define CHGDET_M		(1 << 2)
#define DETACH_M		(1 << 1)
#define ATTACH_M		(1 << 0)
#define INTMASK1_INIT		(ADC_CHG_M | CONNECT_M | OVP_M | DCD_OUT_M | CHGDET_M)

/* INTMASK 2 */
#define OVP_OCP_M		(1 << 7)
#define OCP_M			(1 << 6)
#define OCP_LATCH_M		(1 << 5)
#define OVP_FET_M		(1 << 4)
#define POR_M			(1 << 2)
#define UVLO_M			(1 << 1)
#define RID_CHARGER_M		(1 << 0)
#define INTMASK2_INIT		(OVP_OCP_M | POR_M | UVLO_M | RID_CHARGER_M)

/* DEVICE TYPE 1*/
#define DEV_DCP			(1 << 6)	//Max 1.5A
#define DEV_CDP			(1 << 5)	//Max 1.5A with Data
#define DEV_CARKIT_T1		(1 << 4)
#define DEV_UART		(1 << 3)
#define DEV_SDP			(1 << 2)	//Max 500mA with Data
#define DEV_OTG			(1 << 0)
#define DEV_CHARGER		(DEV_DEDICATED_CHG | DEV_USB_CHG)

/* DEVICE TYPE 2*/
#define DEV_UNKNOWN		(1 << 7)
#define DEV_JIG_UART_OFF	(1 << 3)
#define DEV_JIG_UART_ON		(1 << 2)
#define DEV_JIG_USB_OFF		(1 << 1)
#define DEV_JIG_USB_ON		(1 << 0)
#define DEV_JIG_ALL		(DEV_JIG_UART_OFF | DEV_JIG_UART_ON | DEV_JIG_USB_OFF | DEV_JIG_USB_ON)
#define DEV_JIG_WAKEUP		(DEV_JIG_UART_OFF | DEV_JIG_UART_ON | DEV_JIG_USB_ON)

/* MANUAL SWITCH 1 */
#define CON_TO_USB		0x24
#define CON_TO_UART		0x6C
#define CON_OPEN		0x00

/* MANUAL SWITCH 2 */
#define BOOT_SW			(1 << 3)
#define JIG_ON			(1 << 2)
#define VBUS_FET_ONOFF		(1 << 0)

/* RESET */
#define IC_RESET		(1 << 0)

/* REG_CHG_TYPE */
#define TIMEOUT_SDP		(1 << 3)
#define SDP			(1 << 2)
#define CDP			(1 << 1)
#define DCP			(1 << 0)

#define I2C_RW_RETRY_MAX    3
#define I2C_RW_RETRY_DELAY  15

struct sm5504_platform_data {
	void (*charger_cb) (u8 attached);
	u32 qos_val;
};

struct sm5504_usbsw {
	struct i2c_client *client;
	struct sm5504_platform_data *pdata;
	struct work_struct work;
	struct pm_qos_request qos_idle;
	struct switch_dev dock_dev;
	struct mutex mutex;
	u8 id;
	u8 dev1;
	u8 dev2;
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
#endif /* _SM5504_MUIC_H_ */
