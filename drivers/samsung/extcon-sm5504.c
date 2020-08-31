/*
 * Copyright (c) 2014 Samsung Electronics Co, Ltd.
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
#define DEBUG

#include <linux/battery/sec_charging_common.h>
#include <linux/delay.h>
#include <linux/edge_wakeup_mmp.h>
#include <linux/err.h>
#include <linux/extcon/sm5504-muic.h>
#include <linux/gpio.h>
#include <linux/gpio-pxa.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/pm_wakeup.h>
#include <linux/platform_device.h>
#include <linux/sec-common.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

static BLOCKING_NOTIFIER_HEAD(usb_switch_notifier);

static struct sm5504_usbsw *chip;
static struct wakeup_source jig_suspend_wake;
static int jig_wakelock_acq;
static int probing = 1;
static int first_acce;
static int en_uart = 1;
static int attached_dev;

int usb_switch_register_notify(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&usb_switch_notifier, nb);
}
EXPORT_SYMBOL_GPL(usb_switch_register_notify);

int usb_switch_unregister_notify(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&usb_switch_notifier, nb);
}
EXPORT_SYMBOL_GPL(usb_switch_unregister_notify);

static int read_reg(struct i2c_client *client, u8 reg, u8 * data)
{
	int ret = 0;
	u8 buf[1];
	struct i2c_msg msg[2];

	buf[0] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev, "I2C Read Failed REG[0x%2x] (ret=%d)\n",
			reg, ret);
		return -EIO;
	}
	*data = buf[0];

	dev_info(&client->dev, "I2C Read REG[0x%2x] DATA[0x%2x]\n", reg,
		 buf[0]);
	return 0;
}

static int write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int ret = 0;
	u8 buf[2];
	struct i2c_msg msg[1];

	buf[0] = reg;
	buf[1] = data;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	dev_info(&client->dev, "I2C Write REG[0x%2x] DATA[0x%2x]\n", buf[0],
		 buf[1]);

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "I2C Write Failed (ret=%d) \n", ret);
		return -EIO;
	}

	return ret;
}

#if 0 //Fix warning: due to CL# 262121
static int retry_write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int result = -1, i;

	for (i = 0; i < I2C_RW_RETRY_MAX && (result < 0); i++) {
		result = write_reg(client, reg, data);
		if (result < 0)
			msleep(I2C_RW_RETRY_DELAY);
	}
	return result;
}
#endif

static int retry_read_reg(struct i2c_client *client, u8 reg, u8 * data)
{
	int result = -1, i;

	for (i = 0; i < I2C_RW_RETRY_MAX && (result < 0); i++) {
		result = read_reg(client, reg , data);
		if (result < 0)
			msleep(I2C_RW_RETRY_DELAY);
	}
	return result;
}

static ssize_t adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sm5504_usbsw *usbsw = chip;
	u8 adc_value[] = "1C";
	u8 adc_fail = 0;

	if (usbsw->dev2 & DEV_JIG_ALL) {
		pr_info("adc_show JIG_UART_OFF\n");
		return sprintf(buf, "%s\n", adc_value);
	} else {
		pr_info("adc_show no detect\n");
		return sprintf(buf, "%d\n", adc_fail);
	}
}

static ssize_t sm5504_set_syssleep(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct sm5504_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	dev_info(&client->dev, "%s+\n", __func__);
	if (!strncmp(buf, "1", 1)) {
		dev_info(&client->dev, "%s:release wake lock\n", __func__);
		pm_qos_update_request(&usbsw->qos_idle,
				      PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
		__pm_relax(&jig_suspend_wake);
	} else if (!strncmp(buf, "0", 1)) {
		dev_info(&client->dev, "%s:get wake lock\n", __func__);
		__pm_stay_awake(&jig_suspend_wake);
		pm_qos_update_request(&usbsw->qos_idle, PM_QOS_DEFAULT_VALUE);
	}
	dev_info(&client->dev, "%s-\n", __func__);

	return count;
}

static ssize_t attached_dev_attrs(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	dev_info(dev, "%s attached_dev:%d\n",
					__func__, attached_dev);

	switch (attached_dev) {
	case CABLE_NONE_MUIC:
	case CABLE_DESKTOP_DOCK_MUIC:
		return sprintf(buf, "No VPS\n");
	case CABLE_SDP_MUIC:
	case CABLE_CARKIT_T1_MUIC:
	case CABLE_DESKTOP_DOCK_VB_MUIC:
		return sprintf(buf, "USB\n");
	case CABLE_DCP_MUIC:
	case CABLE_CDP_MUIC:
		return sprintf(buf, "TA\n");
	case CABLE_UART_MUIC:
		return sprintf(buf, "UART\n");
	case CABLE_JIG_UART_OFF_MUIC:
		return sprintf(buf, "JIG UART OFF\n");
	case CABLE_JIG_UART_OFF_VB_MUIC:
		return sprintf(buf, "JIG UART OFF VB\n");
	case CABLE_JIG_UART_ON_MUIC:
		return sprintf(buf, "JIG UART ON\n");
	case CABLE_JIG_UART_ON_VB_MUIC:
		return sprintf(buf, "JIG UART ON VB\n");
	case CABLE_JIG_USB_ON_MUIC:
		return sprintf(buf, "JIG USB ON\n");
	case CABLE_JIG_USB_OFF_MUIC:
		return sprintf(buf, "JIG USB OFF\n");
	case CABLE_UNKNOWN:
	case CABLE_OTG_MUIC:
		return sprintf(buf, "UNKNOWN\n");
	default:
		break;
	}

	return sprintf(buf, "UNKNOWN\n");
}

static ssize_t usb_state_show_attrs(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t usb_sel_show_attrs(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "PDA");
}

static ssize_t uart_sel_show_attrs(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t uart_sel_store_attrs(struct device *dev,
				    struct device_attribute *attr, const char *buf,
				    size_t count)
{
	return count;
}
static DEVICE_ATTR(adc, S_IRUGO | S_IXOTH, adc_show, NULL);
static DEVICE_ATTR(syssleep, (S_IWUSR | S_IWGRP), NULL, sm5504_set_syssleep);
static DEVICE_ATTR(usb_state, S_IRUGO, usb_state_show_attrs, NULL);
static DEVICE_ATTR(usb_sel, S_IRUGO, usb_sel_show_attrs, NULL);
static DEVICE_ATTR(uart_sel, S_IRUGO | S_IWUSR | S_IWGRP, uart_sel_show_attrs,
		   uart_sel_store_attrs);
static DEVICE_ATTR(attached_dev, S_IRUGO, attached_dev_attrs, NULL);

static void manual_path_to_usb(int on)
{
	struct sm5504_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	u8 ctrl_reg, mansw1;

	read_reg(client, REG_MANSW1, &mansw1);
	read_reg(client, REG_CTRL, &ctrl_reg);

	mansw1 = on ? CON_TO_USB : CON_OPEN;
	on ? (ctrl_reg &= ~MANUAL_SWITCH) : (ctrl_reg |= MANUAL_SWITCH);

	write_reg(client, REG_MANSW1, mansw1);
	write_reg(client, REG_CTRL, ctrl_reg);
}

static void cutoff_path(int cut)
{
	struct sm5504_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	u8 ctrl_reg, mansw1, mansw2;

	read_reg(client, REG_MANSW1, &mansw1);
	read_reg(client, REG_MANSW2, &mansw2);
	read_reg(client, REG_CTRL, &ctrl_reg);

	mansw1 = CON_OPEN;
	cut ? (ctrl_reg &= ~MANUAL_SWITCH) : (ctrl_reg |= MANUAL_SWITCH);

	write_reg(client, REG_MANSW1, mansw1);
	write_reg(client, REG_CTRL, ctrl_reg);
}

static void additional_uvlo_int(int on)
{
	struct sm5504_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	u8 int2m;

	read_reg(client, REG_INT2_MASK, &int2m);

	on ? (int2m &= ~UVLO_M) : (int2m |= UVLO_M);

	write_reg(client, REG_INT2_MASK, int2m);
}

#if 0 //Fix warning: due to CL# 262121
static int sm5504_ic_reset(void)
{
	struct sm5504_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	u8 sintm1, sintm2, smansw1, sctrl;

	mutex_lock(&usbsw->mutex);
	disable_irq(client->irq);

	read_reg(client, REG_INT1_MASK, &sintm1);
	read_reg(client, REG_INT2_MASK, &sintm2);
	read_reg(client, REG_MANSW1, &smansw1);
	read_reg(client, REG_CTRL, &sctrl);

	write_reg(client, REG_RESET, IC_RESET);
	msleep(20);

	write_reg(client, REG_INT1_MASK, sintm1);
	write_reg(client, REG_INT2_MASK, sintm2);
	write_reg(client, REG_MANSW1, smansw1);
	write_reg(client, REG_CTRL, sctrl);

	dev_info(&client->dev, "sm5504 was reset!\n");

	enable_irq(client->irq);
	mutex_unlock(&usbsw->mutex);

	return 0;
}
#endif

void muic_attached_accessory_inquire(void)
{
	struct sm5504_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	dev_info(&client->dev, "%s\n", __func__);
	blocking_notifier_call_chain(&usb_switch_notifier, first_acce, NULL);
}
EXPORT_SYMBOL_GPL(muic_attached_accessory_inquire);

int get_jig_state(void)
{
	struct sm5504_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;
	int ret;

	switch (attached_dev) {
	case CABLE_NONE_MUIC:
	case CABLE_DESKTOP_DOCK_MUIC:
	case CABLE_SDP_MUIC:
	case CABLE_CARKIT_T1_MUIC:
	case CABLE_DESKTOP_DOCK_VB_MUIC:
	case CABLE_DCP_MUIC:
	case CABLE_CDP_MUIC:
	case CABLE_UART_MUIC:
	case CABLE_UNKNOWN:
	case CABLE_OTG_MUIC:
		ret = 0;
		break;
	case CABLE_JIG_UART_OFF_MUIC:
	case CABLE_JIG_UART_OFF_VB_MUIC:
	case CABLE_JIG_UART_ON_MUIC:
	case CABLE_JIG_UART_ON_VB_MUIC:
	case CABLE_JIG_USB_ON_MUIC:
	case CABLE_JIG_USB_OFF_MUIC:
		ret = 1;
		break;
	default:
		ret = -1;
	}
	dev_info(&client->dev, "%s attached_dev:%d, ret: %d\n",
						__func__, attached_dev, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(get_jig_state);

static irqreturn_t microusb_irq_handler(int irq, void *data)
{
	struct sm5504_usbsw *usbsw = data;

	dev_info(&usbsw->client->dev, "%s\n", __func__);
	schedule_work(&usbsw->work);

	return IRQ_HANDLED;
}

static int sm5504_reg_init(struct sm5504_usbsw *usbsw)
{
	int i, ret;
	struct i2c_client *client = usbsw->client;
	struct ic_vendor muic_list[] = {
		{0x01, "SM5504"},
	};

	ret = read_reg(client, REG_DEVID, &usbsw->id);
	if (ret < 0)
		return ret;
	for (i = 0; i < ARRAY_SIZE(muic_list); i++) {
		if (usbsw->id == muic_list[i].id)
			dev_info(&client->dev, "PartNum : %s\n",
				 muic_list[i].part_num);
	}

	/* INT MASK1, 2 */
	ret = write_reg(client, REG_INT1_MASK, INTMASK1_INIT);
	if (ret < 0)
		return ret;
	ret = write_reg(client, REG_INT2_MASK, INTMASK2_INIT);
	if (ret < 0)
		return ret;
	/* CONTROL */
	ret = write_reg(client, REG_CTRL, CTRL_INIT);
	if (ret < 0)
		return ret;

	return 0;
}

static void detect_dev_sm5504(struct sm5504_usbsw *usbsw, u8 intr1, u8 intr2,
			      void *data)
{
	struct sm5504_platform_data *pdata = usbsw->pdata;
	struct i2c_client *client = usbsw->client;
	u8 val1, val2, adc, chg_type;
	int battery = 0;
	int dev_classifi = CABLE_UNKNOWN;

	read_reg(client, REG_DEV_T1, &val1);
	read_reg(client, REG_DEV_T2, &val2);
	read_reg(client, REG_ADC, &adc);
	read_reg(client, REG_CHG_TYPE, &chg_type);

	if (probing == 1)
		read_reg(client, REG_INT2, &intr2);

	if (intr1 & (ATTACH | CONNECT)) {
		/* Attached */
		if (val1 & DEV_DCP && chg_type & DCP) {
			/* TA */
			dev_classifi = CABLE_DCP_MUIC;
			dev_info(&client->dev, "DCP ATTACHED*****\n");
		}
		if (val1 & DEV_CDP && chg_type & CDP) {
			/* TA */
			dev_classifi = CABLE_CDP_MUIC;
			dev_info(&client->dev, "CDP ATTACHED*****\n");
		}
		if (val1 & DEV_SDP && chg_type & SDP) {
			/* USB */
			dev_classifi = CABLE_SDP_MUIC;
			dev_info(&client->dev, "SDP ATTACHED*****\n");
		}
		if (val1 & DEV_OTG) {
			dev_classifi = CABLE_OTG_MUIC;
			dev_info(&client->dev, "OTG ATTACHED*****\n");
		}
		if (val1 & DEV_UART) {
			dev_classifi = CABLE_UART_MUIC;
			dev_info(&client->dev, "UART ATTACHED*****\n");
		}
		if (val1 & DEV_CARKIT_T1) {
			manual_path_to_usb(1);
			dev_classifi = CABLE_CARKIT_T1_MUIC;
			dev_info(&client->dev,
				 "CARKIT or L USB Cable ATTACHED*****\n");
		}
		if (val2 & DEV_JIG_UART_OFF) {
			if (intr2 & UVLO) {
				dev_classifi = CABLE_JIG_UART_OFF_MUIC;
				dev_info(&client->dev,
					 "JIG_UARTOFF ATTACHED*****\n");
			} else {
				dev_classifi = CABLE_JIG_UART_OFF_VB_MUIC;
				dev_info(&client->dev,
					 "JIG_UARTOFF + VBUS ATTACHED*****\n");
			}
			dev_info(&client->dev, "++ UVLO interrupt on*****\n");
			additional_uvlo_int(1);
			dev_info(&client->dev, "-- UVLO interrupt on*****\n");
#ifndef CONFIG_SEC_FACTORY
			battery = is_battery_connected();
			if (battery > 0 && en_uart == 0) {
				dev_info(&client->dev, "++ Manually cut off*****\n");
				cutoff_path(1);
				dev_info(&client->dev, "-- Manually cut off*****\n");
			}
#endif
		}
		if (val2 & DEV_JIG_UART_ON) {
			if (intr2 & UVLO) {
				dev_classifi = CABLE_JIG_UART_ON_MUIC;
				dev_info(&client->dev,
					 "JIG_UARTON ATTACHED*****\n");
			} else {
				dev_classifi = CABLE_JIG_UART_ON_VB_MUIC;
				dev_info(&client->dev,
					 "JIG_UARTON + VBUS ATTACHED*****\n");
			}
			dev_info(&client->dev, "++ UVLO interrupt on*****\n");
			additional_uvlo_int(1);
			dev_info(&client->dev, "-- UVLO interrupt on*****\n");
#ifndef CONFIG_SEC_FACTORY
			battery = is_battery_connected();
			if (battery > 0 && en_uart == 0) {
				dev_info(&client->dev, "++ Manually cut off*****\n");
				cutoff_path(1);
				dev_info(&client->dev, "-- Manually cut off*****\n");
			}
#endif
		}
		if (val2 & DEV_JIG_USB_OFF) {
			dev_classifi = CABLE_JIG_USB_OFF_MUIC;
			dev_info(&client->dev, "JIG_USB_OFF ATTACHED*****\n");
		}
		if (val2 & DEV_JIG_USB_ON) {
			dev_classifi = CABLE_JIG_USB_ON_MUIC;
			dev_info(&client->dev, "JIG_USB_ON ATTACHED*****\n");
		}
		if (val2 & DEV_JIG_WAKEUP) {
			if (!jig_wakelock_acq) {
				__pm_stay_awake(&jig_suspend_wake);
				pm_qos_update_request(&usbsw->qos_idle,
						      pdata->qos_val);
				jig_wakelock_acq = 1;
				dev_info(&client->dev,
					 "AP WakeLock for FactoryTest *****\n");
			}
		}
		if (val2 & DEV_UNKNOWN && adc == 0x1A) {
			if (intr2 & UVLO) {
				dev_classifi = CABLE_DESKTOP_DOCK_MUIC;
				dev_info(&client->dev, "DESKTOP DOCK ATTACHED*****\n");
			} else {
				dev_classifi = CABLE_DESKTOP_DOCK_VB_MUIC;
				dev_info(&client->dev, "DESKTOP DOCK + VBUS ATTACHED*****\n");
			}
			dev_info(&client->dev, "++ UVLO interrupt on*****\n");
			additional_uvlo_int(1);
			dev_info(&client->dev, "-- UVLO interrupt on*****\n");
		}
		if (pdata->charger_cb)
			pdata->charger_cb(dev_classifi);
		if (probing == 1)
			*(int *)data = dev_classifi;
		blocking_notifier_call_chain(&usb_switch_notifier, dev_classifi,
					     NULL);
		usbsw->dev1 = val1;
		usbsw->dev2 = val2;
		usbsw->adc = adc;
		usbsw->chg_type = chg_type;
		usbsw->intr1 = intr1;
		usbsw->intr2 = intr2;
		attached_dev = dev_classifi;
	}

	if (intr1 & DETACH) {
		/* Detached */
		if (usbsw->dev1 & DEV_DCP && usbsw->chg_type & DCP) {
			dev_info(&client->dev, "DCP DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_CDP && usbsw->chg_type & CDP) {
			dev_info(&client->dev, "CDP DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_SDP && usbsw->chg_type & SDP) {
			dev_info(&client->dev, "SDP DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_OTG) {
			dev_info(&client->dev, "OTG DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_UART) {
			dev_info(&client->dev, "UART DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_CARKIT_T1) {
			manual_path_to_usb(0);
			dev_info(&client->dev,
				 "CARKIT or L USB Cable DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_JIG_UART_OFF) {
			if (usbsw->intr2 & UVLO) {
				dev_info(&client->dev,
					 "JIG_UARTOFF DETACHED*****\n");
			} else {
				dev_info(&client->dev,
					 "JIG_UARTOFF + VBUS DETACHED*****\n");
			}
			dev_info(&client->dev, "++ UVLO interrupt off*****\n");
			additional_uvlo_int(0);
			dev_info(&client->dev, "-- UVLO interrupt off*****\n");
#ifndef CONFIG_SEC_FACTORY
			dev_info(&client->dev, "++ Automatic switching*****\n");
			cutoff_path(0);
			dev_info(&client->dev, "-- Automatic switching*****\n");
#endif
		}
		if (usbsw->dev2 & DEV_JIG_UART_ON) {
			if (usbsw->intr2 & UVLO) {
				dev_info(&client->dev,
					 "JIG_UARTON DETACHED*****\n");
			} else {
				dev_info(&client->dev,
					 "JIG_UARTON + VBUS DETACHED*****\n");
			}
			dev_info(&client->dev, "++ UVLO interrupt off*****\n");
			additional_uvlo_int(0);
			dev_info(&client->dev, "-- UVLO interrupt off*****\n");
#ifndef CONFIG_SEC_FACTORY
			dev_info(&client->dev, "++ Automatic switching*****\n");
			cutoff_path(0);
			dev_info(&client->dev, "-- Automatic switching*****\n");
#endif
		}
		if (usbsw->dev2 & DEV_JIG_USB_OFF) {
			dev_info(&client->dev, "JIG_USB_OFF DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_JIG_USB_ON) {
			dev_info(&client->dev, "JIG_USB_ON DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_JIG_WAKEUP) {
			if (jig_wakelock_acq) {
				__pm_relax(&jig_suspend_wake);
				pm_qos_update_request(&usbsw->qos_idle,
						      PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);

				jig_wakelock_acq = 0;
				dev_info(&client->dev,
					 "AP WakeLock Release *****\n");
			}
		}
		if (usbsw->dev2 & DEV_UNKNOWN && usbsw->adc == 0x1A) {
			if (usbsw->intr2 & UVLO) {
				dev_info(&client->dev, "DESKTOP DOCK DETACHED*****\n");
			} else {
				dev_info(&client->dev, "DESKTOP DOCK + VBUS DETACHED*****\n");
			}
			dev_info(&client->dev, "++ UVLO interrupt off*****\n");
			additional_uvlo_int(0);
			dev_info(&client->dev, "-- UVLO interrupt off*****\n");
		}
		if (pdata->charger_cb)
			pdata->charger_cb(CABLE_NONE_MUIC);
		blocking_notifier_call_chain(&usb_switch_notifier,
					     CABLE_NONE_MUIC, NULL);
		attached_dev = CABLE_NONE_MUIC;
	}

	/*
	 * SM5504 doesn't support DESKTOP-DOCK officially
	 * So, it doesn't report the attaching event in case of VBUS IN&OUT to DESKTOP-DOCK
	 */
	if (intr1 == 0x0) {
		if (val2 & DEV_UNKNOWN && adc == 0x1A) {
			if (intr2 & UVLO) {
				dev_classifi = CABLE_DESKTOP_DOCK_MUIC;
				dev_info(&client->dev, "DESKTOP DOCK ATTACHED*****\n");
			} else {
				dev_classifi = CABLE_DESKTOP_DOCK_VB_MUIC;
				dev_info(&client->dev, "DESKTOP DOCK + VBUS ATTACHED*****\n");
			}
			dev_info(&client->dev, "++ UVLO interrupt on*****\n");
			additional_uvlo_int(1);
			dev_info(&client->dev, "-- UVLO interrupt on*****\n");
		}
		if (pdata->charger_cb)
			pdata->charger_cb(dev_classifi);
		if (probing == 1)
			*(int *)data = dev_classifi;
		blocking_notifier_call_chain(&usb_switch_notifier, dev_classifi,
					     NULL);
		usbsw->dev1 = val1;
		usbsw->dev2 = val2;
		usbsw->adc = adc;
		usbsw->chg_type = chg_type;
		usbsw->intr1 = intr1;
		usbsw->intr2 = intr2;
	}

	if (intr2 & (OCP | OCP_LATCH | OVP_FET)) {
		if (intr2 & OCP)
			dev_info(&client->dev, "OCP *****\n");
		if (intr2 & OCP_LATCH)
			dev_info(&client->dev, "OCP_LATCH *****\n");
		if (intr2 & OVP_FET)
			dev_info(&client->dev, "OVP_FET *****\n");
	}

	return;
}

static void sm5504_work_cb(struct work_struct *work)
{
	u8 intr1, intr2, val1;

	struct sm5504_usbsw *usbsw =
	    container_of(work, struct sm5504_usbsw, work);
	struct i2c_client *client = usbsw->client;

	msleep(50);

	mutex_lock(&usbsw->mutex);
	disable_irq(client->irq);

	/* Read and Clear Interrupt1/2 */
	retry_read_reg(client, REG_INT1, &intr1);
	retry_read_reg(client, REG_INT2, &intr2);

	retry_read_reg (client, REG_CTRL, &val1);
	if (val1 == RESET_DEFAULT)
		sm5504_reg_init(usbsw);

	detect_dev_sm5504(usbsw, intr1, intr2, NULL);

	enable_irq(client->irq);
	mutex_unlock(&usbsw->mutex);
}

static int sm5504_int_init(struct device_node *np, struct sm5504_usbsw *usbsw)
{
	struct i2c_client *client = usbsw->client;
	int ret, irq;
	u8 intr1, intr2;

	INIT_WORK(&usbsw->work, sm5504_work_cb);

	irq = of_get_named_gpio(np, "connector-gpio", 0);
	if (irq < 0) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__, irq);
		return irq;
	}

	client->irq = gpio_to_irq(irq);
	ret =
	    request_irq(client->irq, microusb_irq_handler,
			IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING,
			"sm5504 micro USB", usbsw);
	if (ret) {
		dev_err(&client->dev, "Unable to get IRQ %d\n", client->irq);
		return ret;
	}

	/* Read and Clear INTERRUPT1,2 REGS */
	mutex_lock(&usbsw->mutex);
	disable_irq(client->irq);
	read_reg(client, REG_INT1, &intr1);
	read_reg(client, REG_INT2, &intr2);
	enable_irq(client->irq);
	mutex_unlock(&usbsw->mutex);

	return 0;
}

static const struct of_device_id sm5504_dt_ids[] = {
	{.compatible = "samsung,sm5504",},
	{}
};

MODULE_DEVICE_TABLE(of, sec_charger_dt_ids);

static int sm5504_probe_dt(struct device_node *np,
			   struct device *dev,
			   struct sm5504_platform_data *pdata)
{
	int ret = 0;
	int connector_gpio = 0;
	const struct of_device_id *match;
	u32 lpm;

	if (!np)
		return -EINVAL;

	match = of_match_device(sm5504_dt_ids, dev);
	if (!match)
		return -EINVAL;

	connector_gpio = of_get_named_gpio(np, "connector-gpio", 0);
	if (connector_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
		       connector_gpio);
		return connector_gpio;
	}
	ret = gpio_request(connector_gpio, "connector-gpio");
	if (ret) {
		pr_err("%s:%d gpio_request failed: connector_gpio %d\n",
		       __func__, __LINE__, connector_gpio);
		return ret;
	}

	gpio_direction_input(connector_gpio);

	ret = request_mfp_edge_wakeup(connector_gpio, NULL, pdata, dev);
	if (ret)
		pr_err("%s: failed to request edge wakeup.\n", __func__);

	if (!of_property_read_u32(np, "lpm-qos", &lpm))
		pdata->qos_val = lpm;
	else {
		pr_err("sm5504: failed to get 'lpm-qos' from dt\n");
		return -EINVAL;
	}

	return 0;
}

static struct sm5504_platform_data sm5504_info = {
	.charger_cb = sec_charger_cb,
};

static int sm5504_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sm5504_usbsw *usbsw;
	struct device *switch_dev;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct sm5504_platform_data *pdata = client->dev.platform_data;
	struct device_node *np = client->dev.of_node;
	int ret = 0;

	dev_info(&client->dev, "probe start\n");
	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata)
			pdata = &sm5504_info;

		ret = sm5504_probe_dt(np, &client->dev, pdata);
		if (ret)
			return ret;
	} else if (!pdata) {
		dev_err(&client->dev, "%s: no platform data defined\n",
			__func__);
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Not compatible i2c function\n");
		return -EIO;
	}
	/* For AT Command FactoryTest */
	wakeup_source_init(&jig_suspend_wake, "JIG_UART Connect suspend wake");

	usbsw = kzalloc(sizeof(struct sm5504_usbsw), GFP_KERNEL);
	if (!usbsw) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	chip = usbsw;
	usbsw->client = client;
	usbsw->pdata = pdata;

	i2c_set_clientdata(client, usbsw);

	mutex_init(&usbsw->mutex);

	/* DeskTop Dock  */
	usbsw->dock_dev.name = "dock";
	ret = switch_dev_register(&usbsw->dock_dev);
	if (ret < 0)
		dev_err(&client->dev, "dock_dev_register error !!\n");

	if (!sec_class) {
		sec_class = class_create(THIS_MODULE, "sec");
	}
	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");
	if (device_create_file(switch_dev, &dev_attr_adc) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_adc.attr.name);
	if (device_create_file(switch_dev, &dev_attr_usb_state) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_usb_state.attr.name);
	if (device_create_file(switch_dev, &dev_attr_usb_sel) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_usb_sel.attr.name);
	if (device_create_file(switch_dev, &dev_attr_uart_sel) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_uart_sel.attr.name);
	if (device_create_file(switch_dev, &dev_attr_syssleep) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_syssleep.attr.name);
	if (device_create_file(switch_dev, &dev_attr_attached_dev) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_attached_dev.attr.name);
	dev_set_drvdata(switch_dev, usbsw);

	usbsw->qos_idle.name = "Jig driver";
	pm_qos_add_request(&usbsw->qos_idle, PM_QOS_CPUIDLE_BLOCK,
			   PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);

	ret = sm5504_reg_init(usbsw);
	if (ret)
		goto sm5504_probe_fail;

	/* device detection */
	dev_info(&client->dev, "First Detection\n");
	detect_dev_sm5504(usbsw, ATTACH, 0, &first_acce);

	ret = sm5504_int_init(np, usbsw);
	if (ret)
		goto sm5504_probe_fail;

	probing = 0;
	dev_info(&client->dev, "PROBE Done.\n");

	return 0;

sm5504_probe_fail:
	device_destroy(sec_class, 0);
	i2c_set_clientdata(client, NULL);
	kfree(usbsw);
	return ret;
}

static int sm5504_resume(struct i2c_client *client)
{
	struct sm5504_usbsw *usbsw = chip;
	u8 val1;

	dev_info(&client->dev, "sm5504_resume\n");

	retry_read_reg (client, REG_CTRL, &val1);
	if ( val1 == RESET_DEFAULT)
		sm5504_reg_init(usbsw);

	return 0;
}

static const struct i2c_device_id sm5504_id[] = {
	{"sm5504", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sm5504_id);

static struct i2c_driver sm5504_i2c_driver = {
	.driver = {
		   .name = "sm5504",
		   .of_match_table = sm5504_dt_ids,
		   },
	.probe = sm5504_probe,
	.resume = sm5504_resume,
	.id_table = sm5504_id,
};

static int __init en_uart_setup(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	en_uart = n;
	printk(KERN_INFO "en_uart = %d\n", en_uart);

	return 1;
}
__setup("uart_swc_en=", en_uart_setup);

static int __init sm5504_init(void)
{
	pr_info("%s\n", __func__);
	return i2c_add_driver(&sm5504_i2c_driver);
}

module_init(sm5504_init);

MODULE_LICENSE("GPL");
