/*
 * Copyright (C) 2011 Samsung Electronics Co. Ltd.
 *  Inchul Im <inchul.im@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#define pr_fmt(fmt) "usb_notifier: " fmt

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/usb_notify.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif
#if defined (CONFIG_EXTCON_SM5504) || defined (CONFIG_EXTCON_SM5502)
#include <linux/extcon/muic-sm550x.h>
#endif
#include <linux/platform_data/mv_usb.h>
#include <linux/battery/sec_charger.h>
struct usb_state_t {
	int pre_cable;
	bool b_host;
	bool b_gadget;
} usb_state;

struct usb_notifier_platform_data {
	struct	notifier_block usb_nb;
	bool	gpio_redriver_en;
};

static int sec_cable_notifier(struct notifier_block *nb,
		unsigned long cable_type, void *v)
{
	bool cable_state = 0;
	struct otg_notify *o_notify;

	o_notify = get_otg_notify();

	if ((cable_type == CABLE_NONE_MUIC)
		||(cable_type == CABLE_TYPE_NONE_MUIC))
		cable_state = 0;
	else
		cable_state = 1;

	pr_info("%s %d is %s\n", __func__, (u8)cable_type,
			cable_state ? "attached" : "detached");

	switch (cable_type) {
	case CABLE_SDP_MUIC:
	case CABLE_CDP_MUIC:
	case CABLE_JIG_USB_ON_MUIC:
	case CABLE_JIG_USB_OFF_MUIC:
	case CABLE_TYPE1_USB_MUIC:
	case CABLE_TYPE2_JIG_USB_ON_MUIC:
	case CABLE_TYPE2_JIG_USB_OFF_MUIC:
	case CABLE_TYPE3_NONSTD_SDP_MUIC:
	case CABLE_CARKIT_T1_MUIC:
		send_otg_notify(o_notify, NOTIFY_EVENT_VBUS,
			cable_state);
		usb_state.pre_cable = NOTIFY_EVENT_VBUS;
		break;
	case CABLE_OTG_MUIC:
	case CABLE_TYPE1_OTG_MUIC:
		send_otg_notify(o_notify, NOTIFY_EVENT_HOST,
			cable_state);
		usb_state.pre_cable = NOTIFY_EVENT_HOST;
		break;
	case CABLE_TYPE_SMARTDOCK_TA_MUIC:
		send_otg_notify(o_notify, NOTIFY_EVENT_SMARTDOCK_TA,
			cable_state);
		usb_state.pre_cable = NOTIFY_EVENT_SMARTDOCK_TA;
		break;
	case CABLE_TYPE_SMARTDOCK_USB_MUIC:
		send_otg_notify(o_notify, NOTIFY_EVENT_SMARTDOCK_USB,
			cable_state);
		usb_state.pre_cable = NOTIFY_EVENT_SMARTDOCK_USB;
		break;
	case CABLE_TYPE_AUDIODOCK_MUIC:
		send_otg_notify(o_notify, NOTIFY_EVENT_AUDIODOCK,
			cable_state);
		usb_state.pre_cable = NOTIFY_EVENT_AUDIODOCK;
		break;
	case CABLE_NONE_MUIC:
	case CABLE_TYPE_NONE_MUIC:
		send_otg_notify(o_notify, usb_state.pre_cable,
			cable_state);
		usb_state.pre_cable = NOTIFY_EVENT_NONE;
		break;
	default:
		break;
	}
	return 1;

}
#ifdef CONFIG_OF
static void of_get_usb_redriver_dt(struct device_node *np,
		struct usb_notifier_platform_data *pdata)
{
	int g_count = 0;

	g_count = of_gpio_named_count(np, "gpios_redriver_en");
	pr_info("usb: %s, gpios_redriver_en count %d\n", __func__, g_count);

	if (g_count > 0) {
		pdata->gpio_redriver_en = of_get_named_gpio(np,
			"gpios_redriver_en", 0);
		if (!gpio_is_valid(pdata->gpio_redriver_en))
			pr_err("%s:usb30_redriver_en:Invalied gpio pins\n",
				__func__);
	} else
		pdata->gpio_redriver_en = -1;
	return;
}

static int of_usb_notifier_dt(struct device *dev,
		struct usb_notifier_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	if (!np)
		return -EINVAL;

	of_get_usb_redriver_dt(np, pdata);
	return 0;
}
#endif

static int otg_accessory_power(bool enable)
{
	union power_supply_propval value;
	u8 on = (u8)!!enable;
	struct power_supply *psy = power_supply_get_by_name("sec-charger");

	if(on)
		value.intval = 1;
	else
		value.intval = 0;

	psy->set_property(psy, POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL, &value);

	pr_info("usb notifier: otg accessory power = %d\n", on);

	return 0;
}
static int set_online(int event, int state)
{
	union power_supply_propval value;
	struct power_supply *psy = power_supply_get_by_name("sec-charger");

	if (state)
		value.intval = POWER_SUPPLY_TYPE_OTG;
	else
		value.intval = POWER_SUPPLY_TYPE_BATTERY;

	psy->set_property(psy, POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL, &value);

	if (event == NOTIFY_EVENT_SMTD_EXT_CURRENT) {
		pr_info("usb_notifier: request smartdock charging current = %s\n",
			state ? "1000mA" : "1700mA");
	} else if (event == NOTIFY_EVENT_MMD_EXT_CURRENT) {
		pr_info("usb_notifier: request mmdock charging current = %s\n",
			state ? "900mA" : "1400mA");
	}

	return 0;
}


static int mv_set_peripheral(bool enable)
{
	pr_info("%s - %s\n", __func__, enable ? "ATTACHED":"DETACHED");
	usb_state.b_gadget = !!enable;
	pxa_usb_notify(PXA_USB_DEV_OTG, EVENT_VBUS, 0);
	return 0;
}

static int mv_get_vbus(unsigned int *vbus)
{
	*vbus = !!usb_state.b_gadget;
	return 0;
}
#ifdef CONFIG_USB_HOST_NOTIFY
static int mv_set_host(bool enable)
{
	pr_info("%s - %s\n", __func__, enable ? "ATTACHED":"DETACHED");
	usb_state.b_host = !!enable;
	pxa_usb_notify(PXA_USB_DEV_OTG, EVENT_ID, 0);
	return 0;
}
#endif

static void mv_set_idpin(unsigned int *id)
{
	*id = (u8)!usb_state.b_host;
	return;
}

static int mv_init_idpin(void)
{
	usb_state.b_host = 0;
	return (u8)!usb_state.b_host;
}
static struct otg_notify set_mv_notify = {
	.vbus_drive	= otg_accessory_power,
#ifdef CONFIG_USB_HOST_NOTIFY
	.set_host = mv_set_host,
#else
	.unsupport_host = 1,
#endif
	.set_peripheral = mv_set_peripheral,
	.vbus_detect_gpio = -1,
	.redriver_en_gpio = -1,
	.is_wakelock = 1,
	.set_battcall = set_online,
};



static int usb_notifier_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct usb_notifier_platform_data *pdata = NULL;

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct usb_notifier_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&pdev->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = of_usb_notifier_dt(&pdev->dev, pdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to get device of_node\n");
			return ret;
		}

		pdev->dev.platform_data = pdata;
	} else
		pdata = pdev->dev.platform_data;

	set_otg_notify(&set_mv_notify);
	set_notify_data(&set_mv_notify, pdata);

	pdata->usb_nb.notifier_call = sec_cable_notifier;
	usb_switch_register_notify(&pdata->usb_nb);
	pxa_usb_set_extern_call(PXA_USB_DEV_OTG, vbus, get_vbus,
				mv_get_vbus);
	pxa_usb_set_extern_call(PXA_USB_DEV_OTG, idpin, get_idpin,
			mv_set_idpin);
#ifdef CONFIG_USB_HOST_NOTIFY
	pxa_usb_set_extern_call(PXA_USB_DEV_OTG, vbus, set_vbus,
			otg_accessory_power);
#endif
	pxa_usb_set_extern_call(PXA_USB_DEV_OTG, idpin, init,
				mv_init_idpin);
	pr_info("%s done\n", __func__);
	return 0;
}

static int usb_notifier_remove(struct platform_device *pdev)
{
	struct usb_notifier_platform_data *pdata = pdev->dev.platform_data;
	usb_switch_unregister_notify(&pdata->usb_nb);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id usb_notifier_dt_ids[] = {
	{ .compatible = "samsung,usb-notifier",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, usb_notifier_dt_ids);
#endif

static struct platform_driver usb_notifier_driver = {
	.probe		= usb_notifier_probe,
	.remove		= usb_notifier_remove,
	.driver		= {
		.name	= "usb_notifier",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(usb_notifier_dt_ids),
#endif
	},
};

static int __init usb_notifier_init(void)
{
	pr_info("%s\n", __func__);
	return platform_driver_register(&usb_notifier_driver);
}

static void __init usb_notifier_exit(void)
{
	platform_driver_unregister(&usb_notifier_driver);
}

module_init(usb_notifier_init);
module_exit(usb_notifier_exit);

MODULE_AUTHOR("Jaejin Lee <jjinn.lee@samsung.com>");
MODULE_DESCRIPTION("USB notifier mv");
MODULE_LICENSE("GPL");
