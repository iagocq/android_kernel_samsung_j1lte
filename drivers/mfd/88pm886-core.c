/*
 * Base driver for Marvell 88PM886
 *
 * Copyright (C) 2014 Marvell International Ltd.
 *  Yi Zhang <yizhang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/mfd/core.h>
#include <linux/mfd/88pm886.h>
#include <linux/regulator/machine.h>
#ifdef CONFIG_SEC_DEBUG
#include <linux/sec-common.h>
#endif

#define PM886_POWER_UP_LOG		(0x17)
#define PM886_LOWPOWER1			(0x20)
#define PM886_LOWPOWER2			(0x21)
#define PM886_LOWPOWER3			(0x22)
#define PM886_BK_OSC_CTRL1		(0x50)
#define PM886_BK_OSC_CTRL6		(0x55)
#define PM886_POWER_DOWN_LOG1		(0xe5)
#define PM886_POWER_DOWN_LOG2		(0xe6)
#define PM886_SW_PDOWN			(1 << 5)
#define PM886_CHGBK_CONFIG6		(0x50)
#define PM886_HVSEL_CONFIG1		(0x53)

#define CELL_IRQ_RESOURCE(_name, _irq) { \
	.name = _name, \
	.start = _irq, .end = _irq, \
	.flags = IORESOURCE_IRQ, \
	}
#define CELL_DEV(_name, _r, _compatible, _id) { \
	.name = _name, \
	.of_compatible = _compatible, \
	.num_resources = ARRAY_SIZE(_r), \
	.resources = _r, \
	.id = _id, \
	}

/* don't export it at present */
static struct pm886_chip *pm886_chip_priv;

extern struct regmap *get_88pm860_codec_regmap(void);
struct regmap *companion_base_page;

#if defined(CONFIG_SEC_DEBUG)
static u8 power_on_reason;
unsigned char pm80x_get_power_on_reason(void)
{
	return power_on_reason;
}
#endif

static bool ta_connected = 0;

struct regmap *get_companion(void)
{
	return companion_base_page;
}
EXPORT_SYMBOL(get_companion);

struct regmap *get_codec_companion(void)
{
	return get_88pm860_codec_regmap();
}
EXPORT_SYMBOL(get_codec_companion);

static const struct resource onkey_resources[] = {
	CELL_IRQ_RESOURCE(PM886_ONKEY_NAME, PM886_IRQ_ONKEY),
};

static const struct resource rtc_resources[] = {
	CELL_IRQ_RESOURCE(PM886_RTC_NAME, PM886_IRQ_RTC),
};

static const struct resource charger_resources[] = {
	CELL_IRQ_RESOURCE("88pm886-chg-fail", PM886_IRQ_CHG_FAIL),
	CELL_IRQ_RESOURCE("88pm886-chg-done", PM886_IRQ_CHG_DONE),
};

static const struct resource battery_resources[] = {
	CELL_IRQ_RESOURCE("88pm886-bat-cc", PM886_IRQ_CC),
	CELL_IRQ_RESOURCE("88pm886-bat-volt", PM886_IRQ_VBAT),
};

static const struct resource headset_resources[] = {
	CELL_IRQ_RESOURCE("88pm886-headset-det", PM886_IRQ_HS_DET),
	CELL_IRQ_RESOURCE("88pm886-mic-det", PM886_IRQ_MIC_DET),
};

static const struct resource vbus_resources[] = {
	CELL_IRQ_RESOURCE("88pm886-chg-det", PM886_IRQ_CHG_GOOD),
	CELL_IRQ_RESOURCE("88pm886-gpadc0", PM886_IRQ_GPADC0),
	CELL_IRQ_RESOURCE("88pm886-gpadc1", PM886_IRQ_GPADC1),
	CELL_IRQ_RESOURCE("88pm886-gpadc2", PM886_IRQ_GPADC2),
	CELL_IRQ_RESOURCE("88pm886-gpadc3", PM886_IRQ_GPADC3),
	CELL_IRQ_RESOURCE("88pm886-otg-fail", PM886_IRQ_OTG_FAIL),
};

static const struct resource leds_resources[] = {
	CELL_IRQ_RESOURCE("88pm886-cfd-fail", PM886_IRQ_CFD_FAIL),
};

static const struct resource regulator_resources[] = {
	{
	.name = PM886_REGULATOR_NAME,
	},
};

static const struct resource dvc_resources[] = {
	{
	.name = PM886_DVC_NAME,
	},
};

static const struct resource rgb_resources[] = {
	{
	.name = PM886_RGB_NAME,
	},
};

static const struct resource debugfs_resources[] = {
	{
	.name = PM886_DEBUGFS_NAME,
	},
};

static const struct resource gpadc_resources[] = {
	{
	.name = PM886_GPADC_NAME,
	},
};

static const struct mfd_cell pm886_cell_devs[] = {
	CELL_DEV(PM886_RTC_NAME, rtc_resources, "marvell,88pm886-rtc", -1),
	CELL_DEV(PM886_ONKEY_NAME, onkey_resources, "marvell,88pm886-onkey", -1),
	CELL_DEV(PM886_CHARGER_NAME, charger_resources, "marvell,88pm886-charger", -1),
	CELL_DEV(PM886_BATTERY_NAME, battery_resources, "marvell,88pm886-battery", -1),
	CELL_DEV(PM886_HEADSET_NAME, headset_resources, "marvell,88pm886-headset", -1),
	CELL_DEV(PM886_VBUS_NAME, vbus_resources, "marvell,88pm886-vbus", -1),
	CELL_DEV(PM886_CFD_NAME, leds_resources, "marvell,88pm886-leds", PM886_FLASH_LED),
	CELL_DEV(PM886_CFD_NAME, leds_resources, "marvell,88pm886-leds", PM886_TORCH_LED),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-buck1", 0),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-buck2", 1),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-buck3", 2),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-buck4", 3),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-buck5", 4),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo1", 5),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo2", 6),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo3", 7),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo4", 8),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo5", 9),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo6", 10),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo7", 11),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo8", 12),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo9", 13),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo10", 14),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo11", 15),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo12", 16),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo13", 17),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo14", 18),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo15", 19),
	CELL_DEV(PM886_REGULATOR_NAME, regulator_resources, "marvell,88pm886-ldo16", 20),
	CELL_DEV(PM886_DVC_NAME, dvc_resources, "marvell,88pm886-dvc", -1),
	CELL_DEV(PM886_RGB_NAME, rgb_resources, "marvell,88pm886-rgb0", PM886_RGB_LED0),
	CELL_DEV(PM886_RGB_NAME, rgb_resources, "marvell,88pm886-rgb1", PM886_RGB_LED1),
	CELL_DEV(PM886_RGB_NAME, rgb_resources, "marvell,88pm886-rgb2", PM886_RGB_LED2),
	CELL_DEV(PM886_DEBUGFS_NAME, debugfs_resources, "marvell,88pm886-debugfs", -1),
	CELL_DEV(PM886_GPADC_NAME, gpadc_resources, "marvell,88pm886-gpadc", -1),
};

const struct of_device_id pm886_of_match[] = {
	{ .compatible = "marvell,88pm886", .data = (void *)PM886 },
	{},
};
EXPORT_SYMBOL_GPL(pm886_of_match);

static bool pm886_base_readable_reg(struct device *dev, unsigned int reg)
{
	bool ret = false;

	switch (reg) {
	case 0x00:
	case 0x01:
	case 0x14:
	case 0x15:
	case 0x17:
	case 0x18:
	case 0x19:
	case 0x1d:
	case 0x25:
	case 0x36:
	case 0x6f:
		ret = true;
		break;
	default:
		break;
	}
	if ((reg >= 0x05) && (reg <= 0x08))
		ret = true;
	if ((reg >= 0x0a) && (reg <= 0x0d))
		ret = true;
	if ((reg >= 0x1f) && (reg <= 0x23))
		ret = true;
	if ((reg >= 0x30) && (reg <= 0x33))
		ret = true;
	if ((reg >= 0x38) && (reg <= 0x3b))
		ret = true;
	if ((reg >= 0x40) && (reg <= 0x48))
		ret = true;
	if ((reg >= 0x50) && (reg <= 0x5c))
		ret = true;
	if ((reg >= 0x61) && (reg <= 0x6d))
		ret = true;
	if ((reg >= 0xce) && (reg <= 0xef))
		ret = true;

	return ret;
}

static bool pm886_power_readable_reg(struct device *dev, unsigned int reg)
{
	bool ret = false;

	switch (reg) {
	case 0x00:
	case 0x06:
	case 0x16:
	case 0x19:
	case 0x1a:
	case 0x20:
	case 0x21:
	case 0x23:
	case 0x2c:
	case 0x2d:
	case 0x2f:
	case 0x32:
	case 0x33:
	case 0x35:
	case 0x38:
	case 0x39:
	case 0x3b:
	case 0x3e:
	case 0x3f:
	case 0x41:
	case 0x44:
	case 0x45:
	case 0x47:
	case 0x4a:
	case 0x4b:
	case 0x4d:
	case 0x50:
	case 0x51:
	case 0x53:
	case 0x56:
	case 0x57:
	case 0x59:
	case 0x5c:
	case 0x5d:
	case 0x5f:
	case 0x62:
	case 0x63:
	case 0x65:
	case 0x68:
	case 0x69:
	case 0x6b:
	case 0x6e:
	case 0x6f:
	case 0x71:
	case 0x74:
	case 0x75:
	case 0x77:
	case 0x7a:
	case 0x7b:
	case 0x7d:
		ret = true;
		break;
	default:
		break;
	}
	if ((reg >= 0x01) && (reg <= 0x03))
		ret = true;
	if ((reg >= 0x08) && (reg <= 0x0a))
		ret = true;
	if ((reg >= 0x0e) && (reg <= 0x10))
		ret = true;
	if ((reg >= 0x0e) && (reg <= 0x10))
		ret = true;
	if ((reg >= 0x27) && (reg <= 0x29))
		ret = true;

	return ret;
}

static bool pm886_gpadc_readable_reg(struct device *dev, unsigned int reg)
{
	bool ret = false;

	switch (reg) {
	case 0x06:
	case 0x13:
	case 0x14:
	case 0x18:
	case 0x1a:
	case 0x1b:
	case 0x28:
	case 0x2a:
	case 0x2b:
	case 0x38:
	case 0x3d:
	case 0x80:
	case 0x81:
	case 0x90:
	case 0x91:
	case 0xa0:
	case 0xa1:
		ret = true;
		break;
	default:
		break;
	}

	if ((reg >= 0x00) && (reg <= 0x03))
		ret = true;
	if ((reg >= 0x05) && (reg <= 0x08))
		ret = true;
	if ((reg >= 0x0a) && (reg <= 0x0e))
		ret = true;
	if ((reg >= 0x20) && (reg <= 0x26))
		ret = true;
	if ((reg >= 0x30) && (reg <= 0x34))
		ret = true;
	if ((reg >= 0x40) && (reg <= 0x43))
		ret = true;
	if ((reg >= 0x46) && (reg <= 0x5d))
		ret = true;
	if ((reg >= 0x84) && (reg <= 0x8b))
		ret = true;
	if ((reg >= 0x94) && (reg <= 0x9b))
		ret = true;
	if ((reg >= 0xa4) && (reg <= 0xad))
		ret = true;
	if ((reg >= 0xb0) && (reg <= 0xb3))
		ret = true;
	if ((reg >= 0xc0) && (reg <= 0xc7))
		ret = true;

	return ret;
}

static bool pm886_battery_readable_reg(struct device *dev, unsigned int reg)
{
	bool ret = false;

	switch (reg) {
	case 0x47:
	case 0x53:
	case 0x54:
	case 0x58:
	case 0x5b:
	case 0x65:
		ret = true;
		break;
	default:
		break;
	}

	if ((reg >= 0x00) && (reg <= 0x15))
		ret = true;
	if ((reg >= 0x28) && (reg <= 0x31))
		ret = true;
	if ((reg >= 0x34) && (reg <= 0x36))
		ret = true;
	if ((reg >= 0x38) && (reg <= 0x3b))
		ret = true;
	if ((reg >= 0x3e) && (reg <= 0x40))
		ret = true;
	if ((reg >= 0x42) && (reg <= 0x45))
		ret = true;
	if ((reg >= 0x4a) && (reg <= 0x51))
		ret = true;
	if ((reg >= 0x60) && (reg <= 0x63))
		ret = true;
	if ((reg >= 0x6b) && (reg <= 0x71))
		ret = true;

	return ret;
}

static bool pm886_test_readable_reg(struct device *dev, unsigned int reg)
{
	return true;
}

const struct regmap_config pm886_base_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.readable_reg = pm886_base_readable_reg,

	.max_register = PM886_BASE_PAGE_NUMS,
};
EXPORT_SYMBOL_GPL(pm886_base_i2c_regmap);

const struct regmap_config pm886_power_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.readable_reg = pm886_power_readable_reg,

	.max_register = PM886_POWER_PAGE_NUMS,
};
EXPORT_SYMBOL_GPL(pm886_power_i2c_regmap);

const struct regmap_config pm886_gpadc_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.readable_reg = pm886_gpadc_readable_reg,

	.max_register = PM886_GPADC_PAGE_NUMS,
};
EXPORT_SYMBOL_GPL(pm886_gpadc_i2c_regmap);

const struct regmap_config pm886_battery_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.readable_reg = pm886_battery_readable_reg,

	.max_register = PM886_BATTERY_PAGE_NUMS,
};
EXPORT_SYMBOL_GPL(pm886_battery_i2c_regmap);

const struct regmap_config pm886_test_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.readable_reg = pm886_test_readable_reg,

	.max_register = PM886_TEST_PAGE_NUMS,
};
EXPORT_SYMBOL_GPL(pm886_test_i2c_regmap);

void reset_charger_status(int status)
{
	status = (status != POWER_SUPPLY_TYPE_BATTERY);

	if (status)
		ta_connected = true;
	else
		ta_connected = false;

}
EXPORT_SYMBOL(reset_charger_status);

struct pm886_chip *pm886_init_chip(struct i2c_client *client)
{
	struct pm886_chip *chip;

	chip = devm_kzalloc(&client->dev, sizeof(struct pm886_chip), GFP_KERNEL);
	if (!chip)
		return ERR_PTR(-ENOMEM);

	chip->client = client;
	chip->irq = client->irq;
	chip->dev = &client->dev;
	chip->power_page_addr = client->addr + 1;
	chip->gpadc_page_addr = client->addr + 2;
	chip->battery_page_addr = client->addr + 3;
	chip->test_page_addr = client->addr + 7;

	dev_set_drvdata(chip->dev, chip);
	i2c_set_clientdata(chip->client, chip);

	device_init_wakeup(&client->dev, 1);

	return chip;
}

int pm886_parse_dt(struct device_node *np, struct pm886_chip *chip)
{
	if (!chip)
		return -EINVAL;

	chip->irq_mode =
		!of_property_read_bool(np, "marvell,88pm886-irq-write-clear");

	return 0;
}


static void parse_powerup_down_log(struct pm886_chip *chip)
{
	int powerup, powerdown1, powerdown2, bit;
	static const char * const powerup_name[] = {
		"ONKEY_WAKEUP	",
		"CHG_WAKEUP	",
		"EXTON_WAKEUP	",
		"SMPL_WAKEUP	",
		"ALARM_WAKEUP	",
		"FAULT_WAKEUP	",
		"BAT_WAKEUP	",
		"RESERVED	",
	};
	static const char * const powerdown1_name[] = {
		"OVER_TEMP ",
		"UV_VSYS1  ",
		"SW_PDOWN  ",
		"FL_ALARM  ",
		"WD        ",
		"LONG_ONKEY",
		"OV_VSYS   ",
		"RTC_RESET "
	};
	static const char * const powerdown2_name[] = {
		"HYB_DONE   ",
		"UV_VSYS2   ",
		"HW_RESET   ",
		"PGOOD_PDOWN",
		"LONKEY_RTC "
	};

	/*power up log*/
	regmap_read(chip->base_regmap, PM886_POWER_UP_LOG, &powerup);
	dev_info(chip->dev, "powerup log 0x%x: 0x%x\n",
		 PM886_POWER_UP_LOG, powerup);
	dev_info(chip->dev, " -------------------------------\n");
	dev_info(chip->dev, "|     name(power up) |  status  |\n");
	dev_info(chip->dev, "|--------------------|----------|\n");
	for (bit = 0; bit < 7; bit++)
		dev_info(chip->dev, "|  %s  |    %x     |\n",
			powerup_name[bit], (powerup >> bit) & 1);
	dev_info(chip->dev, " -------------------------------\n");

	/*power down log1*/
	regmap_read(chip->base_regmap, PM886_POWER_DOWN_LOG1, &powerdown1);
	dev_info(chip->dev, "PowerDW Log1 0x%x: 0x%x\n",
		PM886_POWER_DOWN_LOG1, powerdown1);
	dev_info(chip->dev, " -------------------------------\n");
	dev_info(chip->dev, "| name(power down1)  |  status  |\n");
	dev_info(chip->dev, "|--------------------|----------|\n");
	for (bit = 0; bit < 8; bit++)
		dev_info(chip->dev, "|    %s      |    %x     |\n",
			powerdown1_name[bit], (powerdown1 >> bit) & 1);
	dev_info(chip->dev, " -------------------------------\n");

	/*power down log2*/
	regmap_read(chip->base_regmap, PM886_POWER_DOWN_LOG2, &powerdown2);
	dev_info(chip->dev, "PowerDW Log2 0x%x: 0x%x\n",
		PM886_POWER_DOWN_LOG2, powerdown2);
	dev_info(chip->dev, " -------------------------------\n");
	dev_info(chip->dev, "|  name(power down2) |  status  |\n");
	dev_info(chip->dev, "|--------------------|----------|\n");
	for (bit = 0; bit < 5; bit++)
		dev_info(chip->dev, "|    %s     |    %x     |\n",
			powerdown2_name[bit], (powerdown2 >> bit) & 1);
	dev_info(chip->dev, " -------------------------------\n");

	/* write to clear */
	regmap_write(chip->base_regmap, PM886_POWER_DOWN_LOG1, 0xff);
	regmap_write(chip->base_regmap, PM886_POWER_DOWN_LOG2, 0xff);

	/* mask reserved bits and sleep indication */
	powerdown2 &= 0x1e;

	/* keep globals for external usage */
	chip->powerup = powerup;
	chip->powerdown1 = powerdown1;
	chip->powerdown2 = powerdown2;
}

int pm886_post_init_chip(struct pm886_chip *chip)
{
	int ret;
	unsigned int val, data;

	if (!chip || !chip->base_regmap || !chip->power_regmap ||
	    !chip->gpadc_regmap || !chip->battery_regmap)
		return -EINVAL;

	/* save chip stepping */
	ret = regmap_read(chip->base_regmap, PM886_ID_REG, &val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read chip ID: %d\n", ret);
		return ret;
	}
	chip->chip_id = val;

	dev_info(chip->dev, "PM886 chip ID = 0x%x\n", val);

	/* read before alarm wake up bit before initialize interrupt */
	ret = regmap_read(chip->base_regmap, PM886_RTC_ALARM_CTRL1, &val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read RTC register: %d\n", ret);
		return ret;
	}
	chip->rtc_wakeup = !!(val & PM886_ALARM_WAKEUP);

	parse_powerup_down_log(chip);

#if defined(CONFIG_SEC_DEBUG)
	/* read power on reason from PMIC general use register */
	ret = regmap_read(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER, &data);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read PMIC_GENERAL_USE_REGISTER : %d\n"
				, ret);
		return ret;
	}

	pr_info("%s read register PMIC_GENERAL_USE_REGISTER [%d]\n", __func__,
			data);
	val = data & (PMIC_GENERAL_USE_REBOOT_DN_MASK);
	/* read power on reason from PMIC general use register */
	if (val != PMIC_GENERAL_USE_BOOT_BY_FULL_RESET)
	{
		data &= ~(PMIC_GENERAL_USE_REBOOT_DN_MASK);
		data |= PMIC_GENERAL_USE_BOOT_BY_HW_RESET;
		regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,data);
	}
	power_on_reason = (u8)val;
#endif

	return 0;
}

int pm886_init_pages(struct pm886_chip *chip)
{
	struct i2c_client *client = chip->client;
	const struct regmap_config *base_regmap_config;
	const struct regmap_config *power_regmap_config;
	const struct regmap_config *gpadc_regmap_config;
	const struct regmap_config *battery_regmap_config;
	const struct regmap_config *test_regmap_config;
	int ret = 0;

	if (!chip || !chip->power_page_addr ||
	    !chip->gpadc_page_addr || !chip->battery_page_addr)
		return -ENODEV;

	chip->type = pm886_of_get_type(&client->dev);
	switch (chip->type) {
	case PM886:
		base_regmap_config = &pm886_base_i2c_regmap;
		power_regmap_config = &pm886_power_i2c_regmap;
		gpadc_regmap_config = &pm886_gpadc_i2c_regmap;
		battery_regmap_config = &pm886_battery_i2c_regmap;
		test_regmap_config = &pm886_test_i2c_regmap;
		break;
	default:
		return -ENODEV;
	}

	/* base page */
	chip->base_regmap = devm_regmap_init_i2c(client, base_regmap_config);
	if (IS_ERR(chip->base_regmap)) {
		dev_err(chip->dev, "Failed to init base_regmap: %d\n", ret);
		ret = PTR_ERR(chip->base_regmap);
		goto out;
	}

	companion_base_page = chip->base_regmap;
	/* power page */
	chip->power_page = i2c_new_dummy(client->adapter, chip->power_page_addr);
	if (!chip->power_page) {
		dev_err(chip->dev, "Failed to new power_page: %d\n", ret);
		ret = -ENODEV;
		goto out;
	}
	chip->power_regmap = devm_regmap_init_i2c(chip->power_page,
						  power_regmap_config);
	if (IS_ERR(chip->power_regmap)) {
		dev_err(chip->dev, "Failed to init power_regmap: %d\n", ret);
		ret = PTR_ERR(chip->power_regmap);
		goto out;
	}

	/* gpadc page */
	chip->gpadc_page = i2c_new_dummy(client->adapter, chip->gpadc_page_addr);
	if (!chip->gpadc_page) {
		dev_err(chip->dev, "Failed to new gpadc_page: %d\n", ret);
		ret = -ENODEV;
		goto out;
	}
	chip->gpadc_regmap = devm_regmap_init_i2c(chip->gpadc_page,
						  gpadc_regmap_config);
	if (IS_ERR(chip->gpadc_regmap)) {
		dev_err(chip->dev, "Failed to init gpadc_regmap: %d\n", ret);
		ret = PTR_ERR(chip->gpadc_regmap);
		goto out;
	}

	/* battery page */
	chip->battery_page = i2c_new_dummy(client->adapter, chip->battery_page_addr);
	if (!chip->battery_page) {
		dev_err(chip->dev, "Failed to new gpadc_page: %d\n", ret);
		ret = -ENODEV;
		goto out;
	}
	chip->battery_regmap = devm_regmap_init_i2c(chip->battery_page,
						  battery_regmap_config);
	if (IS_ERR(chip->battery_regmap)) {
		dev_err(chip->dev, "Failed to init battery_regmap: %d\n", ret);
		ret = PTR_ERR(chip->battery_regmap);
		goto out;
	}

	/* test page */
	chip->test_page = i2c_new_dummy(client->adapter, chip->test_page_addr);
	if (!chip->test_page) {
		dev_err(chip->dev, "Failed to new test_page: %d\n", ret);
		ret = -ENODEV;
		goto out;
	}
	chip->test_regmap = devm_regmap_init_i2c(chip->test_page,
						 test_regmap_config);
	if (IS_ERR(chip->test_regmap)) {
		dev_err(chip->dev, "Failed to init test_regmap: %d\n", ret);
		ret = PTR_ERR(chip->test_regmap);
		goto out;
	}

out:
	return ret;
}

void pm800_exit_pages(struct pm886_chip *chip)
{
	if (!chip)
		return;

	if (chip->power_page)
		i2c_unregister_device(chip->power_page);
	if (chip->gpadc_page)
		i2c_unregister_device(chip->gpadc_page);
	if (chip->test_page)
		i2c_unregister_device(chip->test_page);
}


int pm886_init_subdev(struct pm886_chip *chip)
{
	int ret;
	if (!chip)
		return -EINVAL;

	ret = mfd_add_devices(chip->dev, 0, pm886_cell_devs,
			      ARRAY_SIZE(pm886_cell_devs), NULL, 0,
			      regmap_irq_get_domain(chip->irq_data));
	return ret;
}

static const struct reg_default pm886_base_patch[] = {
	{PM886_WDOG, 0x1},	 /* disable watchdog */
	{PM886_GPIO_CTRL1, 0x40}, /* gpio1: dvc    , gpio0: input   */
	{PM886_GPIO_CTRL2, 0x00}, /*               , gpio2: input   */
	{PM886_GPIO_CTRL3, 0x44}, /* dvc2          , dvc1           */
	{PM886_GPIO_CTRL4, 0x00}, /* gpio5v_1:input, gpio5v_2: input*/
	{PM886_AON_CTRL2, 0x2a},  /* output 32kHZ from XO */
	{PM886_BK_OSC_CTRL1, 0x0f}, /* OSC_FREERUN = 1, to lock FLL */
	{PM886_LOWPOWER2, 0x20}, /* XO_LJ = 1, enable low jitter for 32kHZ */
	/* enable LPM for internal reference group in sleep */
	{PM886_LOWPOWER4, 0xc0},
};

static const struct reg_default pm886_power_patch[] = {
	{PM886_BUCK5_SLP_CTRL, 0x20}, /* enable BUCK5 in sleep mode */
#if 0
	{PM886_BUCK1_SLP_CTRL, 0x30}, /*TODO: change to use sleep voltage */
	{PM886_LDO1_SLP_CTRL,  0x00}, /* disable LDO in sleep */
	{PM886_LDO2_SLP_CTRL,  0x00},
	{PM886_LDO3_SLP_CTRL,  0x00},
	{PM886_LDO4_SLP_CTRL,  0x00},
	{PM886_LDO5_SLP_CTRL,  0x00},
	{PM886_LDO6_SLP_CTRL,  0x00},
	{PM886_LDO7_SLP_CTRL,  0x00},
	{PM886_LDO8_SLP_CTRL,  0x00},
	{PM886_LDO9_SLP_CTRL,  0x00},
	{PM886_LDO10_SLP_CTRL, 0x00},
	{PM886_LDO11_SLP_CTRL, 0x00},
	{PM886_LDO12_SLP_CTRL, 0x00},
	{PM886_LDO13_SLP_CTRL, 0x00},
	{PM886_LDO14_SLP_CTRL, 0x00},
	{PM886_LDO15_SLP_CTRL, 0x00},
	{PM886_LDO16_SLP_CTRL, 0x00},
#endif
};

static const struct reg_default pm886_gpadc_patch[] = {
	{PM886_GPADC_CONFIG6, 0x03}, /* enable non-stop mode */
};

static const struct reg_default pm886_battery_patch[] = {
	{PM886_CHGBK_CONFIG6, 0xEA},	 /* Improve Charging perf at Reduced VBUS*/
	{PM886_HVSEL_CONFIG1, 0x30},	/* increase deglitch time to 8us */
};

static const struct reg_default pm886_test_patch[] = {
};

/* PMIC chip itself related */
int pm886_apply_patch(struct pm886_chip *chip)
{
	int ret, size;

	if (!chip || !chip->base_regmap || !chip->power_regmap ||
	    !chip->gpadc_regmap || !chip->battery_regmap ||
	    !chip->test_regmap)
		return -EINVAL;

	size = ARRAY_SIZE(pm886_base_patch);
	if (size == 0)
		goto power;
	ret = regmap_register_patch(chip->base_regmap, pm886_base_patch, size);
	if (ret < 0)
		return ret;

power:
	size = ARRAY_SIZE(pm886_power_patch);
	if (size == 0)
		goto gpadc;
	ret = regmap_register_patch(chip->power_regmap, pm886_power_patch, size);
	if (ret < 0)
		return ret;

gpadc:
	size = ARRAY_SIZE(pm886_gpadc_patch);
	if (size == 0)
		goto battery;
	ret = regmap_register_patch(chip->gpadc_regmap, pm886_gpadc_patch, size);
	if (ret < 0)
		return ret;
battery:
	size = ARRAY_SIZE(pm886_battery_patch);
	if (size == 0)
		goto test;
	ret = regmap_register_patch(chip->battery_regmap, pm886_battery_patch, size);
	if (ret < 0)
		return ret;

test:
	size = ARRAY_SIZE(pm886_test_patch);
	if (size == 0)
		goto out;
	ret = regmap_register_patch(chip->test_regmap, pm886_test_patch, size);
	if (ret < 0)
		return ret;
out:
	return 0;
}

int pm886_stepping_fixup(struct pm886_chip *chip)
{
	if (!chip || !chip->client)
		return -EINVAL;

	chip->type = pm886_of_get_type(&chip->client->dev);
	switch (chip->type) {
	case PM886:
		if (chip->chip_id == PM886_A1) {
			/* set HPFM bit for buck1 */
			regmap_update_bits(chip->power_regmap, 0xa0, 0x88, 0x88);
			/* clear LPFM bit for buck1 */
			regmap_update_bits(chip->power_regmap, 0x9f, 1 << 3, 0 << 3);
		}
		/* set USE_XO */
		regmap_update_bits(chip->base_regmap, PM886_RTC_ALARM_CTRL1,
			PM886_USE_XO, PM886_USE_XO);
		break;
	default:
		break;
	}

	return 0;
}

int pm886_apply_bd_patch(struct pm886_chip *chip, struct device_node *np)
{
	unsigned int page, reg, mask, data;
	const __be32 *values;
	int size, rows, index;

	if (!chip || !chip->base_regmap ||
	    !chip->power_regmap || !chip->gpadc_regmap ||
	    !chip->battery_regmap ||
	    !chip->test_regmap)
		return -EINVAL;

	values = of_get_property(np, "marvell,pmic-bd-cfg", &size);
	if (!values) {
		dev_warn(chip->dev, "No valid property for %s\n", np->name);
		/* exit SUCCESS */
		return 0;
	}

	/* number of elements in array */
	size /= sizeof(*values);
	rows = size / 4;
	dev_info(chip->dev, "pmic board specific configuration.\n");
	index = 0;

	while (rows--) {
		page = be32_to_cpup(values + index++);
		reg = be32_to_cpup(values + index++);
		mask = be32_to_cpup(values + index++);
		data = be32_to_cpup(values + index++);
		switch (page) {
		case PM886_BASE_PAGE:
			regmap_update_bits(chip->base_regmap, reg, mask, data);
			break;
		case PM886_POWER_PAGE:
			regmap_update_bits(chip->power_regmap, reg, mask, data);
			break;
		case PM886_GPADC_PAGE:
			regmap_update_bits(chip->gpadc_regmap, reg, mask, data);
			break;
		case PM886_BATTERY_PAGE:
			regmap_update_bits(chip->battery_regmap, reg, mask, data);
			break;
		case PM886_TEST_PAGE:
			regmap_update_bits(chip->test_regmap, reg, mask, data);
			break;
		default:
			dev_err(chip->dev, "wrong page index for %d\n", page);
			break;
		}
	}
	return 0;
}

long pm886_of_get_type(struct device *dev)
{
	const struct of_device_id *id = of_match_device(pm886_of_match, dev);

	if (id)
		return (long)id->data;
	else
		return 0;
}

void pm886_dev_exit(struct pm886_chip *chip)
{
	mfd_remove_devices(chip->dev);
	pm886_irq_exit(chip);
}

void pm886_set_chip(struct pm886_chip *chip)
{
	pm886_chip_priv = chip;
}

struct pm886_chip *pm886_get_chip(void)
{
	return pm886_chip_priv;
}

static int i2c_raw_update_bits(u8 reg, u8 value)
{
	int ret;
	u8 data, buf[2];

	/* only for base page */
	struct i2c_client *client = pm886_get_chip()->client;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &data,
		},
	};

	/*
	 * I2C pins may be in non-AP pinstate, and __i2c_transfer
	 * won't change it back to AP pinstate like i2c_transfer,
	 * so change i2c pins to AP pinstate explicitly here.
	 */
	i2c_pxa_set_pinstate(client->adapter, "default");

	/*
	 * set i2c to pio mode
	 * for in power off sequence, irq has been disabled
	 */
	i2c_set_pio_mode(client->adapter, 1);

	/* 1. read the original value */
	buf[0] = reg;
	ret = __i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("%s read register fails...\n", __func__);
		WARN_ON(1);
		goto out;
	}

	/* 2. update value */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf[0] = reg;
	msgs[0].buf[1] = data | value;
	ret = __i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0) {
		pr_err("%s write data fails: ret = %d\n", __func__, ret);
		WARN_ON(1);
	}
out:
	return ret;
}

static int i2c_raw_read_bits(u8 reg, u8 *data)
{
	int ret;
	u8 buf[2];

	/* only for base page */
	struct i2c_client *client = pm886_get_chip()->client;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = data,
		},
	};

	/*
	 * I2C pins may be in non-AP pinstate, and __i2c_transfer
	 * won't change it back to AP pinstate like i2c_transfer,
	 * so change i2c pins to AP pinstate explicitly here.
	 */
	i2c_pxa_set_pinstate(client->adapter, "default");

	/*
	 * set i2c to pio mode
	 * for in power off sequence, irq has been disabled
	 */
	i2c_set_pio_mode(client->adapter, 1);

	/* 1. read the original value */
	buf[0] = reg;
	ret = __i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("%s read register fails...\n", __func__);
		WARN_ON(1);
	}
	return ret;
}

static int i2c_raw_update_bits_mask(u8 reg, u8 value, u8 mask)
{
	int ret;
	u8 data, buf[2];

	/* only for base page */
	struct i2c_client *client = pm886_get_chip()->client;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &data,
		},
	};

	/*
	 * I2C pins may be in non-AP pinstate, and __i2c_transfer
	 * won't change it back to AP pinstate like i2c_transfer,
	 * so change i2c pins to AP pinstate explicitly here.
	 */
	i2c_pxa_set_pinstate(client->adapter, "default");

	/*
	 * set i2c to pio mode
	 * for in power off sequence, irq has been disabled
	 */
	i2c_set_pio_mode(client->adapter, 1);

	/* 1. read the original value */
	buf[0] = reg;
	ret = __i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("%s read register fails...\n", __func__);
		WARN_ON(1);
		goto out;
	}

	data &= ~(mask);
	/* 2. update value */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf[0] = reg;
	msgs[0].buf[1] = data | value;
	ret = __i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0) {
		pr_err("%s write data fails: ret = %d\n", __func__, ret);
		WARN_ON(1);
	}
out:
	return ret;
}

static int i2c_raw_write_bits(u8 reg, u8 value)
{
	int ret;
	u8 buf[2];

	/* only for base page */
	struct i2c_client *client = pm886_get_chip()->client;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
	};

	/*
	 * I2C pins may be in non-AP pinstate, and __i2c_transfer
	 * won't change it back to AP pinstate like i2c_transfer,
	 * so change i2c pins to AP pinstate explicitly here.
	 */
	i2c_pxa_set_pinstate(client->adapter, "default");

	/*
	 * set i2c to pio mode
	 * for in power off sequence, irq has been disabled
	 */
	i2c_set_pio_mode(client->adapter, 1);

	/* update value */
	msgs[0].buf[0] = reg;
	msgs[0].buf[1] = value;
	ret = __i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0) {
		pr_err("%s write data fails: ret = %d\n", __func__, ret);
		WARN_ON(1);
	}

	return ret;
}

#ifdef CONFIG_SEC_DEBUG
static int i2c_raw_update_bits_gpadc_sec(u8 reg, u8 value)
{
	int ret;
	u8 buf[2];

	/* only for base page */
	struct i2c_client *client = pm886_get_chip()->client;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr+2,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
	};

	/*
	 * I2C pins may be in non-AP pinstate, and __i2c_transfer
	 * won't change it back to AP pinstate like i2c_transfer,
	 * so change i2c pins to AP pinstate explicitly here.
	 */
	i2c_pxa_set_pinstate(client->adapter, "default");

	/*
	 * set i2c to pio mode
	 * for in power off sequence, irq has been disabled
	 */
	i2c_set_pio_mode(client->adapter, 1);

	/* update value */
	msgs[0].buf[0] = reg;
	msgs[0].buf[1] = value;
	ret = __i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0) {
		pr_err("%s write data fails: ret = %d\n", __func__, ret);
		WARN_ON(1);
	}

	return ret;
}
#endif
void pm886_power_off(void)
{
	int ret = 0;
#ifdef CONFIG_SEC_DEBUG
	int data;
	data = PMIC_GENERAL_USE_BOOT_BY_NONE | PMIC_GENERAL_USE_SHUTDOWN_BY_POWEROFF;
	/*PMIC_GENERAL_USE_REGISTER is GPADC RTC6*/
	i2c_raw_update_bits_gpadc_sec(PMIC_GENERAL_USE_REGISTER, data);
	pr_info("PMIC_GENERAL_USE_BOOT_BY_NONE");
	if (ret < 0)
		pr_err("%s sec boot cause power off fails", __func__);
#endif
	if (ta_connected) {
		//pr_info("begin to PMIC watchdog reset\n");
		/* 1.Enable WD_PIN_DIS field */
		ret = i2c_raw_update_bits(PM886_RTC_ALARM_CTRL1, PM886_WD_PIN_DIS);
		if (ret < 0) {
			pr_err("%s fail to update PM886_RTC_ALARM_CTRL1\n", __func__);
			goto p_off;
		}
		/* 2.set WD timer counter as 1024 ms */
		ret = i2c_raw_update_bits_mask(PM886_MISC_CONFIG2, PM886_WD_TIMER_ACT_1S,
				PM886_WD_TIMER_ACT_MASK);
		if (ret < 0) {
			pr_err("%s fail to update PM886_MISC_CONFIG2\n", __func__);
			goto p_off;
		}
		/* 3.disable FAULT_WU and Enable FAULT_WU_EN */
		ret = i2c_raw_update_bits_mask(PM886_AON_CTRL7, PM886_FAULT_WU_EN,
				PM886_FAULT_WU);
		if (ret < 0) {
			pr_err("%s fail to update PM886_AON_CTRL7\n", __func__);
			goto p_off;
		}
		/* 4.Issue SW power down */
		ret = i2c_raw_update_bits(PM886_MISC_CONFIG1, PM886_PWR_HOLD);
		if (ret < 0) {
			pr_err("%s fail to update PM886_MISC_CONFIG1\n", __func__);
			goto p_off;
		}
#if 0
		u8 val;
		ret = i2c_raw_read_bits(PM886_MISC_CONFIG1, &val);
		if (ret < 0)
			pr_err("%s fail read CONFIG1\n", __func__);
		else
			pr_info("PM886_MISC_CONFIG1 0x%02x\n", val);
		ret = i2c_raw_read_bits(PM886_MISC_CONFIG2, &val);
		if (ret < 0)
			pr_err("%s fail read CONFIG2\n", __func__);
		else
			pr_info("PM886_MISC_CONFIG2 0x%02x\n", val);
#endif
		/* 5.enable PMIC WD */
		//pr_info("Enable PMIC WatchDog reset\n");
		ret = i2c_raw_write_bits(PM886_WDOG, PM886_WD_EN);
		if (ret < 0) {
			pr_err("%s fail to update PM886_WDOG\n", __func__);
			goto p_off;
		}
		/* wait for reset */
		for (;;)
			cpu_relax();
	}

p_off:
	pr_info("begin to power off system\n");
	ret = i2c_raw_update_bits(PM886_MISC_CONFIG1, PM886_SW_PDOWN);
	if (ret < 0)
		pr_err("%s power off fails\n", __func__);
	pr_info("finish powering off system: this line shouldn't appear\n");

	/* wait for power off */
	for (;;)
		cpu_relax();
}

int pm886_reboot_notifier_callback(struct notifier_block *this,
				   unsigned long code, void *cmd)
{
	struct pm886_chip *chip;
	unsigned char pmic_download_register = 0;
	int data;
	char *arg = cmd;

	pr_info("%s: code = %ld, cmd = '%s'\n", __func__, code, (char *)cmd);

	chip = container_of(this, struct pm886_chip, reboot_notifier);

#ifdef CONFIG_SEC_DEBUG
	pr_info("reboot notifier sec...\n");

	regmap_read(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER, &data);
	data &= ~(PMIC_GENERAL_USE_REBOOT_DN_MASK);

	if (cmd) {
		if (!strcmp(cmd, "recovery")) {
			pr_info("Device will enter recovery mode on next booting\n");
			data |= PMIC_GENERAL_USE_BOOT_BY_FULL_RESET;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "recovery_done")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_RECOVERY_DONE;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "arm11_fota")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_FOTA;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "alarm")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_RTC_ALARM;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "debug0x4f4c")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_DEBUGLEVEL_LOW;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "debug0x494d")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_DEBUGLEVEL_MID;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "debug0x4948")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_DEBUGLEVEL_HIGH;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "swsel9")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_ENABLE_UART_SWITCHING;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "swsel1")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_DISABLE_UART_SWITCHING;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "GlobalActions restart")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_INTENDED_RESET;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "download")) {
			pmic_download_register = PMIC_GENERAL_DOWNLOAD_MODE_FUS
							+ DOWNLOAD_FUS_SUD_BASE;
			data |= pmic_download_register;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strncmp(cmd, "sud", 3)) {
			/* Value : 21 ~ 29 */
			pmic_download_register = arg[3] - '0' +
							DOWNLOAD_FUS_SUD_BASE;
			data |= pmic_download_register;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else {
			if (get_recoverymode() == 1) {
				pr_info("reset noti recovery mode\n");
				data |= PMIC_GENERAL_USE_BOOT_BY_RECOVERY_DONE;
				regmap_write(chip->gpadc_regmap,
					PMIC_GENERAL_USE_REGISTER, data);
			} else {
				pr_info("reset noti intended\n");
				data |= PMIC_GENERAL_USE_BOOT_BY_INTENDED_RESET;
				regmap_write(chip->gpadc_regmap,
					PMIC_GENERAL_USE_REGISTER, data);
			}
		}
	} else {
		if (get_recoverymode() == 1) {
			pr_info("reset noti recovery p = null\n");
			data |= PMIC_GENERAL_USE_BOOT_BY_RECOVERY_DONE;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
								data);
		} else {
			pr_info("reset noti intended p = null\n");
			data |= PMIC_GENERAL_USE_BOOT_BY_INTENDED_RESET;
			regmap_write(chip->gpadc_regmap, PMIC_GENERAL_USE_REGISTER,
								data);
		}
	}
#else
	if (cmd && (0 == strncmp(cmd, "recovery", 8))) {
		pr_info("%s: --> handle recovery mode\n", __func__);
		regmap_update_bits(chip->base_regmap, PM886_RTC_SPARE6,
				   1 << 0, 1 << 0);

	} else {
		/* clear the recovery indication bit */
		regmap_update_bits(chip->base_regmap,
				   PM886_RTC_SPARE6, 1 << 0, 0);
	}
	/*
	 * the uboot recognize the "reboot" case via power down log,
	 * which is 0 in this case
	 */
#endif

	if (code != SYS_POWER_OFF) {
		pr_info("enable PMIC watchdog\n");
		/* 1.Enable WD_PIN_DIS field */
		regmap_read(chip->base_regmap, PM886_RTC_ALARM_CTRL1, &data);
		//pr_info("PM886_RTC_ALARM_CTRL1 Reg(0x%02x) is 0x%02x\n", PM886_RTC_ALARM_CTRL1, data);
		data |= PM886_WD_PIN_DIS;
		regmap_write(chip->base_regmap, PM886_RTC_ALARM_CTRL1, data);

		/* 2.set WD timer counter as 4s */
		regmap_read(chip->base_regmap, PM886_MISC_CONFIG2, &data);
		//pr_info("PM886_MISC_CONFIG2 Reg(0x%02x) is 0x%02x\n", PM886_MISC_CONFIG2, data);
		data &= ~(PM886_WD_TIMER_ACT_MASK);
		data |= PM886_WD_TIMER_ACT_4S;
		regmap_write(chip->base_regmap,
				PM886_MISC_CONFIG2, data);
		//pr_info("0x%02x is written to PM886_MISC_CONFIG2 Reg(0x%02x)\n", data, PM886_MISC_CONFIG2);

		/* 3.disable FAULT_WU and Enable FAULT_WU_EN */
		regmap_read(chip->base_regmap, PM886_AON_CTRL7, &data);
		//pr_info("PM886_AON_CTRL7 Reg(0x%02x) is 0x%02x\n", PM886_AON_CTRL7, data);
		data &= ~(PM886_FAULT_WU);
		data |= PM886_FAULT_WU_EN;
		regmap_write(chip->base_regmap,
				PM886_AON_CTRL7, data);
		//pr_info("0x%02x is written to PM886_AON_CTRL7 Reg(0x%02x)\n", data, PM886_AON_CTRL7);

		/* 4.Issue SW power down */
		regmap_read(chip->base_regmap, PM886_MISC_CONFIG1, &data);
		//pr_info("PM886_MISC_CONFIG1 Reg(0x%02x) is 0x%02x\n", PM886_MISC_CONFIG1, data);
		data |= PM886_PWR_HOLD;
		regmap_write(chip->base_regmap, PM886_MISC_CONFIG1, data);

		/* 5.enable PMIC WD */
		regmap_write(chip->base_regmap, PM886_WDOG,
				PM886_WD_EN);
		regmap_read(chip->base_regmap, PM886_WDOG, &data);
		//pr_info("WATCHDOG Reg(0x%02x) is 0x%02x\n", PM886_WDOG, data);
	}

	return 0;
}
