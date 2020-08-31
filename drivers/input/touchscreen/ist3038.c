/*
 *  Imagis IST30xx Touchscreen driver
 *
 *  Copyright (C) 2010,Imagis Technology Co. Ltd. All Rights Reserved.
 *  Copyright (C) 2014 Samsung Electronics Co. Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>

#include "ist3038.h"


int ist30xx_key_code[] = {KEY_RECENT, KEY_BACK};

static int ist30xx_power_setup(struct ist30xx_data *data)
{
	struct i2c_client *client = data->client;
	struct regulator *regulator;
	int min_uv, max_uv;
	int ret;

	regulator = devm_regulator_get(&client->dev, "tsp_avdd_3v1");
	if (IS_ERR(regulator)) {
		ret = PTR_ERR(regulator);
		dev_err(&client->dev,
			"Failed to get vdd regulator(%d)\n", ret);
		return ret;
	}

	min_uv = max_uv = data->pdata->avdd_volt;

	ret = regulator_set_voltage(regulator, min_uv, max_uv);
	if (ret < 0) {
		dev_err(&client->dev,
			"Failed to set vdd regulator to %d, %d uV(%d)\n",
			min_uv, max_uv, ret);
		goto out;
	}

	dev_info(&client->dev,
			"Set vdd regulator to %d, %d uV\n", min_uv, max_uv);

	data->pdata->avdd_regulator = regulator;

out:
	return ret;
}

static void ist30xx_power_onoff(struct ist30xx_data *data, bool onoff)
{
	struct regulator *regulator = data->pdata->avdd_regulator;
	struct i2c_client *client = data->client;
	int ret;

	if (!regulator) {
		dev_err(&client->dev, "%s : No vdd regulator\n", __func__);
		return -EPERM;
	}

	if (onoff) {
		ret = regulator_is_enabled(regulator);
		if (ret > 0) {
			dev_info(&client->dev,
				"%s : Already enabled\n", __func__);
			return 0;
		}

		ret = regulator_enable(regulator);
	} else {
		ret = regulator_is_enabled(regulator);
		if (!ret) {
			dev_info(&client->dev,
				"%s : Already disabled\n", __func__);
			return 0;
		}

		ret = regulator_disable(regulator);
	}

	if (!ret)
		dev_info(&client->dev, "%s : %s\n", __func__,
			(onoff) ? "on" : "off");

	return ret;
}

static void ist30xx_power_reset(struct ist30xx_data *data)
{
	ist30xx_power_onoff(data, false);
	ist30xx_power_onoff(data, true);
	msleep(100);
}

static void clear_input_data(struct ist30xx_data *data)
{

}

static void ist30xx_report_input_data(struct ist30xx_data *data, u32 *buff)
{
	struct i2c_client *client = data->client;
	struct finger_info *finger_info;
	bool down_flags[IST30XX_MAX_MT_FINGERS] = { 0 };
	u8 finger_cnt = PARSE_FINGER_CNT(*buff);
	u16 x, y, w, id;
	u16 status;
	bool pressed;
	int i;

	for (i = 1; i <= finger_cnt; i++) {
		finger_info = (struct finger_info *)&buff[i];
		id = finger_info->id - 1;
		data->fingers_status[id].x = finger_info->x;
		data->fingers_status[id].y = finger_info->y;
		data->fingers_status[id].w = finger_info->w;
	}

	status = PARSE_FINGER_STATUS(*buff);
	for (i = 0; i < IST30XX_MAX_MT_FINGERS; i++) {
		pressed = PRESSED_STATUS(status, i);

		if (!pressed)
			if (!data->fingers_status[i].status)
				continue;

		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev,
						MT_TOOL_FINGER, pressed);

		if (pressed) {
			down_flags[i] = data->fingers_status[i].status;

			x = data->fingers_status[i].x;
			y = data->fingers_status[i].y;
			w = data->fingers_status[i].w;

			input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(data->input_dev,
						ABS_MT_TOUCH_MAJOR, w);

			if (!down_flags[i])
				dev_info(&client->dev,
					"[P][%d] : X[%d], Y[%d]\n", i, x, y);

			data->fingers_status[i].status = true;
		} else {
			dev_info(&client->dev, "[R][%d]\n", i);

			data->fingers_status[i].status = false;
		}
	}

	status = PARSE_KEY_STATUS(*buff);
	for (i = 0; i < IST30XX_MAX_KEYS; i++) {
		pressed = PRESSED_STATUS(status, i);

		if (pressed) {
			data->keys_status[i] = true;
		} else {
			if (!data->keys_status[i])
				continue;

			data->keys_status[i] = false;
		}

		input_report_key(data->input_dev,
					ist30xx_key_code[i], pressed);

		dev_info(&client->dev, "key[%3d] is %s\n",
			ist30xx_key_code[i],
			pressed ? "pressed" : "released");
	}

	input_sync(data->input_dev);
}

static irqreturn_t ist30xx_irq_thread(int irq, void *dev_id)
{
	struct ist30xx_data *data = (struct ist30xx_data *)dev_id;
	struct i2c_client *client = data->client;
	u32 buff[IST30XX_MAX_MT_FINGERS + 1] = { 0, };
	u8 reg = CMD_GET_COORD;
	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.buf	= &reg,
			.len	= IST30XX_ADDR_LEN,
		}, {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
		},
	};
	int finger_cnt;
	int ret, i;

	msg[1].buf = (__u8 *)buff;
	msg[1].len = IST30XX_DATA_LEN;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev, "Failed to read touch info(%d)\n", ret);
		goto out;
	}

	*buff = cpu_to_be32(*buff);

	finger_cnt = PARSE_FINGER_CNT(*buff);
	msg[1].buf = (__u8 *)&buff[1];
	msg[1].len = finger_cnt * IST30XX_DATA_LEN;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev, "Failed to read touch packet(%d)\n", ret);
		goto out;
	}

	for (i = 1; i <= finger_cnt; i++)
		buff[i] = cpu_to_be32(buff[i]);

	ist30xx_report_input_data(data, buff);

	return IRQ_HANDLED;

out:
	ist30xx_power_reset(data);
	return IRQ_HANDLED;
}

static struct ist30xx_platform_data *ist30xx_parse_dt(struct device *dev)
{
	struct ist30xx_platform_data *pdata;
	struct device_node *np = dev->of_node;

	if (!np)
		return NULL;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata)) {
		dev_err(dev, "Failed to allocate platform data\n");
		return NULL;
	}

	if (of_property_read_u32(np, "imagis,max_x", &pdata->max_x))
		goto out;

	if (of_property_read_u32(np, "imagis,max_y", &pdata->max_y))
		goto out;

	if (of_property_read_u32(np, "imagis,max_w", &pdata->max_w))
		goto out;

	if (of_property_read_u32(np, "imagis,irq-flag", &pdata->irq_flag))
		goto out;

	if (of_property_read_u32(np, "imagis,avdd-volt", &pdata->avdd_volt))
		goto out;

	return pdata;

out:
	dev_err(dev, "Failed to get property\n");

	return NULL;
}

static int ist30xx_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct ist30xx_data *data;
	struct ist30xx_platform_data *pdata;
	struct input_dev *input_dev;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"Not supported of i2c or smbus function\n");
		return -EIO;
	}

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (unlikely(!data)) {
		dev_err(&client->dev, "Failed to allocate driver data\n");
		return -ENOMEM;
	}

	pdata = ist30xx_parse_dt(&client->dev);
	if (unlikely(!pdata)) {
		dev_err(&client->dev, "Failed to obtain platform data\n");
		return -EINVAL;
	}

	data->client = client;
	data->pdata = pdata;

	ret = ist30xx_power_setup(data);
	if (ret)
		return ret;

	ist30xx_power_onoff(data, true);

	msleep(100);

	input_dev = devm_input_allocate_device(&client->dev);
	if (unlikely(!input_dev)) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input_dev = input_dev;

	input_dev->name = IST30XX_DEV_NAME" Touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_mt_init_slots(input_dev, IST30XX_MAX_MT_FINGERS, 0);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
				0, pdata->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
				0, pdata->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
				0, pdata->max_w, 0, 0);

	{
		int i;

		__set_bit(EV_KEY, input_dev->evbit);
		__set_bit(EV_SYN, input_dev->evbit);
		for (i = 0; i < ARRAY_SIZE(ist30xx_key_code); i++)
			__set_bit(ist30xx_key_code[i], input_dev->keybit);
	}

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "Failed to register input device(%d)\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
		ist30xx_irq_thread, pdata->irq_flag,
		dev_name(&client->dev), data);
	if (ret) {
		dev_err(&client->dev,
			"Failed to register interrupt(%d)\n", ret);
		return ret;
	}

	pm_runtime_enable(&client->dev);

	data->enable = true;

	dev_info(&client->dev, "%s initialized\n", dev_name(&client->dev));

	return 0;
}

static int ist30xx_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ist30xx_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	if (!data->enable) {
		dev_err(&client->dev, "%s : Already disabled\n", __func__);
		goto out;
	}

	disable_irq(data->client->irq);
	ist30xx_power_onoff(data, false);

	data->enable = false;

out:
	return 0;
}

static int ist30xx_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ist30xx_data *data = i2c_get_clientdata(client);

	if (data->enable) {
		dev_err(&client->dev, "%s : Already enabled\n", __func__);
		goto out;
	}

	ist30xx_power_onoff(data, true);
	msleep(100);

	enable_irq(data->client->irq);

	data->enable = true;

out:
	return 0;
}

static const struct dev_pm_ops ist30xx_pm_ops = {
	SET_RUNTIME_PM_OPS(ist30xx_suspend, ist30xx_resume, NULL)
};

static struct i2c_device_id ist30xx_idtable[] = {
	{ IST30XX_DEV_NAME, 0 },
	{ },
};

static const struct of_device_id ist30xx_dt_of_match[] = {
	{ .compatible = "imagis,"IST30XX_DEV_NAME, },
	{ },
};

static struct i2c_driver ist30xx_driver = {
	.id_table	= ist30xx_idtable,
	.probe		= ist30xx_probe,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= IST30XX_DEV_NAME,
		.of_match_table	= ist30xx_dt_of_match,
		.pm		= &ist30xx_pm_ops,
	},
};

module_i2c_driver(ist30xx_driver)

MODULE_DESCRIPTION("Imagis IST30xx touch driver");
MODULE_LICENSE("GPL");

