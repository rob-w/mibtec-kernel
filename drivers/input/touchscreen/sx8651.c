/*
 * Driver for Semtech SX8651 I2C touchscreen controller.
 *
 * Copyright (c) 2016
 *	Robert Wörle <robert@linuxdevelopment.de>
 *
 * Using code from:
 *  - sx8654.c
 * Sébastien Szymanski <sebastien.szymanski@armadeus.com>
 *	Copyright (c) 2013 U-MoBo Srl
 *	Pierluigi Passaro <p.passaro@u-mobo.com>
 *  - sx8650.c
 *      Copyright (c) 2009 Wayne Roberts
 *  - tsc2007.c
 *      Copyright (c) 2008 Kwangwoo Lee
 *  - ads7846.c
 *      Copyright (c) 2005 David Brownell
 *      Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *      Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *      Copyright (C) 2002 MontaVista Software
 *      Copyright (C) 2004 Texas Instruments
 *      Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

/* register addresses */
#define I2C_REG_CTRL0			0x00
#define I2C_REG_CTRL1			0x01
#define I2C_REG_CTRL2			0x02
#define I2C_REG_CTRL3			0x03
#define I2C_REG_CHANMASK		0x04
#define I2C_REG_STAT			0x05
#define I2C_REG_SOFTRESET		0x1f

/* commands */
#define CMD_READ_REGISTER		0x40
#define CMD_MANUAL				0xb0
#define CMD_PENTRG				0xe0

/* value for I2C_REG_SOFTRESET */
#define SOFTRESET_VALUE			0xde

/* bits for RegTouch1 */
#define CONDIRQ				0x20
#define RPDNT_25KOHM		0xC0
#define FILT_7SA			0x03

/* bits for I2C_REG_CHANMASK */
#define CONV_X				0x80
#define CONV_Y				0x40
#define CONV_Z1				0x20
#define CONV_Z2				0x10

/* coordinates rate: higher nibble of CTRL0 register */
#define RATE_MANUAL			0x00
#define RATE_100CPS			0x60
#define RATE_5000CPS		0xf0

/* power delay: lower nibble of CTRL0 register */
#define POWDLY_1_1MS			0x0b

#define UNTOUCH_TIMEOUT_MS		50

#define MAX_12BIT			((1 << 12) - 1)

struct sx8651 {
	struct input_dev *input;
	struct i2c_client *client;
	struct timer_list timer;
};

static void sx8651_timer(unsigned long data)
{
	struct sx8651 *ts = (void *)data;

	dev_dbg(&ts->client->dev, "pen untouch timer");

	input_report_key(ts->input, BTN_TOUCH, 0);
	input_sync(ts->input);
}

static irqreturn_t sx8651_irq(int irq, void *handle)
{
	struct sx8651 *sx8651 = handle;
	unsigned int x, y, z1, z2;
	int retval;
	u8 data[8];

	dev_dbg(&sx8651->client->dev, "pen touch interrupt");

	retval = i2c_master_recv(sx8651->client, data, sizeof(data));
	if (retval != sizeof(data))
		goto out;

	/* invalid data */
	if (unlikely(data[0] & 0x80 || data[2] & 0x80))
		goto out;

	x = ((data[0] & 0xf) << 8) | (data[1]);
	y = ((data[2] & 0xf) << 8) | (data[3]);

	z1 = ((data[4] & 0xf) << 8) | (data[5]);
	z2 = ((data[6] & 0xf) << 8) | (data[7]);

	dev_dbg(&sx8651->client->dev, "x %d y %d z1 %d z2 %d\n", x, y, z1, z2);

	if (likely(x && z1)) {
		input_report_abs(sx8651->input, ABS_X, x);
		input_report_abs(sx8651->input, ABS_Y, y);
		input_report_key(sx8651->input, BTN_TOUCH, 1);
		input_sync(sx8651->input);

		mod_timer(&sx8651->timer, jiffies + msecs_to_jiffies(UNTOUCH_TIMEOUT_MS));

		dev_dbg(&sx8651->client->dev, "point(%4d,%4d)\n", x, y);
	}

out:
	return IRQ_HANDLED;
}

static int sx8651_open(struct input_dev *dev)
{
	struct sx8651 *sx8651 = input_get_drvdata(dev);
	struct i2c_client *client = sx8651->client;
	int error;

	/* enable pen trigger mode */
	error = i2c_smbus_write_byte_data(client, I2C_REG_CTRL0,
					  RATE_100CPS | POWDLY_1_1MS);
	if (error) {
		dev_err(&client->dev, "writing to I2C_REG_CTRL0 failed");
		return error;
	}

	error = i2c_smbus_write_byte(client, CMD_PENTRG);
	if (error) {
		dev_err(&client->dev, "writing command CMD_PENTRG failed");
		return error;
	}

	enable_irq(client->irq);

	return 0;
}

static void sx8651_close(struct input_dev *dev)
{
	struct sx8651 *sx8651 = input_get_drvdata(dev);
	struct i2c_client *client = sx8651->client;
	int error;

	disable_irq(client->irq);

	/* enable manual mode mode */
	error = i2c_smbus_write_byte(client, CMD_MANUAL);
	if (error) {
		dev_err(&client->dev, "writing command CMD_MANUAL failed");
		return;
	}

	error = i2c_smbus_write_byte_data(client, I2C_REG_CTRL0, 0);
	if (error) {
		dev_err(&client->dev, "writing to I2C_REG_CTRL0 failed");
		return;
	}
}

static int sx8651_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sx8651 *sx8651;
	struct input_dev *input;
	int error;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -ENXIO;

	sx8651 = devm_kzalloc(&client->dev, sizeof(*sx8651), GFP_KERNEL);
	if (!sx8651)
		return -ENOMEM;

	input = devm_input_allocate_device(&client->dev);
	if (!input)
		return -ENOMEM;

	input->name = "SX8651 I2C Touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	input->open = sx8651_open;
	input->close = sx8651_close;

	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_set_capability(input, EV_KEY, BTN_TOUCH);
	input_set_abs_params(input, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, MAX_12BIT, 0, 0);

	sx8651->client = client;
	sx8651->input = input;

	input_set_drvdata(sx8651->input, sx8651);

	setup_timer(&sx8651->timer, sx8651_timer,
		    (unsigned long)sx8651);

	error = i2c_smbus_write_byte_data(client, I2C_REG_SOFTRESET,
					  SOFTRESET_VALUE);
	if (error) {
		dev_err(&client->dev, "writing softreset value failed");
		return error;
	}

	error = i2c_smbus_write_byte_data(client, I2C_REG_CHANMASK,
					  CONV_X | CONV_Y | CONV_Z1 | CONV_Z2);
	if (error) {
		dev_err(&client->dev, "writing to I2C_REG_CHANMASK failed");
		return error;
	}

	error = i2c_smbus_write_byte_data(client, I2C_REG_CTRL1,
					  CONDIRQ | RPDNT_25KOHM | FILT_7SA);
	if (error) {
		dev_err(&client->dev, "writing to I2C_REG_CTRL1 failed");
		return error;
	}

	error = i2c_smbus_write_byte_data(client, I2C_REG_CTRL2, POWDLY_1_1MS);
	if (error) {
		dev_err(&client->dev, "writing to I2C_REG_CTRL2 failed");
		return error;
	}

	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, sx8651_irq,
					  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					  client->name, sx8651);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ %d, error: %d\n",
			client->irq, error);
		return error;
	}

	/* Disable the IRQ, we'll enable it in sx8654_open() */
	disable_irq(client->irq);

	error = input_register_device(sx8651->input);
	if (error)
		return error;

	dev_info(&client->dev, "registered with irq (%d)\n", client->irq);

	i2c_set_clientdata(client, sx8651);
	return 0;
}

static int sx8651_remove(struct i2c_client *client)
{
	struct sx8651 *ts = i2c_get_clientdata(client);

	del_timer_sync(&ts->timer);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sx8651_of_match[] = {
	{ .compatible = "semtech,sx8651", },
	{ },
};
MODULE_DEVICE_TABLE(of, sx8651_of_match);
#endif

static const struct i2c_device_id sx8651_id_table[] = {
	{ "sx8651", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sx8651_id_table);

static struct i2c_driver sx8651_driver = {
	.driver = {
		.name = "sx8651",
		.of_match_table = of_match_ptr(sx8651_of_match),
	},
	.id_table = sx8651_id_table,
	.probe = sx8651_probe,
	.remove = sx8651_remove,
};
module_i2c_driver(sx8651_driver);

MODULE_AUTHOR("Robert Woerle <robert@linuxdevelopment.de>");
MODULE_DESCRIPTION("Semtech SX8651 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
