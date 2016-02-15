/*
 * ltc2499.c - driver for the Microchip ltc2499 24bit 16 chan ADC chip
 *
 * Copyright (C) 2016, Robert Woerle
 * Author: Robert Wörle <robert@linuxdevelopment.de>
 *
 * base on drivers/iio/adc/mcp3422.c
 * Copyright (C) 2013, Angelo Compagnucci

 * This driver exports the value of analog input voltage to sysfs, the
 * voltage unit is nV.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define DRIVER_VERSION "v1.1"

#define LTC2499_SPEED_MODE		1<<3
#define LTC2499_REJECT_FB		1<<4
#define LTC2499_REJECT_FA		1<<5
#define LTC2499_INTERNAL_TEMP	1<<6
#define LTC2499_NEW_CONFIG		1<<7

#define LTC2499_SAMPLE_HEADER	0x80
#define LTC2499_SAMPLE_EN		1<<5
#define LTC2499_SAMPLE_SGL		1<<4
#define LTC2499_SAMPLE_ODD		1<<3

#define LTC2499_SLEEP_M0		164
#define LTC2499_SLEEP_M1		82


#define LTC2499_CHAN(index)					\
	{										\
		.type = IIO_VOLTAGE,				\
		.indexed = 1,						\
		.channel = index,					\
		.channel2 = index,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.scan_index = index + 1,			\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 24,					\
			.storagebits = 32,				\
			.shift = 6,						\
			.endianness = IIO_BE,			\
		},									\
		.differential = 1,					\
	}

/* Client data (each client gets its own) */
struct ltc2499 {
	struct i2c_client *i2c;
	u8 id;
	u8 config;
	u8 speedmode;
	u8 pga[4];
	struct mutex lock;
};

static int ltc2499_read_channel(struct ltc2499 *adc,
				struct iio_chan_spec const *channel, int *value)
{
	signed int ret;
	u8 outbuf[] = {0, 0};
	u8 in_buf[4];
	u16 sleeper = 0;

	if (channel->type == IIO_TEMP) {
		sleeper = LTC2499_SLEEP_M0;
		outbuf[0] = (LTC2499_SAMPLE_HEADER | LTC2499_SAMPLE_EN);
		outbuf[1] = (LTC2499_INTERNAL_TEMP | LTC2499_REJECT_FB | LTC2499_NEW_CONFIG);
	} else {
		sleeper = adc->speedmode ? LTC2499_SLEEP_M1 : LTC2499_SLEEP_M0;
		outbuf[0] = (LTC2499_SAMPLE_HEADER | LTC2499_SAMPLE_EN |
					channel->channel);
		outbuf[1] = ((adc->speedmode ? LTC2499_SPEED_MODE : 0) |
					LTC2499_REJECT_FB | LTC2499_NEW_CONFIG);
	}

	ret = i2c_master_send(adc->i2c, outbuf, 2);

	if (ret < 0)
		return -EBUSY;
	else if (ret != 2)
		return -EIO;

	msleep(sleeper);

	ret = i2c_master_recv(adc->i2c, in_buf, 4);

	if (ret < 0)
		return -EBUSY;
	else if (ret != 4)
		return -EIO;

	*value = (in_buf[0] << 24 | in_buf[1] << 16 | in_buf[2] << 8 | in_buf[3]);

	/// MASK away bit 31
	*value &= 0x7fffffff;

	/// shift out unused lower 6bits
	*value >>= channel->scan_type.shift;

	/// set new bit25 to be signed bit
	if (channel->scan_type.sign == 's')
		*value = sign_extend32(*value, 25);

	return (0);
}

static int ltc2499_read_raw(struct iio_dev *iio,
			struct iio_chan_spec const *channel, int *val1,
			int *val2, long mask)
{
	struct ltc2499 *adc = iio_priv(iio);
	int err;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		err = ltc2499_read_channel(adc, channel, val1);
		if (err < 0)
			return (err);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		err = ltc2499_read_channel(adc, channel, val1);
		if (err < 0)
			return (err);

		*val1 = ((*val1 * 125) / 15700) - 2730;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val1 = 0;
		*val2 = 0;
		return IIO_VAL_INT_PLUS_NANO;

	default:
		break;
	}

	return -EINVAL;
}

static ssize_t ltc2499_set_speedmode(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct ltc2499 *adc = iio_priv(dev_to_iio_dev(dev));
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto error_ret;

	if (val < 0 || val >= 2)
		return -EINVAL;

	adc->speedmode = val;

error_ret:
	return ret ? ret : len;
}

static ssize_t ltc2499_show_speedmode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ltc2499 *adc = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", adc->speedmode);
}

static IIO_DEVICE_ATTR(speedmode, (S_IWUSR | S_IRUGO),
		ltc2499_show_speedmode, ltc2499_set_speedmode, 0);

static struct attribute *ltc2499_attributes[] = {
	&iio_dev_attr_speedmode.dev_attr.attr,
	NULL,
};

static const struct attribute_group ltc2499_attribute_group = {
	.attrs = ltc2499_attributes,
};

static const struct iio_chan_spec ltc2499_channels[] = {
	LTC2499_CHAN(0),
	LTC2499_CHAN(1),
	LTC2499_CHAN(2),
	LTC2499_CHAN(3),
	LTC2499_CHAN(4),
	LTC2499_CHAN(5),
	LTC2499_CHAN(6),
	LTC2499_CHAN(7),
	{
		.type = IIO_TEMP,
		.indexed = 0,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 24,					\
			.storagebits = 32,				\
			.shift = 6,						\
			.endianness = IIO_BE,			\
		},									\
	}
};

static const struct iio_info ltc2499_info = {
	.read_raw = ltc2499_read_raw,
	.attrs = &ltc2499_attribute_group,
	.driver_module = THIS_MODULE,
};

static int ltc2499_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct ltc2499 *adc;
	int err;

	dev_info(&client->dev, "8x IIO ADC 24bit differential - %s\n", DRIVER_VERSION);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->i2c = client;
	adc->id = (u8)(id->driver_data);

	adc->speedmode = 1;

	mutex_init(&adc->lock);

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = dev_name(&client->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ltc2499_info;

	indio_dev->channels = ltc2499_channels;
	indio_dev->num_channels = ARRAY_SIZE(ltc2499_channels);

	err = devm_iio_device_register(&client->dev, indio_dev);
	if (err < 0)
		return err;

	i2c_set_clientdata(client, indio_dev);

	return 0;
}

static const struct i2c_device_id ltc2499_id[] = {
	{ "ltc2499", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc2499_id);

#ifdef CONFIG_OF
static const struct of_device_id ltc2499_of_match[] = {
	{ .compatible = "ltc,ltc2499" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2499_of_match);
#endif

static struct i2c_driver ltc2499_driver = {
	.driver = {
		.name = "ltc2499",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ltc2499_of_match),
	},
	.probe = ltc2499_probe,
	.id_table = ltc2499_id,
};
module_i2c_driver(ltc2499_driver);

MODULE_AUTHOR("Robert Woerle <robert@linuxdevelopment.de>");
MODULE_VERSION(DRIVER_VERSION);
MODULE_DESCRIPTION("Linear Technology ltc2499 driver");
MODULE_LICENSE("GPL v2");
