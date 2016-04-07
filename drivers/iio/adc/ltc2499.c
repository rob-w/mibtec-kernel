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
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
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

#define SCALED_TO_20mA			20000

#define LTC2499_CHAN(index, _type)			\
	{										\
		.type = (_type),					\
		.indexed = 1,						\
		.channel = index,					\
		.channel2 = index,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)|\
			BIT(IIO_CHAN_INFO_SCALE)|\
			BIT(IIO_CHAN_INFO_PROCESSED),\
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

struct ltc2499 {
	struct i2c_client *i2c;
	u8 id;
	u8 config;
	u8 speedmode;
	u16 channels;
	u32 prefetch;
	u32 scale[9];
	u32 fetched[9];
	struct work_struct fetch_work;
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
	unsigned long long tmp;
	int err;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (adc->prefetch) {
			*val1 = adc->fetched[channel->channel];
			err = 0;
		} else
			err = ltc2499_read_channel(adc, channel, val1);

		if (err < 0)
			return (err);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		if (adc->prefetch) {
			*val1 = adc->fetched[channel->channel];
			err = 0;
		} else
			err = ltc2499_read_channel(adc, channel, val1);
		if (err < 0)
			return (err);

		if (channel->type == IIO_TEMP)
			*val1 = ((*val1 * 125) / 15700) - 2730;

		if (channel->type == IIO_CURRENT) {
			tmp = div_s64((s64)adc->scale[channel->channel], SCALED_TO_20mA);
			if (tmp > 0)
				*val1 = div_s64((s64)*val1,  tmp);
		}

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		if (channel->type == IIO_CURRENT)
			*val1 = adc->scale[channel->channel];
		if (channel->type == IIO_TEMP)
			*val1 = adc->scale[channel->channel + 8];

		*val2 = 0;
		return IIO_VAL_INT;

	default:
		break;
	}

	return -EINVAL;
}

static int ltc2499_write_raw(struct iio_dev *iio,
			       struct iio_chan_spec const *channel,
			       int val, int val2, long mask)
{
	struct ltc2499 *adc = iio_priv(iio);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (channel->type == IIO_CURRENT)
			adc->scale[channel->channel] = val;
		if (channel->type == IIO_TEMP)
			adc->scale[channel->channel + 8] = val;
		break;
	default:
		break;
	}

	return 0;
}

static void fetch_thread(struct work_struct *work_arg)
{
	struct ltc2499 *adc = container_of(work_arg, struct ltc2499, fetch_work);
	struct i2c_client *client = adc->i2c;
	struct iio_dev *indio_dev = i2c_get_clientdata(adc->i2c);
	struct iio_chan_spec const *chan;
	int i, err, val1, sleeper;

	dev_info(&client->dev, "Starting prefetching on PID: [%d]\n",  current->pid);

	set_current_state(TASK_INTERRUPTIBLE);

	while (adc->prefetch) {
		chan = indio_dev->channels;
		for (i = 0; i < 9 && adc->prefetch; i++) {
			if (adc->channels & (1 << i)) {
				if (chan->type == IIO_TEMP)
					sleeper = LTC2499_SLEEP_M0;
				else
					sleeper = adc->speedmode ? LTC2499_SLEEP_M1 : LTC2499_SLEEP_M0;

				err = ltc2499_read_channel(adc, chan, &val1);
				if (!err)
					adc->fetched[i] = val1;

				/* as we cycle through channels, we need to wait one more conversion
				*  before we reconfigure a new channel */
				msleep(sleeper);
			}
			chan++;
		}
		msleep(adc->prefetch);
	}
	return;
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

static ssize_t ltc2499_set_channels(struct device *dev,
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

	if (val < 0)
		return -EINVAL;

	adc->channels = val;

error_ret:
	return ret ? ret : len;
}

static ssize_t ltc2499_show_channels(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ltc2499 *adc = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", adc->channels);
}

static ssize_t ltc2499_set_prefetch(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct ltc2499 *adc = iio_priv(dev_to_iio_dev(dev));
	unsigned int val;
	int ret;
	int on = 0;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto error_ret;

	if (val < 0)
		return -EINVAL;

	if (!adc->prefetch && val)
		on = 1;
	adc->prefetch = val;
	if (on)
		schedule_work(&adc->fetch_work);

error_ret:
	return ret ? ret : len;
}

static ssize_t ltc2499_show_prefetch(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ltc2499 *adc = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", adc->prefetch);
}

static IIO_DEVICE_ATTR(speedmode, (S_IWUSR | S_IRUGO),
		ltc2499_show_speedmode, ltc2499_set_speedmode, 0);

static IIO_DEVICE_ATTR(channels, (S_IWUSR | S_IRUGO),
		ltc2499_show_channels, ltc2499_set_channels, 0);

static IIO_DEVICE_ATTR(prefetchpause, (S_IWUSR | S_IRUGO),
		ltc2499_show_prefetch, ltc2499_set_prefetch, 0);

static struct attribute *ltc2499_attributes[] = {
	&iio_dev_attr_speedmode.dev_attr.attr,
	&iio_dev_attr_channels.dev_attr.attr,
	&iio_dev_attr_prefetchpause.dev_attr.attr,
	NULL,
};

static const struct attribute_group ltc2499_attribute_group = {
	.attrs = ltc2499_attributes,
};

static const struct iio_chan_spec ltc2499_channels[] = {
	LTC2499_CHAN(0, IIO_CURRENT),
	LTC2499_CHAN(1, IIO_CURRENT),
	LTC2499_CHAN(2, IIO_CURRENT),
	LTC2499_CHAN(3, IIO_CURRENT),
	LTC2499_CHAN(4, IIO_CURRENT),
	LTC2499_CHAN(5, IIO_CURRENT),
	LTC2499_CHAN(6, IIO_CURRENT),
	LTC2499_CHAN(7, IIO_CURRENT),
	LTC2499_CHAN(0, IIO_TEMP),
};

static const struct iio_info ltc2499_info = {
	.read_raw = ltc2499_read_raw,
	.write_raw = ltc2499_write_raw,
	.attrs = &ltc2499_attribute_group,
	.driver_module = THIS_MODULE,
};

#ifdef CONFIG_OF
static const struct of_device_id ltc2499_of_match[] = {
	{ .compatible = "ltc,ltc2499" },
	{ }
};

static int ltc2499_of_probe(struct i2c_client *client,
		struct ltc2499 *adc)
{
	struct device *dev = &client->dev;
	u32 val;

	if(of_property_read_u32(dev->of_node, "speedmode", &val) >= 0)
		if (val == 0 || val == 1)
			adc->speedmode = val;

	if(of_property_read_u32(dev->of_node, "channels", &val) >= 0)
		if (val >= 0)
			adc->channels = val;

	if(of_property_read_u32(dev->of_node, "prefetchpause", &val) >= 0) {
		if (val >= 0)
			adc->prefetch = val;
		else
			adc->prefetch = 0;
	}

	return 0;
}
MODULE_DEVICE_TABLE(of, ltc2499_of_match);
#else
static inline int ltc2499_of_probe(struct i2c_client *client,
		struct ltc2499 *pdata)
{
	return -ENODEV;
}
#endif

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
	adc->speedmode = 0;
	adc->prefetch = 0;
	adc->channels = 0x1FF;

	if (client->dev.of_node) {
		err = ltc2499_of_probe(client, adc);
		if (err)
			dev_err(&client->dev, "invalid devicetree data");
	}

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

	INIT_WORK(&adc->fetch_work, fetch_thread);
	if (adc->prefetch)
		schedule_work(&adc->fetch_work);

	return 0;
}

static int ltc2499_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ltc2499 *adc = iio_priv(indio_dev);

	adc->prefetch = 0;
	flush_work(&adc->fetch_work);

	return 0;
}

static const struct i2c_device_id ltc2499_id[] = {
	{ "ltc2499", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc2499_id);

static struct i2c_driver ltc2499_driver = {
	.driver = {
		.name = "ltc2499",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ltc2499_of_match),
	},
	.probe = ltc2499_probe,
	.remove = ltc2499_remove,
	.id_table = ltc2499_id,
};
module_i2c_driver(ltc2499_driver);

MODULE_AUTHOR("Robert Woerle <robert@linuxdevelopment.de>");
MODULE_VERSION(DRIVER_VERSION);
MODULE_DESCRIPTION("Linear Technology ltc2499 driver");
MODULE_LICENSE("GPL v2");
