/*
 * ap4a.c - driver for the Fujikara AP4/AG4 pressure sensors
 *
 * Copyright (C) 2017, Robert Woerle
 * Author: Robert Wörle <robert@linuxdevelopment.de>
 *
 * base on drivers/iio/adc/mcp3422.c
 * Copyright (C) 2013, Angelo Compagnucci
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
#define MAX_RAW		15565

#define AP4A_CHAN(index, _type, _info)		\
	{										\
		.type = (_type),					\
		.indexed = 1,						\
		.channel = index,					\
		.info_mask_separate = (_info),		\
		.scan_index = index + 1,			\
		.scan_type = {						\
			.shift = 5,						\
			.endianness = IIO_BE,			\
		},									\
	}

struct ap4a {
	struct i2c_client *i2c;
	u8 id;
	u8 config;
	u8 speedmode;
	u16 channels;
	u32 prefetch;
	u32 offset[2];
	u32 scale[2];
	u32 fetched[2];
	struct work_struct fetch_work;
	struct mutex lock;
};

static int ap4a_read_channel(struct ap4a *adc,
				struct iio_chan_spec const *channel, int *value)
{
	signed int ret;
	u8 outbuf[] = {0};
	u8 in_buf[4];

	ret = i2c_master_send(adc->i2c, outbuf, 1);

	if (ret < 0)
		return -EBUSY;
	else if (ret != 1)
		return -EIO;

	ret = i2c_master_recv(adc->i2c, in_buf, 4);

	if (ret < 0)
		return -EBUSY;
	else if (ret != 4)
		return -EIO;

	if (channel->type == IIO_TEMP) {
		*value = (in_buf[2] << 8 | in_buf[3]);
		*value >>= channel->scan_type.shift;
	} else {
		*value = (in_buf[0] << 8 | in_buf[1]);
		/// MASK away status bits
		*value &= 0x3fff;
	}

	return (0);
}

static int ap4a_read_raw(struct iio_dev *iio,
			struct iio_chan_spec const *channel, int *val1,
			int *val2, long mask)
{
	struct ap4a *adc = iio_priv(iio);
	int err;
	unsigned long long tmp;
	int range = 0;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (adc->prefetch) {
			*val1 = adc->fetched[channel->channel];
			err = 0;
		} else
			err = ap4a_read_channel(adc, channel, val1);

		if (err < 0)
			return (err);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		if (adc->prefetch) {
			*val1 = adc->fetched[channel->channel];
			err = 0;
		} else
			err = ap4a_read_channel(adc, channel, val1);
		if (err < 0)
			return (err);

		if (channel->type == IIO_TEMP) {
			*val1 = ((*val1 - 512) * 9765);
			*val1 = div_s64((s64) *val1, 10000);
		}

		if (channel->type == IIO_PRESSURE) {
			if (adc->offset[channel->channel] > 0) {
				*val1 -= adc->offset[channel->channel];
				range = MAX_RAW - adc->offset[channel->channel];
			}

			if (adc->scale[channel->channel]) {
				if (range > 0) {
					tmp = div_s64((s64) adc->scale[channel->channel]* 1000, range);
					*val1 = div_s64(((s64)*val1 * tmp), 1000);
				} else
					*val1 = 0;
			}
		}

		if (*val1 < 0)
			*val1 = 0;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_OFFSET:
		*val1 = adc->offset[channel->channel];
		*val2 = 0;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val1 = adc->scale[channel->channel];
		*val2 = 0;
		return IIO_VAL_INT;

	default:
		break;
	}

	return -EINVAL;
}

static int ap4a_write_raw(struct iio_dev *iio,
			       struct iio_chan_spec const *channel,
			       int val, int val2, long mask)
{
	struct ap4a *adc = iio_priv(iio);

	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		adc->offset[channel->channel] = val;
		break;
	case IIO_CHAN_INFO_SCALE:
		adc->scale[channel->channel] = val;
		break;
	default:
		break;
	}

	return 0;
}

static void fetch_thread(struct work_struct *work_arg)
{
	struct ap4a *adc = container_of(work_arg, struct ap4a, fetch_work);
	struct i2c_client *client = adc->i2c;
	struct iio_dev *indio_dev = i2c_get_clientdata(adc->i2c);
	struct iio_chan_spec const *chan;
	int i, err, val1;

	dev_info(&client->dev, "Starting prefetching on PID: [%d]\n",  current->pid);

	set_current_state(TASK_INTERRUPTIBLE);

	while (adc->prefetch) {
		chan = indio_dev->channels;
		for (i = 0; i < 2; i++) {
			err = ap4a_read_channel(adc, chan, &val1);
			if (!err) {
				adc->fetched[i] = val1;
			}
			msleep(adc->prefetch);
			chan++;
		}
	}
	return;
}

static ssize_t ap4a_set_prefetch(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct ap4a *adc = iio_priv(dev_to_iio_dev(dev));
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto error_ret;

	if (val < 0)
		return -EINVAL;

	if (!adc->prefetch && val) {
		adc->prefetch = val;
		schedule_work(&adc->fetch_work);
	}

	if (adc->prefetch && !val) {
		adc->prefetch = val;
		cancel_work_sync(&adc->fetch_work);
	}

error_ret:
	return ret ? ret : len;
}

static ssize_t ap4a_show_prefetch(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ap4a *adc = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", adc->prefetch);
}

static IIO_DEVICE_ATTR(prefetchpause, (S_IWUSR | S_IRUGO),
		ap4a_show_prefetch, ap4a_set_prefetch, 0);

static struct attribute *ap4a_attributes[] = {
	&iio_dev_attr_prefetchpause.dev_attr.attr,
	NULL,
};

static const struct attribute_group ap4a_attribute_group = {
	.attrs = ap4a_attributes,
};

static const struct iio_chan_spec ap4a_channels[] = {
	AP4A_CHAN(0,
			IIO_PRESSURE,
			BIT(IIO_CHAN_INFO_RAW)|\
			BIT(IIO_CHAN_INFO_OFFSET)|\
			BIT(IIO_CHAN_INFO_SCALE)|\
			BIT(IIO_CHAN_INFO_PROCESSED)),
	AP4A_CHAN(1,
			IIO_TEMP,
			BIT(IIO_CHAN_INFO_RAW)|\
			BIT(IIO_CHAN_INFO_PROCESSED)),
};

static const struct iio_info ap4a_info = {
	.read_raw = ap4a_read_raw,
	.write_raw = ap4a_write_raw,
	.attrs = &ap4a_attribute_group,
	.driver_module = THIS_MODULE,
};

#ifdef CONFIG_OF
static const struct of_device_id ap4a_of_match[] = {
	{ .compatible = "fujikara,ap4a" },
	{ }
};

static int ap4a_of_probe(struct i2c_client *client,
		struct ap4a *adc)
{
	struct device *dev = &client->dev;
	u32 val;

	if(of_property_read_u32(dev->of_node, "prefetchpause", &val) >= 0) {
		if (val >= 0)
			adc->prefetch = val;
		else
			adc->prefetch = 0;
	}

	return 0;
}
MODULE_DEVICE_TABLE(of, ap4a_of_match);
#else
static inline int ap4a_of_probe(struct i2c_client *client,
		struct ap4a *pdata)
{
	return -ENODEV;
}
#endif

static int ap4a_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct ap4a *adc;
	int err;

	dev_info(&client->dev, "Fujikara AP4/AG4 Pressure Sensor - %s\n", DRIVER_VERSION);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->i2c = client;
	adc->id = (u8)(id->driver_data);
	adc->prefetch = 0;

	if (client->dev.of_node) {
		err = ap4a_of_probe(client, adc);
		if (err)
			dev_err(&client->dev, "invalid devicetree data");
	}

	mutex_init(&adc->lock);

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = dev_name(&client->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ap4a_info;
	indio_dev->channels = ap4a_channels;
	indio_dev->num_channels = ARRAY_SIZE(ap4a_channels);

	err = devm_iio_device_register(&client->dev, indio_dev);
	if (err < 0)
		return err;

	i2c_set_clientdata(client, indio_dev);

	INIT_WORK(&adc->fetch_work, fetch_thread);
	if (adc->prefetch)
		schedule_work(&adc->fetch_work);

	return 0;
}

static int ap4a_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ap4a *adc = iio_priv(indio_dev);

	adc->prefetch = 0;
	flush_work(&adc->fetch_work);
	cancel_work_sync(&adc->fetch_work);

	return 0;
}

static const struct i2c_device_id ap4a_id[] = {
	{ "ap4a", 0 },
	{ "ag4a", 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ap4a_id);

static struct i2c_driver ap4a_driver = {
	.driver = {
		.name = "ap4a",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ap4a_of_match),
	},
	.probe = ap4a_probe,
	.remove = ap4a_remove,
	.id_table = ap4a_id,
};
module_i2c_driver(ap4a_driver);

MODULE_AUTHOR("Robert Woerle <robert@linuxdevelopment.de>");
MODULE_VERSION(DRIVER_VERSION);
MODULE_DESCRIPTION("Fujikara AP4/AG4 pressure sensor driver");
MODULE_LICENSE("GPL v2");
