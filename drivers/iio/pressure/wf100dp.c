/*
 * ap4a.c - driver for the Wengfengheng WF100DP pressure sensor
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
#include <linux/gpio/consumer.h>

#define DRIVER_VERSION "v0.2"
#define TEMP_ZERO	32768
#define PRESS_ZERO	8388608

#define WF100DP_CHAN(index, _type, _info)		\
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

struct wf100dp {
	struct i2c_client *i2c;
	u8 id;
	u8 config;
	u8 speedmode;
	u16 channels;
	u32 prefetch;
	u32 offset[4];
	u32 scale[4];
	u32 fetched[4];
	struct work_struct fetch_work;
	struct mutex lock;
	struct gpio_desc *pressure_select;
};

static int wf100dp_read_channel(struct wf100dp *adc,
				struct iio_chan_spec const *channel, int *value)
{
	signed int ret;
	u8 outbuf[2];
	u8 in_buf[4];

	if (channel->channel == 2 || channel->channel == 3)
		gpiod_set_value(adc->pressure_select, 1);
	else
		gpiod_set_value(adc->pressure_select, 0);

	/// request single conversion
	outbuf[0] = 0x30;
	outbuf[1] = 0x0A;
	ret = i2c_master_send(adc->i2c, outbuf, 2);
	if (ret < 0)
		return -EBUSY;
	else if (ret != 2)
		return -EIO;

	if (channel->type == IIO_TEMP)
		outbuf[0] = 0x09;
	else
		outbuf[0] = 0x06;
	ret = i2c_master_send(adc->i2c, outbuf, 1);
	if (ret < 0)
		return -EBUSY;
	else if (ret != 1)
		return -EIO;

	ret = i2c_master_recv(adc->i2c, in_buf, 3);
	if (ret < 0)
		return -EBUSY;
	else if (ret != 3)
		return -EIO;

	if (channel->type == IIO_TEMP)
		*value = (in_buf[0] << 8 | in_buf[1]);
	else
		*value = (in_buf[0] << 16 | in_buf[1] << 8 | in_buf[2]);

	return (0);
}

static int wf100dp_read_raw(struct iio_dev *iio,
			struct iio_chan_spec const *channel, int *val1,
			int *val2, long mask)
{
	struct wf100dp *adc = iio_priv(iio);
	int err;
	s64 tmp;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (adc->prefetch) {
			*val1 = adc->fetched[channel->channel];
			err = 0;
		} else
			err = wf100dp_read_channel(adc, channel, val1);

		if (err < 0)
			return (err);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		if (adc->prefetch) {
			*val1 = adc->fetched[channel->channel];
			err = 0;
		} else
			err = wf100dp_read_channel(adc, channel, val1);
		if (err < 0)
			return (err);

		if (channel->type == IIO_TEMP) {
			if (*val1 >= TEMP_ZERO)
				*val1 -= 65844;
			else
				*val1 -= 308;
			*val1 = div_s64((s64) *val1 * 10, 256);
		}

		if (channel->type == IIO_PRESSURE) {
			if (adc->offset[channel->channel] > 0)
				*val1 -= adc->offset[channel->channel];
			if (*val1 >= PRESS_ZERO)
				*val1 -= 16777216;
			*val1 = *val1 * 600;
			tmp = div_s64((s64) *val1, 8388608);
			*val1 = tmp + 440;
//			if (adc->scale[channel->channel]) {
//			}
		}

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

static int wf100dp_write_raw(struct iio_dev *iio,
			       struct iio_chan_spec const *channel,
			       int val, int val2, long mask)
{
	struct wf100dp *adc = iio_priv(iio);

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
	struct wf100dp *adc = container_of(work_arg, struct wf100dp, fetch_work);
	struct i2c_client *client = adc->i2c;
	struct iio_dev *indio_dev = i2c_get_clientdata(adc->i2c);
	struct iio_chan_spec const *chan;
	int i, err, val1;

	dev_info(&client->dev, "Starting prefetching on PID: [%d]\n",  current->pid);

	set_current_state(TASK_INTERRUPTIBLE);

	while (adc->prefetch) {
		chan = indio_dev->channels;
		for (i = 0; i < 4; i++) {
			err = wf100dp_read_channel(adc, chan, &val1);
			if (!err) {
				adc->fetched[i] = val1;
			}
			msleep(adc->prefetch);
			chan++;
		}
	}
	return;
}

static ssize_t wf100dp_set_prefetch(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct wf100dp *adc = iio_priv(dev_to_iio_dev(dev));
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

static ssize_t wf100dp_show_prefetch(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct wf100dp *adc = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", adc->prefetch);
}

static IIO_DEVICE_ATTR(prefetchpause, (S_IWUSR | S_IRUGO),
		wf100dp_show_prefetch, wf100dp_set_prefetch, 0);

static struct attribute *wf100dp_attributes[] = {
	&iio_dev_attr_prefetchpause.dev_attr.attr,
	NULL,
};

static const struct attribute_group wf100dp_attribute_group = {
	.attrs = wf100dp_attributes,
};

static const struct iio_chan_spec wf100dp_channels[] = {
	WF100DP_CHAN(0,
			IIO_PRESSURE,
			BIT(IIO_CHAN_INFO_RAW)|\
			BIT(IIO_CHAN_INFO_OFFSET)|\
			BIT(IIO_CHAN_INFO_SCALE)|\
			BIT(IIO_CHAN_INFO_PROCESSED)),
	WF100DP_CHAN(1,
			IIO_TEMP,
			BIT(IIO_CHAN_INFO_RAW)|\
			BIT(IIO_CHAN_INFO_PROCESSED)),
	WF100DP_CHAN(2,
			IIO_PRESSURE,
			BIT(IIO_CHAN_INFO_RAW)|\
			BIT(IIO_CHAN_INFO_OFFSET)|\
			BIT(IIO_CHAN_INFO_SCALE)|\
			BIT(IIO_CHAN_INFO_PROCESSED)),
	WF100DP_CHAN(3,
			IIO_TEMP,
			BIT(IIO_CHAN_INFO_RAW)|\
			BIT(IIO_CHAN_INFO_PROCESSED)),
};

static const struct iio_info wf100dp_info = {
	.read_raw = wf100dp_read_raw,
	.write_raw = wf100dp_write_raw,
	.attrs = &wf100dp_attribute_group,
	.driver_module = THIS_MODULE,
};

#ifdef CONFIG_OF
static const struct of_device_id wf100dp_of_match[] = {
	{ .compatible = "weifengheng,wf100dp" },
	{ }
};

static int wf100dp_of_probe(struct i2c_client *client,
		struct wf100dp *adc)
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
MODULE_DEVICE_TABLE(of, wf100dp_of_match);
#else
static inline int wf100dp_of_probe(struct i2c_client *client,
		struct wf100dp *pdata)
{
	return -ENODEV;
}
#endif

static int wf100dp_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct wf100dp *adc;
	int err;

	dev_info(&client->dev, "Weifenghen WF100DP Pressure Sensor - %s\n", DRIVER_VERSION);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->i2c = client;
	adc->id = (u8)(id->driver_data);
	adc->prefetch = 0;
	adc->scale[0] = 10000;
	adc->scale[1] = 10000;

	if (client->dev.of_node) {
		err = wf100dp_of_probe(client, adc);
		if (err)
			dev_err(&client->dev, "invalid devicetree data");
	}

	if (IS_ERR(adc->pressure_select = devm_gpiod_get(&client->dev, "wf100dp,pressure-select", GPIOD_OUT_LOW)))
		return PTR_ERR(adc->pressure_select);

	mutex_init(&adc->lock);

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = dev_name(&client->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &wf100dp_info;
	indio_dev->channels = wf100dp_channels;
	indio_dev->num_channels = ARRAY_SIZE(wf100dp_channels);

	err = devm_iio_device_register(&client->dev, indio_dev);
	if (err < 0)
		return err;

	i2c_set_clientdata(client, indio_dev);

	INIT_WORK(&adc->fetch_work, fetch_thread);
	if (adc->prefetch)
		schedule_work(&adc->fetch_work);

	return 0;
}

static int wf100dp_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct wf100dp *adc = iio_priv(indio_dev);

	adc->prefetch = 0;
	flush_work(&adc->fetch_work);
	cancel_work_sync(&adc->fetch_work);

	return 0;
}

static const struct i2c_device_id wf100dp_id[] = {
	{ "wf100dp", 0 },
	{ "wf100dc", 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wf100dp_id);

static struct i2c_driver wf100dp_driver = {
	.driver = {
		.name = "wf100dp",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(wf100dp_of_match),
	},
	.probe = wf100dp_probe,
	.remove = wf100dp_remove,
	.id_table = wf100dp_id,
};
module_i2c_driver(wf100dp_driver);

MODULE_AUTHOR("Robert Woerle <rwoerle@mibtec.de.de>");
MODULE_VERSION(DRIVER_VERSION);
MODULE_DESCRIPTION("Weifengheng WF100DP pressure sensor driver");
MODULE_LICENSE("GPL v2");
