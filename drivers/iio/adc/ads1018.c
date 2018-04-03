/*
 * ap4a.c - driver for the Texas Instruments ADS1018 ADC
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
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define DRIVER_VERSION "v0.94"

#define ADS1018_MODE_SINGLE		1<<0
#define ADS1018_VALID_DATA		3
#define ADS1018_PULL_UP_EN		1<<3
#define ADS1018_TS_MODE			1<<4
#define ADS1018_SS_START		1<<7

#define ADS1018_RATE_SHIFT		5
#define ADS1018_INPUT_SHIFT		4
#define ADS1018_GAIN_SHIFT		1

enum {
	ADS1018_AMP_FSR_6_144V = 0,
	ADS1018_AMP_FSR_4_096V,
	ADS1018_AMP_FSR_2_048V,
	ADS1018_AMP_FSR_1_024V,
	ADS1018_AMP_FSR_0_512V,
	ADS1018_AMP_FSR_0_256V
};

enum {
	ADS1018_AIN0_AIN1 = 0,
	ADS1018_AIN0_AIN3,
	ADS1018_AIN1_AIN3,
	ADS1018_AIN2_AIN3,
	ADS1018_AIN0_GND,
	ADS1018_AIN1_GND,
	ADS1018_AIN2_GND,
	ADS1018_AIN3_GND
};

#define ADS1018_CHAN(index, _type, _info)	\
	{										\
		.type = (_type),					\
		.indexed = 1,						\
		.channel = index,					\
		.info_mask_separate = (_info),		\
		.scan_index = index + 1,			\
		.scan_type = {						\
			.shift = 4,						\
			.endianness = IIO_BE,			\
		},									\
		.ext_info = ads1018_ext_info,		\
	}

struct ads1018 {
	struct spi_device *spi;
	struct device *dev;
	u8 id;
	u8 config;
	u8 single_shot;
	u16 channels;
	u32 prefetch;
	u32 offset[2];
	u32 scale[2];
	u32 fetched[3];
	unsigned sample_rate_mode;
	struct work_struct fetch_work;
	struct mutex lock;
};

static int ads1018_read_channel(struct ads1018 *adc,
				struct iio_chan_spec const *channel, int *val)
{
	int ret;
	struct spi_message message;
	struct spi_transfer xfer[2];
	u8 tx[2] = {0x0, 0x0};
	u8 rx[2] = {0x0, 0x0};

	spi_message_init(&message);
	memset(xfer, 0, sizeof(xfer));

	tx[1] = (ADS1018_VALID_DATA
		| ADS1018_PULL_UP_EN
		| adc->sample_rate_mode << ADS1018_RATE_SHIFT);

	if (channel->type == IIO_CURRENT) {
		tx[0] = (adc->single_shot ? ADS1018_MODE_SINGLE : 0)
				| (adc->single_shot ? ADS1018_SS_START : 0)
				| (ADS1018_AMP_FSR_1_024V << ADS1018_GAIN_SHIFT);

		if (channel->channel == 0)
			tx[0] |= (ADS1018_AIN0_AIN1 << ADS1018_INPUT_SHIFT);
		else if (channel->channel == 1)
			tx[0] |= (ADS1018_AIN2_AIN3 << ADS1018_INPUT_SHIFT);
	}

	if (channel->type == IIO_TEMP)
		tx[1] = (ADS1018_VALID_DATA
			| ADS1018_PULL_UP_EN
			| adc->sample_rate_mode << ADS1018_RATE_SHIFT
			| ADS1018_TS_MODE);

	xfer[0].tx_buf = &tx;
	xfer[0].rx_buf = &rx;
	xfer[0].len = 2;
	spi_message_add_tail(&xfer[0], &message);

	ret = spi_sync(adc->spi, &message);
	if (unlikely(ret)) {
		dev_err(adc->dev, "SPI error: %d\n", ret);
		return ret;
	}

	msleep(2);

	spi_message_init(&message);
	memset(xfer, 0, sizeof(xfer));
	xfer[0].tx_buf = &tx;
	xfer[0].rx_buf = &rx;
	xfer[0].len = 2;
	spi_message_add_tail(&xfer[0], &message);

	ret = spi_sync(adc->spi, &message);
	if (unlikely(ret)) {
		dev_err(adc->dev, "SPI error: %d\n", ret);
		return ret;
	}

	*val = (rx[0] << 8 | rx[1]);

	/// shift out unused bits
	*val >>= channel->scan_type.shift;

	return 0;
}

static int ads1018_read_raw(struct iio_dev *iio,
			struct iio_chan_spec const *channel, int *val1,
			int *val2, long mask)
{
	struct ads1018 *adc = iio_priv(iio);
	int err;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (adc->prefetch) {
			*val1 = adc->fetched[channel->channel];
			err = 0;
		} else
			err = ads1018_read_channel(adc, channel, val1);

		if (err < 0)
			return (err);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		if (adc->prefetch) {
			*val1 = adc->fetched[channel->channel];
			err = 0;
		} else
			err = ads1018_read_channel(adc, channel, val1);
		if (err < 0)
			return (err);

		if (channel->type == IIO_TEMP) {
			if ((signed short int )*val1 > 0)
				*val1 = *val1 * 1250;
			else
				*val1 = *val1 * -1250;
			*val1 = div_s64(*val1, 1000);
		}

		if (channel->type == IIO_CURRENT) {
			if (adc->offset[channel->channel] > 0)
				*val1 -= adc->offset[channel->channel];

			if (adc->scale[channel->channel])
				*val1 *= adc->scale[channel->channel];
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

static int ads1018_write_raw(struct iio_dev *iio,
			       struct iio_chan_spec const *channel,
			       int val, int val2, long mask)
{
	struct ads1018 *adc = iio_priv(iio);

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
	struct ads1018 *adc = container_of(work_arg, struct ads1018, fetch_work);
	struct iio_dev *indio_dev = spi_get_drvdata(adc->spi);
	struct iio_chan_spec const *chan;
	int i, err, val1;

	dev_info(adc->dev, "Starting prefetching on PID: [%d]\n",  current->pid);

	set_current_state(TASK_INTERRUPTIBLE);

	while (adc->prefetch) {
		chan = indio_dev->channels;
		for (i = 0; i < 2; i++) {
			err = ads1018_read_channel(adc, chan, &val1);
			if (!err) {
				adc->fetched[i] = val1;
			}
			msleep(adc->prefetch);
			chan++;
		}
	}
	return;
}

static ssize_t ads1018_set_prefetch(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct ads1018 *adc = iio_priv(dev_to_iio_dev(dev));
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

static ssize_t ads1018_show_prefetch(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ads1018 *adc = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", adc->prefetch);
}

static ssize_t ads1018_set_single_shot(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct ads1018 *adc = iio_priv(dev_to_iio_dev(dev));
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto error_ret;

	if (val < 0)
		return -EINVAL;

	if (val > 1)
		val = 1;

	adc->single_shot = val;

error_ret:
	return ret ? ret : len;
}

static ssize_t ads1018_show_single_shot(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ads1018 *adc = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", adc->single_shot);
}

static IIO_DEVICE_ATTR(prefetch_pause, (S_IWUSR | S_IRUGO),
		ads1018_show_prefetch, ads1018_set_prefetch, 0);
static IIO_DEVICE_ATTR(single_shot, (S_IWUSR | S_IRUGO),
		ads1018_show_single_shot, ads1018_set_single_shot, 0);

static struct attribute *ads1018_attributes[] = {
	&iio_dev_attr_prefetch_pause.dev_attr.attr,
	&iio_dev_attr_single_shot.dev_attr.attr,
	NULL,
};

static const char * const ads1018_sample_rate_modes[] = {
	"0_128SPS",
	"1_250SPS",
	"2_490SPS",
	"3_920SPS",
	"4_1600SPS",
	"5_2400SPS",
	"6_3300SPS"
};

static int ads1018_get_sample_rate(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct ads1018 *adc = iio_priv(indio_dev);

	return adc->sample_rate_mode;
}

static int ads1018_set_sample_rate(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	struct ads1018 *adc = iio_priv(indio_dev);

	adc->sample_rate_mode = mode;
	return 0;
}

static const struct attribute_group ads1018_attribute_group = {
	.attrs = ads1018_attributes,
};

static const struct iio_enum ads1018_sample_rate_mode_enum = {
	.items = ads1018_sample_rate_modes,
	.num_items = ARRAY_SIZE(ads1018_sample_rate_modes),
	.get = ads1018_get_sample_rate,
	.set = ads1018_set_sample_rate,
};

static const struct iio_chan_spec_ext_info ads1018_ext_info[] = {
	IIO_ENUM("sample_rate", IIO_SHARED_BY_ALL, &ads1018_sample_rate_mode_enum),
	IIO_ENUM_AVAILABLE("sample_rate", &ads1018_sample_rate_mode_enum),
	{ },
};

static const struct iio_chan_spec ads1018_channels[] = {
	ADS1018_CHAN(0, IIO_CURRENT,
			BIT(IIO_CHAN_INFO_RAW)|\
			BIT(IIO_CHAN_INFO_OFFSET)|\
			BIT(IIO_CHAN_INFO_SCALE)|\
			BIT(IIO_CHAN_INFO_PROCESSED)),
	ADS1018_CHAN(1, IIO_CURRENT,
			BIT(IIO_CHAN_INFO_RAW)|\
			BIT(IIO_CHAN_INFO_OFFSET)|\
			BIT(IIO_CHAN_INFO_SCALE)|\
			BIT(IIO_CHAN_INFO_PROCESSED)),
	ADS1018_CHAN(0, IIO_TEMP,
			BIT(IIO_CHAN_INFO_RAW)|\
			BIT(IIO_CHAN_INFO_PROCESSED)),
};

static const struct iio_info ads1018_info = {
	.read_raw = ads1018_read_raw,
	.write_raw = ads1018_write_raw,
	.attrs = &ads1018_attribute_group,
	.driver_module = THIS_MODULE,
};

#ifdef CONFIG_OF
static const struct of_device_id ads1018_of_match[] = {
	{ .compatible = "ti,ads1018" },
	{ }
};

static int ads1018_of_probe(struct spi_device *spi,
		struct ads1018 *adc)
{
	struct device *dev = &spi->dev;
	u32 val;

	if(of_property_read_u32(dev->of_node, "prefetch_pause", &val) >= 0) {
		if (val >= 0)
			adc->prefetch = val;
		else
			adc->prefetch = 0;
	}

	if(of_property_read_u32(dev->of_node, "single_shot", &val) >= 0)
		if (val == 0 || val == 1)
			adc->single_shot = val;

	return 0;
}
MODULE_DEVICE_TABLE(of, ads1018_of_match);
#else
static inline int ads1018_of_probe(struct spi_device *spi,
		struct ads1018 *pdata)
{
	return -ENODEV;
}
#endif

static int ads1018_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ads1018 *adc;
	int err;

	dev_info(&spi->dev, "Texas Instrument ADS1018 ADC - %s\n", DRIVER_VERSION);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	adc->dev = &spi->dev;
	adc->spi = spi;
	adc->prefetch = 0;
	adc->single_shot = 0;
	adc->sample_rate_mode = 3;
	adc->scale[0] = 10;
	adc->scale[1] = 10;

	if (spi->dev.of_node) {
		err = ads1018_of_probe(spi, adc);
		if (err)
			dev_err(adc->dev, "invalid devicetree data");
	}

	mutex_init(&adc->lock);

	indio_dev->dev.parent = adc->dev;
	indio_dev->name = dev_name(adc->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ads1018_info;
	indio_dev->channels = ads1018_channels;
	indio_dev->num_channels = ARRAY_SIZE(ads1018_channels);

	err = devm_iio_device_register(adc->dev, indio_dev);
	if (err < 0)
		return err;


	INIT_WORK(&adc->fetch_work, fetch_thread);
	if (adc->prefetch)
		schedule_work(&adc->fetch_work);

	return 0;
}

static int ads1018_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ads1018 *adc = iio_priv(indio_dev);

	adc->prefetch = 0;
	flush_work(&adc->fetch_work);
	cancel_work_sync(&adc->fetch_work);

	return 0;
}

static const struct spi_device_id ads1018_id[] = {
	{ "ads1018", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ads1018_id);

static struct spi_driver ads1018_driver = {
	.driver = {
		.name = "ads1018",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ads1018_of_match),
	},
	.probe = ads1018_probe,
	.remove = ads1018_remove,
	.id_table = ads1018_id,
};
module_spi_driver(ads1018_driver);

MODULE_AUTHOR("Robert Woerle <robert@linuxdevelopment.de>");
MODULE_VERSION(DRIVER_VERSION);
MODULE_DESCRIPTION("Texas Instrument ADS1018 ADC driver");
MODULE_LICENSE("GPL v2");
