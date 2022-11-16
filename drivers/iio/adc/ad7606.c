// SPDX-License-Identifier: GPL-2.0
/*
 * AD7606 SPI ADC driver
 *
 * Copyright 2011 Analog Devices Inc.
 */
/// RANGE PIN ???
/// AI6

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/util_macros.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#include "ad7606.h"

#define AD7606_MODULE_VERSION "1.3.2"

/*
 * Scales are computed as 5000/32768 and 10000/32768 respectively,
 * so that when applied to the raw values they provide mV values
 * also increase to have 2 factors for current meassured vals
 */
static const unsigned int ad7606_scale_avail[4] = {
	152588, 305176, 615192, 1230384,
};

static const unsigned int ad7606_oversampling_avail[7] = {
	1, 2, 4, 8, 16, 32, 64,
};

static const unsigned int ad7616_oversampling_avail[8] = {
	1, 2, 4, 8, 16, 32, 64, 128,
};

static int ad7606_reset(struct ad7606_state *st)
{
	dev_dbg(st->dev, "%s()\n", __func__);

	if (st->gpio_reset) {
		if (gpiod_cansleep(st->gpio_reset))
			gpiod_set_value_cansleep(st->gpio_reset, 1);
		else
			gpiod_set_value(st->gpio_reset, 1);
		ndelay(100); /* t_reset >= 100ns */
		if (gpiod_cansleep(st->gpio_reset))
			gpiod_set_value_cansleep(st->gpio_reset, 0);
		else
			gpiod_set_value(st->gpio_reset, 0);
		return 0;
	}

	return -ENODEV;
}

static int ad7606_read_samples(struct ad7606_state *st)
{
	unsigned int num = st->chip_info->num_channels -1;
	bool is_curr_n_volt = st->chip_info->is_curr_n_volt;
	u16 *data = st->data;
	int ret;

	dev_dbg(st->dev, "%s()\n", __func__);

	///HARDCODE FOR NOW FOR US
	if (is_curr_n_volt)
		num = 8;

	/*
	 * The frstdata signal is set to high while and after reading the sample
	 * of the first channel and low for all other channels. This can be used
	 * to check that the incoming data is correctly aligned. During normal
	 * operation the data should never become unaligned, but some glitch or
	 * electrostatic discharge might cause an extra read or clock cycle.
	 * Monitoring the frstdata signal allows to recover from such failure
	 * situations.
	 */

	if (st->gpio_frstdata) {
		ret = st->bops->read_block(st->dev, 1, data);
		if (ret)
			return ret;

		if (!gpiod_get_value(st->gpio_frstdata)) {
			ad7606_reset(st);
			return -EIO;
		}

		data++;
		num--;
	}

	return st->bops->read_block(st->dev, num, data);
}

static irqreturn_t ad7606_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad7606_state *st = iio_priv(indio_dev);
	int ret;

	dev_dbg(st->dev, "%s(%d)\n", __func__, irq);

	mutex_lock(&st->lock);

	ret = ad7606_read_samples(st);
	if (ret == 0)
		iio_push_to_buffers_with_timestamp(indio_dev, st->data,
						   iio_get_time_ns(indio_dev));

	iio_trigger_notify_done(indio_dev->trig);
	usleep_range(st->usec_sleep, st->usec_sleep+1);

	/* The rising edge of the CONVST signal starts a new conversion. */
	if (gpiod_cansleep(st->gpio_convst))
		gpiod_set_value_cansleep(st->gpio_convst, 1);
	else
		gpiod_set_value(st->gpio_convst, 1);

	mutex_unlock(&st->lock);

	return IRQ_HANDLED;
}

static int ad7606_scan_direct(struct iio_dev *indio_dev, unsigned int ch)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	int ret;

	dev_dbg(st->dev, "%s(%d)\n", __func__, ch);

	if (gpiod_cansleep(st->gpio_convst))
		gpiod_set_value_cansleep(st->gpio_convst, 1);
	else
		gpiod_set_value(st->gpio_convst, 1);
	ret = wait_for_completion_timeout(&st->completion,
					  msecs_to_jiffies(1000));
	if (!ret) {
		ret = -ETIMEDOUT;
		goto error_ret;
	}

	ret = ad7606_read_samples(st);
	if (ret == 0)
		ret = st->data[ch];

error_ret:
	if (gpiod_cansleep(st->gpio_convst))
		gpiod_set_value_cansleep(st->gpio_convst, 0);
	else
		gpiod_set_value(st->gpio_convst, 0);

	return ret;
}

static int ad7606_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	uint64_t ret;
	int pin_range;
	int is_neg = 0;
	struct ad7606_state *st = iio_priv(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_PROCESSED:

		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = ad7606_scan_direct(indio_dev, chan->address);
		iio_device_release_direct_mode(indio_dev);

		/// if we are set to CURRENT we zero a voltage read and vice versa
		if ((st->aixb[chan->address] && chan->type == IIO_CURRENT)
			|| (!st->aixb[chan->address] && chan->type == IIO_VOLTAGE) ) {
			*val = 0;
			return IIO_VAL_INT;
		}

		if (gpiod_cansleep(st->gpio_range))
			pin_range = gpiod_get_value_cansleep(st->gpio_range);
		else
			pin_range = gpiod_get_value(st->gpio_range);

		ret -= st->offset[chan->scan_index];

		if (chan->type == IIO_CURRENT)
			pin_range += 2;

		if ((short) ret < 0)
			is_neg = 1;
		ret = abs((short) ret);

		ret = ret * st->scale_avail[pin_range];
		do_div(ret, 1000000);

		/// factor with calibscale
		ret = ret * st->calibscale[chan->scan_index];
		do_div(ret, 100000);

		if (is_neg)
			*val = ((short) ret) * -1;
		else
			*val = (short)ret;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = ad7606_scan_direct(indio_dev, chan->address);
		iio_device_release_direct_mode(indio_dev);

		*val = (short)ret;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_OFFSET:
		*val = st->offset[chan->scan_index];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = st->scale_avail[st->range];
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = st->oversampling;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		*val = st->calibscale[chan->scan_index];
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static ssize_t ad7606_show_avail(char *buf, const unsigned int *vals,
				 unsigned int n, bool micros)
{
	size_t len = 0;
	int i;

	for (i = 0; i < n; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
			micros ? "0.%06u " : "%u ", vals[i]);
	}
	buf[len - 1] = '\n';

	return len;
}

static int ad7606_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	int values[3];
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		st->calibscale[chan->scan_index] = val;
		return 0;
	case IIO_CHAN_INFO_OFFSET:
		st->offset[chan->scan_index] = val;
		return 0;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		if (val2)
			return -EINVAL;

		if (val == 2)
			i = 1;
		else
			i = find_closest(val, st->oversampling_avail,
				 st->num_os_ratios);

		values[0] = (i & 0x01);
		values[1] = (i & 0x02) >> 1;
		values[2] = (i & 0x04) >> 2;

		mutex_lock(&st->lock);
		gpiod_set_array_value_cansleep(ARRAY_SIZE(values), st->gpio_os->desc, values);

		/* AD7616 requires a reset to update value */
		if (st->chip_info->os_req_reset)
			ad7606_reset(st);

		st->oversampling = st->oversampling_avail[i];
		mutex_unlock(&st->lock);

		return 0;
	default:
		return -EINVAL;
	}
}

static ssize_t ad7606_oversampling_ratio_avail(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7606_state *st = iio_priv(indio_dev);

	return ad7606_show_avail(buf, st->oversampling_avail,
				 st->num_os_ratios, false);
}

static IIO_DEVICE_ATTR(oversampling_ratio_available, 0444,
		       ad7606_oversampling_ratio_avail, NULL, 0);


static ssize_t ai1b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf, "%d\n", st->aixb[0]);
}

static ssize_t ai2b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf, "%d\n", st->aixb[1]);
}

static ssize_t ai3b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf, "%d\n", st->aixb[2]);
}

static ssize_t ai4b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf, "%d\n", st->aixb[3]);
}

static ssize_t ai5b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf, "%d\n", st->aixb[4]);
}

static ssize_t ai6b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf, "%d\n", st->aixb[5]);
}

static ssize_t ai7b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf, "%d\n", st->aixb[6]);
}

static ssize_t ai8b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));
	return sprintf(buf, "%d\n", st->aixb[7]);
}

static ssize_t aixb_set(struct device *dev,
		const char *buf, int id,
		size_t len)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto error_ret;

	if (val < 0)
		return -EINVAL;

	st->aixb[id] = val;
	gpiod_set_array_value_cansleep(ARRAY_SIZE(st->aixb), st->gpio_aixb->desc, st->aixb);

error_ret:
	return ret ? ret : len;
}

static ssize_t ai1b_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return aixb_set(dev, buf, 0, len);
}

static ssize_t ai2b_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return aixb_set(dev, buf, 1, len);
}

static ssize_t ai3b_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return aixb_set(dev, buf, 2, len);
}

static ssize_t ai4b_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return aixb_set(dev, buf, 3, len);
}

static ssize_t ai5b_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return aixb_set(dev, buf, 4, len);
}

static ssize_t ai6b_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return aixb_set(dev, buf, 5, len);
}

static ssize_t ai7b_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return aixb_set(dev, buf, 6, len);
}

static ssize_t ai8b_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return aixb_set(dev, buf, 7, len);
}

static ssize_t range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));
	int val = 0;

	if (gpiod_cansleep(st->gpio_range))
		val = gpiod_get_value_cansleep(st->gpio_range);
	else
		val = gpiod_get_value(st->gpio_range);

	return sprintf(buf, "%d\n", val);
}

static ssize_t range_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto error_ret;

	if (val < 0)
		return -EINVAL;

	if (gpiod_cansleep(st->gpio_range))
		gpiod_set_value_cansleep(st->gpio_range, val);

error_ret:
	return ret ? ret : len;
}


static ssize_t sleep_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->usec_sleep);
}

static ssize_t sleep_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct ad7606_state *st = iio_priv(dev_to_iio_dev(dev));
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto error_ret;

	if (val < 90)
		return -EINVAL;
	st->usec_sleep = val;

error_ret:
	return ret ? ret : len;
}


static IIO_DEVICE_ATTR(ai1b, (S_IWUSR | S_IRUGO),
		ai1b_show, ai1b_set, 0);
static IIO_DEVICE_ATTR(ai2b, (S_IWUSR | S_IRUGO),
		ai2b_show, ai2b_set, 0);
static IIO_DEVICE_ATTR(ai3b, (S_IWUSR | S_IRUGO),
		ai3b_show, ai3b_set, 0);
static IIO_DEVICE_ATTR(ai4b, (S_IWUSR | S_IRUGO),
		ai4b_show, ai4b_set, 0);
static IIO_DEVICE_ATTR(ai5b, (S_IWUSR | S_IRUGO),
		ai5b_show, ai5b_set, 0);
static IIO_DEVICE_ATTR(ai6b, (S_IWUSR | S_IRUGO),
		ai6b_show, ai6b_set, 0);
static IIO_DEVICE_ATTR(ai7b, (S_IWUSR | S_IRUGO),
		ai7b_show, ai7b_set, 0);
static IIO_DEVICE_ATTR(ai8b, (S_IWUSR | S_IRUGO),
		ai8b_show, ai8b_set, 0);
static IIO_DEVICE_ATTR(range, (S_IWUSR | S_IRUGO),
		range_show, range_set, 0);
static IIO_DEVICE_ATTR(sleep, (S_IWUSR | S_IRUGO),
		sleep_show, sleep_set, 0);

static struct attribute *ad7606_attributes_os_and_range[] = {
	&iio_dev_attr_oversampling_ratio_available.dev_attr.attr,
	&iio_dev_attr_ai1b.dev_attr.attr,
	&iio_dev_attr_ai2b.dev_attr.attr,
	&iio_dev_attr_ai3b.dev_attr.attr,
	&iio_dev_attr_ai4b.dev_attr.attr,
	&iio_dev_attr_ai5b.dev_attr.attr,
	&iio_dev_attr_ai6b.dev_attr.attr,
	&iio_dev_attr_ai7b.dev_attr.attr,
	&iio_dev_attr_ai8b.dev_attr.attr,
	&iio_dev_attr_range.dev_attr.attr,
	&iio_dev_attr_sleep.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad7606_attribute_group_os_and_range = {
	.attrs = ad7606_attributes_os_and_range,
};

static struct attribute *ad7606_attributes_os[] = {
	&iio_dev_attr_oversampling_ratio_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad7606_attribute_group_os = {
	.attrs = ad7606_attributes_os,
};

#define AD760X_CHANNEL(num, idx, typ, mask) {		\
		.type = typ,							\
		.indexed = 1,							\
		.channel = num,							\
		.address = num,							\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)					\
							| BIT(IIO_CHAN_INFO_PROCESSED)				\
							| BIT(IIO_CHAN_INFO_CALIBSCALE)				\
							| BIT(IIO_CHAN_INFO_OFFSET),				\
		.info_mask_shared_by_type = 0,									\
		.info_mask_shared_by_all = mask,								\
		.scan_index = idx,						\
		.scan_type = {							\
			.sign = 'u',						\
			.realbits = 16,						\
			.storagebits = 16,					\
			.endianness = IIO_CPU,				\
		},										\
}

#define AD7605_CHANNEL(num, idx, typ)	\
	AD760X_CHANNEL(num, idx, typ, 0)

#define AD7606_CHANNEL(num, idx, typ)	\
	AD760X_CHANNEL(num, idx, typ, BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO))

static const struct iio_chan_spec ad7605_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(4),
	AD7605_CHANNEL(0, 0, IIO_VOLTAGE),
	AD7605_CHANNEL(1, 1, IIO_VOLTAGE),
	AD7605_CHANNEL(2, 2, IIO_VOLTAGE),
	AD7605_CHANNEL(3, 3, IIO_VOLTAGE),
};

static const struct iio_chan_spec ad7606_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(16),
	AD7606_CHANNEL(0, 0, IIO_VOLTAGE),
	AD7606_CHANNEL(1, 1, IIO_VOLTAGE),
	AD7606_CHANNEL(2, 2, IIO_VOLTAGE),
	AD7606_CHANNEL(3, 3, IIO_VOLTAGE),
	AD7606_CHANNEL(4, 4, IIO_VOLTAGE),
	AD7606_CHANNEL(5, 5, IIO_VOLTAGE),
	AD7606_CHANNEL(6, 6, IIO_VOLTAGE),
	AD7606_CHANNEL(7, 7, IIO_VOLTAGE),
	AD7606_CHANNEL(0, 8, IIO_CURRENT),
	AD7606_CHANNEL(1, 9, IIO_CURRENT),
	AD7606_CHANNEL(2, 10, IIO_CURRENT),
	AD7606_CHANNEL(3, 11, IIO_CURRENT),
	AD7606_CHANNEL(4, 12, IIO_CURRENT),
	AD7606_CHANNEL(5, 13, IIO_CURRENT),
	AD7606_CHANNEL(6, 14, IIO_CURRENT),
	AD7606_CHANNEL(7, 15, IIO_CURRENT),
};

/*
 * The current assumption that this driver makes for AD7616, is that it's
 * working in Hardware Mode with Serial, Burst and Sequencer modes activated.
 * To activate them, following pins must be pulled high:
 *	-SER/PAR
 *	-SEQEN
 * And following pins must be pulled low:
 *	-WR/BURST
 *	-DB4/SER1W
 */
static const struct iio_chan_spec ad7616_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(16),
	AD7606_CHANNEL(0, 0, IIO_VOLTAGE),
	AD7606_CHANNEL(1, 1, IIO_VOLTAGE),
	AD7606_CHANNEL(2, 2, IIO_VOLTAGE),
	AD7606_CHANNEL(3, 3, IIO_VOLTAGE),
	AD7606_CHANNEL(4, 4, IIO_VOLTAGE),
	AD7606_CHANNEL(5, 5, IIO_VOLTAGE),
	AD7606_CHANNEL(6, 6, IIO_VOLTAGE),
	AD7606_CHANNEL(7, 7, IIO_VOLTAGE),
	AD7606_CHANNEL(8, 8, IIO_VOLTAGE),
	AD7606_CHANNEL(9, 9, IIO_VOLTAGE),
	AD7606_CHANNEL(10, 10, IIO_VOLTAGE),
	AD7606_CHANNEL(11, 11, IIO_VOLTAGE),
	AD7606_CHANNEL(12, 12, IIO_VOLTAGE),
	AD7606_CHANNEL(13, 13, IIO_VOLTAGE),
	AD7606_CHANNEL(14, 14, IIO_VOLTAGE),
	AD7606_CHANNEL(15, 15, IIO_VOLTAGE),
};

static const struct ad7606_chip_info ad7606_chip_info_tbl[] = {
	/* More devices added in future */
	[ID_AD7605_4] = {
		.channels = ad7605_channels,
		.num_channels = 5,
	},
	[ID_AD7606_8] = {
		.channels = ad7606_channels,
		.num_channels = 17,
		.oversampling_avail = ad7606_oversampling_avail,
		.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
		.is_curr_n_volt = true,
	},
	[ID_AD7606_6] = {
		.channels = ad7606_channels,
		.num_channels = 7,
		.oversampling_avail = ad7606_oversampling_avail,
		.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	},
	[ID_AD7606_4] = {
		.channels = ad7606_channels,
		.num_channels = 5,
		.oversampling_avail = ad7606_oversampling_avail,
		.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	},
	[ID_AD7616] = {
		.channels = ad7616_channels,
		.num_channels = 17,
		.oversampling_avail = ad7616_oversampling_avail,
		.oversampling_num = ARRAY_SIZE(ad7616_oversampling_avail),
		.os_req_reset = true,
	},
};

static int ad7606_request_gpios(struct ad7606_state *st)
{
	struct device *dev = st->dev;

	st->gpio_convst = devm_gpiod_get(dev, "adi,conversion-start",
					 GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_convst))
		return PTR_ERR(st->gpio_convst);

	st->gpio_reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	st->gpio_range = devm_gpiod_get_optional(dev, "adi,range", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_range))
		return PTR_ERR(st->gpio_range);

	st->gpio_standby = devm_gpiod_get_optional(dev, "standby",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_standby))
		return PTR_ERR(st->gpio_standby);

	st->gpio_frstdata = devm_gpiod_get_optional(dev, "adi,first-data",
						    GPIOD_IN);
	if (IS_ERR(st->gpio_frstdata))
		return PTR_ERR(st->gpio_frstdata);

	if (!st->chip_info->oversampling_num)
		return 0;

	st->gpio_os = devm_gpiod_get_array_optional(dev,
						    "adi,oversampling-ratio",
						    GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_os))
		return PTR_ERR(st->gpio_os);

	st->gpio_aixb = devm_gpiod_get_array_optional(dev,
						    "adi,aixb-i",
						    GPIOD_OUT_LOW);
	return PTR_ERR_OR_ZERO(st->gpio_aixb);
}

/*
 * The BUSY signal indicates when conversions are in progress, so when a rising
 * edge of CONVST is applied, BUSY goes logic high and transitions low at the
 * end of the entire conversion process. The falling edge of the BUSY signal
 * triggers this interrupt.
 */
static irqreturn_t ad7606_interrupt(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct ad7606_state *st = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev)) {
		if (gpiod_cansleep(st->gpio_convst))
			gpiod_set_value_cansleep(st->gpio_convst, 0);
		else
			gpiod_set_value(st->gpio_convst, 0);
		iio_trigger_poll_chained(st->trig);
	} else {
		complete(&st->completion);
	}

	return IRQ_HANDLED;
};

static int ad7606_validate_trigger(struct iio_dev *indio_dev,
				   struct iio_trigger *trig)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	dev_info(st->dev, "%s()\n", __func__);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static int ad7606_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	iio_triggered_buffer_postenable(indio_dev);
	if (gpiod_cansleep(st->gpio_convst))
		gpiod_set_value_cansleep(st->gpio_convst, 1);
	else
		gpiod_set_value(st->gpio_convst, 1);

	return 0;
}

static int ad7606_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	if (gpiod_cansleep(st->gpio_convst))
		gpiod_set_value_cansleep(st->gpio_convst, 0);
	else
		gpiod_set_value(st->gpio_convst, 0);

	return iio_triggered_buffer_predisable(indio_dev);
}

static const struct iio_buffer_setup_ops ad7606_buffer_ops = {
	.postenable = &ad7606_buffer_postenable,
	.predisable = &ad7606_buffer_predisable,
};

static const struct iio_info ad7606_info_no_os_or_range = {
	.read_raw = &ad7606_read_raw,
	.validate_trigger = &ad7606_validate_trigger,
};

static const struct iio_info ad7606_info_os_and_range = {
	.read_raw = &ad7606_read_raw,
	.write_raw = &ad7606_write_raw,
	.attrs = &ad7606_attribute_group_os_and_range,
	.validate_trigger = &ad7606_validate_trigger,
};

static const struct iio_info ad7606_info_os = {
	.read_raw = &ad7606_read_raw,
	.write_raw = &ad7606_write_raw,
	.attrs = &ad7606_attribute_group_os,
	.validate_trigger = &ad7606_validate_trigger,
};

static const struct iio_info ad7606_info_range = {
	.read_raw = &ad7606_read_raw,
	.write_raw = &ad7606_write_raw,
	.attrs = &ad7606_attribute_group_os,
	.validate_trigger = &ad7606_validate_trigger,
};

static const struct iio_trigger_ops ad7606_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};

static void ad7606_regulator_disable(void *data)
{
	struct ad7606_state *st = data;

	regulator_disable(st->reg);
}

int ad7606_probe(struct device *dev, int irq, void __iomem *base_address,
		 const char *name, unsigned int id,
		 const struct ad7606_bus_ops *bops)
{
	struct ad7606_state *st;
	int ret, i;
	struct iio_dev *indio_dev;

	dev_info(dev, "%s() %s\n", __func__, AD7606_MODULE_VERSION);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	st->dev = dev;
	mutex_init(&st->lock);
	st->bops = bops;
	st->base_address = base_address;
	/* tied to logic low, analog input range is +/- 5V */
	st->range = 0;
	st->oversampling = 1;
	st->usec_sleep = 930;

	st->scale_avail = ad7606_scale_avail;

	st->reg = devm_regulator_get(dev, "avcc");
	if (IS_ERR(st->reg))
		return PTR_ERR(st->reg);

	ret = regulator_enable(st->reg);
	if (ret) {
		dev_err(dev, "Failed to enable specified AVcc supply\n");
		return ret;
	}

	ret = devm_add_action_or_reset(dev, ad7606_regulator_disable, st);
	if (ret)
		return ret;

	st->chip_info = &ad7606_chip_info_tbl[id];

	if (st->chip_info->oversampling_num) {
		st->oversampling_avail = st->chip_info->oversampling_avail;
		st->num_os_ratios = st->chip_info->oversampling_num;
	}

	for (i = 0; i < 16; i++)
		st->calibscale[i] = 100000;

	ret = ad7606_request_gpios(st);
	if (ret)
		return ret;

	indio_dev->dev.parent = dev;
	if (st->gpio_os) {
		if (st->gpio_range)
			indio_dev->info = &ad7606_info_os_and_range;
		else
			indio_dev->info = &ad7606_info_os;
	} else {
		if (st->gpio_range)
			indio_dev->info = &ad7606_info_range;
		else
			indio_dev->info = &ad7606_info_no_os_or_range;
	}
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = name;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	init_completion(&st->completion);

	ret = ad7606_reset(st);
	if (ret)
		dev_warn(st->dev, "failed to RESET: no RESET GPIO specified\n");

	st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
					  indio_dev->name, indio_dev->id);
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &ad7606_trigger_ops;
	st->trig->dev.parent = dev;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(dev, st->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trig);

	ret = devm_request_threaded_irq(dev, irq,
					NULL,
					&ad7606_interrupt,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					name, indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &ad7606_trigger_handler,
					      &ad7606_buffer_ops);
	if (ret)
		return ret;

	ad7606_reset(st);

	return devm_iio_device_register(dev, indio_dev);
}
EXPORT_SYMBOL_GPL(ad7606_probe);

#ifdef CONFIG_PM_SLEEP

static int ad7606_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad7606_state *st = iio_priv(indio_dev);

	if (st->gpio_standby) {
		if (gpiod_cansleep(st->gpio_standby)) {
			gpiod_set_value_cansleep(st->gpio_range, 1);
			gpiod_set_value_cansleep(st->gpio_standby, 0);
		} else {
			gpiod_set_value(st->gpio_range, 1);
			gpiod_set_value(st->gpio_standby, 0);
		}
	}
	return 0;
}

static int ad7606_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad7606_state *st = iio_priv(indio_dev);

	if (st->gpio_standby) {
		if (gpiod_cansleep(st->gpio_standby)) {
			gpiod_set_value_cansleep(st->gpio_range, st->range);
			gpiod_set_value_cansleep(st->gpio_standby, 1);
		} else {
			gpiod_set_value(st->gpio_range, st->range);
			gpiod_set_value(st->gpio_standby, 1);
		}
		ad7606_reset(st);
	}

	return 0;
}

SIMPLE_DEV_PM_OPS(ad7606_pm_ops, ad7606_suspend, ad7606_resume);
EXPORT_SYMBOL_GPL(ad7606_pm_ops);

#endif

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7606 ADC");
MODULE_LICENSE("GPL v2");
