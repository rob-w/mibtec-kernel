// SPDX-License-Identifier: GPL-2.0
/*
 * a PRU ADC SPI driver via remoteproc
 *
 * Copyright 2019 MIS (c) Robert Woerle rwoerle@mibtec.de
 * based on TI PRU Example codes and ad7606.c 
 */

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
#include <linux/of.h>
#include <linux/platform_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define DRIVER_VERSION "v1.1"

struct pru_chip_info {
	const struct iio_chan_spec	*channels;
	unsigned int			num_channels;
	const unsigned int		*oversampling_avail;
	unsigned int			oversampling_num;
	bool				os_req_reset;
};

struct pru_state {
	int 					state;
	struct device			*dev;
	const struct pru_chip_info	*chip_info;
	struct regulator		*reg;
	unsigned int			range;
	unsigned int			oversampling;
	void __iomem			*base_address;
	const unsigned int		*scale_avail;
	unsigned int			num_scales;
	const unsigned int		*oversampling_avail;
	unsigned int			num_os_ratios;

	struct mutex			lock; /* protect sensor state */
	struct gpio_desc		*gpio_io_en;
	struct iio_trigger		*trig;
	struct completion		completion;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 * 16 * 16-bit samples + 64-bit timestamp
	 */
	unsigned short			data[20] ____cacheline_aligned;
};


static const unsigned int pru_scale_avail[2] = {
	152588, 305176
};

static const unsigned int pru_oversampling_avail[7] = {
	1, 2, 4, 8, 16, 32, 64,
};

static int pru_reset(struct pru_state *st)
{
	ndelay(100); /* t_reset >= 100ns */
	return 0;
}

static int pru_read_samples(struct pru_state *st)
{
//	unsigned int num = st->chip_info->num_channels -1;
//	u16 *data = st->data;
	int ret = 0;

//	return st->bops->read_block(st->dev, num, data);
	return ret;
}

static irqreturn_t pru_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct pru_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	ret = pru_read_samples(st);
//	if (ret == 0)
//		iio_push_to_buffers_with_timestamp(indio_dev, st->data,
//						   iio_get_time_ns(indio_dev));

	iio_trigger_notify_done(indio_dev->trig);
	mutex_unlock(&st->lock);

	return IRQ_HANDLED;
}

static int pru_scan_direct(struct iio_dev *indio_dev, unsigned int ch)
{
	struct pru_state *st = iio_priv(indio_dev);
	int ret;

	ret = pru_read_samples(st);
	if (ret == 0)
		ret = st->data[ch];

	return ret;
}

static int pru_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	int ret;
	struct pru_state *st = iio_priv(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = pru_scan_direct(indio_dev, chan->address);
		iio_device_release_direct_mode(indio_dev);

		if (ret < 0)
			return ret;
		*val = (short)ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = st->scale_avail[st->range];
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = st->oversampling;
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static ssize_t pru_show_avail(char *buf, const unsigned int *vals,
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

static ssize_t in_voltage_scale_available_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pru_state *st = iio_priv(indio_dev);

	return pru_show_avail(buf, st->scale_avail, st->num_scales, true);
}

static IIO_DEVICE_ATTR_RO(in_voltage_scale_available, 0);

static int pru_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct pru_state *st = iio_priv(indio_dev);
	DECLARE_BITMAP(values, 3);
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&st->lock);
		i = find_closest(val2, st->scale_avail, st->num_scales);
		st->range = i;
		mutex_unlock(&st->lock);

		return 0;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		if (val2)
			return -EINVAL;
		i = find_closest(val, st->oversampling_avail,
				 st->num_os_ratios);

		values[0] = i;

		mutex_lock(&st->lock);

		if (st->chip_info->os_req_reset)
			pru_reset(st);

		st->oversampling = st->oversampling_avail[i];
		mutex_unlock(&st->lock);

		return 0;
	default:
		return -EINVAL;
	}
}

static ssize_t pru_oversampling_ratio_avail(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pru_state *st = iio_priv(indio_dev);

	return pru_show_avail(buf, st->oversampling_avail,
				 st->num_os_ratios, false);
}

static IIO_DEVICE_ATTR(oversampling_ratio_available, 0444,
		       pru_oversampling_ratio_avail, NULL, 0);

static struct attribute *pru_attributes_os_and_range[] = {
	&iio_dev_attr_in_voltage_scale_available.dev_attr.attr,
	&iio_dev_attr_oversampling_ratio_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group pru_attribute_group_os_and_range = {
	.attrs = pru_attributes_os_and_range,
};

static struct attribute *pru_attributes_os[] = {
	&iio_dev_attr_oversampling_ratio_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group pru_attribute_group_os = {
	.attrs = pru_attributes_os,
};

static struct attribute *pru_attributes_range[] = {
	&iio_dev_attr_in_voltage_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group pru_attribute_group_range = {
	.attrs = pru_attributes_range,
};

#define PRUX_CHANNEL(num, mask) {				\
		.type = IIO_VOLTAGE,				\
		.indexed = 1,					\
		.channel = num,					\
		.address = num,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),\
		.info_mask_shared_by_all = mask,		\
		.scan_index = num,				\
		.scan_type = {					\
			.sign = 's',				\
			.realbits = 16,				\
			.storagebits = 16,			\
			.endianness = IIO_CPU,			\
		},						\
}

#define PRU_CHANNEL(num)	\
	PRUX_CHANNEL(num, BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO))

static const struct iio_chan_spec pru_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(8),
	PRU_CHANNEL(0),
	PRU_CHANNEL(1),
	PRU_CHANNEL(2),
	PRU_CHANNEL(3),
	PRU_CHANNEL(4),
	PRU_CHANNEL(5),
	PRU_CHANNEL(6),
	PRU_CHANNEL(7),
};


static const struct pru_chip_info pru_chip_info_tbl[] = {
	[0] = {
		.channels = pru_channels,
		.num_channels = 9,
		.oversampling_avail = pru_oversampling_avail,
		.oversampling_num = ARRAY_SIZE(pru_oversampling_avail),
	},
};

static int pru_request_gpios(struct pru_state *st)
{
	struct device *dev = st->dev;

	st->gpio_io_en = devm_gpiod_get(dev, "pru,io-enable",
					 GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_io_en))
		return PTR_ERR(st->gpio_io_en);
	return 0;
}

static int pru_validate_trigger(struct iio_dev *indio_dev,
				   struct iio_trigger *trig)
{
	struct pru_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static int pru_buffer_postenable(struct iio_dev *indio_dev)
{
	struct pru_state *st = iio_priv(indio_dev);

	iio_triggered_buffer_postenable(indio_dev);

	return 0;
}

static int pru_buffer_predisable(struct iio_dev *indio_dev)
{
	struct pru_state *st = iio_priv(indio_dev);

	return iio_triggered_buffer_predisable(indio_dev);
}

static const struct iio_buffer_setup_ops pru_buffer_ops = {
	.postenable = &pru_buffer_postenable,
	.predisable = &pru_buffer_predisable,
};

static const struct iio_info pru_info_no_os_or_range = {
	.read_raw = &pru_read_raw,
	.validate_trigger = &pru_validate_trigger,
};

static const struct iio_info pru_info_os_and_range = {
	.read_raw = &pru_read_raw,
	.write_raw = &pru_write_raw,
	.attrs = &pru_attribute_group_os_and_range,
	.validate_trigger = &pru_validate_trigger,
};

static const struct iio_info pru_info_os = {
	.read_raw = &pru_read_raw,
	.write_raw = &pru_write_raw,
	.attrs = &pru_attribute_group_os,
	.validate_trigger = &pru_validate_trigger,
};

static const struct iio_info pru_info_range = {
	.read_raw = &pru_read_raw,
	.write_raw = &pru_write_raw,
	.attrs = &pru_attribute_group_range,
	.validate_trigger = &pru_validate_trigger,
};

static const struct iio_trigger_ops pru_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};

static const struct of_device_id of_pru_adc_spi_match[] = {
	{ .compatible = "pru-adc-spi", },
	{},
};

MODULE_DEVICE_TABLE(of, of_pru_adc_spi_match);

static int pru_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pru_state *st;
	int ret;
	struct iio_dev *indio_dev;

	printk("pru-adc-spi: probe() 0.2\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	st->dev = dev;
	mutex_init(&st->lock);
	st->range = 0;
	st->oversampling = 1;
	st->scale_avail = pru_scale_avail;
	st->num_scales = ARRAY_SIZE(pru_scale_avail);

	st->chip_info = &pru_chip_info_tbl[0];

	if (st->chip_info->oversampling_num) {
		st->oversampling_avail = st->chip_info->oversampling_avail;
		st->num_os_ratios = st->chip_info->oversampling_num;
	}

	ret = pru_request_gpios(st);
	if (ret)
		return ret;

	indio_dev->dev.parent = dev;
	indio_dev->info = &pru_info_range;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = "pru-adc-spi";
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	init_completion(&st->completion);

	ret = pru_reset(st);
	if (ret)
		dev_warn(st->dev, "failed to RESET: no RESET GPIO specified\n");

	st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
					  indio_dev->name, indio_dev->id);
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &pru_trigger_ops;
	st->trig->dev.parent = dev;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(dev, st->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trig);

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &pru_trigger_handler,
					      &pru_buffer_ops);
	if (ret)
		return ret;

	gpiod_set_value(st->gpio_io_en, 0);

	return devm_iio_device_register(dev, indio_dev);
}

#ifdef CONFIG_PM_SLEEP

static int pru_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct pru_state *st = iio_priv(indio_dev);
	st->state = 0;

	return 0;
}

static int pru_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct pru_state *st = iio_priv(indio_dev);
	st->state = 1;

	return 0;
}

SIMPLE_DEV_PM_OPS(pru_pm_ops, pru_suspend, pru_resume);
EXPORT_SYMBOL_GPL(pru_pm_ops);

#endif

static void pru_shutdown(struct platform_device *pdev)
{

}

static struct platform_driver pru_adc_spi = {
	.probe		= pru_probe,
	.shutdown	= pru_shutdown,
	.driver		= {
		.name	= "pru-adc-spi",
		.of_match_table = of_pru_adc_spi_match,
	},
};

module_platform_driver(pru_adc_spi);

MODULE_AUTHOR("Robert Woerle <rwoerle@mibtec.de>");
MODULE_DESCRIPTION("PRU ADC SPI remoteproc skeleton");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pru-adc-spi");
