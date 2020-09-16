// SPDX-License-Identifier: GPL-2.0
/*
 * IIO Counter implementation omap dmtimer block
 *
 * Copyright 2020 MIS (c) Robert Woerle rwoerle@mibtec.de, robert@linuxdevelopment.de
 * based/inspired on drivers/pps/client/pps-dmtimer.c apart from other iio drivers
 * 
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/sysfs.h>
#include <linux/util_macros.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/platform_device.h>
#include <linux/platform_data/dmtimer-omap.h>

#include <linux/clocksource.h>
#include <clocksource/timer-ti-dm.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define CNT_GPIO_VERSION "0.1"

struct cnt_gpio_info {
	const struct iio_chan_spec	*channels;
	unsigned int				num_channels;
};

struct cnt_gpio_pdata {
	int 					state;
	int						gate_time;	// in usec ?
	int						prescaler;
	uint32_t				t_delta;
	int						offset;
	struct device			*dev;
	struct iio_dev			*indio_dev;
	struct platform_device	*pdev;
	const struct cnt_gpio_info	*chip_info;

	struct mutex			lock;
	struct iio_trigger		*trig;

	uint32_t frequency;
	int ready;
};

static irqreturn_t cnt_gpio_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);
	dev_info(st->dev, "%s()\n", __func__);

	mutex_lock(&st->lock);
//	st->bufferd = 1;

	iio_trigger_notify_done(indio_dev->trig);
	mutex_unlock(&st->lock);

	return IRQ_HANDLED;
}

static int cnt_gpio_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	uint64_t ret, ret2;
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s(%ld %ld)\n", __func__, chan->address, m);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		*val = st->t_delta;
		iio_device_release_direct_mode(indio_dev);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		*val = st->frequency;
		*val2 = 0;

		iio_device_release_direct_mode(indio_dev);
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_OFFSET:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		*val = st->offset;
		iio_device_release_direct_mode(indio_dev);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = st->prescaler;
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int cnt_gpio_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		st->offset = val;
		return 0;
	}

	return -EINVAL;
}

static ssize_t cnt_gpio_show_gatetime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cnt_gpio_pdata *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->gate_time);
}

static ssize_t cnt_gpio_set_gatetime(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct cnt_gpio_pdata *st = iio_priv(dev_to_iio_dev(dev));
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto error_ret;

	if (val < 0)
		return -EINVAL;

	st->gate_time = val;

error_ret:
	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(gatetime, (S_IWUSR | S_IRUGO),
		cnt_gpio_show_gatetime, cnt_gpio_set_gatetime, 0);

static struct attribute *cnt_gpio_attributes[] = {
	&iio_dev_attr_gatetime.dev_attr.attr,
	NULL,
};

static const struct attribute_group cnt_gpio_attribute_group = {
	.attrs = cnt_gpio_attributes,
};

static const struct iio_info cnt_gpio_info = {
	.read_raw = cnt_gpio_read_raw,
	.write_raw = cnt_gpio_write_raw,
	.attrs = &cnt_gpio_attribute_group,
};

#define CNT_GPIO_CHANNEL(num) {								\
		.type = IIO_COUNT,										\
		.indexed = 1,											\
		.channel = num,											\
		.address = num,											\
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED)		\
							| BIT(IIO_CHAN_INFO_RAW)			\
							| BIT(IIO_CHAN_INFO_OFFSET)			\
							| BIT(IIO_CHAN_INFO_SCALE),			\
		.scan_index = num,										\
		.scan_type = {											\
			.sign = 'u',										\
			.realbits = 32,										\
			.storagebits = 32,									\
			.endianness = IIO_CPU,								\
		},														\
}

static const struct iio_chan_spec cnt_gpio_channels[] = {
	CNT_GPIO_CHANNEL(0),
	CNT_GPIO_CHANNEL(1),
	CNT_GPIO_CHANNEL(2),
	CNT_GPIO_CHANNEL(3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
};

static const struct cnt_gpio_info cnt_gpio_info_tbl[] = {
	[0] = {
		.channels = cnt_gpio_channels,
		.num_channels = 5,
	},
};

static int cnt_gpio_buffer_preenable(struct iio_dev *indio_dev)
{
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);

	return 0;
}

static int cnt_gpio_buffer_postenable(struct iio_dev *indio_dev)
{
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
	iio_triggered_buffer_postenable(indio_dev);

	return 0;
}

static int cnt_gpio_buffer_predisable(struct iio_dev *indio_dev)
{
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
//	st->bufferd = 0;

	return iio_triggered_buffer_predisable(indio_dev);
}

static int cnt_gpio_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);

	return 0;
}

static int cnt_gpio_trigger_set_state(struct iio_trigger *trig, bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s(%d)\n", __func__, enable);

	return 0;
}

static const struct iio_buffer_setup_ops cnt_gpio_buffer_ops = {
	.preenable = &cnt_gpio_buffer_preenable,
	.postenable = &cnt_gpio_buffer_postenable,
	.predisable = &cnt_gpio_buffer_predisable,
	.postdisable = &cnt_gpio_buffer_postdisable,
};

static const struct iio_trigger_ops cnt_gpio_trigger_ops = {
    .set_trigger_state = cnt_gpio_trigger_set_state,
	.validate_device = iio_trigger_validate_own_device,
};

static const struct of_device_id of_cnt_gpio_match[] = {
	{ .compatible = "cnt-gpio", },
	{},
};

MODULE_DEVICE_TABLE(of, of_cnt_gpio_match);

static int cnt_gpio_request_of(struct iio_dev *indio_dev)
{
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);
	struct device *dev = st->dev;

	st->ready = 0;

	return 0;
}

static int cnt_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cnt_gpio_pdata *st;
	struct iio_dev *indio_dev;
	int ret;

	dev_info(dev, "%s\n", CNT_GPIO_VERSION);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	st->dev = dev;
	st->pdev = pdev;

	mutex_init(&st->lock);

	st->chip_info = &cnt_gpio_info_tbl[0];
	st->offset = 1352;

	ret = cnt_gpio_request_of(indio_dev);
	if (ret)
		return -ENOMEM;

	indio_dev->dev.parent = dev;
	indio_dev->info = &cnt_gpio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = "cnt-gpio";
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
					  indio_dev->name, indio_dev->id);
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &cnt_gpio_trigger_ops;
	st->trig->dev.parent = dev;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(dev, st->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trig);

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &cnt_gpio_trigger_handler,
					      &cnt_gpio_buffer_ops);
	if (ret)
		return ret;

	st->ready = 1;

	return devm_iio_device_register(dev, indio_dev);
}

#ifdef CONFIG_PM_SLEEP

static int cnt_gpio_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);
	st->state = 0;

	return 0;
}

static int cnt_gpio_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);
	st->state = 1;

	return 0;
}

SIMPLE_DEV_PM_OPS(cnt_gpio_pm_ops, cnt_gpio_suspend, cnt_gpio_resume);
EXPORT_SYMBOL_GPL(cnt_gpio_pm_ops);

#endif

static int cnt_gpio_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
	st->state = 0;
	return 0;
}

static struct platform_driver cnt_gpio = {
	.probe		= cnt_gpio_probe,
	.remove		= cnt_gpio_remove,
	.driver		= {
		.name	= "cnt-gpio",
		.of_match_table = of_cnt_gpio_match,
	},
};

module_platform_driver(cnt_gpio);
MODULE_AUTHOR("Robert Wörle <rwoerle@mibtec.de>");
MODULE_DESCRIPTION("gpio counter driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(CNT_GPIO_VERSION);
MODULE_ALIAS("platform:cnt-gpio");
