// SPDX-License-Identifier: GPL-2.0
/*
 * IIO Counter implementation via gpio and/or omap gmtimer block
 *
 * Copyright 2020 MIS (c) Robert Woerle rwoerle@mibtec.de, robert@linuxdevelopment.de
 * based pps-gmtimer.c
 * 
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/rpmsg.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/util_macros.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <clocksource/timer-ti-dm.h>
#include <linux/platform_data/dmtimer-omap.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

//#define CREATE_TRACE_POINTS
//#include <trace/events/gpio.h>

#define CNT_GMTIMER_VERSION "0.9"
//#define CNT_GMTIMER_MODULE_DESCRIPTION "Omap Timer counter driver"
//#define CNT_GMTIMER_MODULE_AUTHOR("Robert Wörle");

struct cnt_gmtimer_info {
	const struct iio_chan_spec	*channels;
	unsigned int				num_channels;
	const unsigned int			*oversampling_avail;
	unsigned int				oversampling_num;
};

struct cnt_gmtimer_pdata {
	int 					state, bufferd;
	int						gate_time;	// in usec ?
	int						cnted;
	int						looped;
	int						id;
	struct device			*dev;
	struct iio_dev			*indio_dev;
	const struct cnt_gmtimer_info	*chip_info;
	struct pruss			*pruss;
	struct rproc			*rproc;
	struct regulator		*reg;
	unsigned int			range;
	unsigned int			oversampling;
	void __iomem			*base_address;
	const unsigned int		*oversampling_avail;
	unsigned int			num_os_ratios;

	struct mutex			lock; /* protect sensor state */
	struct gpio_desc		*gpio_mux_a[6];
	struct iio_trigger		*trig;
	struct completion		completion;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 *  6 * 16-bit samples + 64-bit timestamp
	 */
//	unsigned short			data[10] ____cacheline_aligned;
	unsigned int			data[452] ____cacheline_aligned;
};

static int cnt_gmtimer_read_samples(struct iio_dev *indio_dev, int cnt)
{
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);
	int ret = 0;

	st->cnted = 0;

	return ret;
}

static irqreturn_t cnt_gmtimer_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);
	dev_dbg(st->dev, "%s()\n", __func__);

	mutex_lock(&st->lock);
	st->bufferd = 1;
	cnt_gmtimer_read_samples(indio_dev, 0);
	iio_trigger_notify_done(indio_dev->trig);
	mutex_unlock(&st->lock);

	return IRQ_HANDLED;
}

static int cnt_gmtimer_scan_direct(struct iio_dev *indio_dev, unsigned int ch)
{
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);
	int ret;

	dev_dbg(st->dev, "%s(%d)\n", __func__, ch);

	ret = cnt_gmtimer_read_samples(indio_dev, 1000);
	if (ret != 0)
		return -EINVAL;

//	ret = wait_for_completion_timeout(&st->completion,
//					msecs_to_jiffies(1000));
	if (!ret)
		return -ETIMEDOUT;

	return st->data[ch];
}

static int cnt_gmtimer_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	uint64_t ret;
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s(%ld %ld)\n", __func__, chan->address, m);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = cnt_gmtimer_scan_direct(indio_dev, chan->address);
		iio_device_release_direct_mode(indio_dev);
		if (ret == -ETIMEDOUT)
			return ret;

		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_ENABLE:
		*val = 0;
		if (gpiod_get_value(st->gpio_mux_a[chan->address]))
			*val |= (1<<0);
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int cnt_gmtimer_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);
	int i;

	dev_dbg(st->dev, "%s(%ld)\n", __func__, chan->address);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		if (val & (1<<0))
			gpiod_set_value(st->gpio_mux_a[chan->address], 1);
		else
			gpiod_set_value(st->gpio_mux_a[chan->address], 0);
		return 0;
	default:
		return -EINVAL;
	}
}

static ssize_t cnt_gmtimer_show_gatetime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cnt_gmtimer_pdata *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->gate_time);
}

static ssize_t cnt_gmtimer_set_gatetime(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct cnt_gmtimer_pdata *st = iio_priv(dev_to_iio_dev(dev));
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
		cnt_gmtimer_show_gatetime, cnt_gmtimer_set_gatetime, 0);

static struct attribute *cnt_gmtimer_attributes[] = {
	&iio_dev_attr_gatetime.dev_attr.attr,
	NULL,
};

static const struct attribute_group cnt_gmtimer_attribute_group = {
	.attrs = cnt_gmtimer_attributes,
};

static const struct iio_info cnt_gmtimer_info = {
	.read_raw = cnt_gmtimer_read_raw,
	.write_raw = cnt_gmtimer_write_raw,
	.attrs = &cnt_gmtimer_attribute_group,
};

#define CNT_GMTIMER_CHANNEL(num) {				\
		.type = IIO_COUNT,										\
		.indexed = 1,											\
		.channel = num,											\
		.address = num,											\
		.info_mask_separate =  BIT(IIO_CHAN_INFO_ENABLE)		\
							| BIT(IIO_CHAN_INFO_RAW),			\
		.scan_index = num,										\
		.scan_type = {											\
			.sign = 'u',										\
			.realbits = 32,										\
			.storagebits = 32,									\
			.endianness = IIO_CPU,								\
		},														\
}

static const struct iio_chan_spec cnt_gmtimer_channels[] = {
	CNT_GMTIMER_CHANNEL(0),
	IIO_CHAN_SOFT_TIMESTAMP(6),
};


static const struct cnt_gmtimer_info cnt_gmtimer_info_tbl[] = {
	[0] = {
		.channels = cnt_gmtimer_channels,
		.num_channels = 1,
	},
};

static int cnt_gmtimer_request_gpios(struct cnt_gmtimer_pdata *st)
{
	int i;
	char pinpath[256];
	struct device *dev = st->dev;

	for (i = 0; i < 6; i++) {
		sprintf(pinpath, "pru,adc-%d-mux-a", i + 1);
		if (IS_ERR(st->gpio_mux_a[i] = devm_gpiod_get(dev, pinpath, GPIOD_OUT_LOW)))
			return PTR_ERR(st->gpio_mux_a[i]);
	}

	return 0;
}

static int cnt_gmtimer_buffer_preenable(struct iio_dev *indio_dev)
{
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);

	return 0;
}

static int cnt_gmtimer_buffer_postenable(struct iio_dev *indio_dev)
{
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
	iio_triggered_buffer_postenable(indio_dev);

	return 0;
}

static int cnt_gmtimer_buffer_predisable(struct iio_dev *indio_dev)
{
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
//	st->bufferd = 0;

	return iio_triggered_buffer_predisable(indio_dev);
}

static int cnt_gmtimer_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);

	return 0;
}

static int cnt_gmtimer_trigger_set_state(struct iio_trigger *trig, bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s(%d)\n", __func__, enable);
	st->bufferd = enable;
//	if (st->bufferd)
//		pru_read_samples(indio_dev, st->samplecnt);

	return 0;
}

static const struct iio_buffer_setup_ops cnt_gmtimer_buffer_ops = {
	.preenable = &cnt_gmtimer_buffer_preenable,
	.postenable = &cnt_gmtimer_buffer_postenable,
	.predisable = &cnt_gmtimer_buffer_predisable,
	.postdisable = &cnt_gmtimer_buffer_postdisable,
};

static const struct iio_trigger_ops cnt_gmtimer_trigger_ops = {
    .set_trigger_state = cnt_gmtimer_trigger_set_state,
	.validate_device = iio_trigger_validate_own_device,
};

static const struct of_device_id of_cnt_gmtimer_match[] = {
	{ .compatible = "cnt-gmtimer", },
	{},
};

MODULE_DEVICE_TABLE(of, of_cnt_gmtimer_match);

static int cnt_gmtimer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cnt_gmtimer_pdata *st;
	struct iio_dev *indio_dev;
	int ret, i;

	dev_info(dev, "%s() %s\n", __func__, CNT_GMTIMER_VERSION);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	st->dev = dev;
	mutex_init(&st->lock);
	ret = cnt_gmtimer_request_gpios(st);
	if (ret)
		return ret;

	st->chip_info = &cnt_gmtimer_info_tbl[0];

	indio_dev->dev.parent = dev;
	indio_dev->info = &cnt_gmtimer_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = "cnt-gmtimer";
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
					  indio_dev->name, indio_dev->id);
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &cnt_gmtimer_trigger_ops;
	st->trig->dev.parent = dev;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(dev, st->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trig);

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &cnt_gmtimer_trigger_handler,
					      &cnt_gmtimer_buffer_ops);
	if (ret)
		return ret;

	init_completion(&st->completion);

	st->state = 1;
	st->bufferd = 0;

	return devm_iio_device_register(dev, indio_dev);
}

#ifdef CONFIG_PM_SLEEP

static int cnt_gmtimer_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);
	st->state = 0;

	return 0;
}

static int cnt_gmtimer_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);
	st->state = 1;

	return 0;
}

SIMPLE_DEV_PM_OPS(cnt_gmtimer_pm_ops, cnt_gmtimer_suspend, cnt_gmtimer_resume);
EXPORT_SYMBOL_GPL(cnt_gmtimer_pm_ops);

#endif

static int cnt_gmtimer_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
	st->state = 0;

	return 0;
}

static struct platform_driver cnt_gmtimer = {
	.probe		= cnt_gmtimer_probe,
	.remove		= cnt_gmtimer_remove,
	.driver		= {
		.name	= "cnt-gmtimer",
		.of_match_table = of_cnt_gmtimer_match,
	},
};

module_platform_driver(cnt_gmtimer);
MODULE_AUTHOR("Robert Wörle <rwoerle@mibtec.de>");
MODULE_DESCRIPTION("Omap Timer counter driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(CNT_GMTIMER_VERSION);
MODULE_ALIAS("platform:cnt-gmtimer");
