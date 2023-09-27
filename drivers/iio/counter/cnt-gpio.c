// SPDX-License-Identifier: GPL-2.0
/*
 * IIO Counter implementation on generic gpio inputs
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

#include <linux/gpio/consumer.h>
#include <linux/workqueue.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define CNT_GPIO_VERSION "0.3"

struct cnt_gpio_info {
	struct iio_chan_spec		channel[5];
	unsigned int				num_channels;
	unsigned int				num_configured;
};

typedef struct {
	u64 cnt;
	u64 cntlast;
	int state;
	int freq;
	int polarity;
	s64 tmlast;
	s64 tmdiff;
} gpio_dat;

struct cnt_gpio_pdata {
	int 					state;
	int						calc_tm;	/// in ms
	int						get_tm[2];	/// in us
	struct device			*dev;
	struct iio_dev			*indio_dev;
	struct gpio_descs		*input_gpios;
	gpio_dat				input_dat[4];
	struct work_struct		get_state_work;
	struct work_struct		calc_freq_work;

	const struct cnt_gpio_info	*chip_info;

	struct mutex			lock;
	struct iio_trigger		*trig;
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

static void cnt_gpio_state_cnt(struct iio_dev *indio_dev)
{
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);
	struct gpio_descs *gpios = st->input_gpios;
	int i, val;
	gpio_dat *dat;

	for (i = 0; i < gpios->ndescs; i++) {
		dat = &st->input_dat[i];

		val = gpiod_get_value(gpios->desc[i]);
		if ((val && dat->polarity && !dat->state)
			|| (!val && !dat->polarity && dat->state)) {
			dat->cnt++;
		}

		dat->state = val;
	}
}

static void cnt_gpio_calc_freq_thrd(struct work_struct *work_arg)
{
	struct cnt_gpio_pdata *st = container_of(work_arg, struct cnt_gpio_pdata, calc_freq_work);
	struct gpio_descs *gpios = st->input_gpios;
	struct iio_dev *indio_dev = dev_get_drvdata(st->dev);
	gpio_dat *dat;
	int *iio_buf[4];
	s64 now;
	int i;

	dev_info(st->dev, "Starting calc_freq thread on PID: [%d]\n",  current->pid);
	set_current_state(TASK_INTERRUPTIBLE);

	while(st->state) {
		now = iio_get_time_ns(indio_dev);

		for (i = 0; i < gpios->ndescs; i++) {
			dat = &st->input_dat[i];

			dat->tmdiff = div_s64((now - dat->tmlast), 1000);
			dat->tmlast = now;

			if (dat->tmdiff > 0)
				dat->freq = div_s64(dat->cnt * 1000000, dat->tmdiff);

			dev_dbg(st->dev, "%d: %lld / %lld -> %d\n", i, dat->cnt * 1000000, dat->tmdiff, dat->freq);
			dat->cntlast = dat->cnt;
			dat->cnt = 0;

			iio_buf[i] = &dat->freq;
		}

		iio_push_to_buffers_with_timestamp(indio_dev, &iio_buf, now);
		msleep(st->calc_tm);
	}

	return;
}

static void cnt_gpio_get_state_thrd(struct work_struct *work_arg)
{
	struct cnt_gpio_pdata *st = container_of(work_arg, struct cnt_gpio_pdata, get_state_work);
	struct device *pdev = st->dev;
	struct iio_dev *indio_dev = dev_get_drvdata(pdev);

	dev_info(st->dev, "Starting get_state_thread on PID: [%d]\n",  current->pid);
	set_current_state(TASK_INTERRUPTIBLE);

	while(st->state) {
		cnt_gpio_state_cnt(indio_dev);
		usleep_range(st->get_tm[0], st->get_tm[1]);
	}

	return;
}

static int cnt_gpio_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	uint64_t ret;
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s(%ld %ld)\n", __func__, chan->address, m);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		*val = st->input_dat[chan->address].cntlast;
		iio_device_release_direct_mode(indio_dev);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_FREQUENCY:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		*val = st->input_dat[chan->address].freq;
		*val2 = 0;

		iio_device_release_direct_mode(indio_dev);
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_PHASE:
		*val = st->input_dat[chan->address].polarity;
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
	case IIO_CHAN_INFO_PHASE:
		if (val < 0 && val > 1)
			return -EINVAL;

		st->input_dat[chan->address].polarity = val;
		return 0;
	}

	return -EINVAL;
}

static ssize_t cnt_gpio_show_calctime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cnt_gpio_pdata *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->calc_tm);
}

static ssize_t cnt_gpio_show_get_time0(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cnt_gpio_pdata *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->get_tm[0]);
}

static ssize_t cnt_gpio_show_get_time1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cnt_gpio_pdata *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->get_tm[1]);
}

static ssize_t cnt_gpio_set_get_time0(struct device *dev,
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

	if (val < 0 || st->get_tm[1] < val)
		return -EINVAL;

	st->get_tm[0] = val;

error_ret:
	return ret ? ret : len;
}

static ssize_t cnt_gpio_set_get_time1(struct device *dev,
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

	if (val < 0 || st->get_tm[0] > val)
		return -EINVAL;

	st->get_tm[1] = val;

error_ret:
	return ret ? ret : len;
}

static ssize_t cnt_gpio_set_calctime(struct device *dev,
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

	st->calc_tm = val;

error_ret:
	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(calc_time, (S_IWUSR | S_IRUGO),
		cnt_gpio_show_calctime, cnt_gpio_set_calctime, 0);
static IIO_DEVICE_ATTR(get_time0, (S_IWUSR | S_IRUGO),
		cnt_gpio_show_get_time0, cnt_gpio_set_get_time0, 0);
static IIO_DEVICE_ATTR(get_time1, (S_IWUSR | S_IRUGO),
		cnt_gpio_show_get_time1, cnt_gpio_set_get_time1, 0);

static struct attribute *cnt_gpio_attributes[] = {
	&iio_dev_attr_calc_time.dev_attr.attr,
	&iio_dev_attr_get_time0.dev_attr.attr,
	&iio_dev_attr_get_time1.dev_attr.attr,
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
		.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY)		\
							| BIT(IIO_CHAN_INFO_PHASE)			\
							| BIT(IIO_CHAN_INFO_RAW),			\
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
		.channel[0] = CNT_GPIO_CHANNEL(0),
		.channel[1] = CNT_GPIO_CHANNEL(1),
		.channel[2] = CNT_GPIO_CHANNEL(2),
		.channel[3] = CNT_GPIO_CHANNEL(3),
		.channel[4] = IIO_CHAN_SOFT_TIMESTAMP(4),
		.num_channels = 5,
	},
	[1] = {
		.channel[0] = CNT_GPIO_CHANNEL(0),
		.channel[1] = IIO_CHAN_SOFT_TIMESTAMP(1),
		.num_channels = 2,
	},
	[2] = {
		.channel[0] = CNT_GPIO_CHANNEL(0),
		.channel[1] = CNT_GPIO_CHANNEL(1),
		.channel[2] = IIO_CHAN_SOFT_TIMESTAMP(2),
		.num_channels = 3,
	},
	[3] = {
		.channel[0] = CNT_GPIO_CHANNEL(0),
		.channel[1] = CNT_GPIO_CHANNEL(1),
		.channel[2] = CNT_GPIO_CHANNEL(2),
		.channel[3] = IIO_CHAN_SOFT_TIMESTAMP(3),
		.num_channels = 4,
	},
	[4] = {
		.channel[0] = CNT_GPIO_CHANNEL(0),
		.channel[1] = CNT_GPIO_CHANNEL(1),
		.channel[2] = CNT_GPIO_CHANNEL(2),
		.channel[3] = CNT_GPIO_CHANNEL(3),
		.channel[4] = IIO_CHAN_SOFT_TIMESTAMP(4),
		.num_channels = 5,
	},
};

static int cnt_gpio_trigger_set_state(struct iio_trigger *trig, bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct cnt_gpio_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s(%d)\n", __func__, enable);

	return 0;
}

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

	st->input_gpios = devm_gpiod_get_array(st->dev, "cnt", GPIOD_IN);
	if (st->input_gpios->ndescs > 4) {
		dev_err(st->dev, "too many gpios found\n");
		return -EINVAL;
	}

	return st->input_gpios->ndescs;
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

	mutex_init(&st->lock);

	st->calc_tm = 1000;
	st->get_tm[0] = 350;
	st->get_tm[1] = 400;

	ret = cnt_gpio_request_of(indio_dev);
	if (ret <= 0)
		return -EINVAL;
	st->chip_info = &cnt_gpio_info_tbl[ret];

	indio_dev->dev.parent = dev;
	indio_dev->info = &cnt_gpio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = "cnt-gpio";
	indio_dev->channels = st->chip_info->channel;
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
					      NULL);
	if (ret)
		return ret;

	INIT_WORK(&st->get_state_work, cnt_gpio_get_state_thrd);
	INIT_WORK(&st->calc_freq_work, cnt_gpio_calc_freq_thrd);

	st->state = 1;
	schedule_work(&st->get_state_work);
	schedule_work(&st->calc_freq_work);

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
	cancel_work_sync(&st->get_state_work);
	cancel_work_sync(&st->calc_freq_work);
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
