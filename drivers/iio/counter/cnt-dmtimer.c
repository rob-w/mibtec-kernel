// SPDX-License-Identifier: GPL-2.0
/*
 * IIO Counter implementation via gpio and/or omap dmtimer block
 *
 * Copyright 2020 MIS (c) Robert Woerle rwoerle@mibtec.de, robert@linuxdevelopment.de
 * based pps-dmtimer.c
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
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/slab.h>
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

//#define CREATE_TRACE_POINTS
//#include <trace/events/gpio.h>

#define CNT_DMTIMER_VERSION "0.9"

static const unsigned int cnt_dmtimer_prescaler_avail[] = {
	1, 2, 4, 8, 16, 32, 64, 128, 256
};

struct cnt_dmtimer_info {
	const struct iio_chan_spec	*channels;
	unsigned int				num_channels;
	const unsigned int			*oversampling_avail;
	unsigned int				oversampling_num;
};

struct cnt_dmtimer_pdata {
	int 					state, bufferd;
	int						gate_time;	// in usec ?
	int						prescaler;
	uint32_t				t_delta;
	int						offset;
	struct device			*dev;
	struct iio_dev			*indio_dev;
	struct platform_device	*pdev;
	const struct cnt_dmtimer_info	*chip_info;
	struct regulator		*reg;
	unsigned int			num_os_ratios;

	struct mutex			lock; /* protect sensor state */
	struct gpio_desc		*gpio_mux_a[6];
	struct iio_trigger		*trig;
	struct completion		completion;

	struct omap_dm_timer *capture_timer;
	const struct omap_dm_timer_ops *timer_ops;
	const char *timer_name;
	uint32_t frequency;
	struct timespec64 delta;
	struct clocksource clksrc;
	int ready;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 *  6 * 16-bit samples + 64-bit timestamp
	 */
//	unsigned short			data[10] ____cacheline_aligned;
	unsigned int			data[452] ____cacheline_aligned;
};


static void cnt_dmtimer_enable_irq(struct cnt_dmtimer_pdata *st)
{
    unsigned int interrupt_mask;

    interrupt_mask = OMAP_TIMER_INT_CAPTURE | OMAP_TIMER_INT_OVERFLOW;
    __omap_dm_timer_int_enable(st->capture_timer, interrupt_mask);
    st->capture_timer->context.tier = interrupt_mask;
    st->capture_timer->context.twer = interrupt_mask;
}

static void cnt_dmtimer_cleanup_timer(struct cnt_dmtimer_pdata *st)
{
	if (st->capture_timer == NULL)
		return;

	st->timer_ops->set_source(st->capture_timer, OMAP_TIMER_SRC_SYS_CLK); // in case TCLKIN is stopped during boot
	st->timer_ops->set_int_disable(st->capture_timer, OMAP_TIMER_INT_CAPTURE | OMAP_TIMER_INT_OVERFLOW);
	free_irq(st->capture_timer->irq, st);
	st->timer_ops->stop(st->capture_timer);
	st->timer_ops->free(st->capture_timer);
	st->capture_timer = NULL;
}

/* clocksource ***************/
static struct cnt_dmtimer_pdata *clocksource_timer = NULL;

static u64 cnt_dmtimer_read_cycles(struct clocksource *cs)
{
	u64 cycles = __omap_dm_timer_read_counter(clocksource_timer->capture_timer,
							clocksource_timer->capture_timer->posted);
	return (cycles);
}

static irqreturn_t cnt_dmtimer_interrupt(int irq, void *data)
{
	struct cnt_dmtimer_pdata *st;
	unsigned int irq_status;

	st = data;

	if (!st->ready)
		return IRQ_HANDLED;

	irq_status = st->timer_ops->read_status(st->capture_timer);

	if (irq_status & OMAP_TIMER_INT_CAPTURE) {
		uint32_t capture[2];
		capture[0] = __omap_dm_timer_read(st->capture_timer,
									OMAP_TIMER_CAPTURE_REG,
									st->capture_timer->posted);
		capture[1] = __omap_dm_timer_read(st->capture_timer,
									OMAP_TIMER_CAPTURE2_REG,
									st->capture_timer->posted);

		st->t_delta = capture[1] - capture[0];
		dev_dbg(st->dev, "%s() %d vs %d -> %d \n", __func__,
				capture[0], capture[1], st->t_delta);

		usleep_range(929, 930);
		__omap_dm_timer_write_status(st->capture_timer, OMAP_TIMER_INT_CAPTURE);
	}

	if (irq_status & OMAP_TIMER_INT_OVERFLOW) {
		dev_info(st->dev, "%s() overflow\n", __func__);
		__omap_dm_timer_write_status(st->capture_timer, OMAP_TIMER_INT_OVERFLOW);
	}

	return IRQ_HANDLED; // TODO: shared interrupts?
}

static void omap_dm_timer_setup_capture(struct cnt_dmtimer_pdata *st)
{
	struct omap_dm_timer *timer = st->capture_timer;
	const struct omap_dm_timer_ops *timer_ops = st->timer_ops;
	u32 ctrl;

	timer_ops->set_source(timer, OMAP_TIMER_SRC_SYS_CLK);
	timer_ops->enable(timer);

	ctrl = __omap_dm_timer_read(timer, OMAP_TIMER_CTRL_REG, timer->posted);

	// reload prescaler
	ctrl &= ~(OMAP_TIMER_CTRL_PRE | (0x07 << 2));
	if (st->prescaler >= 0x01 && st->prescaler <= 0x07) {
		ctrl |= OMAP_TIMER_CTRL_PRE;
		ctrl |= st->prescaler << 2;
	}

	// autoreload
	ctrl |= OMAP_TIMER_CTRL_AR;
	__omap_dm_timer_write(timer, OMAP_TIMER_LOAD_REG, 0, timer->posted);

	// start timer
	ctrl |= OMAP_TIMER_CTRL_ST;

	// set capture
	ctrl |= OMAP_TIMER_CTRL_CAPTMODE | OMAP_TIMER_CTRL_TCM_LOWTOHIGH | OMAP_TIMER_CTRL_GPOCFG;

	__omap_dm_timer_load_start(timer, ctrl, 0, timer->posted);

	/* Save the context */
	timer->context.tclr = ctrl;
	timer->context.tldr = 0;
	timer->context.tcrr = 0;
}

static int cnt_dmtimer_init_timer(struct device_node *t_dn, struct cnt_dmtimer_pdata *st)
{
	struct clk *gt_fclk;

	of_property_read_string_index(t_dn, "ti,hwmods", 0, &st->timer_name);
	if (!st->timer_name) {
		dev_err(st->dev, "ti,hwmods property missing?\n");
		return -ENODEV;
	}

	st->capture_timer = st->timer_ops->request_by_node(t_dn);
	if (!st->capture_timer) {
		dev_err(st->dev, "request_by_node failed\n");
		return -ENODEV;
    }

	// TODO: use devm_request_irq?
	if (request_irq(st->capture_timer->irq, cnt_dmtimer_interrupt, IRQF_TIMER, "cnt-dmtimer-irq", st)){
		dev_err(st->dev, "cannot register IRQ %d\n", st->capture_timer->irq);
		return -EIO;
	}

	omap_dm_timer_setup_capture(st);

	gt_fclk = st->timer_ops->get_fclk(st->capture_timer);
	st->frequency = clk_get_rate(gt_fclk);

	dev_info(st->dev, "timer name=%s rate=%uHz\n", st->timer_name, st->frequency);

	return 0;
}

static void cnt_dmtimer_clocksource_init(struct cnt_dmtimer_pdata *st)
{
    if (clocksource_timer != NULL)
		return;

	st->clksrc.name = st->timer_name;

	st->clksrc.rating = 299;
	st->clksrc.read = cnt_dmtimer_read_cycles;
	st->clksrc.mask = CLOCKSOURCE_MASK(32);
	st->clksrc.flags = CLOCK_SOURCE_IS_CONTINUOUS;

	clocksource_timer = st;
	if (clocksource_register_hz(&st->clksrc, st->frequency)){
		pr_err("Could not register clocksource %s\n", st->clksrc.name);
		clocksource_timer = NULL;
	} else
		pr_info("clocksource: %s at %u Hz\n", st->clksrc.name, st->frequency);
}

static void cnt_dmtimer_clocksource_cleanup(struct cnt_dmtimer_pdata *st)
{
    if (st == clocksource_timer)
    {
        clocksource_unregister(&st->clksrc);
        clocksource_timer = NULL;
    }
}

static irqreturn_t cnt_dmtimer_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct cnt_dmtimer_pdata *st = iio_priv(indio_dev);
	dev_info(st->dev, "%s()\n", __func__);

	mutex_lock(&st->lock);
	st->bufferd = 1;

	iio_trigger_notify_done(indio_dev->trig);
	mutex_unlock(&st->lock);

	return IRQ_HANDLED;
}

static int cnt_dmtimer_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	uint64_t ret;
	struct cnt_dmtimer_pdata *st = iio_priv(indio_dev);

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

		*val = (st->frequency + st->offset) / st->t_delta;
		*val2 = 00000;
		iio_device_release_direct_mode(indio_dev);
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_OFFSET:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		*val = st->offset;
		iio_device_release_direct_mode(indio_dev);
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int cnt_dmtimer_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct cnt_dmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s(%ld)\n", __func__, chan->address);

	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		st->offset = val;
		return 0;
	}

	return -EINVAL;
}

static ssize_t cnt_dmtimer_show_prescaler(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cnt_dmtimer_pdata *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->prescaler);
}

static ssize_t cnt_dmtimer_set_prescaler(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct cnt_dmtimer_pdata *st = iio_priv(dev_to_iio_dev(dev));
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto error_ret;

	if (val < 0 || val > 7)
		return -EINVAL;

	st->prescaler = val;
	omap_dm_timer_setup_capture(st);

error_ret:
	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(prescaler, (S_IWUSR | S_IRUGO),
		cnt_dmtimer_show_prescaler, cnt_dmtimer_set_prescaler, 0);

static ssize_t cnt_dmtimer_show_gatetime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cnt_dmtimer_pdata *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->gate_time);
}

static ssize_t cnt_dmtimer_set_gatetime(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct cnt_dmtimer_pdata *st = iio_priv(dev_to_iio_dev(dev));
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
		cnt_dmtimer_show_gatetime, cnt_dmtimer_set_gatetime, 0);

static struct attribute *cnt_dmtimer_attributes[] = {
	&iio_dev_attr_gatetime.dev_attr.attr,
	&iio_dev_attr_prescaler.dev_attr.attr,
	NULL,
};

static const struct attribute_group cnt_dmtimer_attribute_group = {
	.attrs = cnt_dmtimer_attributes,
};

static const struct iio_info cnt_dmtimer_info = {
	.read_raw = cnt_dmtimer_read_raw,
	.write_raw = cnt_dmtimer_write_raw,
	.attrs = &cnt_dmtimer_attribute_group,
};

#define CNT_DMTIMER_CHANNEL(num) {								\
		.type = IIO_COUNT,										\
		.indexed = 1,											\
		.channel = num,											\
		.address = num,											\
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED)		\
							| BIT(IIO_CHAN_INFO_RAW)			\
							| BIT(IIO_CHAN_INFO_OFFSET),		\
		.scan_index = num,										\
		.scan_type = {											\
			.sign = 'u',										\
			.realbits = 32,										\
			.storagebits = 32,									\
			.endianness = IIO_CPU,								\
		},														\
}

static const struct iio_chan_spec cnt_dmtimer_channels[] = {
	CNT_DMTIMER_CHANNEL(0),
	IIO_CHAN_SOFT_TIMESTAMP(6),
};


static const struct cnt_dmtimer_info cnt_dmtimer_info_tbl[] = {
	[0] = {
		.channels = cnt_dmtimer_channels,
		.num_channels = 1,
	},
};

static int cnt_dmtimer_request_gpios(struct cnt_dmtimer_pdata *st)
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

static int cnt_dmtimer_buffer_preenable(struct iio_dev *indio_dev)
{
	struct cnt_dmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);

	return 0;
}

static int cnt_dmtimer_buffer_postenable(struct iio_dev *indio_dev)
{
	struct cnt_dmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
	iio_triggered_buffer_postenable(indio_dev);

	return 0;
}

static int cnt_dmtimer_buffer_predisable(struct iio_dev *indio_dev)
{
	struct cnt_dmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
//	st->bufferd = 0;

	return iio_triggered_buffer_predisable(indio_dev);
}

static int cnt_dmtimer_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct cnt_dmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);

	return 0;
}

static int cnt_dmtimer_trigger_set_state(struct iio_trigger *trig, bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct cnt_dmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s(%d)\n", __func__, enable);
	st->bufferd = enable;
//	if (st->bufferd)
//		pru_read_samples(indio_dev, st->samplecnt);

	return 0;
}

static const struct iio_buffer_setup_ops cnt_dmtimer_buffer_ops = {
	.preenable = &cnt_dmtimer_buffer_preenable,
	.postenable = &cnt_dmtimer_buffer_postenable,
	.predisable = &cnt_dmtimer_buffer_predisable,
	.postdisable = &cnt_dmtimer_buffer_postdisable,
};

static const struct iio_trigger_ops cnt_dmtimer_trigger_ops = {
    .set_trigger_state = cnt_dmtimer_trigger_set_state,
	.validate_device = iio_trigger_validate_own_device,
};

static const struct of_device_id of_cnt_dmtimer_match[] = {
	{ .compatible = "cnt-dmtimer", },
	{},
};

MODULE_DEVICE_TABLE(of, of_cnt_dmtimer_match);

static int cnt_dmtimer_request_of(struct iio_dev *indio_dev)
{
	struct cnt_dmtimer_pdata *st = iio_priv(indio_dev);
	struct device *dev = st->dev;
	struct device_node *t_dn;
	phandle t_ph;

	struct platform_device *timer_pdev;
	struct dmtimer_platform_data *timer_pdata;

	st->ready = 0;

	if (of_property_read_u32(dev->of_node, "timer", &t_ph)) {
		dev_err(st->dev, "could not get of property\n");
		return -ENXIO;
	}

	t_dn = of_find_node_by_phandle(t_ph);
	if (!t_dn) {
		dev_err(st->dev, "Unable to find Timer device node\n");
		goto fail2;
	}

	timer_pdev = of_find_device_by_node(t_dn);
	if (!timer_pdev) {
		dev_err(st->dev, "Unable to find Timer pdev\n");
		goto fail2;
	}

	timer_pdata = dev_get_platdata(&timer_pdev->dev);
    if (!timer_pdata) {
		dev_err(st->dev, "dmtimer pdata structure NULL\n");
		goto fail2;
	}

    st->timer_ops = timer_pdata->timer_ops;
	if (!st->timer_ops
		|| !st->timer_ops->request_by_node
		|| !st->timer_ops->free
		|| !st->timer_ops->enable
		|| !st->timer_ops->disable
		|| !st->timer_ops->get_fclk
		|| !st->timer_ops->start
		|| !st->timer_ops->stop
		|| !st->timer_ops->set_load
		|| !st->timer_ops->set_match
		|| !st->timer_ops->set_pwm
		|| !st->timer_ops->set_prescaler
		|| !st->timer_ops->write_counter) {
		dev_err(st->dev, "Incomplete dmtimer pdata structure\n");
		goto fail2;
	}

	if (cnt_dmtimer_init_timer(t_dn, st) < 0)
		goto fail2;

//	of_node_put(t_dn);
	return 0;

fail2:
	return -1;
}

static int cnt_dmtimer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cnt_dmtimer_pdata *st;
	struct iio_dev *indio_dev;
	int ret;

	dev_info(dev, "%s\n", CNT_DMTIMER_VERSION);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	st->dev = dev;
	st->pdev = pdev;

	ret = cnt_dmtimer_request_of(indio_dev);
	if (ret)
		return -ENOMEM;

	mutex_init(&st->lock);

	st->chip_info = &cnt_dmtimer_info_tbl[0];

	indio_dev->dev.parent = dev;
	indio_dev->info = &cnt_dmtimer_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = "cnt-dmtimer";
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
					  indio_dev->name, indio_dev->id);
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &cnt_dmtimer_trigger_ops;
	st->trig->dev.parent = dev;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(dev, st->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trig);

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &cnt_dmtimer_trigger_handler,
					      &cnt_dmtimer_buffer_ops);
	if (ret)
		return ret;

	st->ready = 1;
	cnt_dmtimer_clocksource_init(st);
	cnt_dmtimer_enable_irq(st);

//	init_completion(&st->completion);

	st->state = 1;
	st->bufferd = 0;

	return devm_iio_device_register(dev, indio_dev);
}

#ifdef CONFIG_PM_SLEEP

static int cnt_dmtimer_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cnt_dmtimer_pdata *st = iio_priv(indio_dev);
	st->state = 0;

	return 0;
}

static int cnt_dmtimer_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cnt_dmtimer_pdata *st = iio_priv(indio_dev);
	st->state = 1;

	return 0;
}

SIMPLE_DEV_PM_OPS(cnt_dmtimer_pm_ops, cnt_dmtimer_suspend, cnt_dmtimer_resume);
EXPORT_SYMBOL_GPL(cnt_dmtimer_pm_ops);

#endif

static int cnt_dmtimer_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cnt_dmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
	st->state = 0;

	cnt_dmtimer_clocksource_cleanup(st);
	cnt_dmtimer_cleanup_timer(st);

	return 0;
}

static struct platform_driver cnt_dmtimer = {
	.probe		= cnt_dmtimer_probe,
	.remove		= cnt_dmtimer_remove,
	.driver		= {
		.name	= "cnt-dmtimer",
		.of_match_table = of_cnt_dmtimer_match,
	},
};

module_platform_driver(cnt_dmtimer);
MODULE_AUTHOR("Robert Wörle <rwoerle@mibtec.de>");
MODULE_DESCRIPTION("Omap Timer counter driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(CNT_DMTIMER_VERSION);
MODULE_ALIAS("platform:cnt-dmtimer");
