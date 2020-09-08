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
//	int						looped;
	int						id;
	struct device			*dev;
	struct iio_dev			*indio_dev;
	const struct cnt_gmtimer_info	*chip_info;
	struct regulator		*reg;
//	unsigned int			range;
//	unsigned int			oversampling;
//	void __iomem			*base_address;
//	const unsigned int		*oversampling_avail;
	unsigned int			num_os_ratios;

	struct mutex			lock; /* protect sensor state */
	struct gpio_desc		*gpio_mux_a[6];
	struct iio_trigger		*trig;
	struct completion		completion;

	struct omap_dm_timer *capture_timer;
	const struct omap_dm_timer_ops *timer_ops;
	const char *timer_name;
	uint32_t frequency;
	unsigned int capture;
	unsigned int overflow;
	unsigned int count_at_interrupt;
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

static int cnt_gmtimer_read_samples(struct iio_dev *indio_dev, int cnt)
{
	struct cnt_gmtimer_pdata *st = iio_priv(indio_dev);
	int ret = 0;

	st->cnted = 0;

	return ret;
}

static void cnt_gmtimer_enable_irq(struct cnt_gmtimer_pdata *st)
{
    unsigned int interrupt_mask;

    interrupt_mask = OMAP_TIMER_INT_CAPTURE | OMAP_TIMER_INT_OVERFLOW;
    __omap_dm_timer_int_enable(st->capture_timer, interrupt_mask);
    st->capture_timer->context.tier = interrupt_mask;
    st->capture_timer->context.twer = interrupt_mask;
}

static void cnt_gmtimer_cleanup_timer(struct cnt_gmtimer_pdata *st)
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
static struct cnt_gmtimer_pdata *clocksource_timer = NULL;

static u64 cnt_gmtimer_read_cycles(struct clocksource *cs)
{
	u64 cycles = __omap_dm_timer_read_counter(clocksource_timer->capture_timer,
							clocksource_timer->capture_timer->posted);
	return (cycles);
}

static irqreturn_t cnt_gmtimer_interrupt(int irq, void *data)
{
	struct cnt_gmtimer_pdata *st;
	unsigned int irq_status;

	st = data;

	if (!st->ready)
		return IRQ_HANDLED;

	irq_status = st->timer_ops->read_status(st->capture_timer);

	if (irq_status & OMAP_TIMER_INT_CAPTURE) {
		uint32_t ps_per_hz;
		unsigned int count_at_capture;

//		pps_get_ts(&pdata->ts);

		st->count_at_interrupt = st->timer_ops->read_counter(st->capture_timer);
		count_at_capture = __omap_dm_timer_read(st->capture_timer,
									OMAP_TIMER_CAPTURE_REG,
									st->capture_timer->posted);

		st->delta.tv_sec = 0;

		// use picoseconds per hz to avoid floating point and limit the rounding error
		ps_per_hz = 1000000000 / (st->frequency / 1000);
		st->delta.tv_nsec = ((st->count_at_interrupt - count_at_capture) * ps_per_hz) / 1000;

		dev_dbg(st->dev, "%s() %d vs %d -> %ld \n", __func__,
				st->count_at_interrupt, count_at_capture, st->delta.tv_nsec);

//		pps_sub_ts(&st->ts, st->delta);
//		pps_event(pdata->pps, &pdata->ts, PPS_CAPTUREASSERT, NULL);

		st->capture++;
		__omap_dm_timer_write_status(st->capture_timer, OMAP_TIMER_INT_CAPTURE);
	}

	if (irq_status & OMAP_TIMER_INT_OVERFLOW) {
		st->overflow++;
		__omap_dm_timer_write_status(st->capture_timer, OMAP_TIMER_INT_OVERFLOW);
	}

	return IRQ_HANDLED; // TODO: shared interrupts?
}

static void omap_dm_timer_setup_capture(struct omap_dm_timer *timer, const struct omap_dm_timer_ops *timer_ops)
{
	u32 ctrl;

	timer_ops->set_source(timer, OMAP_TIMER_SRC_SYS_CLK);
	timer_ops->enable(timer);

	ctrl = __omap_dm_timer_read(timer, OMAP_TIMER_CTRL_REG, timer->posted);

	// disable prescaler
	ctrl &= ~(OMAP_TIMER_CTRL_PRE | (0x07 << 2));

	// autoreload
	ctrl |= OMAP_TIMER_CTRL_AR;
	__omap_dm_timer_write(timer, OMAP_TIMER_LOAD_REG, 0, timer->posted);

	// start timer
	ctrl |= OMAP_TIMER_CTRL_ST;

	// set capture
	ctrl |= OMAP_TIMER_CTRL_TCM_LOWTOHIGH | OMAP_TIMER_CTRL_GPOCFG; // TODO: configurable direction

	__omap_dm_timer_load_start(timer, ctrl, 0, timer->posted);

	/* Save the context */
	timer->context.tclr = ctrl;
	timer->context.tldr = 0;
	timer->context.tcrr = 0;
}

static int cnt_gmtimer_init_timer(struct device_node *t_dn, struct cnt_gmtimer_pdata *st)
{
	struct clk *gt_fclk;

	of_property_read_string_index(t_dn, "ti,hwmods", 0, &st->timer_name);
	if (!st->timer_name) {
		pr_err("ti,hwmods property missing?\n");
		return -ENODEV;
	}

	st->capture_timer = st->timer_ops->request_by_node(t_dn);
	if (!st->capture_timer) {
		pr_err("request_by_node failed\n");
		return -ENODEV;
    }

	// TODO: use devm_request_irq?
	if (request_irq(st->capture_timer->irq, cnt_gmtimer_interrupt, IRQF_TIMER, "cnt-gmtimer-irq", st)){
		pr_err("cannot register IRQ %d\n", st->capture_timer->irq);
		return -EIO;
	}

	omap_dm_timer_setup_capture(st->capture_timer, st->timer_ops);

	gt_fclk = st->timer_ops->get_fclk(st->capture_timer);
	st->frequency = clk_get_rate(gt_fclk);

	pr_info("timer name=%s rate=%uHz\n", st->timer_name, st->frequency);

	return 0;
}

static void cnt_gmtimer_clocksource_init(struct cnt_gmtimer_pdata *st)
{
    if (clocksource_timer != NULL)
		return;

	st->clksrc.name = st->timer_name;

	st->clksrc.rating = 299;
	st->clksrc.read = cnt_gmtimer_read_cycles;
	st->clksrc.mask = CLOCKSOURCE_MASK(32);
	st->clksrc.flags = CLOCK_SOURCE_IS_CONTINUOUS;

	clocksource_timer = st;
	if (clocksource_register_hz(&st->clksrc, st->frequency)){
		pr_err("Could not register clocksource %s\n", st->clksrc.name);
		clocksource_timer = NULL;
	} else
		pr_info("clocksource: %s at %u Hz\n", st->clksrc.name, st->frequency);
}

static void cnt_gmtimer_clocksource_cleanup(struct cnt_gmtimer_pdata *st)
{
    if (st == clocksource_timer)
    {
        clocksource_unregister(&st->clksrc);
        clocksource_timer = NULL;
    }
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

static void cnt_gmtimer_request_of(struct cnt_gmtimer_pdata *st)
{
	struct device_node *np = st->dev->of_node, *t_dn;
	struct platform_device *timer_pdev;
	struct dmtimer_platform_data *timer_pdata;

	st->ready = 0;

	t_dn = of_parse_phandle(np, "timer", 0);
    if (t_dn) {
		dev_err(st->dev, "Unable to parse device node\n");
		return;
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

	if (cnt_gmtimer_init_timer(t_dn, st) < 0)
		goto fail2;

	of_node_put(t_dn);

fail2:
	return;
}

static int cnt_gmtimer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cnt_gmtimer_pdata *st;
	struct iio_dev *indio_dev;
	int ret;

	dev_info(dev, "%s() %s\n", __func__, CNT_GMTIMER_VERSION);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	st->dev = dev;

	cnt_gmtimer_request_of(st);


	mutex_init(&st->lock);
//	ret = cnt_gmtimer_request_gpios(st);
//	if (ret)
//		return ret;

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

	cnt_gmtimer_clocksource_init(st);
	cnt_gmtimer_enable_irq(st);

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

	cnt_gmtimer_clocksource_cleanup(st);
	cnt_gmtimer_cleanup_timer(st);

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
