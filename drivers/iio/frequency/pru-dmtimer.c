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
#include <linux/rpmsg.h>
#include <linux/remoteproc.h>
#include <linux/pruss_driver.h>
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
#include <linux/dma-mapping.h>

//#include <linux/rpmsg/virtio_rpmsg.h>

#define PRU_DMTIMER_VERSION "0.3.1"
#define PRU_DMTIMER_MODULE_DESCRIPTION "PRU CNT-DMTIMER DRIVER"
#define SND_RCV_ADDR_BITS	DMA_BIT_MASK(32)

static const unsigned int pru_dmtimer_prescaler_avail[9] = {
	1, 2, 4, 8, 16, 32, 64, 128, 256
};

struct pru_prepare{
	uint32_t size;
	uint32_t freq;
	uint32_t min;
	uint32_t buffer_addr0;
	uint32_t buffer_addr1;
} __attribute__((__packed__));

struct pru_dmtimer_info {
	const struct iio_chan_spec	*channels;
	unsigned int				num_channels;
	const unsigned int			*prescale_avail;
	unsigned int				prescale_num;
};

struct rpmsg_pru_dev {
	struct rpmsg_device *rpdev;
	wait_queue_head_t wait_list;
};

/// 1.561MB pro DMA - pru as	1560000
#define DATA_BUF_SZ				1561000

struct pru_dmtimer_pdata {
	int 					state, bufferd;
	int						min_mhz;
	int						prescaler;
	uint32_t				t_delta;
	int						offset;
	struct device			*dev;
	struct iio_dev			*indio_dev;

	const struct pru_dmtimer_info	*chip_info;
	unsigned int			num_prescaler;
	const unsigned int		*prescale_avail;

	struct mutex			lock;
	struct iio_trigger		*trig;

	struct omap_dm_timer *capture_timer[2];
	const struct omap_dm_timer_ops *timer_ops[2];
	const char *timer_name[2];
	struct clocksource clksrc[2];
	struct completion		completion;

	uint32_t				frequency;
	int						ready;
	bool					looped;
	int						samplecnt;
	int						cnted;
	uint32_t				*cpu_addr_dma[2];
	dma_addr_t				dma_handle[2];
	struct rpmsg_device 	*rpdev;
	struct pruss			*pruss;
	struct rproc			*rproc;
	int data[4] ____cacheline_aligned; /// 1 x 32bit + 64bit timestamps ?
};
struct pru_dmtimer_pdata *p_st;

static int pru_dmtimer_kick(struct iio_dev *indio_dev, int cnt)
{
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);
	int ret = 0;
	struct pru_prepare prepare;
	uint32_t base = (st->frequency + st->offset) / st->prescaler;

	prepare.size = cnt;
	prepare.freq = base;
	prepare.min = st->min_mhz;
	prepare.buffer_addr0 = st->dma_handle[0];
	prepare.buffer_addr1 = st->dma_handle[1];

	st->cnted = 0;
	ret = rpmsg_send(st->rpdev->ept, &prepare, sizeof(prepare));
	if (ret)
		dev_err(st->dev, "rpmsg_send failed: %d\n", ret);

	return ret;
}

static int rpmsg_pru_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	int i, s_cnt, dma_id = 0;
	int32_t dbg[10];
	struct device *pdev = p_st->dev;
	struct iio_dev *indio_dev = dev_get_drvdata(pdev);

	if (p_st->cpu_addr_dma[0][0])
		dma_id = 0;
	else if (p_st->cpu_addr_dma[1][0])
		dma_id = 1;

	s_cnt = p_st->cpu_addr_dma[dma_id][0];
	dbg[0] = p_st->cpu_addr_dma[dma_id][1];
	dbg[1] = p_st->cpu_addr_dma[dma_id][2];
	dbg[2] = p_st->cpu_addr_dma[dma_id][3];
	dbg[3] = p_st->cpu_addr_dma[dma_id][4];
	dbg[4] = p_st->cpu_addr_dma[dma_id][5];
	dbg[5] = p_st->cpu_addr_dma[dma_id][s_cnt];

	dev_dbg(p_st->dev, "cb() chans %d d0:%d d1:%d dma_id %d "
						"scnt %d 1:%d 2:%d 3:%d 4:%d 5:%d %d:%d\n", p_st->chip_info->num_channels,
						p_st->cpu_addr_dma[0][0], p_st->cpu_addr_dma[1][0], dma_id, s_cnt,
						dbg[0], dbg[1], dbg[2], dbg[3], dbg[4], s_cnt+1, dbg[5]);

	if (p_st->bufferd) {
		p_st->cnted += s_cnt;
		if (p_st->cnted >= p_st->samplecnt) {
			if (p_st->looped)
				p_st->cnted = 0;
		}

		for (i = 0; i < s_cnt; i++)
			iio_push_to_buffers_with_timestamp(indio_dev,
				(int32_t *) p_st->cpu_addr_dma[dma_id] + 1 + (i * (p_st->chip_info->num_channels - 1)),
				iio_get_time_ns(indio_dev));

		/// clear this buffer
		p_st->cpu_addr_dma[dma_id][0] = 0;
		return 0;
	}

	p_st->data[0] =  (int32_t) p_st->cpu_addr_dma[dma_id][1];
	p_st->data[1] =  (int32_t) p_st->cpu_addr_dma[dma_id][2];
	p_st->cpu_addr_dma[dma_id][0] = 0;

	complete(&p_st->completion);
	pru_dmtimer_kick(indio_dev, 0);

	return 0;
}

static void pru_dmtimer_enable_irq(int id, struct pru_dmtimer_pdata *st)
{
    unsigned int interrupt_mask;

    interrupt_mask = OMAP_TIMER_INT_CAPTURE;
//	omap_dm_timer_set_int_enable

 //   __omap_dm_timer_int_enable(st->capture_timer[id], interrupt_mask);
 //   st->capture_timer[id]->context.tier = interrupt_mask;
 //   st->capture_timer[id]->context.twer = interrupt_mask;
}

static void pru_dmtimer_cleanup_timer(int id, struct pru_dmtimer_pdata *st)
{
	if (st->capture_timer[id] == NULL)
		return;

	st->timer_ops[id]->set_source(st->capture_timer[id], OMAP_TIMER_SRC_SYS_CLK); // in case TCLKIN is stopped during boot
	st->timer_ops[id]->set_int_disable(st->capture_timer[id], OMAP_TIMER_INT_CAPTURE | OMAP_TIMER_INT_OVERFLOW);
	st->timer_ops[id]->stop(st->capture_timer[id]);
	st->timer_ops[id]->free(st->capture_timer[id]);
	st->capture_timer[id] = NULL;
}

static int find_prescaler_idx(struct pru_dmtimer_pdata *st, int val)
{
	int i = 0;
	if (val == 1)
		i = 0;
	else if (val == 2)
		i = 1;
	else
		i = find_closest(val, st->prescale_avail, st->num_prescaler);
	return i;
}

static void pru_dmtimer_setup_capture(int id, struct pru_dmtimer_pdata *st)
{
	struct omap_dm_timer *timer = st->capture_timer[id];
	const struct omap_dm_timer_ops *timer_ops = st->timer_ops[id];
	u32 ctrl = 0;
	int idx;

	timer_ops->set_source(timer, OMAP_TIMER_SRC_SYS_CLK);
	timer_ops->enable(timer);

//	ctrl = __omap_dm_timer_read(timer, OMAP_TIMER_CTRL_REG, timer->posted);

	// reload prescaler
	ctrl &= ~(OMAP_TIMER_CTRL_PRE | (0x07 << 2));
	idx = find_prescaler_idx(st, st->prescaler);

	if (idx >= 0x01 && idx <= 0x08) {
		ctrl |= OMAP_TIMER_CTRL_PRE;
		ctrl |= (idx - 1) << 2;
	}

	// autoreload
	ctrl |= OMAP_TIMER_CTRL_AR;
//	__omap_dm_timer_write(timer, OMAP_TIMER_LOAD_REG, 0, timer->posted);

	// start timer
	ctrl |= OMAP_TIMER_CTRL_ST;

	// set capture
	ctrl |= OMAP_TIMER_CTRL_CAPTMODE | OMAP_TIMER_CTRL_TCM_LOWTOHIGH | OMAP_TIMER_CTRL_GPOCFG;

//	__omap_dm_timer_load_start(timer, ctrl, 0, timer->posted);

	/* Save the context */
//	timer->context.tclr = ctrl;
//	timer->context.tldr = 0;
//	timer->context.tcrr = 0;
}

static int pru_dmtimer_init_timer(int id, struct device_node *t_dn, struct pru_dmtimer_pdata *st)
{
	struct clk *gt_fclk;

	st->capture_timer[id] = st->timer_ops[id]->request_by_node(t_dn);
	if (!st->capture_timer[id]) {
		dev_err(st->dev, "request_by_node failed\n");
		return -ENODEV;
    }

	pru_dmtimer_setup_capture(id, st);

	gt_fclk = st->timer_ops[id]->get_fclk(st->capture_timer[id]);
	st->frequency = clk_get_rate(gt_fclk);

	return 0;
}

static void pru_dmtimer_clocksource_init(int id, struct pru_dmtimer_pdata *st)
{
	st->clksrc[id].name = "pru-dmtimer";
	st->clksrc[id].rating = 299;
	st->clksrc[id].mask = CLOCKSOURCE_MASK(32);
	st->clksrc[id].flags = CLOCK_SOURCE_IS_CONTINUOUS;

	if (clocksource_register_hz(&st->clksrc[id], st->frequency))
		dev_err(st->dev, "Could not register clocksource %s\n", st->clksrc[id].name);
	else
		dev_info(st->dev, "clocksource: %s at %u Hz\n", st->clksrc[id].name, st->frequency);
}

static irqreturn_t pru_dmtimer_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);
	dev_info(st->dev, "%s()\n", __func__);

	mutex_lock(&st->lock);
	st->bufferd = 1;
	pru_dmtimer_kick(indio_dev, st->samplecnt);
	iio_trigger_notify_done(indio_dev->trig);
	mutex_unlock(&st->lock);

	return IRQ_HANDLED;
}

static int pru_dmtimer_scan_direct(struct iio_dev *indio_dev, unsigned int ch)
{
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);
	int ret;

	pru_dmtimer_kick(indio_dev, 2);
	ret = wait_for_completion_timeout(&st->completion,
					msecs_to_jiffies(3000));
	if (!ret)
		return -ETIMEDOUT;

	return st->data[ch];
}

static int pru_dmtimer_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	uint64_t ret;
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_FREQUENCY:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = pru_dmtimer_scan_direct(indio_dev, chan->address);
		*val = div_u64(ret, 1000);
		*val2 = (ret - (*val * 1000)) * 1000;

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

static int pru_dmtimer_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);
	int idx;

	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		st->offset = val;
		return 0;

	case IIO_CHAN_INFO_SCALE:
		if (val2)
			return -EINVAL;
		if (val <= 0)
			return -EINVAL;

		idx = find_prescaler_idx(st, val);
		st->prescaler = st->prescale_avail[idx];
		pru_dmtimer_setup_capture(0, st);
		return 0;
	}

	return -EINVAL;
}
static ssize_t pru_dmtimer_show_min_mhz(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pru_dmtimer_pdata *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->min_mhz);
}

static ssize_t pru_dmtimer_set_min_mhz(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct pru_dmtimer_pdata *st = iio_priv(dev_to_iio_dev(dev));
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto error_ret;

	/// cap min to ~5rpm -> 0.08 hz
	if (val <= 80)
		val = 80;

	st->min_mhz = val;

error_ret:
	return ret ? ret : len;
}

static ssize_t pru_dmtimer_show_avail(char *buf, struct pru_dmtimer_pdata *st,
				 const unsigned int *vals, unsigned int n, bool micros)
{
	size_t len = 0;
	int i;

	len += scnprintf(buf + len, PAGE_SIZE - len, "%d Hz prescaler: ", st->frequency);

	for (i = 0; i < n; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
			micros ? "0.%06u " : "%u ", vals[i]);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t pru_dmtimer_prescale_ratio_avail(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);

	return pru_dmtimer_show_avail(buf, st, st->prescale_avail,
				 st->num_prescaler, false);
}

static IIO_DEVICE_ATTR(min_mhz, (S_IWUSR | S_IRUGO),
		pru_dmtimer_show_min_mhz, pru_dmtimer_set_min_mhz, 0);
static IIO_DEVICE_ATTR(scale_ratio_available, 0444,
		       pru_dmtimer_prescale_ratio_avail, NULL, 0);

static struct attribute *pru_dmtimer_attributes[] = {
	&iio_dev_attr_scale_ratio_available.dev_attr.attr,
	&iio_dev_attr_min_mhz.dev_attr.attr,
	NULL,
};

static const struct attribute_group pru_dmtimer_attribute_group = {
	.attrs = pru_dmtimer_attributes,
};

static const struct iio_info pru_dmtimer_info = {
	.read_raw = pru_dmtimer_read_raw,
	.write_raw = pru_dmtimer_write_raw,
	.attrs = &pru_dmtimer_attribute_group,
};

#define PRU_DMTIMER_CHANNEL(num) {								\
		.type = IIO_COUNT,										\
		.indexed = 1,											\
		.channel = num,											\
		.address = num,											\
		.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY)		\
							| BIT(IIO_CHAN_INFO_OFFSET)			\
							| BIT(IIO_CHAN_INFO_SCALE),			\
		.scan_index = num,										\
		.scan_type = {											\
			.sign = 's',										\
			.realbits = 32,										\
			.storagebits = 32,									\
			.endianness = IIO_CPU,								\
		},														\
}

static const struct iio_chan_spec pru_dmtimer_channels[] = {
	PRU_DMTIMER_CHANNEL(0),
	PRU_DMTIMER_CHANNEL(1),
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static const struct pru_dmtimer_info pru_dmtimer_info_tbl[] = {
	[0] = {
		.channels = pru_dmtimer_channels,
		.num_channels = 3,
		.prescale_avail = pru_dmtimer_prescaler_avail,
		.prescale_num = ARRAY_SIZE(pru_dmtimer_prescaler_avail),
	},
};

static int pru_dmtimer_buffer_preenable(struct iio_dev *indio_dev)
{
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);

	return 0;
}
/*
static int pru_dmtimer_buffer_postenable(struct iio_dev *indio_dev)
{
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
	iio_triggered_buffer_postenable(indio_dev);

	return 0;
}

static int pru_dmtimer_buffer_predisable(struct iio_dev *indio_dev)
{
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
	st->bufferd = 0;
	return iio_triggered_buffer_predisable(indio_dev);
}
*/
static int pru_dmtimer_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
	return 0;
}

static int pru_dmtimer_trigger_set_state(struct iio_trigger *trig, bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);

	st->bufferd = enable;
	if (st->bufferd)
		pru_dmtimer_kick(indio_dev, st->samplecnt);
	else
		pru_dmtimer_kick(indio_dev, 0);

	dev_info(st->dev, "%s(%d %d)\n", __func__, st->samplecnt, enable);
	return 0;
}

static const struct iio_buffer_setup_ops pru_dmtimer_buffer_ops = {
	.preenable = &pru_dmtimer_buffer_preenable,
//	.postenable = &pru_dmtimer_buffer_postenable,
//	.predisable = &pru_dmtimer_buffer_predisable,
	.postdisable = &pru_dmtimer_buffer_postdisable,
};

static const struct iio_trigger_ops pru_dmtimer_trigger_ops = {
    .set_trigger_state = pru_dmtimer_trigger_set_state,
	.validate_device = iio_trigger_validate_own_device,
};

static const struct of_device_id of_pru_dmtimer_match[] = {
	{ .compatible = "pru-dmtimer", },
	{},
};

MODULE_DEVICE_TABLE(of, of_pru_dmtimer_match);

static int pru_dmtimer_request_of(struct iio_dev *indio_dev)
{
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);
	struct device *dev = st->dev;
	struct device_node *t_dn;
	int id;
	char id_name[32];

	phandle t_ph;

	struct platform_device *timer_pdev;
	struct dmtimer_platform_data *timer_pdata;

	st->ready = 0;

	for (id = 0; id < 2; id++) {
		sprintf(id_name, "timer_%d", id + 1);
		if (of_property_read_u32(dev->of_node, id_name, &t_ph)) {
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

		st->timer_ops[id] = timer_pdata->timer_ops;
		if (!st->timer_ops[id]
			|| !st->timer_ops[id]->request_by_node
			|| !st->timer_ops[id]->free
			|| !st->timer_ops[id]->enable
			|| !st->timer_ops[id]->disable
			|| !st->timer_ops[id]->get_fclk
			|| !st->timer_ops[id]->start
			|| !st->timer_ops[id]->stop
			|| !st->timer_ops[id]->set_load
			|| !st->timer_ops[id]->set_match
			|| !st->timer_ops[id]->set_pwm
			|| !st->timer_ops[id]->set_prescaler
			|| !st->timer_ops[id]->write_counter) {
			dev_err(st->dev, "Incomplete dmtimer pdata structure\n");
			goto fail2;
		}

		if (pru_dmtimer_init_timer(id, t_dn, st) < 0)
			goto fail2;

		of_node_put(t_dn);
	}
	return 0;

fail2:
	return -1;
}

static int rpmsg_pru_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_pru_dev *prudev;

	prudev = devm_kzalloc(&rpdev->dev, sizeof(*prudev), GFP_KERNEL);
	if (!prudev)
		return -ENOMEM;

	dev_set_drvdata(&rpdev->dev, prudev);

	prudev->rpdev = rpdev;
	p_st->rpdev = rpdev;

	return 0;
}

static void rpmsg_pru_remove(struct rpmsg_device *rpdev)
{
	dev_dbg(&rpdev->dev, "%s()\n", __func__);
}

/* .name matches on RPMsg Channels and causes a probe */
static const struct rpmsg_device_id rpmsg_driver_pru_dmtimer_id_table[] = {
	{ .name	= "pru-dmtimer" },
	{ },
};

static struct rpmsg_driver rpmsg_pru_driver = {
	.drv.name	= "pru-dmtimer_rpmsg",
	.id_table	= rpmsg_driver_pru_dmtimer_id_table,
	.probe		= rpmsg_pru_probe,
	.callback	= rpmsg_pru_cb,
	.remove		= rpmsg_pru_remove,
};

static void free_dma(struct pru_dmtimer_pdata *st)
{
	dma_free_coherent(st->dev, DATA_BUF_SZ,
			st->cpu_addr_dma[0], st->dma_handle[0]);
	dma_free_coherent(st->dev, DATA_BUF_SZ,
			st->cpu_addr_dma[1], st->dma_handle[1]);
}

static int pru_dmtimer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pru_dmtimer_pdata *st;
	phandle rproc_phandle;
	struct iio_dev *indio_dev;
	int ret, id;

	dev_info(dev, "%s() %s 2x%d bytes DMA\n",
		__func__, PRU_DMTIMER_VERSION, DATA_BUF_SZ);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	p_st = st;

	if (!dma_set_mask(dev, SND_RCV_ADDR_BITS)) {
		dev_info(dev, "dma mask available\n");
	} else
		return -ENOMEM;

	st->cpu_addr_dma[0] = dma_alloc_coherent(dev, DATA_BUF_SZ, &st->dma_handle[0], GFP_KERNEL);
	st->cpu_addr_dma[1] = dma_alloc_coherent(dev, DATA_BUF_SZ, &st->dma_handle[1], GFP_KERNEL);
	/// clr all buffers
	st->cpu_addr_dma[0][0] = 0;
	st->cpu_addr_dma[1][0] = 0;

	ret = register_rpmsg_driver(&rpmsg_pru_driver);
	if (ret) {
		pr_err("Unable to register rpmsg driver");
		free_dma(st);
		return (-ENXIO);
	}

	if (of_property_read_u32(dev->of_node, "ti,rproc", &rproc_phandle)) {
		dev_err(&pdev->dev, "could not get ti,rproc of property\n");
		free_dma(st);
		unregister_rpmsg_driver(&rpmsg_pru_driver);
		return -ENXIO;
	}

	st->rproc = rproc_get_by_phandle(rproc_phandle);
	if (!st->rproc) {
		dev_err(&pdev->dev, "could not get rproc handle, deferring probe\n");
		free_dma(st);
		unregister_rpmsg_driver(&rpmsg_pru_driver);
		return -EPROBE_DEFER;
	}

	st->dev = dev;
	mutex_init(&st->lock);

	st->chip_info = &pru_dmtimer_info_tbl[0];
	st->num_prescaler = st->chip_info->prescale_num;
	st->prescale_avail = st->chip_info->prescale_avail;
	st->prescaler = 1;
	st->offset = 1352;
	st->samplecnt = 10;
	st->min_mhz = 80;			/// 0.08 Hz

	ret = pru_dmtimer_request_of(indio_dev);
	if (ret)
		return -ENOMEM;

	indio_dev->dev.parent = dev;
	indio_dev->info = &pru_dmtimer_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = "pru-dmtimer";
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
					  indio_dev->name, iio_device_id(indio_dev));
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &pru_dmtimer_trigger_ops;
	st->trig->dev.parent = dev;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(dev, st->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trig);

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &pru_dmtimer_trigger_handler,
					      &pru_dmtimer_buffer_ops);
	if (ret)
		return ret;

	st->pruss = pruss_get(st->rproc);
	if (!st->pruss) {
		dev_err(&pdev->dev, "error did not get pruss\n");
		free_dma(st);
		unregister_rpmsg_driver(&rpmsg_pru_driver);
		return -ENOMEM;
	}
//	st->id = pru_rproc_get_id(st->rproc);

	/// start pru execution
	rproc_boot(st->rproc);
	dev_dbg(st->dev, "%s() rproc_boot()\n", __func__);

	st->ready = 1;
	st->state = 1;
	init_completion(&st->completion);

	for (id = 0; id < 2; id++) {
		pru_dmtimer_clocksource_init(id, st);
		pru_dmtimer_enable_irq(id, st);
	}

	return devm_iio_device_register(dev, indio_dev);
}

#ifdef CONFIG_PM_SLEEP

static int pru_dmtimer_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);
	st->state = 0;

	return 0;
}

static int pru_dmtimer_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);
	st->state = 1;

	return 0;
}

SIMPLE_DEV_PM_OPS(pru_dmtimer_pm_ops, pru_dmtimer_suspend, pru_dmtimer_resume);
EXPORT_SYMBOL_GPL(pru_dmtimer_pm_ops);

#endif

static int pru_dmtimer_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct pru_dmtimer_pdata *st = iio_priv(indio_dev);
	int id;

	for (id = 0; id < 2; id++) {
		st->timer_ops[id]->set_int_disable(st->capture_timer[id], OMAP_TIMER_INT_CAPTURE | OMAP_TIMER_INT_OVERFLOW);
		clocksource_unregister(&st->clksrc[id]);
		pru_dmtimer_cleanup_timer(id, st);
	}

	st->state = 0;

	rproc_shutdown(st->rproc);
	pruss_put(st->pruss);
	unregister_rpmsg_driver(&rpmsg_pru_driver);
	free_dma(st);
	dev_info(dev, "%s() %s remove()\n", __func__, PRU_DMTIMER_VERSION);
	return 0;
}

static struct platform_driver pru_dmtimer = {
	.probe		= pru_dmtimer_probe,
	.remove		= pru_dmtimer_remove,
	.driver		= {
		.name	= "pru-dmtimer",
		.of_match_table = of_pru_dmtimer_match,
	},
};
module_platform_driver(pru_dmtimer);

MODULE_SOFTDEP("pre: pru_rproc");
MODULE_AUTHOR("Robert Wörle <rwoerle@mibtec.de>");
MODULE_DESCRIPTION("Omap Timer counter driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(PRU_DMTIMER_VERSION);
MODULE_ALIAS("platform:pru-dmtimer");
