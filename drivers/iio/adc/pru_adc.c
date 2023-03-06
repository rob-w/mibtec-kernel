// SPDX-License-Identifier: GPL-2.0
/*
 * a PRU ADC driver via remoteproc
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
#include <linux/rpmsg.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/util_macros.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/pruss.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#include <linux/dma-mapping.h>

#include <linux/rpmsg/virtio_rpmsg.h>

//#define CREATE_TRACE_POINTS
//#include <trace/events/gpio.h>

#define PRU_ADC_MODULE_VERSION "1.95"
#define PRU_ADC_MODULE_DESCRIPTION "PRU ADC DRIVER"

#define SND_RCV_ADDR_BITS	DMA_BIT_MASK(32)

#define CHIP_054	54
#define CHIP_060	60
#define CHIP_062	62
#define CHIP_070	70

struct pru_prepare{
	uint32_t size;
	uint32_t cfg;
	uint32_t buffer_addr0;
	uint32_t buffer_addr1;
} __attribute__((__packed__));

struct pru_chip_info {
	int 						id;
	const struct iio_chan_spec	*channels;
	unsigned int				num_channels;
	const unsigned int			*oversampling_avail;
	unsigned int				oversampling_num;
	bool					os_req_reset;
};

struct rpmsg_pru_dev {
	struct rpmsg_device *rpdev;
	wait_queue_head_t wait_list;
};

/// 1.561MB pro DMA - pru as	1560000
#define DATA_BUF_SZ				1561000

struct pru_priv {
	int 					state, bufferd;
	int						samplecnt;
	int						cnted;
	bool					looped;
	int						id;
	struct device			*dev;
	struct iio_dev			*indio_dev;
	const struct pru_chip_info	*chip_info;
	struct pruss			*pruss;
	struct rproc			*rproc;
	struct regulator		*reg;
	unsigned int			range;
	unsigned int			oversampling;
	void __iomem			*base_address;
	const unsigned int		*oversampling_avail;
	unsigned int			num_os_ratios;
	uint32_t				*cpu_addr_dma[2];
	dma_addr_t				dma_handle[2];
	unsigned				filter_mode;
	unsigned				dec_rate;

	struct mutex			lock; /* protect sensor state */
	struct gpio_desc		*gpio_mux_a[6];
	struct gpio_desc		*gpio_mux_b[6];
	struct gpio_desc		*gpio_gain0[6];
	struct gpio_desc		*gpio_gain1[6];
	struct gpio_desc		*gpio_gain2[6];
	struct gpio_desc		*gpio_select[4];

	short					offset[6];
	int						calibscale[6];
	struct iio_trigger		*trig;
	struct completion		completion;
	struct rpmsg_device 	*rpdev;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 *  6 * 16-bit samples + 64-bit timestamp
	 */
//	unsigned short			data[10] ____cacheline_aligned;
	unsigned int			data[32] ____cacheline_aligned;/// 6 x 32bit + 64bit timestamps ?
};

struct pru_priv *p_st;

static int pru_reset(struct pru_priv *st)
{
	ndelay(100); /* t_reset >= 100ns */
	return 0;
}

static int pru_read_samples(struct iio_dev *indio_dev, int cnt)
{
	struct pru_priv *st = iio_priv(indio_dev);
	int ret = 0;
	struct pru_prepare prepare;

	prepare.size = cnt;
	prepare.cfg = 0;
	if (st->looped)
		prepare.cfg |= (1<<0);
	prepare.cfg |= (st->dec_rate << 1);
	prepare.cfg |= (st->filter_mode << 5);

	prepare.buffer_addr0 = st->dma_handle[0];
	prepare.buffer_addr1 = st->dma_handle[1];

	dev_dbg(st->dev, "cnt %d addr0 0x%x addr1 0x%x fmode %d dec %d loop %d cfg %d\n",
		cnt, st->dma_handle[0], st->dma_handle[1], st->filter_mode, st->dec_rate, st->looped, prepare.cfg);

	st->cnted = 0;
	ret = rpmsg_send(st->rpdev->ept, &prepare, sizeof(prepare));
	if (ret)
		dev_err(st->dev, "rpmsg_send failed: %d\n", ret);

	return ret;
}

static irqreturn_t pru_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct pru_priv *st = iio_priv(indio_dev);
	dev_dbg(st->dev, "%s()\n", __func__);

	mutex_lock(&st->lock);
	st->bufferd = 1;
	pru_read_samples(indio_dev, st->samplecnt);
	iio_trigger_notify_done(indio_dev->trig);
	mutex_unlock(&st->lock);

	return IRQ_HANDLED;
}

static int pru_scan_direct(struct iio_dev *indio_dev, unsigned int ch)
{
	struct pru_priv *st = iio_priv(indio_dev);
	int ret;

	dev_dbg(st->dev, "%s(%d)\n", __func__, ch);

	ret = pru_read_samples(indio_dev, 1000);
	if (ret != 0)
		return -EINVAL;

	ret = wait_for_completion_timeout(&st->completion,
					msecs_to_jiffies(1000));
	if (!ret)
		return -ETIMEDOUT;

	return st->data[ch];
}

static int pru_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	uint64_t ret;
	struct pru_priv *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s(%ld %ld)\n", __func__, chan->address, m);

	switch (m) {
	case IIO_CHAN_INFO_PROCESSED:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		ret = pru_scan_direct(indio_dev, chan->address);
		iio_device_release_direct_mode(indio_dev);

		if (ret == -ETIMEDOUT || ret == -EINVAL)
			return ret;

		ret -= st->offset[chan->scan_index];

		/// factor with calibscale
		ret = ret * st->calibscale[chan->scan_index];
		do_div(ret, 100000);

		*val = ret;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = pru_scan_direct(indio_dev, chan->address);
		iio_device_release_direct_mode(indio_dev);
		if (ret == -ETIMEDOUT)
			return ret;

		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		*val = st->calibscale[chan->scan_index];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		*val = st->offset[chan->scan_index];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = st->oversampling;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*val = 0;
		if (st->chip_info->id == CHIP_070)
			return IIO_VAL_INT;

		if (gpiod_get_value_cansleep(st->gpio_gain0[chan->address]))
			*val |= (1<<0);
		if (gpiod_get_value_cansleep(st->gpio_gain1[chan->address]))
			*val |= (1<<1);

		if (st->chip_info->id == CHIP_062)
			return IIO_VAL_INT;

		if (gpiod_get_value_cansleep(st->gpio_gain2[chan->address]))
			*val |= (1<<2);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_ENABLE:
		*val = 0;
		if (st->chip_info->id == CHIP_070)
			return IIO_VAL_INT;

		if (gpiod_get_value_cansleep(st->gpio_mux_a[chan->address]))
			*val |= (1<<0);

		if (st->chip_info->id == CHIP_054)
			return IIO_VAL_INT;

		if (gpiod_get_value_cansleep(st->gpio_mux_b[chan->address]))
			*val |= (1<<1);
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int pru_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct pru_priv *st = iio_priv(indio_dev);
	DECLARE_BITMAP(values, 3);
	int i;

	dev_dbg(st->dev, "%s(%ld)\n", __func__, chan->address);

	switch (mask) {
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
	case IIO_CHAN_INFO_OFFSET:
		st->offset[chan->scan_index] = val;
		return 0;
	case IIO_CHAN_INFO_CALIBSCALE:
		st->calibscale[chan->scan_index] = val;
		return 0;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (st->chip_info->id == CHIP_070)
			return 0;

		if (val & (1<<0))
			gpiod_set_value_cansleep(st->gpio_gain0[chan->address], 1);
		else
			gpiod_set_value_cansleep(st->gpio_gain0[chan->address], 0);
		if (val & (1<<1))
			gpiod_set_value_cansleep(st->gpio_gain1[chan->address], 1);
		else
			gpiod_set_value_cansleep(st->gpio_gain1[chan->address], 0);

		if (st->chip_info->id == CHIP_062)
			return 0;

		if (val & (1<<2))
			gpiod_set_value_cansleep(st->gpio_gain2[chan->address], 1);
		else
			gpiod_set_value_cansleep(st->gpio_gain2[chan->address], 0);

		return 0;
	case IIO_CHAN_INFO_ENABLE:
		if (st->chip_info->id == CHIP_070)
			return 0;

		if (val & (1<<0))
			gpiod_set_value_cansleep(st->gpio_mux_a[chan->address], 1);
		else
			gpiod_set_value_cansleep(st->gpio_mux_a[chan->address], 0);

		if (st->chip_info->id == CHIP_054)
			return 0;

		if (val & (1<<1))
			gpiod_set_value_cansleep(st->gpio_mux_b[chan->address], 1);
		else
			gpiod_set_value_cansleep(st->gpio_mux_b[chan->address], 0);
		return 0;
	default:
		return -EINVAL;
	}
}

static ssize_t pru_show_looped(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pru_priv *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->looped);
}

static ssize_t pru_set_looped(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct pru_priv *st = iio_priv(dev_to_iio_dev(dev));
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		goto error_ret;

	st->looped = val;

error_ret:
	return ret ? ret : len;
}

static ssize_t pru_show_cs_chans(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val = 0;
	int i;
	struct pru_priv *st = iio_priv(dev_to_iio_dev(dev));

	if (st->chip_info->id != CHIP_062)
		return sprintf(buf, "%d\n", val);

	for (i = 0, val = 0; i < 4; i++) {
		if (gpiod_get_value(st->gpio_select[i]))
			val |= (1 << i);
	}

	return sprintf(buf, "%d\n", val);
}

static ssize_t pru_set_cs_chans(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	int i;
	struct pru_priv *st = iio_priv(dev_to_iio_dev(dev));
	unsigned int val;
	int ret;

	if (st->chip_info->id != CHIP_062)
		return 0;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto error_ret;

	if (val < 0)
		return -EINVAL;

	for (i = 0; i < 4; i++) {
		if (val & (1<<i))
			gpiod_set_value(st->gpio_select[i], 1);
		else
			gpiod_set_value(st->gpio_select[i], 0);
	}

error_ret:
	return ret ? ret : len;
}

static ssize_t pru_show_samplecnt(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pru_priv *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->samplecnt);
}

static ssize_t pru_set_samplecnt(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct pru_priv *st = iio_priv(dev_to_iio_dev(dev));
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto error_ret;

	if (val < 0)
		return -EINVAL;

	st->samplecnt = val;

error_ret:
	return ret ? ret : len;
}

static const char * const pru_filter_modes[] = {
	"sinc5_dec32_1024",
	"sinc5_dec8",
	"sinc5_dec16",
	"sinc3_dec32",
	"fir",
};

static int pru_get_filter_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct pru_priv *st = iio_priv(indio_dev);

	return st->filter_mode;
}

static int pru_set_filter_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned mode)
{
	struct pru_priv *st = iio_priv(indio_dev);

	st->filter_mode = mode;

	return 0;
}

static const struct iio_enum pru_filter_mode_enum = {
	.items = pru_filter_modes,
	.num_items = ARRAY_SIZE(pru_filter_modes),
	.get = pru_get_filter_mode,
	.set = pru_set_filter_mode,
};

static const char * const pru_dec_rates[] = {
	"32",
	"64",
	"128",
	"256",
	"512",
	"1024",
};

static int pru_get_dec_rate(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct pru_priv *st = iio_priv(indio_dev);

	return st->dec_rate;
}

static int pru_set_dec_rate(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned rate)
{
	struct pru_priv *st = iio_priv(indio_dev);

	st->dec_rate = rate;

	return 0;
}

static const struct iio_enum pru_dec_rate_enum = {
	.items = pru_dec_rates,
	.num_items = ARRAY_SIZE(pru_dec_rates),
	.get = pru_get_dec_rate,
	.set = pru_set_dec_rate,
};

static const struct iio_chan_spec_ext_info pru_ext_info[] = {
	IIO_ENUM("filter_mode", IIO_SHARED_BY_TYPE, &pru_filter_mode_enum),
	IIO_ENUM_AVAILABLE("filter_mode", &pru_filter_mode_enum),
	IIO_ENUM("decimation_rate", IIO_SHARED_BY_TYPE, &pru_dec_rate_enum),
	IIO_ENUM_AVAILABLE("decimation_rate", &pru_dec_rate_enum),
	{ },
};

static IIO_DEVICE_ATTR(looped, (S_IWUSR | S_IRUGO),
		pru_show_looped, pru_set_looped, 0);

static IIO_DEVICE_ATTR(samplecnt, (S_IWUSR | S_IRUGO),
		pru_show_samplecnt, pru_set_samplecnt, 0);

static IIO_DEVICE_ATTR(cs_chans, (S_IWUSR | S_IRUGO),
		pru_show_cs_chans, pru_set_cs_chans, 0);

static struct attribute *pru_attributes[] = {
	&iio_dev_attr_samplecnt.dev_attr.attr,
	&iio_dev_attr_looped.dev_attr.attr,
	&iio_dev_attr_cs_chans.dev_attr.attr,
	NULL,
};

static const struct attribute_group pru_attribute_group = {
	.attrs = pru_attributes,
};

static const struct iio_info pru_info = {
	.read_raw = pru_read_raw,
	.write_raw = pru_write_raw,
	.attrs = &pru_attribute_group,
};

#define PRUX_CHANNEL(num, bits, typ,  mask) {					\
		.type = typ,											\
		.indexed = 1,											\
		.channel = num,											\
		.address = num,											\
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED)		\
							| BIT(IIO_CHAN_INFO_CALIBSCALE)		\
							| BIT(IIO_CHAN_INFO_HARDWAREGAIN)	\
							| BIT(IIO_CHAN_INFO_ENABLE)			\
							| BIT(IIO_CHAN_INFO_OFFSET)			\
							| BIT(IIO_CHAN_INFO_RAW),			\
		.info_mask_shared_by_type = mask,						\
		.info_mask_shared_by_all = mask,						\
		.scan_index = num,										\
		.scan_type = {											\
			.sign = 's',										\
			.realbits = bits,									\
			.storagebits = 32,									\
			.endianness = IIO_CPU,								\
		},														\
		.ext_info = pru_ext_info,							\
}

#define PRU_CHANNEL(num, bits, typ)	\
	PRUX_CHANNEL(num, bits, typ, BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO))

static const struct iio_chan_spec pru_6_24_channels[] = {
	PRU_CHANNEL(0, 24, IIO_VOLTAGE),
	PRU_CHANNEL(1, 24, IIO_VOLTAGE),
	PRU_CHANNEL(2, 24, IIO_VOLTAGE),
	PRU_CHANNEL(3, 24, IIO_VOLTAGE),
	PRU_CHANNEL(4, 24, IIO_VOLTAGE),
	PRU_CHANNEL(5, 24, IIO_VOLTAGE),
	IIO_CHAN_SOFT_TIMESTAMP(6),
};

static const struct iio_chan_spec pru_6_channels[] = {
	PRU_CHANNEL(0, 16, IIO_VOLTAGE),
	PRU_CHANNEL(1, 16, IIO_VOLTAGE),
	PRU_CHANNEL(2, 16, IIO_VOLTAGE),
	PRU_CHANNEL(3, 16, IIO_VOLTAGE),
	PRU_CHANNEL(4, 16, IIO_VOLTAGE),
	PRU_CHANNEL(5, 16, IIO_VOLTAGE),
	IIO_CHAN_SOFT_TIMESTAMP(6),
};

static const struct iio_chan_spec pru_4_channels[] = {
	PRU_CHANNEL(0, 16, IIO_VOLTAGE),
	PRU_CHANNEL(1, 16, IIO_VOLTAGE),
	PRU_CHANNEL(2, 16, IIO_VOLTAGE),
	PRU_CHANNEL(3, 16, IIO_VOLTAGE),
	IIO_CHAN_SOFT_TIMESTAMP(6),
};

static const struct pru_chip_info pru_chip_info_tbl[] = {
	[0] = {
		.channels = pru_6_channels,
		.num_channels = 7,
		.id = CHIP_060,
	},
	[1] = {
		.channels = pru_6_24_channels,
		.num_channels = 7,
		.id = CHIP_062,
	},
	[2] = {
		.channels = pru_4_channels,
		.num_channels = 5,
		.id = CHIP_054,
	},
	[3] = {
		.channels = pru_6_channels,
		.num_channels = 7,
		.id = CHIP_070,
	},
};

static int pru_request_gpios(struct pru_priv *st)
{
	int i;
	char pinpath[256];
	struct device *dev = st->dev;

	if (st->chip_info->id == CHIP_070)
		return 0;

	for (i = 0; i < st->chip_info->num_channels -1; i++) {
		sprintf(pinpath, "pru,adc-%d-mux-a", i + 1);
		if (IS_ERR(st->gpio_mux_a[i] = devm_gpiod_get(dev, pinpath, GPIOD_OUT_LOW)))
			return PTR_ERR(st->gpio_mux_a[i]);

		if (st->chip_info->id == CHIP_060 || st->chip_info->id == CHIP_062) {
			sprintf(pinpath, "pru,adc-%d-mux-b", i + 1);
			if (IS_ERR(st->gpio_mux_b[i] = devm_gpiod_get(dev, pinpath, GPIOD_OUT_LOW)))
				return PTR_ERR(st->gpio_mux_b[i]);
		}

		sprintf(pinpath, "pru,adc-%d-gain0", i + 1);
		if (IS_ERR(st->gpio_gain0[i] = devm_gpiod_get(dev, pinpath, GPIOD_OUT_LOW)))
			return PTR_ERR(st->gpio_gain0[i]);

		sprintf(pinpath, "pru,adc-%d-gain1", i + 1);
		if (IS_ERR(st->gpio_gain1[i] = devm_gpiod_get(dev, pinpath, GPIOD_OUT_LOW)))
			return PTR_ERR(st->gpio_gain1[i]);

		if (st->chip_info->id == CHIP_060 || st->chip_info->id == CHIP_054) {
			sprintf(pinpath, "pru,adc-%d-gain2", i + 1);
			if (IS_ERR(st->gpio_gain2[i] = devm_gpiod_get(dev, pinpath, GPIOD_OUT_LOW)))
				return PTR_ERR(st->gpio_gain2[i]);
		}
	}

	if (st->chip_info->id == CHIP_062) {
		if (IS_ERR(st->gpio_select[0] = devm_gpiod_get(dev, "pru,adc-select-a0", GPIOD_OUT_LOW)))
			return PTR_ERR(st->gpio_select[0]);
		if (IS_ERR(st->gpio_select[1] = devm_gpiod_get(dev, "pru,adc-select-a1", GPIOD_OUT_LOW)))
			return PTR_ERR(st->gpio_select[1]);
		if (IS_ERR(st->gpio_select[2] = devm_gpiod_get(dev, "pru,adc-select-a2", GPIOD_OUT_LOW)))
			return PTR_ERR(st->gpio_select[2]);
		if (IS_ERR(st->gpio_select[3] = devm_gpiod_get(dev, "pru,adc-select-e3", GPIOD_OUT_LOW)))
			return PTR_ERR(st->gpio_select[3]);
	}

	return 0;
}

static int pru_buffer_preenable(struct iio_dev *indio_dev)
{
	struct pru_priv *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);

	return 0;
}

static int pru_buffer_postenable(struct iio_dev *indio_dev)
{
	struct pru_priv *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
	iio_triggered_buffer_postenable(indio_dev);

	return 0;
}

static int pru_buffer_predisable(struct iio_dev *indio_dev)
{
	struct pru_priv *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s() rebooting? rproc\n", __func__);
	st->bufferd = 0;
	return iio_triggered_buffer_predisable(indio_dev);
}

static int pru_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct pru_priv *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s()\n", __func__);
	return 0;
}

static int pru_trigger_set_state(struct iio_trigger *trig, bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct pru_priv *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s(%d)\n", __func__, enable);
	st->bufferd = enable;
	if (st->bufferd)
		pru_read_samples(indio_dev, st->samplecnt);

	return 0;
}

static const struct iio_buffer_setup_ops pru_buffer_ops = {
	.preenable = &pru_buffer_preenable,
	.postenable = &pru_buffer_postenable,
	.predisable = &pru_buffer_predisable,
	.postdisable = &pru_buffer_postdisable,
};

static const struct iio_trigger_ops pru_trigger_ops = {
    .set_trigger_state = pru_trigger_set_state,
	.validate_device = iio_trigger_validate_own_device,
};

static const struct of_device_id of_pru_adc_match[] = {
	{ .compatible = "pru-adc-070", },
	{ .compatible = "pru-adc-060", },
	{ .compatible = "pru-adc-062", },
	{ .compatible = "pru-adc-054", },
	{},
};

static int rpmsg_pru_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	int i, s_cnt, dma_id;
	int32_t dbg1, dbg2, dbg3;
	struct device *pdev = p_st->dev;
	struct iio_dev *indio_dev = dev_get_drvdata(pdev);

	if (p_st->cpu_addr_dma[0][0])
		dma_id = 0;
	else if (p_st->cpu_addr_dma[1][0])
		dma_id = 1;
	else {
		pru_read_samples(indio_dev, 0);
		return 0;
	}

	s_cnt = p_st->cpu_addr_dma[dma_id][0];

	if (p_st->bufferd) {

		dbg1 = p_st->cpu_addr_dma[dma_id][1];
		dbg2 = p_st->cpu_addr_dma[dma_id][2];
		dbg3 = p_st->cpu_addr_dma[dma_id][3];

		p_st->cnted += s_cnt;

		dev_dbg(p_st->dev, "cb() dma_id %d scnt %d dbg %i %i %i\n", dma_id, s_cnt, dbg1, dbg2, dbg3);

		if (p_st->cnted >= p_st->samplecnt) {
			if (p_st->looped)
				p_st->cnted = 0;
			else
				pru_read_samples(indio_dev, 0);
		}

		for (i = 0; i < s_cnt; i++)
			iio_push_to_buffers_with_timestamp(indio_dev,
				(int32_t *) p_st->cpu_addr_dma[dma_id] + 1 + (i * (p_st->chip_info->num_channels - 1)),
				iio_get_time_ns(indio_dev));

		/// clear this buffer
		p_st->cpu_addr_dma[dma_id][0] = 0;

//		trace_pru_call(dma_id, "end");
//		trace_gpio_value(1, 1, 1);

		return 0;
	}

	p_st->data[0] = p_st->cpu_addr_dma[dma_id][1] & 0xFFFF;
	p_st->data[1] = p_st->cpu_addr_dma[dma_id][2] & 0xFFFF;
	p_st->data[2] = p_st->cpu_addr_dma[dma_id][3] & 0xFFFF;
	p_st->data[3] = p_st->cpu_addr_dma[dma_id][4] & 0xFFFF;
	if (p_st->chip_info->id == CHIP_060 || p_st->chip_info->id == CHIP_062 || p_st->chip_info->id == CHIP_070) {
		p_st->data[4] = p_st->cpu_addr_dma[dma_id][5] & 0xFFFF;
		p_st->data[5] = p_st->cpu_addr_dma[dma_id][6] & 0xFFFF;
		dev_dbg(p_st->dev, "IDD_%d 1:%d 2:%d 3:%d 4:%d 5:%d 6:%d\n", dma_id,
			p_st->data[0], p_st->data[1], p_st->data[2], p_st->data[3], p_st->data[4], p_st->data[5]);
	} else
		dev_dbg(p_st->dev, "IDD_%d 1:%d 2:%d 3:%d 4:%d\n", dma_id,
			p_st->data[0], p_st->data[1], p_st->data[2], p_st->data[3]);

	/// clear this buffer
	p_st->cpu_addr_dma[dma_id][0] = 0;
	complete(&p_st->completion);
	pru_read_samples(indio_dev, 0);

	return 0;
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

MODULE_DEVICE_TABLE(of, of_pru_adc_match);

/* .name matches on RPMsg Channels and causes a probe */
static const struct rpmsg_device_id rpmsg_driver_pru_id_table[] = {
	{ .name	= "pru-adc" },
	{ },
};

MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_pru_id_table);

static struct rpmsg_driver rpmsg_pru_driver = {
	.drv.name	= "pru-adc_rpmsg",
	.id_table	= rpmsg_driver_pru_id_table,
	.probe		= rpmsg_pru_probe,
	.callback	= rpmsg_pru_cb,
	.remove		= rpmsg_pru_remove,
};

static void free_dma(struct pru_priv *st)
{
	dma_free_coherent(st->dev, DATA_BUF_SZ,
			st->cpu_addr_dma[0], st->dma_handle[0]);
	dma_free_coherent(st->dev, DATA_BUF_SZ,
			st->cpu_addr_dma[1], st->dma_handle[1]);
}

static int pru_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pru_priv *st;
	phandle rproc_phandle;
	int ret, i;
	struct iio_dev *indio_dev;

	dev_info(dev, "%s() %s 2x%d bytes DMA\n", __func__, PRU_ADC_MODULE_VERSION, DATA_BUF_SZ);

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

	if (of_property_match_string(dev->of_node, "compatible", "pru-adc-060") == 0) {
		st->chip_info = &pru_chip_info_tbl[0];
		dev_info(dev, "loading %03d 6x16bit\n", st->chip_info->id);
	} else if (of_property_match_string(dev->of_node, "compatible", "pru-adc-062") == 0) {
		st->chip_info = &pru_chip_info_tbl[1];
		dev_info(dev, "loading %03d 6x24bit\n", st->chip_info->id);
	} else if (of_property_match_string(dev->of_node, "compatible", "pru-adc-054") == 0) {
		st->chip_info = &pru_chip_info_tbl[2];
		dev_info(dev, "loading %03d 4x16bit\n", st->chip_info->id);
	} else if (of_property_match_string(dev->of_node, "compatible", "pru-adc-070") == 0) {
		st->chip_info = &pru_chip_info_tbl[3];
		dev_info(dev, "loading %03d MISDIMM 6x16bit\n", st->chip_info->id);
	} else {
		pr_err("no compatible of_device");
		free_dma(st);
		return -ENODEV;
	}

	ret = register_rpmsg_driver(&rpmsg_pru_driver);
	if (ret) {
		pr_err("Unable to register rpmsg driver");
		free_dma(st);
		return (-ENXIO);
	}

	if (of_property_read_u32(dev->of_node, "ti,rproc", &rproc_phandle)) {
		dev_err(&pdev->dev, "could not get of property\n");
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
	ret = pru_request_gpios(st);
	if (ret) {
		dev_err(&pdev->dev, "error requesting gpios\n");
		free_dma(st);
		unregister_rpmsg_driver(&rpmsg_pru_driver);
		return ret;
	}

	indio_dev->dev.parent = dev;
	indio_dev->info = &pru_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = "pru-adc";
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	init_completion(&st->completion);

	ret = pru_reset(st);
	if (ret)
		dev_warn(st->dev, "failed to RESET: no RESET GPIO specified\n");

	st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
					  indio_dev->name, indio_dev->id);
	if (!st->trig) {
		dev_err(&pdev->dev, "error alloc iio_trigger\n");
		free_dma(st);
		unregister_rpmsg_driver(&rpmsg_pru_driver);
		return -ENOMEM;
	}

	st->trig->ops = &pru_trigger_ops;
	st->trig->dev.parent = dev;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(dev, st->trig);
	if (ret) {
		free_dma(st);
		unregister_rpmsg_driver(&rpmsg_pru_driver);
		return ret;
	}

	indio_dev->trig = iio_trigger_get(st->trig);

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &pru_trigger_handler,
					      &pru_buffer_ops);
	if (ret){
		free_dma(st);
		return ret;
	}

	st->pruss = pruss_get(st->rproc);
	if (!st->pruss) {
		dev_err(&pdev->dev, "error did not get pruss\n");
		free_dma(st);
		unregister_rpmsg_driver(&rpmsg_pru_driver);
		return -ENOMEM;
	}
	st->id = pru_rproc_get_id(st->rproc);

	/// start pru execution
	rproc_boot(st->rproc);
	dev_dbg(st->dev, "%s() rproc_boot()\n", __func__);

	for (i = 0; i <indio_dev->num_channels - 1; i++)
		st->calibscale[i] = 100000;

	init_completion(&st->completion);

	st->state = 1;
	st->bufferd = 0;

	return devm_iio_device_register(dev, indio_dev);
}

#ifdef CONFIG_PM_SLEEP

static int pru_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct pru_priv *st = iio_priv(indio_dev);
	st->state = 0;

	return 0;
}

static int pru_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct pru_priv *st = iio_priv(indio_dev);
	st->state = 1;

	return 0;
}

SIMPLE_DEV_PM_OPS(pru_adc_pm_ops, pru_suspend, pru_resume);
EXPORT_SYMBOL_GPL(pru_adc_pm_ops);

#endif

static int pru_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct pru_priv *st = iio_priv(indio_dev);

	dev_info(st->dev, "%s()\n", __func__);

	st->state = 0;

	/// stop pru execution
	rproc_shutdown(st->rproc);
	/// disable intc
	pruss_put(st->pruss);

	unregister_rpmsg_driver(&rpmsg_pru_driver);

	free_dma(st);

	return 0;
}

static struct platform_driver pru_adc = {
	.probe		= pru_probe,
	.remove		= pru_remove,
	.driver		= {
		.name	= "pru-adc",
		.of_match_table = of_pru_adc_match,
	},
};

module_platform_driver(pru_adc);
MODULE_AUTHOR("Robert Woerle <rwoerle@mibtec.de>");
MODULE_DESCRIPTION("PRU ADC remoteproc skeleton");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pru-adc");
