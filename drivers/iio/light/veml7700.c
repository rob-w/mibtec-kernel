/*
 * veml7700.c - Support for Vishay VEML7700 ambient light sensor
 *
 * Copyright (C) 2016 Robert Woerle <robert@linuxdevelopment.de>
 * Author: Robert Woerle <robert@linuxdevelopment.de>
 * based on vcnl4400.c - Copyright 2012 Peter Meerwald <pmeerw@pmeerw.net>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for VEML7700 (7-bit I2C slave address 0x10)
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define VEML7700_DRV_NAME "veml7700"

#define VEML7700_CMD_CONFIG			0x00
#define VEML7700_CMD_HIGH_THR_WIN	0x01
#define VEML7700_CMD_LOW_THR_WIN	0x02
#define VEML7700_CMD_ALS_HIGH_RES	0x04
#define VEML7700_CMD_WHITE_OUT		0x05
#define VEML7700_CMD_IRG_STATUS		0x06

#define VEML7700_CFG_ALS_IT_100MS		0x00
#define VEML7700_CFG_ALS_IT_200MS		0x01
#define VEML7700_CFG_ALS_IT_400MS		0x02
#define VEML7700_CFG_ALS_IT_800MS		0x03
#define VEML7700_CFG_ALS_IT_50MS		0x08
#define VEML7700_CFG_ALS_IT_25MS		0x0C

#define VEML7700_CFG_ALS_SM_1			0x00
#define VEML7700_CFG_ALS_SM_2			0x01
#define VEML7700_CFG_ALS_SM_1_8			0x02
#define VEML7700_CFG_ALS_SM_1_4			0x03


struct veml7700_data {
	struct i2c_client *client;
	unsigned gain_mode;
	unsigned sample_mode;
	u16 gain;
	u16 it;
};


static const int veml7700_it_ms[6] = {
	100, 200, 400, 800, 50, 25
};

static const int veml7700_it[6] = {
	VEML7700_CFG_ALS_IT_100MS,
	VEML7700_CFG_ALS_IT_200MS,
	VEML7700_CFG_ALS_IT_400MS,
	VEML7700_CFG_ALS_IT_800MS,
	VEML7700_CFG_ALS_IT_50MS,
	VEML7700_CFG_ALS_IT_25MS
};

static const int veml7700_scales[6][4] = {
	{ 576, 288, 4608, 2304 },
	{ 288, 144, 2304, 1152 },
	{ 144, 72, 1152, 576 },
	{ 72, 36, 576, 288 },
	{ 1152, 576, 9216, 4608 },
	{ 2304, 1152, 18432, 9216 }
};

static const struct i2c_device_id veml7700_id[] = {
	{ "veml7700", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, veml7700_id);

static int veml7700_measure(struct veml7700_data *data, int *val)
{
	int ret;
	int cfg = 0;

	cfg =  data->gain << 11 | veml7700_it[data->it] << 6;

	ret = i2c_smbus_write_byte_data(data->client, VEML7700_CMD_CONFIG, cfg);
	if (ret < 0)
		return ret;

	msleep(veml7700_it_ms[data->it]);

	ret = i2c_smbus_read_word_data(data->client, VEML7700_CMD_ALS_HIGH_RES);
	*val = ret;

	return 0;
}

static const char * const veml7700_gain_modes[] = {
	"1x",
	"2x",
	"0.125x",
	"0.25x",
};

static const char * const veml7700_sample_modes[] = {
	"100ms",
	"200ms",
	"400ms",
	"800ms",
	"50ms",
	"25ms",
};

static int veml7700_get_gain_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct veml7700_data *data = iio_priv(indio_dev);

	return data->gain_mode;
}

static int veml7700_get_sample_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct veml7700_data *data = iio_priv(indio_dev);

	return data->sample_mode;
}

static const struct iio_enum veml7700_gain_enum = {
	.items = veml7700_gain_modes,
	.num_items = ARRAY_SIZE(veml7700_gain_modes),
	.get = veml7700_get_gain_mode,
};

static const struct iio_enum veml7700_sample_enum = {
	.items = veml7700_sample_modes,
	.num_items = ARRAY_SIZE(veml7700_sample_modes),
	.get = veml7700_get_sample_mode,
};

static const struct iio_chan_spec_ext_info veml7700_ext_info[] = {
	IIO_ENUM_AVAILABLE("sampling", &veml7700_sample_enum),
	IIO_ENUM_AVAILABLE("hardwaregain", &veml7700_gain_enum),
	{ },
};

static const struct iio_chan_spec veml7700_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
							| BIT(IIO_CHAN_INFO_SAMP_FREQ)
							| BIT(IIO_CHAN_INFO_CALIBSCALE)
							| BIT(IIO_CHAN_INFO_SCALE)
							| BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.ext_info = veml7700_ext_info,	\
	}
};

static int veml7700_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	int ret = -EINVAL;
	struct veml7700_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = veml7700_measure(data, val);
		if (ret < 0)
			return ret;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = veml7700_measure(data, val);
		if (ret < 0)
			return ret;
		*val = (*val * veml7700_scales[data->it][data->gain]) / 10000;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = data->it;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*val = data->gain;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = veml7700_scales[data->it][data->gain] * 10;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	default:
		break;
	}

	return ret;
}

static int veml7700_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct veml7700_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (val > 3 || val < 0)
			return -EINVAL;
		data->gain = val;
		ret = 0;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val > 5 || val < 0)
			return -EINVAL;
		data->it = val;
		ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct iio_info veml7700_info = {
	.read_raw = veml7700_read_raw,
	.write_raw = veml7700_write_raw,
	.driver_module = THIS_MODULE,
};

static int veml7700_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct veml7700_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	ret = i2c_smbus_write_byte_data(data->client, VEML7700_CMD_CONFIG, 0);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte_data(data->client, VEML7700_CMD_ALS_HIGH_RES);
	if (ret < 0)
		return ret;

	dev_info(&client->dev, "VEML7700 Ambient light sensor\n");

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &veml7700_info;
	indio_dev->channels = veml7700_channels;
	indio_dev->num_channels = ARRAY_SIZE(veml7700_channels);
	indio_dev->name = VEML7700_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;

	return devm_iio_device_register(&client->dev, indio_dev);
}

#ifdef CONFIG_OF
static const struct of_device_id veml7700_of_match[] = {
	{ .compatible = "veml7700" },
	{ }
};
MODULE_DEVICE_TABLE(of, veml7700_of_match);
#endif

static struct i2c_driver veml7700_driver = {
	.driver = {
		.name   = VEML7700_DRV_NAME,
		.owner  = THIS_MODULE,
	},
	.probe  = veml7700_probe,
	.id_table = veml7700_id,
};

module_i2c_driver(veml7700_driver);

MODULE_AUTHOR("Robert Woerle <robert@linuxdevelopment.de>");
MODULE_DESCRIPTION("Vishay VEML7700 proximity/ambient light sensor driver");
MODULE_LICENSE("GPL");
