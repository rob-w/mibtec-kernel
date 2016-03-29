/*
 * mcp4725.c - Support for Microchip MCP4725
 *
 * Copyright (C) 2012 Peter Meerwald <pmeerw@pmeerw.net>
 *
 * Based on max517 by Roland Stigge <stigge@antcom.de>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * driver for the Microchip I2C 12-bit digital-to-analog converter (DAC)
 * (7-bit I2C slave address 0x60, the three LSBs can be configured in
 * hardware)
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/iio/dac/mcp4725.h>

#define DRIVER_VERSION "v1.1"

#define MCP4725_DRV_NAME "mcp4725"

#define TYPE_VOLTAGE	0
#define TYPE_CURRENT	1

#define	MAX_12BIT			((1 << 12) - 1)


struct mcp4725_data {
	struct i2c_client *client;
	u16 vref_mv;
	u16 dac_value[4];
	u32 dac_scale[4];
	bool powerdown;
	u16 channel_type;
	unsigned powerdown_mode;
};

static int mcp4725_suspend(struct device *dev)
{
	struct mcp4725_data *data = iio_priv(i2c_get_clientdata(
		to_i2c_client(dev)));
	u8 outbuf[2];

	outbuf[0] = (data->powerdown_mode + 1) << 4;
	outbuf[1] = 0;
	data->powerdown = true;

	return i2c_master_send(data->client, outbuf, 2);
}

static int mcp4725_resume(struct device *dev)
{
	struct mcp4725_data *data = iio_priv(i2c_get_clientdata(
		to_i2c_client(dev)));
	u8 outbuf[8];

	/* restore previous DAC value */
	outbuf[0] = (data->dac_value[0] >> 8) & 0xf;
	outbuf[1] = data->dac_value[0] & 0xff;
	outbuf[2] = (data->dac_value[1] >> 8) & 0xf;
	outbuf[3] = data->dac_value[1] & 0xff;
	outbuf[4] = (data->dac_value[2] >> 8) & 0xf;
	outbuf[5] = data->dac_value[2] & 0xff;
	outbuf[6] = (data->dac_value[3] >> 8) & 0xf;
	outbuf[7] = data->dac_value[3] & 0xff;
	data->powerdown = false;

	return i2c_master_send(data->client, outbuf, 8);
}

#ifdef CONFIG_PM_SLEEP
static SIMPLE_DEV_PM_OPS(mcp4725_pm_ops, mcp4725_suspend, mcp4725_resume);
#define MCP4725_PM_OPS (&mcp4725_pm_ops)
#else
#define MCP4725_PM_OPS NULL
#endif

static ssize_t mcp4725_store_eeprom(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct mcp4725_data *data = iio_priv(indio_dev);
	int tries = 20;
	u8 inoutbuf[3];
	bool state;
	int ret;

	ret = strtobool(buf, &state);
	if (ret < 0)
		return ret;

	if (!state)
		return 0;

	inoutbuf[0] = 0x60; /* write EEPROM */
	inoutbuf[1] = data->dac_value[0] >> 4;
	inoutbuf[2] = (data->dac_value[0] & 0xf) << 4;

	ret = i2c_master_send(data->client, inoutbuf, 3);
	if (ret < 0)
		return ret;
	else if (ret != 3)
		return -EIO;

	/* wait for write complete, takes up to 50ms */
	while (tries--) {
		msleep(20);
		ret = i2c_master_recv(data->client, inoutbuf, 3);
		if (ret < 0)
			return ret;
		else if (ret != 3)
			return -EIO;

		if (inoutbuf[0] & 0x80)
			break;
	}

	if (tries < 0) {
		dev_err(&data->client->dev,
			"mcp4725_store_eeprom() failed, incomplete\n");
		return -EIO;
	}

	return len;
}

static IIO_DEVICE_ATTR(store_eeprom, S_IWUSR, NULL, mcp4725_store_eeprom, 0);

static struct attribute *mcp4725_attributes[] = {
	&iio_dev_attr_store_eeprom.dev_attr.attr,
	NULL,
};

static const struct attribute_group mcp4725_attribute_group = {
	.attrs = mcp4725_attributes,
};

static const char * const mcp4725_powerdown_modes[] = {
	"1kohm_to_gnd",
	"100kohm_to_gnd",
	"500kohm_to_gnd"
};

static int mcp4725_get_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct mcp4725_data *data = iio_priv(indio_dev);

	return data->powerdown_mode;
}

static int mcp4725_set_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned mode)
{
	struct mcp4725_data *data = iio_priv(indio_dev);

	data->powerdown_mode = mode;

	return 0;
}

static ssize_t mcp4725_read_powerdown(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct mcp4725_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", data->powerdown);
}

static ssize_t mcp4725_write_powerdown(struct iio_dev *indio_dev,
	 uintptr_t private, const struct iio_chan_spec *chan,
	 const char *buf, size_t len)
{
	struct mcp4725_data *data = iio_priv(indio_dev);
	bool state;
	int ret;

	ret = strtobool(buf, &state);
	if (ret)
		return ret;

	if (state)
		ret = mcp4725_suspend(&data->client->dev);
	else
		ret = mcp4725_resume(&data->client->dev);
	if (ret < 0)
		return ret;

	return len;
}

static const struct iio_enum mcp4725_powerdown_mode_enum = {
	.items = mcp4725_powerdown_modes,
	.num_items = ARRAY_SIZE(mcp4725_powerdown_modes),
	.get = mcp4725_get_powerdown_mode,
	.set = mcp4725_set_powerdown_mode,
};

static const struct iio_chan_spec_ext_info mcp4725_ext_info[] = {
	{
		.name = "powerdown",
		.read = mcp4725_read_powerdown,
		.write = mcp4725_write_powerdown,
		.shared = IIO_SEPARATE,
	},
	IIO_ENUM("powerdown_mode", IIO_SEPARATE, &mcp4725_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", &mcp4725_powerdown_mode_enum),
	{ },
};

#define MCP472x_CHAN(chan, _type) {	\
	.type = (_type),			\
	.output = 1,					\
	.indexed = 1,					\
	.channel = chan,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)|\
				BIT(IIO_CHAN_INFO_CALIBSCALE) |\
				BIT(IIO_CHAN_INFO_SCALE),\
	.ext_info = mcp4725_ext_info,	\
}

static const struct iio_chan_spec mcp4725_channel[2][4] = {
	[TYPE_VOLTAGE] = {
		MCP472x_CHAN(0, IIO_VOLTAGE),
		MCP472x_CHAN(1, IIO_VOLTAGE),
		MCP472x_CHAN(2, IIO_VOLTAGE),
		MCP472x_CHAN(3, IIO_VOLTAGE),
	},
	[TYPE_CURRENT] = {
		MCP472x_CHAN(0, IIO_CURRENT),
		MCP472x_CHAN(1, IIO_CURRENT),
		MCP472x_CHAN(2, IIO_CURRENT),
		MCP472x_CHAN(3, IIO_CURRENT),
	},
};

static int mcp4725_set_value(struct iio_dev *indio_dev, int val)
{
	struct mcp4725_data *data = iio_priv(indio_dev);
	u8 outbuf[2];
	int ret;

	if (val >= (1 << 12) || val < 0)
		return -EINVAL;

	outbuf[0] = (val >> 8) & 0xf;
	outbuf[1] = val & 0xff;

	ret = i2c_master_send(data->client, outbuf, 2);
	if (ret < 0)
		return ret;
	else if (ret != 2)
		return -EIO;
	else
		return 0;
}

static int mcp4725_set_all(struct iio_dev *indio_dev)
{
	struct mcp4725_data *data = iio_priv(indio_dev);
	u8 outbuf[8];
	int ret;

	/// setting internal Vref to 2.0v
	outbuf[0] = 0xC0;

	ret = i2c_master_send(data->client, outbuf, 1);
	if (ret < 0)
		return ret;
	else if (ret != 1)
		return -EIO;

	outbuf[0] = (data->dac_value[0] >> 8) & 0xf;
	outbuf[1] = data->dac_value[0] & 0xff;

	outbuf[2] = (data->dac_value[1] >> 8) & 0xf;
	outbuf[3] = data->dac_value[1] & 0xff;

	outbuf[4] = (data->dac_value[2] >> 8) & 0xf;
	outbuf[5] = data->dac_value[2] & 0xff;

	outbuf[6] = (data->dac_value[3] >> 8) & 0xf;
	outbuf[7] = data->dac_value[3] & 0xff;

	ret = i2c_master_send(data->client, outbuf, 8);
	if (ret < 0)
		return ret;
	else if (ret != 8)
		return -EIO;
	else
		return 0;
}

static int mcp4725_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct mcp4725_data *data = iio_priv(indio_dev);
	unsigned long long tmp;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = data->dac_value[chan->channel];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = data->dac_scale[chan->channel];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_CALIBSCALE:
		tmp = div_s64(((s64)data->dac_scale[chan->channel] * 1000LL), MAX_12BIT);
		*val = div_s64((s64)(data->dac_value[chan->channel] * tmp), 1000LL);
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int mcp4725_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct mcp4725_data *data = iio_priv(indio_dev);
	int ret;
	unsigned long long tmp;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val > MAX_12BIT || val < 0)
			return -EINVAL;
		data->dac_value[chan->channel] = val;
		ret = mcp4725_set_all(indio_dev);
		break;

	case IIO_CHAN_INFO_SCALE:
		data->dac_scale[chan->channel] = val;

	case IIO_CHAN_INFO_CALIBSCALE:
		if (val >= data->dac_scale[chan->channel])
			val = data->dac_scale[chan->channel];

		tmp = div_s64(((s64)data->dac_scale[chan->channel] * 1000LL), MAX_12BIT);

		if (tmp > 0)
			data->dac_value[chan->channel] =  div_s64((s64)(val * 1000LL), tmp );

		ret = mcp4725_set_all(indio_dev);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct iio_info mcp4725_info = {
	.read_raw = mcp4725_read_raw,
	.write_raw = mcp4725_write_raw,
	.attrs = &mcp4725_attribute_group,
	.driver_module = THIS_MODULE,
};

#if defined(CONFIG_OF)
static int mcp4725_of_probe(struct i2c_client *client,
		struct mcp4725_data *pdata)
{
	struct device *dev = &client->dev;
	u32 val;
	int err;
	int scale = 23000;

	pdata->vref_mv = 2500;
	pdata->channel_type = 0;

	err = of_property_read_u32(dev->of_node, "vref_mv", &val);
	if (err)
		dev_info(&client->dev, "missing vref_mv in dt table, assuming 2500mV\n");
	if (val >= 0 && val < 5000)
		pdata->vref_mv = val;

	err = of_property_read_u32(dev->of_node, "channel_type", &val);
	if (err) {
		dev_err(&client->dev, "missing channel_type in dt table, assuming 0\n");
	} else {
		if (val == 0 || val == 1)
			pdata->channel_type = val;
	}

	err = of_property_read_u32(dev->of_node, "scale", &val);
	if (err) {
		dev_err(&client->dev, "missing scale in dt table, assuming 23000\n");
	} else {
		if (val >= 0)
			scale = val;
	}

	pdata->dac_scale[0] = scale;
	pdata->dac_scale[1] = scale;
	pdata->dac_scale[2] = scale;
	pdata->dac_scale[3] = scale;

	return 0;
}

static const struct of_device_id mcp4725_dt_ids[] = {
	{
		.compatible = "mcp4725",
	}, {
		.compatible = "mcp4728",
	},{
	}
};
MODULE_DEVICE_TABLE(of, mcp4725_dt_ids);
#else
static inline int mcp4725_of_probe(struct i2c_client *client,
		struct mcp4725_data *pdata)
{
	return -ENODEV;
}
#endif

static int mcp4725_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct mcp4725_data *data;
	struct iio_dev *indio_dev;
	struct mcp4725_platform_data *platform_data = client->dev.platform_data;
	u8 inbuf[3];
	u8 pd;
	int err;

	dev_info(&client->dev, "DAC 12bit - %s\n", DRIVER_VERSION);

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (indio_dev == NULL)
		return -ENOMEM;
	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	if (client->dev.of_node) {
		err = mcp4725_of_probe(client, data);
		if (err)
			dev_err(&client->dev, "invalid devicetree data");
	} else {
		if (!platform_data || !platform_data->vref_mv) {
			dev_err(&client->dev, "invalid platform data");
			return -EINVAL;
		}
		data->vref_mv = platform_data->vref_mv;
		data->channel_type = 0;
	}

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &mcp4725_info;
	indio_dev->channels = mcp4725_channel[data->channel_type];
	indio_dev->num_channels = 4;
	indio_dev->modes = INDIO_DIRECT_MODE;

	/* read current DAC value */
	err = i2c_master_recv(client, inbuf, 3);
	if (err < 0) {
		dev_err(&client->dev, "failed to read DAC value");
		return err;
	}
	pd = (inbuf[0] >> 1) & 0x3;
	data->powerdown = pd > 0 ? true : false;
	data->powerdown_mode = pd ? pd-1 : 2; /* 500kohm_to_gnd */
	data->dac_value[0] = (inbuf[1] << 4) | (inbuf[2] >> 4);

	return iio_device_register(indio_dev);
}

static int mcp4725_remove(struct i2c_client *client)
{
	iio_device_unregister(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id mcp4725_id[] = {
	{ "mcp4725", 0 },
	{ "mcp4728", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mcp4725_id);

static struct i2c_driver mcp4725_driver = {
	.driver = {
		.name	= MCP4725_DRV_NAME,
		.pm	= MCP4725_PM_OPS,
	},
	.probe		= mcp4725_probe,
	.remove		= mcp4725_remove,
	.id_table	= mcp4725_id,
};
module_i2c_driver(mcp4725_driver);

MODULE_AUTHOR("Peter Meerwald <pmeerw@pmeerw.net>");
MODULE_DESCRIPTION("MCP4725 12-bit DAC");
MODULE_LICENSE("GPL");
