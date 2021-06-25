/*
 * Copyright (c) 2021 MIBTEC.de robert@linuxdevelopment,de
 *  base on drivers/gpio/gpio-pisosr.c
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether expressed or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License version 2 for more details.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/spi/spi.h>

#define SI838X_MODULE_VERSION "0.4"

#define DEFAULT_NGPIO 8

/**
 * struct pisosr_gpio - GPIO driver data
 * @chip: GPIO controller chip
 * @spi: SPI device pointer
 * @buffer: Buffer for device reads
 * @buffer_size: Size of buffer
 * @lock: Protects read sequences
 */
struct si838x_gpio {
	struct		gpio_chip chip;
	struct		spi_device *spi;
	u8			*buffer;
	size_t		buffer_size;
	struct		mutex lock;
	bool		inv_order;
};

static inline struct si838x_gpio *to_si838x_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct si838x_gpio, chip);
}

#define SI838x_ADR_CHAN_STATUS		0x0
#define SI838x_ADR_DBNC_MODE0		0x1
#define SI838x_ADR_DBNC_MODE1		0x2
#define SI838x_ADR_DBNC_DLY0		0x3
#define SI838x_ADR_DBNC_DLY1		0x4

#define SI838x_BRCAT			(1<<7)
#define SI838x_READ_EN			(1<<6)

#define SI838x_M_DEGLITCH		0x0
#define SI838x_M_LOWPASS		0x1
#define SI838x_M_BLANKING		0x2

#define SI838x_D_BOUNCE_0_MS		0x0
#define SI838x_D_BOUNCE_10_MS		0x1
#define SI838x_D_BOUNCE_30_MS		0x2
#define SI838x_D_BOUNCE_100_MS		0x3

static int si838x_gpio_set_debounce(struct gpio_chip *chip, unsigned offset, unsigned debounce)
{
	int ret;
	u8 tx_buf[3];
	struct si838x_gpio *gpio = to_si838x_gpio(chip);

	mutex_lock(&gpio->lock);

	tx_buf[0] = SI838x_BRCAT;
	tx_buf[1] = SI838x_ADR_DBNC_MODE0;

	ret = spi_write(gpio->spi, &tx_buf, sizeof(tx_buf));
	if (ret)
		return ret;

	dev_info(gpio->chip.dev, "%s() buf %d %d\n", __func__, tx_buf[0], tx_buf[1]);

	mutex_unlock(&gpio->lock);

	return 0;
}

static int si838x_gpio_refresh(struct si838x_gpio *gpio)
{
	int ret, i;
	u8 dat;
	u8 tx_buf[2];

	mutex_lock(&gpio->lock);

	tx_buf[0] = 0x40;
	tx_buf[1] = 0x0;

	ret = spi_write_then_read(gpio->spi, &tx_buf, sizeof(tx_buf), gpio->buffer, gpio->buffer_size);
	if (ret)
		return ret;

	if (gpio->inv_order) {
		dat = gpio->buffer[0];
		gpio->buffer[0] = 0;
		for (i = 0; i < 8; i++) {
			if (dat & (1<<i))
				gpio->buffer[0] |= (1<< (7 - i));
		}
	}

	mutex_unlock(&gpio->lock);

	return 0;
}

static int si838x_gpio_get_direction(struct gpio_chip *chip,
				     unsigned offset)
{
	/* This device always input */
	return 1;
}

static int si838x_gpio_direction_input(struct gpio_chip *chip,
				       unsigned offset)
{
	/* This device always input */
	return 0;
}

static int si838x_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset, int value)
{
	/* This device is input only */
	return -EINVAL;
}

static void si838x_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
/*	struct si838x_gpio *si838x = to_si838x_gpio(chip);
	u8 temp;
	unsigned long flags;*/
}

static int si838x_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct si838x_gpio *gpio = to_si838x_gpio(chip);
	int ret;

	si838x_gpio_refresh(gpio);

	/// to stay stupidly compatible we invert here again
	if ((gpio->buffer[offset / 8] >> (offset % 8)) & 0x1)
		ret = 0;
	else
		ret = 1;
	return (ret);
}

static struct gpio_chip template_chip = {
	.label			= "si838x-gpio",
	.owner			= THIS_MODULE,
	.get_direction		= si838x_gpio_get_direction,
	.direction_input	= si838x_gpio_direction_input,
	.direction_output	= si838x_gpio_direction_output,
	.get			= si838x_gpio_get,
	.set			= si838x_gpio_set,
	.set_debounce	= si838x_gpio_set_debounce,
	.base			= -1,
	.ngpio			= DEFAULT_NGPIO,
	.can_sleep		= true,
};

static int si838x_gpio_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct si838x_gpio *gpio;
	int ret;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	spi_set_drvdata(spi, gpio);

	gpio->chip = template_chip;
	gpio->chip.dev = dev;
	of_property_read_u16(dev->of_node, "ngpios", &gpio->chip.ngpio);
	gpio->inv_order = of_property_read_bool(dev->of_node, "invert_order");

	gpio->spi = spi;

	gpio->buffer_size = DIV_ROUND_UP(gpio->chip.ngpio, 8);
	gpio->buffer = devm_kzalloc(dev, gpio->buffer_size, GFP_KERNEL);
	if (!gpio->buffer)
		return -ENOMEM;

	mutex_init(&gpio->lock);

	dev_info(dev, "v%s\n", SI838X_MODULE_VERSION);

	ret = gpiochip_add(&gpio->chip);
	if (ret < 0) {
		dev_err(dev, "Unable to register gpiochip\n");
		return ret;
	}

	return 0;
}

static int si838x_gpio_remove(struct spi_device *spi)
{
	struct si838x_gpio *gpio = spi_get_drvdata(spi);

	gpiochip_remove(&gpio->chip);

	mutex_destroy(&gpio->lock);

	return 0;
}

static const struct spi_device_id si838x_gpio_id_table[] = {
	{ "si838x-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, si838x_gpio_id_table);

static const struct of_device_id si838x_gpio_of_match_table[] = {
	{ .compatible = "si838x-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, si838x_gpio_of_match_table);

static struct spi_driver si838x_gpio_driver = {
	.driver = {
		.name = "si838x-gpio",
		.of_match_table = si838x_gpio_of_match_table,
	},
	.probe = si838x_gpio_probe,
	.remove = si838x_gpio_remove,
	.id_table = si838x_gpio_id_table,
};
module_spi_driver(si838x_gpio_driver);

MODULE_AUTHOR("Robert Woerle <robert@linuxdevelopment.de>");
MODULE_DESCRIPTION("SPI Compatible SI838x INPUT GPIO Driver");
MODULE_LICENSE("GPL v2");
