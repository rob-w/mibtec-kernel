/*
 * si838x spi based bipolar input controller
 * Copyright (c) 2021 Mibtec.de
 * 
 * base on drivers/gpio/gpio-mc33880.c
 * Copyright (c) 2009 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* Supports:
 * Silicon Labs 838x bipolar input controller
 */

#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/spi/si838x.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/module.h>

#define DRIVER_NAME "si838x"

/*
 * Pin configurations, see MAX7301 datasheet page 6
 */
#define PIN_CONFIG_MASK 0x00
#define PIN_CONFIG_IN_PULLUP 0x00
#define PIN_CONFIG_IN_WO_PULLUP 0x00
#define PIN_CONFIG_OUT 0x00

#define PIN_NUMBER 8


/*
 * Some registers must be read back to modify.
 * To save time we cache them here in memory
 */
struct si838x {
	struct mutex	lock;	/* protect from simultaneous accesses */
	u8		port_config;
	struct gpio_chip chip;
	struct spi_device *spi;
};

static int si838x_write_config(struct si838x *mc)
{
	return spi_write(mc->spi, &mc->port_config, sizeof(mc->port_config));
}


static int __si838x_set(struct si838x *mc, unsigned offset, int value)
{
	if (value)
		mc->port_config |= 1 << offset;
	else
		mc->port_config &= ~(1 << offset);

	return si838x_write_config(mc);
}


static void si838x_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct si838x *mc = container_of(chip, struct si838x, chip);

	mutex_lock(&mc->lock);

	__si838x_set(mc, offset, value);

	mutex_unlock(&mc->lock);
}

static int si838x_probe(struct spi_device *spi)
{
	struct si838x *mc;
	struct si838x_platform_data *pdata;
	int ret;

	pdata = dev_get_platdata(&spi->dev);
	if (!pdata || !pdata->base) {
		dev_dbg(&spi->dev, "incorrect or missing platform data\n");
		return -EINVAL;
	}

	/*
	 * bits_per_word cannot be configured in platform data
	 */
	spi->bits_per_word = 8;

	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	mc = devm_kzalloc(&spi->dev, sizeof(struct si838x), GFP_KERNEL);
	if (!mc)
		return -ENOMEM;

	mutex_init(&mc->lock);

	spi_set_drvdata(spi, mc);

	mc->spi = spi;

	mc->chip.label = DRIVER_NAME,
	mc->chip.set = si838x_set;
	mc->chip.base = pdata->base;
	mc->chip.ngpio = PIN_NUMBER;
	mc->chip.can_sleep = true;
	mc->chip.dev = &spi->dev;
	mc->chip.owner = THIS_MODULE;

	mc->port_config = 0x00;
	/* write twice, because during initialisation the first setting
	 * is just for testing SPI communication, and the second is the
	 * "real" configuration
	 */
	ret = si838x_write_config(mc);
	mc->port_config = 0x00;
	if (!ret)
		ret = si838x_write_config(mc);

	if (ret) {
		dev_err(&spi->dev, "Failed writing to " DRIVER_NAME ": %d\n",
			ret);
		goto exit_destroy;
	}

	ret = gpiochip_add(&mc->chip);
	if (ret)
		goto exit_destroy;

	return ret;

exit_destroy:
	mutex_destroy(&mc->lock);
	return ret;
}

static int si838x_remove(struct spi_device *spi)
{
	struct si838x *mc;

	mc = spi_get_drvdata(spi);
	if (!mc)
		return -ENODEV;

	gpiochip_remove(&mc->chip);
	mutex_destroy(&mc->lock);

	return 0;
}

static struct spi_driver si838x_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= si838x_probe,
	.remove		= si838x_remove,
};

static int __init si838x_init(void)
{
	return spi_register_driver(&si838x_driver);
}
/* register after spi postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(si838x_init);

static void __exit si838x_exit(void)
{
	spi_unregister_driver(&si838x_driver);
}
module_exit(si838x_exit);

MODULE_AUTHOR("Mocean Laboratories <info@mocean-labs.com>");
MODULE_LICENSE("GPL v2");

