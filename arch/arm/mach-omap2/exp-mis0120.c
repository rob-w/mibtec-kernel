/*
 * Copyright (C) 2010-2011 - ISEE 2007 SL
 * Copyright (C) 2010-2012 - MIS 2012
 * 
 * MIS IgepV2 OQ Expansion 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>

#include <linux/spi/spi.h>
#include <linux/spi/max1233.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/mcspi.h>

#include "board-igep00x0.h"
#include "mux.h"

#define IGEP2_LED1_GPIO			132
#define IGEP2_LED2_GPIO			133
#define IGEP2_LED3_GPIO			134
#define IGEP2_LED4_GPIO			135
#define IGEP2_SUMMER_GPIO		136
#define IGEP2_LCD_RESET			137
#define IGEP2_LCD_POWER			138
#define IGEP2_KP_IRQ			139
#define IGEP2_RS485_GPIO		149
#define IGEP2_MAX1233_TOUCH		175


static void request_gpios(void)
{
	if ((gpio_request(IGEP2_LCD_RESET, "GPIO_LCD_RESET") == 0) && (gpio_direction_output(IGEP2_LCD_RESET, 1) == 0))
		gpio_export(IGEP2_LCD_RESET, 1);
	else
		printk(KERN_ERR "could not obtain gpio for " "LCD_RESET\n");

	if ((gpio_request(IGEP2_LCD_POWER, "GPIO_LCD_POWER") == 0) && (gpio_direction_output(IGEP2_LCD_POWER, 1) == 0))
		gpio_export(IGEP2_LCD_POWER, 1);
	else
		printk(KERN_ERR "could not obtain gpio for " "LCD_POWER\n");

	gpio_set_value(IGEP2_LCD_RESET, 1);

	/// we need to pull this one down on dm3730 igepv2-C to disable WIFI and have our LEDS free to toggle
    if (cpu_is_omap3630())
		gpio_set_value(IGEP2_LCD_POWER, 1);
	else {
		gpio_set_value(IGEP2_LCD_POWER, 1);

		/// Make extra sure for old REVs and old boards with new u-boot ( which does not pull this one now)
		if ((gpio_request(94, "WIFI_NPD") == 0) && (gpio_direction_output(94, 1) == 0))
			gpio_export(94, 1);
		else
			printk(KERN_ERR "could not obtain gpio for " "94\n");

		gpio_set_value(94, 0);
	}

	if ((gpio_request(IGEP2_RS485_GPIO, "GPIO_RS485_GPIO") == 0) && (gpio_direction_output(IGEP2_RS485_GPIO, 1) == 0))
		gpio_export(IGEP2_RS485_GPIO, 1);
	else
		printk(KERN_ERR "could not obtain gpio for " "RS485_GPIO\n");

	if ((gpio_request(IGEP2_LED1_GPIO, "GPIO_LED1") == 0) && (gpio_direction_output(IGEP2_LED1_GPIO, 1) == 0))
		gpio_export(IGEP2_LED1_GPIO, 1);
	else
		printk(KERN_ERR "could not obtain gpio for " "LED1_GPIO\n");

	if ((gpio_request(IGEP2_LED2_GPIO, "GPIO_LED2") == 0) && (gpio_direction_output(IGEP2_LED2_GPIO, 1) == 0))
		gpio_export(IGEP2_LED2_GPIO, 1);
	else
		printk(KERN_ERR "could not obtain gpio for " "LED2_GPIO\n");

	gpio_set_value(IGEP2_LED2_GPIO, 1);

	if ((gpio_request(IGEP2_LED3_GPIO, "GPIO_LED3") == 0) && (gpio_direction_output(IGEP2_LED3_GPIO, 1) == 0))
		gpio_export(IGEP2_LED3_GPIO, 1);
	else
		printk(KERN_ERR "could not obtain gpio for " "LED3_GPIO\n");

	if ((gpio_request(IGEP2_LED4_GPIO, "GPIO_LED4") == 0) && (gpio_direction_output(IGEP2_LED4_GPIO, 1) == 0))
		gpio_export(IGEP2_LED4_GPIO, 1);
	else
		printk(KERN_ERR "could not obtain gpio for " "LED4_GPIO\n");

	if ((gpio_request(IGEP2_SUMMER_GPIO, "GPIO_SUMMER") == 0) && (gpio_direction_output(IGEP2_SUMMER_GPIO, 1) == 0))
		gpio_export(IGEP2_SUMMER_GPIO, 1);
	else
		printk(KERN_ERR "could not obtain gpio for " "SUMMER_GPIO\n");

	gpio_set_value(IGEP2_SUMMER_GPIO, 0);
	gpio_set_value(IGEP2_LED1_GPIO, 1);
	gpio_set_value(IGEP2_LED2_GPIO, 1);
	gpio_set_value(IGEP2_LED3_GPIO, 0);
}

static void max1233_dev_init(void)
{
	int ret = gpio_request(IGEP2_MAX1233_TOUCH, "max1233_pen_down");

	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for max1233 pen down IRQ\n",
			IGEP2_MAX1233_TOUCH);
	} else {
		printk(KERN_INFO "GPIO IRQ %d requested\n", IGEP2_MAX1233_TOUCH);
		gpio_export(IGEP2_MAX1233_TOUCH, 1);
	}
}

struct max1233_platform_data mis0120touch_config = {
	.x_max                  = 0x0fff,
	.y_max                  = 0x0fff,
	.x_plate_ohms           = 690,
	.y_plate_ohms           = 375,
	.pressure_max           = 255,
	.kp_irq                 = OMAP_GPIO_IRQ(IGEP2_KP_IRQ),
};

static struct omap2_mcspi_device_config max1233_mcspi_config = {
	.turbo_mode     = 0,
	.single_channel = 1,  /* 0: slave, 1: master */
};

static struct spi_board_info igep2_spi_board_info[] __initdata = {
	{
		.modalias               = "max1233",
		.bus_num                = 1,
		.chip_select            = 0,
		.max_speed_hz           = 2000000,
		.controller_data        = &max1233_mcspi_config,
		.irq                    = OMAP_GPIO_IRQ(IGEP2_MAX1233_TOUCH),
		.platform_data          = &mis0120touch_config,
		.mode                   = SPI_MODE_3,
	},
	{
		.modalias               = "spidev",
		.bus_num                = 2,
		.chip_select            = 0,
		.max_speed_hz           = 20000000,
		.mode                   = SPI_MODE_2,
	},
};

struct omap_dss_device mis0120_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.phy.dpi.data_lines	= 18,
};

static struct omap_dss_device *dss_devices[] = {
	&mis0120_dvi_device,
};

static struct omap_dss_board_info dss_board_data = {
	.num_devices	= ARRAY_SIZE(dss_devices),
	.devices	= dss_devices,
	.default_device	= &mis0120_dvi_device,
};

static struct platform_device mis0120_dss_device = {
	.name	= "omapdss",
	.id	= -1,
	.dev	= {
		.platform_data = &dss_board_data,
	},
};

void
mis0120_init_display(void)
{
 platform_device_register(&mis0120_dss_device);
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux mis0120_mux[] __initdata = {
	/* McSPI 1 */
	OMAP3_MUX(MCSPI1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC2_DAT0, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/// GPIO 132 LED1
	OMAP3_MUX(SDMMC2_DAT1, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/// GPIO 133 LED2
	OMAP3_MUX(SDMMC2_DAT2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/// GPIO 134 LED3
	OMAP3_MUX(SDMMC2_DAT3, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/// GPIO 135 LED4
	OMAP3_MUX(SDMMC2_DAT7, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/// GPIO 139 KP_IRQ
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define mis0120_mux	NULL
#endif

void __init mis0120_init(void)
{
	mux_partition = omap_mux_get("core");

	/* Mux initialitzation for mis0120 */
	omap_mux_write_array(mux_partition, mis0120_mux);

	request_gpios();
	spi_register_board_info(igep2_spi_board_info, ARRAY_SIZE(igep2_spi_board_info));
	max1233_dev_init();
	printk(KERN_INFO "exp-mis0120.c finished\n");
}
