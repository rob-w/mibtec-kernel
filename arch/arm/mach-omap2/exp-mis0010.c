/*
 * Copyright (C) 2010-2011 - ISEE 2007 SL
 * Copyright (C) 2010-2011 - MIB Robert Woerle
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 12" TOUCH Panel
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/smsc911x.h>

#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/mfd/twl4030-codec.h>

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/spi/pic32io.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/mcspi.h>
#include "twl-common.h"
#include "board-igep00x0.h"
#include "mux.h"

/* MIS */
#define IGEP3_GPIO_5VNOT		10
#define IGEP3_MIS_RS232EN		13
#define IGEP3_GPIO_IGEPIRQ1		12
#define IGEP3_GPIO_IGEPIRQ2		20
#define IGEP3_GPIO_IGEPIRQ3		23
#define IGEP3_GPIO_PICIRQ1		22
#define IGEP3_GPIO_PICIRQ2		19
#define IGEP3_GPIO_PICIRQ3		18

#define IGEP3_GPIO_PENIRQ		114
#define IGEP3_GPIO_SUMMER		144
#define IGEP3_RESET1			185
#define IGEP3_USBH_CPEN			168
#define IGEP3_USBH_CPEN_REV_E	53
#define IGEP3_MIS_LED1			41
#define IGEP3_MIS_LED2			42
#define IGEP3_MIS_LED3			43

#define IGEP3_LCD_NSHUTDOWN		128

#define IGEP3_LCD_ENABLE		170

/* SMSC911X Ethernet controller */
#define IGEP3_RA_SMSC911X0_CS       5
#define IGEP3_RA_SMSC911X0_IRQ		52
#define IGEP3_RA_SMSC911X0_NRESET	64
#define IGEP3_RA_SMSC911X1_CS       4
#define IGEP3_RA_SMSC911X1_IRQ		65
#define IGEP3_RA_SMSC911X1_NRESET	57

#define IGEP3_RB_SMSC911X0_CS       4
#define IGEP3_RB_SMSC911X0_IRQ		52
#define IGEP3_RB_SMSC911X0_NRESET	42
#define IGEP3_RB_SMSC911X1_CS       6
#define IGEP3_RB_SMSC911X1_IRQ		41
#define IGEP3_RB_SMSC911X1_NRESET	43

#define IGEP3_BOARD_HWREV_D	0xD
#define IGEP3_BOARD_HWREV_E	0xE

static u8 hwrev;

static void __init igep0030_get_revision(void)
{
	if (cpu_is_omap3630()) {
		pr_info("MIS: Hardware Rev. E\n");
		hwrev = IGEP3_BOARD_HWREV_E;
	} else {
		pr_info("MIS: Hardware Rev. D\n");
		hwrev = IGEP3_BOARD_HWREV_D;
	}
}

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)

static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags		= SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
};

static struct resource smsc911x0_resources[] = {
	{
		.name	= "smsc911x0-memory",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= -EINVAL,
		.end	= -EINVAL,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct resource smsc911x1_resources[] = {
	{
		.name	= "smsc911x1-memory",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= -EINVAL,
		.end	= -EINVAL,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device smsc911x0_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smsc911x0_resources),
	.resource	= smsc911x0_resources,
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static struct platform_device smsc911x1_device = {
	.name		= "smsc911x",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(smsc911x1_resources),
	.resource	= smsc911x1_resources,
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static inline void __init mis0010_smsc911x_init(void)
{
	/* SMSC911X Ethernet controller */
	if (igep00x0_buddy_pdata.revision & IGEP00X0_BUDDY_HWREV_A) {
		/* Configure MUX for hardware rev. A */
		omap_mux_init_signal("gpmc_ncs5", 0);
		omap_mux_init_gpio(IGEP3_RA_SMSC911X1_IRQ, OMAP_PIN_INPUT);
		omap_mux_init_gpio(IGEP3_RA_SMSC911X1_NRESET, OMAP_PIN_OUTPUT);

		smsc911x1_resources[1].start =
					OMAP_GPIO_IRQ(IGEP3_RA_SMSC911X1_IRQ);
		smsc911x1_resources[1].end =
					OMAP_GPIO_IRQ(IGEP3_RA_SMSC911X1_IRQ);

		/* Set up second smsc911x chip */
		igep00x0_smsc911x_init(&smsc911x1_device, IGEP3_RA_SMSC911X1_CS,
			IGEP3_RA_SMSC911X1_IRQ,	IGEP3_RA_SMSC911X1_NRESET);
	} else {
		/* Configure MUX for hardware rev. B */
		omap_mux_init_signal("gpmc_ncs6", 0);
		omap_mux_init_gpio(IGEP3_RB_SMSC911X1_IRQ, OMAP_PIN_INPUT);
		omap_mux_init_gpio(IGEP3_RB_SMSC911X1_NRESET, OMAP_PIN_OUTPUT);

		smsc911x1_resources[1].start =
					OMAP_GPIO_IRQ(IGEP3_RB_SMSC911X1_IRQ);
		smsc911x1_resources[1].end =
					OMAP_GPIO_IRQ(IGEP3_RB_SMSC911X1_IRQ);

		/* Set up second smsc911x chip */
		igep00x0_smsc911x_init(&smsc911x1_device, IGEP3_RB_SMSC911X1_CS,
			IGEP3_RB_SMSC911X1_IRQ,	IGEP3_RB_SMSC911X1_NRESET);
	}
}
#else
static inline void __init mis0010_smsc911x_init(void) { }
#endif

static int mis0010_enable_dvi(struct omap_dss_device *dssdev)
{
	gpio_direction_output(IGEP3_LCD_ENABLE, 1);

	return 0;
}

static void mis0010_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_direction_output(IGEP3_LCD_ENABLE, 0);
}

struct omap_dss_device mis0010_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "au121x01",
	.driver_name		= "au_panel",
	.phy.dpi.data_lines	= 24,
};

static struct omap_dss_device *mis0010_dss_devices[] = {
	&mis0010_dvi_device,
	&igep00x0_dvi_device,
};

static struct omap_dss_board_info mis0010_dss_data = {
	.num_devices	= ARRAY_SIZE(mis0010_dss_devices),
	.devices	= mis0010_dss_devices,
	.default_device	= &mis0010_dvi_device,
};

static struct platform_device mis0010_dss_device = {
	.name	= "omapdss",
	.id	= -1,
	.dev	= {
		.platform_data = &mis0010_dss_data,
	},
};

static inline void mis0010_display_init(void)
{
	if ((gpio_request(IGEP3_LCD_NSHUTDOWN, "NSHUTDOWN") == 0) &&
		(gpio_direction_output(IGEP3_LCD_NSHUTDOWN, 1) == 0)) {
		gpio_export(IGEP3_LCD_NSHUTDOWN, 0);
		gpio_set_value(IGEP3_LCD_NSHUTDOWN, 1);
		}
	else
		pr_warning("IGEP: Could not obtain gpio NSHUTDOWN\n"); 
		
	igep00x0_dvi_device.platform_enable = mis0010_enable_dvi;
	igep00x0_dvi_device.platform_disable = mis0010_disable_dvi;

	platform_device_register(&mis0010_dss_device);
}

static inline void mis0010_gpio_init(void)
{
	if ((gpio_request(IGEP3_GPIO_5VNOT, "5vnot") == 0) &&
		(gpio_direction_output(IGEP3_GPIO_5VNOT, 1) == 0)) {
		gpio_export(IGEP3_GPIO_5VNOT, 0);
		gpio_set_value(IGEP3_GPIO_5VNOT, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio GPIO_5vNOT\n"); 

	if ((gpio_request(IGEP3_GPIO_SUMMER, "buzzer") == 0) &&
		(gpio_direction_output(IGEP3_GPIO_SUMMER, 1) == 0)) {
		gpio_export(IGEP3_GPIO_SUMMER, 0);
		gpio_set_value(IGEP3_GPIO_SUMMER, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio GPIO_SUMMER\n"); 

	if (hwrev == IGEP3_BOARD_HWREV_D) {
		if ((gpio_request(IGEP3_USBH_CPEN, "usbh_cpen") == 0) &&
			(gpio_direction_output(IGEP3_USBH_CPEN, 1) == 0)) {
			gpio_export(IGEP3_USBH_CPEN, 0);
			gpio_set_value(IGEP3_USBH_CPEN, 0);
			}
		else
			pr_warning("IGEP: Could not obtain gpio USBH_CPEN\n"); 
		
	} else {
		/* Hardware Rev. E */
		if ((gpio_request(IGEP3_USBH_CPEN_REV_E, "usbh_cpen_rev_e") == 0) &&
			(gpio_direction_output(IGEP3_USBH_CPEN_REV_E, 1) == 0)) {
			gpio_export(IGEP3_USBH_CPEN_REV_E, 0);
			gpio_set_value(IGEP3_USBH_CPEN_REV_E, 0);
			}
		else
			pr_warning("IGEP: Could not obtain gpio USBH_CPEN REV E\n"); 
		}
		
	if ((gpio_request(IGEP3_RESET1, "reset1") == 0) &&
		(gpio_direction_output(IGEP3_RESET1, 1) == 0)) {
		gpio_export(IGEP3_RESET1, 0);
		gpio_set_value(IGEP3_RESET1, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio RESET1\n"); 

	if ((gpio_request(IGEP3_LCD_ENABLE, "LCD_ENABLE") == 0) &&
		(gpio_direction_output(IGEP3_LCD_ENABLE, 1) == 0)) {
		gpio_export(IGEP3_LCD_ENABLE, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio LCD_ENABLE\n"); 
	
	if ((gpio_request(IGEP3_MIS_RS232EN, "RS232_EN") == 0) &&
		(gpio_direction_output(IGEP3_MIS_RS232EN, 1) == 0)) {
		gpio_export(IGEP3_MIS_RS232EN, 0);
		gpio_set_value(IGEP3_MIS_RS232EN, 1);
		}
	else
		pr_warning("IGEP: Could not obtain gpio RS232_EN\n");

	if ((gpio_request(IGEP3_GPIO_IGEPIRQ1, "IGEPIRQ1") == 0) &&
		(gpio_direction_output(IGEP3_GPIO_IGEPIRQ1, 1) == 0)) {
		gpio_export(IGEP3_GPIO_IGEPIRQ1, 0);
		gpio_set_value(IGEP3_GPIO_IGEPIRQ1, 1);
		}
	else
		pr_warning("IGEP: Could not obtain gpio IGEPIRQ1\n");

	if ((gpio_request(IGEP3_GPIO_IGEPIRQ2, "IGEPIRQ2") == 0) &&
		(gpio_direction_output(IGEP3_GPIO_IGEPIRQ2, 1) == 0)) {
		gpio_export(IGEP3_GPIO_IGEPIRQ2, 0);
		gpio_set_value(IGEP3_GPIO_IGEPIRQ2, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio IGEPIRQ2\n");

	if ((gpio_request(IGEP3_GPIO_IGEPIRQ3, "IGEPIRQ3") == 0) &&
		(gpio_direction_output(IGEP3_GPIO_IGEPIRQ3, 1) == 0)) {
		gpio_export(IGEP3_GPIO_IGEPIRQ3, 0);
		gpio_set_value(IGEP3_GPIO_IGEPIRQ3, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio IGEPIRQ3\n");
}
		
static void spidevs_dev_init(void)
{
	int ret;

	ret = gpio_request(IGEP3_GPIO_5VNOT, "5vnot");
	if (ret < 0)
		printk(KERN_ERR "Failed to request for 5vnot\n");
	else
		gpio_export(IGEP3_GPIO_5VNOT, 1);
}

struct ads7846_platform_data tsc2046_config = {
        .x_max                  = 0x0fff,
        .y_max                  = 0x0fff,
        .x_plate_ohms           = 603,
        .y_plate_ohms           = 363,
        .pressure_max           = 0x0fff,
        .gpio_pendown			= IGEP3_GPIO_PENIRQ,
        .debounce_max           = 10,
        .debounce_tol           = 10,
        .debounce_rep           = 2,
};


static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
        .turbo_mode     = 0,
        .single_channel = 1,  /* 0: slave, 1: master */
};

static struct spi_board_info igep3_spi_board_info[] __initdata = {
        {
                .modalias               = "ads7846",
                .bus_num                = 1,
                .chip_select            = 1, /* 175 pwm0 168usb*/
                .max_speed_hz           = 2000000,
                .controller_data        = &tsc2046_mcspi_config,
                .irq                    = OMAP_GPIO_IRQ(IGEP3_GPIO_PENIRQ),
                .platform_data          = &tsc2046_config,
                .mode                   = SPI_MODE_2,
        },
};

static struct gpio_led gpio_led_data[] = {
	[0] = {
		.name = "mis-led1:green",
		.gpio = IGEP3_MIS_LED1,
		.default_trigger = "heartbeat",
		.active_low = false,
	},
	[1] = {
		.name = "mis-led2:green",
		.gpio = IGEP3_MIS_LED2,
		.default_trigger = "default-off",
		.active_low = false,
	},
	[2] = {
		.name = "mis-led3:green",
		.gpio = IGEP3_MIS_LED3,
		.default_trigger = "default-off",
		.active_low = false,
	},
};

static struct gpio_led_platform_data gpio_led_pdata = {
	.leds           = gpio_led_data,
	.num_leds       = ARRAY_SIZE(gpio_led_data),
};

static struct platform_device gpio_led_device = {
	 .name   = "leds-gpio",
	 .id     =  0,
	 .dev    = {
		 .platform_data = &gpio_led_pdata,
	},
};

static struct led_pwm twl4030_pwm_leds[] = {
	{
		.name		= "leda",
		.pwm_id		= 0,
		.max_brightness = 255,
		.default_trigger = "default-on",
	},{
		.name		= "ledb",
		.pwm_id		= 1,
	},
	{
		.name		= "pwm0",
		.pwm_id		= 2,
	},{
		.name		= "pwm1",
		.pwm_id		= 3,
		.default_trigger = "default-on",
	},
};

static struct led_pwm_platform_data twl4030_pwm_data = {
	.num_leds	= ARRAY_SIZE(twl4030_pwm_leds),
	.leds		= twl4030_pwm_leds,
};

static struct platform_device twl4030_leds_pwm = {
	.name	= "leds-twl4030-pwm",
	.id	= -1,
	.dev	= {
		.platform_data = &twl4030_pwm_data,
	},
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux mis0010_mux[] __initdata = {
		
	/* disable the others SP3 possibilitys*/
	OMAP3_MUX(SDMMC2_CLK, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC2_CMD, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC2_DAT0, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC2_DAT2, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	
	OMAP3_MUX(I2C2_SDA, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(I2C3_SDA, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	/* SPI3 */

	OMAP3_MUX(ETK_D1, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(ETK_D0, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(ETK_D3, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),	
	OMAP3_MUX(ETK_D7, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),

	OMAP3_MUX(ETK_CLK, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(ETK_D6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(ETK_D9, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	


	/*	DM 3730 - and new setup without changes to u-boot
		DSS LCD VIDEO */
	OMAP3_MUX(DSS_PCLK, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_HSYNC, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_VSYNC, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_ACBIAS, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA0, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA1, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA2, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA3, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA4, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA5, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA6, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA7, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA8, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA9, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA10, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA11, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA12, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA13, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA14, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA15, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA16, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA17, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA18, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA19, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA20, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA21, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA22, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA23, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	
	/* MIS_LED1-3 */
	OMAP3_MUX(GPMC_A8, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_A9, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_A10, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* PCI32 IRQS & SPI */
	OMAP3_MUX(ETK_CLK, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/* GPIO_12 OMAP2PIC1 */
	OMAP3_MUX(ETK_D0, OMAP_MUX_MODE1 | OMAP_PIN_INPUT), 	/* McSPI3_SIMO */
	OMAP3_MUX(ETK_D1, OMAP_MUX_MODE1 | OMAP_PIN_INPUT), 	/* McSPI3_SOMI */
	OMAP3_MUX(ETK_D3, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),		/* McSPI3_CLK */
	OMAP3_MUX(ETK_D4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* GPIO_18 PIC2OMAP3 */
	OMAP3_MUX(ETK_D5, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* GPIO_19 PIC2OMAP2 */
	OMAP3_MUX(ETK_D6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/* GPIO_20 OMAP2PIC2 */
	OMAP3_MUX(ETK_D7, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),	/* McSPI3_CS1 */
	OMAP3_MUX(ETK_D8, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* GPIO_22 PIC2OMAP1 */
	OMAP3_MUX(ETK_D9, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/* GPIO_23 OMAP2PIC3 */
	
	/* UART2 */
	OMAP3_MUX(UART2_CTS, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/* GPIO_144 BUZZER */
	/* SPI TSC2046 PENIRQ */
	OMAP3_MUX(CSI2_DX1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT), 	/* GPIO_114 */
	OMAP3_MUX(MCSPI1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT), 
	OMAP3_MUX(MCSPI1_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_INPUT), 
	OMAP3_MUX(MCSPI1_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT), 
	OMAP3_MUX(MCSPI1_CS1, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT), 
	OMAP3_MUX(MCSPI1_CS2, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT), 
	OMAP3_MUX(MCSPI1_CS3, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT), 
	
	/* USB */
	OMAP3_MUX(GPMC_NCS2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/* GPIO_53 */
	OMAP3_MUX(I2C2_SCL, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/* GPIO_168 */
	OMAP3_MUX(I2C2_SDA, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/* GPIO_183 */
	
	
	/* Display Sub System */
	OMAP3_MUX(ETK_CLK, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SYS_CLKOUT2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	
	/* Serial ports RS232EN ???? */
	OMAP3_MUX(ETK_CTL, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	
	OMAP3_MUX(UART1_TX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART1_RX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART1_CTS, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART1_RTS, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART2_TX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART2_RX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART3_RX_IRRX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART3_TX_IRTX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	/* McSPI 1 */
	OMAP3_MUX(MCSPI1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	/* disable other eth */
	OMAP3_MUX(GPMC_WAIT3, OMAP_MUX_MODE7 ),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define mis0010_mux	NULL
#endif


void __init mis0010_init(struct twl4030_platform_data *pdata)
{

	pr_info("IGEP: mis0010_init()\n");

	mux_partition = omap_mux_get("core");
  
	/* Mux initialitzation for mis0010 */
	omap_mux_write_array(mux_partition, mis0010_mux);

	igep0030_get_revision();

	mis0010_gpio_init();

	platform_device_register(&gpio_led_device);
	platform_device_register(&twl4030_leds_pwm);

	/* Add twl4030 platform data */
	omap3_pmic_get_config(pdata, 0, TWL_COMMON_REGULATOR_VPLL2);
	
///	/* Register I2C3 bus */
///	omap_register_i2c_bus(3, 100, NULL, 0);

	spidevs_dev_init();
	spi_register_board_info(igep3_spi_board_info, ARRAY_SIZE(igep3_spi_board_info));

	/* Display initialitzation */
	mis0010_display_init();

	/* Ethernet with SMSC9221 LAN Controller */
	mis0010_smsc911x_init();
}
