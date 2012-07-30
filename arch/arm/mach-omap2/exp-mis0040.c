/*
 * Copyright (C) 2010-2011 - ISEE 2007 SL
 * Copyright (C) 2010-2011 - MIB Robert Woerle
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 7" TOUCH T07BA Panel 
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

#include <linux/lis3lv02d.h>
#include <linux/input/edt-ft5x06.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/mcspi.h>
#include "twl-common.h"
#include "board-igep00x0.h"
#include "mux.h"

/* MIS */
#define IGEP3_TS_IRQ			10
#define IGEP3_TS_NRESET			186
#define IGEP3_SUMMER			14
#define IGEP3_MIS_LED1			19
#define IGEP3_MIS_LED2			21 
#define IGEP3_UART2_TERM		174
#define IGEP3_UART3_TERM		15
#define IGEP3_UART2_RI			13
#define IGEP3_UART3_RI			12
#define IGEP3_LCD_NSHUTDOWN		128
#define IGEP3_LCD_ENABLE		69
#define IGEP3_LCD_PWRCTRL		78
#define IGEP3_USB_NOC			17

#define MIS_ID_BIT0				170
#define MIS_ID_BIT1				79
#define MIS_ID_BIT2				87
#define MIS_ID_BIT3				70

#define MIS_LOW_VOLTAGE			71
#define MIS_1V8_DISABLE			149
#define MIS_3V3_DISABLE			20

/* SMSC911X Ethernet controller */
#define IGEP3_RA_SMSC911X0_CS       1
#define IGEP3_RA_SMSC911X0_IRQ		56
#define IGEP3_RA_SMSC911X0_NRESET	61
#define IGEP3_RA_SMSC911X1_CS		4
#define IGEP3_RA_SMSC911X1_IRQ		176
#define IGEP3_RA_SMSC911X1_NRESET	57

#define IGEP3_BOARD_HWREV_D	0xD
#define IGEP3_BOARD_HWREV_E	0xE

static u8 hwrev;
static u8 misrev;

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

static inline void __init mis0040_smsc911x_init(void)
{
	/* SMSC911X Ethernet controller */
	omap_mux_init_signal("gpmc_ncs1", 0);
	omap_mux_init_gpio(IGEP3_RA_SMSC911X0_IRQ, OMAP_PIN_INPUT);
	omap_mux_init_gpio(IGEP3_RA_SMSC911X0_NRESET, OMAP_PIN_OUTPUT);

	omap_mux_init_signal("gpmc_ncs4", 0);
	omap_mux_init_gpio(IGEP3_RA_SMSC911X1_IRQ, OMAP_PIN_INPUT);
	omap_mux_init_gpio(IGEP3_RA_SMSC911X1_NRESET, OMAP_PIN_OUTPUT);

	smsc911x0_resources[1].start =
				OMAP_GPIO_IRQ(IGEP3_RA_SMSC911X0_IRQ);
	smsc911x0_resources[1].end =
				OMAP_GPIO_IRQ(IGEP3_RA_SMSC911X0_IRQ);

	smsc911x1_resources[1].start =
				OMAP_GPIO_IRQ(IGEP3_RA_SMSC911X1_IRQ);
	smsc911x1_resources[1].end =
				OMAP_GPIO_IRQ(IGEP3_RA_SMSC911X1_IRQ);

	/* Set up first smsc911x chip */
	igep00x0_smsc911x_init(&smsc911x0_device, IGEP3_RA_SMSC911X0_CS,
		IGEP3_RA_SMSC911X0_IRQ, IGEP3_RA_SMSC911X0_NRESET);

	/* Set up second smsc911x chip */
	igep00x0_smsc911x_init(&smsc911x1_device, IGEP3_RA_SMSC911X1_CS,
		IGEP3_RA_SMSC911X1_IRQ,	IGEP3_RA_SMSC911X1_NRESET);
}
#else
static inline void __init mis0040_smsc911x_init(void) { }
#endif

static int mis0040_enable_dvi(struct omap_dss_device *dssdev)
{
	gpio_direction_output(IGEP3_LCD_ENABLE, 1);

	return 0;
}

static void mis0040_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_direction_output(IGEP3_LCD_ENABLE, 0);
}

struct omap_dss_device mis0040_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd-70",
	.driver_name		= "70wvw1tz3",
	.phy.dpi.data_lines	= 24,
};

static struct omap_dss_device *mis0040_dss_devices[] = {
	&mis0040_dvi_device,
	&igep00x0_dvi_device,
};

static struct omap_dss_board_info mis0040_dss_data = {
	.num_devices	= ARRAY_SIZE(mis0040_dss_devices),
	.devices	= mis0040_dss_devices,
	.default_device	= &mis0040_dvi_device,
};

static struct platform_device mis0040_dss_device = {
	.name	= "omapdss",
	.id	= -1,
	.dev	= {
		.platform_data = &mis0040_dss_data,
	},
};

static inline void mis0040_display_init(void)
{
	if ((gpio_request(IGEP3_LCD_NSHUTDOWN, "NSHUTDOWN") == 0) &&
		(gpio_direction_output(IGEP3_LCD_NSHUTDOWN, 1) == 0)) {
		gpio_export(IGEP3_LCD_NSHUTDOWN, 0);
		gpio_set_value(IGEP3_LCD_NSHUTDOWN, 1);
		}
	else
		pr_warning("IGEP: Could not obtain gpio NSHUTDOWN\n"); 
		
	igep00x0_dvi_device.platform_enable = mis0040_enable_dvi;
	igep00x0_dvi_device.platform_disable = mis0040_disable_dvi;

	platform_device_register(&mis0040_dss_device);
}

static inline void mis0040_gpio_init(void)
{
	if ((gpio_request(IGEP3_SUMMER, "SUMMER") == 0) &&
		(gpio_direction_output(IGEP3_SUMMER, 1) == 0)) {
		gpio_export(IGEP3_SUMMER, 0);
		gpio_set_value(IGEP3_SUMMER, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio GPIO_SUMMER\n"); 

	if ((gpio_request(IGEP3_LCD_ENABLE, "LCD_ENABLE") == 0) &&
		(gpio_direction_output(IGEP3_LCD_ENABLE, 1) == 0)) {
		gpio_export(IGEP3_LCD_ENABLE, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio LCD_ENABLE\n");

	if ((gpio_request(IGEP3_USB_NOC, "USB_NOC") == 0) &&
		(gpio_direction_output(IGEP3_USB_NOC, 0) == 0)) {
		gpio_export(IGEP3_USB_NOC, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio LCD_ENABLE\n");

	if ((gpio_request(IGEP3_UART2_TERM, "UART2_TERM") == 0) &&
		(gpio_direction_output(IGEP3_UART2_TERM, 1) == 0)) {
		gpio_export(IGEP3_UART2_TERM, 0);
		gpio_set_value(IGEP3_UART2_TERM, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio UART2_TERM\n");

	if ((gpio_request(IGEP3_UART2_RI, "UART2_RI") == 0) &&
		(gpio_direction_output(IGEP3_UART2_RI, 0) == 0)) {
		gpio_export(IGEP3_UART2_RI, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio UART2_RI\n");

	if ((gpio_request(IGEP3_UART3_TERM, "UART3_TERM") == 0) &&
		(gpio_direction_output(IGEP3_UART3_TERM, 1) == 0)) {
		gpio_export(IGEP3_UART3_TERM, 0);
		gpio_set_value(IGEP3_UART3_TERM, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio UART3_TERM\n");

	if ((gpio_request(IGEP3_UART3_RI, "UART3_RI") == 0) &&
		(gpio_direction_output(IGEP3_UART3_RI, 0) == 0)) {
		gpio_export(IGEP3_UART3_RI, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio UART3_RI\n");

	if ((gpio_request(IGEP3_LCD_PWRCTRL, "LCD_PWRCtRL") == 0) &&
		(gpio_direction_output(IGEP3_LCD_PWRCTRL, 1) == 0)) {
		gpio_export(IGEP3_LCD_PWRCTRL, 0);
		gpio_set_value(IGEP3_LCD_PWRCTRL, 1);
		}
	else
		pr_warning("IGEP: Could not obtain gpio LCD_PWRCTRL\n");

	if ((gpio_request(MIS_LOW_VOLTAGE, "MIS_LOW_VOLTAGE") == 0) &&
		(gpio_direction_input(MIS_LOW_VOLTAGE) == 0)) {
		gpio_export(MIS_LOW_VOLTAGE, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio MIS_LOW_VOLTAGE\n");
		
	if ((gpio_request(MIS_3V3_DISABLE, "MIS_3V3_DISABLE") == 0) &&
		(gpio_direction_output(MIS_3V3_DISABLE, 1) == 0)) {
		gpio_export(MIS_3V3_DISABLE, 0);
		gpio_set_value(MIS_3V3_DISABLE, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio MIS_3V3_DISABLE\n");
		
	if ((gpio_request(MIS_1V8_DISABLE, "MIS_1V8_DISABLE") == 0) &&
		(gpio_direction_output(MIS_1V8_DISABLE, 1) == 0)) {
		gpio_export(MIS_1V8_DISABLE, 0);
		gpio_set_value(MIS_1V8_DISABLE, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio MIS_1V8_DISABLE\n");

	if ((gpio_request(MIS_ID_BIT0, "MIS_ID_BIT0") == 0) &&
		(gpio_direction_input(MIS_ID_BIT0) == 0)){
		misrev = gpio_get_value(MIS_ID_BIT0);
		gpio_export(MIS_ID_BIT0, 0);

		if (misrev) {
			if ((gpio_request(MIS_ID_BIT1, "MIS_ID_BIT1") == 0) &&
				(gpio_direction_input(MIS_ID_BIT1) == 0)){
				if (gpio_get_value(MIS_ID_BIT1))
					misrev |= (1<<1);
				gpio_export(MIS_ID_BIT1, 0);
				}
			else
				pr_warning("IGEP: Could not obtain gpio MIS_ID_BIT3\n");

			if ((gpio_request(MIS_ID_BIT2, "MIS_ID_BIT2") == 0) &&
				(gpio_direction_input(MIS_ID_BIT2) == 0)){
				if (gpio_get_value(MIS_ID_BIT2))
					misrev |= (1<<2);
				gpio_export(MIS_ID_BIT2, 0);
				}
			else
				pr_warning("IGEP: Could not obtain gpio MIS_ID_BIT3\n");

			if ((gpio_request(MIS_ID_BIT3, "MIS_ID_BIT3") == 0) &&
				(gpio_direction_input(MIS_ID_BIT3) == 0)){
				if (gpio_get_value(MIS_ID_BIT3))
					misrev |= (1<<3);
				gpio_export(MIS_ID_BIT3, 0);
				}
			else
				pr_warning("IGEP: Could not obtain gpio MIS_ID_BIT3\n");
			}
		pr_info("MIS: Rev. %d\n", misrev);
		}
	else
		pr_warning("IGEP: Could not obtain gpio MIS_ID_BIT0\n");
}

static struct gpio_led gpio_led_data[] = {
	[0] = {
		.name = "mis-led1:yellow",
		.gpio = IGEP3_MIS_LED1,
		.default_trigger = "heartbeat",
		.active_low = false,
	},
	[1] = {
		.name = "mis-led2:yellow",
		.gpio = IGEP3_MIS_LED2,
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
		.active_low = 0,
	},{
		.name		= "ledb",
		.pwm_id		= 1,
		.active_low = 0,
	},
	{
		.name		= "pwm0",
		.pwm_id		= 2,
		.active_low = 0,
	},{
		.name		= "pwm1",
		.pwm_id		= 3,
		.active_low = 1,
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

static struct edt_ft5x06_platform_data  edt_ft5x06_pdata = {
       .reset_pin      = IGEP3_TS_NRESET,
};

static struct i2c_board_info __initdata mis0040_i2c3_boardinfo[] = {
	{
		I2C_BOARD_INFO("edt-ft5x06", 0x38),
		.flags          = I2C_CLIENT_WAKE,
		.irq            = OMAP_GPIO_IRQ(IGEP3_TS_IRQ),
		.platform_data  = &edt_ft5x06_pdata,
	},
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux mis0040_mux[] __initdata = {
	
	/* I2C3 Touchscreen */
	OMAP3_MUX(I2C3_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(I2C3_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(SYS_CLKOUT1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* GPIO 10 TS IRQ */
	OMAP3_MUX(SYS_CLKOUT2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/* GPIO 186 TS N_RESET */

	/*	DM 3730 - and new setup without changes to u-boot
		DSS LCD VIDEO */
	OMAP3_MUX(DSS_PCLK, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_HSYNC, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_VSYNC, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_ACBIAS, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA2, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA3, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA4, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA5, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA6, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA7, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA8, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), 	/* GPIO_78 IGEP3_LCD_PWRCTRL*/
	OMAP3_MUX(DSS_DATA10, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA11, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA12, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA13, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA14, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA15, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA18, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA19, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA20, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA21, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA22, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA23, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	/* MIS ID BITS */
	OMAP3_MUX(HDQ_SIO, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* GPIO_170 MIS_ID_BIT0 */
	OMAP3_MUX(DSS_DATA9, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* GPIO_79  MIS_ID_BIT1*/
	OMAP3_MUX(DSS_DATA17, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* GPIO_87  MIS_ID_BIT2 */
	OMAP3_MUX(DSS_DATA0, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* GPIO_70  MIS_ID_BIT3 */

	/* MIS LOW VOLTAGE IRQ */
	OMAP3_MUX(DSS_DATA1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* GPIO_71 LOW VOLTAGE */
	OMAP3_MUX(ETK_D6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),		/* GPIO_20 3V3_DISABLE */
	OMAP3_MUX(UART1_RTS, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),		/* GPIO_114 1V8_DISABLE HACK */
	
	/* MIS_LED1-3 */
	OMAP3_MUX(ETK_D5, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),			/* GPIO_19 MIS_LED1 */
	OMAP3_MUX(ETK_D7, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),			/* GPIO_21 MIS_LED2 */

	/* SUMMER */
	OMAP3_MUX(ETK_D0, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),		/*GPIO_14 SUMMER */

	/* ETH0 */
	OMAP3_MUX(GPMC_NCS5, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* GPIO_56 ETH_IRQ */

	/* ETH1 */
	OMAP3_MUX(MCSPI1_CS2, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* GPIO_176 ETH_IRQ */

	/* USB_NOC */
	OMAP3_MUX(ETK_D3, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),			/* GPIO_17 USB_NOC */

	/* UART 1 */
	OMAP3_MUX(UART1_TX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART1_RX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART1_CTS, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	/* UART 2 */
	OMAP3_MUX(MCSPI1_CS0, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	/* GPIO_174 ART2_TERM */
	OMAP3_MUX(UART2_TX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART2_RX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART2_RTS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART2_CTS, OMAP_MUX_MODE7),						/* GPIO_144 UART2_CTS CROSSED WITH RTS*/
	OMAP3_MUX(ETK_CTL, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* GPIO_13 UART2_RI INPUT */

	/* UART 3 */
	OMAP3_MUX(ETK_D1, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),		/* GPIO_15 UART3_TERM */
	OMAP3_MUX(UART3_RX_IRRX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART3_TX_IRTX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART3_RTS_SD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART3_CTS_RCTX, OMAP_MUX_MODE7),					/* GPIO_163 UART3_CTS CROSSED WITH RTS*/
	OMAP3_MUX(ETK_CLK, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* GPIO_12 UART3_RI INPUT */

	/* UART 4 */
	OMAP3_MUX(GPMC_WAIT2, OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT),	/* ttyO3 TX */
	OMAP3_MUX(GPMC_WAIT3, OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT),	/* ttyO3 RX */

	/* all unneeded disabled */
	OMAP3_MUX(MCSPI1_CS1, OMAP_MUX_MODE7),
	OMAP3_MUX(MCSPI1_CLK, OMAP_MUX_MODE7),
	OMAP3_MUX(MCSPI1_SIMO, OMAP_MUX_MODE7),
	OMAP3_MUX(MCSPI1_SOMI, OMAP_MUX_MODE7),
	
	
	OMAP3_MUX(ETK_D4, OMAP_MUX_MODE7),
	OMAP3_MUX(ETK_D8, OMAP_MUX_MODE7),
	OMAP3_MUX(ETK_D9, OMAP_MUX_MODE7),
	
	OMAP3_MUX(CSI2_DX1, OMAP_MUX_MODE7),
	OMAP3_MUX(MCSPI1_CS3, OMAP_MUX_MODE7),

	OMAP3_MUX(SDMMC2_CLK, OMAP_MUX_MODE7),
	OMAP3_MUX(SDMMC2_CMD, OMAP_MUX_MODE7),
	OMAP3_MUX(SDMMC2_DAT0, OMAP_MUX_MODE7),
	OMAP3_MUX(SDMMC2_DAT2, OMAP_MUX_MODE7),

	OMAP3_MUX(GPMC_A9, OMAP_MUX_MODE7),
	OMAP3_MUX(GPMC_A10, OMAP_MUX_MODE7),
	OMAP3_MUX(GPMC_NCS2, OMAP_MUX_MODE7),
	
	OMAP3_MUX(I2C2_SDA, OMAP_MUX_MODE7),
	OMAP3_MUX(I2C2_SCL, OMAP_MUX_MODE7),

	OMAP3_MUX(DSS_DATA16, OMAP_MUX_MODE7),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define mis0040_mux	NULL
#endif


void __init mis0040_init(struct twl4030_platform_data *pdata)
{

	pr_info("IGEP: mis0040_init()\n");

	mux_partition = omap_mux_get("core");
  
	/* Mux initialitzation for mis0040 */
	omap_mux_write_array(mux_partition, mis0040_mux);

	igep0030_get_revision();

	mis0040_gpio_init();

	platform_device_register(&gpio_led_device);
	platform_device_register(&twl4030_leds_pwm);

	/* Add twl4030 platform data */
	omap3_pmic_get_config(pdata, 0, TWL_COMMON_REGULATOR_VPLL2);

	/* Register I2C3 bus with EDT-FT5x06 touch sensor*/
	omap_register_i2c_bus(3, 200, mis0040_i2c3_boardinfo,
		ARRAY_SIZE(mis0040_i2c3_boardinfo));

	/* Display initialitzation */
	mis0040_display_init();

	/* Ethernet with SMSC9221 LAN Controller */
	mis0040_smsc911x_init();
}
