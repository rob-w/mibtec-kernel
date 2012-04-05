/*
 * Copyright (C) 2010-2011 - ISEE 2007 SL
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * MIS TCAM Modules
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
#include <linux/lis3lv02d.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/mfd/twl4030-codec.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/mcspi.h>
#include "twl-common.h"
#include "board-igep00x0.h"
#include "mux.h"

/* MIS */

#define IGEP3_GPIO_PENIRQ		114

#define IGEP3_MIS_UART3_RX_EN_NOT	163
#define IGEP3_MIS_UART3_TX_EN		164
#define IGEP3_MIS_EXTERN_UI		17

#define IGEP3_MIS_UART2_EN		145
#define IGEP3_MIS_UART1_EN		13


#define IGEP3_MIS_LEDA1_REV_D	168
#define IGEP3_MIS_LEDA1_REV_E	53
#define IGEP3_MIS_LEDA2			20
#define IGEP3_MIS_LEDB1			14
#define IGEP3_MIS_LEDB2			22

#define IGEP3_MIS_BTT1			21
#define IGEP3_MIS_BTT2			23

#define IGEP3_MIS_PWRBTT		18
#define IGEP3_MIS_PWRHLD		19

#define IGEP3_LCD_NRESET		186

/* Display interface */
#define IGEP3_GPIO_DVI_PUP		12

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

static int mis0020_enable_dvi(struct omap_dss_device *dssdev)
{
//	gpio_direction_output(IGEP3_LCD_ENABLE, 1);

	return 0;
}

static void mis0020_disable_dvi(struct omap_dss_device *dssdev)
{
//	gpio_direction_output(IGEP3_LCD_ENABLE, 0);
}

struct omap_dss_device mis0020_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "edt0430",
	.driver_name		= "edt0430_panel",
	.phy.dpi.data_lines	= 24,
};

static struct omap_dss_device *mis0020_dss_devices[] = {
	&mis0020_dvi_device,
	&igep00x0_dvi_device,
};

static struct omap_dss_board_info mis0020_dss_data = {
	.num_devices	= ARRAY_SIZE(mis0020_dss_devices),
	.devices	= mis0020_dss_devices,
	.default_device	= &mis0020_dvi_device,
};

static struct platform_device mis0020_dss_device = {
	.name	= "omapdss",
	.id	= -1,
	.dev	= {
		.platform_data = &mis0020_dss_data,
	},
};

static inline void mis0020_display_init(void)
{

	if ((gpio_request(IGEP3_GPIO_DVI_PUP, "DVI PUP") == 0) &&
	    (gpio_direction_output(IGEP3_GPIO_DVI_PUP, 1) == 0))
		gpio_export(IGEP3_GPIO_DVI_PUP, 0);
	else
		pr_err("IGEP: Could not obtain gpio DVI PUP\n");
		
	if ((gpio_request(IGEP3_LCD_NRESET, "NRESET") == 0) &&
		(gpio_direction_output(IGEP3_LCD_NRESET, 1) == 0)) {
		gpio_export(IGEP3_LCD_NRESET, 0);
		gpio_set_value(IGEP3_LCD_NRESET, 1);
		}
	else
		pr_warning("IGEP: Could not obtain gpio NRESET\n"); 
		
	igep00x0_dvi_device.platform_enable = mis0020_enable_dvi;
	igep00x0_dvi_device.platform_disable = mis0020_disable_dvi;

	platform_device_register(&mis0020_dss_device);
}

static inline void mis0020_gpio_init(void)
{

	if ((gpio_request(IGEP3_MIS_PWRHLD, "PWRHOLD") == 0) &&
		(gpio_direction_output(IGEP3_MIS_PWRHLD, 1) == 0)) {
		gpio_export(IGEP3_MIS_PWRHLD, 0);
		gpio_set_value(IGEP3_MIS_PWRHLD, 1);
		}
	else
		pr_warning("IGEP: Could not obtain gpio PWRHOLD\n"); 

	if ((gpio_request(IGEP3_MIS_UART3_RX_EN_NOT, "UART3_RX_EN_NOT") == 0) &&
		(gpio_direction_output(IGEP3_MIS_UART3_RX_EN_NOT, 1) == 0)) {
		gpio_export(IGEP3_MIS_UART3_RX_EN_NOT, 0);
		gpio_set_value(IGEP3_MIS_UART3_RX_EN_NOT, 0);
		}
	else
		pr_warning("IGEP: Could not obtain gpio UART3_RX_EN_NOT\n"); 
		
	if ((gpio_request(IGEP3_MIS_UART3_TX_EN, "UART3_TX_EN") == 0) &&
		(gpio_direction_output(IGEP3_MIS_UART3_TX_EN, 1) == 0)) {
		gpio_export(IGEP3_MIS_UART3_TX_EN, 0);
		gpio_set_value(IGEP3_MIS_UART3_TX_EN, 1);
		}
	else
		pr_warning("IGEP: Could not obtain gpio UART3_TX_EN\n"); 

	if ((gpio_request(IGEP3_MIS_UART1_EN, "UART1_EN") == 0) &&
		(gpio_direction_output(IGEP3_MIS_UART1_EN, 1) == 0)) {
		gpio_export(IGEP3_MIS_UART1_EN, 0);
		gpio_set_value(IGEP3_MIS_UART1_EN, 1);
		}
	else
		pr_warning("IGEP: Could not obtain gpio UART1_EN\n");
		
	if ((gpio_request(IGEP3_MIS_UART2_EN, "UART2_EN") == 0) &&
		(gpio_direction_output(IGEP3_MIS_UART2_EN, 1) == 0)) {
		gpio_export(IGEP3_MIS_UART2_EN, 0);
		gpio_set_value(IGEP3_MIS_UART2_EN, 1);
		}
	else
		pr_warning("IGEP: Could not obtain gpio UART2_EN\n");

	if (hwrev == IGEP3_BOARD_HWREV_D) {
		if ((gpio_request(IGEP3_MIS_LEDA1_REV_D, "LEDA1") == 0) &&
			(gpio_direction_output(IGEP3_MIS_LEDA1_REV_D, 1) == 0)) {
			gpio_export(IGEP3_MIS_LEDA1_REV_D, 0);
			gpio_set_value(IGEP3_MIS_LEDA1_REV_D, 1);
			}
		else
			pr_warning("IGEP: Could not obtain gpio MIS_LEDA1\n");
		}
	else {
		if ((gpio_request(IGEP3_MIS_LEDA1_REV_E, "LEDA1") == 0) &&
			(gpio_direction_output(IGEP3_MIS_LEDA1_REV_E, 1) == 0)) {
			gpio_export(IGEP3_MIS_LEDA1_REV_E, 0);
			gpio_set_value(IGEP3_MIS_LEDA1_REV_E, 1);
			}
		else
			pr_warning("IGEP: Could not obtain gpio MIS_LEDA1\n");
		}

	if ((gpio_request(IGEP3_MIS_LEDA2, "LEDA2") == 0) &&
		(gpio_direction_output(IGEP3_MIS_LEDA2, 1) == 0)) {
		gpio_export(IGEP3_MIS_LEDA2, 0);
		gpio_set_value(IGEP3_MIS_LEDA2, 1);
		}
	else
		pr_warning("IGEP: Could not obtain gpio MIS_LEDA2\n");

}

struct ads7846_platform_data mis0020_tsc2046_config = {
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
		.platform_data          = &mis0020_tsc2046_config,
		.mode                   = SPI_MODE_2,
	},
};

static struct led_pwm mis0020_pwm_leds[] = {
	{
		.name		= "pwm0",
		.pwm_id		= 2,
//		.max_brightness	= 255,
//		.pwm_period_ns	= 7812500,
	},{
		.name		= "pwm1",
		.pwm_id		= 3,
//		.max_brightness	= 255,
//		.pwm_period_ns	= 7812500,
	},
};

static struct led_pwm_platform_data mis0020_pwm_data = {
	.num_leds	= ARRAY_SIZE(mis0020_pwm_leds),
	.leds		= mis0020_pwm_leds,
};

static struct platform_device mis0020_leds_pwm = {
	.name	= "leds-twl4030-pwm",
	.id	= -1,
	.dev	= {
		.platform_data = &mis0020_pwm_data,
	},
};

static struct gpio_keys_button mis0020_gpio_keys[] = {
	{
		.code	= KEY_RECORD,
		.gpio	= IGEP3_MIS_BTT1,
		.desc	= "btt1",
		.wakeup	= 1,
	},{
		.code	= KEY_DOWN,
		.gpio	= IGEP3_MIS_BTT2,
		.desc	= "btt2",
		.wakeup	= 1,
	},{
		.code	= KEY_UP,
		.gpio	= IGEP3_MIS_PWRBTT,
		.desc	= "btt3",
		.wakeup	= 1,
	},
};

static struct gpio_keys_platform_data mis0020_gpio_keys_pdata = {
	.buttons	= mis0020_gpio_keys,
	.nbuttons	= ARRAY_SIZE(mis0020_gpio_keys),
};

static struct platform_device mis0020_gpio_keys_device = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data = &mis0020_gpio_keys_pdata,
	},
};

static struct lis3lv02d_platform_data lis3d_pdata = {
	.wakeup_thresh	= 10,
};

static struct i2c_board_info __initdata igep3_lis3d_boardinfo[] = {
        {
                I2C_BOARD_INFO("lis3lv02d", 0x18),
                .flags          = I2C_CLIENT_WAKE,
                .irq            = OMAP_GPIO_IRQ(175),
                .platform_data  = &lis3d_pdata,
        },
};

static struct gpio_led gpio_led_data[] = {
	[0] = {
		.name = "led-white",
		.gpio = IGEP3_MIS_LEDB1,
		.default_trigger = "default-on",
		.active_low = false,
	},
	[1] = {
		.name = "led-blue",
		.gpio = IGEP3_MIS_LEDB2,
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

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux mis0020_mux[] __initdata = {

	/* DSS LCD VIDEO */
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
	

	OMAP3_MUX(UART3_CTS_RCTX, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), 	/* GPIO_163 IGEP_MIS_UART3_RX_EN_NOT */
	OMAP3_MUX(UART3_RTS_SD, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),  	/* GPIO_164 IGEP_MIS_UART3_TX_EN */
	OMAP3_MUX(UART2_CTS, OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT),			/* PWM1 GPIO144 */
	OMAP3_MUX(ETK_D3, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),  			/* GPIO_17 IGEP_MIS_EXTERN_UI */
	OMAP3_MUX(UART2_RTS, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),			/* GPIO_145 IGEP3_MIS_UART2_EN */
	OMAP3_MUX(ETK_CTL, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),			/* GPIO_13 IGEP3_MIS_UART1_EN */
	OMAP3_MUX(SYS_CLKOUT2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),		/* GPIO_186 IGEP3_LCD_NRESET */
	OMAP3_MUX(I2C2_SCL, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),			/* GPIO_168 IGEP3_MIS_LEDA1 */
	OMAP3_MUX(ETK_D6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), 			/* GPIO_20 IGEP3_MIS_LEDA2 */
	OMAP3_MUX(ETK_D0, OMAP_MUX_MODE4 | OMAP_PIN_INPUT), 			/* GPIO_14 IGEP3_MIS_LEDB1 */
	OMAP3_MUX(ETK_D4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),				/* GPIO_18 IGEP3_MIS_PWRBTT */
	OMAP3_MUX(ETK_D5, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),			/* GPIO_19 IGEP3_MIS_PWRHOLD */
	OMAP3_MUX(ETK_D7, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),				/* GPIO_21 IGEP3_MIS_BTT1 */
	OMAP3_MUX(ETK_D9, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),				/* GPIO_23 IGEP3_MIS_BTT2 */
	OMAP3_MUX(ETK_D8, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),			/* GPIO_22 IGEP3_MIS_LEDB2 */

	/* SPI TSC2046 PENIRQ */
	OMAP3_MUX(CSI2_DX1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT), 			/* GPIO_114 */
	OMAP3_MUX(MCSPI1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT), 
	OMAP3_MUX(MCSPI1_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_INPUT), 
	OMAP3_MUX(MCSPI1_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT), 
	OMAP3_MUX(MCSPI1_CS1, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT), 
	OMAP3_MUX(MCSPI1_CS2, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT), 
	OMAP3_MUX(MCSPI1_CS3, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT), 
	
	/* Serial ports */
	OMAP3_MUX(UART1_TX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART1_RX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART1_CTS, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART1_RTS, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART2_TX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART2_RX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART3_RX_IRRX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART3_TX_IRTX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	/* McSPI 1 */
	OMAP3_MUX(MCSPI1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define mis0020_mux	NULL
#endif


void __init mis0020_init(struct twl4030_platform_data *pdata)
{

	pr_info("IGEP: mis0020_init()\n");

	mux_partition = omap_mux_get("core");
  
	/* Mux initialitzation for mis0010 */
	omap_mux_write_array(mux_partition, mis0020_mux);

	igep0030_get_revision();

	platform_device_register(&mis0020_leds_pwm);
	platform_device_register(&gpio_led_device);

	mis0020_gpio_init();
	
	/* Add twl4030 platform data */
	omap3_pmic_get_config(pdata, 0, TWL_COMMON_REGULATOR_VPLL2);
	
	/* Register I2C3 bus with LIS3dh sensor*/
	omap_register_i2c_bus(3, 200, igep3_lis3d_boardinfo,
		ARRAY_SIZE(igep3_lis3d_boardinfo));

	/* Add platform gpio key devices */
	platform_device_register(&mis0020_gpio_keys_device);

	spi_register_board_info(igep3_spi_board_info, ARRAY_SIZE(igep3_spi_board_info));

	/* Display initialitzation */
	mis0020_display_init();
}
