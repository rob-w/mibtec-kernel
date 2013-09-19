/*
 * Copyright (C) 2011 - ISEE 2007 SL
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
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>

#include <linux/can/platform/mcp251x.h>

#include <linux/i2c/at24.h>
#include <linux/i2c/twl.h>

#include <linux/mtd/nand.h>

#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/mcspi.h>
#include <plat/onenand.h>
#include <plat/usb.h>

#include "board-flash.h"
#include "board-igep00x0.h"
#include "control.h"
#include "devices.h"
#include "mux.h"
#include "sdram-numonyx-m65kxxxxam.h"

struct omap_mux_partition *mux_partition = NULL;

struct omap_dss_device igep00x0_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.phy.dpi.data_lines	= 24,
};

struct omap_dss_device igep00x0_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
};

/* Powertip 4.3 inch (480 x RGB x 272) TFT with Touch-Panel */
struct omap_dss_device igep00x0_lcd43_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd-43",
	.driver_name		= "ph480272t",
	.phy.dpi.data_lines	= 24,
};

/* Seiko 7.0 inch WVGA (800 x RGB x 480) TFT with Touch-Panel */
struct omap_dss_device igep00x0_lcd70_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd-70",
	.driver_name		= "70wvw1tz3",
	.phy.dpi.data_lines	= 24,
};

struct omap_musb_board_data igep00x0_musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	.mode		= MUSB_OTG,
	.power		= 100,
};

/* TWL4030 power init scripts */
static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset VDD1 regulator.
 * Reset VDD2 regulator.
 * Reset VPLL1 regulator.
 * Enable sysclk output.
 * Reenable twl4030.
 */
        {MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_OFF), 2},
        {MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_WRST), 15},
        {MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_WRST), 15},
        {MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_WRST), 0x60},
        {MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
        {MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wrst_script __initdata = {
        .script = wrst_seq,
        .size   = ARRAY_SIZE(wrst_seq),
        .flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] __initdata = {
        &wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] = {
        { .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3, .type = -1,
                .type2 = -1 },
        { .resource = RES_VDD1, .devgroup = DEV_GRP_P1, .type = -1,
                .type2 = -1 },
        { .resource = RES_VDD2, .devgroup = DEV_GRP_P1, .type = -1,
                .type2 = -1 },
        { 0, 0},
};

static struct twl4030_power_data igep00x0_twl4030_power_data = {
        .scripts        = twl4030_scripts,
        .num            = ARRAY_SIZE(twl4030_scripts),
        .resource_config = twl4030_rconfig,
};

static struct regulator_consumer_supply vmmc1_supply =
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.0");

static struct regulator_consumer_supply vio_supply =
	REGULATOR_SUPPLY("hsusb0", "ehci-omap.0");

static struct regulator_consumer_supply vdd33_supplies[] = {
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.1"),
};

/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
struct regulator_init_data igep00x0_vmmc1_idata = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &vmmc1_supply,
};

struct regulator_init_data igep00x0_vio_idata = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1850000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &vio_supply,
};

static struct regulator_init_data vdd33_data = {
	.constraints		= {
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(vdd33_supplies),
	.consumer_supplies	= vdd33_supplies,
};

static struct fixed_voltage_config vdd33_voltage_config = {
	.supply_name		= "VDD 3V3",
	.microvolts		= 3300000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &vdd33_data,
};

struct platform_device igep00x0_vdd33_device = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev = {
		.platform_data	= &vdd33_voltage_config,
	},
};

void __init igep00x0_pmic_get_config(struct twl4030_platform_data *pmic_data,
				  u32 pdata_flags, u32 regulators_flags)
{
	if (!pmic_data->irq_base)
		pmic_data->irq_base = TWL4030_IRQ_BASE;
	if (!pmic_data->irq_end)
		pmic_data->irq_end = TWL4030_IRQ_END;

	/* Common platform data configurations */

	/* Common regulator configurations */
	if (regulators_flags & TWL_IGEP00X0_REGULATOR_VMMC1 && !pmic_data->vmmc1)
		pmic_data->vmmc1 = &igep00x0_vmmc1_idata;

	if (regulators_flags &  TWL_IGEP00X0_REGULATOR_VIO && !pmic_data->vio)
		pmic_data->vio = &igep00x0_vio_idata;

	/* add generic power scripts */
	if (!pmic_data->power)
		pmic_data->power = &igep00x0_twl4030_power_data;
}

/* Expansion boards */
struct buddy_platform_data igep00x0_buddy_pdata = {
	.model = IGEP00X0_BUDDY_NONE,
	.options = 0,
	.revision = IGEP00X0_BUDDY_HWREV_A,
};

static int __init buddy_early_param(char *str)
{
	char name[IGEP00X0_BUDDY_MAX_STRLEN];

	if (!str)
		return -EINVAL;

	strncpy(name, str, IGEP00X0_BUDDY_MAX_STRLEN);

	if (machine_is_igep0020()) {
		if (!strcmp(name, "igep0022")) {
			igep00x0_buddy_pdata.model = IGEP00X0_BUDDY_IGEP0022;
			pr_info("IGEP: IGEP0020 machine + IGEP0022 (buddy)\n");
		} else if (!strcmp(name, "mis0110")) {
			igep00x0_buddy_pdata.model = IGEP00X0_BUDDY_MIS0110;
			pr_info("IGEP: IGEP0020 machine + MIS0110 (buddy)\n");
		} else if (!strcmp(name, "mis0120")) {
			igep00x0_buddy_pdata.model = IGEP00X0_BUDDY_MIS0120;
			pr_info("IGEP: IGEP0020 machine + MIS0120 (buddy)\n");
		} else
			pr_err("IGEP: Unknown buddy for IGEP0020 machine\n");
	} else if (machine_is_igep0030() || machine_is_igep0032()) {
		if (!strcmp(name, "base0010")) {
			igep00x0_buddy_pdata.model = IGEP00X0_BUDDY_BASE0010;
			pr_info("IGEP: IGEP003x machine + BASE0010 (buddy)\n");
		} else if (!strcmp(name, "ilms0015")) {
			igep00x0_buddy_pdata.model = IGEP00X0_BUDDY_ILMS0015;
			pr_info("IGEP: IGEP003x machine + ILMS0015 (buddy)\n");
		} else if (!strcmp(name, "mis0010")) {
			igep00x0_buddy_pdata.model = IGEP00X0_BUDDY_MIS0010;
			pr_info("IGEP: IGEP003x machine + MIS0010 (buddy)\n");
		} else if (!strcmp(name, "mis0020")) {
			igep00x0_buddy_pdata.model = IGEP00X0_BUDDY_MIS0020;
			pr_info("IGEP: IGEP003x machine + MIS0020 (buddy)\n");
		} else if (!strcmp(name, "mis0030")) {
			igep00x0_buddy_pdata.model = IGEP00X0_BUDDY_MIS0030;
			pr_info("IGEP: IGEP003x machine + MIS0030 (buddy)\n");
		} else if (!strcmp(name, "mis0040")) {
			igep00x0_buddy_pdata.model = IGEP00X0_BUDDY_MIS0040;
			pr_info("IGEP: IGEP003x machine + MIS0040 (buddy)\n");
		} else
			pr_err("IGEP: Unknown buddy for IGEP003x machine\n");
	} else
		pr_err("IGEP: Unknown buddy for IGEP0032 machine\n");

	return 0;
}

static int __init buddy_modem_early_param(char *str)
{
	char opt[IGEP00X0_BUDDY_MAX_STRLEN];

	if (!str)
		return -EINVAL;

	strncpy(opt, str, IGEP00X0_BUDDY_MAX_STRLEN);

	if (!strcmp(opt, "yes")) {
		igep00x0_buddy_pdata.options  |= IGEP00X0_BUDDY_OPT_MODEM;
		pr_info("IGEP: buddy options: modem=yes \n");
	} else {
		pr_info("IGEP: buddy options: Invalid option\n");
		return -EINVAL;
	}

	return 0;
}

static int __init buddy_revision_early_param(char *str)
{
	char rev[IGEP00X0_BUDDY_MAX_STRLEN];

	if (!str)
		return -EINVAL;

	strncpy(rev, str, IGEP00X0_BUDDY_MAX_STRLEN);

	if (!strcmp(rev, "A")) {
		igep00x0_buddy_pdata.revision = IGEP00X0_BUDDY_HWREV_A;
		pr_info("IGEP: buddy: Hardware rev. A\n");
	} else if (!strcmp(rev, "B")) {
		igep00x0_buddy_pdata.revision = IGEP00X0_BUDDY_HWREV_B;
		pr_info("IGEP: buddy: Hardware rev. B\n");
	} else {
		pr_info("IGEP: buddy: Unknown revision\n");
		return -EINVAL;
	}

	return 0;
}

early_param("buddy", buddy_early_param);
early_param("buddy.modem", buddy_modem_early_param);
early_param("buddy.revision", buddy_revision_early_param);

#if defined(CONFIG_MTD_ONENAND_OMAP2) || \
	defined(CONFIG_MTD_ONENAND_OMAP2_MODULE)

#define ONENAND_MAP             0x20000000

/* NAND04GR4E1A ( x2 Flash built-in COMBO POP MEMORY )
 * Since the device is equipped with two DataRAMs, and two-plane NAND
 * Flash memory array, these two component enables simultaneous program
 * of 4KiB. Plane1 has only even blocks such as block0, block2, block4
 * while Plane2 has only odd blocks such as block1, block3, block5.
 * So MTD regards it as 4KiB page size and 256KiB block size 64*(2*2048)
 */

static struct mtd_partition igep00x0_flash_partitions[] = {
	{
		.name           = "X-Loader",
		.offset         = 0,
		.size           = 2 * (64*(2*2048))
	},
	{
		.name           = "Boot",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 20 * (64*(2*2048)),
	},
	{
		.name           = "File System",
		.offset         = MTDPART_OFS_APPEND,
		.size           = MTDPART_SIZ_FULL,
	},
};

static inline u32 get_sysboot_value(void)
{
	return omap_ctrl_readl(OMAP343X_CONTROL_STATUS) & IGEP00X0_SYSBOOT_MASK;
}

void __init igep00x0_flash_init(void)
{
	u32 mux;
	mux = get_sysboot_value();

	if (mux == IGEP00X0_SYSBOOT_NAND) {
		pr_info("IGEP: initializing NAND memory device\n");
		board_nand_init(igep00x0_flash_partitions,
			ARRAY_SIZE(igep00x0_flash_partitions),
			0, NAND_BUSWIDTH_16);
	} else if (mux == IGEP00X0_SYSBOOT_ONENAND) {
		pr_info("IGEP: initializing OneNAND memory device\n");
		board_onenand_init(igep00x0_flash_partitions,
			ARRAY_SIZE(igep00x0_flash_partitions), 0);
	} else
		pr_err("IGEP: Flash: unsupported sysboot sequence found\n");
}

#else
void __init igep00x0_flash_init(void) {}
#endif

#if defined(CONFIG_SMSC911X) || \
	defined(CONFIG_SMSC911X_MODULE)

void __init igep00x0_smsc911x_init(struct platform_device *pdev,
					int cs, int irq_gpio, int nreset)
{
	unsigned long cs_mem_base;

	/* Set smsc911x chip */
	if (gpmc_cs_request(cs, SZ_16M, &cs_mem_base) < 0) {
		pr_err("IGEP: Failed request for GPMC mem for smsc911x-%d\n",
				pdev->id);
		gpmc_cs_free(cs);
		return;
	}

	pdev->resource[0].start = cs_mem_base + 0x0;
	pdev->resource[0].end   = cs_mem_base + 0xff;

	if (gpio_request(irq_gpio, "SMSC911X IRQ")
	    || gpio_direction_input(irq_gpio)) {
		pr_err("IGEP: Could not obtain gpio SMSC911X IRQ\n");
		return;
	}

	if ((gpio_request(nreset, "SMSC911X NRESET") == 0) &&
	    (gpio_direction_output(nreset, 0) == 0)) {
		gpio_export(nreset, 0);
		mdelay(30);
		gpio_set_value(nreset, 1);
	} else {
		pr_err("IGEP: Could not obtain gpio NRESET for smsc911x-%d\n",
				pdev->id);
		return;
	}

	platform_device_register(pdev);
}

#else
void __init igep00x0_smsc911x_init(struct platform_device *pdev,
					int cs, int irq_gpio, int nreset) { }
#endif

#if defined(CONFIG_TOUCHSCREEN_ADS7846) || \
	defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)

static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct ads7846_platform_data tsc2046_pdata = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 10,
	.debounce_tol		= 3,
	.debounce_rep		= 1,
	.gpio_pendown		= -EINVAL,
	.keep_vref_on		= 1,
};

static struct spi_board_info tsc2046_spi_board_info __initdata = {
	/*
	 * TSC2046 operates at a max freqency of 2MHz, so
	 * operate slightly below at 1.5MHz
	 */
	.modalias		= "ads7846",
	.bus_num		= -EINVAL,
	.chip_select		= -EINVAL,
	.max_speed_hz		= 1500000,
	.controller_data	= &tsc2046_mcspi_config,
	.irq			= -EINVAL,
	.platform_data		= &tsc2046_pdata,
};

void __init igep00x0_tsc2046_init(int busnum, int cs, int irq,
			int debounce)
{
	struct spi_board_info *spi = &tsc2046_spi_board_info;

	if (debounce)
		gpio_set_debounce(irq, debounce);

	tsc2046_pdata.gpio_pendown = irq;

	spi->bus_num = busnum;
	spi->chip_select = cs;
	spi->irq = OMAP_GPIO_IRQ(irq);

	spi_register_board_info(&tsc2046_spi_board_info, 1);
}

#else
void __init igep00x0_tsc2046_init(int busnum, int cs, int irq, 
				  int debounce) {}
#endif

#if defined(CONFIG_CAN_MCP251X) || \
	defined(CONFIG_CAN_MCP251X_MODULE)

static struct mcp251x_platform_data mcp251x_pdata = {
	.oscillator_frequency	= 20*1000*1000,
	.irq_flags		= IRQF_TRIGGER_FALLING,
};

static struct spi_board_info mcp251x_spi_board_info  __initdata = {
	.modalias	= "mcp2515",
	.bus_num	= -EINVAL,
	.chip_select	= -EINVAL,
	.max_speed_hz	= 1*1000*1000,
	.irq		= -EINVAL,
	.mode		= SPI_MODE_0,
	.platform_data	= &mcp251x_pdata,
};

void __init igep00x0_mcp251x_init(int bus_num, int cs, int irq, int nreset)
{
	struct spi_board_info *spi = &mcp251x_spi_board_info;

	if ((gpio_request(irq, "MCP251X IRQ") == 0)
		&& (gpio_direction_input(irq) == 0))
		gpio_export(irq, 0);
	else {
		pr_err("IGEP: Could not obtain gpio MCP251X IRQ\n");
		return;
	}

	if (nreset) {
		if ((gpio_request(nreset, "MCP251X NRESET") == 0)
			&& (gpio_direction_output(nreset, 0) == 0)) {
			gpio_export(nreset, 0);
			gpio_set_value(nreset, 1);
		}
		else
			pr_err("IGEP: Could not obtain gpio MCP251X NRESET\n");
	}

	spi->bus_num = bus_num;
	spi->chip_select = cs;
	spi->irq = OMAP_GPIO_IRQ(irq),

	spi_register_board_info(&mcp251x_spi_board_info, 1);
}

#else
void __init igep00x0_mcp251x_init(int bus_num, int cs, int irq, int nreset) {}
#endif

#if defined(CONFIG_LIBERTAS_SDIO) || \
	defined(CONFIG_LIBERTAS_SDIO_MODULE)
void __init igep00x0_wifi_bt_init(int npd, int wifi_nreset, int bt_nreset,
		int bt_enable)
{
	omap_mux_init_gpio(npd, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(wifi_nreset, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(bt_nreset, OMAP_PIN_OUTPUT);

	/* Set GPIO's for  W-LAN + Bluetooth combo module */
	if ((gpio_request(npd, "WIFI NPD") == 0) &&
	    (gpio_direction_output(npd, 1) == 0))
		gpio_export(npd, 0);
	else
		pr_warning("IGEP: Could not obtain gpio WIFI NPD\n");

	if ((gpio_request(wifi_nreset, "WIFI NRESET") == 0) &&
	    (gpio_direction_output(wifi_nreset, 1) == 0)) {
		gpio_export(wifi_nreset, 0);
		gpio_set_value(wifi_nreset, 0);
		udelay(10);
		gpio_set_value(wifi_nreset, 1);
	} else
		pr_warning("IGEP: Could not obtain gpio WIFI NRESET\n");

	if ((gpio_request(bt_nreset, "BT NRESET") == 0) &&
	    (gpio_direction_output(bt_nreset, bt_enable) == 0))
		gpio_export(bt_nreset, 0);
	else
		pr_warning("IGEP: Could not obtain gpio BT NRESET\n");
}

#else
void __init igep00x0_wifi_bt_init(int npd, int wifi_nreset, int bt_nreset,
			int bt_enable) { }
#endif

#if defined(CONFIG_EEPROM_AT24) || \
	defined(CONFIG_EEPROM_AT24_MODULE)

static struct at24_platform_data at24c01_pdata __initdata = {
	.byte_len	= SZ_1K / 8,
	.page_size	= 8,
};

static struct i2c_board_info at24c01_i2c_board_info __initdata = {
	I2C_BOARD_INFO("24c01", (0xa0 >> 1)),
	.platform_data = &at24c01_pdata,
};

void __init igep00x0_at24c01_init(int busnum)
{
	i2c_register_board_info(busnum, &at24c01_i2c_board_info, 1);
}

#else
void __init igep00x0_at24c01_init(int busnum) {}
#endif

/* Use MCBSP3_* pins as UART2 mode */
static struct omap_board_mux uart2_mcbsp3_mode1_mux[] = {
	OMAP3_MUX(UART2_TX, OMAP_MUX_MODE7 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART2_RX, OMAP_MUX_MODE7 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP3_FSX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

/* Use UART2_* pins as UART2 mode */
static struct omap_board_mux uart2_mcbsp3_mode7_mux[] = {
	OMAP3_MUX(UART2_TX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART2_RX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP3_FSX, OMAP_MUX_MODE7 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE7 | OMAP_PIN_INPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

void __init igep00x0_modem_init(int on, int nreset, int pwrmon)
{
	/*
	 * We need to set the TWL4030 PCM voice interface output pins to high
	 * impedance, otherwise this module forces the level of mcbsp3_fsx and
	 * mcbsp3_clkx pins.
	 *
	 * PCM/codec/TDM/Bluetooth interface modes cannot be changed on the fly,
	 * so we need to set the value 0x04 to TWL4030_REG_VOICE_IF at file
	 * sound/soc/codecs/twl4030.c
	 * -       0x00,  REG_VOICE_IF  (0xF)
	 * +       0x04,  REG_VOICE_IF  (0xF)
	 *
	 * NOTE: PCM voice interface (PCM VSP) is INCOMPATIBLE with modem.
	 */

	/*
	 * OMAP3530 CBB package can have UART2 muxed on 2 pins: uart2_* and
	 * mcbsp3_*.
	 *
	 * On IGEP0020 + IGEP0022 the modem uses UART2 on mcbsp3_* pins so we
	 * need to configure the MUX properly.
	 */
	mux_partition = omap_mux_get("core");
	if (machine_is_igep0020())
		omap_mux_write_array(mux_partition, uart2_mcbsp3_mode1_mux);
	else
		/*
		 * On IGEP003x + BASE0010 the modem uses UART2 on uart2_* pins
		 * so we need to configure the MUX properly.
		 */
		omap_mux_write_array(mux_partition, uart2_mcbsp3_mode7_mux);

	/* Modem I/O */
	omap_mux_init_gpio(on, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(nreset, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(pwrmon, OMAP_PIN_INPUT);

	if ((gpio_request(on, "MODEM ON NOFF") == 0)
	    && (gpio_direction_output(on, 0) == 0))
		gpio_export(on, 0);
	else
		pr_warning("IGEP: Could not obtain gpio MODEM ON NOFF\n");

	if ((gpio_request(nreset, "MODEM NRESET") == 0) &&
	    (gpio_direction_output(nreset, 0) == 0))
		gpio_export(nreset, 0);
	else
		pr_warning("IGEP: Could not obtain gpio MODEM NRESET\n");

	if ((gpio_request(pwrmon, "MODEM PWRMON") == 0) &&
	    (gpio_direction_input(pwrmon) == 0))
		gpio_export(pwrmon, 0);
	else
		pr_warning("IGEP: Could not obtain gpio MODEM PWRMON\n");
}

#if defined(CONFIG_VIDEO_OMAP3) ||		\
	defined(CONFIG_VIDEO_OMAP3_MODULE)
void __init igep00x0_camera_init(struct isp_platform_data *isp_pdata,
				 int reset_pin, int pdn_pin)
{
	/*
	 * From TVP5151 datasheet Table 3-8. Reset and Power-Down Modes
	 *   PDN RESETB CONFIGURATION
	 *    0    0    Reserved (unknown state)
	 *    0    1    Powers down the decoder
	 *    1    0    Resets the decoder
	 *    1    1    Normal operation
	 *
	 * If TVP5151_PDN and TPVP5151_RESET is set to 0 the I2C2_SDA line
	 * is forced to low level and all devices connected to I2C2 stop
	 * working, this affects to EEPROM connected to the same bus. By default
	 * we should configure these pins to logical 1 (Normal operation)
	 *
	 */
	if ((gpio_request(pdn_pin, "TVP5151 PDN") == 0) &&
		(gpio_direction_output(pdn_pin, 0) == 0))
		gpio_export(pdn_pin, 0);
	else
		pr_warning("IGEP: Could not obtain gpio TVP5151 PDN\n");

	if ((gpio_request(reset_pin, "TVP5151 RESET") == 0)
		&& (gpio_direction_output(reset_pin, 0) == 0)) {
		gpio_export(reset_pin, 0);
		/* Initialize TVP5151 power up sequence */
		udelay(10);
		gpio_set_value(pdn_pin, 1);
		udelay(10);
		gpio_set_value(reset_pin, 1);
		udelay(200);
	} else
		pr_warning("IGEP: Could not obtain gpio TVP5151 RESET\n");

	if (omap3_init_camera(isp_pdata) < 0)
		pr_warning("IGEP: Unable to register camera platform\n");
}
#else
void __init igep00x0_camera_init(void) {}
#endif

void __init igep00x0_init_irq(void)
{ 
	omap2_init_common_infrastructure();
	omap2_init_common_devices (m65kxxxxam_sdrc_params, m65kxxxxam_sdrc_params);
	omap_init_irq();
	gpmc_init();
}


