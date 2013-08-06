/*
 * arch/arm/mach-tegra/board-tostab12AL.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/panjit_ts.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>
#include <linux/regulator/consumer.h>
#include <linux/rfkill-gpio.h>
#include <linux/rfkill.h>

#include <mach/tegra_aic325x_pdata.h>
#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/io_dpd.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <sound/fm34.h>
#include <mach/thermal.h>
#include <mach/pci.h>
#include <mach/tegra_fiq_debugger.h>

#include "board.h"
#include "clock.h"
#include "board-tostab12AL.h"
#include "board-touch.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "baseband-xmm-power.h"
#include "wdt-recovery.h"


static struct balanced_throttle throttle_list[] = {
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_TJ,
		.throt_tab_size = 10,
		.throt_tab = {
			{      0, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 760000, 1000 },
			{ 760000, 1050 },
			{1000000, 1050 },
			{1000000, 1100 },
		},
	},
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_SKIN,
		.throt_tab_size = 6,
		.throt_tab = {
			{ 640000, 1200 },
			{ 640000, 1200 },
			{ 760000, 1200 },
			{ 760000, 1200 },
			{1000000, 1200 },
			{1000000, 1200 },
		},
	},
#endif
};

/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.shutdown_device_id = THERMAL_DEVICE_ID_NCT_EXT,
	.temp_shutdown = 90000,

#if defined(CONFIG_TEGRA_EDP_LIMITS) || defined(CONFIG_TEGRA_THERMAL_THROTTLE)
	.throttle_edp_device_id = THERMAL_DEVICE_ID_NCT_EXT,
#endif
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	.temp_throttle = 85000,
	.tc1 = 0,
	.tc2 = 1,
	.passive_delay = 2000,
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	.skin_device_id = THERMAL_DEVICE_ID_SKIN,
	.temp_throttle_skin = 43000,
	.tc1_skin = 0,
	.tc2_skin = 1,
	.passive_delay_skin = 5000,

	.skin_temp_offset = 9793,
	.skin_period = 1100,
	.skin_devs_size = 2,
	.skin_devs = {
		{
			THERMAL_DEVICE_ID_NCT_EXT,
			{
				2, 1, 1, 1,
				1, 1, 1, 1,
				1, 1, 1, 0,
				1, 1, 0, 0,
				0, 0, -1, -7
			}
		},
		{
			THERMAL_DEVICE_ID_NCT_INT,
			{
				-11, -7, -5, -3,
				-3, -2, -1, 0,
				0, 0, 1, 1,
				1, 2, 2, 3,
				4, 6, 11, 18
			}
		},
	},
#endif
};

static struct rfkill_gpio_platform_data tostab12AL_bt_rfkill_pdata[] = {
	{
		.name           = "bt_rfkill",
		.shutdown_gpio  = BT_SHUTDOWN_GPIO,
		.reset_gpio     = BT_RST_GPIO,
		.type           = RFKILL_TYPE_BLUETOOTH,
	},
};

static struct platform_device tostab12AL_bt_rfkill_device = {
	.name = "rfkill_gpio",
	.id             = -1,
	.dev = {
		.platform_data = &tostab12AL_bt_rfkill_pdata,
	},
};

static struct resource tostab12AL_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
        .start  = BT_IRQ_GPIO,
        .end    = BT_IRQ_GPIO,
        .flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
        .start  = BT_WAKEUP_GPIO,
        .end    = BT_WAKEUP_GPIO,
        .flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(BT_IRQ_GPIO),
			.end    = TEGRA_GPIO_TO_IRQ(BT_IRQ_GPIO),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device tostab12AL_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(tostab12AL_bluesleep_resources),
	.resource       = tostab12AL_bluesleep_resources,
};

static noinline void __init tostab12AL_setup_bluesleep(void)
{
    platform_device_register(&tostab12AL_bluesleep_device);
	tegra_gpio_enable(BT_IRQ_GPIO);
	tegra_gpio_enable(BT_WAKEUP_GPIO);
	return;
}

static __initdata struct tegra_clk_init_table tostab12AL_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	3187500,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s0",	"pll_a_out0",	0,		false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ NULL,		NULL,		0,		0},
};

#ifdef CONFIG_SND_SOC_FM34
const static u16 fm34_property[][2] = {
	//2303,  0x0000   noise suppression , disable
	//2304,  0x0000    noise suppression , disable

	{0x3FA0, 0x82F3},
	{0x3FB0, 0x02F5},
	{0x3FA1, 0x82B5},
	{0x3FB1, 0x5000},
	{0x3FA2, 0x83F4},
	{0x3FB2, 0x500B},
	{0x3FA3, 0x9666},
	{0x3FB3, 0x501C},
	{0x3FA4, 0x82CC},
	{0x3FB4, 0x501F},
	{0x3FA5, 0xC2CC},
	{0x3FB5, 0x501F},
	{0x3FA6, 0xC210},
	{0x3FB6, 0x5023},
	{0x3FA7, 0x8210},
	{0x3FB7, 0x5023},
	{0x3FA8, 0x8231},
	{0x3FB8, 0x5024},
	{0x3FA9, 0xC231},
	{0x3FB9, 0x5024},
	{0x3FAA, 0x8205},
	{0x3FBA, 0x502F},
	{0x3FAB, 0xC205},
	{0x3FBB, 0x502F},
	{0x3FAC, 0x823F},
	{0x3FBC, 0x5048},
	{0x3FAD, 0xC23F},
	{0x3FBD, 0x5048},
	{0x3FAE, 0x83E8},
	{0x3FBE, 0x505D},
	{0x3FAF, 0x9758},
	{0x3FBF, 0x5066},
	{0x22F8, 0x8005},
	{0x22C8, 0x0026},
	{0x22F9, 0x087F},
	{0x2305, 0x0000},
	{0x2301, 0x0002},
	{0x2307, 0x0000},
	{0x2309, 0x0800},
	{0x230D, 0x0100},
	{0x230C, 0x0200}, //gain 0x100: 0db, 0x200: 6db, 0x400: 12db
	{0x22F2, 0x0044},
	{0x22F6, 0x0002},
	{0x22D2, 0x8A94},
	{0x2303, 0x0001},
	{0x2300, 0x0000},
	{0x2304, 0x2310}, //echo cancellation,  off = 0x0010
	{0x232F, 0x0200},
	{0x2339, 0x0008},
	{0x23D0, 0x1000},
	{0x236E, 0x2500},
	{0x2370, 0x4000},
	{0x3FD2, 0x0032},
	{0x2390, 0x7842},
	{0x2391, 0x4000},
	{0x2392, 0x4000},
	{0x2393, 0x4000},
	{0x2394, 0x4000},
	{0x2395, 0x4000},
	{0x2333, 0x000C},
	{0x23B4, 0x0004},
	{0x23B3, 0x0010},
	{0x23CF, 0x1000},
	{0x23D5, 0x6000},
	{0x22C6, 0x000C},
	{0x22C7, 0x000C},
	{0x22FA, 0xA481},
	{0x2348, 0x4000}, //+18db
	{0x2349, 0x4000},
	{0x22EE, 0x0000},
	{0x2332, 0x00A0},
	{0x233C, 0x0400},
	{0x2372, 0x1A00},
	{0x238C, 0x0400},
	{0x22FB, 0x0000} /* must be last one */
};
const static u16 fm34_property2[][2] = {
	{0x3FA0, 0x82F3},
	{0x3FB0, 0x02F5},
	{0x3FA1, 0x82B5},
	{0x3FB1, 0x5000},
	{0x3FA2, 0x83F4},
	{0x3FB2, 0x500B},
	{0x3FA3, 0x9666},
	{0x3FB3, 0x501C},
	{0x3FA4, 0x82CC},
	{0x3FB4, 0x501F},
	{0x3FA5, 0xC2CC},
	{0x3FB5, 0x501F},
	{0x3FA6, 0xC210},
	{0x3FB6, 0x5023},
	{0x3FA7, 0x8210},
	{0x3FB7, 0x5023},
	{0x3FA8, 0x8231},
	{0x3FB8, 0x5024},
	{0x3FA9, 0xC231},
	{0x3FB9, 0x5024},
	{0x3FAA, 0x8205},
	{0x3FBA, 0x502F},
	{0x3FAB, 0xC205},
	{0x3FBB, 0x502F},
	{0x3FAC, 0x823F},
	{0x3FBC, 0x5048},
	{0x3FAD, 0xC23F},
	{0x3FBD, 0x5048},
	{0x3FAE, 0x83E8},
	{0x3FBE, 0x505D},
	{0x3FAF, 0x9758},
	{0x3FBF, 0x5066},
	{0x22F8, 0x8005},
	{0x22C8, 0x0026},
	{0x22F9, 0x087F},
	{0x2305, 0x0000},
	{0x2301, 0x0002},
	{0x2307, 0x0000},
	{0x2309, 0x0800},
	{0x230D, 0x0100},
	{0x230C, 0x0100}, //gain 0x100: 0db, 0x200: 6db, 0x400: 12db
	{0x22F2, 0x0044},
	{0x22F6, 0x0002},
	{0x22D2, 0x8A94},
	{0x2303, 0x0001},
	{0x2300, 0x0000},
	{0x2304, 0x0010}, //echo off
	{0x232F, 0x0200},
	{0x2339, 0x0008},
	{0x23D0, 0x1000},
	{0x236E, 0x2500},
	{0x2370, 0x4000},
	{0x3FD2, 0x0032},
	{0x2390, 0x7842},
	{0x2391, 0x4000},
	{0x2392, 0x4000},
	{0x2393, 0x4000},
	{0x2394, 0x4000},
	{0x2395, 0x4000},
	{0x2333, 0x000C},
	{0x23B4, 0x0004},
	{0x23B3, 0x0010},
	{0x23CF, 0x1000},
	{0x23D5, 0x6000},
	{0x22C6, 0x000C},
	{0x22C7, 0x000C},
	{0x22FA, 0xA481},
	{0x2348, 0x4000}, //+18db ??
	{0x2349, 0x4000},
	{0x22EE, 0x0000},
	{0x2332, 0x00A0},
	{0x233C, 0x0400},
	{0x2372, 0x1A00},
	{0x238C, 0x0400},
	{0x22FB, 0x0000} /* must be last one */

};

const static u16 fm34_property3[][2] = {
	{0x3FA0, 0x82F3},
	{0x3FB0, 0x02F5},
	{0x3FA1, 0x82B5},
	{0x3FB1, 0x5000},
	{0x3FA2, 0x83F4},
	{0x3FB2, 0x500B},
	{0x3FA3, 0x9666},
	{0x3FB3, 0x501C},
	{0x3FA4, 0x82CC},
	{0x3FB4, 0x501F},
	{0x3FA5, 0xC2CC},
	{0x3FB5, 0x501F},
	{0x3FA6, 0xC210},
	{0x3FB6, 0x5023},
	{0x3FA7, 0x8210},
	{0x3FB7, 0x5023},
	{0x3FA8, 0x8231},
	{0x3FB8, 0x5024},
	{0x3FA9, 0xC231},
	{0x3FB9, 0x5024},
	{0x3FAA, 0x8205},
	{0x3FBA, 0x502F},
	{0x3FAB, 0xC205},
	{0x3FBB, 0x502F},
	{0x3FAC, 0x823F},
	{0x3FBC, 0x5048},
	{0x3FAD, 0xC23F},
	{0x3FBD, 0x5048},
	{0x3FAE, 0x83E8},
	{0x3FBE, 0x505D},
	{0x3FAF, 0x9758},
	{0x3FBF, 0x5066},
	{0x22F8, 0x8005},
	{0x22C8, 0x0026},
	{0x22F9, 0x087F},
	{0x2305, 0x0000},
	{0x2301, 0x0002},
	{0x2307, 0x0000},
	{0x2309, 0x0800},
	{0x230D, 0x0100},
	{0x230C, 0x0400}, //gain 0x100: 0db, 0x200: 6db, 0x400: 12db
	{0x22F2, 0x0044},
	{0x22F6, 0x0002},
	{0x22D2, 0x8A94},
	{0x2303, 0x0001},
	{0x2300, 0x0000},
	{0x2304, 0x0010}, //echo off
	{0x232F, 0x0200},
	{0x2339, 0x0008},
	{0x23D0, 0x1000},
	{0x236E, 0x2500},
	{0x2370, 0x4000},
	{0x3FD2, 0x0032},
	{0x2390, 0x7842},
	{0x2391, 0x4000},
	{0x2392, 0x4000},
	{0x2393, 0x4000},
	{0x2394, 0x4000},
	{0x2395, 0x4000},
	{0x2333, 0x000C},
	{0x23B4, 0x0004},
	{0x23B3, 0x0010},
	{0x23CF, 0x1000},
	{0x23D5, 0x6000},
	{0x22C6, 0x000C},
	{0x22C7, 0x000C},
	{0x22FA, 0xA481},
	{0x2348, 0x4000}, //+18db ??
	{0x2349, 0x4000},
	{0x22EE, 0x0000},
	{0x2332, 0x00A0},
	{0x233C, 0x0400},
	{0x2372, 0x1A00},
	{0x238C, 0x0400},
	{0x22FB, 0x0000} /* must be last one */

};

static struct fm34_conf fm34_conf = {
	.pwdn = VD_PWD_GPIO,
	.rst = VD_RST_GPIO,
	.bp = VD_BP_GPIO,
	.cprop = sizeof(fm34_property)/sizeof(u16)/2,
	.cprop2 = sizeof(fm34_property2)/sizeof(u16)/2,
	.cprop3 = sizeof(fm34_property3)/sizeof(u16)/2,
	.pprop = (u16 *)fm34_property,
	.pprop2 = (u16 *)fm34_property2,
	.pprop3 = (u16 *)fm34_property3,
};

static const struct i2c_board_info tostab12AL_fm34_board_info[] = {
	{
		I2C_BOARD_INFO("fm34_i2c", 0x60),
		.platform_data = &fm34_conf,
	},
};

static int __init tostab12AL_fm34_init(void)
{
	tegra_gpio_enable(fm34_conf.pwdn);
	if(fm34_conf.bp != -1)
		tegra_gpio_enable(fm34_conf.bp);
	if(fm34_conf.rst != -1)
		tegra_gpio_enable(fm34_conf.rst);

	i2c_register_board_info(0, tostab12AL_fm34_board_info, 1);

	return 0;
}
#endif

/* GEN1_I2C */
static struct tegra_i2c_platform_data tostab12AL_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

/* GEN2_I2C */
static struct tegra_i2c_platform_data tostab12AL_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

/* CAM_I2C */
static struct tegra_i2c_platform_data tostab12AL_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

/* DDC_I2C */
static struct tegra_i2c_platform_data tostab12AL_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

/* PWR_I2C */
static struct tegra_i2c_platform_data tostab12AL_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

static void tostab12AL_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &tostab12AL_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &tostab12AL_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &tostab12AL_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &tostab12AL_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &tostab12AL_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);
}

static struct platform_device *tostab12AL_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data tostab12AL_uart_pdata;
static struct tegra_uart_platform_data tostab12AL_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = get_tegra_uart_debug_port_id();
	if (debug_port_id < 0) {
		debug_port_id = 0;
	}

    tegra_init_debug_uart_rate();
	switch (debug_port_id) {
	case 0:
		/* UARTA is the debug port. */
		pr_info("Selecting UARTA as the debug console\n");
		tostab12AL_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;

	case 1:
		/* UARTB is the debug port. */
		pr_info("Selecting UARTB as the debug console\n");
		tostab12AL_uart_devices[1] = &debug_uartb_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartb");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->mapbase;
		break;

	case 2:
		/* UARTC is the debug port. */
		pr_info("Selecting UARTC as the debug console\n");
		tostab12AL_uart_devices[2] = &debug_uartc_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartc");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartc_device.dev.platform_data))->mapbase;
		break;

	case 3:
		/* UARTD is the debug port. */
		pr_info("Selecting UARTD as the debug console\n");
		tostab12AL_uart_devices[3] = &debug_uartd_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartd");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
		break;

	case 4:
		/* UARTE is the debug port. */
		pr_info("Selecting UARTE as the debug console\n");
		tostab12AL_uart_devices[4] = &debug_uarte_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uarte");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarte_device.dev.platform_data))->mapbase;
		break;

	default:
		pr_info("The debug console id %d is invalid, Assuming UARTA", debug_port_id);
		tostab12AL_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;
	}
	return;
}

int uart_debug_switch = 0;     /* disable debug uart, turn on audio out through phone jack */

int uart_debug_switch_set(const char *val, struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);
	if(ret) return ret;

	printk(KERN_INFO "uart_debug_switch = %d \n", uart_debug_switch );
	if ( board_rev() <= 2 /* PVT */ )
		gpio_direction_output(DEBUG_MODE_SEL_GPIO, uart_debug_switch ? 0: 1);
	else
		gpio_direction_output(DEBUG_MODE_SEL_GPIO, uart_debug_switch ? 1: 0);

	return 0;
}
module_param_call(uart_debug_switch, uart_debug_switch_set, param_get_int, &uart_debug_switch, 0644);

static int __init uart_debug_disable(void)
{
       int ret = 0;

       ret = gpio_request(TEGRA_GPIO_PJ7, "uart_tx");
       if (ret < 0)
               return ret;
       ret = gpio_direction_output(TEGRA_GPIO_PJ7, 0);
       if (ret < 0)
               gpio_free(TEGRA_GPIO_PJ7);
       else
               tegra_gpio_enable(TEGRA_GPIO_PJ7);

       return ret;
}

static void __init tostab12AL_uart_init(void)
{
	struct clk *c;
	int i;

	tegra_gpio_enable(DEBUG_MODE_SEL_GPIO);
	gpio_request(DEBUG_MODE_SEL_GPIO, "uart_debug_audio_switch");
	if ( board_rev() <= 2 /* PVT */ )
		gpio_direction_output(DEBUG_MODE_SEL_GPIO, uart_debug_switch ? 0: 1);
	else
		gpio_direction_output(DEBUG_MODE_SEL_GPIO, uart_debug_switch ? 1: 0);

	if ( uart_debug_switch == 0 ) {
		uart_debug_disable();
	}
	
	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	tostab12AL_uart_pdata.parent_clk_list = uart_parent_clk;
	tostab12AL_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tostab12AL_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	tostab12AL_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	tostab12AL_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &tostab12AL_uart_pdata;
	tegra_uartb_device.dev.platform_data = &tostab12AL_uart_pdata;
	tegra_uartc_device.dev.platform_data = &tostab12AL_uart_pdata;
	tegra_uartd_device.dev.platform_data = &tostab12AL_uart_pdata;
	/* UARTE is used for loopback test purpose */
	tegra_uarte_device.dev.platform_data = &tostab12AL_loopback_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, clk_get_rate(c));
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	platform_add_devices(tostab12AL_uart_devices,
				ARRAY_SIZE(tostab12AL_uart_devices));
}

static struct tegra_aic325x_platform_data tostab12AL_audio_pdata = {
    .gpio_rst		     = CODEC_RST_GPIO,
    .gpio_spkr_en           = -1,
    .gpio_hp_det            = HP_DET_GPIO,
    .gpio_hp_mute           = -1,
    .gpio_int_mic_en        = -1,
    .gpio_ext_mic_en        = EN_MIC_EXT_GPIO,
    .gpio_ext_mic_det       = AMIC_DET_GPIO,
	.i2s_param[HIFI_CODEC]	= {
		.audio_port_id	= 0,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
	},
	.i2s_param[BASEBAND]	= {
		.audio_port_id	= -1,
	},
	.i2s_param[BT_SCO]	= {
		.audio_port_id	= 3,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_DSP_A,
	},
};

static struct platform_device tostab12AL_audio_aic325x_device = {
	.name	= "tegra-snd-aic325x",
	.id	= 0,
	.dev	= {
		.platform_data  = &tostab12AL_audio_pdata,
	},
};


static struct i2c_board_info __initdata aic325x_board_info = {

		I2C_BOARD_INFO("tlv320aic325x", 0x18),
		.platform_data=&tostab12AL_audio_pdata,
		.irq = 0, //TEGRA_GPIO_TO_IRQ(CDC_IRQ_GPIO),
};

static int __init tostab12AL_audio_init(void)
{
	i2c_register_board_info(0,	&aic325x_board_info, 1);

	return 0;
}

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

#ifdef CONFIG_TEGRA_INFO_DEV
static struct platform_device tegra_info_device = {
    .name = "tegra_info",
    .id = -1,
};
#endif

static struct platform_device *tostab12AL_spi_devices[] __initdata = {
    &tegra_spi_device1,
};

struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data tostab12AL_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (128),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};

static void __init tostab12AL_spi_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	tostab12AL_spi_pdata.parent_clk_list = spi_parent_clk;
	tostab12AL_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device1.dev.platform_data = &tostab12AL_spi_pdata;
	platform_add_devices(tostab12AL_spi_devices,
				ARRAY_SIZE(tostab12AL_spi_devices));

}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};



static struct platform_device *tostab12AL_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU) ||  defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt0_device,
	&tegra_wdt1_device,
	&tegra_wdt2_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device0,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&tostab12AL_bt_rfkill_device,
	&tegra_pcm_device,
	&tostab12AL_audio_aic325x_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
#ifdef CONFIG_TEGRA_INFO_DEV
    &tegra_info_device,
#endif
};

static __initdata struct tegra_clk_init_table spi_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "sbc1",       "pll_p",        52000000,       true},
	{ NULL,         NULL,           0,              0},
};

static __initdata struct tegra_clk_init_table touch_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "extern3",    "pll_p",        41000000,       true},
	{ "clk_out_3",  "extern3",      40800000,       true},
	{ NULL,         NULL,           0,              0},
};

struct mutex touch_onoff_mutex;

static int tostab12AL_touch_poweron(void)
{
	mutex_lock(&touch_onoff_mutex);
	gpio_set_value(TS_EN_GPIO, 1); //ts_en	
	clk_enable(tegra_get_clock_by_name("clk_out_3"));
	mdelay(5); //minimum 1 ms

	enable_irq(TEGRA_GPIO_TO_IRQ(TOUCH_INT_GPIO));
	gpio_set_value(TOUCH_RST_GPIO, 1); //reset gpio
	//mdelay(5);
	mutex_unlock(&touch_onoff_mutex);

    return 0;
}
 int  tostab12AL_touch_poweroff(void)
{
	mutex_lock(&touch_onoff_mutex);
	disable_irq(TEGRA_GPIO_TO_IRQ(TOUCH_INT_GPIO));
	gpio_set_value(TS_EN_GPIO, 0); //ts_en
	mdelay(10);
	gpio_set_value(TOUCH_RST_GPIO, 0);  //reset gpio
	clk_disable(tegra_get_clock_by_name("clk_out_3"));
	
	mutex_unlock(&touch_onoff_mutex);

    return 0;
}

static int __init tostab12AL_touch_init(void)
{
	mutex_init(&touch_onoff_mutex);
    tegra_gpio_enable(TS_EN_GPIO);
    gpio_request(TS_EN_GPIO, "ts_en");
    gpio_direction_output(TS_EN_GPIO, 0);
    gpio_set_value(TS_EN_GPIO, 1);
	mdelay(5); //minimum 1 ms

    pr_info("Raydium On-Board touch init \n");
    tegra_clk_init_from_table(spi_clk_init_table);
    tegra_clk_init_from_table(touch_clk_init_table); /////
    clk_enable(tegra_get_clock_by_name("clk_out_3")); ////

	touch_set_callback(tostab12AL_touch_poweron, tostab12AL_touch_poweroff);
    touch_init_raydium(TOUCH_INT_GPIO, TOUCH_RST_GPIO, 1);
	//touch_init_raydium(TEGRA_GPIO_PQ4, TEGRA_GPIO_PQ6, 1);

	return 0;
}

#if defined(CONFIG_USB_SUPPORT)
static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode        = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		/* .vbus_reg = "vdd_vbus_typea_usb", */
		.hot_plug = true,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = USB_OTG_EN_GPIO,
		/* .vbus_reg = "vdd_vbus_micro_usb", */
		.hot_plug = true,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};
#endif

#if defined(CONFIG_USB_SUPPORT)
static void tostab12AL_usb_init(void)
{
	/* OTG should be the first to be registered */
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	/* setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

	/* tegra_ehci2_device.dev.platform_data = &tegra_ehci2_utmi_pdata; */
	/* platform_device_register(&tegra_ehci2_device); */

	/* tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata; */
	/* platform_device_register(&tegra_ehci3_device); */

}
#else
static void tostab12AL_usb_init(void) { }
#endif


static void __init tegra_tostab12AL_init(void)
{
    struct board_info board_info;

    tegra_get_board_info(&board_info);
    pr_info("Board ID: %d, Revision: %d\n", board_info.board_id, board_rev());

	tegra_thermal_init(&thermal_data,
				throttle_list,
				ARRAY_SIZE(throttle_list));
	tegra_clk_init_from_table(tostab12AL_clk_init_table);
	tostab12AL_pinmux_init();
	tostab12AL_i2c_init();
	tostab12AL_spi_init();
	tostab12AL_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	tostab12AL_edp_init();
#endif
	tostab12AL_uart_init();
	platform_add_devices(tostab12AL_devices, ARRAY_SIZE(tostab12AL_devices));
	tegra_ram_console_debug_init();

	tostab12AL_audio_init();
	
#ifdef CONFIG_SND_SOC_FM34
	tostab12AL_fm34_init();
#endif
    tegra_io_dpd_init();
	tostab12AL_sdhci_init();
	tostab12AL_regulator_init();
	tostab12AL_ec_init();		
	tostab12AL_suspend_init();
    tostab12AL_power_off_init();
	tostab12AL_touch_init();
	tostab12AL_keys_init();
	tostab12AL_panel_init();
	tostab12AL_sensors_init();
	tostab12AL_setup_bluesleep();
	//audio_wired_jack_init();
	tostab12AL_pins_state_init();
	tostab12AL_emc_init();
	tegra_release_bootloader_fb();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
}

static void __init tegra_tostab12AL_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* support 1920X1200 with 24bpp */
	tegra_reserve(0, SZ_8M + SZ_1M, SZ_8M + SZ_1M);
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
	tegra_ram_console_debug_reserve(SZ_1M);
}

MACHINE_START(TOSTAB12AL, "tostab12AL")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_tostab12AL_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_tostab12AL_init,
MACHINE_END
