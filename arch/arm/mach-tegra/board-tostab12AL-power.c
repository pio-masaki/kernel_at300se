/*
 * arch/arm/mach-tegra/board-tostab12AL-power.c
 *
 * Copyright (C) 2011-2012, NVIDIA Corporation.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/tps6591x.h>
#include <linux/mfd/max77663-core.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/tps6591x-regulator.h>
#include <linux/regulator/tps62360.h>
#include <linux/power/gpio-charger.h>

#include <asm/mach-types.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/edp.h>

#include "gpio-names.h"
#include "board.h"
#include "board-tostab12AL.h"
#include "pm.h"
#include "wakeups-t3.h"
#include "tegra3_tsensor.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

static struct regulator_consumer_supply tps6591x_vdd1_supply_0[] = {
	REGULATOR_SUPPLY("unused_vdd1_rails", NULL),
};

static struct regulator_consumer_supply tps6591x_vdd2_supply_tostab12AL[] = {
    REGULATOR_SUPPLY("unused_vdd2_rails", NULL),
};

static struct regulator_consumer_supply tps6591x_vddctrl_supply_0[] = {
	REGULATOR_SUPPLY("vdd_cpu_pmu", NULL),
	REGULATOR_SUPPLY("vdd_cpu", NULL),
	REGULATOR_SUPPLY("vdd_sys", NULL),
};

static struct regulator_consumer_supply tps6591x_vio_supply_0[] = {
	REGULATOR_SUPPLY("vdd_gen1v8", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("vdd1v8_satelite", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vddio_lcd_pmu", NULL),
	REGULATOR_SUPPLY("pwrdet_lcd", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("vddio_vi", NULL),
	REGULATOR_SUPPLY("pwrdet_vi", NULL),
	REGULATOR_SUPPLY("ldo6", NULL),
	REGULATOR_SUPPLY("ldo7", NULL),
	REGULATOR_SUPPLY("ldo8", NULL),
	REGULATOR_SUPPLY("vcore_audio", NULL),
	REGULATOR_SUPPLY("avcore_audio", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
	REGULATOR_SUPPLY("vcore1_lpddr2", NULL),
	REGULATOR_SUPPLY("vcom_1v8", NULL),
	REGULATOR_SUPPLY("pmuio_1v8", NULL),
	REGULATOR_SUPPLY("avdd_ic_usb", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo1_supply_tostab12AL[] = {
    REGULATOR_SUPPLY("unused_rail_ldo1", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo2_supply_tostab12AL[] = {
    REGULATOR_SUPPLY("ununsed_rail_ldo2", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo3_supply_tostab12AL[] = {
    REGULATOR_SUPPLY("vdd_1v8_codec", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo4_supply_0[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo5_supply_tostab12AL[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo6_supply_0[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("pwrdet_mipi", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo7_supply_0[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c_s", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d2", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo8_supply_0[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

#define TPS_PDATA_INIT(_name, _sname, _minmv, _maxmv, _supply_reg, _always_on, \
	_boot_on, _apply_uv, _init_uV, _init_enable, _init_apply, _ectrl, _flags) \
	static struct tps6591x_regulator_platform_data pdata_##_name##_##_sname = \
	{								\
		.regulator = {						\
			.constraints = {				\
				.min_uV = (_minmv)*1000,		\
				.max_uV = (_maxmv)*1000,		\
				.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
						     REGULATOR_MODE_STANDBY), \
				.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
						   REGULATOR_CHANGE_STATUS |  \
						   REGULATOR_CHANGE_VOLTAGE), \
				.always_on = _always_on,		\
				.boot_on = _boot_on,			\
				.apply_uV = _apply_uv,			\
			},						\
			.num_consumer_supplies =			\
				ARRAY_SIZE(tps6591x_##_name##_supply_##_sname),	\
			.consumer_supplies = tps6591x_##_name##_supply_##_sname,	\
			.supply_regulator = _supply_reg,		\
		},							\
		.init_uV =  _init_uV * 1000,				\
		.init_enable = _init_enable,				\
		.init_apply = _init_apply,				\
		.ectrl = _ectrl,					\
		.flags = _flags,					\
	}

/* TOSTAB12AL common TPS6591x Power rails */
TPS_PDATA_INIT(vdd1,    0,      600,  1500, 0, 1, 1, 0, -1, 0, 0, EXT_CTRL_SLEEP_OFF, 0);
TPS_PDATA_INIT(vddctrl, 0,      600,  1400, 0, 1, 1, 0, -1, 0, 0, EXT_CTRL_EN1, 0);
TPS_PDATA_INIT(vio,     0,      1500, 3300, 0, 1, 1, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo4, 0,         1000, 3300, 0, 1, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo6, 0,         1200, 1200, tps6591x_rails(VIO), 0, 0, 1, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo7, 0,         1200, 1200, tps6591x_rails(VIO), 1, 1, 1, -1, 0, 0, EXT_CTRL_SLEEP_OFF, LDO_LOW_POWER_ON_SUSPEND);
TPS_PDATA_INIT(ldo8, 0,         1000, 3300, tps6591x_rails(VIO), 1, 0, 0, -1, 0, 0, EXT_CTRL_SLEEP_OFF, LDO_LOW_POWER_ON_SUSPEND);

/* tps6591x: add tostab12AL-specific power rails here*/
TPS_PDATA_INIT(vdd2, tostab12AL,    600,  1500, 0, 1, 1, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo1, tostab12AL,    1000, 3300, tps6591x_rails(VDD_2), 0, 0, 0, -1, 0, 1, 0, 0);
TPS_PDATA_INIT(ldo2, tostab12AL,    1050, 1050, tps6591x_rails(VDD_2), 0, 0, 1, -1, 0, 1, 0, 0);
/*TPS_PDATA_INIT(ldo3, tostab12AL,    1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0);*/
TPS_PDATA_INIT(ldo3, tostab12AL,    1000, 3300, 0, 0, 0, 0, 1800, 0, 1, 0, 0);
TPS_PDATA_INIT(ldo5, tostab12AL,    1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0);

#if defined(CONFIG_RTC_DRV_TPS6591x)
static struct tps6591x_rtc_platform_data rtc_data = {
	.irq = TEGRA_NR_IRQS + TPS6591X_INT_RTC_ALARM,
	.time = {
		.tm_year = 2000,
		.tm_mon = 0,
		.tm_mday = 1,
		.tm_hour = 0,
		.tm_min = 0,
		.tm_sec = 0,
	},
};

#define TPS_RTC_REG()					\
	{						\
		.id	= 0,				\
		.name	= "rtc_tps6591x",		\
		.platform_data = &rtc_data,		\
	}
#endif

#define TPS_REG(_id, _name, _sname)				\
	{							\
		.id	= TPS6591X_ID_##_id,			\
		.name	= "tps6591x-regulator",			\
		.platform_data	= &pdata_##_name##_##_sname,	\
	}

/* TOSTAB12AL common TPS6591x power rails */
#define TPS6591X_DEV_COMMON_TOSTAB12AL		\
    TPS_REG(VDD_1, vdd1, 0),        \
    TPS_REG(VDDCTRL, vddctrl, 0),   \
    TPS_REG(VIO, vio, 0),           \
    TPS_REG(LDO_4, ldo4, 0),        \
    TPS_REG(LDO_6, ldo6, 0),        \
    TPS_REG(LDO_7, ldo7, 0),        \
    TPS_REG(LDO_8, ldo8, 0)

/* power-rail: add tostab12AL-specific power rail here */
#define TPS6591X_DEV_TOSTAB12AL         \
    TPS_REG(VDD_2, vdd2, tostab12AL),   \
    TPS_REG(LDO_1, ldo1, tostab12AL),   \
    TPS_REG(LDO_2, ldo2, tostab12AL),   \
    TPS_REG(LDO_3, ldo3, tostab12AL),   \
    TPS_REG(LDO_5, ldo5, tostab12AL)

static struct tps6591x_subdev_info tps_devs_tostab12AL[] = {
	TPS6591X_DEV_COMMON_TOSTAB12AL,
	TPS6591X_DEV_TOSTAB12AL,
#if defined(CONFIG_RTC_DRV_TPS6591x)
	TPS_RTC_REG(),
#endif
};

#define TPS_GPIO_INIT_PDATA(gpio_nr, _init_apply, _sleep_en, _pulldn_en, _output_en, _output_val)	\
	[gpio_nr] = {					\
			.sleep_en	= _sleep_en,	\
			.pulldn_en	= _pulldn_en,	\
			.output_mode_en	= _output_en,	\
			.output_val	= _output_val,	\
			.init_apply	= _init_apply,	\
		     }
static struct tps6591x_gpio_init_data tps_gpio_pdata_e1291_a04[] =  {
	TPS_GPIO_INIT_PDATA(0, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(1, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(2, 1, 1, 0, 1, 1),
	TPS_GPIO_INIT_PDATA(3, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(4, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(5, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(6, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(7, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(8, 0, 0, 0, 0, 0),
};

static struct tps6591x_sleep_keepon_data tps_slp_keepon = {
	.clkout32k_keepon = 1,
};

static struct tps6591x_platform_data tps_platform = {
	.irq_base	= TPS6591X_IRQ_BASE,
	.gpio_base	= TPS6591X_GPIO_BASE,
	.dev_slp_en	= true,
	.slp_keepon	= &tps_slp_keepon,
	.use_power_off	= false, 
};

static struct i2c_board_info __initdata tostab12AL_regulators[] = {
	{
		I2C_BOARD_INFO("tps6591x", 0x2D),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
};

/* TPS62361B DC-DC converter */
static struct regulator_consumer_supply tps62361_dcdc_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct tps62360_regulator_platform_data tps62361_pdata = {
	.reg_init_data = {					\
		.constraints = {				\
			.min_uV = 500000,			\
			.max_uV = 1770000,			\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
					     REGULATOR_MODE_STANDBY), \
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
					   REGULATOR_CHANGE_STATUS |  \
					   REGULATOR_CHANGE_VOLTAGE), \
			.always_on = 1,				\
			.boot_on =  1,				\
			.apply_uV = 0,				\
		},						\
		.num_consumer_supplies = ARRAY_SIZE(tps62361_dcdc_supply), \
		.consumer_supplies = tps62361_dcdc_supply,	\
		},						\
	.en_discharge = true,					\
	.vsel0_gpio = -1,					\
	.vsel1_gpio = -1,					\
	.vsel0_def_state = 1,					\
	.vsel1_def_state = 1,					\
};

static struct i2c_board_info __initdata tps62361_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps62361", 0x60),
		.platform_data	= &tps62361_pdata,
	},
};

int __init tostab12AL_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */

	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	/* The regulator details have complete constraints */
	regulator_has_full_constraints();

	tps_platform.num_subdevs = ARRAY_SIZE(tps_devs_tostab12AL);
	tps_platform.subdevs = tps_devs_tostab12AL;

    tps_platform.dev_slp_en = true;
    tps_platform.gpio_init_data = tps_gpio_pdata_e1291_a04;
    tps_platform.num_gpioinit_data =
        ARRAY_SIZE(tps_gpio_pdata_e1291_a04);

	i2c_register_board_info(4, tostab12AL_regulators, 1);

	/* Resgister the TPS6236x for all boards whose sku bit 0 is set. */
    pr_info("Registering the device TPS62361\n");
    i2c_register_board_info(4, tps62361_boardinfo, 1);

	return 0;
}


/**************** GPIO based fixed regulator *****************/
/* EN_5V_CP from PMU GP0 */
static struct regulator_consumer_supply fixed_reg_en_5v_cp_supply[] = {
	REGULATOR_SUPPLY("vdd_5v0_sby", NULL),
	REGULATOR_SUPPLY("vdd_hall", NULL),
	REGULATOR_SUPPLY("vterm_ddr", NULL),
	REGULATOR_SUPPLY("v2ref_ddr", NULL),
};

/* EN_5V0 From PMU GP2 */
static struct regulator_consumer_supply fixed_reg_en_5v0_supply[] = {
	REGULATOR_SUPPLY("vdd_5v0_sys", NULL),
};

/* EN_DDR From PMU GP6 */
static struct regulator_consumer_supply fixed_reg_en_ddr_supply[] = {
	REGULATOR_SUPPLY("mem_vddio_ddr", NULL),
	REGULATOR_SUPPLY("t30_vddio_ddr", NULL),
};

/* EN_3V3_SYS From PMU GP7 */
static struct regulator_consumer_supply fixed_reg_en_3v3_sys_supply[] = {
	REGULATOR_SUPPLY("vdd_lvds", NULL),
	REGULATOR_SUPPLY("vdd_pnl", NULL),
	REGULATOR_SUPPLY("vcom_3v3", NULL),
	REGULATOR_SUPPLY("vdd_3v3", NULL),
	REGULATOR_SUPPLY("vcore_mmc", NULL),
	REGULATOR_SUPPLY("vddio_pex_ctl", NULL),
	REGULATOR_SUPPLY("pwrdet_pex_ctl", NULL),
	REGULATOR_SUPPLY("hvdd_pex_pmu", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("vpp_fuse", NULL),
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
	REGULATOR_SUPPLY("vcore_nand", NULL),
	REGULATOR_SUPPLY("hvdd_sata", NULL),
	REGULATOR_SUPPLY("vddio_gmi_pmu", NULL),
	REGULATOR_SUPPLY("pwrdet_nand", NULL),
	REGULATOR_SUPPLY("avdd_cam1", NULL),
	REGULATOR_SUPPLY("vdd_af", NULL),
	REGULATOR_SUPPLY("avdd_cam2", NULL),
	REGULATOR_SUPPLY("vdd_acc", NULL),
	REGULATOR_SUPPLY("vdd_phtl", NULL),
	REGULATOR_SUPPLY("vddio_tp", NULL),
	REGULATOR_SUPPLY("vdd_led", NULL),
	REGULATOR_SUPPLY("vddio_cec", NULL),
	REGULATOR_SUPPLY("vdd_cmps", NULL),
	REGULATOR_SUPPLY("vdd_temp", NULL),
	REGULATOR_SUPPLY("vpp_kfuse", NULL),
	REGULATOR_SUPPLY("vddio_ts", NULL),
	REGULATOR_SUPPLY("vdd_ir_led", NULL),
	REGULATOR_SUPPLY("vddio_1wire", NULL),
	REGULATOR_SUPPLY("avddio_audio", NULL),
	REGULATOR_SUPPLY("vdd_ec", NULL),
	REGULATOR_SUPPLY("vcom_pa", NULL),
	REGULATOR_SUPPLY("vdd_3v3_devices", NULL),
	REGULATOR_SUPPLY("vdd_3v3_dock", NULL),
	REGULATOR_SUPPLY("vdd_3v3_edid", NULL),
	REGULATOR_SUPPLY("vdd_3v3_hdmi_cec", NULL),
	REGULATOR_SUPPLY("vdd_3v3_gmi", NULL),
	REGULATOR_SUPPLY("vdd_spk_amp", "tegra-snd-aic325x"),
	REGULATOR_SUPPLY("vdd_hp_amp", "tegra-snd-aic325x"),
	REGULATOR_SUPPLY("vdd_3v3_cam", NULL),
	REGULATOR_SUPPLY("debug_cons", NULL),
	REGULATOR_SUPPLY("vdd", "4-004c"),
};

/* EN_3V3_SPK */
static struct regulator_consumer_supply fixed_reg_en_3v3_spk_supply[] = {
	REGULATOR_SUPPLY("vdd_spk_amp", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_3v3_hp_amp_supply[] = {
	REGULATOR_SUPPLY("vdd_hp_amp", NULL),
};

/* EN_1V8_DMIC */
static struct regulator_consumer_supply fixed_reg_en_1v8_dmic_supply[] = {
	REGULATOR_SUPPLY("vdd_dmic", NULL),
};

/* EN_VDD_PNL */
static struct regulator_consumer_supply fixed_reg_en_vdd_pnl_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_panel", NULL),
};

/* EN_VDD_BL */
static struct regulator_consumer_supply fixed_reg_en_vdd_bl_supply[] = {
	REGULATOR_SUPPLY("vdd_backlight", NULL),
};

/* CAM1_LDO1_EN */
static struct regulator_consumer_supply fixed_reg_cam1_ldo1_en_supply[] = {
	REGULATOR_SUPPLY("vdd_1v8_cam1", NULL),
};

/* CAM1_LDO2_EN */
static struct regulator_consumer_supply fixed_reg_cam1_ldo2_en_supply[] = {
	REGULATOR_SUPPLY("vddio_1v8_cam1", NULL),
};

/* CAM1_LDO3_EN */
static struct regulator_consumer_supply fixed_reg_cam1_ldo3_en_supply[] = {
	REGULATOR_SUPPLY("avdd_2v8_cam1", NULL),
};

/* CAM1_LDO4_EN */
static struct regulator_consumer_supply fixed_reg_cam1_ldo4_en_supply[] = {
	REGULATOR_SUPPLY("af_vdd_2v8_cam1", NULL),
};

/* CAM2_LDO1_EN */
static struct regulator_consumer_supply fixed_reg_cam2_ldo1_en_supply[] = {
	REGULATOR_SUPPLY("vdd_1v8_cam2", NULL),
};


/* CAM2_LDO2_EN */
static struct regulator_consumer_supply fixed_reg_cam2_ldo2_en_supply[] = {
	REGULATOR_SUPPLY("vddio_1v8_cam2", NULL),
};


/* CAM2_LDO3_EN */
static struct regulator_consumer_supply fixed_reg_cam2_ldo3_en_supply[] = {
	REGULATOR_SUPPLY("avdd_2v8_cam2", NULL),
};

/* EN_VDD_SDMMC1 from AP GPIO GPIO_PBB5*/
static struct regulator_consumer_supply fixed_reg_en_vdd_sdmmc1_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.0"),
};

/* EN_3V3_EMMC from AP GPIO SDMMC3_DAT4 D01*/
static struct regulator_consumer_supply fixed_reg_en_3v3_emmc_supply[] = {
	REGULATOR_SUPPLY("vdd_emmc_core", NULL),
};

/* EN_3v3_FUSE from AP GPIO VI_D08 L06*/
static struct regulator_consumer_supply fixed_reg_en_3v3_fuse_supply[] = {
	REGULATOR_SUPPLY("vdd_fuse", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_3v3_sensor_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3_sensor", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_1v8_sensor_supply[] = {
	REGULATOR_SUPPLY("vdd_1v8_sensor", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_3v3_wlan_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3_wlan", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_1v8_wlan_supply[] = {
	REGULATOR_SUPPLY("vdd_1v8_wlan", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_3v3_gps_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3_gps", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_1v8_gps_supply[] = {
	REGULATOR_SUPPLY("vdd_1v8_gps", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_3v3_ts_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3_ts", NULL),
};

/* Macro for defining fixed regulator sub device data */
#define FIXED_SUPPLY(_name) "fixed_reg_"#_name
#define FIXED_REG_OD(_id, _var, _name, _in_supply, _always_on,		\
		_boot_on, _gpio_nr, _active_high, _boot_state,		\
		_millivolts, _od_state)					\
	static struct regulator_init_data ri_data_##_var =		\
	{								\
		.supply_regulator = _in_supply,				\
		.num_consumer_supplies =				\
			ARRAY_SIZE(fixed_reg_##_name##_supply),		\
		.consumer_supplies = fixed_reg_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on,			\
			.boot_on = _boot_on,				\
		},							\
	};								\
	static struct fixed_voltage_config fixed_reg_##_var##_pdata =	\
	{								\
		.supply_name = FIXED_SUPPLY(_name),			\
		.microvolts = _millivolts * 1000,			\
		.gpio = _gpio_nr,					\
		.enable_high = _active_high,				\
		.enabled_at_boot = _boot_state,				\
		.init_data = &ri_data_##_var,				\
		.gpio_is_open_drain = _od_state,			\
	};								\
	static struct platform_device fixed_reg_##_var##_dev = {	\
		.name   = "reg-fixed-voltage",				\
		.id     = _id,						\
		.dev    = {						\
			.platform_data = &fixed_reg_##_var##_pdata,	\
		},							\
	}

#define FIXED_REG(_id, _var, _name, _in_supply, _always_on, _boot_on,	\
		 _gpio_nr, _active_high, _boot_state, _millivolts)	\
	FIXED_REG_OD(_id, _var, _name, _in_supply, _always_on, _boot_on,  \
		_gpio_nr, _active_high, _boot_state, _millivolts, false)

FIXED_REG(0,  en_5v_cp,         en_5v_cp,	    NULL,			    1,	0,  TPS6591X_GPIO_0,	true,	1,	5000);
FIXED_REG(1,  en_3v3_sys,       en_3v3_sys,	    NULL,			    0,  0,  TPS6591X_GPIO_6,	true,	1,	3300);
FIXED_REG(2,  en_ddr,           en_ddr,		    NULL,			    0,  0,  TPS6591X_GPIO_7,	true,	0,	1500);
FIXED_REG(3,  en_5v0,           en_5v0,		    NULL,			    1,  0,  TPS6591X_GPIO_8,	true,	1,	5000);

FIXED_REG(4,  en_3v3_sensor,	en_3v3_sensor,	FIXED_SUPPLY(en_3v3_sys),	1,	1,	SEN_3V3_EN_GPIO,	true,	0,	3300);
FIXED_REG(5,  en_vdd_sdmmc1,	en_vdd_sdmmc1,	FIXED_SUPPLY(en_3v3_sys),	0,	0,	SD_3V3_EN_GPIO,	    true,	0,	3300);
FIXED_REG(6,  cam1_ldo1_en,     cam1_ldo1_en,	tps6591x_rails(VIO),		0,	0,	CAM1_LDO1_EN_GPIO,	true,	0,	1800);
FIXED_REG(7,  cam1_ldo2_en,	    cam1_ldo2_en,	tps6591x_rails(VIO),		0,	0,	CAM1_LDO2_EN_GPIO,	true,	0,	1800);
FIXED_REG(8,  cam1_ldo3_en,	    cam1_ldo3_en,	FIXED_SUPPLY(en_3v3_sys),	0,	0,	CAM1_LDO3_EN_GPIO,	true,	0,	3300);
FIXED_REG(9,  cam1_ldo4_en,	    cam1_ldo4_en,	FIXED_SUPPLY(en_3v3_sys),	0,	0,	CAM1_LDO4_EN_GPIO,	true,	0,	3300);
FIXED_REG(10, cam2_ldo1_en,	    cam2_ldo1_en,	tps6591x_rails(VIO),		0,	0,	CAM2_LDO1_EN_GPIO,	true,	0,	1800);
FIXED_REG(11, cam2_ldo2_en,	    cam2_ldo2_en,	tps6591x_rails(VIO),		0,	0,	CAM2_LDO2_EN_GPIO,	true,	0,	1800);
FIXED_REG(12, cam2_ldo3_en,	    cam2_ldo3_en,	FIXED_SUPPLY(en_3v3_sys),	0,	0,	CAM2_LDO3_EN_GPIO,	true,	0,	3300);
FIXED_REG(13, en_3v3_gps,	    en_3v3_gps,	    FIXED_SUPPLY(en_3v3_sys),	0,	0,	EN_GPS_3V3_GPIO,	true,	0,	3300);
FIXED_REG(14, en_1v8_gps,	    en_1v8_gps,	    tps6591x_rails(VIO),		0,	0,	EN_GPS_1V8_GPIO,	true,	0,	1800);
FIXED_REG(15, en_vdd_pnl,	    en_vdd_pnl,	    FIXED_SUPPLY(en_3v3_sys),	0,  0,  EN_VDD_PNL_GPIO,	true,	1,	3300);
FIXED_REG(16, en_3v3_fuse,	    en_3v3_fuse,	FIXED_SUPPLY(en_3v3_sys),	0,  0,  EN_3V3_FUSE_GPIO,	true,	0,	3300);
FIXED_REG(17, en_3v3_hp_amp,    en_3v3_hp_amp,	FIXED_SUPPLY(en_3v3_sys),	0,	0,	HP_AMP_EN_GPIO,		true,	0,	3300);
FIXED_REG(18, en_3v3_wlan,	    en_3v3_wlan,	FIXED_SUPPLY(en_3v3_sys),	0,	0,	EN_WLAN_3V3_GPIO,	true,	0,	3300);
FIXED_REG(19, en_1v8_wlan,	    en_1v8_wlan,	tps6591x_rails(VIO),		0,	0,	EN_WLAN_1V8_GPIO,	true,	0,	1800);
FIXED_REG(20, en_3v3_spk,	    en_3v3_spk,		FIXED_SUPPLY(en_3v3_sys),	0,	0,	SPKR_EN_GPIO,		true,	0,	3300);
FIXED_REG(21, en_1v8_dmic,	    en_1v8_dmic,	tps6591x_rails(VIO),		0,	0,	DMIC_EN_GPIO,		true,	0,	1800);

/* gpio-switch: add tostab12AL-specific gpio switch regulator here */
FIXED_REG(30, en_3v3_emmc_tostab12AL,	    en_3v3_emmc,	FIXED_SUPPLY(en_3v3_sys),	1,  0,  EN_3V3_EMMC_GPIO,	true,	1,	3300);
FIXED_REG(31, en_1v8_sensor_tostab12AL,	    en_1v8_sensor,	tps6591x_rails(VIO),		0,	1,	SEN_1V8_EN_GPIO,	true,	0,	1800);
FIXED_REG(32, en_3v3_ts_tostab12AL,	        en_3v3_ts,	    FIXED_SUPPLY(en_3v3_sys),	0,  0,  TS_EN_GPIO,		    true,	0,	3300);
FIXED_REG(34, en_vdd_bl,	                en_vdd_bl,	    NULL,               0,	0,	EN_VDD_BL_GPIO,		true,	1,	9000);
/* FIXED_REG(33, en_1v8_audio,	                en_1v8_audio,	tps6591x_rails(VIO),		0,	0,	TEGRA_GPIO_PD4,		true,	0,	1800); */


/*
 * Creating the fixed/gpio-switch regulator device tables for different boards
 */
#define ADD_FIXED_REG(_name)	(&fixed_reg_##_name##_dev)

#define COMMON_FIXED_REG			\
	ADD_FIXED_REG(en_5v_cp),			\
    ADD_FIXED_REG(en_3v3_sys),       \
	ADD_FIXED_REG(en_5v0),		    \
    ADD_FIXED_REG(en_3v3_sensor),    \
    ADD_FIXED_REG(en_vdd_sdmmc1),    \
    ADD_FIXED_REG(cam1_ldo1_en),     \
	ADD_FIXED_REG(cam1_ldo2_en),	    \
	ADD_FIXED_REG(cam1_ldo3_en),     \
    ADD_FIXED_REG(cam1_ldo4_en),     \
	ADD_FIXED_REG(cam2_ldo1_en),	    \
	ADD_FIXED_REG(cam2_ldo2_en),	    \
	ADD_FIXED_REG(cam2_ldo3_en),     \
    ADD_FIXED_REG(en_3v3_gps),       \
    ADD_FIXED_REG(en_1v8_gps),       \
    ADD_FIXED_REG(en_vdd_pnl),		\
    ADD_FIXED_REG(en_vdd_bl),		\
    ADD_FIXED_REG(en_3v3_fuse),      \
    ADD_FIXED_REG(en_3v3_wlan),      \
	ADD_FIXED_REG(en_1v8_wlan),      \
	ADD_FIXED_REG(en_3v3_spk),		\
    ADD_FIXED_REG(en_1v8_dmic),		\
	ADD_FIXED_REG(en_3v3_hp_amp),	\
    ADD_FIXED_REG(en_1v8_sensor_tostab12AL),
//ADD_FIXED_REG(en_3v3_ts_tostab12AL),
    /* ADD_FIXED_REG(en_1v8_audio), */



/* Fixed regulator devices for tostab12AL */
static struct platform_device *fixed_reg_devs_tostab12AL[] = {
	COMMON_FIXED_REG
};


int __init tostab12AL_fixed_regulator_init(void)
{
	struct platform_device **fixed_reg_devs;
	int    nfixreg_devs;

	nfixreg_devs = ARRAY_SIZE(fixed_reg_devs_tostab12AL);
	fixed_reg_devs = fixed_reg_devs_tostab12AL;

	return platform_add_devices(fixed_reg_devs, nfixreg_devs);
}
subsys_initcall_sync(tostab12AL_fixed_regulator_init);

static void tostab12AL_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void tostab12AL_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data tostab12AL_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 200,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.cpu_lp2_min_residency = 2000,
	.board_suspend = tostab12AL_board_suspend,
	.board_resume = tostab12AL_board_resume,
#ifdef CONFIG_TEGRA_LP1_950
    .lp1_lowvolt_support = true,
    .i2c_base_addr = TEGRA_I2C5_BASE,
    .pmuslave_addr = 0x78,
    .core_reg_addr = 0x17,
    .lp1_core_volt_low = 0x0C,
    .lp1_core_volt_high = 0x20,
#endif
};

int __init tostab12AL_suspend_init(void)
{
    tostab12AL_suspend_data.corereq_high = true;

	tegra_init_suspend(&tostab12AL_suspend_data);
	return 0;
}

#ifdef CONFIG_TEGRA_EDP_LIMITS

int __init tostab12AL_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA) {
		regulator_mA = 6000; /* regular T30/s */
	}
	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_cpu_edp_limits(regulator_mA);
	return 0;
}
#endif

static void tostab12AL_power_off(void)
{
    int ret;
    pr_err("tostab12AL: Powering off  touch\n");
	tostab12AL_touch_poweroff();

	pr_err("tostab12AL: Powering off the device\n");
    ret = tps6591x_power_off();
    if (ret)
        pr_err("tostab12AL: failed to power off\n");
    
    while (1);
}

int __init tostab12AL_power_off_init(void)
{
    pm_power_off = tostab12AL_power_off;
    return 0;
}
