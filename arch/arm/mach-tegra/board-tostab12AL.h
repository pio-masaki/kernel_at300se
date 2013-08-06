/*
 * arch/arm/mach-tegra/board-tostab12AL.h
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#ifndef _MACH_TEGRA_BOARD_TOSTAB12AL_H
#define _MACH_TEGRA_BOARD_TOSTAB12AL_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/tps6591x.h>

/* External peripheral act as gpio */
/* TPS6591x GPIOs */
#define TPS6591X_GPIO_BASE	TEGRA_NR_GPIOS
#define TPS6591X_GPIO_0		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP0)
#define TPS6591X_GPIO_1		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP1)
#define TPS6591X_GPIO_2		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP2)
#define TPS6591X_GPIO_3		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP3)
#define TPS6591X_GPIO_4		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP4)
#define TPS6591X_GPIO_5		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP5)
#define TPS6591X_GPIO_6		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP6)
#define TPS6591X_GPIO_7		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP7)
#define TPS6591X_GPIO_8		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP8)
#define TPS6591X_GPIO_END	(TPS6591X_GPIO_BASE + TPS6591X_GPIO_NR)

/* WiFi related GPIOs */
#define EN_WLAN_3V3_GPIO        TEGRA_GPIO_PV3
#define EN_WLAN_1V8_GPIO        TEGRA_GPIO_PCC5
#define WF_RST_GPIO             TEGRA_GPIO_PD3
#define WF_WAKEUP_GPIO          TEGRA_GPIO_PO4

/* Bluetooth related GPIOs */
#define BT_RST_GPIO             TEGRA_GPIO_PU0
#define BT_WAKEUP_GPIO          TEGRA_GPIO_PU1
#define BT_SHUTDOWN_GPIO        TEGRA_GPIO_PB2
#define BT_IRQ_GPIO             TEGRA_GPIO_PU6

/* GPS related GPIOs */
#define EN_GPS_3V3_GPIO         TEGRA_GPIO_PV2
#define EN_GPS_1V8_GPIO         TEGRA_GPIO_PW5
#define GPS_EN_GPIO             TEGRA_GPIO_PU2
#define GPS_RST_GPIO            TEGRA_GPIO_PU3

/* Camera related GPIOs */
#define CAM1_LDO1_EN_GPIO       TEGRA_GPIO_PH7
#define CAM1_LDO2_EN_GPIO       TEGRA_GPIO_PS7
#define CAM1_LDO3_EN_GPIO       TEGRA_GPIO_PP1
#define CAM1_LDO4_EN_GPIO       TEGRA_GPIO_PN2
#define CAM1_RST_GPIO           TEGRA_GPIO_PQ7
#define CAM1_PWDN_GPIO          TEGRA_GPIO_PI2
#define CAM2_LDO1_EN_GPIO       TEGRA_GPIO_PR7
#define CAM2_LDO2_EN_GPIO       TEGRA_GPIO_PBB4
#define CAM2_LDO3_EN_GPIO       TEGRA_GPIO_PBB7
#define CAM2_RST_GPIO           TEGRA_GPIO_PBB0
#define CAM2_PWDN_GPIO          TEGRA_GPIO_PBB6

/* uP related GPIOs */
#define EC_WAKE_GPIO            TEGRA_GPIO_PB1
#define EC_REQUEST_GPIO         TEGRA_GPIO_PK7
#define EC_PWR_STATE_GPIO       TEGRA_GPIO_PDD7
#define AP_WAKE_GPIO            TEGRA_GPIO_PK2
#define MCU_RST_GPIO            TEGRA_GPIO_PD2
#define AP_ACOK_GPIO            TEGRA_GPIO_PV1

/* Charger related GPIO */
#define CHG_INT_GPIO            TEGRA_GPIO_PS5
#define CHARGER_SHDN_GPIO       TEGRA_GPIO_PI4
#define CHG_DET_CPU_GPIO        TEGRA_GPIO_PY2


/* SD/eMMC related GPIOs */
#define EN_3V3_EMMC_GPIO        TEGRA_GPIO_PD1
#define SD_3V3_EN_GPIO          TEGRA_GPIO_PBB5
#define SDMMC_CD_GPIO           TEGRA_GPIO_PI5

/* Audio-related GPIOs */
#define CODEC_RST_GPIO          TEGRA_GPIO_PY3
#define CDC_IRQ_GPIO            TEGRA_GPIO_PW3
#define SPKR_EN_GPIO            TEGRA_GPIO_PH2
#define HP_DET_GPIO             TEGRA_GPIO_PW2
#define AMIC_DET_GPIO           TEGRA_GPIO_PS6
#define DMIC_EN_GPIO            TEGRA_GPIO_PX0
#define EN_MIC_EXT_GPIO         TEGRA_GPIO_PO1
#define HP_AMP_EN_GPIO          TEGRA_GPIO_PBB3

/* Voice decoder related GPIO */
#define VD_BP_GPIO              TEGRA_GPIO_PN3
#define VD_RST_GPIO             TEGRA_GPIO_PN0
#define VD_PWD_GPIO             TEGRA_GPIO_PN1

/* Button related GPIOs */
#define VOL_UP_BUTTON_GPIO      TEGRA_GPIO_PQ0
#define VOL_DWN_BUTTON_GPIO     TEGRA_GPIO_PQ1
#define AP_ONKEY_GPIO           TEGRA_GPIO_PV0

/* Board ID releated GPIOs */
#define BOARD_ID_A0_GPIO        TEGRA_GPIO_PR0
#define BOARD_ID_A1_GPIO        TEGRA_GPIO_PR1
#define BOARD_ID_A2_GPIO        TEGRA_GPIO_PR2
#define BOARD_ID_A3_GPIO        TEGRA_GPIO_PR3
#define BOARD_ID_A4_GPIO        TEGRA_GPIO_PR4

/* LCD related GPIOs */
#define LCD_BL_EN_GPIO          TEGRA_GPIO_PH3
#define LCD_BL_PWM_GPIO         TEGRA_GPIO_PH0
#define EN_VDD_BL_GPIO          TEGRA_GPIO_PH5
#define EN_VDD_PNL_GPIO         TEGRA_GPIO_PW1
#define LVDS_SHTDN_GPIO         TEGRA_GPIO_PN6
#define LCD_RS_GPIO             TEGRA_GPIO_PV6

/* Touch related GPIOs */
#define TOUCH_3V3_INT_GPIO      TEGRA_GPIO_PH4
#define TOUCH_3V3_WAKE_GPIO     TEGRA_GPIO_PJ2
#define TOUCH_3V3_ID_GPIO       TEGRA_GPIO_PK3
#define TOUCH_3V3_RST_GPIO      TEGRA_GPIO_PH6
#define TS_EN_GPIO              TEGRA_GPIO_PH1
#define TOUCH_INT_GPIO          TEGRA_GPIO_PZ3
#define TOUCH_RST_GPIO          TEGRA_GPIO_PN5

/* Sensors related GPIOs */
#define SEN_3V3_EN_GPIO         TEGRA_GPIO_PK5
#define SEN_1V8_EN_GPIO         TEGRA_GPIO_PS1
#define LS_INT_GPIO             TEGRA_GPIO_PZ2
#define COMPASS_RSTN_GPIO       TEGRA_GPIO_PN7
#define COMPASS_DRDY_GPIO       TEGRA_GPIO_PW0
#define TEMP_ALERT_GPIO         TEGRA_GPIO_PCC2

/* Misc GPIOs */
#define USB_OTG_EN_GPIO         TEGRA_GPIO_PU4
#define EN_3V3_FUSE_GPIO        TEGRA_GPIO_PC1
#define DEBUG_MODE_SEL_GPIO     TEGRA_GPIO_PC6

/*****************Interrupt tables ******************/
/* External peripheral act as interrupt controller */
/* TPS6591x IRQs */
#define TPS6591X_IRQ_BASE	TEGRA_NR_IRQS
#define TPS6591X_IRQ_END	(TPS6591X_IRQ_BASE + 18)

int tostab12AL_charge_init(void);
int tostab12AL_regulator_init(void);
int tostab12AL_suspend_init(void);
int tostab12AL_sdhci_init(void);
int tostab12AL_pinmux_init(void);
int tostab12AL_panel_init(void);
int tostab12AL_sensors_init(void);
int tostab12AL_kbc_init(void);
int tostab12AL_scroll_init(void);
int tostab12AL_keys_init(void);
int tostab12AL_gpio_switch_regulator_init(void);
int tostab12AL_pins_state_init(void);
int tostab12AL_emc_init(void);
int tostab12AL_edp_init(void);
int tostab12AL_ec_init(void);
int tostab12AL_power_off_init(void);
int tostab12AL_touch_poweroff(void);
int touch_set_callback(void* power_on, void* power_off);
void __init tostab12AL_tsensor_init(void);
int __init touch_init_raydium(int irq_gpio, int reset_gpio, int platform);

/* Invensense MPU Definitions */
#define MPU_GYRO_NAME		"mpu6050"
#define MPU_GYRO_INT         	TEGRA_GPIO_PX1
#define MPU_GYRO_ADDR        	0x68

#define MPU_COMPASS_RESET	TEGRA_GPIO_PN7
#define MPU_COMPASS_NAME     	"akm8963"
#define MPU_COMPASS_INT      	TEGRA_GPIO_PW0
#define MPU_COMPASS_ADDR 	0x0C

/* Orientation matrix of TOSTAB12AL */
#define MPU_GYRO_ORIENTATION    { -1, 0, 0, 0, 1, 0, 0, 0, -1 }
#define MPU_COMPASS_ORIENTATION	{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }	

#define MPU_GYRO_KEY	{0xfe, 0x85, 0xc0, 0x63, 0xb2, 0x89, 0x98, 0x25,\
			 0xb2, 0xee, 0x3e, 0x42, 0xce, 0x0e, 0x38, 0xb7}

#define TDIODE_OFFSET	(10000)	/* in millicelsius */

#define VDD_BACKLIGHT_VOLTAGE         (5000)

extern unsigned int system_rev;
#define is_evt_board()       (system_rev == 0)
#define is_dvt_board()       (system_rev == 1)
#define is_pvt_board()       (system_rev == 2)
#define is_mp_board()        (system_rev == 3)
#define board_rev()     (system_rev)

enum power_type{
    WIFI_POWER=0,
    BT_POWER=1
};

#endif
