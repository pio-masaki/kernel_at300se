/*
 * arch/arm/mach-tegra/board-touch-raydium_spi.c
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

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/rm31080a_ts.h>

/*#include <mach/gpio-tegra.h>*/

struct rm_spi_ts_platform_data rm31080ts_data = {
	.gpio_reset = 0,
	.config = 0,
	.power_on = NULL,
	.power_off = NULL,
};

struct spi_board_info rm31080a_spi_board[1] = {
	{
	 .modalias = "rm_ts_spidev",
	 .bus_num = 0,
	 .chip_select = 0,
	 .max_speed_hz = 18 * 1000 * 1000,
	 .mode = SPI_MODE_0,
	 .platform_data = &rm31080ts_data,
	 },
};

int touch_set_callback(void* power_on, void* power_off)
{
	rm31080ts_data.power_on = power_on;
	rm31080ts_data.power_off = power_off;
	return 0;
}

int __init touch_init_raydium_and_spi(int irq_gpio, int reset_gpio, int platform, u16 bus_num, u16 chip_select)
{
	int err = 0;
	
	tegra_gpio_enable(irq_gpio);
        gpio_request(irq_gpio, "raydium-irq");
	gpio_direction_input(irq_gpio);

	tegra_gpio_enable(reset_gpio); //removed by nvidia 
	gpio_request(reset_gpio, "raydium-reset");
	gpio_direction_output(reset_gpio, 0);

	rm31080ts_data.gpio_reset = reset_gpio;

	msleep(5);
	gpio_set_value(reset_gpio, 1);
	msleep(5);

	rm31080a_spi_board[0].irq = TEGRA_GPIO_TO_IRQ(irq_gpio);
	rm31080a_spi_board[0].bus_num = bus_num;
	rm31080a_spi_board[0].chip_select = chip_select;

	rm31080ts_data.platform_id = platform;

	switch (platform) {
	case RM_PLATFORM_KAI_PCB:/*0x00*/
		pr_info("Raydium Kai PCB based touch init\n");
		break;
	case RM_PLATFORM_KAI:/*0x01*/
		pr_info("Raydium Kai On-Board touch init\n");
		break;
	case RM_PLATFORM_CARDHU:/*0x02*/
		pr_info("Raydium cardhu touch init\n");
		break;
	case RM_PLATFORM_DALMORE:/*0x03*/
		pr_info("Raydium dalmore touch init\n");
		break;
	case RM_PLATFORM_PLUTO:/*0x04*/
		pr_info("Raydium pluto touch init\n");
		break;
	case RM_PLATFORM_A10L:
		pr_info("Raydium a10L touch init\n");
		break;
	default:
		pr_err("touch_id error, no touch\n");
		err = -ENODEV;
		break;
	}

	if (!err)
		spi_register_board_info(rm31080a_spi_board,
					ARRAY_SIZE(rm31080a_spi_board));

	return err;
}

int __init touch_init_raydium(int irq_gpio, int reset_gpio, int platform)
{
	u16 bus_num = 0;
	u16 chip_select = 0;
	return touch_init_raydium_and_spi(irq_gpio,reset_gpio,RM_PLATFORM_A10L,bus_num,chip_select);
}
