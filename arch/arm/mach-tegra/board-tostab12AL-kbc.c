/*
 * arch/arm/mach-tegra/board-tostab12AL-kbc.c
 * Keys configuration for Nvidia tegra3 tostab12AL platform.
 *
 * Copyright (C) 2011 NVIDIA, Inc.
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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/tps6591x.h>
#include <linux/mfd/max77663-core.h>
#include <linux/gpio_scrollwheel.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <mach/irqs.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include "board.h"
#include "board-tostab12AL.h"

#include "gpio-names.h"
#include "devices.h"
#include "wakeups-t3.h"

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

#define GPIO_IKEY(_id, _irq, _iswake, _deb)	\
	{					\
		.code = _id,			\
		.gpio = -1,			\
		.irq = _irq,			\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = _deb,	\
	}

static struct gpio_keys_button tostab12AL_keys[] = {
	[0] = GPIO_KEY(KEY_POWER, PV0, 1),
	[1] = GPIO_KEY(KEY_VOLUMEUP, PQ0, 0),
	[2] = GPIO_KEY(KEY_VOLUMEDOWN, PQ1, 0),
};

#define PMC_WAKE_STATUS		0x14
static int tostab12AL_wakeup_key(void)
{
    unsigned long status = readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
  
    if ((status & (1ul << TEGRA_WAKE_GPIO_PV0)) ||
        (get_pending_wakeup_irq() == TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV0))) { /* power key */
        return KEY_POWER;
    } else {
        return KEY_RESERVED;
    }
}

static struct gpio_keys_platform_data tostab12AL_keys_pdata = {
	.buttons	= tostab12AL_keys,
	.nbuttons	= ARRAY_SIZE(tostab12AL_keys),
	.wakeup_key 	= tostab12AL_wakeup_key,
};

static struct platform_device tostab12AL_keys_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &tostab12AL_keys_pdata,
	},
};

int __init tostab12AL_keys_init(void)
{
	pr_info("Registering gpio keys\n");

	platform_device_register(&tostab12AL_keys_device);
	return 0;
}
