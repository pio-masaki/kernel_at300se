#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/resource.h>

#include <linux/io.h>
#include <linux/mfd/nvtec.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include <asm/mach-types.h>

#include "board-tostab12AL.h"

#include "gpio-names.h"
#include "board.h"

#define TS_PINS_NULL 0

static struct nvtec_battery_platform_data nvtec_battery_pdata = {
    .ac_in_pin = AP_ACOK_GPIO,
    .batt_low_pin = TS_PINS_NULL,
};

static struct nvtec_subdev_info nvtec_devs[] = {
    {
        .id = 0,
        .name = "nvtec-battery",
        .platform_data = &nvtec_battery_pdata,
    },
};

static struct nvtec_platform_data nvtec_pdata = {
    .num_subdevs = ARRAY_SIZE(nvtec_devs),
    .subdevs = nvtec_devs,
    .request_pin = EC_REQUEST_GPIO,
    .pwr_state_pin = EC_PWR_STATE_GPIO,
    .ap_wake_pin = AP_WAKE_GPIO,
    .ec_wake_pin = TS_PINS_NULL,
};

static struct i2c_board_info __initdata tostab12AL_ec[] = {
    {
        I2C_BOARD_INFO("nvtec", 0x1b),
        .irq         = TEGRA_GPIO_TO_IRQ(AP_WAKE_GPIO),
        .platform_data = &nvtec_pdata,
    },
};

int __init tostab12AL_ec_init(void)
{
    gpio_request(EC_REQUEST_GPIO, "ec_request");
    gpio_direction_output(EC_REQUEST_GPIO, 1);
    tegra_gpio_enable(EC_REQUEST_GPIO);

    gpio_request(EC_PWR_STATE_GPIO, "ec_pwr_state");
    gpio_direction_output(EC_PWR_STATE_GPIO, 1);
    tegra_gpio_enable(EC_PWR_STATE_GPIO);

    gpio_request(AP_WAKE_GPIO, "ap_wake");
    gpio_direction_input(AP_WAKE_GPIO);
    tegra_gpio_enable(AP_WAKE_GPIO);

    gpio_request(AP_ACOK_GPIO, "ac_in");
    gpio_direction_input(AP_ACOK_GPIO);
    tegra_gpio_enable(AP_ACOK_GPIO);

    gpio_request(MCU_RST_GPIO, "ec_rst");
    gpio_direction_output(MCU_RST_GPIO, 0);
    tegra_gpio_enable(MCU_RST_GPIO);

    i2c_register_board_info(0, tostab12AL_ec, ARRAY_SIZE(tostab12AL_ec));

    return 0;
}
