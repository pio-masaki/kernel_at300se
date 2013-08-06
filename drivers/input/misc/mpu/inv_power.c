#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include "inv_power.h"

static struct regulator *mpu_sensor_1v8_reg = NULL;
static struct regulator *mpu_sensor_3v3_reg = NULL;

int inv_power_control(bool enable) {
	if (enable) {
		mpu_sensor_3v3_reg = regulator_get(NULL, REG_3V3_NAME);
		if (IS_ERR_OR_NULL(mpu_sensor_3v3_reg)) {
			pr_err("%s: couldn't get regulator vdd_3v3_sensor\n", __func__);
			return -ENODEV;
		}
		regulator_enable(mpu_sensor_3v3_reg);
	    
		mpu_sensor_1v8_reg = regulator_get(NULL, REG_1V8_NAME);
		if (IS_ERR_OR_NULL(mpu_sensor_1v8_reg)) {
			pr_err("%s: couldn't get regulator vdd_1v8_sensor\n", __func__);
			return -ENODEV;
		}
		regulator_enable(mpu_sensor_1v8_reg);
		printk("%s: regulator 1v8 and 3v3 enabled\n", __func__);
	} else {
		if (mpu_sensor_1v8_reg != NULL) {
			regulator_put(mpu_sensor_1v8_reg);
			mpu_sensor_1v8_reg = NULL;
		}
		if (mpu_sensor_3v3_reg != NULL) {
			regulator_put(mpu_sensor_3v3_reg);
			mpu_sensor_3v3_reg = NULL;
		}
		printk("%s: regulator 1v8 and 3v3 disabled\n", __func__);
	}
	return 0;
}
EXPORT_SYMBOL(inv_power_control);
