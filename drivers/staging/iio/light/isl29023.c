/*
 * A iio driver for the light sensor ISL 29023.
 *
 * Hwmon driver for monitoring ambient light intensity in luxi, proximity
 * sensing and infrared sensing.
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/earlysuspend.h>
#include "../iio.h"
#include <linux/isl29023.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <linux/poll.h>
#include <linux/miscdevice.h> /* for misc register */

#define DEVICE_NAME         "isl29023"

#define CONVERSION_TIME_MS		100
#define CONVERSION_TIME_MICRO_S		400

#define ISL29023_REG_ADD_COMMAND1	0x00
#define COMMMAND1_OPMODE_SHIFT		5
#define COMMMAND1_OPMODE_MASK		(7 << COMMMAND1_OPMODE_SHIFT)
#define COMMMAND1_OPMODE_POWER_DOWN	0
#define COMMMAND1_OPMODE_ALS_ONCE	1
#define COMMMAND1_OPMODE_IR_ONCE	2
#define COMMMAND1_OPMODE_PROX_ONCE	3

#define ISL29023_REG_ADD_COMMANDII	0x01

#define COMMANDII_RESOLUTION_SHIFT	2
#define COMMANDII_RESOLUTION_MASK	(0x3 << COMMANDII_RESOLUTION_SHIFT)

#define COMMANDII_RANGE_SHIFT		0
#define COMMANDII_RANGE_MASK		(0x3 << COMMANDII_RANGE_SHIFT)

#define COMMANDII_SCHEME_SHIFT		7
#define COMMANDII_SCHEME_MASK		(0x1 << COMMANDII_SCHEME_SHIFT)

#define ISL29023_REG_ADD_DATA_LSB	0x02
#define ISL29023_REG_ADD_DATA_MSB	0x03
#define ISL29023_MAX_REGS		0x09 //ISL29023_REG_ADD_DATA_MSB
#define ISL29023_DEFAULT_RANGE		64000 
#define ISL29023_DEFAULT_RESOLUTION	16
#define ENABLE_DYNAMIC_RANGE		1
#define ISL29023_REG_ADD_INT_LT_LSB	0x04
#define ISL29023_REG_ADD_INT_LT_MSB	0x05
#define ISL29023_REG_ADD_INT_HT_LSB	0x06
#define ISL29023_REG_ADD_INT_HT_MSB	0x07
#define ISL29023_REG_ADD_TEST		0x08

struct isl29023_chip {
	struct iio_dev		*indio_dev;
	struct i2c_client	*client;
	struct early_suspend early_suspend;
	struct mutex		lock;
        u8                      minor;
        u8                      dev_open_cnt;
	unsigned int		range;
	unsigned int		adc_bit;
	int			prox_scheme;
	u8			reg_cache[ISL29023_MAX_REGS];
	struct lightsensor_platform_data *pdata;
	unsigned int            enable_ls;
        int                     event_ready;
        u64                     next_timer;
};
//static int isl29023_suspend(struct i2c_client *client);
//static int isl29023_resume(struct i2c_client *client);
static void isl29023_early_suspend(struct early_suspend *handler);
static void isl29023_early_resume(struct early_suspend *handler);

static struct isl29023_chip * isl29023_data;

static bool isl29023_write_data(struct i2c_client *client, u8 reg,
	u8 val, u8 mask, u8 shift)
{
	u8 regval;
	int ret = 0;
	struct isl29023_chip *chip = i2c_get_clientdata(client);

	regval = chip->reg_cache[reg];
	regval &= ~mask;
	regval |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, regval);
	if (ret) {
		dev_err(&client->dev, "Write to device fails status %x\n", ret);
		return false;
	}
	chip->reg_cache[reg] = regval;
	return true;
}

static bool isl29023_set_range(struct i2c_client *client, unsigned long range,
		unsigned int *new_range)
{
	unsigned long supp_ranges[] = {1000, 4000, 16000, 64000};
	int i;

	for (i = 0; i < (ARRAY_SIZE(supp_ranges) -1); ++i) {
		if (range <= supp_ranges[i])
			break;
	}
	*new_range = (unsigned int)supp_ranges[i];

	return isl29023_write_data(client, ISL29023_REG_ADD_COMMANDII,
		i, COMMANDII_RANGE_MASK, COMMANDII_RANGE_SHIFT);
}

static bool isl29023_set_resolution(struct i2c_client *client,
			unsigned long adcbit, unsigned int *conf_adc_bit)
{
	unsigned long supp_adcbit[] = {16, 12, 8, 4};
	int i;

	for (i = 0; i < (ARRAY_SIZE(supp_adcbit)); ++i) {
		if (adcbit == supp_adcbit[i])
			break;
	}
	*conf_adc_bit = (unsigned int)supp_adcbit[i];

	return isl29023_write_data(client, ISL29023_REG_ADD_COMMANDII,
		i, COMMANDII_RESOLUTION_MASK, COMMANDII_RESOLUTION_SHIFT);
}

//mills
static void check_proper_range(struct isl29023_chip * chip, int value)
{
    int supp_ranges[] = {1000, 4000, 16000, 64000};
    int k;
    bool status;
    unsigned int new_range;
    unsigned int new_result;
  
    value += value/5;
    if ( value > 64000 )
    {
      k = (ARRAY_SIZE(supp_ranges) -1);
    }
    else
    {
      for (k = 0; k < (ARRAY_SIZE(supp_ranges) -1); ++k)
      {
        if (value <= supp_ranges[k])
                    break;
      }
      if ( k >= (ARRAY_SIZE(supp_ranges) -1) )
        k = (ARRAY_SIZE(supp_ranges) -1);
    }
    new_range = (unsigned int)supp_ranges[k];
    if ( chip -> range != new_range )
    {
      // change to new range
        //mutex_lock(&chip->lock);
        status = isl29023_set_range(chip->client, new_range, &new_result);
        if (!status)
        {
            //mutex_unlock(&chip->lock);
//            dev_err(dev, "Error in setting max range\n");
            return;
        }
        chip->range = new_range;
        //mutex_unlock(&chip->lock);
    }
}

static int isl29023_read_sensor_input(struct i2c_client *client, int mode)
{
	bool status;
	int lsb;
	int msb;

	/* Set mode */
	status = isl29023_write_data(client, ISL29023_REG_ADD_COMMAND1,
			mode, COMMMAND1_OPMODE_MASK, COMMMAND1_OPMODE_SHIFT);
	if (!status) {
		dev_err(&client->dev, "Error in setting operating mode\n");
		return -EBUSY;
	}

	msleep(CONVERSION_TIME_MS);
//        usleep(CONVERSION_TIME_MICRO_S); // Jackal add

	lsb = i2c_smbus_read_byte_data(client, ISL29023_REG_ADD_DATA_LSB);
	if (lsb < 0) {
		dev_err(&client->dev, "Error in reading LSB DATA\n");
		return lsb;
	}

        msb = i2c_smbus_read_byte_data(client, ISL29023_REG_ADD_DATA_MSB);
	if (msb < 0) {
		dev_err(&client->dev, "Error in reading MSB DATA\n");
		return msb;
	}

	dev_vdbg(&client->dev, "MSB 0x%x and LSB 0x%x\n", msb, lsb);

	return ((msb << 8) | lsb);
}

// get the state of mt9d115 indicating led, 0-OFF, ON otherwise
// defined in board_ast_sensors.c
extern int ast_mt9d115_led_get_state ( void ); 

static bool isl29023_read_lux(struct i2c_client *client, int *lux, int *lux2)
{
	int lux_data;
	int lux_data_remaing;
	struct isl29023_chip *chip = i2c_get_clientdata(client);

	lux_data = isl29023_read_sensor_input(client, COMMMAND1_OPMODE_ALS_ONCE);
	if (lux_data > 0) {
		*lux = (lux_data * chip->range) >> chip->adc_bit;
		check_proper_range(chip, *lux);
		lux_data_remaing=(((lux_data * chip->range) & 0xffff)*10) >> chip->adc_bit;
		*lux2 = lux_data_remaing>9?9:lux_data_remaing;
#if 0                
                if ( *lux < 1 )
                {
                    if ( *lux2 <= 5 )
                    {
                        if ( ast_mt9d115_led_get_state () > 0 )
                            *lux2 = 0;
                    }

                }
#endif                
                chip->next_timer = get_jiffies_64() + CONVERSION_TIME_MS * HZ / 1000;
                chip->event_ready = 1;
		return true;
	}
	return false;
}

static bool isl29023_read_ir(struct i2c_client *client, int *ir)
{
	int ir_data;

	ir_data = isl29023_read_sensor_input(client, COMMMAND1_OPMODE_IR_ONCE);
	if (ir_data > 0) {
		*ir = ir_data;
		return true;
	}
	return false;
}

static bool isl29023_read_proximity_ir(struct i2c_client *client, int scheme,
	int *near_ir)
{
	bool status;
	int prox_data = -1;
	int ir_data = -1;

	/* Do proximity sensing with required scheme */
	status = isl29023_write_data(client, ISL29023_REG_ADD_COMMANDII,
			scheme, COMMANDII_SCHEME_MASK, COMMANDII_SCHEME_SHIFT);
	if (!status) {
		dev_err(&client->dev, "Error in setting operating mode\n");
		return false;
	}

	prox_data = isl29023_read_sensor_input(client,
		COMMMAND1_OPMODE_PROX_ONCE);
	if (scheme == 1) {
		if (prox_data >= 0) {
			*near_ir = prox_data;
			return true;
		}
		return false;
	}

	if (prox_data >= 0)
		ir_data = isl29023_read_sensor_input(client,
				COMMMAND1_OPMODE_IR_ONCE);

	if (prox_data >= 0 && ir_data >= 0) {
		if (prox_data >= ir_data) {
			*near_ir = prox_data - ir_data;
			return true;
		}
	}

	return false;
}


static ssize_t get_sensor_data(struct device *dev, char *buf, int mode)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29023_chip *chip = indio_dev->dev_data;
	struct i2c_client *client = chip->client;
	int value = 0;
	int value2 = 0;
	bool status;

	mutex_lock(&chip->lock);
	switch (mode) {
		case COMMMAND1_OPMODE_PROX_ONCE:
			status = isl29023_read_proximity_ir(client,
				chip->prox_scheme, &value);
			break;

		case COMMMAND1_OPMODE_ALS_ONCE:
			status = isl29023_read_lux(client, &value, &value2);
			break;

		case COMMMAND1_OPMODE_IR_ONCE:
			status = isl29023_read_ir(client, &value);
			break;

		default:
			dev_err(&client->dev,"Mode %d is not supported\n",mode);
			mutex_unlock(&chip->lock);
			return -EBUSY;
	}

	if (!status) {
		dev_err(&client->dev, "Error in Reading data");
		mutex_unlock(&chip->lock);
		return -EBUSY;
	}

	mutex_unlock(&chip->lock);
	if (mode == COMMMAND1_OPMODE_ALS_ONCE)
		return sprintf(buf, "%d.%01d\n", value, value2);
	else
		return sprintf(buf, "%d\n", value);
}

/* Sysfs interface */
/* range */
static ssize_t show_range(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29023_chip *chip = indio_dev->dev_data;

	dev_vdbg(dev, "%s()\n", __func__);
	return sprintf(buf, "%u\n", chip->range);
}

static ssize_t store_range(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29023_chip *chip = indio_dev->dev_data;
	struct i2c_client *client = chip->client;
	bool status;
	unsigned long lval;
	unsigned int new_range;

	dev_vdbg(dev, "%s()\n", __func__);

	if (strict_strtoul(buf, 10, &lval))
		return -EINVAL;

	if (!(lval == 1000UL || lval == 4000UL ||
		lval == 16000UL || lval == 64000UL)) {
		dev_err(dev, "The range is not supported\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	status = isl29023_set_range(client, lval, &new_range);
	if (!status) {
		mutex_unlock(&chip->lock);
		dev_err(dev, "Error in setting max range\n");
		return -EINVAL;
	}
	chip->range = new_range;
	mutex_unlock(&chip->lock);
	return count;
}

/* resolution */
static ssize_t show_resolution(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29023_chip *chip = indio_dev->dev_data;

	dev_vdbg(dev, "%s()\n", __func__);
	return sprintf(buf, "%u\n", chip->adc_bit);
}

static ssize_t store_resolution(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29023_chip *chip = indio_dev->dev_data;
	struct i2c_client *client = chip->client;
	bool status;
	unsigned long lval;
	unsigned int new_adc_bit;

	dev_vdbg(dev, "%s()\n", __func__);

	if (strict_strtoul(buf, 10, &lval))
		return -EINVAL;
	if (!(lval == 4 || lval == 8 || lval == 12 || lval == 16)) {
		dev_err(dev, "The resolution is not supported\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	status = isl29023_set_resolution(client, lval, &new_adc_bit);
	if (!status) {
		mutex_unlock(&chip->lock);
		dev_err(dev, "Error in setting resolution\n");
		return -EINVAL;
	}
	chip->adc_bit = new_adc_bit;
	mutex_unlock(&chip->lock);
	return count;
}

/* proximity scheme */
static ssize_t show_prox_scheme(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29023_chip *chip = indio_dev->dev_data;

	dev_vdbg(dev, "%s()\n", __func__);
	return sprintf(buf, "%d\n", chip->prox_scheme);
}

static ssize_t store_prox_scheme(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29023_chip *chip = indio_dev->dev_data;
	unsigned long lval;

	dev_vdbg(dev, "%s()\n", __func__);

	if (strict_strtoul(buf, 10, &lval))
		return -EINVAL;
	if (!(lval == 0UL || lval == 1UL)) {
		dev_err(dev, "The scheme is not supported\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	chip->prox_scheme = (int)lval;
	mutex_unlock(&chip->lock);
	return count;
}

/* Read lux */
static ssize_t show_lux(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	return get_sensor_data(dev, buf, COMMMAND1_OPMODE_ALS_ONCE);
}

/* Read ir */
static ssize_t show_ir(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	return get_sensor_data(dev, buf, COMMMAND1_OPMODE_IR_ONCE);
}

/* Read nearest ir */
static ssize_t show_proxim_ir(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	return get_sensor_data(dev, buf, COMMMAND1_OPMODE_PROX_ONCE);
}

/* Read name */
static ssize_t show_name(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29023_chip *chip = indio_dev->dev_data;
	return sprintf(buf, "%s\n", chip->client->name);
}

/* enable lightsensor*/
static ssize_t show_enable_ls(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29023_chip *chip = indio_dev->dev_data;

	dev_vdbg(dev, "%s()\n", __func__);
	return sprintf(buf, "%d\n", chip->enable_ls);
}

static ssize_t store_enable_ls(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29023_chip *chip = indio_dev->dev_data;
	unsigned long lval;

	dev_vdbg(dev, "%s()\n", __func__);
	if (strict_strtoul(buf, 10, &lval))
		return -EINVAL;
	if (!(lval == 0UL || lval == 1UL)) {
		dev_err(dev, "The control state is not supported\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	chip->enable_ls = (int)lval;
	mutex_unlock(&chip->lock);

	if (chip->pdata && chip->pdata->power_on && chip->enable_ls)
		chip->pdata->power_on();

	if (chip->pdata && chip->pdata->power_off && !(chip->enable_ls))
		chip->pdata->power_off();

	return count;
}

static IIO_DEVICE_ATTR(range, S_IRUGO | S_IWUSR, show_range, store_range, 0);
static IIO_DEVICE_ATTR(resolution, S_IRUGO | S_IWUSR,
	show_resolution, store_resolution, 0);
static IIO_DEVICE_ATTR(proximity_scheme, S_IRUGO | S_IWUSR,
	show_prox_scheme, store_prox_scheme, 0);
static IIO_DEVICE_ATTR(lux, S_IRUGO, show_lux, NULL, 0);
static IIO_DEVICE_ATTR(ir, S_IRUGO, show_ir, NULL, 0);
static IIO_DEVICE_ATTR(proxim_ir, S_IRUGO, show_proxim_ir, NULL, 0);
static IIO_DEVICE_ATTR(name, S_IRUGO, show_name, NULL, 0);
static IIO_DEVICE_ATTR(enable_ls, S_IRUGO | S_IWUSR,
	show_enable_ls, store_enable_ls, 0);

static struct attribute *isl29023_attributes[] = {
	&iio_dev_attr_name.dev_attr.attr,
	&iio_dev_attr_range.dev_attr.attr,
	&iio_dev_attr_resolution.dev_attr.attr,
	&iio_dev_attr_proximity_scheme.dev_attr.attr,
	&iio_dev_attr_lux.dev_attr.attr,
	&iio_dev_attr_ir.dev_attr.attr,
	&iio_dev_attr_proxim_ir.dev_attr.attr,
	&iio_dev_attr_enable_ls.dev_attr.attr,
	NULL
};

static const struct attribute_group isl29108_group = {
	.attrs = isl29023_attributes,
};

static int isl29023_chip_init(struct i2c_client *client)
{
	struct isl29023_chip *chip = i2c_get_clientdata(client);
	bool status;
	int i;
	int new_adc_bit;
	unsigned int new_range;

	for (i = 0; i < ARRAY_SIZE(chip->reg_cache); i++) {
		chip->reg_cache[i] = 0;
	}

	/* power-on reset */
	isl29023_write_data(client, ISL29023_REG_ADD_TEST, 0x0, 0xff, 0);
	isl29023_write_data(client, ISL29023_REG_ADD_COMMAND1, 0x0, 0xff, 0);
	printk(KERN_INFO "[light] power-on reset\n");

	/* set defaults */
        isl29023_write_data(client, ISL29023_REG_ADD_INT_LT_LSB, 0x0, 0xff, 0);
        isl29023_write_data(client, ISL29023_REG_ADD_INT_LT_MSB, 0x0, 0xff, 0);
        isl29023_write_data(client, ISL29023_REG_ADD_INT_HT_LSB, 0xff, 0xff, 0);
        isl29023_write_data(client, ISL29023_REG_ADD_INT_HT_MSB, 0xff, 0xff, 0);

	status = isl29023_set_range(client, chip->range, &new_range);
	if (status)
		status = isl29023_set_resolution(client, chip->adc_bit,
				&new_adc_bit);
	if (!status) {
		dev_err(&client->dev, "Init of isl29023 fails\n");
		return -ENODEV;
	}
	return 0;
}

static int isl29023_open(struct inode *inode, struct file *filp)
{
    u8 i_minor;

    i_minor = MINOR(inode->i_rdev);

    /* check open file conut */
    mutex_lock(&isl29023_data->lock);
//    if ( i_minor != isl29018_data->minor) goto no_dev_err;
    if ( isl29023_data->dev_open_cnt > 0) goto has_open_err;
    isl29023_data->dev_open_cnt++;
    mutex_unlock(&isl29023_data->lock);

    filp->private_data = (void *)isl29023_data;

    return 0;

/*
no_dev_err:
    mutex_unlock(&isl29018_data->lock);
    return -ENODEV;
*/
has_open_err:
    mutex_unlock(&isl29023_data->lock);
    return -EMFILE;
}

static int isl29023_close(struct inode *inode, struct file *filp)
{
    u8 i_minor;
    struct isl29023_chip *p_data;

    p_data = (struct isl29023_chip *)filp->private_data;
    i_minor = MINOR(inode->i_rdev);

    mutex_lock(&(p_data->lock));
    if ( i_minor != p_data->minor) goto no_dev_err;
    if (p_data->dev_open_cnt > 0)
        p_data->dev_open_cnt--;
    else
    {
        /* power off the isl29023 */
        if (p_data->pdata && p_data->pdata->power_off)
                p_data->pdata->power_off();
    }
//    i2c_smbus_write_byte_data(isl29023_data.client, 0x00, 0x00);
    mutex_unlock(&(p_data->lock));

    return 0;

no_dev_err:
    mutex_unlock(&(p_data->lock));
    return -ENODEV;
}

static unsigned int isl29023_poll(struct file *file, struct poll_table_struct *poll)
{
    int mask = 0;
    struct isl29023_chip *chip;
    u64         jf64;

    chip = (struct isl29023_chip *)file->private_data;
//    jf64 = get_jiffies_64();
//    if ( jf64 < chip->next_timer )
//        goto err_not_yet;
//    if(chip->event_ready)
//    {
//        chip->event_ready = 0;
        mask |= POLLIN | POLLRDNORM;
//    }
//err_not_yet:
    return mask;
}

static struct file_operations isl29023_fops = {
    .owner = THIS_MODULE,
    .open = isl29023_open,
    .release = isl29023_close,
    .poll = isl29023_poll,
};

static struct miscdevice isl29023_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
    .fops = &isl29023_fops,
};

static const struct iio_info isl29023_info = {
        .attrs = &isl29108_group,
        .driver_module = THIS_MODULE,
};

static int __devinit isl29023_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct isl29023_chip *chip;
	int err;
        int lux_value = 0;
	int lux_value2 = 0;
        int ir_value = 0;
	bool lux_status,ir_status;

          printk(KERN_INFO "[light] isl29023 probe!\n");
	chip = kzalloc(sizeof (struct isl29023_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

        isl29023_data = chip;
	i2c_set_clientdata(client, chip);
	chip->pdata = client->dev.platform_data;
	chip->client = client;
	mutex_init(&chip->lock);

        chip->minor = 0;
        chip->dev_open_cnt = 0;
        chip->event_ready = 1;
        chip->next_timer = get_jiffies_64();
	chip->range = ISL29023_DEFAULT_RANGE;
	chip->adc_bit = ISL29023_DEFAULT_RESOLUTION;
	chip->enable_ls = 0;

	err = isl29023_chip_init(client);
	if (err)
		goto exit_free;

	chip->indio_dev = iio_allocate_device(0);
	if (!chip->indio_dev) {
		dev_err(&client->dev, "iio allocation fails\n");
		goto exit_free;
	}

        chip->indio_dev->info = &isl29023_info;
//	chip->indio_dev->attrs = &isl29108_group;
	chip->indio_dev->dev.parent = &client->dev;
	chip->indio_dev->dev_data = (void *)(chip);
//	chip->indio_dev->driver_module = THIS_MODULE;
	chip->indio_dev->modes = INDIO_DIRECT_MODE;
	err = iio_device_register(chip->indio_dev);
	if (err) {
		dev_err(&client->dev, "iio registration fails\n");
		goto exit_iio_free;
	}

	err = misc_register(&isl29023_device);
        if (err) {
                dev_err(&client->dev, "misc registration fails\n");
                goto exit_iio_free;
        }

        lux_status = isl29023_read_lux(chip->client, &lux_value, &lux_value2);
        printk(KERN_INFO "[light] isl29023 lux_status=%d,lux_value =%d!\n",lux_status,lux_value);

        ir_status = isl29023_read_ir(chip->client, &ir_value);
        printk(KERN_INFO "[light] isl29023 ir_status=%d,ir_value =%d!\n",ir_status,ir_value);

	chip->early_suspend.suspend = isl29023_early_suspend;
	chip->early_suspend.resume = isl29023_early_resume;
	register_early_suspend(&(chip->early_suspend));

	if (chip->pdata && chip->pdata->power_off)
		chip->pdata->power_off();
	return 0;

exit_iio_free:
        printk(KERN_INFO "[light] isl29023 iio error!\n");
	iio_free_device(chip->indio_dev);
exit_free:
        printk(KERN_INFO "[light] isl29023 exit free!\n");
	kfree(chip);
exit:
        printk(KERN_INFO "[light] isl29023 exit!\n");
	return err;
}

static int __devexit isl29023_remove(struct i2c_client *client)
{
	struct isl29023_chip *chip = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s()\n", __func__);
	iio_device_unregister(chip->indio_dev);
	kfree(chip);
	return 0;
}

static void isl29023_early_suspend(struct early_suspend *handler)
{
	struct isl29023_chip *chip =
		container_of((struct early_suspend *)handler,
		struct isl29023_chip, early_suspend);

	if (chip->pdata && chip->pdata->power_off)
		chip->pdata->power_off();

}
static void isl29023_early_resume(struct early_suspend *handler)
{
	struct isl29023_chip *chip =
		container_of((struct early_suspend *)handler,
		struct isl29023_chip, early_suspend);

	if (chip->pdata && chip->pdata->power_on && chip->enable_ls)
		chip->pdata->power_on();

}

static const struct i2c_device_id isl29023_id[] = {
	{"isl29023", 0},
	{}
};


MODULE_DEVICE_TABLE(i2c, isl29023_id);

static struct i2c_driver isl29023_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver  = {
		.name = "isl29023",
		.owner = THIS_MODULE,
	},
	.probe	 = isl29023_probe,
	.remove  = __devexit_p(isl29023_remove),
	.id_table = isl29023_id,
};

static int __init isl29023_init(void)
{
	return i2c_add_driver(&isl29023_driver);
}

static void __exit isl29023_exit(void)
{
	i2c_del_driver(&isl29023_driver);
}

module_init(isl29023_init);
module_exit(isl29023_exit);
