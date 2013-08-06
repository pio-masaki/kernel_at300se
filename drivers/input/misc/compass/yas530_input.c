/*
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

/**
  *  @addtogroup COMPASSDL
  *
  *  @{
  *      @file   yas530_input.c
  *      @brief  Compass setup and handling methods for Yamaha YAS530
  *              using sysfs and input subsystem
  */

/* -------------------------------------------------------------------------- */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>

#include <linux/mpu.h>

/**
 *  struct inv_compass_state - Driver state variables.
 *  @dev:	Represents read-only node for accessing buffered data.
 *  @idev:	Handle to input device.
 *  @sl_handle:	Handle to I2C port.
 */
struct inv_compass_state {
	struct i2c_client *i2c;
	atomic_t enable;
	atomic_t delay;
	struct mpu_platform_data plat_data;
	short i2c_addr;
	void *sl_handle;
	struct device *inv_dev;
	struct input_dev *idev;
	struct delayed_work work;

	struct mutex value_mutex;
	struct mutex enable_mutex;
	short value[3];
};

/* -------------------------------------------------------------------------- */
#define YAS530_REGADDR_DEVICE_ID          (0x80)
#define YAS530_REGADDR_ACTUATE_INIT_COIL  (0x81)
#define YAS530_REGADDR_MEASURE_COMMAND    (0x82)
#define YAS530_REGADDR_CONFIG             (0x83)
#define YAS530_REGADDR_MEASURE_INTERVAL   (0x84)
#define YAS530_REGADDR_OFFSET_X           (0x85)
#define YAS530_REGADDR_OFFSET_Y1          (0x86)
#define YAS530_REGADDR_OFFSET_Y2          (0x87)
#define YAS530_REGADDR_TEST1              (0x88)
#define YAS530_REGADDR_TEST2              (0x89)
#define YAS530_REGADDR_CAL                (0x90)
#define YAS530_REGADDR_MEASURE_DATA       (0xb0)

#define YAS530_MAX_DELAY                  (100)
#define YAS530_MIN_DELAY                  (10)
/* -------------------------------------------------------------------------- */
static int Cx, Cy1, Cy2;
static int /*a1, */ a2, a3, a4, a5, a6, a7, a8, a9;
static int k;

static unsigned char dx, dy1, dy2;
static unsigned char d2, d3, d4, d5, d6, d7, d8, d9, d0;
static unsigned char dck;

/* -------------------------------------------------------------------------- */
/**
 *  inv_serial_read() - Read one or more bytes from the device registers.
 *  @st:	Device driver instance.
 *  @reg:	First device register to be read from.
 *  @length:	Number of bytes to read.
 *  @data:	Data read from device.
 *  NOTE: The slave register will not increment when reading from the FIFO.
 */
int inv_serial_read(struct inv_compass_state *st,
	unsigned char reg, unsigned short length, unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (!data || !st->sl_handle)
		return -EINVAL;

	msgs[0].addr = st->i2c_addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;

	msgs[1].addr = st->i2c_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = length;

	pr_debug("%s RD%02X%02X%02X\n",
		 st->idev->name, st->i2c_addr, reg, length);
	res = i2c_transfer(st->sl_handle, msgs, 2);
	if (res < 2) {
		if (res >= 0)
			res = -EIO;
		return res;
	} else
		return 0;
}

/**
 *  inv_serial_single_write() - Write a byte to a device register.
 *  @st:	Device driver instance.
 *  @reg:	Device register to be written to.
 *  @data:	Byte to write to device.
 */
int inv_serial_single_write(struct inv_compass_state *st,
	unsigned char reg, unsigned char data)
{
	unsigned char tmp[2];
	struct i2c_msg msg;
	int res;

	if (!st->sl_handle)
		return -EINVAL;

	tmp[0] = reg;
	tmp[1] = data;

	msg.addr = st->i2c_addr;
	msg.flags = 0;	/* write */
	msg.buf = tmp;
	msg.len = 2;

	pr_debug("%s WS%02X%02X%02X\n",
		 st->idev->name, st->i2c_addr, reg, data);
	res = i2c_transfer(st->sl_handle, &msg, 1);
	if (res < 1) {
		if (res == 0)
			res = -EIO;
		return res;
	} else
		return 0;
}
static int set_hardware_offset(struct inv_compass_state *st,
			       char offset_x, char offset_y1, char offset_y2)
{
	char data;
	int result = 0;

	data = offset_x & 0x3f;
	result = inv_serial_single_write(st, YAS530_REGADDR_OFFSET_X, data);
	if (result)
		return result;

	data = offset_y1 & 0x3f;
	result = inv_serial_single_write(st, YAS530_REGADDR_OFFSET_Y1, data);
	if (result)
		return result;

	data = offset_y2 & 0x3f;
	result = inv_serial_single_write(st, YAS530_REGADDR_OFFSET_Y2, data);
	return result;
}

static int set_measure_command(struct inv_compass_state *st)
{
	int result = 0;
	result = inv_serial_single_write(st,
					 YAS530_REGADDR_MEASURE_COMMAND, 0x01);
	return result;
}

static int measure_normal(struct inv_compass_state *st,
			  int *busy, unsigned short *t,
			  unsigned short *x, unsigned short *y1,
			  unsigned short *y2)
{
	unsigned char data[8];
	unsigned short b, to, xo, y1o, y2o;
	int result;
	ktime_t sleeptime;
	result = set_measure_command(st);
	sleeptime = ktime_set(0, 2 * NSEC_PER_MSEC);
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_hrtimeout(&sleeptime, HRTIMER_MODE_REL);

	result = inv_serial_read(st,
				 YAS530_REGADDR_MEASURE_DATA, 8, data);
	if (result)
		return result;

	b = (data[0] >> 7) & 0x01;
	to = ((data[0] << 2) & 0x1fc) | ((data[1] >> 6) & 0x03);
	xo = ((data[2] << 5) & 0xfe0) | ((data[3] >> 3) & 0x1f);
	y1o = ((data[4] << 5) & 0xfe0) | ((data[5] >> 3) & 0x1f);
	y2o = ((data[6] << 5) & 0xfe0) | ((data[7] >> 3) & 0x1f);

	*busy = b;
	*t = to;
	*x = xo;
	*y1 = y1o;
	*y2 = y2o;

	return result;
}

static int measure_int(struct inv_compass_state *st,
			  int *busy, unsigned short *t,
			  unsigned short *x, unsigned short *y1,
			  unsigned short *y2)
{
	unsigned char data[8];
	unsigned short b, to, xo, y1o, y2o;
	int result;
	result = inv_serial_read(st,
				 YAS530_REGADDR_MEASURE_DATA, 8, data);
	if (result)
		return result;

	b = (data[0] >> 7) & 0x01;
	to = ((data[0] << 2) & 0x1fc) | ((data[1] >> 6) & 0x03);
	xo = ((data[2] << 5) & 0xfe0) | ((data[3] >> 3) & 0x1f);
	y1o = ((data[4] << 5) & 0xfe0) | ((data[5] >> 3) & 0x1f);
	y2o = ((data[6] << 5) & 0xfe0) | ((data[7] >> 3) & 0x1f);

	*busy = b;
	*t = to;
	*x = xo;
	*y1 = y1o;
	*y2 = y2o;

	result = set_measure_command(st);
	return result;
}

static int check_offset(struct inv_compass_state *st,
			char offset_x, char offset_y1, char offset_y2,
			int *flag_x, int *flag_y1, int *flag_y2)
{
	int result;
	int busy;
	short t, x, y1, y2;

	result = set_hardware_offset(st, offset_x, offset_y1, offset_y2);
	if (result)
		return result;
	result = measure_normal(st, &busy, &t, &x, &y1, &y2);
	if (result)
		return result;
	*flag_x = 0;
	*flag_y1 = 0;
	*flag_y2 = 0;

	if (x > 2048)
		*flag_x = 1;
	if (y1 > 2048)
		*flag_y1 = 1;
	if (y2 > 2048)
		*flag_y2 = 1;
	if (x < 2048)
		*flag_x = -1;
	if (y1 < 2048)
		*flag_y1 = -1;
	if (y2 < 2048)
		*flag_y2 = -1;

	return result;
}

static int measure_and_set_offset(struct inv_compass_state *st,
				  char *offset)
{
	int i;
	int result = 0;
	char offset_x = 0, offset_y1 = 0, offset_y2 = 0;
	int flag_x = 0, flag_y1 = 0, flag_y2 = 0;
	static const int correct[5] = { 16, 8, 4, 2, 1 };

	for (i = 0; i < 5; i++) {
		result = check_offset(st,
				      offset_x, offset_y1, offset_y2,
				      &flag_x, &flag_y1, &flag_y2);
		if (result)
			return result;
		if (flag_x)
			offset_x += flag_x * correct[i];
		if (flag_y1)
			offset_y1 += flag_y1 * correct[i];
		if (flag_y2)
			offset_y2 += flag_y2 * correct[i];
	}

	result = set_hardware_offset(st, offset_x, offset_y1, offset_y2);
	if (result)
		return result;
	offset[0] = offset_x;
	offset[1] = offset_y1;
	offset[2] = offset_y2;

	return result;
}

static void coordinate_conversion(short x, short y1, short y2, short t,
				  int32_t *xo, int32_t *yo, int32_t *zo)
{
	int32_t sx, sy1, sy2, sy, sz;
	int32_t hx, hy, hz;

	sx = x - (Cx * t) / 100;
	sy1 = y1 - (Cy1 * t) / 100;
	sy2 = y2 - (Cy2 * t) / 100;

	sy = sy1 - sy2;
	sz = -sy1 - sy2;

	hx = k * ((100 * sx + a2 * sy + a3 * sz) / 10);
	hy = k * ((a4 * sx + a5 * sy + a6 * sz) / 10);
	hz = k * ((a7 * sx + a8 * sy + a9 * sz) / 10);

	*xo = hx;
	*yo = hy;
	*zo = hz;
}

static int yas530_resume(struct inv_compass_state *st)
{
	int result = 0;

	unsigned char dummyData = 0x00;
	char offset[3] = { 0, 0, 0 };
	unsigned char data[16];
	unsigned char read_reg[1];

	/* =============================================== */

	/* Step 1 - Test register initialization */
	dummyData = 0x00;
	result = inv_serial_single_write(st,
					 YAS530_REGADDR_TEST1, dummyData);
	if (result)
		return result;
	result =
	    inv_serial_single_write(st,
				    YAS530_REGADDR_TEST2, dummyData);
	if (result)
		return result;
	/* Device ID read  */
	result = inv_serial_read(st,
				 YAS530_REGADDR_DEVICE_ID, 1, read_reg);

	/*Step 2 Read the CAL register */
	/* CAL data read */
	result = inv_serial_read(st,
				 YAS530_REGADDR_CAL, 16, data);
	if (result)
		return result;
	/* CAL data Second Read */
	result = inv_serial_read(st,
				 YAS530_REGADDR_CAL, 16, data);
	if (result)
		return result;
	/*Cal data */
	dx = data[0];
	dy1 = data[1];
	dy2 = data[2];
	d2 = (data[3] >> 2) & 0x03f;
	d3 = ((data[3] << 2) & 0x0c) | ((data[4] >> 6) & 0x03);
	d4 = data[4] & 0x3f;
	d5 = (data[5] >> 2) & 0x3f;
	d6 = ((data[5] << 4) & 0x30) | ((data[6] >> 4) & 0x0f);
	d7 = ((data[6] << 3) & 0x78) | ((data[7] >> 5) & 0x07);
	d8 = ((data[7] << 1) & 0x3e) | ((data[8] >> 7) & 0x01);
	d9 = ((data[8] << 1) & 0xfe) | ((data[9] >> 7) & 0x01);
	d0 = (data[9] >> 2) & 0x1f;
	dck = ((data[9] << 1) & 0x06) | ((data[10] >> 7) & 0x01);

	/*Correction Data */
	Cx = (int)dx * 6 - 768;
	Cy1 = (int)dy1 * 6 - 768;
	Cy2 = (int)dy2 * 6 - 768;
	a2 = (int)d2 - 32;
	a3 = (int)d3 - 8;
	a4 = (int)d4 - 32;
	a5 = (int)d5 + 38;
	a6 = (int)d6 - 32;
	a7 = (int)d7 - 64;
	a8 = (int)d8 - 32;
	a9 = (int)d9;
	k = (int)d0 + 10;

	/*Obtain the [49:47] bits */
	dck &= 0x07;

	/*Step 3 : Storing the CONFIG with the CLK value */
	dummyData = 0x00 | (dck << 2);
	result = inv_serial_single_write(st,
					 YAS530_REGADDR_CONFIG, dummyData);
	if (result)
		return result;
	/*Step 4 : Set Acquisition Interval Register */
	dummyData = 0x00;
	result = inv_serial_single_write(st,
					 YAS530_REGADDR_MEASURE_INTERVAL,
					 dummyData);
	if (result)
		return result;

	/*Step 5 : Reset Coil */
	dummyData = 0x00;
	result = inv_serial_single_write(st,
					 YAS530_REGADDR_ACTUATE_INIT_COIL,
					 dummyData);
	if (result)
		return result;
	/* Offset Measurement and Set */
	result = measure_and_set_offset(st, offset);
	if (result)
		return result;
	return result;
}

static int yas530_read(struct inv_compass_state *st, short rawfixed[3])
{
	int result = 0;

	int busy;
	short t, x, y1, y2;
	int32_t xyz[3];

	result = measure_int(st, &busy, &t, &x, &y1, &y2);
	if (result)
		return result;
	coordinate_conversion(x, y1, y2, t, &xyz[0], &xyz[1], &xyz[2]);

	rawfixed[0] = (short)(xyz[0] / 100);
	rawfixed[1] = (short)(xyz[1] / 100);
	rawfixed[2] = (short)(xyz[2] / 100);
	if (busy)
		return -1;
	return result;
}

static void yas530_work_func(struct work_struct *work)
{
	struct inv_compass_state *st =
		container_of((struct delayed_work *)work,
			struct inv_compass_state, work);
	unsigned long delay = msecs_to_jiffies(atomic_read(&st->delay));
	short c[3];
	c[0] = c[1] = c[2] = 0;
	if (0 == yas530_read(st, c)) {
		input_report_rel(st->idev, REL_X, c[0]);
		input_report_rel(st->idev, REL_Y, c[1]);
		input_report_rel(st->idev, REL_Z, c[2]);
		input_sync(st->idev);
	}

	mutex_lock(&st->value_mutex);
	st->value[0] = c[0];
	st->value[1] = c[1];
	st->value[2] = c[2];
	mutex_unlock(&st->value_mutex);
	schedule_delayed_work(&st->work, delay);
}

static ssize_t yas530_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_compass_state *st = dev_get_drvdata(dev);
	short c[3];

	mutex_lock(&st->value_mutex);
	c[0] = st->value[0];
	c[1] = st->value[1];
	c[2] = st->value[2];
	mutex_unlock(&st->value_mutex);
	return sprintf(buf, "%d, %d, %d\n", c[0], c[1], c[2]);
}

static ssize_t yas530_scale_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* The sensitivity is 0.1 uT/LSB for all axes based on the conversion
	   performed from X, Y1, Y1 to X, Y, Z coordinates and adjustments.
	   The scale is the 0.1 * 2^15 = 3276.8 uT
	   The scale value is represented in q15 format, therefore multipled
	   by 2^15 */
	return sprintf(buf, "%ld\n", 107374182L);
}

static ssize_t yas530_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_compass_state *st = dev_get_drvdata(dev);
	int result;
	result = yas530_resume(st);
	return sprintf(buf, "%d\n", result);
}

static ssize_t yas530_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_compass_state *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", atomic_read(&st->enable));
}

static ssize_t yas530_rate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_compass_state *st = dev_get_drvdata(dev);
	/* transform delay in ms to rate */
	return sprintf(buf, "%d\n", 1000 / atomic_read(&st->delay));
}

static ssize_t yas530_matrix_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_compass_state *st = dev_get_drvdata(dev);
	signed char *m;
	m = st->plat_data.orientation;
	return sprintf(buf,
		"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}

static ssize_t yas530_rate_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct inv_compass_state *st = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	/* transform rate to delay in ms */
	data = 1000 / data;
	if (data > YAS530_MAX_DELAY)
		data = YAS530_MAX_DELAY;
	if (data < YAS530_MIN_DELAY)
		data = YAS530_MIN_DELAY;
	atomic_set(&st->delay, (unsigned int) data);
	return count;
}

static void yas530_set_enable(struct device *dev, int enable)
{
	struct inv_compass_state *st = dev_get_drvdata(dev);
	int pre_enable = atomic_read(&st->enable);

	mutex_lock(&st->enable_mutex);
	if (enable) {
		if (pre_enable == 0) {
			schedule_delayed_work(&st->work,
				msecs_to_jiffies(atomic_read(&st->delay)));
			atomic_set(&st->enable, 1);
		}

	} else {
		if (pre_enable == 1) {
			cancel_delayed_work_sync(&st->work);
			atomic_set(&st->enable, 0);
		}
	}
	mutex_unlock(&st->enable_mutex);
}

static ssize_t yas530_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if ((data == 0) || (data == 1))
		yas530_set_enable(dev, data);

	return count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		yas530_enable_show, yas530_enable_store);
static DEVICE_ATTR(value, S_IRUGO, yas530_value_show, NULL);
static DEVICE_ATTR(scale, S_IRUGO, yas530_scale_show, NULL);
static DEVICE_ATTR(reset, S_IRUGO, yas530_reset_show, NULL);
static DEVICE_ATTR(compass_matrix, S_IRUGO, yas530_matrix_show, NULL);
static DEVICE_ATTR(rate, S_IRUGO | S_IWUSR, yas530_rate_show,
		yas530_rate_store);

static struct attribute *yas530_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_value.attr,
	&dev_attr_scale.attr,
	&dev_attr_reset.attr,
	&dev_attr_rate.attr,
	&dev_attr_compass_matrix.attr,
	NULL
};

static struct attribute_group yas530_attribute_group = {
	.name = "yas530",
	.attrs = yas530_attributes
};

/**
 *  inv_setup_input() - internal setup input device.
 *  @st:	Device driver instance.
 *  @**idev_in  pointer to input device
 *  @*client    i2c client
 *  @*name      name of the input device.
 */
static int inv_setup_input(struct inv_compass_state *st,
	struct input_dev **idev_in, struct i2c_client *client,
	unsigned char *name) {
	int result;
	struct input_dev *idev;
	idev = input_allocate_device();
	if (!idev) {
		result = -ENOMEM;
		return result;
	}
	/* Setup input device. */
	idev->name = name;

	idev->id.bustype = BUS_I2C;
	idev->id.product = 'S';
	idev->id.vendor  = ('I' << 8) | 'S';
	idev->id.version = 1;
	idev->dev.parent = &client->dev;
	/* Open and close method. */
	idev->open = NULL;
	idev->close = NULL;

	__set_bit(EV_REL, idev->evbit);
	input_set_capability(idev, EV_REL, REL_X);
	input_set_capability(idev, EV_REL, REL_Y);
	input_set_capability(idev, EV_REL, REL_Z);

	input_set_capability(idev, EV_REL, REL_MISC);
	input_set_capability(idev, EV_REL, REL_WHEEL);

	input_set_drvdata(idev, st);
	result = input_register_device(idev);
	if (result)
		input_free_device(idev);

	*idev_in = idev;
	return result;
}
static unsigned short normal_i2c[] = { I2C_CLIENT_END };

static int yas530_mod_probe(struct i2c_client *client,
			   const struct i2c_device_id *devid)
{
	struct mpu_platform_data *pdata;
	struct inv_compass_state *st;
	struct input_dev *idev;
	int result = 0;

	dev_info(&client->adapter->dev, "%s: %s\n", __func__, devid->name);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		goto out_no_free;
	}

	pdata = (struct mpu_platform_data *)dev_get_platdata(&client->dev);
	if (!pdata) {
		dev_err(&client->adapter->dev,
			"Missing platform data for slave %s\n", devid->name);
		result = -EFAULT;
		goto out_no_free;
	}

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st) {
		result = -ENOMEM;
		goto out_no_free;
	}

	i2c_set_clientdata(client, st);
	st->i2c = client;
	mutex_init(&st->value_mutex);
	mutex_init(&st->enable_mutex);
	atomic_set(&st->delay, 100);
	st->sl_handle = client->adapter;
	st->plat_data = *pdata;
	st->i2c_addr = client->addr;
	INIT_DELAYED_WORK(&st->work, yas530_work_func);
	result = inv_setup_input(st, &idev, client, "INV_YAS530");
	if (result)
		goto out_free_memory;
	st->idev = idev;
	result = sysfs_create_group(&st->idev->dev.kobj,
				    &yas530_attribute_group);
	if (result < 0)
		goto error_sysfs;

	yas530_resume(st);

	return result;
error_sysfs:
	input_unregister_device(st->idev);
out_free_memory:
	kfree(st);
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	return result;

}

static int yas530_mod_remove(struct i2c_client *client)
{
	struct inv_compass_state *st =
		i2c_get_clientdata(client);

	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	yas530_set_enable(&st->idev->dev, 0);
	sysfs_remove_group(&st->idev->dev.kobj, &yas530_attribute_group);
	input_unregister_device(st->idev);
	kfree(st);
	return 0;
}

static const struct i2c_device_id yas530_mod_id[] = {
	{ "yas530", COMPASS_ID_YAS530 },
	{}
};

MODULE_DEVICE_TABLE(i2c, yas530_mod_id);

static struct i2c_driver yas530_mod_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = yas530_mod_probe,
	.remove = yas530_mod_remove,
	.id_table = yas530_mod_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "yas530_mod",
		   },
	.address_list = normal_i2c,
};

static int __init yas530_mod_init(void)
{
	int res = i2c_add_driver(&yas530_mod_driver);
	pr_info("%s: Probe name %s\n", __func__, "yas530_mod");
	if (res)
		pr_err("%s failed\n", __func__);
	return res;
}

static void __exit yas530_mod_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&yas530_mod_driver);
}

module_init(yas530_mod_init);
module_exit(yas530_mod_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Driver for YAS530 sensor with input subsystem");
MODULE_LICENSE("GPL");
MODULE_ALIAS("yas530_mod");

/**
 *  @}
 */
