/*
 * kernel/drivers/media/video/tegra
 *
 * Samsung S5K5CAGA sensor driver
 *
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <media/s5k5caga.h>

#include "s5k5caga_reg.h"

#define MODULE_FOXLINK_I2C_ADDR         0x2D
#define MODULE_BISON_I2C_ADDR           0x3C
#define COMMAND_READ_ADDR_H             0x002C
#define COMMAND_READ_ADDR_L             0x002E
#define COMMAND_DATA_ADDR               0x0F12
#define CHIP_ID_ADDR                    0x00000040
#define CHIP_ID_VAL                     0x05CA
#define EXPOSURE_TIME_ADDR              0x700023E8
#define ISO_VALUE_ADDR                  0x700023EC
#define AF_STATUS_ADDR                  0x700026FE
#define AF_STATUS_PROGRESS              1
#define AE_MATRIX_SIZE                  8
#define AE_MATRIX_MAX                   5
#define AE_MATRIX_MIN                   4
#define AE_MATRIX_DEFAULT               1

static struct s5k5caga_info *info = NULL;

static int s5k5caga_write_reg(struct i2c_client *client, u8 *buf, u16 len)
{
	struct i2c_msg msg;
	int retry = 0;

	if (len < 4 || buf == NULL)
		return -EIO;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = buf;
	do {
		if (i2c_transfer(client->adapter, &msg, 1) == 1)
			return 0;

		retry++;
		pr_err("%s : i2c transfer failed, addr: 0x%x%x, len:%u\n",
		       __func__, buf[1], buf[0], len);
		msleep(S5K5CAGA_MAX_WAITMS);
	} while (retry < S5K5CAGA_MAX_RETRIES);

	return -EIO;
}

static int s5k5caga_write_reg_help(struct i2c_client *client,
				   const struct s5k5caga_reg **table)
{
	int err, index;
	u16 len;
	u8 *buf = NULL;

	err = index = 0;
	if ((*table)->addr == COMMAND_DATA_ADDR &&
		(*table + 1)->addr == COMMAND_DATA_ADDR) {
		const struct s5k5caga_reg *count_reg = *table + 2;

		len = 6;
		while (count_reg->addr == COMMAND_DATA_ADDR) {
			len += 2;
			count_reg++;
		}
	} else {
		len = 4;
	}

	buf = kmalloc(len, GFP_ATOMIC);
	if (!buf)
		return -ENOMEM;

	buf[index++] = (*table)->addr >> 8;
	buf[index++] = (*table)->addr;
	while ((*table)->purpose == S5K5CAGA_REG) {
		buf[index++] = (*table)->val >> 8;
		buf[index++] = (*table)->val;
		if (index >= len)
			break;

		(*table)++;
	}

	err = s5k5caga_write_reg(client, buf, index);
	kfree(buf);

	return err;
}

static int s5k5caga_read_reg(struct i2c_client *client, u32 addr)
{
	struct i2c_msg msg[2];
	u8 write_buf[4];
	u16 buf, cmd_data_addr = swab16(COMMAND_DATA_ADDR);
	int retry = 0;

	write_buf[0] = COMMAND_READ_ADDR_H >> 8;
	write_buf[1] = COMMAND_READ_ADDR_H;
	write_buf[2] = addr >> 24;
	write_buf[3] = addr >> 16;
	if (s5k5caga_write_reg(client, write_buf, 4) < 0)
		return -EIO;

	write_buf[0] = COMMAND_READ_ADDR_L >> 8;
	write_buf[1] = COMMAND_READ_ADDR_L;
	write_buf[2] = addr >> 8;
	write_buf[3] = addr;
	if (s5k5caga_write_reg(client, write_buf, 4) < 0)
		return -EIO;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = (u8 *) &cmd_data_addr;
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = (u8 *) &buf;
	do {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			return swab16(buf);

		retry++;
		pr_err("%s : i2c address 0x%x read failed.\n", __func__, addr);
		msleep(S5K5CAGA_MAX_WAITMS);
	} while (retry < S5K5CAGA_MAX_RETRIES);

	return -EIO;
}

static int s5k5caga_write_table(struct i2c_client *client,
				const struct s5k5caga_reg *table)
{
	int err = 0;
	const struct s5k5caga_reg *next;

	for (next = table; next->purpose != S5K5CAGA_TABLE_END; next++) {
		switch (next->purpose) {
		case S5K5CAGA_REG:
			err = s5k5caga_write_reg_help(client, &next);
			break;

		case S5K5CAGA_WAIT_MS:
			msleep(next->val);
			break;

		default:
			pr_err("%s: invalid operation 0x%x\n", __func__,
			       next->purpose);
			break;
		}

		if (err)
			break;
	}

	return err;
}

static int s5k5caga_set_mode(struct s5k5caga_info *info,
			     struct s5k5caga_mode *mode)
{
	if (mode->xres == 1024 && mode->yres == 768) {
		if (info->mode == S5K5CAGA_MODE_PREVIEW)
			return 0;

		info->mode = S5K5CAGA_MODE_PREVIEW;
	} else if (mode->xres == 2048 && mode->yres == 1536) {
		info->mode = S5K5CAGA_MODE_CAPTURE;
	} else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
			__func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	if (info->mode == S5K5CAGA_MODE_CAPTURE && info->continuous_trigger) {
		msleep(370);
		info->continuous_trigger = 0;
	}

	return s5k5caga_write_table(info->i2c_client, mode_table[info->mode]);
}

static void s5k5caga_set_exposure_rect(struct s5k5caga_rect rect)
{
	u8 exposure_table[AE_MATRIX_SIZE][AE_MATRIX_SIZE];
	int x = rect.x + rect.width / 2;
	int y = rect.y + rect.height / 2;
	int i, j;

	x = x % 32 >= 16 ? x / 32 + 1 : x / 32;
	y = y % 32 >= 16 ? y / 32 + 1 : y / 32;
	for (i = 0; i < AE_MATRIX_SIZE; i ++) {
		for (j = 0; j < AE_MATRIX_SIZE; j ++) {
			s8 exposure_value = x - i;
			int er_index = i / 2 + j * AE_MATRIX_SIZE / 2 + 2;

			exposure_value = (exposure_value > 0) ?
				AE_MATRIX_MAX - exposure_value :
				AE_MATRIX_MAX + exposure_value;
			if (exposure_value < AE_MATRIX_MIN ||
				exposure_value > AE_MATRIX_MAX)
				exposure_value = AE_MATRIX_DEFAULT;

			exposure_table[i][j] = exposure_value;
			if (exposure_value == AE_MATRIX_DEFAULT) {
				if (i % 2)
					exposoure_rect[er_index].val |= exposure_table[i][j] << 8;
				else
					exposoure_rect[er_index].val = exposure_table[i][j];

				continue;
			}

			exposure_value = y - j;
			exposure_value = (exposure_value > 0) ?
				AE_MATRIX_MAX - exposure_value :
				AE_MATRIX_MAX + exposure_value;
			if (exposure_value < AE_MATRIX_MIN ||
				exposure_value > AE_MATRIX_MAX)
				exposure_value = AE_MATRIX_DEFAULT;

			if (exposure_value <
				exposure_table[i][j])
				exposure_table[i][j] = exposure_value;

			if (i % 2)
				exposoure_rect[er_index].val |= exposure_table[i][j] << 8;
			else
				exposoure_rect[er_index].val = exposure_table[i][j];
		}
	}
}

static long s5k5caga_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	struct s5k5caga_info *info = file->private_data;
	struct s5k5caga_mode mode;
	struct s5k5caga_rect rect;
	int ret;

	ret = mutex_lock_interruptible(&info->lock);
	if (ret)
		return ret;

	switch (cmd) {
	case S5K5CAGA_IOCTL_SET_MODE:
		if (copy_from_user(&mode, (const void __user *)arg,
			sizeof(struct s5k5caga_mode))) {
			ret = -EFAULT;
			break;
		}

		ret = s5k5caga_set_mode(info, &mode);
		break;

	case S5K5CAGA_IOCTL_SET_WHITE_BALANCE:
		switch (arg) {
		case S5K5CAGA_WHITE_BALANCE_AUTO:
			if (info->i2c_client->addr == MODULE_BISON_I2C_ADDR)
				ret = s5k5caga_write_table(info->i2c_client,
						      white_balance_auto_bison);
			else
				ret = s5k5caga_write_table(info->i2c_client,
						    white_balance_auto_foxlink);

			break;

		case S5K5CAGA_WHITE_BALANCE_INCANDESCENT:
			if (info->i2c_client->addr == MODULE_BISON_I2C_ADDR)
				ret = s5k5caga_write_table(info->i2c_client,
					      white_balance_incandescent_bison);
			else
				ret = s5k5caga_write_table(info->i2c_client,
					    white_balance_incandescent_foxlink);

			break;

		case S5K5CAGA_WHITE_BALANCE_DAYLIGHT:
			if (info->i2c_client->addr == MODULE_BISON_I2C_ADDR)
				ret = s5k5caga_write_table(info->i2c_client,
						  white_balance_daylight_bison);
			else
				ret = s5k5caga_write_table(info->i2c_client,
						white_balance_daylight_foxlink);

			break;

		case S5K5CAGA_WHITE_BALANCE_FLUORESCENT:
			if (info->i2c_client->addr == MODULE_BISON_I2C_ADDR)
				ret = s5k5caga_write_table(info->i2c_client,
					       white_balance_fluorescent_bison);
			else
				ret = s5k5caga_write_table(info->i2c_client,
					     white_balance_fluorescent_foxlink);

			break;

		case S5K5CAGA_WHITE_BALANCE_CLOUDY:
			if (info->i2c_client->addr == MODULE_BISON_I2C_ADDR)
				ret = s5k5caga_write_table(info->i2c_client,
						    white_balance_cloudy_bison);
			else
				ret = s5k5caga_write_table(info->i2c_client,
						  white_balance_cloudy_foxlink);

			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case S5K5CAGA_IOCTL_SET_EXPOSURE:
		switch ((int)arg) {
		case S5K5CAGA_EXPOSURE_0:
			ret = s5k5caga_write_table(info->i2c_client,
						   exposure_0);
			break;

		case S5K5CAGA_EXPOSURE_PLUS_1:
			ret = s5k5caga_write_table(info->i2c_client,
						   exposure_plus_1);
			break;

		case S5K5CAGA_EXPOSURE_PLUS_2:
			ret = s5k5caga_write_table(info->i2c_client,
						   exposure_plus_2);
			break;

		case S5K5CAGA_EXPOSURE_MINUS_1:
			ret = s5k5caga_write_table(info->i2c_client,
						   exposure_minus_1);
			break;

		case S5K5CAGA_EXPOSURE_MINUS_2:
			ret = s5k5caga_write_table(info->i2c_client,
						   exposure_minus_2);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case S5K5CAGA_IOCTL_SET_AF_MODE:
		if (info->af_mode == arg)
			break;

		switch (arg) {
		case S5K5CAGA_FOCUS_AUTO:
			ret = s5k5caga_write_table(info->i2c_client,
						   focus_auto);
			info->af_mode = S5K5CAGA_FOCUS_AUTO;
			break;

		case S5K5CAGA_FOCUS_INFINITY:
			ret = s5k5caga_write_table(info->i2c_client,
						   focus_infinity);
			info->af_mode = S5K5CAGA_FOCUS_INFINITY;
			break;

		case S5K5CAGA_FOCUS_MACRO:
			ret = s5k5caga_write_table(info->i2c_client,
						   focus_macro);
			info->af_mode = S5K5CAGA_FOCUS_MACRO;
			break;

		case S5K5CAGA_FOCUS_CONTINUOUS:
			ret = s5k5caga_write_table(info->i2c_client,
						   focus_continuous);
			info->af_mode = S5K5CAGA_FOCUS_CONTINUOUS;
			break;
		default:
			ret = -EINVAL;
			break;
		}

		break;

	case S5K5CAGA_IOCTL_SET_AF_TRIGGER:
		if (!arg)
			break;

		ret = s5k5caga_write_table(info->i2c_client, focus_trigger);
		break;

	case S5K5CAGA_IOCTL_GET_AF_STATUS:
		ret = (s5k5caga_read_reg(info->i2c_client, AF_STATUS_ADDR) ==
			AF_STATUS_PROGRESS) ? 0 : 1;
		if (info->af_mode == S5K5CAGA_FOCUS_CONTINUOUS && !ret &&
			!info->continuous_trigger)
			info->continuous_trigger = 1;

		break;

	case S5K5CAGA_IOCTL_SET_FPS:
		switch (arg) {
		case S5K5CAGA_FPS_MIN:
			ret = s5k5caga_write_table(info->i2c_client, fps_min);

			break;

		case S5K5CAGA_FPS_MAX:
			ret = s5k5caga_write_table(info->i2c_client, fps_max);

			break;

		case S5K5CAGA_FPS_MID:
		case S5K5CAGA_FPS_DEFAULT:
			ret = s5k5caga_write_table(info->i2c_client, fps_default);

			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case S5K5CAGA_IOCTL_GET_ISO:
		ret = s5k5caga_read_reg(info->i2c_client, ISO_VALUE_ADDR);
		if (ret < 0)
			break;

		ret = ret * 10 / 256;
		if (ret < 10)
			ret = 50;
		else if(ret >= 10 && ret < 19)
			ret = 50;
		else if(ret >= 19 && ret < 23)
			ret = 100;
		else if(ret >= 23 && ret < 28)
			ret = 200;
		else if(ret >= 28)
			ret = 400;
		else {
			printk("%s: iso read unknown range %d\n",__func__,ret);
			ret = 0;
		}

		break;

	case S5K5CAGA_IOCTL_GET_EXPOSURE_TIME:
		ret = s5k5caga_read_reg(info->i2c_client, EXPOSURE_TIME_ADDR);
		if (ret < 0)
			break;

		ret = ret / 400;
		break;

	/* rectangular of focus */
	case S5K5CAGA_IOCTL_SET_AF_RECT:
		if (copy_from_user(&rect, (const void __user *)arg,
			sizeof(struct s5k5caga_rect))) {
			ret = -EFAULT;
			break;
		}

		if (!rect.width || !rect.height) {
			if (!memcmp(focus_rect, focus_rect_deafult,
				sizeof(focus_rect)))
				break;

			/* set back to deafult */
			memcpy(focus_rect, focus_rect_deafult, sizeof(focus_rect));
		} else {
			focus_rect[2].val = rect.x;
			focus_rect[3].val = rect.y;
			focus_rect[4].val = rect.width;
			focus_rect[5].val = rect.height;
			focus_rect[6].val = rect.x;
			focus_rect[7].val = rect.y;
			focus_rect[8].val = rect.width;
			focus_rect[9].val = rect.height;
		}

		ret = s5k5caga_write_table(info->i2c_client, focus_rect);
		break;

	/* rectangular of exposure */
	case S5K5CAGA_IOCTL_SET_EXPOSURE_RECT:
		if (copy_from_user(&rect, (const void __user *)arg,
			sizeof(struct s5k5caga_rect))) {
			ret = -EFAULT;
			break;
		}

		if (!rect.width || !rect.height) {
			if (!memcmp(exposoure_rect, exposoure_rect_deafult,
				sizeof(exposoure_rect)))
				break;

			/* set back to deafult */
			memcpy(exposoure_rect, exposoure_rect_deafult,
			       sizeof(exposoure_rect));
		} else {
			s5k5caga_set_exposure_rect(rect);
		}

		ret = s5k5caga_write_table(info->i2c_client, exposoure_rect);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&info->lock);

	return ret;
}

static int s5k5caga_open(struct inode *inode, struct file *file)
{
	int err = 0;

	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();

	/* Camera initialization */
	if (info->i2c_client->addr == MODULE_BISON_I2C_ADDR)
		err = s5k5caga_write_table(info->i2c_client, mode_init_bison);
	else
		err = s5k5caga_write_table(info->i2c_client, mode_init_foxlink);

	if (err)
		return err;

	info->mode = S5K5CAGA_MODE_PREVIEW;
	info->af_mode = -1;
	info->continuous_trigger = 0;

	return 0;
}

static int s5k5caga_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();

	file->private_data = NULL;

	return 0;
}


static const struct file_operations s5k5caga_fileops = {
	.owner = THIS_MODULE,
	.open = s5k5caga_open,
	.unlocked_ioctl = s5k5caga_ioctl,
	.release = s5k5caga_release,
};

static struct miscdevice s5k5caga_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = S5K5CAGA_NAME,
	.fops = &s5k5caga_fileops,
};

static int s5k5caga_remove(struct i2c_client *client)
{
	struct s5k5caga_info *info;

	info = i2c_get_clientdata(client);
	misc_deregister(&s5k5caga_device);
	mutex_destroy(&info->lock);
	kfree(info);
	info = NULL;

	return 0;
}

static int s5k5caga_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int err = 0;

	if (info)
		return -EBUSY;

	pr_info("%s\n", __func__);
	info = kzalloc(sizeof(struct s5k5caga_info), GFP_KERNEL);
	if (!info) {
		pr_err("%s : Unable to allocate memory!\n", __func__);
		return -ENOMEM;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
	i2c_set_clientdata(client, info);
	if (info->pdata && info->pdata->init)
		info->pdata->init();

	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();

	if (s5k5caga_read_reg(info->i2c_client, CHIP_ID_ADDR) != CHIP_ID_VAL) {
		pr_err("%s : Unable to read chip ID!\n", __func__);
		err = -ENODEV;
	}

	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();

	if (err) {
		kfree(info);
		info = NULL;
		return err;
	}

	err = misc_register(&s5k5caga_device);
	if (err) {
		pr_err("%s : Unable to register misc device!\n", __func__);
		kfree(info);
		info = NULL;
		return err;
	}

	mutex_init(&info->lock);

	return 0;
}

static const struct i2c_device_id s5k5caga_id[] = {
	{S5K5CAGA_NAME, 0},
};

MODULE_DEVICE_TABLE(i2c, s5k5caga_id);

static struct i2c_driver s5k5caga_i2c_driver = {
	.driver = {
		.name = S5K5CAGA_NAME,
		.owner = THIS_MODULE,
	},
	.probe = s5k5caga_probe,
	.remove = s5k5caga_remove,
	.id_table = s5k5caga_id,
};

static int __init s5k5caga_init(void)
{
	return i2c_add_driver(&s5k5caga_i2c_driver);
}

static void __exit s5k5caga_exit(void)
{
	i2c_del_driver(&s5k5caga_i2c_driver);
}

module_init(s5k5caga_init);
module_exit(s5k5caga_exit);
