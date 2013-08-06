/*
 * kernel/drivers/media/video/tegra
 *
 * Aptina MT9M114 sensor driver
 *
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <media/mt9m114.h>

#include "mt9m114_reg.h"

#define MODULE_FOXLINK_I2C_ADDR         0x5D
#define MODULE_BISON_I2C_ADDR           0x48
#define CHIP_ID_ADDR                    0x0000
#define CHIP_ID_VAL                     0x2481
#define AE_TRACK_FDZONE_ADDR            0xA818
#define LINE_LENGTH_PCK_ADDR            0x300C
#define COARSE_INTEGRATION_TIME_ADDR    0x3012
#define FINE_INTEGRATION_TIME_ADDR      0x3014
#define OUTPUT_CLK                      48

static struct mt9m114_info *info = NULL;

static int mt9m114_write_reg(struct i2c_client *client, u8 *buf, u16 len)
{
	struct i2c_msg msg;
	int retry = 0;

	if (len < 3 || buf == NULL)
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
		msleep(MT9M114_MAX_WAITMS);
	} while (retry < MT9M114_MAX_RETRIES);

	return -EIO;
}

static int mt9m114_write_reg32(struct i2c_client *client, u16 addr, u32 val)
{
	struct i2c_msg msg;
	u8 buf[6];
	int retry = 0;

	buf[0] = addr >> 8;
	buf[1] = addr;
	buf[2] = val >> 24;
	buf[3] = val >> 16;
	buf[4] = val >> 8;
	buf[5] = val;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 6;
	msg.buf = buf;
	do {
		if (i2c_transfer(client->adapter, &msg, 1) == 1)
			return 0;

		retry++;
		pr_err("%s : i2c transfer failed, addr: 0x%x, val:0x%x\n",
		       __func__, addr, val);
		msleep(MT9M114_MAX_WAITMS);
	} while (retry < MT9M114_MAX_RETRIES);

	return -EIO;
}

static int mt9m114_write_reg16(struct i2c_client *client,
			       const struct mt9m114_reg **table)
{
	int err, index;
	u16 len;
	u8 *buf = NULL;

	err = index = 0;
	if ((*table + 1)->purpose == MT9M114_REG16 &&
		(*table)->addr + 2 == (*table + 1)->addr) {
		const struct mt9m114_reg *count_reg = *table + 2;

		len = 6;
		while (count_reg->purpose == MT9M114_REG16 &&
			count_reg->addr == (count_reg - 1)->addr + 2) {
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
	while ((*table)->purpose == MT9M114_REG16) {
		buf[index++] = (*table)->val >> 8;
		buf[index++] = (*table)->val;
		if (index >= len)
			break;

		(*table)++;
	}

	err = mt9m114_write_reg(client, buf, index);
	kfree(buf);

	return err;
}

static int mt9m114_write_reg8(struct i2c_client *client,
			      const struct mt9m114_reg **table)
{
	int err, index;
	u16 len;
	u8 *buf = NULL;

	err = index = 0;
	if ((*table + 1)->purpose == MT9M114_REG8 &&
		(*table)->addr + 1 == (*table + 1)->addr) {
		const struct mt9m114_reg *count_reg = *table + 2;

		len = 4;
		while (count_reg->purpose == MT9M114_REG8 &&
			count_reg->addr == (count_reg - 1)->addr + 1) {
			len++;
			count_reg++;
		}
	} else {
		len = 3;
	}

	buf = kmalloc(len, GFP_ATOMIC);
	if (!buf)
		return -ENOMEM;

	buf[index++] = (*table)->addr >> 8;
	buf[index++] = (*table)->addr;
	while ((*table)->purpose == MT9M114_REG8) {
		buf[index++] = (*table)->val;
		if (index >= len)
			break;

		(*table)++;
	}

	err = mt9m114_write_reg(client, buf, index);
	kfree(buf);

	return err;
}

static int mt9m114_read_reg16(struct i2c_client *client, u16 addr)
{
	struct i2c_msg msg[2];
	u16 buf;
	int retry = 0;

	addr = swab16(addr);
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = (u8 *) &addr;
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = (u8 *) &buf;
	do {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			return swab16(buf);

		retry++;
		pr_err("%s : i2c address 0x%x read failed.\n", __func__,
		       swab16(addr));
		msleep(MT9M114_MAX_WAITMS);
	} while (retry < MT9M114_MAX_RETRIES);

	return -EIO;
}

static int mt9m114_poll_reg16(struct i2c_client *client, u16 addr,
			      u16 expect_val, u16 mask)
{
	int i;

	for (i = 0; i < MT9M114_POLL_RETRIES; i++) {
		if ((mt9m114_read_reg16(client, addr) & mask) == expect_val)
			return 0;

		msleep(MT9M114_POLL_WAITMS);
	}

	return -ETIME;
}

static int mt9m114_write_table(struct i2c_client *client,
			       const struct mt9m114_reg *table)
{
	int err = 0;
	const struct mt9m114_reg *next;

	for (next = table; next->purpose != MT9M114_TABLE_END; next++) {
		switch (next->purpose) {
		case MT9M114_REG32:
			err = mt9m114_write_reg32(client, next->addr,
						  next->val);
			break;

		case MT9M114_REG16:
			err = mt9m114_write_reg16(client, &next);
			break;

		case MT9M114_REG8:
			err = mt9m114_write_reg8(client, &next);
			break;

		case MT9M114_POLL16:
			err = mt9m114_poll_reg16(client, next->addr,
						 next->val, next->mask);
			break;

		case MT9M114_WAIT_MS:
			msleep(next->val);
			break;

		default:
			err = -EINVAL;
			pr_err("%s: invalid operation 0x%x\n", __func__,
			       next->purpose);
			break;
		}

		if (err)
			break;
	}

	return err;
}

static int mt9m114_set_mode(struct mt9m114_info *info,
			    struct mt9m114_mode *mode)
{
	int err = 0;

	if (mode->xres != 1280 || mode->yres != 960) {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	return err;
}

static long mt9m114_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct mt9m114_info *info = file->private_data;
	struct mt9m114_mode mode;
	struct mt9m114_rect rect;
	struct mt9m114_iso *iso;
	int ret, et;

	ret = mutex_lock_interruptible(&info->lock);
	if (ret)
		return ret;

	switch (cmd) {
	case MT9M114_IOCTL_SET_MODE:
		if (copy_from_user(&mode, (const void __user *)arg,
			sizeof(struct mt9m114_mode))) {
			ret = -EFAULT;
			break;
		}

		ret = mt9m114_set_mode(info, &mode);
		break;

	case MT9M114_IOCTL_SET_COLOR_EFFECT:
		switch (arg & MT9M114_COLOR_EFFECT_MASK) {
		case MT9M114_COLOR_EFFECT_NONE:
			ret = mt9m114_write_table(info->i2c_client,
						color_effect_none);
			break;

		case MT9M114_COLOR_EFFECT_MONO:
			ret = mt9m114_write_table(info->i2c_client,
						color_effect_mono);
			break;

		case MT9M114_COLOR_EFFECT_SEPIA:
			ret = mt9m114_write_table(info->i2c_client,
						  color_effect_sepia);
			break;

		case MT9M114_COLOR_EFFECT_NEGATIVE:
			ret = mt9m114_write_table(info->i2c_client,
						  color_effect_negative);
			break;

		case MT9M114_COLOR_EFFECT_SOLARIZE:
			ret = mt9m114_write_table(info->i2c_client,
						  color_effect_solarize);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9M114_IOCTL_SET_WHITE_BALANCE:
		switch (arg) {
		case MT9M114_WHITE_BALANCE_AUTO:
			ret = mt9m114_write_table(info->i2c_client,
						  white_balance_auto);
			break;

		case MT9M114_WHITE_BALANCE_INCANDESCENT:
			ret = mt9m114_write_table(info->i2c_client,
						  white_balance_incandescent);
			break;

		case MT9M114_WHITE_BALANCE_DAYLIGHT:
			ret = mt9m114_write_table(info->i2c_client,
						  white_balance_daylight);
			break;

		case MT9M114_WHITE_BALANCE_FLUORESCENT:
			ret = mt9m114_write_table(info->i2c_client,
						  white_balance_fluorescent);
			break;

		case MT9M114_WHITE_BALANCE_CLOUDY:
			ret = mt9m114_write_table(info->i2c_client,
						  white_balance_cloudy);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9M114_IOCTL_SET_EXPOSURE:
		switch ((int)arg) {
		case MT9M114_EXPOSURE_0:
			ret = mt9m114_write_table(info->i2c_client, exposure_0);
			break;

		case MT9M114_EXPOSURE_PLUS_1:
			ret = mt9m114_write_table(info->i2c_client,
						  exposure_plus_1);
			break;

		case MT9M114_EXPOSURE_PLUS_2:
			ret = mt9m114_write_table(info->i2c_client,
						  exposure_plus_2);
			break;

		case MT9M114_EXPOSURE_MINUS_1:
			ret = mt9m114_write_table(info->i2c_client,
						  exposure_minus_1);
			break;

		case MT9M114_EXPOSURE_MINUS_2:
			ret = mt9m114_write_table(info->i2c_client,
						  exposure_minus_2);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9M114_IOCTL_SET_FPS:
		switch (arg) {
		case MT9M114_FPS_MIN:
			if (info->i2c_client->addr == MODULE_BISON_I2C_ADDR) {
				ret = mt9m114_write_table(info->i2c_client, fps_min_bison);
			}
			else {
				ret = mt9m114_write_table(info->i2c_client, fps_min_foxlink);
			}
			break;

		case MT9M114_FPS_MAX:
			if (info->i2c_client->addr == MODULE_BISON_I2C_ADDR) {
				ret = mt9m114_write_table(info->i2c_client, fps_max_bison);
			}
			else {
				ret = mt9m114_write_table(info->i2c_client, fps_max_foxlink);
			}
			break;

		case MT9M114_FPS_MID:
		case MT9M114_FPS_DEFAULT:
			if (info->i2c_client->addr == MODULE_BISON_I2C_ADDR) {
				ret = mt9m114_write_table(info->i2c_client,
								fps_default_bison);
			}
			else {
				ret = mt9m114_write_table(info->i2c_client,
								fps_default_foxlink);
			}
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9M114_IOCTL_GET_ISO:
		ret = mt9m114_read_reg16(info->i2c_client,
					 AE_TRACK_FDZONE_ADDR);
		if (ret < 0)
			break;

		for (iso = iso_table + 1; iso->value >= 0; iso++) {
			struct mt9m114_iso *pre_iso;

			if (iso->fdzone < ret)
				continue;

			pre_iso = iso - 1;
			ret = (iso->value - pre_iso->value) *
				(ret - pre_iso->fdzone) /
				(iso->fdzone - pre_iso->fdzone) + pre_iso->value;
			break;
		}

		break;

	case MT9M114_IOCTL_GET_EXPOSURE_TIME:
		ret = mt9m114_read_reg16(info->i2c_client,
					 COARSE_INTEGRATION_TIME_ADDR);
		if (ret < 0)
			break;

		et = ret;
		ret = mt9m114_read_reg16(info->i2c_client,
					 LINE_LENGTH_PCK_ADDR);
		if (ret < 0)
			break;

		et = ret * et;
		ret = mt9m114_read_reg16(info->i2c_client,
					 FINE_INTEGRATION_TIME_ADDR);
		if (ret < 0)
			break;

		ret = (ret + et) / OUTPUT_CLK;
		break;

	/* rectangular of exposure */
	case MT9M114_IOCTL_SET_EXPOSURE_RECT:
		if (copy_from_user(&rect, (const void __user *)arg,
			sizeof(struct mt9m114_rect))) {
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
			int x = rect.x + rect.width / 2;
			int y = rect.y + rect.height / 2;
			int i;

			for (i = 1; i <= 25; i ++)
				exposoure_rect[i].val = 25;

			x = x % 20 >= 10 ? x / 20 + 1 : x / 20;
			y = y % 20 >= 10 ? y / 20 + 1 : y / 20;
			exposoure_rect[x + y * 5 + 1].val = 100;
			if (y + 1 <= 4)
				exposoure_rect[x + 1 + (y + 1) * 5].val = 75;

			if (y - 1 >= 0)
				exposoure_rect[x + 1 + (y - 1) * 5].val = 75;

			if (x + 1 <= 4) {
				exposoure_rect[x + 2 + y * 5].val = 75;
				if (y + 1 <= 4)
					exposoure_rect[x + 2 + (y + 1) * 5].val = 75;

				if (y - 1 >= 0)
					exposoure_rect[x + 2 + (y - 1) * 5].val = 75;
			}

			if (x - 1 >= 0) {
				exposoure_rect[x + y * 5].val = 75;
				if (y + 1 <= 4)
					exposoure_rect[x + (y + 1) * 5].val = 75;

				if (y - 1 >= 0)
					exposoure_rect[x + (y - 1) * 5].val = 75;
			}
		}

		ret = mt9m114_write_table(info->i2c_client, exposoure_rect);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&info->lock);

	return ret;
}

static int mt9m114_open(struct inode *inode, struct file *file)
{
	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();

	/* Camera initialization */
	if (info->i2c_client->addr == MODULE_BISON_I2C_ADDR)
		return mt9m114_write_table(info->i2c_client, mode_init_bison);
	else
		return mt9m114_write_table(info->i2c_client, mode_init_foxlink);
}

static int mt9m114_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();

	file->private_data = NULL;

	return 0;
}


static const struct file_operations mt9m114_fileops = {
	.owner = THIS_MODULE,
	.open = mt9m114_open,
	.unlocked_ioctl = mt9m114_ioctl,
	.release = mt9m114_release,
};

static struct miscdevice mt9m114_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MT9M114_NAME,
	.fops = &mt9m114_fileops,
};

static int mt9m114_remove(struct i2c_client *client)
{
	struct mt9m114_info *info;

	info = i2c_get_clientdata(client);
	misc_deregister(&mt9m114_device);
	mutex_destroy(&info->lock);
	kfree(info);
	info = NULL;

	return 0;
}

static int mt9m114_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;

	if (info)
		return -EBUSY;

	pr_info("%s\n", __func__);
	info = kzalloc(sizeof(struct mt9m114_info), GFP_KERNEL);
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

	if (mt9m114_read_reg16(info->i2c_client, CHIP_ID_ADDR) != CHIP_ID_VAL) {
		pr_err("%s : Unable to read correct chip ID!\n", __func__);
		err = -ENODEV;
	}

	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();

	if (err) {
		kfree(info);
		info = NULL;
		return err;
	}

	err = misc_register(&mt9m114_device);
	if (err) {
		pr_err("%s : Unable to register misc device!\n", __func__);
		kfree(info);
		info = NULL;
		return err;
	}

	mutex_init(&info->lock);

	return 0;
}

static const struct i2c_device_id mt9m114_id[] = {
	{MT9M114_NAME, 0},
};

MODULE_DEVICE_TABLE(i2c, mt9m114_id);

static struct i2c_driver mt9m114_i2c_driver = {
	.driver = {
		.name = MT9M114_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mt9m114_probe,
	.remove = mt9m114_remove,
	.id_table = mt9m114_id,
};

static int __init mt9m114_init(void)
{
	return i2c_add_driver(&mt9m114_i2c_driver);
}

static void __exit mt9m114_exit(void)
{
	i2c_del_driver(&mt9m114_i2c_driver);
}

module_init(mt9m114_init);
module_exit(mt9m114_exit);
