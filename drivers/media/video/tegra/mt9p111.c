/*
 * kernel/drivers/media/video/tegra
 *
 * Aptina MT9P111 sensor driver
 *
 * Copyright (C) 2010 NVIDIA Corporation
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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <media/mt9p111.h>

#include "mt9p111_reg.h"

#define CHIP_ID_ADDR                    0x0000
#define CHIP_ID_VAL                     0x2880
#define AE_TRACK_AGAIN_ADDR             0xA82A
#define AE_TRACK_INT_TIME_LINES_ADDR    0xA83A
#define LINE_LENGTH_PCK_ADDR            0x300C
#define FINE_INTEGRATION_TIME_ADDR      0x3014
#define OUTPUT_CLK                      118
#define AF_PROGRESS_ADDR                0xB006
#define FLASH_STATUS_ADDR               0xC05A
#define OTPM_DATA_L_ADDR                0x380C
#define OTPM_DATA_H_ADDR                0x380E
#define OTPM_DATA_L_VAL                 0x4090
#define OTPM_DATA_H_VAL                 0x0000

static struct mt9p111_info *info;

static int mt9p111_write_reg(struct i2c_client *client, u8 *buf, u16 len)
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
		msleep(MT9P111_MAX_WAITMS);
	} while (retry < MT9P111_MAX_RETRIES);

	return -EIO;
}

static int mt9p111_write_reg16(struct i2c_client *client,
			       const struct mt9p111_reg **table)
{
	int err, index;
	u16 len;
	u8 *buf = NULL;

	err = index = 0;
	if ((*table + 1)->purpose == MT9P111_REG16 &&
		(*table)->addr + 2 == (*table + 1)->addr) {
		const struct mt9p111_reg *count_reg = *table + 2;

		len = 6;
		while (count_reg->purpose == MT9P111_REG16 &&
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
	while ((*table)->purpose == MT9P111_REG16) {
		buf[index++] = (*table)->val >> 8;
		buf[index++] = (*table)->val;
		if (index >= len)
			break;

		(*table)++;
	}

	err = mt9p111_write_reg(client, buf, index);
	kfree(buf);

	return err;
}

static int mt9p111_write_reg8(struct i2c_client *client,
			      const struct mt9p111_reg **table)
{
	int err, index;
	u16 len;
	u8 *buf = NULL;

	err = index = 0;
	if ((*table + 1)->purpose == MT9P111_REG8 &&
		(*table)->addr + 1 == (*table + 1)->addr) {
		const struct mt9p111_reg *count_reg = *table + 2;

		len = 4;
		while (count_reg->purpose == MT9P111_REG8 &&
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
	while ((*table)->purpose == MT9P111_REG8) {
		buf[index++] = (*table)->val;
		if (index >= len)
			break;

		(*table)++;
	}

	err = mt9p111_write_reg(client, buf, index);
	kfree(buf);

	return err;
}

static int mt9p111_read_reg16(struct i2c_client *client, u16 addr)
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
		msleep(MT9P111_MAX_WAITMS);
	} while (retry < MT9P111_MAX_RETRIES);

	return -EIO;
}

static int mt9p111_read_reg8(struct i2c_client *client, u16 addr)
{
	struct i2c_msg msg[2];
	u8 buf;
	int retry = 0;

	addr = swab16(addr);
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = (u8 *) &addr;
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &buf;
	do {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			return buf;

		retry++;
		pr_err("%s : i2c address 0x%x read failed.\n", __func__,
		       swab16(addr));
		msleep(MT9P111_MAX_WAITMS);
	} while (retry < MT9P111_MAX_RETRIES);

	return -EIO;
}

static int mt9p111_poll_reg16(struct i2c_client *client, u16 addr,
			      u16 expect_val, u16 mask)
{
	int i;

	for (i = 0; i < MT9P111_POLL_RETRIES; i++) {
		if ((mt9p111_read_reg16(client, addr) & mask) == expect_val)
			return 0;

		msleep(MT9P111_POLL_WAITMS);
	}

	return -ETIME;
}

static int mt9p111_poll_reg8(struct i2c_client *client, u16 addr, u8 expect_val,
			     u8 mask)
{
	int i;

	for (i = 0; i < MT9P111_POLL_RETRIES; i++) {
		if ((mt9p111_read_reg8(client, addr) & mask) == expect_val)
			return 0;

		msleep(MT9P111_POLL_WAITMS);
	}

	return -ETIME;
}

static int mt9p111_write_table(struct i2c_client *client,
			       const struct mt9p111_reg *table)
{
	int err = 0;
	const struct mt9p111_reg *next;

	for (next = table; next->purpose != MT9P111_TABLE_END; next++) {
		switch (next->purpose) {
		case MT9P111_REG16:
			err = mt9p111_write_reg16(client, &next);
			break;

		case MT9P111_REG8:
			err = mt9p111_write_reg8(client, &next);
			break;

		case MT9P111_POLL16:
			err = mt9p111_poll_reg16(client, next->addr, next->val,
						 next->mask);
			break;

		case MT9P111_POLL8:
			err = mt9p111_poll_reg8(client, next->addr, next->val,
						next->mask);
			break;

		case MT9P111_WAIT_MS:
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

static int mt9p111_set_mode(struct mt9p111_info *info,
			    struct mt9p111_mode *mode)
{
	int err;

	if (mode->mode == MT9P111_MODE_PREVIEW && mode->xres == 1280 &&
		mode->yres == 960) {
		if (info->mode == MT9P111_MODE_PREVIEW)
			return 0;

		info->mode = MT9P111_MODE_PREVIEW;
	} else if (mode->mode == MT9P111_MODE_CAPTURE && mode->xres <= 2592 &&
		mode->yres <= 1944) {
		if (capture_XxY[1].val != mode->xres ||
			capture_XxY[2].val != mode->yres) {
			capture_XxY[1].val = mode->xres;
			capture_XxY[2].val = mode->yres;
			mt9p111_write_table(info->i2c_client, capture_XxY);
		}

		info->mode = MT9P111_MODE_CAPTURE;
	} else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	err = mt9p111_write_table(info->i2c_client, mode_table[info->mode]);
	if (err)
		return err;

	if (info->mode == MT9P111_MODE_PREVIEW &&
		info->af_mode == MT9P111_FOCUS_CONTINUOUS)
		mt9p111_write_table(info->i2c_client, focus_trigger);

	return 0;
}

static long mt9p111_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct mt9p111_info *info = file->private_data;
	struct mt9p111_mode mode;
	struct mt9p111_rect rect;
	struct mt9p111_iso *iso;
	int ret, et;

	ret = mutex_lock_interruptible(&info->lock);
	if (ret)
		return ret;

	switch (cmd) {
	case MT9P111_IOCTL_SET_MODE:
		if (copy_from_user(&mode, (const void __user *)arg,
			sizeof(struct mt9p111_mode))) {
			ret = -EFAULT;
			break;
		}

		ret = mt9p111_set_mode(info, &mode);
		break;

	case MT9P111_IOCTL_SET_COLOR_EFFECT:
		switch (arg & MT9P111_COLOR_EFFECT_MASK) {
		case MT9P111_COLOR_EFFECT_NONE:
			ret = mt9p111_write_table(info->i2c_client,
						  color_effect_none);
			break;

		case MT9P111_COLOR_EFFECT_MONO:
			ret = mt9p111_write_table(info->i2c_client,
						  color_effect_mono);
			break;

		case MT9P111_COLOR_EFFECT_SEPIA:
			ret = mt9p111_write_table(info->i2c_client,
						  color_effect_sepia);
			break;

		case MT9P111_COLOR_EFFECT_NEGATIVE:
			ret = mt9p111_write_table(info->i2c_client,
						  color_effect_negative);
			break;

		case MT9P111_COLOR_EFFECT_SOLARIZE:
			ret = mt9p111_write_table(info->i2c_client,
						  color_effect_solarize);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9P111_IOCTL_SET_WHITE_BALANCE:
		switch (arg) {
		case MT9P111_WHITE_BALANCE_AUTO:
			ret = mt9p111_write_table(info->i2c_client,
						  white_balance_auto);
			break;

		case MT9P111_WHITE_BALANCE_INCANDESCENT:
			ret = mt9p111_write_table(info->i2c_client,
						  white_balance_incandescent);
			break;

		case MT9P111_WHITE_BALANCE_DAYLIGHT:
			ret = mt9p111_write_table(info->i2c_client,
						  white_balance_daylight);
			break;

		case MT9P111_WHITE_BALANCE_FLUORESCENT:
			ret = mt9p111_write_table(info->i2c_client,
						  white_balance_fluorescent);
			break;

		case MT9P111_WHITE_BALANCE_CLOUDY:
			ret = mt9p111_write_table(info->i2c_client,
						  white_balance_cloudy);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9P111_IOCTL_SET_EXPOSURE:
		switch ((int)arg) {
		case MT9P111_EXPOSURE_0:
			ret = mt9p111_write_table(info->i2c_client, exposure_0);
			break;

		case MT9P111_EXPOSURE_PLUS_1:
			ret = mt9p111_write_table(info->i2c_client,
						  exposure_plus_1);
			break;

		case MT9P111_EXPOSURE_PLUS_2:
			ret = mt9p111_write_table(info->i2c_client,
						  exposure_plus_2);
			break;

		case MT9P111_EXPOSURE_MINUS_1:
			ret = mt9p111_write_table(info->i2c_client,
						  exposure_minus_1);
			break;

		case MT9P111_EXPOSURE_MINUS_2:
			ret = mt9p111_write_table(info->i2c_client,
						  exposure_minus_2);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9P111_IOCTL_SET_AF_MODE:
		if (info->af_mode == arg)
			break;

		switch (arg) {
		case MT9P111_FOCUS_AUTO:
			ret = mt9p111_write_table(info->i2c_client, focus_auto);
			info->af_mode = MT9P111_FOCUS_AUTO;
			break;

		case MT9P111_FOCUS_INFINITY:
			ret = mt9p111_write_table(info->i2c_client,
						  focus_infinity);
			info->af_mode = MT9P111_FOCUS_INFINITY;
			break;

		case MT9P111_FOCUS_MACRO:
			ret = mt9p111_write_table(info->i2c_client,
						  focus_macro);
			info->af_mode = MT9P111_FOCUS_MACRO;
			break;

		case MT9P111_FOCUS_CONTINUOUS:
			ret = mt9p111_write_table(info->i2c_client,
						  focus_continuous);
			mt9p111_write_table(info->i2c_client, focus_trigger);
			info->af_mode = MT9P111_FOCUS_CONTINUOUS;
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9P111_IOCTL_SET_AF_TRIGGER:
		if (!arg || mt9p111_read_reg8(info->i2c_client,
			AF_PROGRESS_ADDR))
			break;

		if (info->touch_active) {
			mt9p111_write_table(info->i2c_client, seq_refresh);
			info->touch_active = 0;
		}

		ret = mt9p111_write_table(info->i2c_client, focus_trigger);
		break;

	case MT9P111_IOCTL_GET_AF_STATUS:
		ret = (mt9p111_read_reg8(info->i2c_client, AF_PROGRESS_ADDR)) ?
			0 : 1;
		break;

	case MT9P111_IOCTL_SET_FPS:
		switch (arg) {
		case MT9P111_FPS_MIN:
			ret = mt9p111_write_table(info->i2c_client, fps_min);
			break;

		case MT9P111_FPS_MAX:
			ret = mt9p111_write_table(info->i2c_client, fps_max);
			break;

		case MT9P111_FPS_MID:
		case MT9P111_FPS_DEFAULT:
			ret = mt9p111_write_table(info->i2c_client,
						  fps_default);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9P111_IOCTL_GET_ISO:
		ret = mt9p111_read_reg16(info->i2c_client,
					 AE_TRACK_AGAIN_ADDR);
		if (ret < 0)
			break;

		for (iso = iso_table + 1; iso->value >= 0; iso++) {
			struct mt9p111_iso *pre_iso;

			if (iso->again < ret)
				continue;

			pre_iso = iso - 1;
			ret = (iso->value - pre_iso->value) *
				(ret - pre_iso->again) /
				(iso->again - pre_iso->again) +
				pre_iso->value;
			break;
		}

		if (iso->value == -1)
			ret = (iso - 1)->value;
		break;

	case MT9P111_IOCTL_GET_EXPOSURE_TIME:
		ret = mt9p111_read_reg16(info->i2c_client,
					 AE_TRACK_INT_TIME_LINES_ADDR);
		if (ret < 0)
			break;

		et = ret;
		ret = mt9p111_read_reg16(info->i2c_client,
					 LINE_LENGTH_PCK_ADDR);
		if (ret < 0)
			break;

		et = ret * et;
		ret = mt9p111_read_reg16(info->i2c_client,
					 FINE_INTEGRATION_TIME_ADDR);
		if (ret < 0)
			break;

		ret = (ret + et) / OUTPUT_CLK;
		break;

	case MT9P111_IOCTL_SET_FLASH_MODE:
		switch (arg) {
		case MT9P111_FLASH_ON:
			ret = mt9p111_write_table(info->i2c_client, flash_on);
			break;

		case MT9P111_FLASH_OFF:
			ret = mt9p111_write_table(info->i2c_client, flash_off);
			break;

		case MT9P111_FLASH_AUTO:
			ret = mt9p111_write_table(info->i2c_client, flash_auto);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9P111_IOCTL_GET_FLASH_STATUS:
		ret = mt9p111_read_reg8(info->i2c_client, FLASH_STATUS_ADDR);
		break;

	/* rectangular of exposure */
	case MT9P111_IOCTL_SET_EXPOSURE_RECT:
		if (copy_from_user(&rect, (const void __user *)arg,
				   sizeof(struct mt9p111_rect))) {
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
			exposoure_rect[1].val = rect.x;
			exposoure_rect[2].val = rect.y;
			exposoure_rect[3].val = rect.width;
			exposoure_rect[4].val = rect.height;
		}

		ret = mt9p111_write_table(info->i2c_client, exposoure_rect);
		info->touch_active = 1;
		break;

	/* rectangular of focus */
	case MT9P111_IOCTL_SET_AF_RECT:
		if (copy_from_user(&rect, (const void __user *)arg,
			sizeof(struct mt9p111_rect))) {
			ret = -EFAULT;
			break;
		}

		if (!rect.width || !rect.height) {
			if (!memcmp(focus_rect, focus_rect_deafult,
				sizeof(focus_rect)))
				break;

			/* set back to deafult */
			memcpy(focus_rect, focus_rect_deafult,
			       sizeof(focus_rect));
		} else {
			focus_rect[1].val = rect.x;
			focus_rect[2].val = rect.y;
			focus_rect[3].val = rect.width;
			focus_rect[4].val = rect.height;
		}

		ret = mt9p111_write_table(info->i2c_client, focus_rect);
		info->touch_active = 1;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&info->lock);

	return ret;
}

static int mt9p111_open(struct inode *inode, struct file *file)
{
	int err = 0;

	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();

	err = mt9p111_write_table(info->i2c_client, mode_init_start);
	if (err)
		return err;

	/* Check OTPM */
	if (mt9p111_read_reg16(info->i2c_client, OTPM_DATA_L_ADDR) ==
		OTPM_DATA_L_VAL && mt9p111_read_reg16(info->i2c_client,
		OTPM_DATA_H_ADDR) == OTPM_DATA_H_VAL)
		mt9p111_write_table(info->i2c_client, apga_otpm_lsc);
	else
		mt9p111_write_table(info->i2c_client, apga_patch_ram_lsc);

	err = mt9p111_write_table(info->i2c_client, mode_init_end);
	if (err)
		return err;

	info->mode = MT9P111_MODE_PREVIEW;
	info->af_mode = -1;
	info->touch_active = 0;
	capture_XxY[1].val = 2592;
	capture_XxY[2].val = 1944;

	return 0;
}

static int mt9p111_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();

	file->private_data = NULL;

	return 0;
}


static const struct file_operations mt9p111_fileops = {
	.owner = THIS_MODULE,
	.open = mt9p111_open,
	.unlocked_ioctl = mt9p111_ioctl,
	.release = mt9p111_release,
};

static struct miscdevice mt9p111_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MT9P111_NAME,
	.fops = &mt9p111_fileops,
};

static int mt9p111_remove(struct i2c_client *client)
{
	struct mt9p111_info *info;

	info = i2c_get_clientdata(client);
	misc_deregister(&mt9p111_device);
	mutex_destroy(&info->lock);
	kfree(info);

	return 0;
}

static int mt9p111_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;

	pr_info("%s\n", __func__);
	info = kzalloc(sizeof(struct mt9p111_info), GFP_KERNEL);
	if (!info) {
		pr_err("%s : Unable to allocate memory!\n", __func__);
		return -ENOMEM;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
	i2c_set_clientdata(client, info);
	if (info->pdata && info->pdata->init)
		err = info->pdata->init();

	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();

	if (mt9p111_read_reg16(info->i2c_client, CHIP_ID_ADDR) != CHIP_ID_VAL) {
		pr_err("%s : Unable to read chip ID!\n", __func__);
		err = -ENODEV;
	}

	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();

	if (err) {
		kfree(info);
		return err;
	}

	err = misc_register(&mt9p111_device);
	if (err) {
		pr_err("%s : Unable to register misc device!\n", __func__);
		kfree(info);
		return err;
	}

	mutex_init(&info->lock);

	return 0;
}

static const struct i2c_device_id mt9p111_id[] = {
	{MT9P111_NAME, 0},
};

MODULE_DEVICE_TABLE(i2c, mt9p111_id);

static struct i2c_driver mt9p111_i2c_driver = {
	.driver = {
		.name = MT9P111_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mt9p111_probe,
	.remove = mt9p111_remove,
	.id_table = mt9p111_id,
};

static int __init mt9p111_init(void)
{
	return i2c_add_driver(&mt9p111_i2c_driver);
}

static void __exit mt9p111_exit(void)
{
	i2c_del_driver(&mt9p111_i2c_driver);
}

module_init(mt9p111_init);
module_exit(mt9p111_exit);
