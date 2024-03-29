#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <mach/i2s.h>
#include <sound/fm34.h>
//#include "../tegra/tegra_soc.h"

#define FM34_DEBUG          0
#if FM34_DEBUG
#define DEBUG(f, a...) printk(KERN_DEBUG "%s: " f, __func__ , ## a)
#else
#define DEBUG(f, a...) do { ; } while(0)
#endif

#define FM34_REG_RW         1

struct fm34_drvdata {
	struct snd_soc_codec *codec;
	void (*codec_gain)(void *, int);
	int fm34_enable;	/* true: fm34 function can be turn on */
	int fm34_expect;	/* true: fm34 function is turn on  */
	int fm34_status;
};

static struct i2c_client *fm34_client;

static const u8 fm34_patch[][9] = {
	{5, 0xFC, 0xF3, 0x68, 0x64, 0x04, 0x00, 0x00, 0x00},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x00, 0x80, 0x53, 0x9A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x01, 0x93, 0xE2, 0xAA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x02, 0x80, 0x4F, 0xBA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x03, 0x22, 0x7A, 0x0F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x04, 0x34, 0x00, 0x0E},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x05, 0x18, 0x2F, 0xD0},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x06, 0x94, 0x4F, 0xB6},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x07, 0x80, 0x4F, 0x6A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x08, 0x26, 0x7A, 0x0F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x09, 0x18, 0x2F, 0x80},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x0A, 0x18, 0x2B, 0xCF},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x0B, 0x95, 0x62, 0x06},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x0C, 0x95, 0x61, 0x46},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x0D, 0x40, 0xFA, 0x0A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x0E, 0x40, 0xE5, 0xB0},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x0F, 0x82, 0x30, 0x14},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x10, 0x27, 0x00, 0x0F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x11, 0x22, 0x78, 0x00},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x12, 0x83, 0xFD, 0x44},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x13, 0x26, 0xE2, 0x0F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x14, 0x19, 0x01, 0xA0},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x15, 0x93, 0xFD, 0x4A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x16, 0x83, 0xFD, 0x5A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x17, 0x23, 0xA2, 0x1F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x18, 0x93, 0xFD, 0x5A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x19, 0x00, 0x00, 0x00},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x1A, 0x34, 0x00, 0x0E},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x1B, 0x18, 0x3F, 0x7F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x1C, 0x34, 0x00, 0x0E},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x1D, 0x19, 0x6C, 0xA4},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x1E, 0x19, 0x66, 0x7F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x1F, 0x38, 0x00, 0x87},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x20, 0x09, 0x00, 0x1B},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x21, 0x34, 0x00, 0x0E},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x22, 0x18, 0x2D, 0x1F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x23, 0x18, 0x21, 0x5F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x24, 0x88, 0x4F, 0x47},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x25, 0x82, 0x2A, 0x0A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x26, 0x22, 0x62, 0x1F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x27, 0x26, 0x62, 0x7F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x28, 0x19, 0x02, 0xB5},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x29, 0x3B, 0xFF, 0xC7},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x2A, 0x19, 0x02, 0xCF},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x2B, 0x0D, 0x01, 0xA6},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x2C, 0x92, 0x2A, 0x0A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x2D, 0x34, 0x00, 0x0E},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x2E, 0x18, 0x23, 0x2F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x2F, 0x96, 0x2A, 0xA1},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x30, 0x96, 0x2A, 0xB0},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x31, 0x9A, 0x2A, 0xCA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x32, 0x9A, 0x2A, 0xD7},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x33, 0x96, 0x2A, 0xE8},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x34, 0x92, 0x2B, 0xC3},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x35, 0x92, 0x2B, 0xD2},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x36, 0x9A, 0x2B, 0xE2},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x37, 0x38, 0x00, 0x0A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x38, 0x83, 0xFE, 0x0A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x39, 0x3B, 0x20, 0x02},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x3A, 0x36, 0x2B, 0x01},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x3B, 0x36, 0x2B, 0x60},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x3C, 0x1C, 0x72, 0x2F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x3D, 0x0D, 0x00, 0x4A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x3E, 0x86, 0x2A, 0xA1},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x3F, 0x86, 0x2A, 0xB0},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x40, 0x8A, 0x2A, 0xCA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x41, 0x8A, 0x2A, 0xD7},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x42, 0x86, 0x2A, 0xE8},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x43, 0x82, 0x2B, 0xC3},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x44, 0x82, 0x2B, 0xD2},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x45, 0x8A, 0x2B, 0xE2},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x46, 0x34, 0x00, 0x0E},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x47, 0x18, 0x20, 0x6F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x48, 0x96, 0x2A, 0xA1},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x49, 0x96, 0x2A, 0xB0},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x4A, 0x9A, 0x2A, 0xCA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x4B, 0x9A, 0x2A, 0xD7},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x4C, 0x96, 0x2A, 0xE8},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x4D, 0x38, 0x00, 0x0A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x4E, 0x83, 0xFF, 0xAA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x4F, 0x3B, 0x20, 0x02},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x50, 0x36, 0x29, 0x01},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x51, 0x36, 0x29, 0x60},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x52, 0x1C, 0x72, 0x2F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x53, 0x93, 0xFF, 0x9A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x54, 0x93, 0xFF, 0xAA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x55, 0x86, 0x2A, 0xA1},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x56, 0x86, 0x2A, 0xB0},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x57, 0x8A, 0x2A, 0xCA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x58, 0x8A, 0x2A, 0xD7},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x59, 0x86, 0x2A, 0xE8},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x5A, 0x0D, 0x08, 0x28},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x5B, 0x34, 0x00, 0x0E},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x5C, 0x18, 0x24, 0x0F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x5D, 0x83, 0xFC, 0xFA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x5E, 0x23, 0x8A, 0xBF},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x5F, 0x82, 0x2D, 0x2F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x60, 0x27, 0x97, 0xDF},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x61, 0x23, 0xAA, 0x91},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x62, 0x93, 0xFC, 0xFA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x63, 0x83, 0x80, 0x30},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x64, 0x34, 0x00, 0x0E},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x65, 0x18, 0x3E, 0x9F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x66, 0x80, 0x7A, 0x4A},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x67, 0x19, 0x07, 0x02},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x68, 0x23, 0x3E, 0x0F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x69, 0x0D, 0x00, 0xEA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x6A, 0x80, 0x7B, 0xCA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x6B, 0x38, 0x7E, 0x02},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x6C, 0x1C, 0x76, 0x9F},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x6D, 0x90, 0x7B, 0xCA},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x6E, 0x34, 0x00, 0x0E},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x6F, 0x19, 0x75, 0xCF},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x70, 0x34, 0x00, 0x0E},
    {8, 0xFC, 0xF3, 0x0D, 0x10, 0x71, 0x19, 0x75, 0x9F},

	{5, 0xFC, 0xF3, 0x68, 0x64, 0x00, 0x00, 0x00, 0x00},
};

static int fm34_register_read(struct i2c_client *client, u16 addr, u16 *pval)
{
	struct i2c_msg msg[5];
	u8 CmdBuf0[] = {0xFC, 0xF3, 0x37, 0xFF, 0xFF};
	u8 CmdBuf1[] = {0xFC, 0xF3, 0x60, 0x25};
	u8 CmdBuf2[] = {0xFC, 0xF3, 0x60, 0x26};
	u8 DataLow, DataHigh;

	CmdBuf0[3] = (addr & 0xFF00) >> 8;
	CmdBuf0[4] = addr & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = sizeof(CmdBuf0);
	msg[0].buf = CmdBuf0;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].len = sizeof(CmdBuf1);
	msg[1].buf = CmdBuf1;

	msg[2].addr = client->addr;
	msg[2].flags = client->flags & I2C_M_TEN;
	msg[2].flags |= I2C_M_RD;
	msg[2].len = sizeof(DataLow);
	msg[2].buf = &DataLow;

	msg[3].addr = client->addr;
	msg[3].flags = client->flags & I2C_M_TEN;
	msg[3].len = sizeof(CmdBuf2);
	msg[3].buf = CmdBuf2;

	msg[4].addr = client->addr;
	msg[4].flags = client->flags & I2C_M_TEN;
	msg[4].flags |= I2C_M_RD;
	msg[4].len = sizeof(DataHigh);
	msg[4].buf = &DataHigh;

	if(5 != i2c_transfer(client->adapter, msg, 5))
		return -1;

	if(pval)
		*pval = ((DataHigh << 8) | DataLow);

	DEBUG("ReadFM34Register Addr:0x%02X%02X Data:0x%02X%02X successful\n", CmdBuf0[3], CmdBuf0[4], DataHigh, DataLow);

	return 0;
}

static int fm34_register_write(struct i2c_client *client, u16 addr, u16 value)
{
	u8 CmdBuf[] = {0xFC, 0xF3, 0x3B, 0xFF, 0xFF, 0xFF, 0xFF};

	CmdBuf[3] = (addr & 0xFF00) >> 8;
	CmdBuf[4] = addr & 0xFF;
	CmdBuf[5] = (value & 0xFF00) >> 8;
	CmdBuf[6] = value & 0xFF;

	if(sizeof(CmdBuf) != i2c_master_send(client, CmdBuf, sizeof(CmdBuf))) {
		DEBUG("WriteFM34Register Addr:0x%02X%02X Data:0x%02X%02X failed\n", CmdBuf[3], CmdBuf[4], CmdBuf[5], CmdBuf[6]);
		return -1;
	} else {
		DEBUG("WriteFM34Register Addr:0x%02X%02X Data:0x%02X%02X successful\n", CmdBuf[3], CmdBuf[4], CmdBuf[5], CmdBuf[6]);
		return 0;
	}
}

bool set_fM34_bypass(int expect)
{
	struct fm34_conf *fm34_conf = fm34_client->dev.platform_data;
	struct fm34_drvdata *pfm34_drvdata = i2c_get_clientdata(fm34_client);

	printk(KERN_DEBUG "set_fM34_bypass\n");

	if(pfm34_drvdata->fm34_status) {
		fm34_register_write(fm34_client, 0x2300, 0x0004);
		msleep(15);
		gpio_direction_output(fm34_conf->pwdn, 0);
		pfm34_drvdata->fm34_status = 0;
	}

	pfm34_drvdata->fm34_expect = expect;

	return 1;
}
EXPORT_SYMBOL(set_fM34_bypass);

bool get_fM34_status(void)
{
	struct fm34_drvdata *pfm34_drvdata = i2c_get_clientdata(fm34_client);

	return pfm34_drvdata->fm34_status;
}
EXPORT_SYMBOL(get_fM34_status);

void fm34_set_codec_link(void *codec, void (*codec_gain)(void *, int))
{
	struct fm34_drvdata *pfm34_drvdata = i2c_get_clientdata(fm34_client);

	pfm34_drvdata->codec = (struct snd_soc_codec *)codec;
	pfm34_drvdata->codec_gain = codec_gain;
}
EXPORT_SYMBOL(fm34_set_codec_link);

static bool set_fm34_patch(struct i2c_client *client)
{
	u16 index;

	DEBUG("set_fm34_patch\n");

	for(index=0;index<sizeof(fm34_patch)/sizeof(u8)/9;index++) {
		i2c_master_send(client, &fm34_patch[index][1], fm34_patch[index][0]);
	}

	return 1;
}

static bool set_fm34_prop(struct i2c_client *client)
{
	struct fm34_conf *fm34_conf = fm34_client->dev.platform_data;
	u16 index, *pprop = (u16 *)fm34_conf->pprop;

	DEBUG("set_fm34_prop\n");
	if(pprop) {
		for(index=0; index<fm34_conf->cprop;index++) {
			fm34_register_write(client, *pprop, *(pprop+1));
			pprop += 2;
		}
	}

	return 1;
}

static bool set_fm34_prop2(struct i2c_client *client)
{
	struct fm34_conf *fm34_conf = fm34_client->dev.platform_data;
	u16 index, *pprop2 = (u16 *)fm34_conf->pprop2;

	DEBUG("set_fm34_prop2\n");
	if(pprop2) {
		for(index=0; index<fm34_conf->cprop2;index++) {
			fm34_register_write(client, *pprop2, *(pprop2+1));
			pprop2 += 2;
		}
	}

	return 1;
}

static bool set_fm34_prop3(struct i2c_client *client)
{
	struct fm34_conf *fm34_conf = fm34_client->dev.platform_data;
	u16 index, *pprop3 = (u16 *)fm34_conf->pprop3;

	DEBUG("set_fm34_prop3\n");
	if(pprop3) {
		for(index=0; index<fm34_conf->cprop3;index++) {
			fm34_register_write(client, *pprop3, *(pprop3+1));
			pprop3 += 2;
		}
	}

	return 1;
}

bool set_fM34_echo(int prop)
{
        struct fm34_conf *fm34_conf = fm34_client->dev.platform_data;
        struct fm34_drvdata *pfm34_drvdata = i2c_get_clientdata(fm34_client);

        pfm34_drvdata->fm34_expect = 1;
        if(!pfm34_drvdata->fm34_enable) {
                DEBUG("cancel set_fM34_echo\n");
                return 0;
        }
        gpio_direction_output(fm34_conf->pwdn, 1);
        gpio_direction_output(fm34_conf->bp, 1);
        gpio_direction_output(fm34_conf->rst, 0);
        msleep(10);
        gpio_direction_output(fm34_conf->rst, 1);
        msleep(15);

        switch(prop) {
		case ECHO_CANCELLATION:
			printk(KERN_DEBUG "set_fM34_echo\n");
			set_fm34_prop(fm34_client);
			break;
		case NOISE_SUPPRESSION:
			printk(KERN_DEBUG "set_fM34_noise_suppress\n");
			set_fm34_prop2(fm34_client);
			break;
		case CAMCORDING:
			printk(KERN_DEBUG "set_fM34_camcording\n");
			set_fm34_prop3(fm34_client);
			break;
		default:
			break;
		}

        if(0 == pfm34_drvdata->fm34_status) {
                fm34_register_write(fm34_client, 0x2300, 0x0000);
                pfm34_drvdata->fm34_status = 1;
        }

        return 1;
}
EXPORT_SYMBOL(set_fM34_echo);

int fm34_enable_get(void)
{
	struct fm34_drvdata *pfm34_drvdata = i2c_get_clientdata(fm34_client);
	
	return pfm34_drvdata->fm34_enable;
}

ssize_t fm34_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct fm34_drvdata *pfm34_drvdata = i2c_get_clientdata(fm34_client);

	if(buf[0] == '0')
		pfm34_drvdata->fm34_enable = 0;
	else
		pfm34_drvdata->fm34_enable = 1;

	if(pfm34_drvdata->fm34_enable && pfm34_drvdata->fm34_expect && 
		(pfm34_drvdata->codec && pfm34_drvdata->codec_gain) ) {
		if ( set_fM34_echo(ECHO_CANCELLATION) )
			pfm34_drvdata->codec_gain(pfm34_drvdata->codec, true );
	}
	else {
		set_fM34_bypass(pfm34_drvdata->fm34_expect);
		if(pfm34_drvdata->codec && pfm34_drvdata->codec_gain)
			pfm34_drvdata->codec_gain(pfm34_drvdata->codec, false );
	}
	return len;
}

ssize_t fm34_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct fm34_drvdata *pfm34_drvdata = i2c_get_clientdata(fm34_client);

	sprintf(buf, "%d\n", pfm34_drvdata->fm34_enable);
	return strlen(buf);
}
DEVICE_ATTR(fm34_enable, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, fm34_enable_show, fm34_enable_store);

#if FM34_REG_RW
ssize_t fm34_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
    int reg, val;
    int pwr_en = 0;
    struct fm34_conf *fm34_conf = fm34_client->dev.platform_data;
    struct fm34_drvdata *pfm34_drvdata = i2c_get_clientdata(fm34_client);
	u16 index, *pprop = (u16 *)fm34_conf->pprop;

    if(len != 14){
        pr_err("%s: command length wrong\n", __func__);
        return -EINVAL;
    }

    sscanf(buf ,"%x %x", &reg, &val);

	for(index=0; index<fm34_conf->cprop;index++) {
		if ( *pprop == reg )
			*(pprop+1) = val;
		pprop += 2;
	}

	pprop = (u16 *)fm34_conf->pprop2;
	for(index=0; index<fm34_conf->cprop2;index++) {
		if ( *pprop == reg )
			*(pprop+1) = val;
		pprop += 2;
	}

	pprop = (u16 *)fm34_conf->pprop3;
	for(index=0; index<fm34_conf->cprop3;index++) {
		if ( *pprop == reg )
			*(pprop+1) = val;
		pprop += 2;
	}

    if(0 == pfm34_drvdata->fm34_status) {
        gpio_direction_output(fm34_conf->pwdn, 1);
        msleep(15);
        pwr_en = 1;
    }

    fm34_register_write(fm34_client, reg, val);

    if(pwr_en == 1) {
        gpio_direction_output(fm34_conf->pwdn, 0);
    }

    return len;
}

const static u16 fm34_prop_show[] = {
	0x22C6,
	0x22C7,
	0x22C8,
	0x22D2,
	0x22E3,
	0x22EE,
	0x22F2,
	0x22F6,
	0x22F8,
	0x22F9,
	0x22FA,
	0x22FB,
	0x2300,	// Bypass enable/disable
	0x2301,
	0x2303,
	0x2304,
	0x2305,
	0x2306,
	0x2307,	// Entry MIC in Gain
	0x2309,
	0x230C,	// Leave MIC In Gain
	0x230D,	// Speaker Out Gain
	0x2310,
	0x232F,
	0x2339,
	0x233A,
	0x233B,
	0x233C,
	0x2348,
	0x2349,
	0x2357,
	0x236E,
	0x236F,
	0x2370,
	0x2371,
	0x2390,
	0x23D7,
	0x23CF,
	0x23D5,
	0x23E0,
	0x23E1,
	0x23E2,
	0x23E3,
	0x23E4,
	0x3FAF,
	0x3FBF,
};

ssize_t fm34_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    u16 regValue;

    struct fm34_conf *fm34_conf = fm34_client->dev.platform_data;
    struct fm34_drvdata *pfm34_drvdata = i2c_get_clientdata(fm34_client);
    int pwr_en = 0;
    char tmp[32] = "";
    u16 index;

    if(0 == pfm34_drvdata->fm34_status) {
        gpio_direction_output(fm34_conf->pwdn, 1);
        msleep(15);
        pwr_en = 1;
    }

    for(index=0;index<sizeof(fm34_prop_show)/sizeof(u16);index++) {
        fm34_register_read(fm34_client, fm34_prop_show[index], &regValue);
        sprintf(tmp, "0x%x : 0x%x\n", fm34_prop_show[index], regValue);

        DEBUG("%s: CmdBuf[%d] = 0x%x, tmp=>%s<=\n", __func__, index, fm34_prop_show[index], tmp);

        strcat(buf, tmp);
        memset(tmp, 0, sizeof(tmp));
    }
	sprintf(tmp, "echo func ctl:%s\n", pfm34_drvdata->fm34_enable?"enable":"disable");
	strcat(buf, tmp);
	memset(tmp, 0, sizeof(tmp));
	sprintf(tmp, "echo func:    %s\n", pfm34_drvdata->fm34_status?"enable":"disable");
	strcat(buf, tmp);

    if(pwr_en == 1) {
        gpio_direction_output(fm34_conf->pwdn, 0);
    }

    return strlen(buf);
}
DEVICE_ATTR(fm34_reg, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, fm34_reg_show, fm34_reg_store);
#endif

static int __devinit fm34_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fm34_conf *fm34_conf = client->dev.platform_data;
	struct fm34_drvdata *pfm34_drvdata;
	u16 sysReady;
	int err;

	fm34_client = client;

	pfm34_drvdata = kzalloc(sizeof(struct fm34_drvdata), GFP_KERNEL);
	if (pfm34_drvdata == NULL)
		return -ENOMEM;
	pfm34_drvdata->fm34_enable = 1;
	pfm34_drvdata->fm34_status = 1;
	i2c_set_clientdata(fm34_client, pfm34_drvdata);

	DEBUG("%s: pwdn gpio = %d\n", __func__, fm34_conf->pwdn);
	DEBUG("%s: bp gpio = %d\n", __func__, fm34_conf->bp);
	DEBUG("%s: rst gpio = %d\n", __func__, fm34_conf->rst);

	err = gpio_request(fm34_conf->pwdn, "fm34_pwdn");
	if (err < 0) {
		pr_err("%s: gpio_request failed for gpio %d\n", __func__, fm34_conf->pwdn);
		return err;
	}
	gpio_direction_output(fm34_conf->pwdn, 1);
	gpio_export(fm34_conf->pwdn, false);

	if(fm34_conf->bp != -1) {
		err = gpio_request(fm34_conf->bp, "fm34_bp");
		if (err < 0) {
			pr_err("%s: gpio_request failed for gpio %d\n", __func__, fm34_conf->bp);
			return err;
		}
		gpio_direction_output(fm34_conf->bp, 1);
		gpio_export(fm34_conf->bp, false);
	}
	if(fm34_conf->rst != -1) {
		err = gpio_request(fm34_conf->rst, "fm34_rst");
		if (err < 0) {
			pr_err("%s: gpio_request failed for gpio %d\n", __func__, fm34_conf->rst);
			return err;
		}
		gpio_direction_output(fm34_conf->rst, 0);
		gpio_export(fm34_conf->rst, false);
		msleep(10); //0.01 second
	}

	#if 0
	dap_mclk = i2s_get_clock_by_name("extern1");
	if (!dap_mclk) {
		pr_err("%s: could not get DAP clock\n", __func__);
		kfree(pfm34_drvdata);
		return -EIO;
	}
	clk_enable(dap_mclk);
	#endif

	if(fm34_conf->rst != -1) {
		gpio_direction_output(fm34_conf->rst, 1);
	}

	msleep(15);
	err = fm34_register_read(client, 0x22FB, &sysReady);
	if (err < 0) {
		pr_err("%s: dsp probe failed\n", __FUNCTION__);
		goto pwr_gpio_fail;
	}

	if(sysReady != 0x5A5A) {
		set_fm34_patch(client);
		set_fm34_prop(client);
		msleep(250);
		fm34_register_read(client, 0x22FB, &sysReady);

		printk("%s: read 0x22FB = %x\n", __FUNCTION__, sysReady);
		if(sysReady != 0x5A5A) {
			err = -EFAULT;
			goto pwr_gpio_fail;
		}
	}

	set_fM34_bypass(0);

	err = device_create_file(&client->dev, &dev_attr_fm34_enable);
	if (err) {
		pr_err("%s: add_sysfs_entry fm34_enable failed\n", __FUNCTION__);
	}

#if FM34_REG_RW
	err = device_create_file(&client->dev, &dev_attr_fm34_reg);
	if (err) {
		pr_err("%s: add_sysfs_entry fm34_reg failed\n", __FUNCTION__);
	}
#endif

	printk(KERN_INFO "%s successfully registered\n", client->driver->driver.name);
	return err;

pwr_gpio_fail:
	gpio_free(fm34_conf->pwdn);
	if(fm34_conf->rst != -1)
		gpio_free(fm34_conf->rst);
	if(fm34_conf->bp != -1)
		gpio_free(fm34_conf->bp);

	return err;
}

static int __devexit fm34_i2c_remove(struct i2c_client *client)
{
	struct fm34_conf *fm34_conf = client->dev.platform_data;
	struct fm34_drvdata *pfm34_drvdata;

	pfm34_drvdata = i2c_get_clientdata(client);
	kfree(pfm34_drvdata);

	gpio_free(fm34_conf->pwdn);
	if(fm34_conf->rst != -1)
		gpio_free(fm34_conf->rst);
	if(fm34_conf->bp != -1)
		gpio_free(fm34_conf->bp);

	return 0;
}

static const struct i2c_device_id fm34_i2c_id[] = {
	{ "fm34_i2c", 0 },
	{ }
};

static struct i2c_driver fm34_i2c_driver = {
    .probe   = fm34_i2c_probe,
    .remove  = fm34_i2c_remove,
	.id_table	= fm34_i2c_id,
    .driver  = {
        .name   = "FM34",
    },
};

static int __init fm34_i2c_init(void) {
    int e;

	e = i2c_add_driver(&fm34_i2c_driver);
	if (e != 0) {
		pr_err("%s: failed to register with I2C bus with "
		       "error: 0x%x\n", __func__, e);
	}
	return e;
}

static void __exit fm34_i2c_exit(void) {

    i2c_del_driver(&fm34_i2c_driver);
}

module_init(fm34_i2c_init);
module_exit(fm34_i2c_exit);

MODULE_DESCRIPTION("Fortemedia FM34 acoustic echo cancellation driver");

