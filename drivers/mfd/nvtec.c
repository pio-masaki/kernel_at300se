#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/notifier.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#include <linux/mfd/core.h>
#include <linux/mfd/nvtec.h>
#include <linux/nvcommon.h>

#include <linux/wakelock.h>
#include <asm/mach-types.h>

#define NVTEC_MAGIC_ENTER_FLASH_MODE_DEFAULT    (0xAA55)
#define NVTEC_MAGIC_ENTER_FLASH_MODE_CONFIG     (0xEFBE)
#define NVTEC_MAGIC_EXIT_FLASH_MODE             (0xAA55)

#define NVTEC_MAX_FW_SIZE               32768 /* 32K */
#define NVTEC_I2C_MAXLEN	32

#define NVTEC_MCU_CMD_STR_CHG_LIMIT		"chg_limit"
#define NVTEC_MCU_CMD_STR_SHIP_MODE		"ship_mode"
#define NVTEC_MCU_CMD_STR_RESET_MCU		"reset_mcu"
#define NVTEC_MCU_CMD_STR_CUR_LEVEL		"cur_lv"
#define NVTEC_MCU_CMD_STR_TCUR_LEVEL	"tcur_lv"
#define NVTEC_MCU_CMD_STR_DUMP_ALL		"dump"

struct nvtec {
    struct mutex    lock;
    struct device   *dev;
    struct i2c_client *client;
    struct miscdevice miscdev;

    int request_pin;
    int pwr_state_pin;
    int ap_wake_pin;

    int flash_mode;
    int state;

    struct blocking_notifier_head system_notifier;
    struct blocking_notifier_head battery_notifier;

    wait_queue_head_t wq_ack;
    struct wake_lock upgrade_lock;
};

enum nvtec_state {
    NVTEC_STATE_S0 = 0,
    NVTEC_STATE_S1 = 1,
    NVTEC_STATE_S2 = 2,
    NVTEC_STATE_S3 = 3,
    NVTEC_STATE_S4 = 4,

    NVTEC_STATE_FACTORY = 0xe0,
    NVTEC_STATE_TEST = 0xf0,
    NVTEC_STATE_RESET = 0xff,
};

struct mcu_info{
    uint16_t raw;
    char *name;
};

static unsigned int percentage = 0;
static int cmd_ack = 0;

static int nvtec_set_state(struct i2c_client *client, uint16_t state);
static int nvtec_read_event(struct i2c_client *client, uint16_t *event);

static int __nvtec_read_byte(struct i2c_client *client, int reg, uint8_t *val)
{
    int ret = 0;
    int retry = 5;
    
    while (retry --) {
        ret = i2c_smbus_read_byte_data(client, reg);
        if (ret < 0) {
            dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    if (ret >= 0)
        *val = (uint8_t) ret;

    return (ret < 0) ? ret : 0;
}

static int nvtec_read_byte_lock(struct i2c_client *client, int reg, uint8_t *val)
{
    struct nvtec *ec = i2c_get_clientdata(client);
    int ret;

    mutex_lock(&ec->lock);
    ret = __nvtec_read_byte(client, reg, val);
    mutex_unlock(&ec->lock);

    return ret;
}

static int __nvtec_read_word(struct i2c_client *client, int reg, uint16_t *val)
{
    int ret = 0;
    int retry = 5;
    
    while (retry --) {
        ret = i2c_smbus_read_word_data(client, reg);
        if (ret < 0) {
            dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    if (ret >= 0)
        *val = (uint16_t) ret;

    return (ret < 0) ? ret : 0;
}

static int nvtec_read_word_lock(struct i2c_client *client, int reg, uint16_t *val)
{
    struct nvtec *ec = i2c_get_clientdata(client);
    int ret;

    mutex_lock(&ec->lock);
    ret = __nvtec_read_word(client, reg, val);
    mutex_unlock(&ec->lock);

    return ret;
}

static int __nvtec_read_block(struct i2c_client *client, int reg, 
                                     int len, uint8_t *val)
{
    int ret = 0;
    int retry = 5;

    while (retry --) {
        ret = i2c_smbus_read_i2c_block_data(client, reg, len, val);
        if (ret < 0) {
            dev_err(&client->dev, "failed reading from 0x%02x\n", reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    return (ret < 0) ? ret : 0;
}

static int nvtec_read_block_lock(struct i2c_client *client, int reg,
                                 int len, uint8_t *val)
{
    struct nvtec *ec = i2c_get_clientdata(client);
    int ret;

    mutex_lock(&ec->lock);
    ret = __nvtec_read_block(client, reg, len, val);
    mutex_unlock(&ec->lock);

    return ret;
}

static int __nvtec_write_byte(struct i2c_client *client, int reg, uint8_t val)
{
    int ret = 0;
    int retry = 5;

    while (retry --) {
        ret = i2c_smbus_write_byte_data(client, reg, val);
        if (ret < 0) {
            dev_err(&client->dev, "failed writing 0x%02x ot 0x%02x\n",
                    val, reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    return (ret < 0) ? ret : 0;
}

static int nvtec_write_byte_lock(struct i2c_client *client, int reg, uint8_t val)
{
    struct nvtec *ec = i2c_get_clientdata(client);
    int ret;

    mutex_lock(&ec->lock);
    ret = __nvtec_write_byte(client, reg, val);
    mutex_unlock(&ec->lock);

    return ret;
}

static int __nvtec_write_word(struct i2c_client *client, int reg, uint16_t val)
{
    int ret = 0;
    int retry = 5;

    while (retry --) {
        ret = i2c_smbus_write_word_data(client, reg, val);
        if (ret < 0) {
            dev_err(&client->dev, "failed writing 0x%04x to 0x%02x\n",
                    val, reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    return (ret < 0) ? ret : 0;
}

static int nvtec_write_word_lock(struct i2c_client *client, int reg, uint16_t val)
{
    struct nvtec *ec = i2c_get_clientdata(client);
    int ret;

    mutex_lock(&ec->lock);
    ret = __nvtec_write_word(client, reg, val);
    mutex_unlock(&ec->lock);

    return ret;
}

static int __nvtec_write_block(struct i2c_client *client, int reg,
                                      int len, uint8_t *val)
{
    int ret = 0;
    int retry = 5;

    while (retry --) {
        ret = i2c_smbus_write_block_data(client, reg, len, val);
        if (ret < 0) {
            dev_err(&client->dev, "failed write to 0x%02x\n", reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    return (ret < 0) ? ret : 0;
}

static int nvtec_write_block_lock(struct i2c_client *client, int reg,
                                  int len, uint8_t *val)
{
    struct nvtec *ec = i2c_get_clientdata(client);
    int ret;

    mutex_lock(&ec->lock);
    ret = __nvtec_write_block(client, reg, len, val);
    mutex_unlock(&ec->lock);

    return ret;
}

int nvtec_read_byte(struct device *dev, int reg, uint8_t *val)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct nvtec *ec = i2c_get_clientdata(client);

    if (ec->state != NVTEC_STATE_S0 ||
        ec->flash_mode == 1)
        return -EINVAL;

    return nvtec_read_byte_lock(client, reg, val);
}
EXPORT_SYMBOL_GPL(nvtec_read_byte);

int nvtec_read_word(struct device *dev, int reg, uint16_t *val)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct nvtec *ec = i2c_get_clientdata(client);

    if (ec->state != NVTEC_STATE_S0 ||
        ec->flash_mode == 1)
        return -EINVAL;

    return nvtec_read_word_lock(client, reg, val);
}
EXPORT_SYMBOL_GPL(nvtec_read_word);

int nvtec_read_block(struct device *dev, int reg, int len, uint8_t *val)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct nvtec *ec = i2c_get_clientdata(client);

    if (ec->state != NVTEC_STATE_S0 ||
        ec->flash_mode == 1)
        return -EINVAL;

    return nvtec_read_block_lock(client, reg, len, val);
}
EXPORT_SYMBOL_GPL(nvtec_read_block);

int nvtec_write_byte(struct device *dev, int reg, uint8_t val)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct nvtec *ec = i2c_get_clientdata(client);

    if (ec->state != NVTEC_STATE_S0 ||
        ec->flash_mode == 1)
        return -EINVAL;

    return nvtec_write_byte_lock(client, reg, val);
}
EXPORT_SYMBOL_GPL(nvtec_write_byte);

int nvtec_write_word(struct device *dev, int reg, uint16_t val)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct nvtec *ec = i2c_get_clientdata(client);

    if (ec->state != NVTEC_STATE_S0 ||
        ec->flash_mode == 1)
        return -EINVAL;

    return nvtec_write_word_lock(client, reg, val);
}
EXPORT_SYMBOL_GPL(nvtec_write_word);

int nvtec_write_block(struct device *dev, int reg, int len, uint8_t *val)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct nvtec *ec = i2c_get_clientdata(client);

    if (ec->state != NVTEC_STATE_S0 ||
        ec->flash_mode == 1)
        return -EINVAL;

    return nvtec_write_block_lock(client, reg, len, val);
}
EXPORT_SYMBOL_GPL(nvtec_write_block);

int nvtec_register_event_notifier(struct device *dev, 
                                  enum nvtec_event_type event,
                                  struct notifier_block *nb)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct nvtec *ec = i2c_get_clientdata(client);
    int ret;

    switch (event) {
    case NVTEC_EVENT_SYSTEM:
        ret = blocking_notifier_chain_register(&ec->system_notifier, nb);
        break;
    case NVTEC_EVENT_BATTERY:
        ret = blocking_notifier_chain_register(&ec->battery_notifier, nb);
        break;
    default:
        ret = -EINVAL;
        break;
    }
    
    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_register_event_notifier);

int nvtec_unregister_event_notifier(struct device *dev,
                                    enum nvtec_event_type event,
                                    struct notifier_block *nb)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct nvtec *ec = i2c_get_clientdata(client);
    int ret;

    switch (event) {
    case NVTEC_EVENT_SYSTEM:
        ret = blocking_notifier_chain_unregister(&ec->system_notifier, nb);
        break;
    case NVTEC_EVENT_BATTERY:
        ret = blocking_notifier_chain_unregister(&ec->battery_notifier, nb);
        break;
    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_unregister_event_notifier);

static int nvtec_get_charger_id(struct i2c_client *client, uint16_t *chg_ver)
{
    return nvtec_read_word_lock(client, 0xc3, chg_ver);
}

static int nvtec_get_firmware_version(struct i2c_client *client, uint8_t *ver_str)
{
    int ret;
    char version[32];
    int len;

    ret = nvtec_read_block_lock(client, 0xC0, 32, version);
    if (ret < 0)
        return ret;

    len = (version[0] >= 31) ? 31 : version[0];
    strncpy(ver_str, &version[1], len);
    ver_str[len] = '\0';

    if ((ver_str[15]=='5') && (ver_str[16] =='A')) {
        dev_info(&client->dev, "MCU chip: M0516LBN 64KB \n");
    } else if ((ver_str[15]=='5') && (ver_str[16] =='8')) {
        dev_info(&client->dev, "MCU chip: M058LBN 32KB \n");
    } else {
        dev_err(&client->dev, "Unknow MCU Chip Set to default %d KB\n",
                NVTEC_MAX_FW_SIZE/1024);
    }

    return ret;
}

static int nvtec_reset_mcu(struct i2c_client *client)
{
    return nvtec_write_word_lock(client, 0xe5, 0x1);
}

static int nvtec_get_shipping_mode(struct i2c_client *client, uint16_t *mode)
{
    return nvtec_read_word_lock(client, 0xe4, mode);
}

static int nvtec_set_shipping_mode(struct i2c_client *client, int mode)
{
    int val;

	if (mode == 1 || mode == -1)
		val = 1;
	else if (mode == 0)
		val = 0;
	else
		return -EINVAL;

    return nvtec_write_word_lock(client, 0xe4, val);
}

static int nvtec_get_charging_limit(struct i2c_client *client, uint16_t *value)
{
    return nvtec_read_word_lock(client, 0xe3, value);
}

static int nvtec_set_charging_limit(struct i2c_client *client, int value)
{
	if (value < 50 || value > 100)
		return -EINVAL;

    return nvtec_write_word_lock(client, 0xe3, value);
}

static int nvtec_get_temporary_current_level(struct i2c_client *client, uint16_t *value)
{
	int ret;
	uint16_t val;

    ret = nvtec_read_word_lock(client, 0xd4, &val);

	if (ret == 0)
	{
		switch(val)
		{
			case 0x0:
				*value = 500;
				break;

			case 0x1:
				*value = 900;
				break;

			case 0x2:
				*value = 1500;
				break;

			case 0x3:
				*value = 2000;
				break;

			default:
				*value = 0;
		}
	}

	return ret;
}

static int nvtec_set_temporary_current_level(struct i2c_client *client, int value)
{
	uint16_t val;

	switch(value)
	{
		case 500:
			val = 0x0;
			break;

		case 900:
			val = 0x1;
			break;

		case 1500:
			val = 0x2;
			break;

		case 2000:
			val = 0x3;
			break;

		default:
			return -EINVAL;
	}

    return nvtec_write_word_lock(client, 0xd4, val);
}

static int nvtec_get_current_level(struct i2c_client *client, uint16_t *value)
{
	int ret;
	uint16_t val;

    ret = nvtec_read_word_lock(client, 0xe8, &val);

	if (ret == 0)
	{
		switch(val)
		{
			case 0x0:
				*value = 500;
				break;

			case 0x1:
				*value = 900;
				break;

			case 0x2:
				*value = 1500;
				break;

			case 0x3:
				*value = 2000;
				break;

			default:
				*value = 0;
		}
	}

	return ret;
}

static int nvtec_set_current_level(struct i2c_client *client, int value)
{
	uint16_t val;

	switch(value)
	{
		case 500:
			val = 0x10;
			break;

		case 900:
			val = 0x11;
			break;

		case 1500:
			val = 0x12;
			break;

		case 2000:
			val = 0x13;
			break;

		default:
			return -EINVAL;
	}

    return nvtec_write_word_lock(client, 0xe8, val);
}

static int nvtec_enter_flash_mode(struct i2c_client *client, uint16_t magic)
{
    struct nvtec *ec = i2c_get_clientdata(client);
    uint16_t event;
    int ret;

    percentage = 0;

    if (ec->flash_mode) {
        dev_warn(&client->dev, "already in flash mode\n");
        return 0;
    }

    if (ec->state != NVTEC_STATE_S0) {
        dev_err(&client->dev, "EC not in S0 state\n");
        return -EINVAL;
    }

    if (magic != NVTEC_MAGIC_ENTER_FLASH_MODE_DEFAULT &&
        magic != NVTEC_MAGIC_ENTER_FLASH_MODE_CONFIG) {
        dev_err(&client->dev, "Invalid magic number\n");
        return -EINVAL;
    }

    wake_lock(&ec->upgrade_lock);
    dev_info(&client->dev, "upgrade wake_lock locked\n");

    suspend_battery_thread();

    mutex_lock(&ec->lock);

    /* clean event */
    while (gpio_get_value(ec->ap_wake_pin) == 0) {
        ret = nvtec_read_event(client, &event);
        if (ret < 0 || event == 0)
            break;
    }

    dev_info(&client->dev, "Enter flash mode\n");
    ec->flash_mode = 1;
    
    cmd_ack = 0;
    ret= __nvtec_write_word(client, 0xe0, magic);
    if (ret < 0) {
        dev_err(&client->dev, "Error set magic number\n");
        goto error;
    }

    dev_info(&client->dev, "Waiting for MCU acknowledge\n");
    wait_event(ec->wq_ack, cmd_ack == 1);
	dev_info(&client->dev, "MCU Ack OK...continue...\n");

    mutex_unlock(&ec->lock);
    return 0;
 error:
    ec->flash_mode = 0;
    mutex_unlock(&ec->lock);
    wake_unlock(&ec->upgrade_lock);
    return ret;
}

static int nvtec_exit_flash_mode(struct i2c_client *client, uint16_t magic)
{
    struct nvtec *ec = i2c_get_clientdata(client);
    int ret;

    if (ec->flash_mode == 0)
        return -EINVAL;

    if (magic != NVTEC_MAGIC_EXIT_FLASH_MODE) {
        dev_err(&client->dev, "Invalid magic number\n");
        return -EINVAL;
    }

    mutex_lock(&ec->lock);

    cmd_ack = 0;
    ret = __nvtec_write_word(client, 0xe3, magic);
    if (ret < 0) {
        mutex_unlock(&ec->lock);
        return ret;
    }

    wait_event(ec->wq_ack, cmd_ack == 1);

    dev_info(&client->dev, "Exit flash mode\n");
    
    ec->flash_mode = 0;

    mutex_unlock(&ec->lock);

    /* EC will reset itself when exit flash mode */
    /* after reset, EC at S0 state Scorpio */
    ec->state = NVTEC_STATE_S0;

    msleep(100); /* wait EC reset */

    resume_battery_thread();

    wake_unlock(&ec->upgrade_lock);
    dev_info(&client->dev, "upgrade wake_lock unlocked\n");

    return 0;
}

static int nvtec_get_flash_id(struct i2c_client *client, uint8_t *id)
{
    return nvtec_read_block_lock(client, 0xc1, 32, id);
}

static int nvtec_get_state(struct i2c_client *client, uint16_t *state)
{
    return nvtec_read_word_lock(client, 0xd1, state);
}

static int nvtec_set_state(struct i2c_client *client, uint16_t state)
{
    struct nvtec *ec = i2c_get_clientdata(client);
    int ret = 0;

    if (state > NVTEC_STATE_S4 &&
        state != NVTEC_STATE_TEST &&
        state != NVTEC_STATE_RESET &&
        state != NVTEC_STATE_FACTORY)
        return -EINVAL;

    mutex_lock(&ec->lock);

    if (ec->state == NVTEC_STATE_S0) {
        if (state == NVTEC_STATE_S3) {
            /* Suspend 	1> send cmd 2> trigger gpio (EC_1 = 1->0  EC_2 = 1->1) */
            ret = __nvtec_write_word(client, 0xd1, state);
            gpio_set_value(ec->request_pin, 0);
            gpio_set_value(ec->pwr_state_pin, 1);
            dev_info(&client->dev, "Enter suspend state (S3)\n");
        } else if (state == NVTEC_STATE_S4) {
            /* Power off	1> send cmd 2> trigger gpio (EC_1 = 1->0  EC_2 = 1->0) */
            //ret = __nvtec_write_word(client, 0xd1, state);
            gpio_set_value(ec->request_pin, 0);
            gpio_set_value(ec->pwr_state_pin, 0);
            dev_info(&client->dev, "Enter power off state (S4)\n");
        } else {
            ret = __nvtec_write_word(client, 0xd1, state);
            dev_warn(&client->dev, "Invalid State (S%d -> S%d)\n", ec->state, state);
            mutex_unlock(&ec->lock);
            return ret;
        }
    } else {
        if (state == NVTEC_STATE_S0) {
            /* Resume   1> trigger gpio 2> send cmd (EC_1 = 0->1  EC_2 = 1->1) */
            gpio_set_value(ec->request_pin, 1);
            gpio_set_value(ec->pwr_state_pin, 1);
            //ret = __nvtec_write_word(client, 0xd1, state);
            msleep(30);
            dev_info(&client->dev, "Enter operation state (S0)\n");
        } else {
            dev_warn(&client->dev, "Invalid state (S%d -> S%d)\n", ec->state, state);
            ret = __nvtec_write_word(client, 0xd1, state);
            mutex_unlock(&ec->lock);
            return ret;
        }
    }
    ec->state = state;
    mutex_unlock(&ec->lock);
    return ret;
}

static int nvtec_read_gpio_input(struct i2c_client *client, uint8_t port, uint8_t *val)
{
    if (port > 9)
        return -EINVAL;

    return nvtec_read_byte_lock(client, 0xa0 + port, val);
}

static int nvtec_read_gpio_output(struct i2c_client *client, uint8_t port, uint8_t *val)
{
    if (port > 9)
        return -EINVAL;

    return nvtec_read_byte_lock(client, 0xb0 + port, val);
}

static int nvtec_set_gpio_output(struct i2c_client *client, uint8_t port, uint8_t val)
{
    if (port > 9)
        return -EINVAL;

    return nvtec_write_byte_lock(client, 0xb0 + port, val);
}

static inline void show_progress(int percent)
{
    static int prev_percent = -1;
    int i;
    int c;

    if (prev_percent == percent)
        return;

    prev_percent = percent;
    c = (percent * 50) / 100;

    // Show the percentage complete.
    printk("nvtec: %3d%% [", percent );

    // Show the prgress bar.
    for (i = 0; i < c; i++)
       printk("=");

    for (i = c; i < 50; i++)
       printk(" ");

    printk("]\r");
}

static int nvtec_transfer_firmware(struct i2c_client *client, uint8_t *buffer, int len)
{
    struct nvtec *ec = i2c_get_clientdata(client);
    int ret;
    int byte_transferred = 0;
    int transfer_len = 0;
    
    if (NULL == buffer || len != NVTEC_MAX_FW_SIZE)
        return -EINVAL;
    
    if (ec->flash_mode == 0)
        return -EINVAL;

    mutex_lock(&ec->lock);
    while (byte_transferred != len) {
        if ((len - byte_transferred) >= I2C_SMBUS_BLOCK_MAX)
            transfer_len = I2C_SMBUS_BLOCK_MAX;
        else
            transfer_len = len - byte_transferred;

        cmd_ack = 0;
        ret = __nvtec_write_block(client, 0xe2,
                                  transfer_len,
                                  &buffer[byte_transferred]);

        if (ret != 0) {
            dev_err(&client->dev, "Transfer firmware fail!\n");
            mutex_unlock(&ec->lock);
            return -EINVAL;
        }

        byte_transferred += transfer_len;
        
        percentage=((byte_transferred * 100) / NVTEC_MAX_FW_SIZE);
        show_progress(percentage);

        wait_event(ec->wq_ack, cmd_ack == 1);
    }

    mutex_unlock(&ec->lock);

    return len;
}

static int nvtec_flash_acknowledge(struct i2c_client *client)
{
    struct nvtec *ec = i2c_get_clientdata(client);
    uint8_t value;
    int ret;

    /* no need protect */
    ret = __nvtec_read_byte(client, 0x0, &value);
    if (ret < 0)
        return -EFAULT;

    if (value == 0xfa) {
        cmd_ack = 1;
        wake_up(&ec->wq_ack);
        return 0;
    } else
        return -EFAULT;
}

static int nvtec_read_event(struct i2c_client *client, uint16_t *event)
{
    /* no need protect */
    return __nvtec_read_word(client, 0xd0, event);
}

static int __remove_subdev(struct device *dev, void *unused)
{
    platform_device_unregister(to_platform_device(dev));
    return 0;
}

static int nvtec_remove_subdevs(struct nvtec *ec)
{
    return device_for_each_child(ec->dev, NULL, __remove_subdev);
}

static int __devinit nvtec_add_subdevs(struct nvtec *ec, struct nvtec_platform_data *pdata)
{
    struct nvtec_subdev_info *subdev;
    struct platform_device *pdev;
    int i, ret = 0;

    for (i = 0; i < pdata->num_subdevs; i ++) {
        subdev = &pdata->subdevs[i];
        
        pdev = platform_device_alloc(subdev->name, subdev->id);

        pdev->dev.parent = ec->dev;
        pdev->dev.platform_data = subdev->platform_data;

        ret = platform_device_add(pdev);
        if (ret)
            goto failed;
    }
    return 0;

 failed:
    nvtec_remove_subdevs(ec);
    return ret;
}

static irqreturn_t nvtec_irq(int irq, void *data)
{
    struct nvtec *ec = data;
    uint16_t event = 0;
    uint8_t event_type = 0;
    int ret;
	
    if (ec->flash_mode) {
        ret = nvtec_flash_acknowledge(ec->client);
        return IRQ_HANDLED;
    }
 
    nvtec_read_event(ec->client, &event);
    event_type = (uint8_t)(event >> 8) & 0xff;

    switch (event_type) {
    case NVTEC_EVENT_BATTERY:
        blocking_notifier_call_chain(&ec->battery_notifier, event, NULL);
        break;
    case NVTEC_EVENT_SYSTEM:
        blocking_notifier_call_chain(&ec->system_notifier, event, NULL);
        break;
    default:
        break;
    }

    return IRQ_HANDLED;
}

static int __devinit nvtec_irq_init(struct nvtec *ec, int irq)
{
    int ret;

    ret = request_threaded_irq(irq, NULL, nvtec_irq, IRQF_TRIGGER_FALLING, 
                               "nvtec", ec);
    if (!ret) {
        device_init_wakeup(ec->dev, 1);
        enable_irq_wake(irq);
    }

    return ret;
}

static int nvtec_open(struct inode *inode, struct file *file)
{
    struct miscdevice *mdev = file->private_data;
    struct nvtec *ec = container_of(mdev, struct nvtec, miscdev);

    file->private_data = ec;
    return nonseekable_open(inode, file);
}

static ssize_t nvtec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    return -EINVAL;
}

static ssize_t nvtec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    struct nvtec *ec = file->private_data;
    int ret = 0;
    uint8_t *fw_buf;
    /* int i; */

    dev_info(ec->dev, "Start firmware upgrade\n");

    if (ec->flash_mode == 0) {
        dev_err(ec->dev, "Not in flash mode\n");
        return -EACCES;
    }
    
    if (count != NVTEC_MAX_FW_SIZE) {
        dev_err(ec->dev, "Invalid firmware size\n");
        return -EACCES;
    }

    dev_info(ec->dev, "Allocate firmware buffer!\n");
    fw_buf = kzalloc(NVTEC_MAX_FW_SIZE, GFP_KERNEL);
    if (!fw_buf) {
        dev_err(ec->dev, "Fail to allocate firmware buffer\n");
        return -ENOMEM;
    }
		
    memset(fw_buf, 0, NVTEC_MAX_FW_SIZE);

    dev_info(ec->dev, "Copy firmware to buffer\n");
    if (copy_from_user(fw_buf, buf, count))
        goto error;

    ret = nvtec_transfer_firmware(ec->client, fw_buf, count);
    if (ret != count) {
        dev_err(ec->dev, "Invalid firmware size\n");
        goto error;
    }

    kfree(fw_buf);
    return count;

 error:
    dev_err(ec->dev, "Fail to upgrade firmware\n");
    kfree(fw_buf);
    return -EINVAL;
}

struct flash_id {
    char _id[8];
};

#define NVTEC_IOCTL_ENTER_FLASH    _IOW('N', 1, unsigned short)
#define NVTEC_IOCTL_EXIT_FLASH     _IOW('N', 2, unsigned short)
#define NVTEC_IOCTL_SET_CONFIG     _IOW('N', 3, int)
#define NVTEC_IOCTL_GET_FLASH_ID   _IOR('N', 3, struct flash_id)
#define NVTEC_IOCTL_SET_STATE_CONFIG _IOW('N', 6, unsigned int)
#define NVTEC_IOCTL_GET_BOARD_ID   _IOR('N', 7, unsigned short)

static long nvtec_ioctl(struct file *filp, unsigned int cmd, 
                       unsigned long arg)
{
    struct nvtec *ec = filp->private_data;
    void __user *argp = (void __user *)arg;
    int ret = -EINVAL;
    uint16_t magic;
    uint16_t state_config;

    switch (cmd) {
    case NVTEC_IOCTL_ENTER_FLASH:
        ret = copy_from_user(&magic, argp, sizeof(unsigned short));
        if (ret)
            return -EFAULT;

        ret = nvtec_enter_flash_mode(ec->client, magic);
        if (ret < 0)
            return -EFAULT;
        
        ret = 0;
        break;
    case NVTEC_IOCTL_EXIT_FLASH:
        ret = copy_from_user(&magic, argp, sizeof(unsigned short));
        if (ret)
            return -EFAULT;

        ret = nvtec_exit_flash_mode(ec->client, magic);
        if (ret < 0)
            return -EFAULT;
        
        ret = 0;
        break;
    case NVTEC_IOCTL_SET_CONFIG:
        break;

    case NVTEC_IOCTL_GET_FLASH_ID:
        break;
    case NVTEC_IOCTL_SET_STATE_CONFIG:
    	ret = copy_from_user(&state_config, argp, sizeof(uint16_t));
        if (ret < 0)
            return -EFAULT;
        ret = nvtec_set_state(ec->client, state_config);
        if (ret < 0)
            return -EACCES;
        ret = 0;
    	break;
    default:
        break;
    }

    return ret;

}

#define NVTEC_MINOR   111
const struct file_operations nvtec_fops = {
    .owner = THIS_MODULE,
    .read = nvtec_read,
    .write = nvtec_write,
    .unlocked_ioctl = nvtec_ioctl,
    .open = nvtec_open,
};

static char *get_state_string(uint16_t state)
{
    switch (state) {
    case NVTEC_STATE_S0:
        return "S0";
    case NVTEC_STATE_S1:
        return "S1";
    case NVTEC_STATE_S2:
        return "S2";
    case NVTEC_STATE_S3:
        return "S3";
    case NVTEC_STATE_S4:
        return "S4";
    case NVTEC_STATE_TEST:
        return "Test";
    case NVTEC_STATE_RESET:
        return "Reset";
    case NVTEC_STATE_FACTORY:
        return "!QAZ2wsx"; 
    default:
        return NULL;
    }
}

static int str2state(const char *state_str)
{
    if (NULL == state_str)
        return -1;

    if(!strncmp("S0", state_str, 2))
        return NVTEC_STATE_S0;
    if(!strncmp("S1", state_str, 2))
        return NVTEC_STATE_S1;
    if(!strncmp("S2", state_str, 2))
        return NVTEC_STATE_S2;
    if(!strncmp("S3", state_str, 2))
        return NVTEC_STATE_S3;
    if(!strncmp("S4", state_str, 2))
        return NVTEC_STATE_S4;

    if(!strncmp("Test", state_str, 4))
        return NVTEC_STATE_TEST;
    if(!strncmp("Reset", state_str, 5))
        return NVTEC_STATE_RESET;
    if(!strncmp("!QAZ2wsx", state_str, 8))
        return NVTEC_STATE_FACTORY;

    return -1;
}

static ssize_t nvtec_state_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    struct miscdevice *mdev = dev_get_drvdata(dev);
    struct nvtec *ec = container_of(mdev, struct nvtec, miscdev);
    int ret;
    uint16_t state;

    ret = nvtec_get_state(ec->client, &state);
    if (ret < 0)
        return 0;

    return sprintf(buf, "%s\n", get_state_string(state));
}

static ssize_t nvtec_state_store(struct device *dev, 
                                 struct device_attribute *attr,
                                 const char *buf,
                                 size_t count)
{
    struct miscdevice *mdev = dev_get_drvdata(dev);
    struct nvtec *ec = container_of(mdev, struct nvtec, miscdev);
    int ret;
    int state;

    state = str2state(buf);
    dev_info(dev, "state = %d\n", state);
    if (state < 0)
        return 0;
        
    ret = nvtec_set_state(ec->client, (uint16_t) state);
    if (ret < 0)
        return 0;
    
    return count;
}

static ssize_t nvtec_charger_id_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    struct miscdevice *mdev = dev_get_drvdata(dev);
    struct nvtec *ec = container_of(mdev, struct nvtec, miscdev);
    int ret;
    uint16_t chg_id;

    ret = nvtec_get_charger_id(ec->client, &chg_id);
    if (ret < 0)
        return 0;

    return sprintf(buf, "0x%x\n", chg_id);
}

static ssize_t nvtec_version_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    struct miscdevice *mdev = dev_get_drvdata(dev);
    struct nvtec *ec = container_of(mdev, struct nvtec, miscdev);
    int ret;
    char version[32];
    
    ret = nvtec_get_firmware_version(ec->client, version);
    if (ret < 0)
        return 0;

    return sprintf(buf, "%s\n", version);
}

static ssize_t nvtec_flash_id_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    struct miscdevice *mdev = dev_get_drvdata(dev);
    struct nvtec *ec = container_of(mdev, struct nvtec, miscdev);
    int ret;
    char flash_id[32];
    
    ret = nvtec_get_flash_id(ec->client, flash_id);
    if (ret < 0)
        return 0;

    return sprintf(buf, "0x%02x 0x%02x 0x%02x 0x%02x\n0x%02x 0x%02x 0x%02x 0x%02x\n", 
                   flash_id[0], flash_id[1], flash_id[2], flash_id[3],
                   flash_id[4], flash_id[5], flash_id[6], flash_id[7]);
}

static ssize_t nvtec_gpio_input_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    struct miscdevice *mdev = dev_get_drvdata(dev);
    struct nvtec *ec = container_of(mdev, struct nvtec, miscdev);
    int ret;
    uint8_t value[10];
    char temp_buf[16];
    int i;

    for (i = 0; i < 10; i ++) {
        ret = nvtec_read_gpio_input(ec->client, i , &value[i]);
        if (ret < 0)
            return 0;

        sprintf(temp_buf, "P%d:0x%02x\n", i, value[i]);
        strcat(buf, temp_buf);
    }

    return strlen(buf);
}

static ssize_t nvtec_gpio_output_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    struct miscdevice *mdev = dev_get_drvdata(dev);
    struct nvtec *ec = container_of(mdev, struct nvtec, miscdev);
    int ret;
    uint8_t value[10];
    char temp_buf[16];
    int i;

    for (i = 0; i < 10; i ++) {
        ret = nvtec_read_gpio_output(ec->client, i, &value[i]);
        if (ret < 0)
            return 0;

        sprintf(temp_buf, "P%d:0x%02x\n", i, value[i]);
        strcat(buf, temp_buf);
    }

    return strlen(buf);
}

static ssize_t nvtec_gpio_output_store(struct device *dev, 
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t count)
{
    struct miscdevice *mdev = dev_get_drvdata(dev);
    struct nvtec *ec = container_of(mdev, struct nvtec, miscdev);
    int ret;
    char *cp, *next;
    int port;
    int value;
    
    next = (char *)buf;

    cp = strsep(&next, " ");
    port = simple_strtoul(cp, NULL, 10);

    cp = strsep(&next, " ");
    value = simple_strtoul(cp, NULL, 16);

    ret = nvtec_set_gpio_output(ec->client, (uint8_t) port, (uint8_t) value & 0xff);
    if (ret < 0) {
        dev_err(dev, "Can't set GPIO\n");
        return 0;
    }

    return count;
}

static ssize_t nvtec_cmd_write_store(struct device *dev, 
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
    struct miscdevice *mdev = dev_get_drvdata(dev);
    struct nvtec *ec = container_of(mdev, struct nvtec, miscdev);
    char *cp, *endp, *next;
    int cmd;
    uint8_t value[32];
    uint16_t word_value;
    int len = 0, i = 0;

    next = (char *)buf;
    cp = strsep(&next, " ");
    if (NULL == cp)
        return count;
    cmd = simple_strtoul(cp, &endp, 16);
    if (cp == endp || *endp != '\0') {
        return count;
    }

    cmd &= 0xff;

    while (len < 32 && cp != endp) {
           cp = strsep(&next, " ");
           if (NULL == cp) {
               break;
           }
           value[len++] = simple_strtoul(cp, &endp, 16);
    }

    printk("cmd:0x%02x, len=%d\n", cmd, len);
    if (len == 0)
        return count;

    for (i = 0; i < len; i++)
        printk("value[%d] = 0x%02x\n", i, value[i]);

    mutex_lock(&ec->lock);
    if (len == 1) {
        __nvtec_write_byte(ec->client, cmd, value[0]);
    }else if (len == 2) {
        word_value = value[1] << 8 | value[0];
        __nvtec_write_word(ec->client, cmd, word_value);
    } else {
        __nvtec_write_block(ec->client, cmd, len, value);
    }
    mutex_unlock(&ec->lock);
    
    return count;

}

static uint8_t g_byte;
static uint16_t g_word;
static uint8_t g_value[32];
static int read_type = 0;

static ssize_t nvtec_cmd_read_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    char temp_buf[32];
    int i;

    if (read_type == 0)
        return strlen(buf);
    else if (read_type == -1) {
        sprintf(buf, "read byte: 0x%02x\n", g_byte);
    } else if (read_type == -2) {
        sprintf(buf, "read word: 0x%04x\n", g_word);
    } else {
        sprintf(temp_buf, "read block:\n");
        strcat(buf, temp_buf);
        
        for (i = 0; i <= read_type; i ++) {
            sprintf(temp_buf, "value[%d]=0x%02x\n", i, g_value[i]);
            strcat(buf, temp_buf);
        }
    }
             
    return strlen(buf);
}

static ssize_t nvtec_cmd_read_store(struct device *dev, 
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    struct miscdevice *mdev = dev_get_drvdata(dev);
    struct nvtec *ec = container_of(mdev, struct nvtec, miscdev);
    char *cp, *endp, *next;
    int cmd;
    int i = 0;

    next = (char *)buf;
    cp = strsep(&next, " ");
    if (NULL == cp)
        return count;
    cmd = simple_strtoul(cp, &endp, 16);
    if (cp == endp || *endp != '\0') {
        return count;
    }

    cmd &= 0xff;
    printk("cmd: 0x%02x\n", cmd);

    cp = strsep(&next, " ");

    mutex_lock(&ec->lock);
    if (!strncmp(cp, "b", 1)) {
        __nvtec_read_byte(ec->client, cmd, &g_byte);
        printk("byte: 0x%02x\n", g_byte);
        read_type = -1;
    } else if (!strncmp(cp, "w", 1)) {
        __nvtec_read_word(ec->client, cmd, &g_word);
        printk("word: 0x%02x\n", g_word);
        read_type = -2;
    } else if  (!strncmp(cp, "B", 1)) {
        __nvtec_read_block(ec->client, cmd, 32, g_value);
        for (i = 0; i <= g_value[0]; i ++)
            printk("value[%d] = 0x%02x\n", i, g_value[i]);
        read_type = g_value[0];
    }
    mutex_unlock(&ec->lock);

    return count;
}

static ssize_t nvtec_upgrade_show(struct device *dev,
                                struct device_attribute *attr,
				char *buf)
{
    return sprintf(buf, "%d\n", percentage);
}

static int nvtec_mcu_cmd_dump_all(struct i2c_client *client)
{
	uint16_t val;
	int ret;
	printk("=============================================\n");

	ret = nvtec_get_charging_limit(client, &val);
	if (ret < 0) return ret;
	printk("Charging limit = %d\n", val);

	ret = nvtec_get_shipping_mode(client, &val);
	if (ret < 0) return ret;
	printk("Shipping mode is %s\n", (val==1)? "Enabled":"Disabled");

	ret = nvtec_get_temporary_current_level(client, &val);
	if (ret < 0) return ret;
	printk("Temporary USB input current = %d\n", val);

	ret = nvtec_get_current_level(client, &val);
	if (ret < 0) return ret;
	printk("USB input current = %d\n", val);

	printk("=============================================\n");

	return ret;
}

static void nvtec_mcu_cmd_usage_show(void)
{
	printk("=============================================\n");
	printk("* %s: Set charging limit\n", NVTEC_MCU_CMD_STR_CHG_LIMIT);
	printk("    chg_limit $val, $val=50~100\n\n");

	printk("* %s: Enable/Disable device into shipping mode\n", NVTEC_MCU_CMD_STR_SHIP_MODE);
	printk("    1. ship_mode (enable)\n");
	printk("    2. ship_mode $val, $val=1(enable), 0(disable)\n\n");

	printk("* %s: Reset MCU\n", NVTEC_MCU_CMD_STR_RESET_MCU);
	printk("    reset_mcu\n\n");

	printk("* %s: Set USB input current\n", NVTEC_MCU_CMD_STR_CUR_LEVEL);
	printk("    cur_lv $val, $val=500, 900, 1500, 2000\n\n");

	printk("* %s: Set temporary USB input current\n", NVTEC_MCU_CMD_STR_TCUR_LEVEL);
	printk("    tcur_lv $val, $val=500, 900, 1500, 2000\n\n");

	printk("=============================================\n");
}

static ssize_t nvtec_mcu_cmd_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
    struct miscdevice *mdev = dev_get_drvdata(dev);
    struct nvtec *ec = container_of(mdev, struct nvtec, miscdev);
	char *cp, *endp, *cmd, *tmp;
	int val=0, ret=0;
	int e=0;

	/* there is a '\n' in the end of 'buf', remove it! */
    tmp = (char *)buf;
    cp = strsep(&tmp, "\n");

    cmd = strsep(&cp, " ");
	if (cmd == NULL)
		return count;

	if (cp == NULL)	{
		val = -1;
	} else {
		val = simple_strtol(cp, &endp, 10);
		if (cmd == endp || *endp != '\0') {
			e = __LINE__;
			goto fail;
		}
	}

	/* cmd: chg_limit $val, $val=50,55,60,...,100 */
	if(!strcmp(NVTEC_MCU_CMD_STR_CHG_LIMIT, cmd)) {
		ret = nvtec_set_charging_limit(ec->client, val);
		if (ret < 0)
			dev_err(dev, "Set charging limit failed!(%d)\n", ret);
		else
			dev_info(dev, "Set charging limit = %d\n", val);

		return count;
	}
	/* cmd: ship_mode / ship_mode $val, $val=1(enable), 0(disable) */
	else if(!strcmp(NVTEC_MCU_CMD_STR_SHIP_MODE, cmd)) {
		ret = nvtec_set_shipping_mode(ec->client, val);

		if (ret < 0)
			dev_err(dev, "Set shipping mode failed!(%d)\n", ret);
		else
		{
			if (val == 0)
				dev_info(dev, "Disable shipping mode successful!\n");
			else
				dev_info(dev, "Enable shipping mode successful!\n");
		}
		return count;
	}
	/* cmd: reset_mcu */
	else if(!strcmp(NVTEC_MCU_CMD_STR_RESET_MCU, cmd)) {
		ret = nvtec_reset_mcu(ec->client);
		if (ret < 0)
			dev_err(dev, "Reset MCU failed!(%d)\n", ret);
		else
			dev_info(dev, "Reset MCU successful!\n");

		return count;
	}
	/* cmd: cur_lv $val, $val=500, 900, 1500, 2000 */
	else if(!strcmp(NVTEC_MCU_CMD_STR_CUR_LEVEL, cmd)) {
		ret = nvtec_set_current_level(ec->client, val);
		if (ret < 0)
			dev_err(dev, "Set USB input current failed!(%d)\n", ret);
		else
			dev_info(dev, "Set USB input current = %d\n", val);

		return count;
	}
	/* cmd: tcur_lv $val, $val=500, 900, 1500, 2000 */
	else if(!strcmp(NVTEC_MCU_CMD_STR_TCUR_LEVEL, cmd)) {
		ret = nvtec_set_temporary_current_level(ec->client, val);
		if (ret < 0)
			dev_err(dev, "Set temporary USB input current failed!(%d)\n", ret);
		else
			dev_info(dev, "Set temporary USB input current = %d\n", val);

		return count;
	}
	/* cmd: dump */
	else if(!strcmp(NVTEC_MCU_CMD_STR_DUMP_ALL, cmd)) {
		ret = nvtec_mcu_cmd_dump_all(ec->client);
		if (ret < 0)
			dev_err(dev, "Dump mcu info failed!(%d)\n", ret);

		return count;
	} else {
		e = __LINE__;
		goto fail;
	}

fail:
	dev_err(dev, "%s, %d, cmd error!\n", __FUNCTION__, e);
	nvtec_mcu_cmd_usage_show();
	return count;
}

static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, nvtec_state_show, nvtec_state_store);
static DEVICE_ATTR(version, S_IRUGO, nvtec_version_show, NULL);
static DEVICE_ATTR(flash_id, S_IRUGO, nvtec_flash_id_show, NULL);
static DEVICE_ATTR(gpio_in, S_IRUGO, nvtec_gpio_input_show, NULL);
static DEVICE_ATTR(gpio_out, S_IWUSR | S_IRUGO, nvtec_gpio_output_show, nvtec_gpio_output_store);
static DEVICE_ATTR(cmd_write, S_IWUSR, NULL, nvtec_cmd_write_store);
static DEVICE_ATTR(cmd_read, S_IWUSR | S_IRUGO, nvtec_cmd_read_show, nvtec_cmd_read_store);
static DEVICE_ATTR(up_state, S_IRUGO, nvtec_upgrade_show,NULL);
static DEVICE_ATTR(mcu_cmd, S_IWUSR, NULL, nvtec_mcu_cmd_store);
static DEVICE_ATTR(charger_id, S_IRUGO, nvtec_charger_id_show, NULL);

static int nvtec_add_sysfs_entry(struct nvtec *ec)
{
    struct device *dev = ec->miscdev.this_device;
    int ret;

    ret = device_create_file(dev, &dev_attr_state);
    if (ret < 0)
        return ret;

    ret = device_create_file(dev, &dev_attr_version);
    if (ret < 0)
        return ret;

    ret = device_create_file(dev, &dev_attr_flash_id);
    if (ret < 0)
        return ret;

    ret = device_create_file(dev, &dev_attr_gpio_in);
    if (ret < 0)
        return ret;

    ret = device_create_file(dev, &dev_attr_gpio_out);
    if (ret < 0)
        return ret;

    ret = device_create_file(dev, &dev_attr_cmd_write);
    if (ret < 0)
        return ret;

    ret = device_create_file(dev, &dev_attr_cmd_read);
    if (ret < 0)
        return ret;
    
    ret = device_create_file(dev, &dev_attr_up_state);
    if (ret < 0)
    	return ret;

    ret = device_create_file(dev, &dev_attr_mcu_cmd);
    if (ret < 0)
    	return ret;

    ret = device_create_file(dev, &dev_attr_charger_id);
    if (ret < 0)
    	return ret;

    return 0;

}

static void nvtec_remove_sysfs_entry(struct nvtec *ec)
{
    struct device *dev = ec->miscdev.this_device;

    device_remove_file(dev, &dev_attr_state);
    device_remove_file(dev, &dev_attr_version);
    device_remove_file(dev, &dev_attr_flash_id);
    device_remove_file(dev, &dev_attr_gpio_in);
    device_remove_file(dev, &dev_attr_gpio_out);
    device_remove_file(dev, &dev_attr_cmd_write);
    device_remove_file(dev, &dev_attr_cmd_read);
    device_remove_file(dev, &dev_attr_up_state);
    device_remove_file(dev, &dev_attr_mcu_cmd);
    device_remove_file(dev, &dev_attr_charger_id);
}


static int __devinit nvtec_i2c_probe(struct i2c_client *client,
                                     const struct i2c_device_id *id)
{
    struct nvtec_platform_data *pdata = client->dev.platform_data;
    struct nvtec *ec;
    uint16_t event;
    char version[32], ver_str[32];
	int len = 0;
    /* char lock_status[16]; */
    int ret = -EINVAL;

    if (!pdata) {
        dev_err(&client->dev, "nvtec requires platform data\n");
        return -ENOTSUPP;
    }

	/* Daniel Wang add for check micro-p exist or not */
	ret = __nvtec_read_block(client, 0xC0, 32, version);

    len = (version[0] >= 31) ? 31 : version[0];
    strncpy(ver_str, &version[1], len);
    ver_str[len] = '\0';

    if (ret < 0) {
        dev_err(&client->dev, "Unknown EC firmware version: %d\n", ret);
        return -ENODEV;
    } else {
        dev_info(&client->dev, "EC version: %s\n", ver_str);
    }

    /* turn-off led again, avoid led always light if bootloader or mcu turn off failed */
    ret = __nvtec_write_word(client, 0x40, 0x0);
    if (ret < 0) {
        dev_err(&client->dev, "Set led off failed: %d\n", ret);
        return -ENODEV;
    }

    ec = kzalloc(sizeof(struct nvtec), GFP_KERNEL);
    if (NULL == ec)
        return -ENOMEM;

    /* g_nvtec = ec; */
    ec->client = client;
    ec->dev = &client->dev;
    i2c_set_clientdata(client, ec);

    mutex_init(&ec->lock);
    init_waitqueue_head(&ec->wq_ack);
    BLOCKING_INIT_NOTIFIER_HEAD(&ec->system_notifier);
    BLOCKING_INIT_NOTIFIER_HEAD(&ec->battery_notifier);

    if (client->irq) {
        ret = nvtec_irq_init(ec, client->irq);
        if (ret) {
            dev_err(&client->dev, "IRQ init failed: %d\n", ret);
            goto err_irq_init;
        }
    }

    ec->request_pin = pdata->request_pin;
    ec->pwr_state_pin = pdata->pwr_state_pin;
    ec->ap_wake_pin = pdata->ap_wake_pin;
    if (ec->request_pin == 0 || ec->pwr_state_pin == 0 || ec->ap_wake_pin == 0) {
        dev_err(&client->dev, "Invalid GPIOs definition\n");
        goto err_gpio;
    }

    gpio_set_value(ec->request_pin, 1);
    gpio_set_value(ec->pwr_state_pin, 1);

    /* clean event */
    while (gpio_get_value(ec->ap_wake_pin) == 0) {
        ret = nvtec_read_event(ec->client, &event);
        if (ret < 0 || event == 0)
            break;
    }

    ret = nvtec_add_subdevs(ec, pdata);
    if (ret) {
        dev_err(&client->dev, "add devices failed: %d\n", ret);
        goto err_add_devs;
    }
 
    ec->miscdev.parent = ec->dev;
    ec->miscdev.minor = NVTEC_MINOR;
    ec->miscdev.name = "nvtec";
    ec->miscdev.fops = &nvtec_fops;
    ret = misc_register(&ec->miscdev);
    if (ret) {
        dev_err(&client->dev, "fail to register misc device: %d\n", ret);
        goto err_misc_reg;
    }

    ret = nvtec_add_sysfs_entry(ec);
    if (ret) {
        dev_err(&client->dev, "fail to add sysfs entry: %d\n", ret);
        goto err_add_sysfs;
    }

    wake_lock_init(&ec->upgrade_lock, WAKE_LOCK_SUSPEND, "NvUpgradeSuspendLock");

    return 0;

 err_add_sysfs:
    misc_deregister(&ec->miscdev);
 err_misc_reg:
    nvtec_remove_subdevs(ec);
 err_gpio:
 err_add_devs:
    if (client->irq)
        free_irq(client->irq, ec);
 err_irq_init:
    mutex_destroy(&ec->lock);

    if (ec->request_pin)
        gpio_free(ec->request_pin);
    if (ec->pwr_state_pin)
        gpio_free(ec->pwr_state_pin);
    if (ec->ap_wake_pin)
        gpio_free(ec->ap_wake_pin);

    kfree(ec);

    return ret;
}

static int __devexit nvtec_i2c_remove(struct i2c_client *client)
{
    struct nvtec *ec = i2c_get_clientdata(client);

    wake_lock_destroy(&ec->upgrade_lock);

    if (client->irq)
        free_irq(client->irq, ec);

    nvtec_remove_sysfs_entry(ec);
    misc_deregister(&ec->miscdev);
    nvtec_remove_subdevs(ec);
    mutex_destroy(&ec->lock);

    if (ec->request_pin)
        gpio_free(ec->request_pin);
    if (ec->pwr_state_pin)
        gpio_free(ec->pwr_state_pin);
    if (ec->ap_wake_pin)
        gpio_free(ec->ap_wake_pin);

    kfree(ec);

    return 0;
}

static int nvtec_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    int ret;

    ret = nvtec_set_state(client, NVTEC_STATE_S3);
    if (ret < 0)
        return -EACCES;

    return 0;
}

static int nvtec_i2c_resume(struct i2c_client *client)
{
    int ret;

    ret = nvtec_set_state(client, NVTEC_STATE_S0);
    if (ret < 0)
        return -EACCES;

    return 0;
}

static void nvtec_i2c_shutdown(struct i2c_client *client)
{
    if (client->irq)
        disable_irq(client->irq);
    
    nvtec_set_state(client, NVTEC_STATE_S4);
}

static const struct i2c_device_id nvtec_id_table[] = {
    { "nvtec", 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, nvtec_id_table);

static struct i2c_driver nvtec_driver = {
    .driver = {
        .name = "nvtec",
        .owner = THIS_MODULE,
    },
    .probe  = nvtec_i2c_probe,
    .remove = nvtec_i2c_remove,
    .suspend = nvtec_i2c_suspend,
    .resume = nvtec_i2c_resume,
    .shutdown = nvtec_i2c_shutdown,
    .id_table = nvtec_id_table,
};

static int __init nvtec_init(void)
{
    return i2c_add_driver(&nvtec_driver);
}
subsys_initcall(nvtec_init);

static void __exit nvtec_exit(void)
{
    i2c_del_driver(&nvtec_driver);
}
module_exit(nvtec_exit);

MODULE_DESCRIPTION("Nuvoton EC core driver");
MODULE_AUTHOR("Ron Lee <ron1_lee@pegatroncorp.com>");
MODULE_LICENSE("GPL");
