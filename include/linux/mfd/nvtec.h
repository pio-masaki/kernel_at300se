#ifndef __LINUX_MFD_NVTEC_H
#define __LINUX_MFD_NVTEC_H

#include <linux/notifier.h>

struct nvtec_subdev_info {
    int id;
    const char *name;
    void  *platform_data;
};

struct nvtec_platform_data {
    int num_subdevs;
    struct nvtec_subdev_info *subdevs;
    int request_pin;
    int pwr_state_pin;
    int ap_wake_pin;
    int ec_wake_pin;
};

struct nvtec_battery_platform_data {
    unsigned int ac_in_pin;
    unsigned int batt_low_pin;
};

enum nvtec_event_type {
    NVTEC_EVENT_SYSTEM = 0xC5,
    NVTEC_EVENT_BATTERY = 0xA8,
    NVTEC_FLASH_ACK = 0xFA,
    NVTEC_FLASH_NACK = 0xFE,
};

extern int nvtec_read_byte(struct device *dev, int reg, uint8_t *val);
extern int nvtec_read_word(struct device *dev, int reg, uint16_t *val);
extern int nvtec_read_block(struct device *dev, int reg, int len, uint8_t *val);
extern int nvtec_write_byte(struct device *dev, int reg, uint8_t val);
extern int nvtec_write_word(struct device *dev, int reg, uint16_t val);
extern int nvtec_write_block(struct device *dev, int reg, int len, uint8_t *val);

extern int nvtec_register_event_notifier(struct device *dev, 
                                         enum nvtec_event_type event,
                                         struct notifier_block *nb);
extern int nvtec_unregister_event_notifier(struct device *dev,
                                           enum nvtec_event_type event,
                                           struct notifier_block *nb);

#ifdef CONFIG_BATTERY_NVTEC
extern void suspend_battery_thread(void);
extern void resume_battery_thread(void);
extern int nvtec_ac_notifier_register(struct notifier_block *nb);
extern int nvtec_ac_notifier_unregister(struct notifier_block *nb);
#else
static inline int nvtec_ac_notifier_register(struct notifier_block *nb)
{
    return 0;
}
static inline int nvtec_ac_notifier_unregister(struct notifier_block *nb)
{
    return 0;
}
#endif

#endif  /* __LINUX_MFD_ */
