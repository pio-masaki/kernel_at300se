#ifndef __S5K5CAGA_H__
#define __S5K5CAGA_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define S5K5CAGA_NAME    "s5k5caga"
#define S5K5CAGA_PATH    "/dev/s5k5caga"

#define S5K5CAGA_IOCTL_SET_MODE          _IOW('o', 1, struct s5k5caga_mode)
#define S5K5CAGA_IOCTL_SET_WHITE_BALANCE _IOW('o', 2, unsigned int)
#define S5K5CAGA_IOCTL_SET_EXPOSURE      _IOW('o', 3, int)
#define S5K5CAGA_IOCTL_SET_EXPOSURE_RECT _IOW('o', 4, struct s5k5caga_rect)
#define S5K5CAGA_IOCTL_GET_EXPOSURE_TIME _IOW('o', 5, unsigned int)
#define S5K5CAGA_IOCTL_SET_AF_MODE       _IOW('o', 6, unsigned int)
#define S5K5CAGA_IOCTL_SET_AF_TRIGGER    _IOW('o', 7, unsigned int)
#define S5K5CAGA_IOCTL_SET_AF_RECT       _IOW('o', 8, struct s5k5caga_rect)
#define S5K5CAGA_IOCTL_GET_AF_STATUS     _IOW('o', 9, unsigned int)
#define S5K5CAGA_IOCTL_SET_FPS           _IOW('o', 10, unsigned int)
#define S5K5CAGA_IOCTL_GET_ISO           _IOW('o', 11, unsigned int)

struct s5k5caga_mode {
	int xres;
	int yres;
};

struct s5k5caga_rect{
    int x;
    int y;
    int width;
    int height;
};

#ifdef __KERNEL__

struct s5k5caga_platform_data {
	int (*init)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

#endif /* __KERNEL__ */
#endif  /* __S5K5CAGA_H__ */

