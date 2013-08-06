#ifndef __MT9M114_H__
#define __MT9M114_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define MT9M114_NAME    "mt9m114"
#define MT9M114_PATH    "/dev/mt9m114"

#define MT9M114_IOCTL_SET_MODE          _IOW('o', 1, struct mt9m114_mode)
#define MT9M114_IOCTL_SET_COLOR_EFFECT  _IOW('o', 2, unsigned int)
#define MT9M114_IOCTL_SET_WHITE_BALANCE _IOW('o', 3, unsigned int)
#define MT9M114_IOCTL_SET_EXPOSURE      _IOW('o', 4, int)
#define MT9M114_IOCTL_SET_EXPOSURE_RECT _IOW('o', 5, struct mt9m114_rect)
#define MT9M114_IOCTL_GET_EXPOSURE_TIME _IOW('o', 6, unsigned int)
#define MT9M114_IOCTL_SET_FPS           _IOW('o', 7, unsigned int)
#define MT9M114_IOCTL_GET_ISO           _IOW('o', 8, unsigned int)

struct mt9m114_mode {
	int xres;
	int yres;
};

struct mt9m114_rect{
	int x;
	int y;
	int width;
	int height;
};

#ifdef __KERNEL__

struct mt9m114_platform_data {
	int (*init)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

#endif /* __KERNEL__ */
#endif  /* __MT9M114_H__ */

