#include <linux/ioctl.h>  /* For IOCTL macros */

#define TEGRA_AIC325X_NAME    "tegra_aic325x"
#define TEGRA_AIC325X_PATH    "/dev/tegra_aic325x"

#define	TEGRA_AIC325X_IOCTL_BASE	'F'

#define	TEGRA_AIC325X_SET_ENABLE_FM34		_IOW(TEGRA_AIC325X_IOCTL_BASE, 0, int)
#define	TEGRA_AIC325X_SET_DISABLE_FM34	_IOW(TEGRA_AIC325X_IOCTL_BASE, 1, int)