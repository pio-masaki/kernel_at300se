/*
 * Raydium RM31080 touchscreen driver
 *
 * Copyright (C) 2012 Raydium Semiconductor Corporation
 * Copyright (C) 2012 NVIDIA Corporation
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
/*=========================================================================*/
/*INCLUDED FILES */
/*=========================================================================*/
#include <linux/module.h>
#include <linux/input.h>	/* BUS_SPI */
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/sched.h>	// wake_up_process()
#include <linux/kthread.h>	// kthread_create(),kthread_run()
#include <asm/uaccess.h>	// copy_to_user(),
#include <linux/miscdevice.h>
#include <asm/siginfo.h>	// siginfo
#include <linux/rcupdate.h>	// rcu_read_lock
#include <linux/sched.h>	// find_task_by_pid_type
#include <linux/syscalls.h>	// sys_clock_gettime()
#include <linux/random.h>	// random32()
#include <linux/suspend.h>  // pm_notifier
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef ENABLE_MANUAL_IDLE_MODE
#include <linux/timer.h>
#endif

#include <linux/spi/rm31080a_ts.h>
#include <linux/spi/rm31080a_ctrl.h>
/*=========================================================================*/
/*DEFINITIONS */
/*=========================================================================*/
#define ENABLE_WORK_QUEUE
#define ENABLE_REPORT_TO_UART
#define ENABLE_RM31080_DEEP_SLEEP
#define ENABLE_AUTO_SCAN
#define ENABLE_RM31080_ESD_PROTECT
/*#define ENABLE_SPEED_TEST_FUNCTION */
/*#define ENABLE_CALC_QUEUE_COUNT */
/*#define ENABLE_SPI_BURST_READ_WRITE */
/*#define ENABLE_SPI_SETTING */
#define ENABLE_SLOW_SCAN
#define ENABLE_SMOOTH_LEVEL

#define MAX_SPI_FREQ_HZ      50000000
#define TS_PEN_UP_TIMEOUT    msecs_to_jiffies(50)

#ifdef ENABLE_RAW_DATA_QUEUE
#define QUEUE_COUNT       128
#define RAW_DATA_LENGTH  2048

#define RM_SCAN_MODE_MANUAL          0x00
#define RM_SCAN_MODE_PREPARE_AUTO    0x01
#define RM_SCAN_MODE_AUTO_SCAN       0x02

#define RM_NEED_NONE                 0x00
#define RM_NEED_TO_SEND_SCAN         0x01
#define RM_NEED_TO_READ_RAW_DATA     0x02
#define RM_NEED_TO_SEND_SIGNAL       0x04
#endif

#ifdef ENABLE_SLOW_SCAN
#define RM_SLOW_SCAN_LEVEL_NORMAL    0
#define RM_SLOW_SCAN_LEVEL_20       20
#define RM_SLOW_SCAN_LEVEL_40       40
#define RM_SLOW_SCAN_LEVEL_60       60
#define RM_SLOW_SCAN_LEVEL_80       80
#define RM_SLOW_SCAN_LEVEL_100     100
#define RM_SLOW_SCAN_LEVEL_120     120
#define RM_SLOW_SCAN_LEVEL_140     140
#define RM_SLOW_SCAN_LEVEL_160     160
#define RM_SLOW_SCAN_LEVEL_180     180
#define RM_SLOW_SCAN_LEVEL_200     200
#define RM_SLOW_SCAN_LEVEL_220     220
#define RM_SLOW_SCAN_LEVEL_240     240
#endif

#ifdef ENABLE_SMOOTH_LEVEL
#define RM_SMOOTH_LEVEL_NORMAL    0
#define RM_SMOOTH_LEVEL_MAX       4
#endif

#define RM_AUO_10_CHANNEL_X 48
#define RM_WINTEK_7_CHANNEL_X 30

#define TIME_QUEUE_COUNT       128

#ifdef ENABLE_WORK_QUEUE
#include <linux/workqueue.h>
#endif

#define MANUAL_IDLE_SCAN_PERIOD		(HZ / 100)

#define rm_printk(msg...)            printk(msg)
#define rmd_printk(msg...)
/*=========================================================================*/
/*STRUCTURE DECLARATION */
/*=========================================================================*/
/*TouchScreen Parameters */
struct rm31080a_ts_para {
	unsigned long ulHalPID;
	bool bInitFinish;
	bool bCalcFinish;
	bool bEnableScriber;
	bool bEnableAutoScan;
	bool bIsSuspended;

#ifdef ENABLE_RAW_DATA_QUEUE
	u8 u8ScanModeState;
#endif

#ifdef ENABLE_SLOW_SCAN
	bool bEnableSlowScan;
	u32 u32SlowScanLevel;
#endif

#ifdef ENABLE_SMOOTH_LEVEL
	u32 u32SmoothLevel;
#endif

	u8 u8SelfTestStatus;
	u8 u8SelfTestResult;
	u8 u8Version;
	u8 u8Repeat;
};

struct rm31080_ts {
	const struct rm31080_bus_ops *bops;
	struct device *dev;
	struct input_dev *input;
    struct spi_device *spi_dev;
    struct miscdevice miscdev;
	unsigned int irq;
	bool disabled;
	bool suspended;
	char phys[32];
	struct mutex mutex_scan_mode;

    struct rm31080a_ts_para stTs;
    struct rm31080a_ctrl_para stCtrl;

    /* struct timer_list ts_manual_idle_timer; */
#ifdef ENABLE_WORK_QUEUE
	struct workqueue_struct *rm_workqueue;
	struct work_struct rm_work;
#endif
#ifdef ENABLE_MANUAL_IDLE_MODE
	struct workqueue_struct *rm_idle_workqueue;
	struct delayed_work rm_idle_work;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

struct rm31080_bus_ops {
	u16 bustype;
	int (*read) (struct device * dev, u8 reg);
	int (*write) (struct device * dev, u8 reg, u16 val);
};

#ifdef ENABLE_RAW_DATA_QUEUE
struct rm31080_queue_info {
	u8(*pQueue)[RAW_DATA_LENGTH];
	u16 u16Front;
	u16 u16Rear;
};
#endif

struct rm31080_time_queue_info {
	struct timespec *time;
	u16 u16Front;
	u16 u16Rear;
};


//=============================================================================
//GLOBAL VARIABLES DECLARATION
//=============================================================================

#ifdef ENABLE_RAW_DATA_QUEUE
struct rm31080_queue_info g_stQ;
#endif

struct rm31080_time_queue_info g_ttQ;
static bool time_queue_dequeue_flag = 0;
static int rm31080_time_queue_is_empty(void);
static int rm31080_time_queue_is_full(void);
static void rm31080_time_enqueue_finish(void);
static void rm31080_time_dequeue_finish(void);

//=============================================================================
//FUNCTION DECLARATION
//=============================================================================
static bool rm31080_spi_byte_read(struct rm31080_ts *ts, unsigned char u8Addr, unsigned char *pu8Value);
static bool rm31080_spi_byte_write(struct rm31080_ts *ts, unsigned char u8Addr, unsigned char u8Value);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void rm31080_early_suspend(struct early_suspend *es);
static void rm31080_early_resume(struct early_suspend *es);
#endif
static bool rm31080_spi_checking(struct rm31080_ts *ts, bool bInfinite);
static bool rm31080_spi_frame_checking(struct rm31080_ts *ts, unsigned int ucheckbyte);
static void rm31080_init_ts_structure_part(struct rm31080_ts *ts);
static void rm31080_start(struct rm31080_ts *ts);
static void rm31080_disable_touch(struct rm31080_ts *ts);
static void rm31080_enable_touch(struct rm31080_ts *ts);
static void rm31080_set_autoscan(struct rm31080_ts *ts, unsigned char val);

#include "rm31080a_ctrl.c"

//=============================================================================
// * Description:
// *      Debug function: test speed.
// * Input:
// *      N/A
// * Output:
// *      1:succeed
// *      0:failed
// *========================================================================= */
#ifdef ENABLE_SPEED_TEST_FUNCTION
static void my_calc_time(int iStart)
{
	static volatile unsigned int u32Max = UINT_MAX;

	static long iTimebuffer[1000];
	static unsigned long long t1, t2;
	unsigned long nanosec_rem;
	static int iIndex = 0;

	if (iStart) {
		t1 = cpu_clock(u32Max);
		return;
	} else
		t2 = cpu_clock(u32Max);

	t2 = t2 - t1;

	nanosec_rem = do_div(t2, 1000000000);

	if (t2) {		/*more than 1 Second */
		iTimebuffer[iIndex] = 999999;
	} else {
		iTimebuffer[iIndex] = nanosec_rem / 1000;	/*micro second */
	}

	iIndex++;
	if (iIndex == 1000) {
		for (iIndex = 0; iIndex < 1000; iIndex++) {
			rm_printk("   %04d,%06d\n", iIndex,
				  (u32) iTimebuffer[iIndex]);
		}
		iIndex = 0;
	}

}
#endif				/*ENABLE_SPEED_TEST_FUNCTION */
/*=========================================================================
   Description:
        RM31080 spi interface.
   Input:
  
   Output:
        1:succeed
        0:failed
  =========================================================================*/
static bool rm31080_spi_read(struct rm31080_ts *ts, u8 u8addr, u8 * rxbuf, size_t len)
{
    struct spi_device *spi = ts->spi_dev;
	struct spi_message message;
	struct spi_transfer x[2];
    int status;

	spi_message_init(&message);
	memset(x, 0, sizeof x);

	u8addr |= 0x80;
	x[0].len = 1;
	x[0].tx_buf = &u8addr;
	spi_message_add_tail(&x[0], &message);

	x[1].len = len;
	x[1].rx_buf = rxbuf;
	spi_message_add_tail(&x[1], &message);

	/* It returns zero on succcess,else a negative error code. */
	status = spi_sync(spi, &message);

	if (status)
		return false;

	return true;
}

/*=========================================================================
   Description:
        RM31080 spi interface.
   Input:
  
   Output:
        1:succeed
        0:failed
  ========================================================================= */
static bool rm31080_spi_write(struct rm31080_ts *ts, u8 * txbuf, size_t len)
{
    struct spi_device *spi = ts->spi_dev;
	int status;

	status = spi_write(spi, txbuf, len);

    /*It returns zero on succcess,else a negative error code.*/
	if (status)
		return false;

	return true;
}

/*=========================================================================
   Description:
        RM31080 spi interface.
   Input:
  
   Output:
        1:succeed
        0:failed
  =========================================================================*/
static bool rm31080_spi_byte_read(struct rm31080_ts *ts, unsigned char u8Addr, unsigned char *pu8Value)
{
	return rm31080_spi_read(ts, u8Addr, pu8Value, 1);
}

/*=========================================================================
   Description:
        RM31080 spi interface.
   Input:
  
   Output:
        1:succeed
        0:failed
  ========================================================================= */
static bool rm31080_spi_byte_write(struct rm31080_ts *ts, unsigned char u8Addr, unsigned char u8Value)
{
	u8 buf[2];

	buf[0] = u8Addr;
	buf[1] = u8Value;
	return rm31080_spi_write(ts, buf, 2);
}

/*=========================================================================
   Description:
        RM31080 spi interface.
   Input:
  
   Output:
        1:succeed
        0:failed
  ========================================================================= */
#ifdef ENABLE_SPI_BURST_READ_WRITE
static bool rm31080_spi_burst_read(struct rm31080_ts *ts, unsigned char u8Addr, 
                                  unsigned char *pu8Value, u32 u32len)
{
	u8 *pMyBuf;
	int ret;

	pMyBuf = kmalloc(u32len, GFP_KERNEL);
	if (pMyBuf == NULL)
		return false;

	ret = rm31080_spi_read(ts, u8Addr, pMyBuf, u32len);

	if (ret) {
		memcpy(pu8Value, pMyBuf, u32len);
	}

	kfree(pMyBuf);

	return ret;
}
#endif
/*=========================================================================
   Description:
        RM31080 spi interface.
   Input:
  
   Output:
        1:succeed
        0:failed
  ========================================================================= */
static bool rm31080_spi_burst_write(struct rm31080_ts *ts, unsigned char *pBuf, unsigned int u32Len)
{
	u8 *pMyBuf;
	int ret;

	pMyBuf = kmalloc(u32Len, GFP_KERNEL);
	if (pMyBuf == NULL)
		return false;

	memcpy(pMyBuf, pBuf, u32Len);
	ret = rm31080_spi_write(ts, pMyBuf, u32Len);
	kfree(pMyBuf);
	return ret;
}

/*=========================================================================*/
#ifdef ENABLE_AUTO_SCAN
/* NOTE: should be protected by mutex_scan_mode */
static void raydium_change_scan_mode(struct rm31080_ts *ts, u8 u8TouchCount)
{
	static u32 u32NoTouchCount = 0;
	u16 u16NTCountThd;

	u16NTCountThd = (u16)ts->stCtrl.bTime2Idle * 100;

	if (u8TouchCount) {
		u32NoTouchCount = 0;
		return;
	}

	if(u32NoTouchCount < u16NTCountThd) {
		u32NoTouchCount++;
	} else if (ts->stTs.u8ScanModeState == RM_SCAN_MODE_MANUAL) {
		if (ts->stTs.bEnableAutoScan)
			ts->stTs.u8ScanModeState = RM_SCAN_MODE_PREPARE_AUTO;
		u32NoTouchCount = 0;
	}
}
#endif				/*ENABLE_AUTO_SCAN*/
/*=========================================================================
  report touch data for scriber
  
  =========================================================================*/
#ifdef ENABLE_REPORT_TO_UART
static void raydium_report_to_uart_printf(unsigned char *ucData, unsigned char ucCount)
{
	unsigned char i;

	for (i = 0; i < ucCount; i++) {
		rm_printk("%02X", ucData[i]);
	}

	rm_printk("\n");
}

static void raydium_report_to_uart(struct rm31080_ts *ts, void *p)
{
	unsigned char ucData[1 + 1 + (4 * 12) + 1];	/*1=Tag,1=Touch count,4=(xH xL ,yH yL) ,12=max point,1=Check sum*/
	rm_touch_event *spTP;
	unsigned short usX, usY;
	int i, j;

	if (ts->stTs.bEnableScriber == 0)
		return;

	spTP = (rm_touch_event *) p;

	ucData[0] = 0x8E;
	ucData[1] = spTP->ucTouchCount;
	j = 2;
	for (i = 0; i < spTP->ucTouchCount; i++) {
		usX = spTP->usX[i] + 1;
		usY = spTP->usY[i] + 1;
		ucData[j++] = ((usX >> 8) & 0xFF) | (spTP->ucID[i] << 4);	/*add id*/
		ucData[j++] = ((usX) & 0xFF);
		ucData[j++] = ((usY >> 8) & 0xFF);
		ucData[j++] = ((usY) & 0xFF);
	}

	/*check sum*/
	ucData[j] = 0;
	for (i = 0; i < j; i++) {
		ucData[j] += ucData[i];
	}
	ucData[j] = 0x100 - ucData[j];
	j++;

	/*print*/
	raydium_report_to_uart_printf(ucData, j);
	if (spTP->ucTouchCount == 0) {	/*send more , to avoid losing*/
		raydium_report_to_uart_printf(ucData, j);
		raydium_report_to_uart_printf(ucData, j);
	}
}
#endif				/*ENABLE_REPORT_TO_UART*/

static void raydium_report_pointer(struct rm31080_ts *ts, void *p)
{
	static unsigned char ucLastTouchCount = 0;
	int i;
	int iCount;
	int iMaxX, iMaxY;
	rm_touch_event *spTP;
	spTP = (rm_touch_event *) p;

#if ENABLE_RESOLUTION_SWITCH
	if ((ts->stCtrl.u16ResolutionX != 0) && (ts->stCtrl.u16ResolutionY != 0)) {
		iMaxX = ts->stCtrl.u16ResolutionX;
		iMaxY = ts->stCtrl.u16ResolutionY;
	} else {
		iMaxX = RM_INPUT_RESOLUTION_X;
		iMaxY = RM_INPUT_RESOLUTION_Y;
	}
#else
	iMaxX = RM_INPUT_RESOLUTION_X;
	iMaxY = RM_INPUT_RESOLUTION_Y;
#endif

	time_queue_dequeue_flag = 0;
	if (!rm31080_time_queue_is_empty()) {
		ts->input->time = (g_ttQ.time[g_ttQ.u16Front]);
		ts->input->time_is_valid = 1;
		rm31080_time_dequeue_finish();
	}

	iCount = max(ucLastTouchCount, spTP->ucTouchCount);
	if (iCount) {
		for (i = 0; i < iCount; i++) {
			if (i == 10)
				break;	/*due to the "touch test" can't support great than 10 points*/

			if (i < spTP->ucTouchCount) {
				input_report_abs(ts->input,
						 ABS_MT_TRACKING_ID,
						 spTP->ucID[i]);
				input_report_abs(ts->input,
						 ABS_MT_TOUCH_MAJOR,
						 spTP->usZ[i]);
				input_report_abs(ts->input,
						 ABS_MT_WIDTH_MAJOR,
						 spTP->usZ[i]);
				input_report_abs(ts->input,
						 ABS_MT_PRESSURE,
						 spTP->usZ[i]);

				if (spTP->usX[i] >= (iMaxX - 1))
					input_report_abs(ts->input,
							 ABS_MT_POSITION_X,
							 (iMaxX - 1));
				else
					input_report_abs(ts->input,
							 ABS_MT_POSITION_X,
							 spTP->usX[i]);

				if (spTP->usY[i] >= (iMaxY - 1))
					input_report_abs(ts->input,
							 ABS_MT_POSITION_Y,
							 (iMaxY - 1));
				else
					input_report_abs(ts->input,
							 ABS_MT_POSITION_Y,
							 spTP->usY[i]);
			}
			input_mt_sync(ts->input);
		}
		ucLastTouchCount = spTP->ucTouchCount;
		input_report_key(ts->input, BTN_TOUCH,
				 spTP->ucTouchCount > 0);
		input_sync(ts->input);
#ifdef ENABLE_REPORT_TO_UART
		raydium_report_to_uart(ts, p);
#endif
	}

	ts->input->time_is_valid = 0;
#ifdef ENABLE_AUTO_SCAN
	if (ts->stCtrl.bfPowerMode) {
        mutex_lock(&ts->mutex_scan_mode);
		raydium_change_scan_mode(ts, spTP->ucTouchCount);
        mutex_unlock(&ts->mutex_scan_mode);
    }
#endif
}

/*=========================================================================
   Description:
        RM31080 control functions.
   Input:
        N/A
   Output:
        1:succeed
        0:failed
  =========================================================================*/
#ifdef ENABLE_RAW_DATA_QUEUE
static bool rm31080_ctrl_read_raw_data(struct rm31080_ts *ts, unsigned char *p)
{
	int ret;

#if ENABLE_T007B1_SETTING
	if (ts->stCtrl.bICVersion == T007_VERSION_B) {
		ret = rm31080_spi_byte_write(ts, RM31080B1_REG_BANK0_00H, 0x00);
		if (ret)
			ret = rm31080_spi_byte_write(ts, RM31080B1_REG_BANK0_01H, 0x00);

		if (ret)
			ret = rm31080_spi_read(ts, RM31080B1_REG_BANK0_03H|0x80, p, ts->stCtrl.u16DataLength); /*0x80 for SPI burst read */
	} else 
#endif
	{
		ret = rm31080_spi_byte_write(ts, RM31080_REG_01, 0x10);
		if (ret)
			ret = rm31080_spi_byte_write(ts, RM31080_REG_02, 0x00);

		if (ret)
			ret = rm31080_spi_read(ts, RM31080_REG_80, p, ts->stCtrl.u16DataLength);
	}

	return ret;
}

#ifdef ENABLE_AUTO_SCAN
static void rm_set_idle(struct rm31080_ts *ts, u8 OnOff)
{
	static u8 reg = 0;

	switch (OnOff)
	{
		case 1:
			/* MD regulator off */
			rm31080_spi_byte_read(ts, 0x3A, &reg);
			reg  &= ~0x04;
			rm31080_spi_byte_write(ts, 0x3A, reg);
			break;
		default:
			/* MD regulator on */
			rm31080_spi_byte_read(ts, 0x3A, &reg);
			rm31080_spi_byte_write(ts, 0x3A, reg|0x04);
			break;
	}
}

static void rm31080_ctrl_enter_auto_mode(struct rm31080_ts *ts)
{
	rm_printk("raydium_ts: Enter Auto Scan mode\n"); 
#if ENABLE_FILTER_SWITCH
#if ENABLE_T007B1_SETTING
	if (ts->stCtrl.bICVersion != T007_VERSION_B)
#endif
	{
        if (ts->stCtrl.bfAnalogFilter) {
            rm31080_analog_filter_config(ts, ts->stCtrl.bIdleRepeatTimes[ts->stCtrl.bfAnalogFilter]);    
        } else {
            rm31080_digital_filter_config(ts, ts->stCtrl.bIdleRepeatTimes[ts->stCtrl.bfAnalogFilter]);
        }    
	}

#endif
	/*Enable auto scan */
	/*rm_printk("Enter Auto Scan Mode\n"); */
#if ENABLE_T007B1_SETTING
#if ENABLE_T007B1_STABLE_IDLE_MODE
	if (ts->stCtrl.bICVersion == T007_VERSION_B) {
		rm_set_idle(ts, 1);
		rm_set_repeat_times(ts, ts->stCtrl.bIdleRepeatTimes[0]);
	}
#endif
#endif
#ifdef ENABLE_MANUAL_IDLE_MODE
	rm31080_spi_byte_write(ts, RM31080_REG_09, 0x40);
	queue_delayed_work(ts->rm_idle_workqueue, &ts->rm_idle_work, 0);
#else  /* ENABLE_MANUAL_IDLE_MODE */
	rm31080_spi_byte_write(ts, RM31080_REG_09, 0x10 | 0x40);
#endif
}

static void rm31080_ctrl_leave_auto_mode(struct rm31080_ts *ts)
{
	/* rm_printk("raydium_ts: Leave Auto Scan mode\n"); */

#if ENABLE_T007B1_SETTING
#if ENABLE_T007B1_STABLE_IDLE_MODE
	if (ts->stCtrl.bICVersion == T007_VERSION_B) {
		rm_set_idle(ts, 0);
		rm_set_repeat_times(ts, ts->stCtrl.bRepeatTimes[0]);
	}
#endif
#endif
#if ENABLE_FILTER_SWITCH
#if ENABLE_T007B1_SETTING
	if (ts->stCtrl.bICVersion != T007_VERSION_B)
#endif
    {
        if (ts->stCtrl.bfAnalogFilter) {
            rm31080_analog_filter_config(ts, ts->stCtrl.bRepeatTimes[ts->stCtrl.bfAnalogFilter]);    
        } else {
            rm31080_digital_filter_config(ts, ts->stCtrl.bRepeatTimes[ts->stCtrl.bfAnalogFilter]);
        }    
    }
#endif
	rm31080_spi_byte_write(ts, RM31080_REG_09, 0x00);
}

static void rm31080_ctrl_pause_auto_mode(struct rm31080_ts *ts)
{
	u8 u8reg11;

	rm31080_spi_byte_write(ts, RM31080_REG_09, 0x40);	/*disable auto scan */
	rm31080_spi_byte_read(ts, RM31080_REG_11, &u8reg11);
	u8reg11 &= ~0x01;
	rm31080_spi_byte_write(ts, RM31080_REG_11, u8reg11);	/*set scan start = 0 */
}
#endif				/*ENABLE_AUTO_SCAN */

/* NOTE: should be protected by mutex_scan_mode */
static u32 rm31080_ctrl_configure(struct rm31080_ts *ts)
{
	u32 u32Flag;

	switch (ts->stTs.u8ScanModeState) {
	case RM_SCAN_MODE_MANUAL:
		u32Flag = RM_NEED_TO_SEND_SCAN | RM_NEED_TO_READ_RAW_DATA |
		    RM_NEED_TO_SEND_SIGNAL;

#if NOISE_SUM_CHECK
		if(ts->stTs.u8Repeat) {
			rm_printk("Change Noise Mode\n");
			rm_set_repeat_times(ts->stTs.u8Repeat);
			ts->stTs.u8Repeat=0;
		}
#endif

		break;
#ifdef ENABLE_AUTO_SCAN
	case RM_SCAN_MODE_PREPARE_AUTO:
		rm31080_ctrl_enter_auto_mode(ts);
		ts->stTs.u8ScanModeState = RM_SCAN_MODE_AUTO_SCAN;
		u32Flag = RM_NEED_NONE;
		break;
	case RM_SCAN_MODE_AUTO_SCAN:
		rm31080_ctrl_leave_auto_mode(ts);
		rm31080_ctrl_scan_start(ts);
		ts->stTs.u8ScanModeState = RM_SCAN_MODE_MANUAL;
#if ENABLE_T007B1_SETTING
		if (ts->stCtrl.bICVersion == T007_VERSION_B) {
			u32Flag =
				RM_NEED_TO_SEND_SCAN | RM_NEED_TO_READ_RAW_DATA |
				RM_NEED_TO_SEND_SIGNAL;
		} else
#endif
		{
			u32Flag = RM_NEED_TO_SEND_SCAN;
		}
		break;
#endif				/*ENABLE_AUTO_SCAN */
	default:
		u32Flag = RM_NEED_NONE;
		break;
	}

	return u32Flag;
}

#endif				/*ENABLE_RAW_DATA_QUEUE */

/*========================================================================= */
#ifdef ENABLE_RM31080_DEEP_SLEEP
static int rm31080_ctrl_suspend(struct rm31080_ts *ts)
{
	
	/*Flow designed by Roger 20110930 */
	/*rm31080_ts_send_signal(ts->stTs.ulHalPID,RM_SIGNAL_SUSPEND); */
	ts->stTs.bInitFinish = 0;

	mutex_lock(&ts->mutex_scan_mode);
	msleep(8);
	rm31080_ctrl_clear_int(ts);
	/*disable auto scan */
#if ENABLE_T007B1_SETTING
	if (ts->stCtrl.bICVersion != T007_VERSION_B)
#endif
	{
		rm31080_spi_byte_write(ts, RM31080_REG_09, 0x00);
		rm31080_spi_byte_write(ts, RM31080_REG_10, 0x14);
		rm31080_ctrl_scan_start(ts);
		rm31080_ctrl_scan_start(ts);
		msleep(15);
	}
	rm31080_spi_byte_write(ts, RM31080_REG_11, 0x06);
	mutex_unlock(&ts->mutex_scan_mode);
	return 1;
}
#endif

/*========================================================================= */
/* NOTE: should be protected by mutex_scan_mode */
static void rm31080_enter_manual_mode(struct rm31080_ts *ts)
{
	flush_workqueue(ts->rm_workqueue);

	if (ts->stTs.u8ScanModeState == RM_SCAN_MODE_MANUAL)
		return;

	if (ts->stTs.u8ScanModeState == RM_SCAN_MODE_PREPARE_AUTO) {
		ts->stTs.u8ScanModeState = RM_SCAN_MODE_MANUAL;
		return;
	}

	if (ts->stTs.u8ScanModeState == RM_SCAN_MODE_AUTO_SCAN) {
		rm31080_ctrl_leave_auto_mode(ts);
		ts->stTs.u8ScanModeState = RM_SCAN_MODE_MANUAL;
		msleep(10);
	}
}

/*=========================================================================
   Description:
        Copy Config(Parameters) to HAL's Buffer
   Input:
        p: HAL's buffer
        u32Len : buffer size
   Output:
        1: succeed
        0: failed
  ========================================================================= */
static long rm31080_get_config(struct rm31080_ts *ts, u8 * p, u32 u32Len)
{
	u32 u32Ret;
	struct rm_spi_ts_platform_data *pdata;

#if ENABLE_T007B1_SETTING
	u8 var;
	if (rm31080_spi_byte_read(ts, RM31080_REG_7E, &var))
		ts->stCtrl.bICVersion = var & 0xF0;
	else
		ts->stCtrl.bICVersion = T007A6;
#endif

	pdata = ts->input->dev.parent->platform_data;
#if ENABLE_T007B1_SETTING
	if (ts->stCtrl.bICVersion == T007_VERSION_B)
		u32Ret = copy_to_user(p, pdata->config + PARAMETER_AMOUNT, u32Len);
	else
#endif
		u32Ret = copy_to_user(p, pdata->config, u32Len);
	if (u32Ret != 0)
		return 0;
	return 1;
}

static u32 rm31080_get_platform_id(struct rm31080_ts *ts, u8 * p)
{
	u32 u32Ret;
	struct rm_spi_ts_platform_data *pdata;
	pdata = ts->input->dev.parent->platform_data;
	u32Ret = copy_to_user(p, &pdata->platform_id, sizeof(pdata->platform_id));
	if (u32Ret != 0)
		return 0;
	return 1;
}

/*=========================================================================*/
static int rm31080_ts_send_signal(int pid, int iInfo)
{
	struct siginfo info;
	struct task_struct *t;
	int ret = 0;

	static DEFINE_MUTEX(lock);

	if (!pid)
		return ret;

	mutex_lock(&lock);
	/* send the signal */
	memset(&info, 0, sizeof(struct siginfo));
	info.si_signo = RM_TS_SIGNAL;
	info.si_code = SI_QUEUE;	/* this is bit of a trickery: SI_QUEUE is normally used by sigqueue from user space, */
	/* and kernel space should use SI_KERNEL. But if SI_KERNEL is used the real_time data */
	/* is not delivered to the user space signal handler function. */
	info.si_int = iInfo;	/*real time signals may have 32 bits of data. */

	rcu_read_lock();
	t = find_task_by_vpid(pid);
	rcu_read_unlock();
	if (t == NULL) {
		rmd_printk("no such pid\n");
		ret = -ENODEV;
	} else
		ret = send_sig_info(RM_TS_SIGNAL, &info, t);	/*send the signal */

	if (ret < 0) {
		rmd_printk("error sending signal\n");
	}
	mutex_unlock(&lock);
	return ret;
}

/*=========================================================================
   Description:
        Queuing functions.
   Input:
        N/A
   Output:
        0:succeed
        others:error code
  ========================================================================= */
#ifdef ENABLE_RAW_DATA_QUEUE
static void rm31080_queue_reset(void)
{
	g_stQ.u16Rear = 0;
	g_stQ.u16Front = 0;
}

static int rm31080_queue_init(void)
{
	rm31080_queue_reset();
	g_stQ.pQueue = kmalloc(QUEUE_COUNT * RAW_DATA_LENGTH, GFP_KERNEL);
	if (g_stQ.pQueue == NULL) {
		return -ENOMEM;
	}
	return 0;
}

static void rm31080_queue_free(void)
{
	if (!g_stQ.pQueue)
		return;
	kfree(g_stQ.pQueue);
	g_stQ.pQueue = NULL;
}

#ifdef ENABLE_CALC_QUEUE_COUNT
static int rm31080_queue_get_current_count(void)
{
	if (g_stQ.u16Rear >= g_stQ.u16Front)
		return g_stQ.u16Rear - g_stQ.u16Front;

	return (QUEUE_COUNT - g_stQ.u16Front) + g_stQ.u16Rear;
}
#endif

/*=========================================================================
   Description:
    About full/empty buffer distinction,
    There are a number of solutions like:
    1.Always keep one slot open.
    2.Use a fill count to distinguish the two cases.
    3.Use read and write counts to get the fill count from.
    4.Use absolute indices.
    we chose "keep one slot open" to make it simple and robust
    and also avoid race condition.
   Input:
        N/A
   Output:
        1:empty
        0:not empty
  ========================================================================= */
static int rm31080_queue_is_empty(void)
{
	if (g_stQ.u16Rear == g_stQ.u16Front)
		return 1;
	return 0;
}

/*=========================================================================
   Description:
    check queue full.
   Input:
        N/A
   Output:
        1:full
        0:not full
  ========================================================================= */
static int rm31080_queue_is_full(void)
{
	u16 u16Front = g_stQ.u16Front;

	if (g_stQ.u16Rear + 1 == u16Front)
		return 1;

	if ((g_stQ.u16Rear == (QUEUE_COUNT - 1)) && (u16Front == 0))
		return 1;

	return 0;
}

static void *rm31080_enqueue_start(void)
{
	if (!g_stQ.pQueue)	/*error handling for no memory */
		return NULL;

	if (!rm31080_queue_is_full())
		return &g_stQ.pQueue[g_stQ.u16Rear];

	rm_printk("rm31080:touch service is busy,try again.\n");
	return NULL;
}

static void rm31080_enqueue_finish(void)
{
	if (g_stQ.u16Rear == (QUEUE_COUNT - 1))
		g_stQ.u16Rear = 0;
	else
		g_stQ.u16Rear++;
}

static void *rm31080_dequeue_start(void)
{
	if (!rm31080_queue_is_empty())
		return &g_stQ.pQueue[g_stQ.u16Front];

	return NULL;
}

static void rm31080_dequeue_finish(void)
{
	if (g_stQ.u16Front == (QUEUE_COUNT - 1))
		g_stQ.u16Front = 0;
	else
		g_stQ.u16Front++;
}

static long rm31080_queue_read_raw_data(u8 * p, u32 u32Len)
{
	u8 *pQueue;
	u32 u32Ret;

	pQueue = rm31080_dequeue_start();
	if (!pQueue)
		return 0;

	u32Ret = copy_to_user(p, pQueue, u32Len);
	if (u32Ret != 0)
		return 0;

	rm31080_dequeue_finish();
	time_queue_dequeue_flag = 1;
	return 1;
}
#endif				/*ENABLE_RAW_DATA_QUEUE */
/*=====================================================================*/


static int rm31080_time_queue_is_empty(void)
{
	if (g_ttQ.u16Rear == g_ttQ.u16Front)
		return 1;
	return 0;
}

static int rm31080_time_queue_is_full(void)
{
	u16 u16Front = g_ttQ.u16Front;
	if (g_ttQ.u16Rear + 1 == u16Front)
		return 1;

	if ((g_ttQ.u16Rear == (TIME_QUEUE_COUNT - 1)) && (u16Front == 0))
		return 1;

	return 0;
}

static void rm31080_time_queue_reset(void)
{
	g_ttQ.u16Rear = 0;
	g_ttQ.u16Front = 0;
}

static int rm31080_time_queue_init(void)
{
	rm31080_time_queue_reset();
	g_ttQ.time = kmalloc(TIME_QUEUE_COUNT * sizeof(struct timespec), GFP_KERNEL);
	if (g_ttQ.time == NULL) {
		return -ENOMEM;
	}

	return 0;
}

static void rm31080_time_queue_free(void)
{
	if (!g_ttQ.time)
		return;
	kfree(g_ttQ.time);
	g_ttQ.time = NULL;
}

static void rm31080_time_enqueue_finish(void)
{
	if (g_ttQ.u16Rear == (TIME_QUEUE_COUNT - 1))
		g_ttQ.u16Rear = 0;
	else
		g_ttQ.u16Rear++;
}

static void rm31080_time_dequeue_finish(void)
{
	if (g_ttQ.u16Front == (TIME_QUEUE_COUNT - 1))
		g_ttQ.u16Front = 0;
	else
		g_ttQ.u16Front++;
}

static int rm31080_time_queue_get_current_count(void)
{
	if (g_ttQ.u16Rear >= g_ttQ.u16Front)
		return g_ttQ.u16Rear - g_ttQ.u16Front;

	return (TIME_QUEUE_COUNT - g_ttQ.u16Front) + g_ttQ.u16Rear;
}

#ifdef ENABLE_WORK_QUEUE
static void rm_work_handler(struct work_struct *work)
{
	struct rm31080_ts *ts = container_of(work, struct rm31080_ts, rm_work);
	void *pKernelBuffer;
	u32 u32Flag;
	int iRet;

	struct timespec time;
	if (!rm31080_time_queue_is_full()) {
		ktime_get_ts(&time);
	}

	if (ts->stTs.bIsSuspended)
		return;

	mutex_lock(&ts->mutex_scan_mode);

	iRet = rm31080_ctrl_clear_int(ts);

	u32Flag = rm31080_ctrl_configure(ts);

	if (u32Flag & RM_NEED_TO_SEND_SCAN) {
		rm31080_ctrl_scan_start(ts);
	}

	if (u32Flag & RM_NEED_TO_READ_RAW_DATA) {
		pKernelBuffer = rm31080_enqueue_start();
		if (pKernelBuffer) {
			iRet = rm31080_ctrl_read_raw_data(ts, (u8 *) pKernelBuffer);
#if !NOISE_SUM_CHECK
			if (iRet) {
				iRet = rm_noise_main(ts, (s8 *) pKernelBuffer);
			}
#endif
			if (iRet) {
				rm31080_enqueue_finish();

				if (!rm31080_time_queue_is_full()) {
					if(g_ttQ.time){
						g_ttQ.time[g_ttQ.u16Rear] = time;
						rm31080_time_enqueue_finish();
					}
				}
			}
		}
	}
	mutex_unlock(&ts->mutex_scan_mode);

	if (u32Flag & RM_NEED_TO_SEND_SIGNAL) {
		if (ts->stTs.bCalcFinish) {
			ts->stTs.bCalcFinish = 0;
			rm31080_ts_send_signal(ts->stTs.ulHalPID, RM_SIGNAL_INTR);
		}
	}
}
#endif				/*ENABLE_WORK_QUEUE */

#ifdef ENABLE_RM31080_ESD_PROTECT
static bool rm31080_spi_checking2(struct rm31080_ts *ts, bool bInfinite)
{
    struct spi_device *spi = ts->spi_dev;
    unsigned int i;
    unsigned int iTestCount = 0;
    unsigned char wbuf[] = { 0x50, 0x55, 0xAA, 0x00, 0xFF };
    unsigned char rbuf[sizeof(wbuf)];
    unsigned int iLen;
    bool bFail = false;

#if ENABLE_T007B1_SETTING
    unsigned char var;
    if (rm31080_spi_byte_read(ts, RM31080_REG_7E, &var)) {
        if((var & 0xF0) != T007A6) {
            wbuf[0] = 0x4F;
            var = 0x01;
            rm31080_spi_byte_write(ts, RM31080_REG_7F, var);
        }
    }
#endif

    rbuf[0] = wbuf[0] | 0x80;       /*address*/
    iLen = sizeof(wbuf) - 1;
    do {
        spi_write(spi, wbuf, sizeof(wbuf));
        memset(&rbuf[1], 0, iLen);
        spi_write_then_read(spi, &rbuf[0], 1, &rbuf[1], iLen);

        /*compare*/
        bFail = false;
        for (i = 1; i < iLen + 1; i++) {
            if (wbuf[i] != rbuf[i]) {
                bFail = true;
                break;
            }
        }

        if (bFail) {
            rm_printk("##Raydium SPI Checking:Compare fail\n"); 
            rm_printk("##SPI Speed:%d hz,Mode:%x\n",
                      spi->max_speed_hz, spi->mode);
            for (i = 1; i < iLen + 1; i++) {
                rm_printk("##[%02d]Write Data:0x%02x ,Read Data:0x%02x\n", i, wbuf[i], rbuf[i]);
            }
            msleep(5);
        } else {
            iTestCount++;
        }
    } while (bInfinite);

    if (!bFail) {
        var = 0x00;
        rm31080_spi_byte_write(ts, RM31080_REG_7F, var);
        /* rm_printk("#Raydium SPI Checking2: ok, Speed:%d hz\n", spi->max_speed_hz); */
    }

    return (!bFail);
}
#endif

static void rm_idle_work_handler(struct work_struct *work)
{
    struct rm31080_ts *ts = container_of(work, struct rm31080_ts, rm_idle_work.work);
    bool spi_status = true;

	if (ts->stTs.bIsSuspended)
		return;

	mutex_lock(&ts->mutex_scan_mode);

    if (ts->stTs.u8ScanModeState != RM_SCAN_MODE_AUTO_SCAN) {
        mutex_unlock(&ts->mutex_scan_mode);
        goto polling;
    }

    /* Auto Scan mode */
    rm31080_spi_byte_write(ts, RM31080_REG_11, 0x37);
    rm31080_spi_byte_write(ts, RM31080_REG_11, 0x37);
    
#ifdef ENABLE_T007B1_SETTING
#ifdef ENABLE_RM31080_ESD_PROTECT
    if (ts->stCtrl.bICVersion == T007_VERSION_B) {
	if (ts->stTs.bInitFinish) {
	    spi_status = rm31080_spi_checking2(ts, 0);
	}
    }
#endif
#endif

	mutex_unlock(&ts->mutex_scan_mode);

#ifdef ENABLE_RM31080_ESD_PROTECT
    /* SPI check fail, restart touch */
	if(spi_status == false) {
		ts->stTs.bIsSuspended = 1;
		rm31080_start(ts);
		rm_printk("##WatchDog Resume\n");
	}
#endif

 polling:
    queue_delayed_work(ts->rm_idle_workqueue, &ts->rm_idle_work, 
                       MANUAL_IDLE_SCAN_PERIOD);

}

/*========================================================================= */
static void __rm31080_enable(struct rm31080_ts *ts)
{
	enable_irq(ts->irq);
}

static void __rm31080_disable(struct rm31080_ts *ts)
{
	disable_irq(ts->irq);
}

#ifdef ENABLE_SLOW_SCAN
static void rm31080_ctrl_ddsc_enable(struct rm31080_ts *ts)
{
	rm31080_spi_byte_write(ts, RM31080_REG_10, 0x40 | 0x10);
}

static void rm31080_ctrl_ddsc_disable(struct rm31080_ts *ts)
{
	rm31080_spi_byte_write(ts, RM31080_REG_10, 0x40);
}

static void rm31080_ctrl_hw_average(struct rm31080_ts *ts, int iTimes)
{
	switch (iTimes) {
	case 1:
		rm31080_spi_byte_write(ts, RM31080_REG_0E, 0x38);
		rm31080_spi_byte_write(ts, RM31080_REG_1F, 0x00);
		break;
	case 2:
		rm31080_spi_byte_write(ts, RM31080_REG_0E, 0x38 | 0x01);
		rm31080_spi_byte_write(ts, RM31080_REG_1F, 0x04);
		break;
	case 3:
		rm31080_spi_byte_write(ts, RM31080_REG_0E, 0x38 | 0x02);
		rm31080_spi_byte_write(ts, RM31080_REG_1F, 0x05);
		break;
	case 5:
		rm31080_spi_byte_write(ts, RM31080_REG_0E, 0x38 | 0x04);
		rm31080_spi_byte_write(ts, RM31080_REG_1F, 0x07);
		break;
	}
}

static void rm31080_ctrl_slowscan_cardhu(struct rm31080_ts *ts, u32 level)
{
	u8 buf[] = { RM31080_REG_40, 0x04, 0x38, 0x06, 0xA0 };
	rmd_printk("change level:%d\n", level);

	switch (level) {
	case RM_SLOW_SCAN_LEVEL_NORMAL:
		/*do nothing */
		break;
	case RM_SLOW_SCAN_LEVEL_20:	/*20Hz */
		buf[1] = 0x0F;
		buf[2] = 0xFF;
		buf[3] = 0x0A;
		buf[4] = 0xFA;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		rm31080_ctrl_hw_average(ts, 5);
		rm31080_spi_byte_write(ts, RM31080_REG_2F, 0x36);
		break;
	case RM_SLOW_SCAN_LEVEL_40:	/*40Hz */
		buf[3] = 0x06;
		buf[4] = 0xA0;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		rm31080_ctrl_hw_average(ts, 5);
		rm31080_spi_byte_write(ts, RM31080_REG_2F, 0x36);
		break;
	case RM_SLOW_SCAN_LEVEL_60:	/*60Hz */
		buf[3] = 0x04;
		buf[4] = 0x32;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		rm31080_ctrl_hw_average(ts, 5);
		rm31080_spi_byte_write(ts, RM31080_REG_2F, 0x36);
		break;
	case RM_SLOW_SCAN_LEVEL_80:	/*80Hz */
		buf[3] = 0x03;
		buf[4] = 0x20;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		rm31080_ctrl_hw_average(ts, 5);
		rm31080_spi_byte_write(ts, RM31080_REG_2F, 0x36);
		break;
	case RM_SLOW_SCAN_LEVEL_100:	/*100Hz*/
		buf[3] = 0x02;
		buf[4] = 0x90;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		rm31080_ctrl_hw_average(ts, 5);
		rm31080_spi_byte_write(ts, RM31080_REG_2F, 0x36);
		break;
	case RM_SLOW_SCAN_LEVEL_120:	/*120Hz*/
		rm31080_ctrl_ddsc_enable(ts);
		rm31080_ctrl_hw_average(ts, 5);
		rm31080_spi_byte_write(ts, RM31080_REG_2F, 0x32);
		break;
	case RM_SLOW_SCAN_LEVEL_140:	/*140Hz  default*/
		rm31080_ctrl_ddsc_enable(ts);
		rm31080_ctrl_hw_average(ts, 5);
		rm31080_spi_byte_write(ts, RM31080_REG_2F, 0x36);
		break;
	case RM_SLOW_SCAN_LEVEL_160:	/*160Hz*/
		rm31080_ctrl_ddsc_enable(ts);
		rm31080_ctrl_hw_average(ts, 3);
		rm31080_spi_byte_write(ts, RM31080_REG_2F, 0x50);
		break;
	case RM_SLOW_SCAN_LEVEL_180:	/*180Hz*/
		rm31080_ctrl_ddsc_enable(ts);
		rm31080_ctrl_hw_average(ts, 3);
		rm31080_spi_byte_write(ts, RM31080_REG_2F, 0x48);
		break;
	case RM_SLOW_SCAN_LEVEL_200:	/*200Hz*/
		rm31080_ctrl_ddsc_enable(ts);
		rm31080_ctrl_hw_average(ts, 3);
		rm31080_spi_byte_write(ts, RM31080_REG_2F, 0x3C);
		break;
	case RM_SLOW_SCAN_LEVEL_220:	/*220Hz*/
		rm31080_ctrl_ddsc_enable(ts);
		rm31080_ctrl_hw_average(ts, 3);
		rm31080_spi_byte_write(ts, RM31080_REG_2F, 0x34);
		break;
	case RM_SLOW_SCAN_LEVEL_240:	/*240Hz*/
		rm31080_ctrl_ddsc_enable(ts);
		rm31080_ctrl_hw_average(ts, 3);
		rm31080_spi_byte_write(ts, RM31080_REG_2F, 0x2C);
		break;
	default:
		break;
	}

}

static void rm31080_ctrl_slowscan_kai(struct rm31080_ts *ts, u32 level)
{
	u8 buf[] = { RM31080_REG_40, 0x04, 0x38, 0x06, 0xA0 };

	rmd_printk("change level:%d\n", level);

	switch (level) {
	case RM_SLOW_SCAN_LEVEL_NORMAL:
		/*do nothing*/
		break;
	case RM_SLOW_SCAN_LEVEL_20:	/*20Hz*/
		buf[1] = 0x0F;
		buf[2] = 0xFF;
		buf[3] = 0x0F;
		buf[4] = 0x80;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_spi_byte_write(ts, RM31080_REG_10, 0x41);
		break;
	case RM_SLOW_SCAN_LEVEL_40:	/*40Hz*/
		buf[1] = 0x0F;
		buf[2] = 0xFF;
		buf[3] = 0x0A;
		buf[4] = 0xFA;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		break;
	case RM_SLOW_SCAN_LEVEL_60:	/*60Hz*/
		buf[3] = 0x08;
		buf[4] = 0xC0;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		break;
	case RM_SLOW_SCAN_LEVEL_80:	/*80Hz*/
		buf[3] = 0x06;
		buf[4] = 0xA0;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		break;
	case RM_SLOW_SCAN_LEVEL_100:	/*100Hz*/
		buf[3] = 0x04;
		buf[4] = 0xFA;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		break;
	case RM_SLOW_SCAN_LEVEL_120:	/*120Hz*/
		buf[3] = 0x04;
		buf[4] = 0x32;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		break;
	case RM_SLOW_SCAN_LEVEL_140:	/*140Hz*/
		buf[3] = 0x03;
		buf[4] = 0xBA;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		break;
	case RM_SLOW_SCAN_LEVEL_160:	/*160Hz*/
		buf[3] = 0x03;
		buf[4] = 0x20;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		break;
	case RM_SLOW_SCAN_LEVEL_180:	/*180Hz*/
		buf[3] = 0x02;
		buf[4] = 0xC8;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		break;
	case RM_SLOW_SCAN_LEVEL_200:	/*200Hz*/
		buf[3] = 0x02;
		buf[4] = 0xA0;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		break;
	case RM_SLOW_SCAN_LEVEL_220:	/*220Hz*/
		buf[3] = 0x02;
		buf[4] = 0x50;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		break;
	case RM_SLOW_SCAN_LEVEL_240:	/*240Hz*/
		buf[3] = 0x02;
		buf[4] = 0x10;
		rm31080_spi_burst_write(ts, buf, sizeof(buf));
		rm31080_ctrl_ddsc_disable(ts);
		break;
	default:
		break;
	}

}

/*=========================================================================
   Description:
        Context dependent touch system.
        Change scan speed for slowscan function.
        Change scan speed flow: (by CY,20120305)
        1.Disable auto scan ([0x09]bit4=0,[0x09]bit6=1)
        2.Clear Scan start bit ([0x11]bit0=0)
        3.Read Scan start bit until it equals 0
        4.Set LACTIVE and YACTIVE configuration
        5.Enable autoscan ([0x09]bit4=1,[0x09]bit6=1)
        6.Sleep 1 minisecond.
        7.Set Scan start bit ([0x11]bit0=1)
   Input:
        N/A
   Output:
        N/A
  ========================================================================= */
/* NOTE: should be protected by mutex_scan_mode */
static void rm31080_ctrl_slowscan(struct rm31080_ts *ts, u32 level)
{
	if (ts->stTs.u8ScanModeState == RM_SCAN_MODE_AUTO_SCAN) {
		rm31080_ctrl_pause_auto_mode(ts);
	}

	rm31080_ctrl_wait_for_scan_finish(ts);

	if (ts->stCtrl.bChannelNumberX == RM_WINTEK_7_CHANNEL_X) {
		rm31080_ctrl_slowscan_kai(ts, level);
	} else {
		rm31080_ctrl_slowscan_cardhu(ts, level);
	}

	if (ts->stTs.u8ScanModeState == RM_SCAN_MODE_AUTO_SCAN) {
		rm31080_ctrl_enter_auto_mode(ts);
		msleep(1);
		rm31080_ctrl_scan_start(ts);
	}
}

static u32 rm31080_slowscan_round(u32 val)
{
	u32 i;

	for (i = RM_SLOW_SCAN_LEVEL_20; i < RM_SLOW_SCAN_LEVEL_240; i += 20) {
		if (i >= val)
			break;
	}
	return i;

}

static ssize_t rm31080_slowscan_handler(struct rm31080_ts *ts, const char *buf, size_t count)
{
	unsigned long val;
	ssize_t error;
	ssize_t ret;

	if (count < 2)
		return count;

	ret = (ssize_t) count;
	mutex_lock(&ts->mutex_scan_mode);

	if (count == 2) {
		if (buf[0] == '0') {
			ts->stTs.bEnableSlowScan = false;
			rm31080_ctrl_slowscan(ts, ts->stTs.u32SlowScanLevel);
		} else if (buf[0] == '1') {
			ts->stTs.bEnableSlowScan = true;
			rm31080_ctrl_slowscan(ts, RM_SLOW_SCAN_LEVEL_20);
		}
	} else if ((buf[0] == '2') && (buf[1] == ' ')) {
		error = strict_strtoul(&buf[2], 10, &val);
		if (error) {
			ret = error;
		} else {
			ts->stTs.bEnableSlowScan = false;
			ts->stTs.u32SlowScanLevel = rm31080_slowscan_round(val);
			rm31080_ctrl_slowscan(ts, ts->stTs.u32SlowScanLevel);
		}
	}

	mutex_unlock(&ts->mutex_scan_mode);
	return ret;
}
#endif

static ssize_t rm31080_slowscan_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
    struct miscdevice *mdev;
    struct rm31080_ts *ts;

    if (strcmp("raydium_ts", dev->kobj.name) == 0) { /* miscdev */
        mdev = dev_get_drvdata(dev);
        ts = container_of(mdev, struct rm31080_ts, miscdev);
    } else {                    /* spi dev */
        ts = dev_get_drvdata(dev);
    }

#ifdef ENABLE_SLOW_SCAN
	return sprintf(buf, "Slow Scan:%s\nScan Rate:%dHz\n",
				ts->stTs.bEnableSlowScan ?
				"Enabled" : "Disabled",
				ts->stTs.bEnableSlowScan ?
				RM_SLOW_SCAN_LEVEL_20 : ts->stTs.u32SlowScanLevel);

#else
	return sprintf(buf, "Not implemented yet\n");
#endif
}

static ssize_t rm31080_slowscan_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
    struct miscdevice *mdev;
    struct rm31080_ts *ts;

    if (strcmp("raydium_ts", dev->kobj.name) == 0) { /* miscdev */
        mdev = dev_get_drvdata(dev);
        ts = container_of(mdev, struct rm31080_ts, miscdev);
    } else {                    /* spi dev */
        ts = dev_get_drvdata(dev);
    }

#ifdef ENABLE_SLOW_SCAN
	return rm31080_slowscan_handler(ts, buf, count);
#else
	return count;
#endif
}

static void rm31080_smooth_level_change(struct rm31080_ts *ts, unsigned long val)
{
	int iInfo;

	if (val > RM_SMOOTH_LEVEL_MAX)
		return;

	ts->stTs.u32SmoothLevel = val;

	iInfo = (RM_SIGNAL_PARA_SMOOTH << 24) |
			(val << 16) |
			RM_SIGNAL_CHANGE_PARA;

	rm31080_ts_send_signal(ts->stTs.ulHalPID, iInfo);
}

static ssize_t rm31080_smooth_level_handler(struct rm31080_ts *ts, const char *buf, size_t count)
{
	unsigned long val;
	ssize_t error;
	ssize_t ret;

	if (count != 2)
		return count;

	ret = (ssize_t) count;
	error = strict_strtoul(buf, 10, &val);
	if (error) {
		ret = error;
	} else {
		rm31080_smooth_level_change(ts, val);
	}

	return ret;
}

static ssize_t rm31080_self_test_handler(struct rm31080_ts *ts, const char *buf, size_t count)
{
	unsigned long val;
	ssize_t error;
	ssize_t ret;
	int iInfo;

	ret = (ssize_t) count;

	if (count != 2)
		return ret;

	if (ts->stTs.u8SelfTestStatus == RM_SELF_TEST_STATUS_TESTING)
		return ret;
	ts->stTs.u8SelfTestResult = RM_SELF_TEST_RESULT_PASS;

	error = strict_strtoul(buf, 10, &val);
	if (error) {
		ret = error;
	} else if (val == 0) {
		ts->stTs.bInitFinish = 0;
		msleep(1000);
		printk("%s:%d\n", __func__, __LINE__);
		rm31080_spi_checking(ts, 1);
	} else if ((val >= 0x01) && (val <= 0xFF)) {
		ts->stTs.u8SelfTestStatus = RM_SELF_TEST_STATUS_TESTING;
		iInfo = (RM_SIGNAL_PARA_SELF_TEST << 24) |
				(val << 16) |
				RM_SIGNAL_CHANGE_PARA;
		rm31080_ts_send_signal(ts->stTs.ulHalPID, iInfo);
	}

	return ret;
}

static ssize_t rm31080_smooth_level_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
    struct miscdevice *mdev;
    struct rm31080_ts *ts;

    if (strcmp("raydium_ts", dev->kobj.name) == 0) { /* miscdev */
        mdev = dev_get_drvdata(dev);
        ts = container_of(mdev, struct rm31080_ts, miscdev);
    } else {                    /* spi dev */
        ts = dev_get_drvdata(dev);
    }

	return sprintf(buf, "Smooth level:%d\n", ts->stTs.u32SmoothLevel);
}

static ssize_t rm31080_smooth_level_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
    struct miscdevice *mdev;
    struct rm31080_ts *ts;

    if (strcmp("raydium_ts", dev->kobj.name) == 0) { /* miscdev */
        mdev = dev_get_drvdata(dev);
        ts = container_of(mdev, struct rm31080_ts, miscdev);
    } else {                    /* spi dev */
        ts = dev_get_drvdata(dev);
    }

	rm31080_smooth_level_handler(ts, buf, count);
	return count;
}

static ssize_t rm31080_self_test_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
    struct miscdevice *mdev;
    struct rm31080_ts *ts;

    if (strcmp("raydium_ts", dev->kobj.name) == 0) { /* miscdev */
        mdev = dev_get_drvdata(dev);
        ts = container_of(mdev, struct rm31080_ts, miscdev);
    } else {                    /* spi dev */
        ts = dev_get_drvdata(dev);
    }

	return sprintf(buf, "Self_Test:Status:%d ,Result:%d\n",
					ts->stTs.u8SelfTestStatus,
					ts->stTs.u8SelfTestResult);
}

static ssize_t rm31080_self_test_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
    struct miscdevice *mdev;
    struct rm31080_ts *ts;

    if (strcmp("raydium_ts", dev->kobj.name) == 0) { /* miscdev */
        mdev = dev_get_drvdata(dev);
        ts = container_of(mdev, struct rm31080_ts, miscdev);
    } else {                    /* spi dev */
        ts = dev_get_drvdata(dev);
    }

    rm31080_self_test_handler(ts, buf, count);
	return count;
}

static ssize_t rm31080_version_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
    struct miscdevice *mdev;
    struct rm31080_ts *ts;

    if (strcmp("raydium_ts", dev->kobj.name) == 0) { /* miscdev */
        mdev = dev_get_drvdata(dev);
        ts = container_of(mdev, struct rm31080_ts, miscdev);
    } else {                    /* spi dev */
        ts = dev_get_drvdata(dev);
    }

	return sprintf(buf, "0x%02X\n", ts->stTs.u8Version);
}

static ssize_t rm31080_version_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return count;
}

#if ENABLE_T007B1_SETTING
static ssize_t rm31080_module_detect_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
    struct miscdevice *mdev;
    struct rm31080_ts *ts;

    if (strcmp("raydium_ts", dev->kobj.name) == 0) { /* miscdev */
        mdev = dev_get_drvdata(dev);
        ts = container_of(mdev, struct rm31080_ts, miscdev);
    } else {                    /* spi dev */
        ts = dev_get_drvdata(dev);
    }

	return sprintf(buf, "%s\n", (ts->stCtrl.bICVersion == T007_VERSION_B) ? "RM31081" : "RM31080");
}

static ssize_t rm31080_module_detect_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	return count;
}
#endif

static ssize_t rm31080_time_queue_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return sprintf(buf, "%d\n",rm31080_time_queue_get_current_count());
}

static DEVICE_ATTR(slowscan_enable, 0644, rm31080_slowscan_show,
		   rm31080_slowscan_store);
static DEVICE_ATTR(smooth_level, 0644, rm31080_smooth_level_show,
		   rm31080_smooth_level_store);
static DEVICE_ATTR(self_test, 0644, rm31080_self_test_show,
		   rm31080_self_test_store);
static DEVICE_ATTR(version, 0644, rm31080_version_show,
		   rm31080_version_store);
#if ENABLE_T007B1_SETTING
static DEVICE_ATTR(module_detect, 0644, rm31080_module_detect_show,
		   rm31080_module_detect_store);
#endif
static DEVICE_ATTR(time_queue, 0444, rm31080_time_queue_show,
		   NULL);

static struct attribute *rm_ts_attributes[] = {
	&dev_attr_slowscan_enable.attr,
	&dev_attr_smooth_level.attr,
	&dev_attr_self_test.attr,
	&dev_attr_version.attr,
#if ENABLE_T007B1_SETTING
	&dev_attr_module_detect.attr,
#endif
	&dev_attr_time_queue.attr,
	NULL
};

static const struct attribute_group rm_ts_attr_group = {
	.attrs = rm_ts_attributes,
};

static int rm31080_input_open(struct input_dev *input)
{
	struct rm31080_ts *ts = input_get_drvdata(input);

	if (!ts->disabled && !ts->suspended)
		__rm31080_enable(ts);

	return 0;
}

static void rm31080_input_close(struct input_dev *input)
{
	struct rm31080_ts *ts = input_get_drvdata(input);

	if (!ts->disabled && !ts->suspended)
		__rm31080_disable(ts);
}

/*=========================================================================*/
static irqreturn_t rm31080_irq(int irq, void *handle)
{
	struct rm31080_ts *ts = handle;

	if (!ts->stTs.bInitFinish)
		return IRQ_HANDLED;

#ifdef ENABLE_WORK_QUEUE
	queue_work(ts->rm_workqueue, &ts->rm_work);
#endif
	return IRQ_HANDLED;
}

/*=============================================================================*/
static void rm31080_init_ts_structure_part(struct rm31080_ts *ts)
{
	ts->stTs.bInitFinish = 0;
	ts->stTs.bCalcFinish = 0;
	ts->stTs.bEnableScriber = 0;
	ts->stTs.bIsSuspended = 0;
	ts->stTs.bEnableAutoScan = 1;

	ts->stTs.bEnableSlowScan = false;

#ifdef ENABLE_RAW_DATA_QUEUE
	ts->stTs.u8ScanModeState = RM_SCAN_MODE_MANUAL;
#endif
	rm31080_ctrl_init(ts);
}

static void rm31080_disable_touch(struct rm31080_ts *ts)
{
	ts->stTs.bInitFinish = 0;
}

static void rm31080_enable_touch(struct rm31080_ts *ts)
{
	ts->stTs.bInitFinish = 1;
}

static void rm31080_set_autoscan(struct rm31080_ts *ts, unsigned char val)
{
	ts->stTs.bEnableAutoScan = val;
}

static void rm31080_set_variable(struct rm31080_ts *ts, unsigned int index, unsigned int arg)
{
    struct spi_device *spi = ts->spi_dev;

	switch (index) {
	case RM_VARIABLE_SELF_TEST_RESULT:
		ts->stTs.u8SelfTestResult = (u8) arg;
		ts->stTs.u8SelfTestStatus = RM_SELF_TEST_STATUS_FINISH;
		break;
	case RM_VARIABLE_SCRIBER_FLAG:
		ts->stTs.bEnableScriber = (bool) arg;
		break;
	case RM_VARIABLE_AUTOSCAN_FLAG:
		ts->stTs.bEnableAutoScan = (bool) arg;
		break;
	case RM_VARIABLE_VERSION:
		ts->stTs.u8Version = (u8) arg;
		dev_info(&spi->dev,"Raydium TS:Firmware v%d\n",
				ts->stTs.u8Version);
		break;
	case RM_VARIABLE_REPEAT:
		rm_printk("Repeat %d\n", arg);
		ts->stTs.u8Repeat = (u8) arg;
		break;
	default:
		break;
	}

}
static u32 rm31080_get_variable(struct rm31080_ts *ts, unsigned int index, unsigned int arg)
{
	u32 ret = 0;

	switch (index)
	{
		case RM_VARIABLE_PLATFORM_ID:
			ret = rm31080_get_platform_id(ts, (u8 *) arg);
			break;
		default:
			break;
	}

	return ret;
}

static void rm31080_init_ts_structure(struct rm31080_ts *ts)
{
	/* ts->stTs.ulHalPID = 0; */
	memset(&ts->stTs, 0, sizeof(struct rm31080a_ts_para));

#ifdef ENABLE_SLOW_SCAN
	ts->stTs.u32SlowScanLevel = RM_SLOW_SCAN_LEVEL_240;
#endif

#ifdef ENABLE_WORK_QUEUE
	ts->rm_workqueue = create_singlethread_workqueue("rm_work");
	INIT_WORK(&ts->rm_work, rm_work_handler);
#endif

#ifdef ENABLE_MANUAL_IDLE_MODE
	ts->rm_idle_workqueue = create_singlethread_workqueue("rm_idle_work");
	INIT_DELAYED_WORK(&ts->rm_idle_work, rm_idle_work_handler);
#endif
	mutex_init(&ts->mutex_scan_mode);
}

/*=========================================================================*/
#ifdef ENABLE_SPI_SETTING
static int rm31080_spi_setting(struct rm31080_ts *ts, u32 speed)
{
    struct spi_device *spi = ts->spi_dev;
	int err;

	if ((speed == 0) || (speed > 18))
		return false;

	spi->max_speed_hz = speed * 1000 * 1000;
	err = spi_setup(spi);
	if (err) {
		dev_dbg(&spi->dev, "Change SPI setting failed\n");
		return false;
	}
	return true;
}
#endif
/*=========================================================================*/
static bool rm31080_spi_checking(struct rm31080_ts *ts, bool bInfinite)
{
    struct spi_device *spi = ts->spi_dev;
	unsigned int i;
	unsigned int iTestCount = 0;
	unsigned char wbuf[] = { 0x50, 0x55, 0xAA, 0x00, 0xFF };
	unsigned char rbuf[sizeof(wbuf)];
	unsigned int iLen;
	bool bFail = false;

#if ENABLE_T007B1_SETTING
	unsigned char var;
	if (rm31080_spi_byte_read(ts, RM31080_REG_7E, &var)) {
		if((var & 0xF0) != T007A6) {
			wbuf[0] = 0x6E;
			var = 0x01;
			rm31080_spi_byte_write(ts, RM31080_REG_7F, var);
		}
	}
#endif

	rbuf[0] = wbuf[0] | 0x80;	/*address*/
	iLen = sizeof(wbuf) - 1;
	do {
		spi_write(spi, wbuf, sizeof(wbuf));
		memset(&rbuf[1], 0, iLen);
		spi_write_then_read(spi, &rbuf[0], 1, &rbuf[1], iLen);

		/*compare*/
		bFail = false;
		for (i = 1; i < iLen + 1; i++) {
			if (wbuf[i] != rbuf[i])	{
				bFail = true;
				break;
			}
		}

		if (bFail) {
			rm_printk("Raydium SPI Checking:Compare fail\n");
			rm_printk("SPI Speed:%d hz,Mode:%x\n",
					spi->max_speed_hz, spi->mode);
			for (i = 1; i < iLen + 1; i++) {
				rm_printk("[%02d]Write Data:0x%02x ,Read Data:0x%02x\n", i, wbuf[i], rbuf[i]);
			}
			msleep(300);
		} else {
			iTestCount++;
			if ((iTestCount % 10000) == 0) {
				rm_printk
					("SPI test:%d times ok.(Speed:%d hz,Mode:%x)\n",
					iTestCount, spi->max_speed_hz,
					spi->mode);
			}
		}
	} while (bInfinite);

	if (!bFail)	{
#if ENABLE_T007B1_SETTING
		if (rm31080_spi_byte_read(ts, RM31080_REG_7E, &var)) {
			if((var & 0xF0) != T007A6) {
				var = 0x00;
				rm31080_spi_byte_write(ts, RM31080_REG_7F, var);
				if (!rm31080_spi_frame_checking(ts, 0x55))
					bFail = true;
			}
		}
#endif
		rm_printk("Raydium SPI Checking: ok, Speed:%d hz\n", spi->max_speed_hz);
	}

	return (!bFail);

}

/*=========================================================================*/
static void rm31080_start(struct rm31080_ts *ts)
{
#ifdef ENABLE_RM31080_DEEP_SLEEP
	struct rm_spi_ts_platform_data *pdata;
#endif

	if (!ts->stTs.bIsSuspended)
		return;

	ts->stTs.bIsSuspended = false;

	mutex_lock(&ts->mutex_scan_mode);
#ifdef ENABLE_RM31080_DEEP_SLEEP
	/*flow designed by Roger 20110930*/
	pdata = ts->input->dev.parent->platform_data;
	gpio_set_value(pdata->gpio_reset, 0);
	msleep(120);
	gpio_set_value(pdata->gpio_reset, 1);
	msleep(10);
	rm31080_init_ts_structure_part(ts);
	rm31080_ts_send_signal(ts->stTs.ulHalPID, RM_SIGNAL_RESUME);
#elif defined(ENABLE_AUTO_SCAN)
	rm31080_ctrl_clear_int();
	rm31080_ctrl_scan_start();
#endif
	mutex_unlock(&ts->mutex_scan_mode);
}

static void rm31080_stop(struct rm31080_ts *ts)
{
	if (ts->stTs.bIsSuspended)
		return;

	ts->stTs.bIsSuspended = true;

    cancel_delayed_work_sync(&ts->rm_idle_work);
	flush_workqueue(ts->rm_workqueue);

#ifdef ENABLE_RM31080_DEEP_SLEEP
	rm31080_ctrl_suspend(ts);
#endif
}

#ifdef CONFIG_PM
static int rm31080_suspend(struct device *dev)
{
	struct rm_spi_ts_platform_data *pdata;
	struct rm31080_ts *ts = dev_get_drvdata(dev);

	rm31080_stop(ts);

	pdata = ts->input->dev.parent->platform_data;
	if(pdata->power_off)
		pdata->power_off();
	
	return 0;
}

static int rm31080_resume(struct device *dev)
{
	struct rm_spi_ts_platform_data *pdata;
	struct rm31080_ts *ts = dev_get_drvdata(dev);

	pdata = ts->input->dev.parent->platform_data;
	if(pdata->power_on)
		pdata->power_on();

	msleep(120);
	ts = dev_get_drvdata(dev);
	rm31080_start(ts);
	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void rm31080_early_suspend(struct early_suspend *es)
{
	struct rm31080_ts *ts;
	struct device *dev;

	ts = container_of(es, struct rm31080_ts, early_suspend);
	dev = ts->dev;

	if (rm31080_suspend(dev) != 0) {
		dev_err(dev, "%s: failed\n", __func__);
	}
}

static void rm31080_early_resume(struct early_suspend *es)
{
	struct rm31080_ts *ts;
	struct device *dev;

	ts = container_of(es, struct rm31080_ts, early_suspend);
	dev = ts->dev;

	if (rm31080_resume(dev) != 0) {
		dev_err(dev, "%s: failed\n", __func__);
	}
}
#else
static const struct dev_pm_ops rm31080_pm_ops = {
	.suspend = rm31080_suspend,
	.resume = rm31080_resume,
};
#endif				//CONFIG_HAS_EARLYSUSPEND
#endif				//CONFIG_PM

static void rm31080_set_input_resolution(struct rm31080_ts *ts, unsigned int x, unsigned int y)
{
	input_set_abs_params(ts->input, ABS_X, 0, x - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_Y, 0, y - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, x - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, y - 1, 0, 0);
}

static int rm31080_input_init(struct rm31080_ts *ts, unsigned int irq)
{
	/* struct rm31080_ts *ts; */
	struct input_dev *input_dev;
	int err;

	if (!irq) {
		dev_err(ts->dev, "no IRQ?\n");
		err = -EINVAL;
		goto err_out;
	}

	input_dev = input_allocate_device();

	if (!input_dev) {
		dev_err(ts->dev, "Failed to allocate memory\n");
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->input = input_dev;
	ts->irq = irq;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(ts->dev));

	input_dev->name = "raydium_ts";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = ts->dev;
	input_dev->id.bustype = ts->bops->bustype;

	input_dev->open = rm31080_input_open;
	input_dev->close = rm31080_input_close;
	input_dev->hint_events_per_packet = 256U;

	input_set_drvdata(input_dev, ts);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	rm31080_set_input_resolution(ts, RM_INPUT_RESOLUTION_X,
                                 RM_INPUT_RESOLUTION_Y);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 0xFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 32, 0, 0);

	err = request_threaded_irq(ts->irq, NULL, rm31080_irq,
					IRQF_TRIGGER_RISING, dev_name(ts->dev), ts);
	if (err) {
		dev_err(ts->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = rm31080_early_suspend;
	ts->early_suspend.resume = rm31080_early_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	__rm31080_disable(ts);

	err = sysfs_create_group(&ts->dev->kobj, &rm_ts_attr_group);
	if (err)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr;

	return 0;

 err_remove_attr:
	sysfs_remove_group(&ts->dev->kobj, &rm_ts_attr_group);
 err_free_irq:
	free_irq(ts->irq, ts);
 err_free_mem:
	input_free_device(input_dev);
 err_out:
	return err;
}

static int dev_open(struct inode *inode, struct file *filp)
{
    struct miscdevice *mdev = filp->private_data;
    struct rm31080_ts *ts = container_of(mdev, struct rm31080_ts, miscdev);

    filp->private_data = ts;
	return 0;
}

static int dev_release(struct inode *inode, struct file *filp)
{
    struct rm31080_ts *ts = filp->private_data;

	ts->stTs.bInitFinish = 0;
    mutex_lock(&ts->mutex_scan_mode);
	rm31080_enter_manual_mode(ts);
    mutex_unlock(&ts->mutex_scan_mode);
	return 0;
}

static ssize_t
dev_read(struct file *filp, char __user * buf, size_t count, loff_t * pos)
{
    struct rm31080_ts *ts = filp->private_data;
	ssize_t missing, status;
	int ret;
	u8 *pMyBuf;

	pMyBuf = kmalloc(count, GFP_KERNEL);
	if (pMyBuf == NULL)
		return -ENOMEM;

	pMyBuf[0] = buf[0];
	ret = rm31080_spi_read(ts, pMyBuf[0], pMyBuf, count);

	if (ret) {
		status = count;
		missing = copy_to_user(buf, pMyBuf, count);

		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	} else {
		status = -EFAULT;
		rmd_printk("rm31080_spi_read() fail\n");
	}

	kfree(pMyBuf);
	return status;
}

static ssize_t
dev_write(struct file *filp, const char __user * buf,
		size_t count, loff_t * pos)
{
    struct rm31080_ts *ts = filp->private_data;
	u8 *pMyBuf;
	int ret;
	unsigned long missing;
	ssize_t status = 0;

	pMyBuf = kmalloc(count, GFP_KERNEL);
	if (pMyBuf == NULL)
		return -ENOMEM;

	missing = copy_from_user(pMyBuf, buf, count);
	if (missing == 0) {
		ret = rm31080_spi_write(ts, pMyBuf, count);
		if (ret)
			status = count;
		else
			status = -EFAULT;
	} else
		status = -EFAULT;

	kfree(pMyBuf);
	return status;
}

/*=========================================================================
   Description:
        I/O Control routin.
   Input:
        file:
        cmd :
        arg :
   Output:
        1: succeed
        0: failed
   Note: To avoid context switch,please don't add debug message in this function.
  ========================================================================= */
static long dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct rm31080_ts *ts = file->private_data;
	long ret = true;
	unsigned int index;
    /* printk("%s: cmd=0x%x\n", __func__, cmd); */

	index = (cmd >> 16) & 0xFFFF;
	switch (cmd & 0xFFFF) {
	case RM_IOCTL_REPORT_POINT:
		raydium_report_pointer(ts, (void *)arg);
		break;
	case RM_IOCTL_SET_HAL_PID:
		ts->stTs.ulHalPID = arg;
		break;
	case RM_IOCTL_INIT_START:
		ts->stTs.bInitFinish = 0;
        mutex_lock(&ts->mutex_scan_mode);
		rm31080_enter_manual_mode(ts);
        mutex_unlock(&ts->mutex_scan_mode);
		break;
	case RM_IOCTL_INIT_END:
		ts->stTs.bInitFinish = 1;
		ts->stTs.bCalcFinish = 1;
#ifdef ENABLE_RAW_DATA_QUEUE
        mutex_lock(&ts->mutex_scan_mode);
		ret = rm31080_ctrl_scan_start(ts);
        mutex_unlock(&ts->mutex_scan_mode);
#endif
		break;
	case RM_IOCTL_FINISH_CALC:
		if(time_queue_dequeue_flag){
			if (!rm31080_time_queue_is_empty()) {
				rm31080_time_dequeue_finish();
			}
		}
		ts->stTs.bCalcFinish = 1;
		break;
	case RM_IOCTL_SCRIBER_CTRL:
		ts->stTs.bEnableScriber = (bool) arg;
		break;
	case RM_IOCTL_AUTOSCAN_CTRL:
		ts->stTs.bEnableAutoScan = (bool) arg;
		break;
#ifdef ENABLE_RAW_DATA_QUEUE
	case RM_IOCTL_READ_RAW_DATA:
		ret = rm31080_queue_read_raw_data((u8 *) arg, index);
		break;
#endif
	case RM_IOCTL_NOISE_CHECK:
		ret = rm31080_ctrl_get_noise_mode(ts, (u8 *) arg);
		break;
	case RM_IOCTL_GET_PARAMETER:
		rm31080_ctrl_get_parameter(ts, (void *)arg);

#if ENABLE_RESOLUTION_SWITCH
		rm31080_set_input_resolution(ts, ts->stCtrl.u16ResolutionX,
                                     ts->stCtrl.u16ResolutionY);
#endif
		break;
	case RM_IOCTL_SET_PARAMETER:
		ret = rm31080_get_config(ts, (u8 *) arg, index);
		break;
	case RM_IOCTL_SEND_BASELINE:	/* Noise_Detector*/
		rm31080_ctrl_set_baseline(ts, (void *)arg);
		break;
#if ENABLE_NEW_NOISE_MODE
	case RM_IOCTL_SEND_ANALOG_BASELINE:	/* Noise_Detector*/
		rm31080_ctrl_set_analog_baseline((void *)arg);
		break;
#endif
	case RM_IOCTL_SET_VARIABLE:
		rm31080_set_variable(ts, index, arg);
		break;

	case RM_IOCTL_GET_VARIABLE:
		ret = rm31080_get_variable(ts, index, arg);
		break;

	default:
		break;
	}

	return ret;
}

static struct file_operations dev_fops = {
	.owner = THIS_MODULE,
	.open = dev_open,
	.release = dev_release,
	.read = dev_read,
	.write = dev_write,
	.unlocked_ioctl = dev_ioctl,
};

static struct miscdevice raydium_ts_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "raydium_ts",
	.fops = &dev_fops,
};

static const struct rm31080_bus_ops rm31080_spi_bus_ops = {
	.bustype = BUS_SPI,
};

static int rm31080_misc_init(struct rm31080_ts *ts)
{
	int err;

	memcpy(&(ts->miscdev), &raydium_ts_miscdev, sizeof(struct miscdevice));

    ts->miscdev.parent = ts->dev;
    err = misc_register(&ts->miscdev);
    if (err < 0) {
        dev_err(ts->dev, "cannot register miscdev\n");
        return -ENODEV;
    }

	err = sysfs_create_group(&ts->miscdev.this_device->kobj,
				 &rm_ts_attr_group);
	if (err) {
        misc_deregister(&ts->miscdev);
		return -ENODEV;
    }
	return 0;
}

static int __devexit rm31080_spi_remove(struct spi_device *spi)
{
	struct rm31080_ts *ts = spi_get_drvdata(spi);
	
#ifdef ENABLE_MANUAL_IDLE_MODE
	if (ts->rm_idle_workqueue)
		destroy_workqueue(ts->rm_idle_workqueue);
#endif

	rm31080_time_queue_free();
#ifdef ENABLE_RAW_DATA_QUEUE
	rm31080_queue_free();
#endif

#ifdef ENABLE_WORK_QUEUE
	if (ts->rm_workqueue)
		destroy_workqueue(ts->rm_workqueue);
#endif
	sysfs_remove_group(&ts->miscdev.this_device->kobj,
			   &rm_ts_attr_group);
	misc_deregister(&ts->miscdev);
	sysfs_remove_group(&ts->dev->kobj, &rm_ts_attr_group);
	free_irq(ts->irq, ts);
	input_unregister_device(ts->input);
	kfree(ts);
	spi_set_drvdata(spi, NULL);
	return 0;
}

#if ENABLE_T007B1_SETTING
static bool rm31080_spi_frame_checking(struct rm31080_ts *ts, unsigned int ucheckbyte)
{
    struct spi_device *spi = ts->spi_dev;
	unsigned int i;
	unsigned int iTestCount = 3;
	u8 * pwbuf;
	u8 * prbuf;
	unsigned int iLen = 7000;
	bool bFail = false;

	pwbuf = kmalloc(iLen+1, GFP_KERNEL);	// Lenth = address byte + data bytes = 1 + 7000
	if (!pwbuf)
		return -ENOMEM;

	prbuf = kmalloc(iLen+1, GFP_KERNEL);
	if (!prbuf)	{
		kfree(pwbuf);
		return -ENOMEM;
	}

	memset(pwbuf, ucheckbyte, iLen+1);
	memset(pwbuf, 0x03, 1);
	memset(prbuf, *pwbuf | 0x80, 1);	/*address*/

	do {
		rm31080_spi_byte_write(ts, 0x05, 0x00);	// set index to baseline data
		rm31080_spi_byte_write(ts, RM31080B1_REG_BANK0_00H, 0x00);
		rm31080_spi_byte_write(ts, RM31080B1_REG_BANK0_01H, 0x00);	// set offset to beginning position
		rm31080_spi_burst_write(ts, pwbuf, iLen+1);
		memset(prbuf+1, 0, iLen);
		rm31080_spi_byte_write(ts, 0x05, 0x00);	// set index to baseline data
		rm31080_spi_byte_write(ts, RM31080B1_REG_BANK0_00H, 0x00);
		rm31080_spi_byte_write(ts, RM31080B1_REG_BANK0_01H, 0x00);	// set offset to beginning position
		rm31080_spi_read(ts, RM31080B1_REG_BANK0_03H|0x80, prbuf+1, iLen); /*0x80 for SPI burst read */

		/*compare*/
		bFail = false;
		for (i = 1; i < iLen; i++) {
			if (*(pwbuf+i) != *(prbuf+i)) {
				bFail = true;
				rm_printk("[%02d]Write Data:0x%02x ,Read Data:0x%02x\n",
                          i, *(pwbuf+i), *(prbuf+i));
				kfree(pwbuf);
				kfree(prbuf);
				return !bFail;
			}
		}

		if (bFail) {
			rm_printk("%s: Raydium SPI Checking:Compare fail\n", __func__);
			rm_printk("%s: SPI Speed:%d hz,Mode:%x\n", __func__, spi->max_speed_hz, spi->mode);
		} else {
			rm_printk("%s: SPI test:%d times ok.(Speed:%d hz,Mode:%x)\n",
                      __func__, iTestCount, spi->max_speed_hz, spi->mode);
		}
		iTestCount--;
	} while (iTestCount);

	kfree(pwbuf);
	kfree(prbuf);
	return (!bFail);
}
#endif

static int __devinit rm31080_spi_probe(struct spi_device *spi)
{
	struct rm31080_ts *ts;
    int ret;

    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (NULL == ts) {
        dev_err(&spi->dev, "Failed to allocated rm301080_ts!\n");
        return -ENOMEM;
    }

    ts->spi_dev = spi;
    ts->bops = &rm31080_spi_bus_ops;
    ts->dev = &spi->dev;

	rm31080_init_ts_structure(ts);
	rm31080_init_ts_structure_part(ts);

	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_err(&spi->dev, "SPI CLK %d Hz?\n", spi->max_speed_hz);
		return -EINVAL;
	}

	if (!rm31080_spi_checking(ts, 0))
		return -EINVAL;

    ret = rm31080_input_init(ts, spi->irq);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to initial input device!\n");
        goto err_input_init;
    }

	ret = rm31080_misc_init(ts);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to initialize misc device!\n");
        goto err_misc_init;
    }

#ifdef ENABLE_RAW_DATA_QUEUE
	ret = rm31080_queue_init();
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to initialize raw data queue!\n");
        goto err_queue_init;
    }
#endif

#ifdef ENABLE_MANUAL_IDLE_MODE
    queue_delayed_work(ts->rm_idle_workqueue, &ts->rm_idle_work,
                       MANUAL_IDLE_SCAN_PERIOD);
#endif

	spi_set_drvdata(spi, ts);
	rm31080_time_queue_init();
	return 0;

 err_queue_init:
    sysfs_remove_group(&ts->miscdev.this_device->kobj,
                       &rm_ts_attr_group);
    misc_deregister(&ts->miscdev);

 err_misc_init:
    sysfs_remove_group(&ts->dev->kobj, &rm_ts_attr_group);
    free_irq(ts->irq, ts);
    unregister_early_suspend(&ts->early_suspend);
    input_unregister_device(ts->input);

 err_input_init:
    kfree(ts);
    return -ENODEV;
}

static struct spi_driver rm31080_spi_driver = {
	.driver = {
			.name = "rm_ts_spidev",
			.bus = &spi_bus_type,
			.owner = THIS_MODULE,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
#if defined(CONFIG_PM)
			.pm = &rm31080_pm_ops,
#endif
#endif
			},
	.probe = rm31080_spi_probe,
	.remove = __devexit_p(rm31080_spi_remove),
};

static int __init rm31080_spi_init(void)
{
	return spi_register_driver(&rm31080_spi_driver);
}

static void __exit rm31080_spi_exit(void)
{
	spi_unregister_driver(&rm31080_spi_driver);
}

module_init(rm31080_spi_init);
module_exit(rm31080_spi_exit);

MODULE_AUTHOR("Valentine Hsu <valentine.hsu@rad-ic.com>");
MODULE_DESCRIPTION("Raydium touchscreen SPI bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:raydium-t007");
