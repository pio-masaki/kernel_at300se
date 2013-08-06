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
//=============================================================================
//INCLUDED FILES
//=============================================================================
#include <linux/device.h>
#include <asm/uaccess.h>	// copy_to_user(),
#include <linux/delay.h>
#include <linux/module.h>	// Module definition

#include <linux/spi/rm31080a_ts.h>
#include <linux/spi/rm31080a_ctrl.h>
//=============================================================================
//DEFINITIONS
//=============================================================================
#define RM31080_RAW_DATA_LENGTH 2048
#define MAX_AVERAGE_TIMES        128
//=============================================================================
//STRUCTURE DECLARATION
//=============================================================================

//=============================================================================
//GLOBAL VARIABLES DECLARATION
//=============================================================================

//Marty added
static u8 g_pbBaseline[RM31080_RAW_DATA_LENGTH];	// Noise_Detector
#if ENABLE_NEW_NOISE_MODE
static u8 g_pbAnalogBaseline[RM31080_RAW_DATA_LENGTH];	// Noise_Detector
#endif
signed int g_noiseLevel[3];	// Noise_Detector
unsigned short g_noisechecktimes;	// Noise_Detector

//Marty added
s8 g_bAverageBuf[MAX_AVERAGE_TIMES][RM31080_RAW_DATA_LENGTH];
s8 g_bMFBlockMax[MAX_AVERAGE_TIMES];
s8 g_bMFBlockMin[MAX_AVERAGE_TIMES];
u8 g_bMFCounter[MAX_AVERAGE_TIMES];
s8 g_bRawData[MAX_AVERAGE_TIMES][16];
u8 g_bfFirstAverage;
//Marty added
unsigned short u16ShowCnt = 0;
signed char bDTImage[60][8];
unsigned char bfSign[60];

//=============================================================================
//FUNCTION DECLARATION
//=============================================================================

//=============================================================================
// Description:
//      Control functions for Touch IC
// Input:
//
// Output:
//
//=============================================================================

static int rm31080_ctrl_clear_int(struct rm31080_ts *ts)
{
	u8 flag;

#if ENABLE_T007B1_SETTING
	if (ts->stCtrl.bICVersion == T007_VERSION_B)
		return rm31080_spi_byte_read(ts, RM31080B1_REG_BANK0_02H, &flag);
	else
#endif
		return rm31080_spi_byte_read(ts, RM31080_REG_F2, &flag);
}

static int rm31080_ctrl_scan_start(struct rm31080_ts *ts)
{
	return rm31080_spi_byte_write(ts, RM31080_REG_11, 0x17);
}

static void rm31080_ctrl_wait_for_scan_finish(struct rm31080_ts *ts)
{
	u8 u8reg11;
	int i;

	//50ms = 20Hz
	for (i = 0; i < 50; i++) {
		rm31080_spi_byte_read(ts, RM31080_REG_11, &u8reg11);
		if (u8reg11 & 0x01)
			msleep(1);
		else
			break;
	}
}

//=============================================================================
// Description:
//
// Input:
//      N/A
// Output:
//      N/A
//=============================================================================
//Marty added
static int rm31080_soft_average(struct rm31080_ts *ts, signed char *pSource)
{
	static u8 u8AverageIndex = 0;
	static u8 u8AverageIndexMinor = 0;
	static u8 u8StartAverage = 0;
	u16 i, j, k;
	s16 s16Sum;
	u8 bMaxBlockId;
	u8 TestA, TestB, TestC;
	s8 bMax, bMin;
	s8 bResult0, bResult1, bResult2;
	u8 bRes;
	u8 bAverageTimes;
	static u8 bAverageRatio = 0;

	bResult1 = 0;
	bResult2 = 0;
	bAverageTimes = ts->stCtrl.bNoiseRepeatTimes;

	if (ts->stCtrl.bfNoisePreHold) {
		u8AverageIndex = 0;
		u8AverageIndexMinor = 0;
		bAverageRatio = 0;
		g_bfFirstAverage = 0;

		return 0;
	}
#if ENABLE_NEW_NOISE_MODE
	if (!((ts->stCtrl.bfNoiseMode & 0x01) || ts->stCtrl.bfNoiseModeDetector))
#else
	if (!(ts->stCtrl.bfNoiseMode || ts->stCtrl.bfNoiseModeDetector))
#endif
	{
		u8AverageIndex = 0;
		u8AverageIndexMinor = 0;
		bAverageRatio = 0;
		g_bfFirstAverage = 0;
		if (ts->stCtrl.bfExitNoiseMode) {
			ts->stCtrl.bfExitNoiseMode = 0;
			return 0;
		} else {
			return 1;
		}
	}

	for (i = 0; i < ts->stCtrl.u16DataLength; i++) {	//RM31080_RAW_DATA_LENGTH =1530
		g_bAverageBuf[u8AverageIndex][i] = g_pbBaseline[i] - pSource[i];
	}
	u8AverageIndex++;
	if (ts->stCtrl.bNoiseRepeatTimes > ts->stCtrl.bNoisePipelineBase)
		u8AverageIndexMinor++;

	if (ts->stCtrl.bNoiseRepeatTimes <= ts->stCtrl.bNoisePipelineBase) {
		bAverageTimes = ts->stCtrl.bNoiseRepeatTimes;
		if (u8AverageIndex == ts->stCtrl.bNoiseRepeatTimes) {
			u8StartAverage = 1;
			u8AverageIndex = 0;
		} else
			u8StartAverage = 0;
	} else {
		if (!g_bfFirstAverage) {
			if (u8AverageIndex < ts->stCtrl.bNoiseRepeatTimes) {
				if (u8AverageIndexMinor == ts->stCtrl.bNoisePipelineBase) {
					u8StartAverage = 1;
					u8AverageIndexMinor = 0;
					bAverageRatio++;
					bAverageTimes = ts->stCtrl.bNoisePipelineBase * bAverageRatio;
				}
				else
					u8StartAverage = 0;
			} else {
				if (u8AverageIndex == ts->stCtrl.bNoiseRepeatTimes)	{
					u8AverageIndex = 0;
					if (u8AverageIndexMinor == ts->stCtrl.bNoisePipelineBase) {
						u8StartAverage = 1;
						u8AverageIndexMinor = 0;
						bAverageRatio++;
						bAverageTimes = ts->stCtrl.bNoisePipelineBase * bAverageRatio;
						g_bfFirstAverage = 1;
					} else {
						bAverageTimes = ts->stCtrl.bNoiseRepeatTimes;
						g_bfFirstAverage = 1;
						u8StartAverage = 0;
					}
				}
			}
		} else {
			bAverageTimes = ts->stCtrl.bNoiseRepeatTimes;
			if (u8AverageIndexMinor == ts->stCtrl.bNoisePipelineBase) {
				u8StartAverage = 1;
				u8AverageIndexMinor = 0;
			} else
				u8StartAverage = 0;

			if (u8AverageIndex == ts->stCtrl.bNoiseRepeatTimes) {
				u8AverageIndex = 0;
			}
		}
	}

	if (u8StartAverage) {
		for (i = 0; i < ts->stCtrl.u16DataLength; i++) {
			if (ts->stCtrl.bfMediumFilter & 0x01) {
				for (j = 0; j < ts->stCtrl.bMFBlockNumber; j++) {
					g_bMFCounter[j] = 0;
				}
				for (j = 0; j < bAverageTimes; j++) {
					for (k = 0; k < ts->stCtrl.bMFBlockNumber; k++) {
						if (g_bAverageBuf[j][i] <= g_bMFBlockMax[k] && g_bAverageBuf[j][i] >= g_bMFBlockMin[k]) {
							g_bRawData[g_bMFCounter[k]][k] = g_bAverageBuf[j][i];
							g_bMFCounter[k]++;
							break;
						}
					}
				}
				bMaxBlockId = 0;
				for (j = 0; j < ts->stCtrl.bMFBlockNumber; j++) {
					if (g_bMFCounter[j] > g_bMFCounter[bMaxBlockId]) {
						bMaxBlockId = j;
					} else if (g_bMFCounter[j] == g_bMFCounter[bMaxBlockId]) {
						if (j > ts->stCtrl.bMFBlockNumber / 2)
							TestA = j - ts->stCtrl.bMFBlockNumber / 2;
						else
							TestA = ts->stCtrl.bMFBlockNumber / 2 - j;
						if (bMaxBlockId > ts->stCtrl.bMFBlockNumber / 2)
							TestB = bMaxBlockId - ts->stCtrl.bMFBlockNumber / 2;
						else
							TestB = ts->stCtrl.bMFBlockNumber / 2 - bMaxBlockId;
						if (TestA < TestB) {
							bMaxBlockId = j;
						}
					}
				}
				//printk("ID = %d, Max Counter ID = %d, Count = %d\n", i, bMaxBlockId, g_bMFCounter[bMaxBlockId]);
				s16Sum = 0;
				for (j = 0; j < g_bMFCounter[bMaxBlockId]; j++) {
					s16Sum += g_bRawData[j][bMaxBlockId];
				}
				if (g_bMFCounter[bMaxBlockId] == 0) {
					bResult1 = 0;
				} else {
					bResult1 = (s16Sum / g_bMFCounter[bMaxBlockId]);
					//if (ts->stCtrl.bfNoiseDetector)
					//{
					//    bResult1 = (s16)bResult1 * 4 / 5;
					//}
				}
			}

			if (ts->stCtrl.bfMediumFilter & 0x02) {
				bMax = -128;
				bMin = 127;
				for (j = 0; j < bAverageTimes; j++) {
					if (g_bAverageBuf[j][i] > bMax)
						bMax = g_bAverageBuf[j][i];
					if (g_bAverageBuf[j][i] < bMin)
						bMin = g_bAverageBuf[j][i];
				}
				bResult2 = (bMax + bMin) / 2;
				//if (ts->stCtrl.bfNoiseDetector)
				//{
				//    bResult2 = (s16)bResult2 * 4 / 5;
				//}
			}

			s16Sum = 0;
			for (j = 0; j < bAverageTimes; j++) {
				bRes = i % (ts->stCtrl.bChannelNumberX + 2 + ts->stCtrl.bADCNumber);
				if (bRes > 0 && bRes < 4) {
					if (g_bAverageBuf[j][i] < ts->stCtrl.bNoiseThresholdLowMax && g_bAverageBuf[j][i] > ts->stCtrl.bNoiseThresholdLowMin)
						g_bAverageBuf[j][i] = 0;
					else if (g_bAverageBuf[j][i] > 40)
						g_bAverageBuf[j][i] = 40;
					else if (g_bAverageBuf[j][i] < -40)
						g_bAverageBuf[j][i] = -40;
				} else {
					if (g_bAverageBuf[j][i] < ts->stCtrl.bNoiseThresholdLowMax && g_bAverageBuf[j][i] > ts->stCtrl.bNoiseThresholdLowMin)
						g_bAverageBuf[j][i] = 0;
					else if (g_bAverageBuf[j][i] > ts->stCtrl.bNoiseThresholdMax)
						g_bAverageBuf[j][i] = ts->stCtrl.bNoiseThresholdMax;
					else if (g_bAverageBuf[j][i] < ts->stCtrl.bNoiseThresholdMin)
						g_bAverageBuf[j][i] = ts->stCtrl.bNoiseThresholdMin;
				}

				s16Sum += g_bAverageBuf[j][i];
			}
			bResult0 = (s16Sum / ts->stCtrl.bNoiseRepeatTimes);	// + 0x80;
			//if (ts->stCtrl.bfNoiseDetector)
			//{
			//    bResult0 = (s16)bResult0 * 4 / 5;
			//}

			if (ts->stCtrl.bfMediumFilter & 0x01) {
				if (bResult0 > 0)
					TestA = bResult0;
				else
					TestA = -bResult0;
				if (bResult1 > 0)
					TestB = bResult1;
				else
					TestB = -bResult1;
				if (TestA < TestB)
					pSource[i] = g_pbBaseline[i] - bResult0;
				else
					pSource[i] = g_pbBaseline[i] - bResult1;
			} else if (ts->stCtrl.bfMediumFilter & 0x02) {
				if (bResult0 > 0)
					TestA = bResult0;
				else
					TestA = -bResult0;
				if (bResult2 > 0)
					TestC = bResult2;
				else
					TestC = -bResult2;
				if (TestA < TestC)
					pSource[i] = g_pbBaseline[i] - bResult0;
				else
					pSource[i] = g_pbBaseline[i] - bResult2;
			} else if (ts->stCtrl.bfMediumFilter & 0x03) {
				if (bResult0 > 0)
					TestA = bResult0;
				else
					TestA = -bResult0;
				if (bResult1 > 0)
					TestB = bResult1;
				else
					TestB = -bResult1;
				if (bResult2 > 0)
					TestC = bResult2;
				else
					TestC = -bResult2;
				if ((TestA < TestB) && (TestA < TestC))
					pSource[i] = g_pbBaseline[i] - bResult0;
				else if ((TestB < TestA) && (TestB < TestC))
					pSource[i] = g_pbBaseline[i] - bResult1;
				else if ((TestC < TestA) && (TestC < TestB))
					pSource[i] = g_pbBaseline[i] - bResult2;

			} else {
				pSource[i] = g_pbBaseline[i] - bResult0;
			}
		}

		return 1;
	}
	return 0;
}

//=============================================================================
// Description:
//
// Input:
//      N/A
// Output:
//      N/A
//=============================================================================
static void rm_set_repeat_times(struct rm31080_ts *ts, u8 u8Times)
{
	u8 bReg1_1Fh = 0x00;
	u8 u8Reg = 0x00;

#if ENABLE_T007B1_SETTING
	if (ts->stCtrl.bICVersion == T007_VERSION_B)
	{
        u8Reg = ts->stCtrl.bSenseNumber - 1;
        rm31080_spi_byte_write(ts, 0x0A, u8Reg&0x0F);
        rm31080_spi_byte_write(ts, 0x0E, u8Times&0x1F);
		if ((ts->stCtrl.bfADFC) && (ts->stCtrl.bICVersion != T007_VERSION_C))
            bReg1_1Fh |= ADFC;

        if (ts->stCtrl.bfTHMode)
            bReg1_1Fh |= FILTER_THRESHOLD_MODE;
        else
            bReg1_1Fh &= ~FILTER_NONTHRESHOLD_MODE;
		//bReg1_1Fh |= FILTER_NONTHRESHOLD_MODE;

        if (u8Times != REPEAT_1)
            bReg1_1Fh |= 0x44;     //Marty 20120820

        rm31080_spi_byte_write(ts, RM31080_REG_1F, bReg1_1Fh);
	} else
#endif
	{
        u8Reg = ((ts->stCtrl.bSenseNumber - 1) << 3) | u8Times;

        rm31080_spi_byte_write(ts, RM31080_REG_0E, u8Reg);
        if (ts->stCtrl.bfADFC)
            bReg1_1Fh |= ADFC;

        if (ts->stCtrl.bfTHMode)
            bReg1_1Fh |= FILTER_THRESHOLD_MODE;
        else
            bReg1_1Fh &= ~FILTER_NONTHRESHOLD_MODE;
		//bReg1_1Fh |= FILTER_NONTHRESHOLD_MODE;

        if (u8Times != REPEAT_1)
            bReg1_1Fh |= (u8Times + 3);

        rm31080_spi_byte_write(ts, RM31080_REG_1F, bReg1_1Fh);
	}
}

//=============================================================================
// Description:
//
// Input:
//      N/A
// Output:
//      N/A
//=============================================================================
static NOISE_DETECTOR_RET_t rm_noise_detect(struct rm31080_ts *ts, signed char *pSource)
{
	NOISE_DETECTOR_RET_t tRet = ND_NORMAL;

	unsigned short Offset, XNum, i, j;
	signed int i32NoiseLevel = 0;
	signed int noise_value = 0;
	signed char bdata, bdata0;
	unsigned char bfTouched = 0;
	signed char bTestData = 0;
	unsigned char *pbBaseline;
	unsigned short wSENum = 2;


#if ENABLE_T007B1_SETTING
	if (ts->stCtrl.bICVersion == T007_VERSION_B)
	{
		wSENum = 0;
		return ND_DETECTOR_OFF;
	}
#endif

	if (!ts->stCtrl.bfNoiseDetector)
		return ND_DETECTOR_OFF;

#if ENABLE_NEW_NOISE_MODE
	if (ts->stCtrl.bfNoiseModeDetector)
		pSource[0] = 0x10;
	else
		pSource[0] = 0x00;

	if (ts->stCtrl.bfNoiseModeDetector && (ts->stCtrl.bfNoiseMode & 0x02))
		pbBaseline = g_pbAnalogBaseline;
	else
#endif
		pbBaseline = g_pbBaseline;

	if (ts->stCtrl.bBaselineReady) {
		if (ts->stCtrl.bChannelDetectorNum > 0) {
			bfTouched = 0;
			XNum = ts->stCtrl.bChannelNumberX + wSENum + ts->stCtrl.bADCNumber;
			Offset = XNum * (ts->stCtrl.bChannelNumberY + ts->stCtrl.bChannelDetectorDummy);
			for (i = 0; i < XNum; i++) {
				if (ts->stCtrl.bADCNumber) {
					if ((i == ts->stCtrl.bDummyChannel[0])
					    || (i == ts->stCtrl.bDummyChannel[1])
					    || (i == ts->stCtrl.bDummyChannel[2])
					    || (i == ts->stCtrl.bDummyChannel[3])) {
						continue;
					}
				} else {
					if ((i == ts->stCtrl.bDummyChannel[0]) || (i == ts->stCtrl.bDummyChannel[1])) {
						continue;
					}
				}

				bTestData = 0;
				for (j = 0; j < ts->stCtrl.bChannelNumberY; j++) {
					bTestData = pbBaseline[j * XNum + i] - pSource[j * XNum + i];
					if (bTestData > ts->stCtrl.bMTTouchThreshold || bTestData < -ts->stCtrl.bMTTouchThreshold) {
						bfTouched = 1;
						break;
					}
				}
				if (bfTouched == 1)
					break;
			}
			for (i = 0; i < XNum; i++) {
				bfSign[i] = 0;
				for (j = 0; j < ts->stCtrl.bChannelDetectorNum; j++) {
					bDTImage[i][j] = pbBaseline[Offset + j * XNum + i] - pSource[Offset + j * XNum + i];
				}
			}
			for (i = 0; i < XNum; i++) {
				bdata = 0;
				bdata0 = 0;
				for (j = 0; j < ts->stCtrl.bChannelDetectorNum; j++) {
					if (bDTImage[i][j] > 0) {
						bdata++;
					} else if (bDTImage[i][j] < 0) {
						bdata--;
					} else {
						bdata0++;
					}
				}
				if (((bdata + bdata0) == ts->stCtrl.bChannelDetectorNum) || ((bdata0 - bdata) == ts->stCtrl.bChannelDetectorNum)) {
					bfSign[i] = 1;
				}
			}

			i32NoiseLevel = 0;
			for (i = 0; i < XNum; i++) {
				if (ts->stCtrl.bADCNumber) {
					if ((i == ts->stCtrl.bDummyChannel[0])
						|| (i == ts->stCtrl.bDummyChannel[1])
						|| (i == ts->stCtrl.bDummyChannel[2])
						|| (i == ts->stCtrl.bDummyChannel[3])) {
						continue;
					}
				} else {
					if ((i == ts->stCtrl.bDummyChannel[0]) || (i == ts->stCtrl.bDummyChannel[1])) {
						continue;
					}
				}

				for (j = 0; j < ts->stCtrl.bChannelDetectorNum;  j++) {
					if (bDTImage[i][j] < 0)
						noise_value = -bDTImage[i][j];
					else
						noise_value = bDTImage[i][j];

					if (noise_value > 2 && ((bfSign[i] == 0 && ts->stCtrl.bfNoiseModeDetector == 0) || ts->stCtrl.bfNoiseModeDetector == 1)) {
						i32NoiseLevel += noise_value;
					}
				}
			}

			if (g_noisechecktimes == 0) {
				g_noiseLevel[0] = i32NoiseLevel;
			} else if (g_noisechecktimes == 1) {
				g_noiseLevel[1] = i32NoiseLevel;
			} else if (g_noisechecktimes == 2) {
				g_noiseLevel[2] = i32NoiseLevel;
			} else {
				g_noiseLevel[0] = g_noiseLevel[1];
				g_noiseLevel[1] = g_noiseLevel[2];
				g_noiseLevel[2] = i32NoiseLevel;
			}
			i32NoiseLevel = g_noiseLevel[0] + g_noiseLevel[1] + g_noiseLevel[2];
			g_noisechecktimes++;
			if (g_noisechecktimes > 50) {
				g_noisechecktimes = 50;
			}

			if (ts->stCtrl.bfNoiseModeDetector == 0) {
				if (i32NoiseLevel > ts->stCtrl.bNoiseDetectThd || ts->stCtrl.bfNoisePreHold > 5) {
					if ((g_noiseLevel[0] != 0) && (g_noiseLevel[1] != 0) && (g_noiseLevel[2] != 0)) {

						rm31080_disable_touch(ts);
						msleep(10);
						rm31080_ctrl_clear_int(ts);
#if ENABLE_NEW_NOISE_MODE
						if (ts->stCtrl.bfNoiseMode & 0x02) {
							rm31080_analog_filter_config(ts->stCtrl.bNewNoiseRepeatTimes);
						} else
#endif
						{
							rm_set_repeat_times(ts, REPEAT_1);
						}
						ts->stCtrl.bfNoiseModeDetector = 1;
						ts->stCtrl.bfNoisePreHold = 0;
						//printk("Enter Noise Mode! \n");
						tRet = ND_NOISE_DETECTED;

						rm31080_enable_touch(ts);
						rm31080_ctrl_scan_start(ts);
					}
				}
				else {
					if (!ts->stCtrl.bfTouched && bfTouched && i32NoiseLevel > 0)
						ts->stCtrl.bfNoisePreHold++;
					else if (ts->stCtrl.bfNoisePreHold && bfTouched && i32NoiseLevel > 0
						&& (g_noiseLevel[0] != 0)
						&& (g_noiseLevel[1] != 0)
						&& (g_noiseLevel[2] != 0))
						ts->stCtrl.bfNoisePreHold++;
					else
						ts->stCtrl.bfNoisePreHold = 0;
				}
			}
			else {
				if (ts->stCtrl.bfNoiseModeDetector == 1 && !ts->stCtrl.bfSuspendReset) {
					if (bfTouched == 0) {
						rm31080_disable_touch(ts);
						msleep(50);
						rm31080_ctrl_clear_int(ts);

#if ENABLE_NEW_NOISE_MODE
						if (ts->stCtrl.bfNoiseMode & 0x02) {
							if (ts->stCtrl.bfAnalogFilter)
								rm_set_repeat_times(ts, ts->stCtrl.bRepeatTimes[1]);
							else
								rm31080_digital_filter_config();
						} else
#endif
						{
							rm_set_repeat_times(ts, ts->stCtrl.bRepeatTimes[ts->stCtrl.bfAnalogFilter]);
						}
						ts->stCtrl.bfNoiseModeDetector = 0;
						ts->stCtrl.bfNoisePreHold = 0;
						ts->stCtrl.bfExitNoiseMode = 1;

						//printk("Exit Noise Mode! \n");
						tRet = ND_LEAVE_NOISE_MODE;

						rm31080_enable_touch(ts);
						rm31080_ctrl_scan_start(ts);
					}

				}
			}
		} else
			return ND_BASELINE_NOT_READY;

		ts->stCtrl.bfTouched = bfTouched;
	}

	return tRet;
}

//=============================================================================
static int rm_noise_main(struct rm31080_ts *ts, signed char *pSource)
{
	int iRet = 1;

	if (ND_NOISE_DETECTED == rm_noise_detect(ts, pSource))
		iRet = 0;

#if ENABLE_T007B1_SETTING
	if (ts->stCtrl.bICVersion != T007_VERSION_B)
#endif
	{
#if ENABLE_NEW_NOISE_MODE
		if (!(ts->stCtrl.bfNoiseMode & 0x02))
#endif
		{
			iRet = rm31080_soft_average(ts, pSource);
		}
	}

	return iRet;
}

//=============================================================================
// Description:
//
// Input:
//      N/A
// Output:
//      N/A
//=============================================================================
static void rm31080_ctrl_set_baseline(struct rm31080_ts *ts, void *arg)
{
	u8 *pRawPtr;
	u16 ii;

	pRawPtr = (u8 *) arg;
	for (ii = 0; ii < ts->stCtrl.u16DataLength; ii++) {
		g_pbBaseline[ii] = pRawPtr[ii + 1];
	}
	ts->stCtrl.bBaselineReady = 1;
}

#if ENABLE_NEW_NOISE_MODE
//=============================================================================
// Description:
//
// Input:
//      N/A
// Output:
//      N/A
//=============================================================================
static void rm31080_ctrl_set_analog_baseline(struct rm31080_ts *ts, void *arg)
{
	u8 *pRawPtr;
	u16 ii;

	pRawPtr = (u8 *) arg;
	for (ii = 0; ii < ts->stCtrl.u16DataLength; ii++) {
		g_pbAnalogBaseline[ii] = pRawPtr[ii];
	}
	ts->stCtrl.bBaselineReady = 1;
}
#endif
//=============================================================================
// Description:
//
// Input:
//      N/A
// Output:
//      N/A
//=============================================================================
static void rm31080_ctrl_init(struct rm31080_ts *ts)
{
#if ENABLE_T007B1_SETTING
	u8 var;
#endif

	memset(&ts->stCtrl, 0, sizeof(struct rm31080a_ctrl_para));
	//Marty added
	ts->stCtrl.bBaselineReady = 0;	// Noise_Detector
	g_noiseLevel[0] = 0;	// Noise_Detector
	g_noiseLevel[1] = 0;	// Noise_Detector
	g_noiseLevel[2] = 0;	// Noise_Detector
	g_noisechecktimes = 0;	// Noise_Detector
	ts->stCtrl.bfNoiseModeDetector = 0;
	ts->stCtrl.bfNoisePreHold = 0;
	ts->stCtrl.bfTouched = 0;
	ts->stCtrl.bfExitNoiseMode = 0;

	ts->stCtrl.u16DataLength = RM31080_RAW_DATA_LENGTH;
	g_bfFirstAverage = 0;

#if ENABLE_T007B1_SETTING
	if (rm31080_spi_byte_read(ts, RM31080_REG_7E, &var))
		ts->stCtrl.bICVersion = var & 0xF0;
	else
		ts->stCtrl.bICVersion = T007A6;
#endif
}

//=============================================================================
// Description:
//
// Input:
//      N/A
// Output:
//      N/A
//=============================================================================
static unsigned char rm31080_ctrl_get_noise_mode(struct rm31080_ts *ts, u8 * p)
{
	u32 u32Ret;

	u32Ret = copy_to_user(p, &ts->stCtrl.bfNoiseModeDetector, 1);
	if (u32Ret != 0)
		return 0;
	return 1;
}


#if ENABLE_T007B1_SETTING

static void rm31080b_sw_reset(struct rm31080_ts *ts)
{
	unsigned char u8Value;

	// sw reset
	rm31080_spi_byte_read(ts, RM31080B1_REG_BANK0_11H, &u8Value);
	u8Value &= ~0x04;
	rm31080_spi_byte_write(ts, RM31080B1_REG_BANK0_11H, u8Value);  // Send software reset.
}

//=============================================================================
// Description: Set T007B analog filter repeat
//
// Input:
//      Analog average tap number
// Output:
//      none
//=============================================================================
static void rm31080b_analog_filter_config(struct rm31080_ts *ts, unsigned char u8Amount)
{
	unsigned char u8Value;

	rm31080_spi_byte_read(ts, RM31080B1_REG_BANK0_0BH,&u8Value);
	u8Value &= ~0x0F;
	u8Value |= u8Amount;
	rm31080_spi_byte_write(ts, RM31080B1_REG_BANK0_0BH, u8Amount);

	rm31080b_sw_reset(ts);
}


//=============================================================================
// Description: Set T007B digital  filter repeat
//
// Input:
//      Digital average tap number
// Output:
//      none
//=============================================================================
static void rm31080b_digital_filter_config(struct rm31080_ts *ts, unsigned char u8Amount)
{
	unsigned char u8Value;

	rm31080_spi_byte_read(ts, RM31080B1_REG_BANK0_0EH, &u8Value);
	u8Value &= ~0x1F;
	u8Value |= u8Amount;
	rm31080_spi_byte_write(ts, RM31080B1_REG_BANK0_0EH, u8Value);

	rm31080b_sw_reset(ts);
}
#endif // #if ENABLE_T007B1_SETTING

//=============================================================================
// Description:
//
// Input:
//      N/A
// Output:
//      N/A
//=============================================================================

#if ENABLE_FILTER_SWITCH
static void rm31080_analog_filter_config(struct rm31080_ts *ts, u8 bRepeatTimes)
{
#if ENABLE_T007B1_SETTING
	if (ts->stCtrl.bICVersion != T007_VERSION_B)
#endif
	{
		rm31080_spi_byte_write(ts, 0x7F, 0x01);     // Switch to 0x01

		rm31080_spi_byte_write(ts, 0x09, ts->stCtrl.bReg1_09h[1]);
		rm31080_spi_byte_write(ts, 0x43, ts->stCtrl.bReg1_43h[1]);
		rm31080_spi_byte_write(ts, 0x48, ts->stCtrl.bReg1_48h[1]);
		rm31080_spi_byte_write(ts, 0x49, ts->stCtrl.bReg1_49h[1]);
		rm31080_spi_byte_write(ts, 0x4A, ts->stCtrl.bReg1_4Ah[1]);
		rm31080_spi_byte_write(ts, 0x4B, ts->stCtrl.bReg1_4Bh[1]);

		rm31080_spi_byte_write(ts, 0x7F, 0x00);     // Switch to BANK0

		rm_set_repeat_times(ts, bRepeatTimes);

	// -------- Set Driving / Sensing Control Mode --------
		rm31080_spi_byte_write(ts, RM31080_REG_10, 0x80);	//SSC_ENB

	// -------- Set PGA/DAC --------
		rm31080_spi_byte_write(ts, 0x6B, 0xF1);	//CSSEL

	// -------- Scan Time Setting --------
		rm31080_spi_byte_write(ts, RM31080_REG_42, ts->stCtrl.bReg0_42h[1]);	//LACTIVE
		rm31080_spi_byte_write(ts, RM31080_REG_43, ts->stCtrl.bReg0_43h[1]);	//LACTIVE

		rm31080_spi_byte_write(ts, 0x20, ts->stCtrl.bReg0_20h[1]);
		rm31080_spi_byte_write(ts, 0x21, ts->stCtrl.bReg0_21h[1]);
		rm31080_spi_byte_write(ts, 0x22, ts->stCtrl.bReg0_22h[1]);
		rm31080_spi_byte_write(ts, 0x23, ts->stCtrl.bReg0_23h[1]);
		rm31080_spi_byte_write(ts, 0x24, ts->stCtrl.bReg0_24h[1]);
		rm31080_spi_byte_write(ts, 0x25, ts->stCtrl.bReg0_25h[1]);
		rm31080_spi_byte_write(ts, 0x26, ts->stCtrl.bReg0_26h[1]);
		rm31080_spi_byte_write(ts, 0x27, ts->stCtrl.bReg0_27h[1]);
		rm31080_spi_byte_write(ts, 0x28, ts->stCtrl.bReg0_28h[1]);
		rm31080_spi_byte_write(ts, 0x29, ts->stCtrl.bReg0_29h[1]);
		rm31080_spi_byte_write(ts, 0x2A, ts->stCtrl.bReg0_2Ah[1]);
		rm31080_spi_byte_write(ts, 0x2B, ts->stCtrl.bReg0_2Bh[1]);
		rm31080_spi_byte_write(ts, 0x2C, ts->stCtrl.bReg0_2Ch[1]);
		rm31080_spi_byte_write(ts, 0x2D, ts->stCtrl.bReg0_2Dh[1]);
		rm31080_spi_byte_write(ts, 0x2E, ts->stCtrl.bReg0_2Eh[1]);
		rm31080_spi_byte_write(ts, 0x2F, ts->stCtrl.bReg0_2Fh[1]);
		rm31080_spi_byte_write(ts, 0x30, ts->stCtrl.bReg0_30h[1]);
		rm31080_spi_byte_write(ts, 0x31, ts->stCtrl.bReg0_31h[1]);
		rm31080_spi_byte_write(ts, 0x32, ts->stCtrl.bReg0_32h[1]);
		rm31080_spi_byte_write(ts, 0x33, ts->stCtrl.bReg0_33h[1]);
	}
}

static void rm31080_digital_filter_config(struct rm31080_ts *ts, u8 bRepeatTimes)
{
#if ENABLE_T007B1_SETTING
	if (ts->stCtrl.bICVersion != T007_VERSION_B)
#endif
    {
        rm31080_spi_byte_write(ts, 0x7F, 0x01);	// Switch to BANK1

        rm31080_spi_byte_write(ts, 0x09, ts->stCtrl.bReg1_09h[0]);
        rm31080_spi_byte_write(ts, 0x43, ts->stCtrl.bReg1_43h[0]);
        rm31080_spi_byte_write(ts, 0x48, ts->stCtrl.bReg1_48h[0]);
        rm31080_spi_byte_write(ts, 0x49, ts->stCtrl.bReg1_49h[0]);
        rm31080_spi_byte_write(ts, 0x4A, ts->stCtrl.bReg1_4Ah[0]);
        rm31080_spi_byte_write(ts, 0x4B, ts->stCtrl.bReg1_4Bh[0]);

        rm31080_spi_byte_write(ts, 0x7F, 0x00);     // Switch to 0x00

        rm_set_repeat_times(ts, bRepeatTimes);

    // -------- Set Driving / Sensing Control Mode --------
        if(!ts->stCtrl.bfAnalogFilter) {
            rm31080_spi_byte_write(ts, RM31080_REG_10, 0x10 |0x40); //ACC | DDSC
        }

    // -------- Set PGA/DAC --------
        rm31080_spi_byte_write(ts, 0x6B, 0x04); //EN_C0

    // -------- Scan Time Setting --------
        rm31080_spi_byte_write(ts, RM31080_REG_42, ts->stCtrl.bReg0_42h[ts->stCtrl.bfAnalogFilter]);  //LACTIVE
        rm31080_spi_byte_write(ts, RM31080_REG_43, ts->stCtrl.bReg0_43h[ts->stCtrl.bfAnalogFilter]);  //LACTIVE

        rm31080_spi_byte_write(ts, 0x20, ts->stCtrl.bReg0_20h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x21, ts->stCtrl.bReg0_21h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x22, ts->stCtrl.bReg0_22h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x23, ts->stCtrl.bReg0_23h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x24, ts->stCtrl.bReg0_24h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x25, ts->stCtrl.bReg0_25h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x26, ts->stCtrl.bReg0_26h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x27, ts->stCtrl.bReg0_27h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x28, ts->stCtrl.bReg0_28h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x29, ts->stCtrl.bReg0_29h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x2A, ts->stCtrl.bReg0_2Ah[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x2B, ts->stCtrl.bReg0_2Bh[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x2C, ts->stCtrl.bReg0_2Ch[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x2D, ts->stCtrl.bReg0_2Dh[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x2E, ts->stCtrl.bReg0_2Eh[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x2F, ts->stCtrl.bReg0_2Fh[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x30, ts->stCtrl.bReg0_30h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x31, ts->stCtrl.bReg0_31h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x32, ts->stCtrl.bReg0_32h[ts->stCtrl.bfAnalogFilter]);
        rm31080_spi_byte_write(ts, 0x33, ts->stCtrl.bReg0_33h[ts->stCtrl.bfAnalogFilter]);
    }
}
#endif

//=============================================================================
// Description:
//
// Input:
//      N/A
// Output:
//      N/A
//=============================================================================
static void rm31080_ctrl_get_parameter(struct rm31080_ts *ts, void *arg)
{
#define PARA_BASIC_LEN 4
#define PARA_HARDWARE_LEN 28
#define PARA_NOISE_LEN 32
#define PARA_ALGORITHM_LEN 128

	//Marty added
	u8 Temp;
	s8 bBlockInterval;
	u16 ii;
	u8 *pPara;

	pPara = (u8 *) arg;
	Temp = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + PARA_NOISE_LEN + PARA_ALGORITHM_LEN + 3];
	rm31080_set_autoscan(ts, Temp);

	ts->stCtrl.bADCNumber = pPara[PARA_BASIC_LEN + 6];
	ts->stCtrl.bChannelNumberX = pPara[PARA_BASIC_LEN];
	ts->stCtrl.bChannelNumberY = pPara[PARA_BASIC_LEN + 1];
	ts->stCtrl.bfNoiseDetector = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 13];
	ts->stCtrl.bChannelDetectorNum = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 19];	// Noise_Detector
	ts->stCtrl.bChannelDetectorDummy = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 20];	// Noise_Detector
	ts->stCtrl.u16DataLength = (ts->stCtrl.bChannelNumberX + 2 + ts->stCtrl.bADCNumber) *
		(ts->stCtrl.bChannelNumberY + ts->stCtrl.bfNoiseDetector * (ts->stCtrl.bChannelDetectorNum + ts->stCtrl.bChannelDetectorDummy));
	ts->stCtrl.bfNoiseMode = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 14];
	ts->stCtrl.bNoiseRepeatTimes = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 15];
	//Marty added
	ts->stCtrl.bNoiseThresholdMax = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 21];	// Noise_Detector
	ts->stCtrl.bNoiseThresholdMin = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 22];	// Noise_Detector
	ts->stCtrl.bNoiseThresholdLowMax = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 23];	// Noise_Detector
	ts->stCtrl.bNoiseThresholdLowMin = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 24];	// Noise_Detector
	ts->stCtrl.bNoiseDetectThd = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 16];	// Noise_Detector
	ts->stCtrl.bNoisePipelineBase = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 17];	// Noise_Detector
	ts->stCtrl.bNewNoiseRepeatTimes = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 18];	// Noise_Detector
	ts->stCtrl.bfMediumFilter = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 25];	// Noise_Detector
	ts->stCtrl.bMFBlockNumber = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 26];	// Noise_Detector
	ts->stCtrl.bRepeatTimes[0] = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 4];	// Noise_Detector
	ts->stCtrl.bRepeatTimes[1] = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 5];	// Noise_Detector
    ts->stCtrl.bIdleRepeatTimes[0] = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 6];
	ts->stCtrl.bIdleRepeatTimes[1] = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 7];	// Noise_Detector 	
	ts->stCtrl.bSenseNumber = pPara[PARA_BASIC_LEN + 11];	// Noise_Detector
	ts->stCtrl.bfADFC = pPara[PARA_BASIC_LEN + 4];	// Noise_Detector
	ts->stCtrl.bfTHMode = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 10];	// Noise_Detector
	ts->stCtrl.bfAnalogFilter = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 8];	// Noise_Detector
	ts->stCtrl.bYChannel[0] = pPara[280];	// Y chan start pin
	ts->stCtrl.bYChannel[1] = pPara[281];	// Y chan end pin
	ts->stCtrl.bXChannel[0] = pPara[282];	// X chan	ADC1 start pin
	ts->stCtrl.bXChannel[1] = pPara[283];	// X chan	ADC1 end pin
	ts->stCtrl.bXChannel[2] = pPara[284];	// X chan	ADC2 start pin
	ts->stCtrl.bXChannel[3] = pPara[285];	// X chan	ADC2 end pin
	ts->stCtrl.bfSuspendReset = pPara[PARA_BASIC_LEN + PARA_HARDWARE_LEN + 27];
	ts->stCtrl.bPressureResolution = pPara[105];
	ts->stCtrl.bMTTouchThreshold = pPara[192];
	ts->stCtrl.bTime2Idle = pPara[194];
	ts->stCtrl.bfPowerMode= pPara[195];
    ts->stCtrl.bfIdleMessage = pPara[207];
	//
	// Store dummy channel to skip it, data sequence:
	// Dummy[0](single end) | raw_data | dummy[1](single end) dummy[2](single end) | raw_data | dummy[3](single end)
	ts->stCtrl.bDummyChannel[0] = 0;
	if (ts->stCtrl.bfMediumFilter) {
		bBlockInterval = (ts->stCtrl.bNoiseThresholdMax - ts->stCtrl.bNoiseThresholdMin) / ts->stCtrl.bMFBlockNumber;
		//printk("Block Interval = %d\n", bBlockInterval);
		for (ii = 0; ii < ts->stCtrl.bMFBlockNumber; ii++) {
			g_bMFBlockMin[ii] = ts->stCtrl.bNoiseThresholdMin + bBlockInterval * ii;
			g_bMFBlockMax[ii] = g_bMFBlockMin[ii] + bBlockInterval;
			//printk("Block %d, Max = %d, Min = %d\n", ii, g_bMFBlockMax[ii], g_bMFBlockMin[ii]);
		}
	}
	if (ts->stCtrl.bADCNumber) {
		if (ts->stCtrl.bYChannel[0] > ts->stCtrl.bXChannel[0]) {
			if (ts->stCtrl.bXChannel[0] < ts->stCtrl.bXChannel[2]) {
				if (ts->stCtrl.bXChannel[0] < ts->stCtrl.bXChannel[1])
					Temp = ts->stCtrl.bXChannel[1] - ts->stCtrl.bXChannel[0];
				else
					Temp = ts->stCtrl.bXChannel[0] - ts->stCtrl.bXChannel[1];
				ts->stCtrl.bDummyChannel[1] = Temp + 1;
				ts->stCtrl.bDummyChannel[2] = Temp + 2;
			} else {
				if (ts->stCtrl.bXChannel[2] < ts->stCtrl.bXChannel[3])
					Temp = ts->stCtrl.bXChannel[3] - ts->stCtrl.bXChannel[2];
				else
					Temp = ts->stCtrl.bXChannel[2] - ts->stCtrl.bXChannel[3];
				ts->stCtrl.bDummyChannel[1] = Temp + 1;
				ts->stCtrl.bDummyChannel[2] = Temp + 2;
			}
			//#endif
		} else {
			if (ts->stCtrl.bXChannel[0] > ts->stCtrl.bXChannel[2]) {
				if (ts->stCtrl.bXChannel[0] < ts->stCtrl.bXChannel[1])
					Temp = ts->stCtrl.bXChannel[1] - ts->stCtrl.bXChannel[0];
				else
					Temp = ts->stCtrl.bXChannel[0] - ts->stCtrl.bXChannel[1];
				ts->stCtrl.bDummyChannel[1] = Temp + 1;
				ts->stCtrl.bDummyChannel[2] = Temp + 2;
			} else {
				if (ts->stCtrl.bXChannel[2] < ts->stCtrl.bXChannel[3])
					Temp = ts->stCtrl.bXChannel[3] - ts->stCtrl.bXChannel[2];
				else
					Temp = ts->stCtrl.bXChannel[2] - ts->stCtrl.bXChannel[3];
				ts->stCtrl.bDummyChannel[1] = Temp + 1;
				ts->stCtrl.bDummyChannel[2] = Temp + 2;
			}

		}
		ts->stCtrl.bDummyChannel[3] = ts->stCtrl.bChannelNumberX + 2;
	} else {
		ts->stCtrl.bDummyChannel[1] = ts->stCtrl.bChannelNumberX + 1;
	}
#if ENABLE_RESOLUTION_SWITCH
	ts->stCtrl.u16ResolutionX = ((u16) pPara[PARA_BASIC_LEN + 13]) << 8 | ((u16)pPara[PARA_BASIC_LEN + 12]);
	ts->stCtrl.u16ResolutionY = ((u16) pPara[PARA_BASIC_LEN + 15]) << 8 | ((u16)pPara[PARA_BASIC_LEN + 14]);
// 	printk("ts->stCtrl.u16ResolutionX=%d", ts->stCtrl.u16ResolutionX);
// 	printk("ts->stCtrl.u16ResolutionY=%d", ts->stCtrl.u16ResolutionY);
	if ((ts->stCtrl.u16ResolutionX == 0) || (ts->stCtrl.u16ResolutionY == 0)) {
		ts->stCtrl.u16ResolutionX = RM_INPUT_RESOLUTION_X;
		ts->stCtrl.u16ResolutionY = RM_INPUT_RESOLUTION_Y;
	}
#endif

#if ENABLE_FILTER_SWITCH
#if ENABLE_T007B1_SETTING
	if (ts->stCtrl.bICVersion != T007_VERSION_B)
#endif
    {
        ts->stCtrl.bReg1_09h[0] = pPara[327];	// Addr. 0327
        ts->stCtrl.bReg1_09h[1] = pPara[328];	// Addr. 0328
        ts->stCtrl.bReg1_43h[0] = pPara[337];	// Addr. 0337
        ts->stCtrl.bReg1_43h[1] = pPara[338];	// Addr. 0338
        ts->stCtrl.bReg1_48h[0] = pPara[339];	// Addr. 0339
        ts->stCtrl.bReg1_48h[1] = pPara[340];	// Addr. 0340
        ts->stCtrl.bReg1_49h[0] = pPara[341];	// Addr. 0341
        ts->stCtrl.bReg1_49h[1] = pPara[342];	// Addr. 0342
        ts->stCtrl.bReg1_4Ah[0] = pPara[343];	// Addr. 0343
        ts->stCtrl.bReg1_4Ah[1] = pPara[344];	// Addr. 0344
        ts->stCtrl.bReg1_4Bh[0] = pPara[345];	// Addr. 0345
        ts->stCtrl.bReg1_4Bh[1] = pPara[346];	// Addr. 0346

        ts->stCtrl.bReg0_40h[0] = pPara[258];
        ts->stCtrl.bReg0_40h[1] = pPara[259];
        ts->stCtrl.bReg0_41h[0] = pPara[260];
        ts->stCtrl.bReg0_41h[1] = pPara[261];
        ts->stCtrl.bReg0_42h[0] = pPara[262];	// Addr. 0262
        ts->stCtrl.bReg0_42h[1] = pPara[263];	// Addr. 0263
        ts->stCtrl.bReg0_43h[0] = pPara[264];	// Addr. 0264
        ts->stCtrl.bReg0_43h[1] = pPara[265];	// Addr. 0265

        // time chart
        ts->stCtrl.bReg0_20h[0] = pPara[213];	// Addr. 0213
        ts->stCtrl.bReg0_20h[1] = pPara[214];	// Addr. 0214
        ts->stCtrl.bReg0_21h[0] = pPara[215];	// Addr. 0215
        ts->stCtrl.bReg0_21h[1] = pPara[216];	// Addr. 0216
        ts->stCtrl.bReg0_22h[0] = pPara[217];	// Addr. 0217
        ts->stCtrl.bReg0_22h[1] = pPara[218];	// Addr. 0218
        ts->stCtrl.bReg0_23h[0] = pPara[219];	// Addr. 0219
        ts->stCtrl.bReg0_23h[1] = pPara[220];	// Addr. 0220
        ts->stCtrl.bReg0_24h[0] = pPara[221];	// Addr. 0221
        ts->stCtrl.bReg0_24h[1] = pPara[222];	// Addr. 0222
        ts->stCtrl.bReg0_25h[0] = pPara[223];	// Addr. 0223
        ts->stCtrl.bReg0_25h[1] = pPara[224];	// Addr. 0224
        ts->stCtrl.bReg0_26h[0] = pPara[225];	// Addr. 0225
        ts->stCtrl.bReg0_26h[1] = pPara[226];	// Addr. 0226
        ts->stCtrl.bReg0_27h[0] = pPara[227];	// Addr. 0227
        ts->stCtrl.bReg0_27h[1] = pPara[228];	// Addr. 0228
        ts->stCtrl.bReg0_28h[0] = pPara[229];	// Addr. 0229
        ts->stCtrl.bReg0_28h[1] = pPara[230];	// Addr. 0230
        ts->stCtrl.bReg0_29h[0] = pPara[231];	// Addr. 0231
        ts->stCtrl.bReg0_29h[1] = pPara[232];	// Addr. 0232
        ts->stCtrl.bReg0_2Ah[0] = pPara[233];	// Addr. 0233
        ts->stCtrl.bReg0_2Ah[1] = pPara[234];	// Addr. 0234
        ts->stCtrl.bReg0_2Bh[0] = pPara[235];	// Addr. 0235
        ts->stCtrl.bReg0_2Bh[1] = pPara[236];	// Addr. 0236
        ts->stCtrl.bReg0_2Ch[0] = pPara[237];	// Addr. 0237
        ts->stCtrl.bReg0_2Ch[1] = pPara[238];	// Addr. 0238
        ts->stCtrl.bReg0_2Dh[0] = pPara[239];	// Addr. 0239
        ts->stCtrl.bReg0_2Dh[1] = pPara[240];	// Addr. 0240
        ts->stCtrl.bReg0_2Eh[0] = pPara[241];	// Addr. 0241
        ts->stCtrl.bReg0_2Eh[1] = pPara[242];	// Addr. 0242
        ts->stCtrl.bReg0_2Fh[0] = pPara[243];	// Addr. 0243
        ts->stCtrl.bReg0_2Fh[1] = pPara[244];	// Addr. 0244
        ts->stCtrl.bReg0_30h[0] = pPara[245];	// Addr. 0245
        ts->stCtrl.bReg0_30h[1] = pPara[246];	// Addr. 0246
        ts->stCtrl.bReg0_31h[0] = pPara[247];	// Addr. 0247
        ts->stCtrl.bReg0_31h[1] = pPara[248];	// Addr. 0248
        ts->stCtrl.bReg0_32h[0] = pPara[249];	// Addr. 0249
        ts->stCtrl.bReg0_32h[1] = pPara[250];	// Addr. 0250
        ts->stCtrl.bReg0_33h[0] = pPara[251];	// Addr. 0251
        ts->stCtrl.bReg0_33h[1] = pPara[252];	// Addr. 0252
    }
#endif
}

//=============================================================================
/* MODULE_AUTHOR("xxxxxxxxxx <xxxxxxxx@rad-ic.com>"); */
/* MODULE_DESCRIPTION("Raydium touchscreen control functions"); */
/* MODULE_LICENSE("GPL"); */
