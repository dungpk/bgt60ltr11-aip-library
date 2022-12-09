/*******************************************************************************
 *
 * Copyright (c) 2020
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description:
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: dungpk $
 * Revision:         $Revision: 1.1 $
 * Last Changed:     $Date: 10/07/20 $
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include "stm32f401re.h"
#include <stm32f401re_rcc.h>
#include "bgt60ltr11.h"
#include "uartcmd.h"
#include "stm32f401re_usart.h"
#include "string.h"
#include "timer.h"
#include "Ucglib.h"
#include "eventman.h"
#include "serial.h"
#include "typedefs.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define CMD_ID_FREQUENCY						0x01 /*! @brief PLL frequency word */
#define CMD_ID_PULSE_WIDTH						0x02 /*! @brief pulse length */
#define CMD_ID_RPT								0x03 /*! @brief Duty cycle repetition rate */
#define CMD_ID_TX_POWER							0x04 /*! @brief power amplifier */
#define CMD_ID_PGA_GAIN							0x05 /*! @brief Baseband PGA gain setting */
#define CMD_ID_HOLD_TIME						0x06 /*! @brief hold time setting */
#define CMD_ID_SENSITIVITY						0x07 /*! @brief hold time setting */

/*! @brief sensitivity config */



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
u16_t g_wFrequencyValue[4] = {FREQUENCY_61_1HZ_VALUE,
						   	   FREQUENCY_61_2HZ_VALUE,
							   FREQUENCY_61_3HZ_VALUE,
							   FREQUENCY_61_4HZ_VALUE,
};

u16_t g_wPulseWidthValue[4] = {PULSE_WIDTH_5MICRO_S,
								PULSE_WIDTH_10MICRO_S,
								PULSE_WIDTH_3MICRO_S,
								PULSE_WIDTH_4MICRO_S,
};

u16_t g_wRPTValue[4] = { 		DUTY_CYCLE_REPETITION_250MICRO_S,
								DUTY_CYCLE_REPETITION_500MICRO_S,
								DUTY_CYCLE_REPETITION_1000MICRO_S,
								DUTY_CYCLE_REPETITION_2000MICRO_S,
};

u16_t g_wPowerTxValue[8] = 	{	DBM_NEGATIVE_34,
								DBM_NEGATIVE_31_5,
								DBM_NEGATIVE_25,
								DBM_NEGATIVE_18,
								DBM_NEGATIVE_11,
								DBM_NEGATIVE_5,
								DBM_0,
								DBM_4_5,

};

u16_t g_wPGAgainValue[9] = {
								PGA_10_DB,
								PGA_15_DB,
								PGA_20_DB,
								PGA_25_DB,
								PGA_30_DB,
								PGA_35_DB,
								PGA_40_DB,
								PGA_45_DB,
								PGA_50_DB,
};

u16_t g_wHoldTime[5] = {
								HOLD_TIME_1S,
								HOLD_TIME_2S,
								HOLD_TIME_3S,
								HOLD_TIME_5S,
								HOLD_TIME_10S,
};

u16_t g_wSensitivity[14] = {
								SENSITIVITY_1,
								SENSITIVITY_2,
								SENSITIVITY_3,
								SENSITIVITY_4,
								SENSITIVITY_5,
								SENSITIVITY_6,
								SENSITIVITY_7,
								SENSITIVITY_8,
								SENSITIVITY_9,
								SENSITIVITY_10,
								SENSITIVITY_11,
								SENSITIVITY_12,
								SENSITIVITY_13,
								SENSITIVITY_14,
};

extern u16_t g_wRegister_2_Value;
extern u16_t g_wRegister_5_Value;
extern u16_t g_wRegister_7_Value;
extern u16_t g_wRegister_9_Value;
extern u16_t g_wRegister_10_Value;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
ucg_t ucg;
extern uint8_t g_byRecive_Data[100];

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

void_t displayReveciveDataFrame(void_t);

//Process command config parameter
void_t processUartReceiveCommandConfigFrequency(u8_t byValue);
void_t processUartReceiveCommandConfigPulseWidth(u8_t byValue);
void_t processUartReceiveCommandConfigRPT(u8_t byValue);
void_t processUartReceiveCommandConfigTxPower(u8_t byValue);
void_t processUartReceiveCommandConfigPGAGain(u8_t byValue);
void_t processUartReceiveCommandConfigHoldTime(u8_t byValue);
void_t processUartReceiveCommandConfigSensitivity(u8_t byValue);

//Process command read parameter
void_t processUartReceiveCommandReadFrequency();
void_t processUartReceiveCommandReadPulseWidth();
void_t processUartReceiveCommandReadRPT();
void_t processUartReceiveCommandReadTxPower();
void_t processUartReceiveCommandReadPGAGain();
void_t processUartReceiveCommandReadHoldTime();
void_t processUartReceiveCommandReadSensitivity();

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void_t bgt60ltr11SendData(u8_t byParam,u8_t byValue);
void_t bgt60ltr11ReadData(u8_t byParam);
void_t delayMs(u32_t dwMs);
void_t procConfigCmd( u8_t byParameter,u8_t byValue);
void_t proReadParameterCmd( u8_t byParameter);
static void_t lcdDefaultInit(void);

int main(void)
{
	SystemCoreClockUpdate();
	bgt60ltr11Init();
	Serial_Init();
	TimerInit();
	lcdDefaultInit();
	EventSerial_Init();
	EventSerial_SetEventBGT60LTR11Callback(bgt60ltr11SendData);
	EventSerial_GetEventBGT60LTR11Callback(bgt60ltr11ReadData);
	while(1)
	{
		processTimerScheduler();
		processSerialReceiver();
	}
}

static void_t lcdDefaultInit(void)
{
	Ucglib4WireSWSPI_begin(&ucg,UCG_FONT_MODE_SOLID);
	ucg_ClearScreen(&ucg);
	ucg_SetFont(&ucg,ucg_font_ncenR12_hr);
	ucg_SetColor(&ucg, 0, 255, 255, 255);
	ucg_SetColor(&ucg, 1, 0, 0, 0);
	ucg_SetRotate180(&ucg);
}

/**
 * @func   delayMs
 * @brief  delay function
 * @param  None
 * @retval None
 */
void_t delayMs(u32_t dwMs)
{
	uint32_t i,j;
	for (i = 0 ; i < dwMs ; i ++)
	{
		for (j = 0; j<5000; j++){;}
	}
}

/**
 * @func   procConfigCmd
 * @brief  Process command config parameter (hold time,RPT,pulse width,...)
 * @param  None
 * @retval None
 */
void_t
bgt60ltr11SendData( u8_t byParameter,u8_t byValue

) {
    switch (byParameter) {
		case CMD_ID_FREQUENCY:
			processUartReceiveCommandConfigFrequency(byValue);
			break;
		case CMD_ID_PULSE_WIDTH:
			processUartReceiveCommandConfigPulseWidth(byValue);
			break;
		case CMD_ID_RPT:
			processUartReceiveCommandConfigRPT(byValue);
			break;
		case CMD_ID_TX_POWER:
			processUartReceiveCommandConfigTxPower(byValue);
			break;
		case CMD_ID_PGA_GAIN:
			processUartReceiveCommandConfigPGAGain(byValue);
			break;
		case CMD_ID_HOLD_TIME:
			processUartReceiveCommandConfigHoldTime(byValue);
			break;
		case CMD_ID_SENSITIVITY:
			processUartReceiveCommandConfigSensitivity(byValue);
			break;

        default:
            break;
    }
    displayReveciveDataFrame();
}

/**
 * @func   bgt60ltr11ReadData
 * @brief  Process command read parameter (hold time,RPT,pulse width,...)
 * @param  byParam need read
 * @retval None
 */
void_t bgt60ltr11ReadData(u8_t byParameter)
{
    switch (byParameter) {
		case CMD_ID_FREQUENCY:
			processUartReceiveCommandReadFrequency();
			break;
		case CMD_ID_PULSE_WIDTH:
			processUartReceiveCommandReadPulseWidth();
			break;
		case CMD_ID_RPT:
			processUartReceiveCommandReadRPT();
			break;
		case CMD_ID_TX_POWER:
			processUartReceiveCommandReadTxPower();
			break;
		case CMD_ID_PGA_GAIN:
			processUartReceiveCommandReadPGAGain();
			break;
		case CMD_ID_HOLD_TIME:
			processUartReceiveCommandReadHoldTime();
			break;
		case CMD_ID_SENSITIVITY:
			processUartReceiveCommandReadSensitivity();
			break;

        default:
            break;
    }
    displayReveciveDataFrame();
}
/**
 * @func   processUartReceiveCommandConfigFrequency
 * @brief  processUartReceiveCommandConfigFrequency
 * @param  parameter and value need change
 * @retval None
 */
void_t processUartReceiveCommandConfigFrequency(u8_t byValue)
{
	g_wRegister_5_Value = ((g_wRegister_5_Value & 0xF000) | g_wFrequencyValue[byValue -1]);
	writeDataRegister(SPI2,PLL_CONFIG2_REG5_REG,g_wRegister_5_Value);
}

/**
 * @func   processUartReceiveCommandConfigPulseWidth
 * @brief  processUartReceiveCommandConfigPulseWidth
 * @param  parameter and value need change
 * @retval None
 */
void_t processUartReceiveCommandConfigPulseWidth(u8_t byValue)
{
	g_wRegister_7_Value = ((g_wRegister_7_Value & 0xFCFF) | (g_wPulseWidthValue[byValue -1] << 8));
	writeDataRegister(SPI2,DC_TMG_PD_MPA_REG7_REG,g_wRegister_7_Value);
}

/**
 * @func   processUartReceiveCommandConfigRPT
 * @brief  processUartReceiveCommandConfigRPT
 * @param  parameter and value need change
 * @retval None
 */
void_t processUartReceiveCommandConfigRPT(u8_t byValue)
{
	g_wRegister_7_Value = ((g_wRegister_7_Value & 0xF3FF) | (g_wRPTValue[byValue -1] << 10));
	writeDataRegister(SPI2,DC_TMG_PD_MPA_REG7_REG,g_wRegister_7_Value);
}

/**
 * @func   processUartReceiveCommandConfigRPT
 * @brief  processUartReceiveCommandConfigRPT
 * @param  parameter and value need change
 * @retval None
 */
void_t processUartReceiveCommandConfigTxPower(u8_t byValue)
{
	g_wRegister_7_Value = ((g_wRegister_7_Value & 0xFFF8) | (g_wPowerTxValue[byValue -1]));
	writeDataRegister(SPI2,DC_TMG_PD_MPA_REG7_REG,g_wRegister_7_Value);
}

/**
 * @func   processUartReceiveCommandConfigPGAGain
 * @brief  processUartReceiveCommandConfigPGAGain
 * @param  parameter and value need change
 * @retval None
 */
void_t processUartReceiveCommandConfigPGAGain(u8_t byValue)
{
	g_wRegister_9_Value = ((g_wRegister_9_Value & 0xFFF0) | (g_wPGAgainValue[byValue -1]));
	writeDataRegister(SPI2,BB_REG9_REG,g_wRegister_9_Value);
}

/**
 * @func   processUartReceiveCommandConfigHoldTime
 * @brief  processUartReceiveCommandConfigHoldTime
 * @param  parameter and value need change
 * @retval None
 */
void_t processUartReceiveCommandConfigHoldTime(u8_t byValue)
{
	g_wRegister_10_Value = g_wHoldTime[byValue -1];
	writeDataRegister(SPI2,HT_REG10_REG,g_wRegister_10_Value);
}

/**
 * @func   processUartReceiveCommandSensitivity
 * @brief  processUartReceiveCommandSensitivity
 * @param  parameter and value need change
 * @retval None
 */
void_t processUartReceiveCommandConfigSensitivity(u8_t byValue)
{
	g_wRegister_2_Value =((g_wRegister_2_Value & 0xE000) | (g_wSensitivity[byValue -1]));
	writeDataRegister(SPI2,THOLD_REG2_REG,g_wRegister_2_Value);
}

/**
 * @func   processUartReceiveCommandReadFrequency
 * @brief  processUartReceiveCommandReadFrequency
 * @param  None
 * @retval None
 */
void_t processUartReceiveCommandReadFrequency(void_t)
{
	ucg_ClearScreen(&ucg);
	readDataRegister(SPI2,PLL_CONFIG2_REG5_REG);
	u16_t wData = g_byRecive_Data[1] * 256 + g_byRecive_Data[2];
	switch(wData){

		case FREQUENCY_61_1HZ_VALUE:
			ucg_DrawString(&ucg, 0, 32, 0, "Frequency: 61.1");
			break;
		case FREQUENCY_61_2HZ_VALUE:
			ucg_DrawString(&ucg, 0, 32, 0, "Frequency: 61.2");
			break;
		case FREQUENCY_61_3HZ_VALUE:
			ucg_DrawString(&ucg, 0, 32, 0, "Frequency: 61.3");
			break;
		case FREQUENCY_61_4HZ_VALUE:
			ucg_DrawString(&ucg, 0, 32, 0, "Frequency: 61.4");
			break;

		default:
			ucg_DrawString(&ucg, 0, 32, 0, "Error");
			ucg_DrawString(&ucg, 0, 52, 0, "FrequenCy");
			break;
	}
}

/**
 * @func   processUartReceiveCommandReadFrequency
 * @brief  processUartReceiveCommandReadFrequency
 * @param  None
 * @retval None
 */
void_t processUartReceiveCommandReadPulseWidth(void_t)
{
	ucg_ClearScreen(&ucg);
	readDataRegister(SPI2,DC_TMG_PD_MPA_REG7_REG);
	u8_t byDataPulseWidth = g_byRecive_Data[1] & 0x03;
	switch(byDataPulseWidth){

		case PULSE_WIDTH_5MICRO_S:
			ucg_DrawString(&ucg, 0, 32, 0, "PulseWidth");
			ucg_DrawString(&ucg, 0, 52, 0, "5microS");
			break;
		case PULSE_WIDTH_10MICRO_S:
			ucg_DrawString(&ucg, 0, 32, 0, "PulseWidth");
			ucg_DrawString(&ucg, 0, 52, 0, "10microS");
			break;
		case PULSE_WIDTH_3MICRO_S:
			ucg_DrawString(&ucg, 0, 32, 0, "PulseWidth");
			ucg_DrawString(&ucg, 0, 52, 0, "3microS");
			break;
		case PULSE_WIDTH_4MICRO_S:
			ucg_DrawString(&ucg, 0, 32, 0, "PulseWidth");
			ucg_DrawString(&ucg, 0, 52, 0, "4microS");
			break;

		default:
			ucg_DrawString(&ucg, 0, 32, 0, "Error");
			ucg_DrawString(&ucg, 0, 52, 0, "Pulse Width");
			break;
	}
}

/**
 * @func   processUartReceiveCommandReadRPT
 * @brief  processUartReceiveCommandReadRPT
 * @param  None
 * @retval None
 */
void_t processUartReceiveCommandReadRPT(void_t)
{
	ucg_ClearScreen(&ucg);
	readDataRegister(SPI2,DC_TMG_PD_MPA_REG7_REG);
	u8_t byDataRPT = (g_byRecive_Data[1] >> 2)  & 0x03;
	switch(byDataRPT){

		case DUTY_CYCLE_REPETITION_250MICRO_S:
			ucg_DrawString(&ucg, 0, 32, 0, "RPT:");
			ucg_DrawString(&ucg, 0, 52, 0, "250 microS");
			break;
		case DUTY_CYCLE_REPETITION_500MICRO_S:
			ucg_DrawString(&ucg, 0, 32, 0, "RPT:");
			ucg_DrawString(&ucg, 0, 52, 0, "500 microS");
			break;
		case DUTY_CYCLE_REPETITION_1000MICRO_S:
			ucg_DrawString(&ucg, 0, 32, 0, "RPT:");
			ucg_DrawString(&ucg, 0, 52, 0, "1S");
			break;
		case DUTY_CYCLE_REPETITION_2000MICRO_S:
			ucg_DrawString(&ucg, 0, 32, 0, "RPT:");
			ucg_DrawString(&ucg, 0, 52, 0, "2S");
			break;

		default:
			ucg_DrawString(&ucg, 0, 32, 0, "Error");
			ucg_DrawString(&ucg, 0, 52, 0, "RPT");
			break;
	}
}

/**
 * @func   processUartReceiveCommandReadTxPower
 * @brief  processUartReceiveCommandReadTxPower
 * @param  None
 * @retval None
 */
void_t processUartReceiveCommandReadTxPower(void_t)
{
	ucg_ClearScreen(&ucg);
	readDataRegister(SPI2,DC_TMG_PD_MPA_REG7_REG);
	u8_t byDataTxPower = (g_byRecive_Data[2] & 0x07);
	switch(byDataTxPower){

		case DBM_NEGATIVE_34:
			ucg_DrawString(&ucg, 0, 32, 0, "Tx Power:");
			ucg_DrawString(&ucg, 0, 52, 0, "-34 dBm");
			break;
		case DBM_NEGATIVE_31_5:
			ucg_DrawString(&ucg, 0, 32, 0, "Tx Power:");
			ucg_DrawString(&ucg, 0, 52, 0, "-31.5 dBm");
			break;
		case DBM_NEGATIVE_25:
			ucg_DrawString(&ucg, 0, 32, 0, "Tx Power:");
			ucg_DrawString(&ucg, 0, 52, 0, "-25 dBm");
			break;
		case DBM_NEGATIVE_18:
			ucg_DrawString(&ucg, 0, 32, 0, "Tx Power:");
			ucg_DrawString(&ucg, 0, 52, 0, "-18 dBm");
			break;
		case DBM_NEGATIVE_11:
			ucg_DrawString(&ucg, 0, 32, 0, "Tx Power:");
			ucg_DrawString(&ucg, 0, 52, 0, " -11 dBm");
			break;
		case DBM_NEGATIVE_5:
			ucg_DrawString(&ucg, 0, 32, 0, "Tx Power:");
			ucg_DrawString(&ucg, 0, 52, 0, "-5 dBm");
			break;
		case DBM_0:
			ucg_DrawString(&ucg, 0, 32, 0, "Tx Power:");
			ucg_DrawString(&ucg, 0, 52, 0, "0 dBm");
			break;
		case DBM_4_5:
			ucg_DrawString(&ucg, 0, 32, 0, "Tx Power:");
			ucg_DrawString(&ucg, 0, 52, 0, "4.5 dBm");
			break;

		default:
			ucg_DrawString(&ucg, 0, 32, 0, "Tx Power:");
			ucg_DrawString(&ucg, 0, 52, 0, "Error");
			break;
	}
}

/**
 * @func   processUartReceiveCommandReadPGAGain
 * @brief  processUartReceiveCommandReadPGAGain
 * @param  None
 * @retval None
 */
void_t processUartReceiveCommandReadPGAGain(void_t)
{
	ucg_ClearScreen(&ucg);
	readDataRegister(SPI2,BB_REG9_REG);
	u8_t byDataPGAGain = g_byRecive_Data[2] & 0x0F;
	switch(byDataPGAGain){

		case PGA_10_DB:
			ucg_DrawString(&ucg, 0, 32, 0, "PGAGain:");
			ucg_DrawString(&ucg, 0, 52, 0, "10 dB");
			break;
		case PGA_15_DB:
			ucg_DrawString(&ucg, 0, 32, 0, "PGAGain:");
			ucg_DrawString(&ucg, 0, 52, 0, "15 dB");
			break;
		case PGA_20_DB:
			ucg_DrawString(&ucg, 0, 32, 0, "PGAGain:");
			ucg_DrawString(&ucg, 0, 52, 0, "20 dB");
			break;
		case PGA_25_DB:
			ucg_DrawString(&ucg, 0, 32, 0, "PGAGain:");
			ucg_DrawString(&ucg, 0, 52, 0, "25 dB");
			break;
		case PGA_30_DB:
			ucg_DrawString(&ucg, 0, 32, 0, "PGAGain:");
			ucg_DrawString(&ucg, 0, 52, 0, "30 dB");
			break;
		case PGA_35_DB:
			ucg_DrawString(&ucg, 0, 32, 0, "PGAGain:");
			ucg_DrawString(&ucg, 0, 52, 0, "35 dB");
			break;
		case PGA_40_DB:
			ucg_DrawString(&ucg, 0, 32, 0, "PGAGain:");
			ucg_DrawString(&ucg, 0, 52, 0, "40 dB");
			break;
		case PGA_45_DB:
			ucg_DrawString(&ucg, 0, 32, 0, "PGAGain:");
			ucg_DrawString(&ucg, 0, 52, 0, "45 dB");
			break;
		case PGA_50_DB:
			ucg_DrawString(&ucg, 0, 32, 0, "PGAGain:");
			ucg_DrawString(&ucg, 0, 52, 0, "50 dB");
			break;

		default:
			ucg_DrawString(&ucg, 0, 32, 0, "PGAGain:");
			ucg_DrawString(&ucg, 0, 52, 0, "Error");
			break;
	}
}

/**
 * @func   processUartReceiveCommandReadHoldTime
 * @brief  processUartReceiveCommandReadHoldTime
 * @param  None
 * @retval None
 */
void_t processUartReceiveCommandReadHoldTime(void_t)
{
	ucg_ClearScreen(&ucg);
	readDataRegister(SPI2,HT_REG10_REG);
	u16_t wDataHoldTime = g_byRecive_Data[1]*256 + g_byRecive_Data[2];
	switch(wDataHoldTime)
	{
		case HOLD_TIME_1S:
			ucg_DrawString(&ucg, 0, 32, 0, "Hold Time:");
			ucg_DrawString(&ucg, 0, 52, 0, "1S");
			break;
		case HOLD_TIME_2S:
			ucg_DrawString(&ucg, 0, 32, 0, "Hold Time:");
			ucg_DrawString(&ucg, 0, 52, 0, "2S");
			break;
		case HOLD_TIME_3S:
			ucg_DrawString(&ucg, 0, 32, 0, "Hold Time:");
			ucg_DrawString(&ucg, 0, 52, 0, "3S");
			break;
		case HOLD_TIME_5S:
			ucg_DrawString(&ucg, 0, 32, 0, "Hold Time:");
			ucg_DrawString(&ucg, 0, 52, 0, "5S");
			break;
		case HOLD_TIME_10S:
			ucg_DrawString(&ucg, 0, 32, 0, "Hold Time:");
			ucg_DrawString(&ucg, 0, 52, 0, "10S");
			break;
		default:
			ucg_DrawString(&ucg, 0, 32, 0, "Hold Time:");
			ucg_DrawString(&ucg, 0, 52, 0, "Error");
			break;
	}
}
/**
 * @func   processUartReceiveCommandReadSensitivity
 * @brief  processUartReceiveCommandReadSensitivity
 * @param  None
 * @retval None
 */
void_t processUartReceiveCommandReadSensitivity(void_t)
{
	ucg_ClearScreen(&ucg);
	readDataRegister(SPI2,THOLD_REG2_REG);
	u16_t wDataSensitivity = (g_byRecive_Data[1] & 0x1F)*256 + g_byRecive_Data[2];
	switch(wDataSensitivity)
	{
		case SENSITIVITY_1:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 1");
			break;
		case SENSITIVITY_2:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 2");
			break;
		case SENSITIVITY_3:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 3");
			break;
		case SENSITIVITY_4:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 4");
			break;
		case SENSITIVITY_5:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 5");
			break;
		case SENSITIVITY_6:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 6");
			break;
		case SENSITIVITY_7:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 7");
			break;
		case SENSITIVITY_8:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 8");
			break;
		case SENSITIVITY_9:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 9");
			break;
		case SENSITIVITY_10:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 10");
			break;
		case SENSITIVITY_11:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 11");
			break;
		case SENSITIVITY_12:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 12");
			break;
		case SENSITIVITY_13:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 13");
			break;
		case SENSITIVITY_14:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity: 14");
			break;
		default:
			ucg_DrawString(&ucg, 0, 32, 0, "Sensitivity:");
			ucg_DrawString(&ucg, 0, 32, 0, "Error:");
			break;
	}
}

/**
 * @func   displayReveciveDataFrame
 * @brief  display Revecive Data Frame after read or write register
 * @param  None
 * @retval None
 */
void_t displayReveciveDataFrame(void_t)
{
	for(u8_t i = 0; i<3 ; i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		USART_SendData(USART2, g_byRecive_Data[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {}
	}
}
