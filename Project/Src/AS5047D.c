/*
 * AS5047D.c
 *
 *  Created on: 24. avg. 2016
 *      Author: user
 */

#include "AS5047D.h"
#include <stdint.h>
#include "stm32f4xx_it.h"

extern uint8_t spiTxFinished;
extern uint8_t spiRxFinished;

uint16_t parity(uint16_t x)
{
	uint16_t parity = 0;

	while(x != 0)
	{
		parity ^= x;
		x >>= 1;
	}

	return (parity & 0x1);
}

uint8_t AS5047D_ReadWrite(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint16_t address, uint16_t *data)
{
	if (parity(address | AS5047D_RD) == 1) address = address | 0x8000; // set parity bit
	address = address | AS5047D_RD; // it's a read command


	SPI_TransmitReceive_DMA(&address, data, 1);
	while (!spiTxFinished || !spiRxFinished);

	uint8_t parityErr = 0, EFerr = 0;
	if ((*data & 0x8000) >> 15 != parity(*data & 0x7FFF))
		parityErr = 1;
	if ((*data & 0x4000) != 0)
		EFerr = 2;

	return parityErr | EFerr;
}

uint8_t AS5047D_Write(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint16_t address, uint16_t data)
{
	if (parity(address & AS5047D_WR) == 1) address = address | 0x8000; // set parity bit
	//address = address & (WR | 0x8000);  // its  a write command and don't change the parity bit (0x8000)

	uint16_t resData1 = 0, resData2 = 0;
	SPI_TransmitReceive_DMA(&address, &resData1, 1);
	while (!spiTxFinished || !spiRxFinished);
	//HAL_Delay(1);

	if (parity(data & AS5047D_WR) == 1) data = data | 0x8000; // set parity bit
	//data = data & (WR | 0x8000); // its a write command and don't change the parity bit (0x8000)

	SPI_TransmitReceive_DMA(&data, &resData2, 1);
	while (!spiTxFinished || !spiRxFinished);

	uint8_t parityErr = 0, EFerr = 0, dataErr = 0;
	if ((resData2 & 0x8000) >> 15 != parity(resData2 & 0x7FFF))
		parityErr = 1;
	if ((resData2 & 0x4000) != 0)
		EFerr = 2;
	if ((resData2 & 0x3FFF) != data)
		dataErr = 4;

	return parityErr | EFerr | dataErr;
}

uint8_t AS5047D_Read(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint16_t address, uint16_t *data)
{
	if (parity(address | AS5047D_RD) == 1) address = address | 0x8000; // set parity bit
	address = address | AS5047D_RD; // it's a read command

	uint16_t resData = 0;
	SPI_TransmitReceive_DMA(&address, &resData, 1);
	while (!spiTxFinished || !spiRxFinished)
	{
		//DelayUS(10);
	}

	uint16_t nop = 0;
	SPI_TransmitReceive_DMA(&nop, data, 1);
	while (!spiTxFinished || !spiRxFinished);
	uint8_t parityErr = 0, EFerr = 0;
	if ((*data & 0x8000) >> 15 != parity(*data & 0x7FFF))
		parityErr = 1;
	if ((*data & 0x4000) != 0)
		EFerr = 2;

	*data = *data & 0x3FFF;  // filter bits outside data, strip bit 14..15
	return parityErr|EFerr;
}



uint8_t AS5047D_SetZero(void)
{
	/** Check diagnostics reg **/
	uint16_t DIAAGC = 0;
	uint8_t errorFlag = AS5047D_Read(AS5047_CS_GPIO_Port, AS5047_CS_Pin, AS5047D_DIAAGC, &DIAAGC);
	if (errorFlag != 0)		return errorFlag;
	//if((AS5047D_Check_MAG_TooLow(DIAAGC)) || (AS5047D_Check_MAG_TooHigh(DIAAGC)) || (AS5047D_Check_CORDIC_Overflow(DIAAGC)) || !(AS5047D_Check_LF_finished(DIAAGC)))
	//{
		//Error_Handler();
	//}

	/** Get uncompensated angle reg value **/
	uint16_t ANGLEUNC = 0;
	errorFlag = AS5047D_Read(AS5047_CS_GPIO_Port, AS5047_CS_Pin, AS5047D_ANGLEUNC, &ANGLEUNC);
	if (errorFlag != 0)		return errorFlag;

	/** Write to zero pos regs **/
	errorFlag = AS5047D_Write(AS5047_CS_GPIO_Port, AS5047_CS_Pin , AS5047D_ZPOSM, (ANGLEUNC >> 6) & 0x00FF);
	if (errorFlag != 0)		return errorFlag;
	errorFlag = AS5047D_Write(AS5047_CS_GPIO_Port, AS5047_CS_Pin , AS5047D_ZPOSL, ANGLEUNC & 0x003F);
	if (errorFlag != 0)		return errorFlag;
}

uint8_t AS5047D_GetZero(uint16_t* zeroPos)
{
	uint16_t ZPOSM = 0;
	uint16_t ZPOSL = 0;

	uint8_t errorFlag = AS5047D_Read(AS5047_CS_GPIO_Port, AS5047_CS_Pin, AS5047D_ZPOSM, &ZPOSM);
	if (errorFlag != 0)		return errorFlag;

	errorFlag = AS5047D_Read(AS5047_CS_GPIO_Port, AS5047_CS_Pin, AS5047D_ZPOSL, &ZPOSL);
	if (errorFlag != 0)		return errorFlag;

	*zeroPos =  (((ZPOSM << 6) & 0x3FC0) | (ZPOSL & 0x003F));
	return errorFlag;
}

uint8_t AS5047D_Get_AGC_Value(uint16_t* DIAAGC)
{
	/** Read diagnostics reg **/
	uint8_t errorFlag = AS5047D_Read(AS5047_CS_GPIO_Port, AS5047_CS_Pin, AS5047D_DIAAGC, DIAAGC);
	if (errorFlag == 0)
		*DIAAGC = (uint8_t)((*DIAAGC >> 8) & 0x00FF);
	return errorFlag;
}

uint8_t AS5047D_Init(void)
{
	/* Initiaize AS5047D */
	uint8_t errorFlag = AS5047D_Write(AS5047_CS_GPIO_Port, AS5047_CS_Pin , AS5047D_SETTINGS1, 0b00000101);
	if (errorFlag != 0)		return errorFlag;

	errorFlag = AS5047D_Write(AS5047_CS_GPIO_Port, AS5047_CS_Pin , AS5047D_SETTINGS2, 0b00000000);
	if (errorFlag != 0)		return errorFlag;
	return 0;
}

uint8_t AS5047D_Get_CORDICMAG_Value(uint16_t* CORDICMAG)
{
	return AS5047D_Read(AS5047_CS_GPIO_Port, AS5047_CS_Pin, AS5047D_CORDICMAG, CORDICMAG);
}

uint8_t AS5047D_Get_ANGLEUNC_Value(uint16_t* ANGLEUNC)
{
	return AS5047D_Read(AS5047_CS_GPIO_Port, AS5047_CS_Pin, AS5047D_ANGLEUNC, ANGLEUNC);
}

uint8_t AS5047D_Get_ANGLECOM_Value(uint16_t* ANGLECOM)
{
	return AS5047D_Read(AS5047_CS_GPIO_Port, AS5047_CS_Pin, AS5047D_ANGLECOM, ANGLECOM);
}

uint8_t AS5047D_Get_True_Angle_Value(float* angle)
{
	uint16_t uangle = 0;
	uint8_t errorFlag = AS5047D_Get_ANGLEUNC_Value(&uangle);
	//if (errorFlag == 0)
	*angle = ((float)uangle * 360.0f / 16383.0f);
	return errorFlag;
	//return((float)AS5047D_Get_ANGLECOM_Value() * 360.0f / 16383.0f);
}
