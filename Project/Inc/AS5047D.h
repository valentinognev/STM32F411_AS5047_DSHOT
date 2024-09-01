/*
 * AS5047D.h
 *
 *  Created on: 24. avg. 2016
 *      Author: user
 */

#ifndef AS5047D_H_
#define AS5047D_H_

#include <stdint.h>
#include "main.h"

// AS5047D Register Addresses

/** volatile **/
#define AS5047D_NOP 0x0000
#define AS5047D_ERRFL 0x0001
#define AS5047D_PROG 0x0003
#define AS5047D_DIAAGC 0x3FFC
#define AS5047D_CORDICMAG 0x3FFD
#define AS5047D_ANGLEUNC 0x3FFE
#define AS5047D_ANGLECOM 0x3FFF

/** non-volatile **/
#define AS5047D_ZPOSM 0x0016
#define AS5047D_ZPOSL 0x0017
#define AS5047D_SETTINGS1 0x0018
#define AS5047D_SETTINGS2 0x0019

#define AS5047D_RD 0x4000    // bit 14 = "1" is Read + parity even
#define AS5047D_WR 0x3FFF    // bit 14 = "0" is Write

uint8_t AS5047D_Init(void);

uint8_t AS5047D_Write(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint16_t address, uint16_t data);
uint8_t AS5047D_Read(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint16_t address, uint16_t *data);
uint8_t AS5047D_ReadWrite(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint16_t address, uint16_t *data);

uint8_t AS5047D_SetZero(void);
uint8_t AS5047D_GetZero(uint16_t* zeroPos);
uint8_t AS5047D_Get_AGC_Value(uint16_t* DIAAGC);

uint8_t AS5047D_Get_CORDICMAG_Value(uint16_t* CORDICMAG);
uint8_t AS5047D_Get_ANGLEUNC_Value(uint16_t* ANGLEUNC);
uint8_t AS5047D_Get_ANGLECOM_Value(uint16_t* ANGLECOM);

uint8_t AS5047D_Get_True_Angle_Value(float* angle);

#define AS5047D_Check_MAG_TooLow(DIAAGC)      ((DIAAGC >> 11) & 0x0001)
#define AS5047D_Check_MAG_TooHigh(DIAAGC)     ((DIAAGC >> 10) & 0x0001)
#define AS5047D_Check_CORDIC_Overflow(DIAAGC) ((DIAAGC >> 9)  & 0x0001)
#define AS5047D_Check_LF_finished(DIAAGC)     ((DIAAGC >> 8)  & 0x0001)

#endif /* AS5047D_H_ */
