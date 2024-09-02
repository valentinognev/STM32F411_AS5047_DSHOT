/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */

uint8_t spiTxFinished = 1;
uint8_t spiRxFinished = 1;

/* USER CODE END 0 */

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI1 DMA Init */

  /* SPI1_RX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_3);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

  /* SPI1_TX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_2);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI1_Init 2 */
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);

  LL_DMA_ClearFlag_TC0(DMA2);
  LL_DMA_ClearFlag_TE0(DMA2);
  LL_DMA_ClearFlag_TC5(DMA2);
  LL_DMA_ClearFlag_TE5(DMA2);

  LL_SPI_EnableDMAReq_TX(SPI1);
  LL_SPI_EnableDMAReq_RX(SPI1);

  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_0);
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);
  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_2);
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_2);
  LL_SPI_Enable(SPI1);

  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, 1);
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, 1);
  /* USER CODE END SPI1_Init 2 */

}

/* USER CODE BEGIN 1 */
void SPI_TransmitReceive_DMA(uint16_t* transferData, uint16_t* receiveData, uint16_t size)
{
  for (uint16_t i=0; i<size; i++)
  {
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0, LL_SPI_DMA_GetRegAddr(SPI1), (uint32_t)(receiveData+i), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_0));
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_2, (uint32_t)(transferData+i), LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_2));

    HAL_GPIO_WritePin(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, GPIO_PIN_RESET); spiTxFinished = 0;spiRxFinished = 0;
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
  }
}

void SPI_Transfer_DMA(uint16_t* transferData, uint16_t size)
{
  for (uint16_t i=0; i<size; i++)
  {
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_2, (uint32_t)(transferData+i), LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_2));

    HAL_GPIO_WritePin(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, GPIO_PIN_RESET); spiTxFinished = 0;
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
  }
}

void SPI_Receive_DMA(uint16_t* receiveData, uint16_t size)
{
  for (uint16_t i=0; i<size; i++)
  {
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0, LL_SPI_DMA_GetRegAddr(SPI1), (uint32_t)(receiveData+i), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_0));

    HAL_GPIO_WritePin(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, GPIO_PIN_RESET); spiRxFinished = 0;
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
  }
}

void DMA2_Stream0_TransferComplete(void)
{
    // TX Done .. Do Something ...
  LL_DMA_ClearFlag_TC0(DMA2);
  if (spiTxFinished)
    HAL_GPIO_WritePin(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, GPIO_PIN_SET);
  spiRxFinished = 1;
}

void DMA2_Stream2_TransferComplete(void)
{
    // RX Done .. Do Something ...
  LL_DMA_ClearFlag_TC2(DMA2);
  if (spiRxFinished)
    HAL_GPIO_WritePin(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, GPIO_PIN_SET);
  spiTxFinished = 1;
}

/* USER CODE END 1 */
