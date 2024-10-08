/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"

#include "stm32f4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SPI_TransmitReceive_DMA(uint16_t* transferData, uint16_t* receiveData, uint16_t size);
void SPI_Transfer_DMA(uint16_t* transferData, uint16_t size);
void SPI_Receive_DMA(uint16_t* receiveData, uint16_t size);

void DMA2_Stream0_TransferComplete(void);
void DMA2_Stream2_TransferComplete(void);
void DelayUS(uint32_t us) ;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define led_Pin GPIO_PIN_13
#define led_GPIO_Port GPIOC
#define btn_Pin GPIO_PIN_0
#define btn_GPIO_Port GPIOA
#define PHASE_Pin GPIO_PIN_1
#define PHASE_GPIO_Port GPIOA
#define AS5047_CS_Pin GPIO_PIN_4
#define AS5047_CS_GPIO_Port GPIOA
#define AS5047_SCK_Pin GPIO_PIN_5
#define AS5047_SCK_GPIO_Port GPIOA
#define AS5047_MISO_Pin GPIO_PIN_6
#define AS5047_MISO_GPIO_Port GPIOA
#define AS5047_MOSI_Pin GPIO_PIN_7
#define AS5047_MOSI_GPIO_Port GPIOA
#define AVGSPEED_Pin GPIO_PIN_0
#define AVGSPEED_GPIO_Port GPIOB
#define AMPSPEED_Pin GPIO_PIN_1
#define AMPSPEED_GPIO_Port GPIOB
#define DSHOT_Pin GPIO_PIN_8
#define DSHOT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t) 2500)        /* Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_V25        ((int32_t)  760)        /* Internal temperature sensor, parameter V25 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_V25_TEMP   ((int32_t)   25)
#define INTERNAL_TEMPSENSOR_V25_VREF   ((int32_t) 3300)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
