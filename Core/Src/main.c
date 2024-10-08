/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "projectMain.h"

#include "AS5047D.h"
#include "debug_scope.h"
#include "dshot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       20

#define ESC_POWER_UP 3000  // wait 3s before arming sequence
#define TROTTLE_MAX 500      // trotle max
#define TROTTLE_MIN 48      // trotle min

#define DSHOT_FRAME_SIZE 18
#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MINVOLTAGE (0)
#define MAXVOLTAGE (3300)
#define MINSPEED (0)
#define MAXSPEED (2000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t my_motor_value[4] = {2345, 0, 0, 0};
#define sinMAX (560)
const int32_t sinedata[360] = {0,10,20,29,39,49,59,68,78,88,97,107,116,126,135,145,154,164,173,182,192,201,210,219,228,
237,245,254,263,271,280,288,297,305,313,321,329,337,345,352,360,367,375,382,389,396,403,410,416,423,429,435,441,447,453,
459,464,470,475,480,485,490,494,499,503,508,512,515,519,523,526,529,533,536,538,541,543,546,548,550,551,553,555,556,557,
558,559,559,560,560,560,560,560,559,559,558,557,556,555,553,551,550,548,546,543,541,538,536,533,529,526,523,519,515,512,
508,503,499,494,490,485,480,475,470,464,459,453,447,441,435,429,423,416,410,403,396,389,382,375,367,360,352,345,337,329,
321,313,305,297,288,280,271,263,254,245,237,228,219,210,201,192,182,173,164,154,145,135,126,116,107,97,88,78,68,59,49,39,
29,20,10,0,-10,-20,-29,-39,-49,-59,-68,-78,-88,-97,-107,-116,-126,-135,-145,-154,-164,-173,-182,-192,-201,-210,-219,-228,
-237,-245,-254,-263,-271,-280,-288,-297,-305,-313,-321,-329,-337,-345,-352,-360,-367,-375,-382,-389,-396,-403,-410,-416,
-423,-429,-435,-441,-447,-453,-459,-464,-470,-475,-480,-485,-490,-494,-499,-503,-508,-512,-515,-519,-523,-526,-529,-533,
-536,-538,-541,-543,-546,-548,-550,-551,-553,-555,-556,-557,-558,-559,-559,-560,-560,-560,-560,-560,-559,-559,-558,-557,
-556,-555,-553,-551,-550,-548,-546,-543,-541,-538,-536,-533,-529,-526,-523,-519,-515,-512,-508,-503,-499,-494,-490,-485,
-480,-475,-470,-464,-459,-453,-447,-441,-435,-429,-423,-416,-410,-403,-396,-389,-382,-375,-367,-360,-352,-345,-337,-329,
-321,-313,-305,-297,-288,-280,-271,-263,-254,-245,-237,-228,-219,-210,-201,-192,-182,-173,-164,-154,-145,-135,-126,-116,
-107,-97,-88,-78,-68,-59,-49,-39,-29,-20,-10};

DebugCommand debugCommand = 0;
DebugScope_Handle_t debugData =
  {
        .sz = DEBUGSCOPESIZE,
        .curCh = 1,
        .id = 0,
        .startWriteFlag = false
    };
int32_t encoderAngle = 0;
float spiAngle = 0;

uint16_t ERRFL = 0;
uint16_t PROG = 0;
uint16_t DIAAGC = 0;
uint16_t ANGLEUNC = 0;

uint16_t NOP = 0;
uint16_t CORDICMAG = 0;
uint16_t ANGLECOM = 0;
uint16_t ZPOSM = 0;
uint16_t ZPOSL = 0;
uint16_t SETTINGS1 = 0;
uint16_t SETTINGS2 = 0;

int32_t timerData = 0;
float true_angle = 0.0f;

__IO uint16_t PHASE_Voltage = 0;        /* Value of voltage on GPIO pin (on which is mapped ADC channel) calculated from ADC conversion data (unit: mV) */
__IO uint16_t AVGSPEED_Voltage = 0;        /* Value of voltage on GPIO pin (on which is mapped ADC channel) calculated from ADC conversion data (unit: mV) */
__IO uint16_t AMPSPEED_Voltage = 0;        /* Value of voltage on GPIO pin (on which is mapped ADC channel) calculated from ADC conversion data (unit: mV) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t VoltageToAVGSpeed(const uint16_t voltage);
uint16_t VoltageToAmpSpeed(const uint16_t voltage, const uint16_t curspeed);
uint16_t VoltageToPhase(const uint16_t voltage);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DelayUS(uint32_t us) 
{
    uint32_t duration = us * 16;
    LL_TIM_SetCounter(TIM5, 0);
    LL_TIM_EnableCounter(TIM5);
    uint32_t start = TIM5->CNT;
    while (TIM5->CNT - start < duration);
}

int32_t sineData(int32_t angle)
{
    while (angle < 0)
    {
        angle = 360 + angle;
    }
    while (angle >= 360)
    {
        angle = angle - 360;
    }
    return sinedata[angle];
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  
   /* Initiaize AS5047D */
  uint16_t nop,AGC;

  uint8_t errorFlag[20]={99, 99, 99, 99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99};
  errorFlag[0] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_NOP, &nop);
  errorFlag[1] = AS5047D_Get_AGC_Value(&AGC);
  errorFlag[2] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ERRFL, &ERRFL);
  errorFlag[3] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ERRFL, &ERRFL);
  errorFlag[4] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_PROG, &PROG);
  errorFlag[5] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ANGLEUNC, &ANGLEUNC);
  errorFlag[6] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_CORDICMAG, &CORDICMAG);
  errorFlag[7] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ANGLECOM, &ANGLECOM);
  errorFlag[8] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ZPOSM, &ZPOSM);
  errorFlag[9] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ZPOSL, &ZPOSL);
  errorFlag[10] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_SETTINGS1, &SETTINGS1);
  errorFlag[11] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_SETTINGS2, &SETTINGS2);

  errorFlag[12] = AS5047D_Get_True_Angle_Value(&true_angle);

  dshot_init(DSHOT600);

  for (int i = 0; i < 10; i++)
  {
    dshot_write(0+0); 
    HAL_Delay(1);
  }

  //ProjectMain();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Activate_ADC();

  int32_t avgSpeed = 0;
  int32_t ampSpeed = 0;
  int32_t phase = 0;
  int32_t spiAngle32 = 0;
  uint16_t totalSpeed = 0;
  int32_t sinAnglepPhase = 0;
  int32_t delta = 0;
  uint8_t debugRes = 0;
  float data[DEBUGSCOPENUMOFCH] = {0.0f, 0.0f};
  DebugScopeStartWrite(&debugData);
  while (1)
  {
    errorFlag[7] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ANGLECOM, &ANGLECOM);
    spiAngle32 = ANGLECOM * 360 / 16384;
    // errorFlag[15] = AS5047D_Get_True_Angle_Value(&spiAngle);
    if (errorFlag[7] != 0)
    {
      errorFlag[16] = AS5047D_Read(AS5047_CS_GPIO_Port, AS5047_CS_Pin, AS5047D_ERRFL, &ERRFL);
      continue;
    }

    read_ADC(&PHASE_Voltage, &AVGSPEED_Voltage, &AMPSPEED_Voltage);
    avgSpeed = VoltageToAVGSpeed(AVGSPEED_Voltage);
    if (avgSpeed>1950)
    {
      avgSpeed = 2000;
    }
    if (avgSpeed<50)
    {
      avgSpeed = 0;
    }

    ampSpeed = VoltageToAmpSpeed(AMPSPEED_Voltage, avgSpeed*3/4);
    phase = VoltageToPhase(PHASE_Voltage);

    sinAnglepPhase = sineData(spiAngle32 + phase);
    delta = ampSpeed*sinAnglepPhase/sinMAX;
    totalSpeed = avgSpeed + delta;

    totalSpeed = min(totalSpeed, 2000);
    totalSpeed = max(totalSpeed, 0);

    dshot_write(&totalSpeed);

    data[0] = (float)spiAngle32;
    data[1] = (float)totalSpeed;
    data[2] = (float)delta;
    debugRes = DebugScopeInsertData(&debugData, data);
    if (debugRes == NO_MORE_PLACE_TO_WRITE)
    {
      DebugScopeStartWrite(&debugData);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t VoltageToAVGSpeed(const uint16_t voltage)
{
  return (uint16_t) ((uint32_t)(voltage - MINVOLTAGE) * (MAXSPEED - MINSPEED)/(MAXVOLTAGE - MINVOLTAGE) + MINSPEED);
}

uint16_t VoltageToAmpSpeed(const uint16_t voltage, const uint16_t curspeed)
{
  return (uint16_t)((uint32_t)(voltage - MINVOLTAGE)* curspeed/(MAXVOLTAGE - MINVOLTAGE) );
}

uint16_t VoltageToPhase(const uint16_t voltage)
{
  return (uint16_t)((uint32_t)(voltage - MINVOLTAGE)* 360 /(MAXVOLTAGE - MINVOLTAGE)) ;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
