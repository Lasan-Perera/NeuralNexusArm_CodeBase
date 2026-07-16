/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
typedef struct {
    int32_t position;
    int32_t target;

    float speed;
    float maxSpeed;
    float acceleration;

    uint32_t stepInterval;
    uint32_t stepCounter;

    GPIO_TypeDef* STEP_Port;
    uint16_t STEP_Pin;

    GPIO_TypeDef* DIR_Port;
    uint16_t DIR_Pin;

    bool pulsePending;   // STEP pin is HIGH, needs clearing at start of next tick

} Stepper_t;

Stepper_t motors[6];

/* ---- Per-joint configuration, all in JOINT order J1..J6 (base -> tip) ---- */

/* Joint (base -> tip) to motor-slot mapping. motors[] is 0-indexed: M1=0 .. M6=5.
 *   J1 Base rotation   -> M6 -> motors[5]
 *   J2 2nd link        -> M4 -> motors[3]
 *   J3 3rd link        -> M5 -> motors[4]
 *   J4 4th joint       -> M2 -> motors[1]
 *   J5 5th joint       -> M1 -> motors[0]
 *   J6 End effector rot-> M3 -> motors[2]                                     */
const uint8_t jointToMotor[6] = { 5, 3, 4, 1, 0, 2 };
const int8_t  jointDir[6]     = { 1, 1, 1, 1, 1, 1 };   // all positive for now

/* Per-joint speed & acceleration (steps/s and steps/s^2). Tune each link. */
const float   jointMaxSpeed[6] = { 150, 150, 150, 150, 150, 150 };
const float   jointAccel[6]    = { 150, 150, 150, 150, 150, 150 };

/* Pulses per JOINT revolution (pulses/motor-rev × gear ratio), J1..J6 */
const float pulsesPerJointRev[6] = { 2800, 1000, 5000, 1600, 1600, 1600 };

/* Joint angle offsets (deg): software zero -> your CAD/model zero.
   J3 and J5 live on the 180°-centered band, so subtract 180 here. */
const float jointOffsetDeg[6] = { 0, 0, 180, 0, 180, 0 };


extern volatile uint8_t rxFlag;
extern char rxTemp[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
void Stepper_InitAll(void);
void Stepper_SetEnableM1M2M3(bool enable);
void Stepper_SetEnableM4M5M6(bool enable);
void Stepper_Update(Stepper_t *m);
void Stepper_MoveAll(int32_t steps[6]);
void Stepper_MoveAllJoints(int32_t jointSteps[6]);
uint8_t Stepper_AnyMoving(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Buzzer_SetFreq(uint32_t freq);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_PWREx_EnableUSBVoltageDetector();

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection    = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
	  Error_Handler();
  }

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
//  MX_SDMMC1_SD_Init();
  MX_SPI3_Init();
//  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  Stepper_InitAll();
  Stepper_SetEnableM1M2M3(true);   // enable M1, M2, M3 (onboard TMC2209) drivers before moving
  Stepper_SetEnableM4M5M6(false);   // enable M4, M5, M6 (external CL57T/CL57T/DM542) drivers before moving
  HAL_TIM_Base_Start_IT(&htim6);

  // Test: move ONE joint at a time to verify mapping/direction.

  int32_t jointSteps[6] = {0, 0, 0, 0, 0, 500};
  Stepper_MoveAllJoints(jointSteps);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* --- PE0/PE1/PE2 comparison test: all three should read 3.3V --- */
	  while (1)
	  {
//	      HAL_GPIO_WritePin(M1_Step_GPIO_Port, M1_Step_Pin, GPIO_PIN_SET);   // PE0 - suspect
//	      HAL_GPIO_WritePin(M2_Step_GPIO_Port, M2_Step_Pin, GPIO_PIN_SET);   // PE1 - known good
//	      HAL_GPIO_WritePin(M3_Step_GPIO_Port, M3_Step_Pin, GPIO_PIN_SET);   // PE2 - known good
	  }

	  if (rxFlag)
	  {
		rxFlag = 0;

		float ang[6] = {0};
		int n = sscanf(rxTemp, "%f,%f,%f,%f,%f,%f",
					   &ang[0], &ang[1], &ang[2],
					   &ang[3], &ang[4], &ang[5]);

		if (n == 6)
		{
		  int32_t jointSteps[6];
		  for (int j = 0; j < 6; j++)
		  {
			float a = ang[j];
			jointSteps[j] = (int32_t)lroundf(pulsesPerJointRev[j] * a / 360.0f);
		  }
		  for (int k = 0; k < 6; k++) motors[k].position = 0;   // treat every command as a fresh delta
		  Stepper_MoveAllJoints(jointSteps);                   // MOVE


		  char reply[128];
		  int len = snprintf(reply, sizeof(reply),
			  "steps: %ld %ld %ld %ld %ld %ld\r\n",
			  (long)jointSteps[0], (long)jointSteps[1], (long)jointSteps[2],
			  (long)jointSteps[3], (long)jointSteps[4], (long)jointSteps[5]);
		  CDC_Transmit_FS((uint8_t*)reply, len);
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
		else
		{
		  char reply[64];
		  int len = snprintf(reply, sizeof(reply), "ERR got %d values\r\n", n);
		  CDC_Transmit_FS((uint8_t*)reply, len);
		}
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 13;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4199;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M3_Step_Pin|LED_Pin|M4_Step_Pin|M5_Step_Pin
                          |M6_Step_Pin|M2_Step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M1_EN_Pin|M2_EN_Pin|M3_EN_Pin|M4_EN_Pin
                          |M5_EN_Pin|M6_EN_Pin|M5_ENC_CS_Pin|M4_ENC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M1_ENC_CS_Pin|M2_ENC_CS_Pin|M3_ENC_CS_Pin|SPI3_CS_Pin
                          |RGB_DIN_Pin|M1_Step_Pin|M4_Dir_Pin|M2_MS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, M2_MS2_Pin|M3_MS1_Pin|M3_Dir_Pin|M6_ENC_CS_Pin
                          |M1_Dir_Pin|M2_Dir_Pin|M5_Dir_Pin|M6_Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M3_Step_Pin M2_Step_Pin */
  GPIO_InitStruct.Pin = M3_Step_Pin|M2_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M4_Step_Pin M5_Step_Pin M6_Step_Pin */
  GPIO_InitStruct.Pin = M4_Step_Pin|M5_Step_Pin|M6_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_EN_Pin M2_EN_Pin M3_EN_Pin */
  GPIO_InitStruct.Pin = M1_EN_Pin|M2_EN_Pin|M3_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M4_EN_Pin M5_EN_Pin M6_EN_Pin */
  GPIO_InitStruct.Pin = M4_EN_Pin|M5_EN_Pin|M6_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_ENC_CS_Pin M2_ENC_CS_Pin M3_ENC_CS_Pin RGB_DIN_Pin
                           M1_Step_Pin */
  GPIO_InitStruct.Pin = M1_ENC_CS_Pin|M2_ENC_CS_Pin|M3_ENC_CS_Pin|RGB_DIN_Pin
                          |M1_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : M2_MS2_Pin */
  GPIO_InitStruct.Pin = M2_MS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M2_MS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M3_MS1_Pin M3_Dir_Pin M6_ENC_CS_Pin M1_Dir_Pin
                           M2_Dir_Pin */
  GPIO_InitStruct.Pin = M3_MS1_Pin|M3_Dir_Pin|M6_ENC_CS_Pin|M1_Dir_Pin
                          |M2_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : M5_ENC_CS_Pin M4_ENC_CS_Pin */
  GPIO_InitStruct.Pin = M5_ENC_CS_Pin|M4_ENC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M5_Dir_Pin M6_Dir_Pin */
  GPIO_InitStruct.Pin = M5_Dir_Pin|M6_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI3_CS_Pin M2_MS1_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin|M2_MS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : M4_Dir_Pin */
  GPIO_InitStruct.Pin = M4_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(M4_Dir_GPIO_Port, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Set buzzer tone frequency (Hz) on TIM2_CH2 / PA1. freq=0 -> silent. */
void Buzzer_SetFreq(uint32_t freq)
{
    if (freq == 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        return;
    }
    uint32_t timClk = HAL_RCC_GetPCLK1Freq() * 2;   // TIM2 kernel clock
    uint32_t psc    = (timClk / 1000000u) - 1;      // counter ticks at 1 MHz
    uint32_t arr    = (1000000u / freq) - 1;

    __HAL_TIM_SET_PRESCALER(&htim2, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, arr / 2);   // 50% duty
    htim2.Instance->EGR = TIM_EGR_UG;               // load new PSC/ARR now
}

/**
 * @brief Drive EN for M1-M3 (CL57T x2 + TB6600, all wired common-anode:
 *        EN+ tied to external +5V, EN- to this open-drain GPIO pin).
 *
 *        Open-drain behavior: GPIO_PIN_RESET (LOW) actively sinks current
 *        -> opto LED conducts. GPIO_PIN_SET (HIGH) releases the pin
 *        (Hi-Z) -> no current path -> opto LED off.
 *
 *        IMPORTANT - polarity is a best guess from datasheets, not yet
 *        verified on your bench:
 *          - CL57T (M1, M2): spec implies LED CONDUCTING = enabled.
 *            -> drive LOW to enable.
 *          - TB6600 (M3): spec implies LED CONDUCTING = "EN valid" =
 *            motor FREE/disabled (opposite convention).
 *            -> drive HIGH (released) to enable.
 *
 *        TEST THIS: with the motor powered and enabled per the logic
 *        below, you should NOT be able to easily turn the shaft by hand
 *        (holding torque present). If it spins freely instead, that
 *        axis's polarity is backwards - flip the corresponding line below.
 */
void Stepper_SetEnableM1M2M3(bool enable)
{
    // Onboard TMC2209 x3 (M1, M2, M3): EN is active-LOW (LOW = driver enabled).
    // All three now share identical push-pull, active-low polarity - if any
    // single axis still doesn't lock after this change, flip only that line.
    HAL_GPIO_WritePin(M1_EN_GPIO_Port, M1_EN_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_EN_GPIO_Port, M2_EN_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(M3_EN_GPIO_Port, M3_EN_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief Enable/disable M4-M6 external drivers (open-drain, common-anode).
 *        M4, M5 = CL57T (NEMA24): LOW = enabled.
 *        M6 = DM542 (NEMA23): HIGH (released) = enabled - opposite of M4/M5.
 *        Same polarity logic that used to live on M1-M3 before those moved
 *        to the onboard TMC2209s - verify on hardware, one axis at a time,
 *        same as the original M1-M3 bring-up.
 */
void Stepper_SetEnableM4M5M6(bool enable)
{
    HAL_GPIO_WritePin(M4_EN_GPIO_Port, M4_EN_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(M5_EN_GPIO_Port, M5_EN_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(M6_EN_GPIO_Port, M6_EN_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Stepper_InitAll(void)
{
    // Motor 1
    motors[0].STEP_Port = M1_Step_GPIO_Port;
    motors[0].STEP_Pin  = M1_Step_Pin;
    motors[0].DIR_Port  = M1_Dir_GPIO_Port;
    motors[0].DIR_Pin   = M1_Dir_Pin;

    // Motor 2
    motors[1].STEP_Port = M2_Step_GPIO_Port;
    motors[1].STEP_Pin  = M2_Step_Pin;
    motors[1].DIR_Port  = M2_Dir_GPIO_Port;
    motors[1].DIR_Pin   = M2_Dir_Pin;

    // Motor 3
    motors[2].STEP_Port = M3_Step_GPIO_Port;
    motors[2].STEP_Pin  = M3_Step_Pin;
    motors[2].DIR_Port  = M3_Dir_GPIO_Port;
    motors[2].DIR_Pin   = M3_Dir_Pin;

    // Motor 4
    motors[3].STEP_Port = M4_Step_GPIO_Port;
    motors[3].STEP_Pin  = M4_Step_Pin;
    motors[3].DIR_Port  = M4_Dir_GPIO_Port;
    motors[3].DIR_Pin   = M4_Dir_Pin;

    // Motor 5
    motors[4].STEP_Port = M5_Step_GPIO_Port;
    motors[4].STEP_Pin  = M5_Step_Pin;
    motors[4].DIR_Port  = M5_Dir_GPIO_Port;
    motors[4].DIR_Pin   = M5_Dir_Pin;

    // Motor 6
    motors[5].STEP_Port = M6_Step_GPIO_Port;
    motors[5].STEP_Pin  = M6_Step_Pin;
    motors[5].DIR_Port  = M6_Dir_GPIO_Port;
    motors[5].DIR_Pin   = M6_Dir_Pin;

    for (int j = 0; j < 6; j++)
        {
            uint8_t m = jointToMotor[j];
            motors[m].position     = 0;
            motors[m].target       = 0;

            motors[m].speed        = 0;
            motors[m].maxSpeed     = jointMaxSpeed[j];   // per-joint now
            motors[m].acceleration = jointAccel[j];      // per-joint now

            motors[m].stepInterval = 1000;
            motors[m].stepCounter  = 0;
            motors[m].pulsePending = false;
        }
}

/**
 * @brief Command all 6 axes to move by the given number of steps
 *        (relative to their current position). Direction is encoded
 *        in the sign of each value (negative = reverse).
 *        The trapezoidal ramp in Stepper_Update() takes it from there.
 */
void Stepper_MoveAll(int32_t steps[6])
{
    for (int i = 0; i < 6; i++)
    {
        motors[i].target = motors[i].position + steps[i];
    }
}

/* Command all axes in JOINT order J1..J6. Handles wiring order + direction. */
void Stepper_MoveAllJoints(int32_t jointSteps[6])
{
    for (int j = 0; j < 6; j++)
    {
        uint8_t m = jointToMotor[j];
        motors[m].target = motors[m].position + jointDir[j] * jointSteps[j];
    }
}
/**
 * @brief Returns 1 if any axis still has steps left to complete.
 */
uint8_t Stepper_AnyMoving(void)
{
    for (int i = 0; i < 6; i++)
    {
        if (motors[i].position != motors[i].target) return 1;
    }
    return 0;
}

void Stepper_Update(Stepper_t *m)
{
    /* Finish whatever pulse was started on the PREVIOUS tick first.
     * This guarantees every HIGH pulse lasts one full 20us tick (50kHz
     * TIM6) instead of a hand-tuned busy-wait that was only ~1-2us -
     * below the minimum pulse width the drivers need. */
    if (m->pulsePending)
    {
        HAL_GPIO_WritePin(m->STEP_Port, m->STEP_Pin, GPIO_PIN_RESET);
        m->pulsePending = false;
    }

    if (m->position == m->target) return;

    int32_t distance = m->target - m->position;

    // Direction
    HAL_GPIO_WritePin(m->DIR_Port, m->DIR_Pin,
                      (distance > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    float dt = 0.00002f; /* matches TIM6 now reconfigured to 50kHz */

    float stepsToStop = (m->speed * m->speed) / (2.0f * m->acceleration);

    if (fabs(distance) < stepsToStop)
    {
        m->speed -= m->acceleration * dt;
        if (m->speed < 0) m->speed = 0;
    }
    else
    {
        if (m->speed < m->maxSpeed)
        {
            m->speed += m->acceleration * dt;
            if (m->speed > m->maxSpeed)
                m->speed = m->maxSpeed;
        }
    }

    if (m->speed > 0)
        m->stepInterval = (uint32_t)(50000.0f / m->speed);

    m->stepCounter++;

    if (m->stepCounter >= m->stepInterval)
    {
        m->stepCounter = 0;

        HAL_GPIO_WritePin(m->STEP_Port, m->STEP_Pin, GPIO_PIN_SET);
        m->pulsePending = true; /* cleared at the START of the next tick, not here */

        if (distance > 0)
            m->position++;
        else
            m->position--;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        for (int i = 0; i < 6; i++)
        {
            Stepper_Update(&motors[i]);
        }
    }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    for (volatile int i = 0; i < 400000; i++);   // fast blink (busy-wait, since IRQs are off)
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
