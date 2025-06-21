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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "takesample.h"
#include "exfunction.h"
#include "ultrasonic.h"
#include "pid.h"
#include "pwm.h"
#include "controlmotor.h"
#include "testprogram00.h"

#include <math.h>
#include <stdint.h>
#include <string.h>
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
ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
/* USER CODE BEGIN PV */

// General Flag/Variable
volatile uint8_t Sample_flag = 0;
volatile uint8_t Sample_enc_flag = 0;

volatile const float Sample_time_ms = 20;
float const PI = 3.14159265359;
uint16_t test = 0;
// ---------------------------------------------------------------------------------


// Ultrasonic Variable
uint32_t InCapFR_01 = 0;
uint32_t InCapFR_02 = 0;
uint32_t InCapRFR_01 = 0;
uint32_t InCapRFR_02 = 0;
uint32_t InCapRBR_01 = 0;
uint32_t InCapRBR_02 = 0;

uint32_t diff_f = 0;
uint32_t diff_rf = 0;
uint32_t diff_rb = 0;
uint8_t IsFirstCap = 0;

float Front_dis = 0;
float RightFront_dis = 0;
float RightBack_dis = 0;

volatile uint8_t F_done = 0;
volatile uint8_t RF_done = 0;
volatile uint8_t RB_done = 0;

float theta = 0;
float dis = 0;
float omega = 0;

volatile uint8_t Front_close = 0;
volatile uint8_t Front_distant = 0;
volatile uint8_t Turn_Left = 0;

volatile uint8_t Turn_Right_RF = 0;
volatile uint8_t Turn_Right_RB = 0;
volatile uint8_t Turn_Right = 0;



// ---------------------------------------------------------------------------------


// Encoder Variable
volatile uint8_t isChange_R = 0;
volatile uint8_t isChange_L = 0;

volatile uint16_t pre_encR_cnt = 0;
volatile float preR = 0;
volatile uint16_t encR_cnt_PA0_PA1 = 0;
volatile uint16_t encR_cnt = 0;
volatile int16_t encR_cnt_0 = 0;
volatile int16_t encR_cnt_1 = 0;


volatile uint16_t pre_encL_cnt = 0;
volatile uint16_t encL_cnt_PA6_PA7 = 0;
volatile uint16_t encL_cnt = 0;
volatile int16_t encL_cnt_0 = 0;
volatile int16_t encL_cnt_1 = 0;


volatile float omegaR = 0;
volatile float omegaL = 0;
float rpmR = 0;
float rpmL = 0;
float rpsR = 0;
float rpsL = 0;

volatile float omegaR_SP = 0;
volatile float omegaL_SP = 0;
// ---------------------------------------------------------------------------------


// Right Motor PID Variable
float Kp_R = 40;
float Ki_R = 80;
float Kd_R = 0.3;
/* 
Ngon:	50	40	0
							
*/
float uk_R = 0;
float uk_1_R = 0;
float ek_R = 0;
float ek_1_R = 0;
float ek_2_R = 0;

float const umax_R = 399;
float const umin_R = -399;
float PID_R = 0;
// ---------------------------------------------------------------------------------


// Left Motor PID Variable
float Kp_L = 40;
float Ki_L = 80;
float Kd_L = 0.3;

float uk_L = 0;
float uk_1_L = 0;
float ek_L = 0;
float ek_1_L = 0;
float ek_2_L = 0;

float const umax_L = 399;
float const umin_L = -399;
float PID_L = 0;
// ---------------------------------------------------------------------------------


// Distance PID Variable
float distance_setpoint = 30;
float v_setpoint = 25;
	
float Kp_dis = 100;	// 100 25 0.75
float Ki_dis = 25;
float Kd_dis = 0.75;

float uk_dis = 0;
float uk_1_dis = 0;
float ek_dis = 0;
float ek_1_dis = 0;
float ek_2_dis = 0;

float const umax_dis = 400;
float const umin_dis = -400;
float PID_dis = 0;
// ---------------------------------------------------------------------------------

// Extern Variable
uint8_t omegaR_buff[sizeof(float)] = {0};
uint8_t omegaL_buff[sizeof(float)] = {0};

uint16_t pwm_setup01 = 0;
uint16_t pwm_setup02 = 0;

// ---------------------------------------------------------------------------------


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
	{ 
		// Ultrasonic
		if(htim->Instance == TIM1)
		{
//			test0 = 2603;
			// TIMER 1, CHANNEL 1 - Ultrasonic RigtBack
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			{
				if(IsFirstCap == 0)
				{
					InCapRBR_01 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
					IsFirstCap = 1; // set the first captured as true
					// Now change the plarity to falling edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
				}
				else if (IsFirstCap == 1) // if the first is already captured
				{
					InCapRBR_02 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
//					__HAL_TIM_SET_COUNTER(htim,0); // reset the counter
					
					if(InCapRBR_02 > InCapRBR_01)
					{
						diff_rb = InCapRBR_02 - InCapRBR_01;
					}
					else if (InCapRBR_01 > InCapRBR_02)
					{
						diff_rb = (19999 - InCapRBR_01) + InCapRBR_02;
					}
					RightBack_dis = roundf((diff_rb * 0.017)*10)/10 + 0.25;
					IsFirstCap = 0; // set it back to false
					
					// set polarity to rising edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
					RB_done = 1;
				}
			}
			// TIMER 1, CHANNEL 2 - Ultrasonic RightFront
			else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			{
				if(IsFirstCap == 0)
				{
					InCapRFR_01 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
					IsFirstCap = 1; // set the first captured as true
					// Now change the plarity to falling edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
				}
				else if (IsFirstCap == 1) // if the first is already captured
				{
					InCapRFR_02 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read second value
//					__HAL_TIM_SET_COUNTER(htim,0); // reset the counter
					
					if(InCapRFR_02 > InCapRFR_01)
					{
						diff_rf = InCapRFR_02 - InCapRFR_01;
					}
					else if (InCapRFR_01 > InCapRFR_02)
					{
						diff_rf = (19999 - InCapRFR_01) + InCapRFR_02;
					}
					RightFront_dis = roundf((diff_rf * 0.017)*10)/10 - 0.25;
					IsFirstCap = 0; // set it back to false
					
					// set polarity to rising edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC2);
					RF_done = 1;
				}
			}
			// TIMER 1, CHANNEL 3 - Ultrasonic Front
			else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
			{
				if(IsFirstCap == 0)
				{
					InCapFR_01 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // read the first value
					IsFirstCap = 1; // set the first captured as true
					// Now change the plarity to falling edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
				}
				else if (IsFirstCap == 1) // if the first is already captured
				{
					InCapFR_02 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // read second value
//					__HAL_TIM_SET_COUNTER(htim,0); // reset the counter
					
					if(InCapFR_02 > InCapFR_01)
					{
						diff_f = InCapFR_02 - InCapFR_01;
					}
					else if (InCapFR_01 > InCapFR_02)
					{
						diff_f = (19999 - InCapFR_01) + InCapFR_02;
					}
					Front_dis = roundf((diff_f * 0.017)*10)/10;
					IsFirstCap = 0; // set it back to false
					
					// set polarity to rising edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC3);
					F_done = 1;
				}
			}
		}
		// Encoder
		else if (htim->Instance == TIM2) 
		{
			encR_cnt = (__HAL_TIM_GET_COUNTER(htim));
//			encL_cnt = (__HAL_TIM_GET_COUNTER(&htim3));

			isChange_R = 1;
		}
		else if (htim->Instance == TIM3) 
		{
//			encL_cnt = (__HAL_TIM_GET_COUNTER(htim));
			isChange_L = 1;
		}
	}


	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)
	{
		Encoder_processing_ver1();
//		
		PID_R = PID_R_control(omegaR_SP, omegaR);
		PID_L = PID_L_control(omegaL_SP, omegaL);
////		
//		PID_R = PID_R_control(5, omegaR);
//		PID_L = PID_L_control(6, omegaL);
		
//		SendToMATLAB_omegaR();
////		
		PIDtoPWM_R_ver1(PID_R);
		PIDtoPWM_L_ver1(PID_L);
		
		//HAL_IWDG_Refresh(&hiwdg);

	}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	
	__HAL_TIM_SET_COUNTER(&htim2, 0x8000);
	__HAL_TIM_SET_COUNTER(&htim3, 0x8000);

	
	HAL_TIM_Base_Start_IT(&htim1);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
	
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	
//	pwm_setup01 = 325;
//	setup01();
	
//	pwm_setup02 = 0;
//	setup02();

//	setup00();
//	HAL_Delay(500);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		setup01();
//		setup02();

//		test_enc_R(); //dem counter
//		test_enc_L();

//		sample_enc_ver0();
//		sample_enc_ver1();
//		sample_enc_ver2();
//		sample_enc_ver3();


//		sample_speedctrl_ver2();

//		sample_all_ver0();		
//		sample_all_ver1();
//		sample_all_ver2();
//		sample_all_ver3();
		sample_all_ver4();
		


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 150;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 399;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_channel1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
  hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Right_control_1_Pin|Right_control_2_Pin|Ultrasonic_Trigger_RightBack_Pin|Ultrasonic_Trigger_RightFront_Pin
                          |Ultrasonic_Trigger_Front_Pin|Left_control_1_Pin|Left_control_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Right_control_1_Pin Right_control_2_Pin Ultrasonic_Trigger_RightBack_Pin Ultrasonic_Trigger_RightFront_Pin
                           Ultrasonic_Trigger_Front_Pin Left_control_1_Pin Left_control_2_Pin */
  GPIO_InitStruct.Pin = Right_control_1_Pin|Right_control_2_Pin|Ultrasonic_Trigger_RightBack_Pin|Ultrasonic_Trigger_RightFront_Pin
                          |Ultrasonic_Trigger_Front_Pin|Left_control_1_Pin|Left_control_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
