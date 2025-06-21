/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encoder_Channel_A_Right_Pin GPIO_PIN_0
#define Encoder_Channel_A_Right_GPIO_Port GPIOA
#define Encoder_Channel_B_Right_Pin GPIO_PIN_1
#define Encoder_Channel_B_Right_GPIO_Port GPIOA
#define Encoder_Channel_A_Left_Pin GPIO_PIN_6
#define Encoder_Channel_A_Left_GPIO_Port GPIOA
#define Encoder_Channel_B_Left_Pin GPIO_PIN_7
#define Encoder_Channel_B_Left_GPIO_Port GPIOA
#define Right_control_1_Pin GPIO_PIN_10
#define Right_control_1_GPIO_Port GPIOB
#define Right_control_2_Pin GPIO_PIN_11
#define Right_control_2_GPIO_Port GPIOB
#define Ultrasonic_Trigger_RightBack_Pin GPIO_PIN_13
#define Ultrasonic_Trigger_RightBack_GPIO_Port GPIOB
#define Ultrasonic_Trigger_RightFront_Pin GPIO_PIN_14
#define Ultrasonic_Trigger_RightFront_GPIO_Port GPIOB
#define Ultrasonic_Trigger_Front_Pin GPIO_PIN_15
#define Ultrasonic_Trigger_Front_GPIO_Port GPIOB
#define Ultrasonic_Echo_RightBack_Pin GPIO_PIN_8
#define Ultrasonic_Echo_RightBack_GPIO_Port GPIOA
#define Ultrasonic_Echo_RightFront_Pin GPIO_PIN_9
#define Ultrasonic_Echo_RightFront_GPIO_Port GPIOA
#define Ultrasonic_Echo_Front_Pin GPIO_PIN_10
#define Ultrasonic_Echo_Front_GPIO_Port GPIOA
#define Left_control_1_Pin GPIO_PIN_4
#define Left_control_1_GPIO_Port GPIOB
#define Left_control_2_Pin GPIO_PIN_5
#define Left_control_2_GPIO_Port GPIOB
#define PWM_Left_Pin GPIO_PIN_6
#define PWM_Left_GPIO_Port GPIOB
#define PWM_Right_Pin GPIO_PIN_7
#define PWM_Right_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
