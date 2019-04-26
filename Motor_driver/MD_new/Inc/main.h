/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define TEST_LED_Pin GPIO_PIN_13
#define TEST_LED_GPIO_Port GPIOC
#define SERVO1_Pin GPIO_PIN_0
#define SERVO1_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_1
#define SERVO2_GPIO_Port GPIOA
#define ENCODER1_A_Pin GPIO_PIN_6
#define ENCODER1_A_GPIO_Port GPIOA
#define ENCODER1_B_Pin GPIO_PIN_7
#define ENCODER1_B_GPIO_Port GPIOA
#define COM_TX_Pin GPIO_PIN_10
#define COM_TX_GPIO_Port GPIOB
#define COM_RX_Pin GPIO_PIN_11
#define COM_RX_GPIO_Port GPIOB
#define STEPPER_A_Pin GPIO_PIN_12
#define STEPPER_A_GPIO_Port GPIOB
#define STEPPER_B_Pin GPIO_PIN_13
#define STEPPER_B_GPIO_Port GPIOB
#define STEPPER_C_Pin GPIO_PIN_14
#define STEPPER_C_GPIO_Port GPIOB
#define STEPPER_D_Pin GPIO_PIN_15
#define STEPPER_D_GPIO_Port GPIOB
#define MOTOR_A_POS_Pin GPIO_PIN_8
#define MOTOR_A_POS_GPIO_Port GPIOA
#define MOTOR_A_NEG_Pin GPIO_PIN_9
#define MOTOR_A_NEG_GPIO_Port GPIOA
#define MOTOR_B_POS_Pin GPIO_PIN_10
#define MOTOR_B_POS_GPIO_Port GPIOA
#define MOTOR_B_NEG_Pin GPIO_PIN_11
#define MOTOR_B_NEG_GPIO_Port GPIOA
#define ENCODER2_A_Pin GPIO_PIN_6
#define ENCODER2_A_GPIO_Port GPIOB
#define ENCODER2_B_Pin GPIO_PIN_7
#define ENCODER2_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
