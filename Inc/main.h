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
#include "stm32g4xx_hal.h"

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
#define NRST_Pin GPIO_PIN_10
#define NRST_GPIO_Port GPIOG
#define ENC_B_1_Pin GPIO_PIN_0
#define ENC_B_1_GPIO_Port GPIOA
#define ENC_B_2_Pin GPIO_PIN_1
#define ENC_B_2_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define PWM_A_Pin GPIO_PIN_4
#define PWM_A_GPIO_Port GPIOA
#define BRAKE_Pin GPIO_PIN_5
#define BRAKE_GPIO_Port GPIOA
#define CURRENT_A_Pin GPIO_PIN_6
#define CURRENT_A_GPIO_Port GPIOA
#define CURRENT_B_Pin GPIO_PIN_7
#define CURRENT_B_GPIO_Port GPIOA
#define Trans0_Pin GPIO_PIN_0
#define Trans0_GPIO_Port GPIOB
#define ENC_A_1_Pin GPIO_PIN_8
#define ENC_A_1_GPIO_Port GPIOA
#define ENC_A_2_Pin GPIO_PIN_9
#define ENC_A_2_GPIO_Port GPIOA
#define DIR_B_Pin GPIO_PIN_10
#define DIR_B_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define Trans1_Pin GPIO_PIN_15
#define Trans1_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define PWM_B_Pin GPIO_PIN_4
#define PWM_B_GPIO_Port GPIOB
#define DIR_A_Pin GPIO_PIN_5
#define DIR_A_GPIO_Port GPIOB
#define Trans2_Pin GPIO_PIN_6
#define Trans2_GPIO_Port GPIOB
#define Trans3_Pin GPIO_PIN_7
#define Trans3_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
