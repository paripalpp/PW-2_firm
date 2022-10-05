/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define IM920_IO1_Pin GPIO_PIN_0
#define IM920_IO1_GPIO_Port GPIOA
#define IM920_IO2_Pin GPIO_PIN_1
#define IM920_IO2_GPIO_Port GPIOA
#define IM920_IO3_Pin GPIO_PIN_3
#define IM920_IO3_GPIO_Port GPIOA
#define IM920_IO4_Pin GPIO_PIN_4
#define IM920_IO4_GPIO_Port GPIOA
#define IM920_IO5_Pin GPIO_PIN_5
#define IM920_IO5_GPIO_Port GPIOA
#define IM920_IO8_Pin GPIO_PIN_6
#define IM920_IO8_GPIO_Port GPIOA
#define IM920_IO10_Pin GPIO_PIN_7
#define IM920_IO10_GPIO_Port GPIOA
#define breaker_Pin GPIO_PIN_0
#define breaker_GPIO_Port GPIOB
#define IM920_RESET_Pin GPIO_PIN_8
#define IM920_RESET_GPIO_Port GPIOA
#define IM920_IO6_Pin GPIO_PIN_9
#define IM920_IO6_GPIO_Port GPIOA
#define IM920_IO7_Pin GPIO_PIN_10
#define IM920_IO7_GPIO_Port GPIOA
#define out_stop_dsrk_Pin GPIO_PIN_3
#define out_stop_dsrk_GPIO_Port GPIOB
#define sw1_Pin GPIO_PIN_4
#define sw1_GPIO_Port GPIOB
#define out_emkl_sw2_Pin GPIO_PIN_5
#define out_emkl_sw2_GPIO_Port GPIOB
#define sw3_Pin GPIO_PIN_6
#define sw3_GPIO_Port GPIOB
#define sw4_Pin GPIO_PIN_7
#define sw4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
