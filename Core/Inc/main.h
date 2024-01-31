/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define TIM_FL_Pin GPIO_PIN_6
#define TIM_FL_GPIO_Port GPIOA
#define TIM_BL_Pin GPIO_PIN_7
#define TIM_BL_GPIO_Port GPIOA
#define TIM_FR_Pin GPIO_PIN_0
#define TIM_FR_GPIO_Port GPIOB
#define TIM_BR_Pin GPIO_PIN_1
#define TIM_BR_GPIO_Port GPIOB
#define TIM_PAN_Pin GPIO_PIN_9
#define TIM_PAN_GPIO_Port GPIOA
#define TIM_TILT_Pin GPIO_PIN_10
#define TIM_TILT_GPIO_Port GPIOA
#define GPIO_FR_TAHO_Pin GPIO_PIN_3
#define GPIO_FR_TAHO_GPIO_Port GPIOB
#define GPIO_FR_TAHO_EXTI_IRQn EXTI3_IRQn
#define GPIO_FL_TAHO_Pin GPIO_PIN_4
#define GPIO_FL_TAHO_GPIO_Port GPIOB
#define GPIO_FL_TAHO_EXTI_IRQn EXTI4_IRQn
#define GPIO_HCSR04_ECHO_Pin GPIO_PIN_6
#define GPIO_HCSR04_ECHO_GPIO_Port GPIOB
#define GPIO_HCSR04_TRIG_Pin GPIO_PIN_7
#define GPIO_HCSR04_TRIG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define HTIM_PAN_TILT		htim1
#define HTIM_PWM			htim3
#define HTIM_HCSR04			htim4

#define FL_TIM_CHANNEL		TIM_CHANNEL_1
#define BL_TIM_CHANNEL		TIM_CHANNEL_2
#define FR_TIM_CHANNEL		TIM_CHANNEL_3
#define BR_TIM_CHANNEL		TIM_CHANNEL_4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
