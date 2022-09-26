/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define qian_7_Pin GPIO_PIN_4
#define qian_7_GPIO_Port GPIOE
#define qian_6_Pin GPIO_PIN_5
#define qian_6_GPIO_Port GPIOE
#define qian_5_Pin GPIO_PIN_6
#define qian_5_GPIO_Port GPIOE
#define qian_4_Pin GPIO_PIN_11
#define qian_4_GPIO_Port GPIOI
#define qian_3_Pin GPIO_PIN_6
#define qian_3_GPIO_Port GPIOF
#define qian_2_Pin GPIO_PIN_7
#define qian_2_GPIO_Port GPIOF
#define qian_1_Pin GPIO_PIN_8
#define qian_1_GPIO_Port GPIOF
#define key_1_Pin GPIO_PIN_0
#define key_1_GPIO_Port GPIOA
#define key_2_Pin GPIO_PIN_2
#define key_2_GPIO_Port GPIOH
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOB
#define hou_6_Pin GPIO_PIN_6
#define hou_6_GPIO_Port GPIOH
#define hou_7_Pin GPIO_PIN_7
#define hou_7_GPIO_Port GPIOH
#define hou_1_Pin GPIO_PIN_8
#define hou_1_GPIO_Port GPIOH
#define hou_2_Pin GPIO_PIN_12
#define hou_2_GPIO_Port GPIOB
#define hou_3_Pin GPIO_PIN_13
#define hou_3_GPIO_Port GPIOB
#define hou_4_Pin GPIO_PIN_14
#define hou_4_GPIO_Port GPIOB
#define hou_5_Pin GPIO_PIN_15
#define hou_5_GPIO_Port GPIOB
#define you_2_Pin GPIO_PIN_0
#define you_2_GPIO_Port GPIOI
#define you_1_Pin GPIO_PIN_1
#define you_1_GPIO_Port GPIOI
#define zuo_7_Pin GPIO_PIN_11
#define zuo_7_GPIO_Port GPIOC
#define zuo_6_Pin GPIO_PIN_12
#define zuo_6_GPIO_Port GPIOC
#define zuo_5_Pin GPIO_PIN_2
#define zuo_5_GPIO_Port GPIOD
#define zuo_4_Pin GPIO_PIN_3
#define zuo_4_GPIO_Port GPIOD
#define zuo_3_Pin GPIO_PIN_7
#define zuo_3_GPIO_Port GPIOD
#define zuo_2_Pin GPIO_PIN_10
#define zuo_2_GPIO_Port GPIOG
#define you_3_Pin GPIO_PIN_11
#define you_3_GPIO_Port GPIOG
#define zuo_1_Pin GPIO_PIN_13
#define zuo_1_GPIO_Port GPIOG
#define you_4_Pin GPIO_PIN_4
#define you_4_GPIO_Port GPIOI
#define you_5_Pin GPIO_PIN_5
#define you_5_GPIO_Port GPIOI
#define you_6_Pin GPIO_PIN_6
#define you_6_GPIO_Port GPIOI
#define you_7_Pin GPIO_PIN_7
#define you_7_GPIO_Port GPIOI
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
