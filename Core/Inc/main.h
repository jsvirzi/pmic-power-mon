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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PROBE_VDD33_Pin GPIO_PIN_0
#define PROBE_VDD33_GPIO_Port GPIOF
#define PROBE_VDD11_Pin GPIO_PIN_0
#define PROBE_VDD11_GPIO_Port GPIOA
#define PROBE_VDD08_Pin GPIO_PIN_1
#define PROBE_VDD08_GPIO_Port GPIOA
#define PROBE_VDD18_Pin GPIO_PIN_4
#define PROBE_VDD18_GPIO_Port GPIOA
#define WD_TRIGGER_Pin GPIO_PIN_7
#define WD_TRIGGER_GPIO_Port GPIOA
#define SOC_POR_Pin GPIO_PIN_0
#define SOC_POR_GPIO_Port GPIOB
#define CORE_PWR_GOOD_Pin GPIO_PIN_10
#define CORE_PWR_GOOD_GPIO_Port GPIOA
#define DISABLE_WD_Pin GPIO_PIN_11
#define DISABLE_WD_GPIO_Port GPIOA
#define APP_MONITOR_Pin GPIO_PIN_12
#define APP_MONITOR_GPIO_Port GPIOA
#define PWR3_ENA1_Pin GPIO_PIN_3
#define PWR3_ENA1_GPIO_Port GPIOB
#define PWR3_ENA2_Pin GPIO_PIN_4
#define PWR3_ENA2_GPIO_Port GPIOB
#define PWR3_ENA3_Pin GPIO_PIN_5
#define PWR3_ENA3_GPIO_Port GPIOB
#define PWR3_ENA4_Pin GPIO_PIN_6
#define PWR3_ENA4_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOB
#define PWR3_ENA_Pin GPIO_PIN_8
#define PWR3_ENA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
