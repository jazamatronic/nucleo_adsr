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
#include "stm32l4xx_hal.h"

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
#define TIM6_PSC 363
#define TIM6_PER 22
#define TIM1_PSC 4
#define TIM1_PER 8191
#define TIM7_PSC 79
#define SH_TIME 7
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define ADC_A_Pin GPIO_PIN_1
#define ADC_A_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define ADC_D_Pin GPIO_PIN_3
#define ADC_D_GPIO_Port GPIOA
#define ADC_S_Pin GPIO_PIN_5
#define ADC_S_GPIO_Port GPIOA
#define ADC_R_Pin GPIO_PIN_6
#define ADC_R_GPIO_Port GPIOA
#define TRIG1_Pin GPIO_PIN_7
#define TRIG1_GPIO_Port GPIOA
#define TRIG2_Pin GPIO_PIN_0
#define TRIG2_GPIO_Port GPIOB
#define TRIG3_Pin GPIO_PIN_1
#define TRIG3_GPIO_Port GPIOB
#define PWM_R_Pin GPIO_PIN_8
#define PWM_R_GPIO_Port GPIOA
#define PWM_G_Pin GPIO_PIN_9
#define PWM_G_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_10
#define PWM_B_GPIO_Port GPIOA
#define SH2_Pin GPIO_PIN_11
#define SH2_GPIO_Port GPIOA
#define SH3_Pin GPIO_PIN_12
#define SH3_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define SH1_Pin GPIO_PIN_4
#define SH1_GPIO_Port GPIOB
#define BTN_Pin GPIO_PIN_5
#define BTN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
