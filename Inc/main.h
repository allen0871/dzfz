/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define CH1_KEY_Pin GPIO_PIN_13
#define CH1_KEY_GPIO_Port GPIOC
#define CH2_KEY_Pin GPIO_PIN_14
#define CH2_KEY_GPIO_Port GPIOC
#define FAN1_Pin GPIO_PIN_15
#define FAN1_GPIO_Port GPIOC
#define FAN2_Pin GPIO_PIN_0
#define FAN2_GPIO_Port GPIOA
#define ADC_SEL2_Pin GPIO_PIN_1
#define ADC_SEL2_GPIO_Port GPIOA
#define ADC_CLK_Pin GPIO_PIN_4
#define ADC_CLK_GPIO_Port GPIOA
#define ADC_SCK_Pin GPIO_PIN_5
#define ADC_SCK_GPIO_Port GPIOA
#define ADC_DIN_Pin GPIO_PIN_6
#define ADC_DIN_GPIO_Port GPIOA
#define ADC_SEL1_Pin GPIO_PIN_7
#define ADC_SEL1_GPIO_Port GPIOA
#define DAC_CS_Pin GPIO_PIN_0
#define DAC_CS_GPIO_Port GPIOB
#define TMP_IN_Pin GPIO_PIN_1
#define TMP_IN_GPIO_Port GPIOB
#define TMP_CS_Pin GPIO_PIN_2
#define TMP_CS_GPIO_Port GPIOB
#define LCD_D2_Pin GPIO_PIN_10
#define LCD_D2_GPIO_Port GPIOB
#define LCD_D3_Pin GPIO_PIN_11
#define LCD_D3_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_12
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_13
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_14
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_15
#define LCD_D7_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_8
#define LCD_RST_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOA
#define LCD_RD_Pin GPIO_PIN_12
#define LCD_RD_GPIO_Port GPIOA
#define LCD_RW_Pin GPIO_PIN_6
#define LCD_RW_GPIO_Port GPIOF
#define LCD_CD_Pin GPIO_PIN_7
#define LCD_CD_GPIO_Port GPIOF
#define E11_A_Pin GPIO_PIN_3
#define E11_A_GPIO_Port GPIOB
#define E11_B_Pin GPIO_PIN_4
#define E11_B_GPIO_Port GPIOB
#define E11_KEY_Pin GPIO_PIN_5
#define E11_KEY_GPIO_Port GPIOB
#define LCD_D0_Pin GPIO_PIN_8
#define LCD_D0_GPIO_Port GPIOB
#define LCD_D1_Pin GPIO_PIN_9
#define LCD_D1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
