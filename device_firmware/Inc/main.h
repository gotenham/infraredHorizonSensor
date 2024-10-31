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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_OSC32_LSClock_IN_OUT_Pin GPIO_PIN_14
#define RCC_OSC32_LSClock_IN_OUT_GPIO_Port GPIOC
#define RCC_OSC32_LSClock_OUT_IN_Pin GPIO_PIN_15
#define RCC_OSC32_LSClock_OUT_IN_GPIO_Port GPIOC
#define RCC_OSC_HSClock_IN_OUT_Pin GPIO_PIN_0
#define RCC_OSC_HSClock_IN_OUT_GPIO_Port GPIOH
#define RCC_OSC_HSClock_OUT_IN_Pin GPIO_PIN_1
#define RCC_OSC_HSClock_OUT_IN_GPIO_Port GPIOH
#define LPUART_DEBUG_TX_Pin GPIO_PIN_2
#define LPUART_DEBUG_TX_GPIO_Port GPIOA
#define LPUART_DEBUG_RX_Pin GPIO_PIN_3
#define LPUART_DEBUG_RX_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_0
#define SPI1_CS_GPIO_Port GPIOB
#define GPIO_EXT_PWM_INPUT_PB12_Pin GPIO_PIN_12
#define GPIO_EXT_PWM_INPUT_PB12_GPIO_Port GPIOB
#define GPIO_EXT_INTERRUPT_PB13_Pin GPIO_PIN_13
#define GPIO_EXT_INTERRUPT_PB13_GPIO_Port GPIOB
#define GPIO_EXT_INTERRUPT_PB13_EXTI_IRQn EXTI15_10_IRQn
#define GPIO_SW2_INPUT_Pin GPIO_PIN_8
#define GPIO_SW2_INPUT_GPIO_Port GPIOA
#define I2C1_SCL_IR_SENSOR_Pin GPIO_PIN_9
#define I2C1_SCL_IR_SENSOR_GPIO_Port GPIOA
#define I2C1_SDA_IR_SENSOR_Pin GPIO_PIN_10
#define I2C1_SDA_IR_SENSOR_GPIO_Port GPIOA
#define GPIO_LED2_YEL_OUTPUT_Pin GPIO_PIN_5
#define GPIO_LED2_YEL_OUTPUT_GPIO_Port GPIOB
#define GPIO_LED1_GRN_OUTPUT_Pin GPIO_PIN_6
#define GPIO_LED1_GRN_OUTPUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
