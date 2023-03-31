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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define rfid_reset_Pin GPIO_PIN_0
#define rfid_reset_GPIO_Port GPIOC
#define rfid_irq_Pin GPIO_PIN_1
#define rfid_irq_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define r0_keypad_Pin GPIO_PIN_7
#define r0_keypad_GPIO_Port GPIOA
#define c3_keypad_Pin GPIO_PIN_5
#define c3_keypad_GPIO_Port GPIOC
#define c3_keypad_EXTI_IRQn EXTI9_5_IRQn
#define rfid_nss_Pin GPIO_PIN_12
#define rfid_nss_GPIO_Port GPIOB
#define c2_keypad_Pin GPIO_PIN_6
#define c2_keypad_GPIO_Port GPIOC
#define c2_keypad_EXTI_IRQn EXTI9_5_IRQn
#define r2_keypad_Pin GPIO_PIN_7
#define r2_keypad_GPIO_Port GPIOC
#define c1_keypad_Pin GPIO_PIN_8
#define c1_keypad_GPIO_Port GPIOC
#define c1_keypad_EXTI_IRQn EXTI9_5_IRQn
#define c0_keypad_Pin GPIO_PIN_9
#define c0_keypad_GPIO_Port GPIOC
#define c0_keypad_EXTI_IRQn EXTI9_5_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define r3_keypad_Pin GPIO_PIN_4
#define r3_keypad_GPIO_Port GPIOB
#define r1_keypad_Pin GPIO_PIN_6
#define r1_keypad_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
