/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_13
#define LED_R_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_14
#define KEY_GPIO_Port GPIOC
#define LED_L_Pin GPIO_PIN_15
#define LED_L_GPIO_Port GPIOC
#define XJ_1_Pin GPIO_PIN_0
#define XJ_1_GPIO_Port GPIOA
#define XJ_2_Pin GPIO_PIN_1
#define XJ_2_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOA
#define XJ_3_Pin GPIO_PIN_3
#define XJ_3_GPIO_Port GPIOA
#define XJ_4_Pin GPIO_PIN_4
#define XJ_4_GPIO_Port GPIOA
#define LQ_B_Pin GPIO_PIN_0
#define LQ_B_GPIO_Port GPIOB
#define LQ_F_Pin GPIO_PIN_1
#define LQ_F_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_12
#define CE_GPIO_Port GPIOB
#define CSN_Pin GPIO_PIN_13
#define CSN_GPIO_Port GPIOB
#define IRQ_Pin GPIO_PIN_14
#define IRQ_GPIO_Port GPIOB
#define XJ_5_Pin GPIO_PIN_15
#define XJ_5_GPIO_Port GPIOB
#define TRIGGER_Pin GPIO_PIN_8
#define TRIGGER_GPIO_Port GPIOA
#define MCU_TX_Pin GPIO_PIN_9
#define MCU_TX_GPIO_Port GPIOA
#define MCU_RX_Pin GPIO_PIN_10
#define MCU_RX_GPIO_Port GPIOA
#define LH_B_Pin GPIO_PIN_11
#define LH_B_GPIO_Port GPIOA
#define LED_HL_Pin GPIO_PIN_12
#define LED_HL_GPIO_Port GPIOA
#define LED_HR_Pin GPIO_PIN_15
#define LED_HR_GPIO_Port GPIOA
#define RQ_F_Pin GPIO_PIN_4
#define RQ_F_GPIO_Port GPIOB
#define RQ_B_Pin GPIO_PIN_5
#define RQ_B_GPIO_Port GPIOB
#define RH_B_Pin GPIO_PIN_6
#define RH_B_GPIO_Port GPIOB
#define RH_F_Pin GPIO_PIN_7
#define RH_F_GPIO_Port GPIOB
#define IR_PWM_Pin GPIO_PIN_8
#define IR_PWM_GPIO_Port GPIOB
#define ECHO_Pin GPIO_PIN_9
#define ECHO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
