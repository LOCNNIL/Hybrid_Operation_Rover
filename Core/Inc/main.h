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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f401xc.h"
#include "stdint.h"
#include "stdio.h"
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
#define LED_BLUE_Pin GPIO_PIN_13
#define LED_BLUE_GPIO_Port GPIOC
#define TESTE_Pin GPIO_PIN_0
#define TESTE_GPIO_Port GPIOH
#define BUT_Pin GPIO_PIN_0
#define BUT_GPIO_Port GPIOA
#define BUT_EXTI_IRQn EXTI0_IRQn
#define IN4_Pin GPIO_PIN_12
#define IN4_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_13
#define IN3_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_14
#define IN2_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_15
#define IN1_GPIO_Port GPIOB
#define MOT_DIR_Pin GPIO_PIN_8
#define MOT_DIR_GPIO_Port GPIOA
#define MOT_ESQ_Pin GPIO_PIN_9
#define MOT_ESQ_GPIO_Port GPIOA
#define INF_RE_Pin GPIO_PIN_10
#define INF_RE_GPIO_Port GPIOA
#define VEL_ESQ_Pin GPIO_PIN_11
#define VEL_ESQ_GPIO_Port GPIOA
#define VEL_ESQ_EXTI_IRQn EXTI15_10_IRQn
#define VEL_DIR_Pin GPIO_PIN_12
#define VEL_DIR_GPIO_Port GPIOA
#define VEL_DIR_EXTI_IRQn EXTI15_10_IRQn
#define ECHO_Pin GPIO_PIN_15
#define ECHO_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_3
#define TRIG_GPIO_Port GPIOB
#define INF_ESQ_Pin GPIO_PIN_4
#define INF_ESQ_GPIO_Port GPIOB
#define BLE_TX_Pin GPIO_PIN_6
#define BLE_TX_GPIO_Port GPIOB
#define BLE_RX_Pin GPIO_PIN_7
#define BLE_RX_GPIO_Port GPIOB
#define INF_DIR_Pin GPIO_PIN_8
#define INF_DIR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/*#define ARM_MATH_CM4*/
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
