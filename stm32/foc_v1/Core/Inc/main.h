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
#define ANGLE_WRAP_THRESHOLD 300
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/**
 * @brief Calculates the cumulated angle.
 * 
 * @param angle_cul_prev int32_t, the previous cumulated angle
 * @param angle_raw_prev uint16_t, the previous raw angle. Range: 0 to 4095
 * @param angle_raw uint16_t, the current raw angle. Range: 0 to 4095
 * @return int32_t 
 */
static inline int32_t calc_cul_angle(int32_t angle_cul_prev, uint16_t angle_raw_prev, uint16_t angle_raw) {
  int32_t angle_cul;
  if(angle_raw_prev > 4095 - ANGLE_WRAP_THRESHOLD && angle_raw < ANGLE_WRAP_THRESHOLD) {
    // forward wrap around
    angle_cul = angle_cul_prev + (4096 - angle_raw_prev + angle_raw);
  } else if(angle_raw_prev < ANGLE_WRAP_THRESHOLD && angle_raw > 4095 - ANGLE_WRAP_THRESHOLD) {
    // backward wrap around
    angle_cul = angle_cul_prev - (4096 - angle_raw + angle_raw_prev);
  } else {
    // no wrap around
    angle_cul = angle_cul_prev + (angle_raw - angle_raw_prev);
  }

  return angle_cul;
}
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
