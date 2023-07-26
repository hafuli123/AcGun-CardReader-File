/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define GC_RST_PIN  GPIO_PIN_8
#define GC_CE_PIN  GPIO_PIN_9
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
#define BAT_DONE_GPIO   GPIOA
#define BAT_DONE_PIN   GPIO_PIN_15
#define BAT_CHRG_GPIO   GPIOB
#define BAT_CHRG_PIN   GPIO_PIN_5
#define BAT_CHRG_NONE   GPIO_PIN_SET
#define BAT_CHRGING     GPIO_PIN_RESET
#define BAT_DONE_N      GPIO_PIN_SET
#define BAT_DONE_Y      GPIO_PIN_RESET
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
