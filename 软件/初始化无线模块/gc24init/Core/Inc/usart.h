/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#include "stm32f4xx_hal.h"
/* Includes ------------------------------------------------------------------*/
#define USARTx                                 USART2
#define USARTx_BAUDRATE                        115200
#define USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART2_CLK_ENABLE()
#define USART_RCC_CLK_DISABLE()                __HAL_RCC_USART2_CLK_DISABLE()

#define USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_Tx_GPIO_PIN                     GPIO_PIN_2
#define USARTx_Tx_GPIO                         GPIOA
#define USARTx_Rx_GPIO_PIN                     GPIO_PIN_2   
#define USARTx_Rx_GPIO                         GPIOA

#define USARTx_AFx                             GPIO_AF7_USART2

#define USARTx_IRQHANDLER                      USART2_IRQHandler
#define USARTx_IRQn                            USART2_IRQn


/* 扩展变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef husartx;

/* 函数声明 ------------------------------------------------------------------*/
void MX_USARTx_Init(void);


#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
