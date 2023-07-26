/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "bsp_usartx.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define RX_MAX_COUNT           30  // 串口接收最大字节数

/* 私有变量 ------------------------------------------------------------------*/
__IO uint8_t aRxBuffer[RX_MAX_COUNT]={0}; // 接收缓冲区
__IO uint16_t RxCount=0;                  // 已接收到的字节数
__IO uint8_t Frame_flag=0;                // 帧标志：1：一个新的数据帧  0：无数据帧

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int apple,orange;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	  uint8_t txbuf[50];
		uint32_t id,idr  ,ip=0x1234,ipr;uint8_t ids;
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	
	MX_USARTx_Init();
	__HAL_UART_ENABLE_IT(&husartx,UART_IT_RXNE);
	HAL_Delay(900);
	HAL_Delay(900);
	HAL_Delay(900);
	HAL_Delay(900);
	HAL_Delay(900);
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

/* 测试用 */
//	uint8_t txx1[]="AT+MODE=0\r\n";	
//	HAL_UART_Transmit(&husartx,txx1,strlen((char *)txx1),1000);
//	HAL_Delay(900);HAL_Delay(900);
//	uint8_t txx3[]="AT+PID=?\r\n";
//	HAL_UART_Transmit(&husartx,txx3,strlen((char *)txx3),1000);
//	HAL_Delay(900);HAL_Delay(900);
//		uint8_t txx8[]="AT+RFCH=?\r\n";
//	HAL_UART_Transmit(&husartx,txx8,strlen((char *)txx8),1000);
//	HAL_Delay(900);HAL_Delay(900);
//	uint8_t txx5[]="AT+MODE=1\r\n";
//	HAL_UART_Transmit(&husartx,txx5,strlen((char *)txx5),1000);
//	HAL_Delay(900);HAL_Delay(900);


/* 初始化用 */
	uint8_t txx1[]="AT+MODE=0\r\n";
	HAL_UART_Transmit(&husartx,txx1,strlen((char *)txx1),1000);
	HAL_Delay(900);HAL_Delay(900);
	
	uint8_t txx2[]="AT+RFBR=1000000\r\n";
	HAL_UART_Transmit(&husartx,txx2,strlen((char *)txx2),1000);
	HAL_Delay(900);HAL_Delay(900);
		
	uint8_t txx3[]="AT+PID=0\r\n";
	HAL_UART_Transmit(&husartx,txx3,strlen((char *)txx3),1000);
	HAL_Delay(900);HAL_Delay(900);
	uint8_t txx9[]="AT+RFCH=0\r\n";
	HAL_UART_Transmit(&husartx,txx9,strlen((char *)txx9),1000);
	HAL_Delay(900);HAL_Delay(900);
	
	uint8_t txx4[]="AT+UART=115200,0,0\r\n";
	HAL_UART_Transmit(&husartx,txx4,strlen((char *)txx4),1000);
	HAL_Delay(900);HAL_Delay(900);

	uint8_t txx5[]="AT+MODE=1\r\n";
	HAL_UART_Transmit(&husartx,txx5,strlen((char *)txx5),1000);
	HAL_Delay(900);HAL_Delay(900);

	apple=1;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
  while (1)
  {
    /* USER CODE END WHILE */
    if(Frame_flag)  // 接收到一个完整数据帧
		{	
		  RxCount=0;    // 清除字节计数
   		Frame_flag=0; // 清除数据帧
		}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void USARTx_IRQHANDLER(void)
{
	if(__HAL_USART_GET_FLAG(&husartx,USART_FLAG_RXNE)!= RESET)  // 接收中断：接收到数据
	{
//		apple++;
		uint8_t data;
		data=READ_REG(husartx.Instance->DR); // 读取数据
		if(RxCount==0) // 如果是重新接收到数据帧，开启串口空闲中断
		{
			__HAL_UART_CLEAR_FLAG(&husartx,USART_FLAG_IDLE); // 清除空闲中断标志
		  __HAL_UART_ENABLE_IT(&husartx,UART_IT_IDLE);     // 使能空闲中断	    
		}
		if(RxCount<RX_MAX_COUNT)    // 判断接收缓冲区未满
		{
			aRxBuffer[RxCount]=data;  // 保存数据
			RxCount++;                // 增加接收字节数计数
		}
	}
	else	if(__HAL_USART_GET_FLAG(&husartx,USART_FLAG_IDLE)!= RESET) // 串口空闲中断
	{
		__HAL_UART_CLEAR_FLAG(&husartx,USART_FLAG_IDLE); // 清除空闲中断标志
		__HAL_UART_DISABLE_IT(&husartx,UART_IT_IDLE);    // 关闭空闲中断
		Frame_flag=1;		                                 // 数据帧置位，标识接收到一个完整数据帧
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
//{
////  HAL_UART_Transmit(&husartx,&aRxBuffer,1,0);
//  HAL_UART_Receive_IT(&husartx,&aRxBuffer,3);
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
