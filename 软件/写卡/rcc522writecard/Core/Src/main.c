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
	读块11-》0x0b
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc522_config.h"
#include "rc522_function.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//uint8_t rc522_read(int fd,void*buf,int count);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t cardtype_temp[5] ,pTagType[2];
uint8_t apple ,writedone, step;
uint8_t rw_buf[]="testrc522";
uint8_t recv_buf[16];
char cStr [ 30 ];
uint8_t ucArray_ID [ 4 ];  //先后存放IC卡的UID(IC卡序列号)
uint8_t ucStatusReturn;          //返回状态	
uint8_t KEY_def[6]={0xff,0xff,0xff,0xff,0xff,0xff};
uint8_t KEY[6]={0xf5,0xfA,0xf0,0x00,0x88,0x18};	

	
uint8_t KEYCON[]={	0xff,0xff,0xff,0xff,0xff,0xff,
			0x08, 0x77, 0x8F, 0x69,
		0xf5,0xfA,0xf0,0x00,0x88,0x18	};
//块012 keyAB均可读，仅keyB可写。块3密码内容，keyA和keyB均不可读，只能用keyB写入。
//块3控制字节keyAB均可读，只有keyB可写。
	
		
#define     RECORD_ADDR     0x07    //记录信息的M1卡块地址
	uint8_t rc522_KeyB[6]={0xf5,0xfA,0xf0,0x00,0x88,0x18};  //B密码
		
	uint8_t ipmessage[16]={0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  /* USER CODE BEGIN 2 */
	SPIx_Init();
	PcdReset();
	M500PcdConfigISOType ( 'A' );//设置工作方式
  /* USER CODE END 2 */

	uint16_t val;
	uint8_t *p,*d;
	val=0x0201;	//IP：0x信道PID  ，注意写入卡里的16字节数组为{0xaa, 0xPID, 0x信道, 和校验 , .. 0x00}
	d=(uint8_t*)&val;
	
	p=ipmessage;
	*p=0xaa;
	*(p+1)=*d;
	*(p+2)=*(d+1);
	
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* step 1*/
		/*
		if ( ( ucStatusReturn = PcdRequest ( PICC_REQALL, pTagType ) ) == MI_OK ){
			if ( PcdAnticoll ( ucArray_ID ) == MI_OK ){
				if(pTagType[0]==0x04){
					if(PcdSelect(ucArray_ID)== MI_OK ){	
							if(PcdAuthState ( 0x61, 0x07, KEY_def, ucArray_ID )== MI_OK){	
								if(apple==0){
									PcdWrite ( 0x07, KEYCON );
									apple=1;									
								}
							}						
					}
				}
			}
		}
		*/
		
		/* step 2*/
		/*
		if ( ( ucStatusReturn = PcdRequest ( PICC_REQALL, pTagType ) ) == MI_OK ){
			if ( PcdAnticoll ( ucArray_ID ) == MI_OK ){
				if(pTagType[0]==0x04){
					if(PcdSelect(ucArray_ID)== MI_OK ){	
							if(PcdAuthState ( 0x61, 0x05, KEY, ucArray_ID )== MI_OK){	
												apple=2;
												PcdRead ( 0x05, recv_buf);
												if(*recv_buf != 0xaa){
													PcdWrite ( 0x05, ipmessage );
													apple=3;
												}
							}						
					}
				}
			}
		}	
		*/
	
		
		HAL_Delay(500);

		
    /* USER CODE BEGIN 3 */
		
  }
  /* USER CODE END 3 */
}


/*
uint8_t rc522_read(int fd,void*buf,int count)
{
    //读内容
    uint8_t ucStatusReturn;
    uint8_t ucArray_ID[4];//存放序列号
    uint8_t pTagType[2];//卡类
    ucStatusReturn = PcdRequest ( PICC_REQALL, pTagType );
    if ( ucStatusReturn!= MI_OK ){
        return ucStatusReturn;
    }
    else if(ucStatusReturn== MI_OK){
        if(*pTagType!=0x04){ //不是M1卡
            return MI_ERR;
        }
        if ( PcdAnticoll ( ucArray_ID ) == MI_OK ){
            if(PcdSelect(ucArray_ID)== MI_OK ){
                if((PcdAuthState ( 0x61, RECORD_ADDR, rc522_KeyB, ucArray_ID )== MI_OK)&&(count==0)){
                    PcdRead ( RECORD_ADDR, buf);
                }
            }
        }
				return MI_OK; 
    }
		return MI_OK; 
}
*/
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
