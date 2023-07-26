/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: RC522��������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */  
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "rc522_config.h"
#include "stm32f4xx_hal.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
SPI_HandleTypeDef RC522spi;
/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/
/**
  * ��������: RC522��ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SPIx_Init(void)
{

  RC522spi.Instance = macRC522_SPIx;
  RC522spi.Init.Mode = SPI_MODE_MASTER;
  RC522spi.Init.Direction = SPI_DIRECTION_2LINES;
  RC522spi.Init.DataSize = SPI_DATASIZE_8BIT;
  RC522spi.Init.CLKPolarity = SPI_POLARITY_LOW;
  RC522spi.Init.CLKPhase = SPI_PHASE_1EDGE;
  RC522spi.Init.NSS = SPI_NSS_SOFT;
  RC522spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  RC522spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  RC522spi.Init.TIMode = SPI_TIMODE_DISABLE;
  RC522spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  RC522spi.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&RC522spi);
  __HAL_SPI_ENABLE(&RC522spi);

  macRC522_Reset_Disable();
	macRC522_CS_Disable();  
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hspi->Instance==macRC522_SPIx)
  {  
    /* SPI����ʱ��ʹ�� */
    macRC522_SPIx_RCC_CLK_ENABLE();
    /* GPIO����ʱ��ʹ�� */
    macRC522_CS_RCC_CLK_ENABLE();
    macRC522_SCK_RCC_CLK_ENABLE();   
    macRC522_M0S_RCC_CLK_ENABLE();
    macRC522_RST_RCC_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = macRC522_GPIO_CS_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(macRC522_GPIO_CS_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = macRC522_GPIO_SCK_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(macRC522_GPIO_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = macRC522_GPIO_MOSI_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;    
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(macRC522_GPIO_MOSI_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = macRC522_GPIO_MISO_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;    
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(macRC522_GPIO_MOSI_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = macRC522_GPIO_RST_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(macRC522_GPIO_RST_PORT, &GPIO_InitStruct);

  }

}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/


