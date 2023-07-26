#ifndef __RC522_FUNCTION_H
#define	__RC522_FUNCTION_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define          macDummy_Data              0x00

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void   PcdReset             ( void );                       //复位
void   M500PcdConfigISOType ( uint8_t type );                    //工作方式
char   PcdRequest           ( uint8_t req_code, uint8_t * pTagType ); //寻卡
char   PcdAnticoll          ( uint8_t * pSnr);                   //读卡号
char   PcdAuthState         ( uint8_t ucAuth_mode, uint8_t ucAddr, uint8_t * pKey, uint8_t * pSnr );//验证卡片密码
char   PcdRead              ( uint8_t ucAddr, uint8_t * pData );      //读卡
char   PcdSelect            ( uint8_t * pSnr );                  //选卡   
char PcdWrite ( uint8_t ucAddr, uint8_t * pData );
#endif /* __RC522_FUNCTION_H */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

