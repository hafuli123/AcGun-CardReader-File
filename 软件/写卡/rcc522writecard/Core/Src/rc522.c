#include "rc522.h"
#include "spi.h"

#define RCC522_RST(N)	HAL_GPIO_WritePin(RCC522_RST_GPIO_Port,RCC522_RST_Pin,N == 1 ? GPIO_PIN_SET:GPIO_PIN_RESET)
#define RCC522_NSS(N)	HAL_GPIO_WritePin(RCC522_NSS_GPIO_Port,RCC522_NSS_Pin,N==1?GPIO_PIN_SET:GPIO_PIN_RESET)


void delay_ms(uint16_t ms)
{
	uint32_t temp;
	SysTick->LOAD = 100000000/1000/8*ms;
	SysTick->VAL=0X00;//清空计数器
	SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源

  do
  {
    temp=SysTick->CTRL;//读取当前倒计数值
  }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
  SysTick->CTRL=0x00; //关闭计数器
  SysTick->VAL =0X00; //清空计数器
}

void MFRC_Init(void)
{
	RCC522_NSS(1);
	RCC522_RST(1);
}

uint8_t SPI2_RW_Byte(uint8_t byte)
{
	uint8_t ret;
	HAL_SPI_TransmitReceive(&hspi1,&byte,&ret,1,10);
	return ret;
}

void MFRC_WriteReg(uint8_t addr,uint8_t data)
{
	uint8_t AddrByte;
	AddrByte = (addr<<1)&0x7e;
	RCC522_NSS(0);
	SPI2_RW_Byte(AddrByte);
	SPI2_RW_Byte(data);
	RCC522_NSS(1);
}

uint8_t MFRC_ReadReg(uint8_t addr)
{
	uint8_t AddrByte,data;
	AddrByte = ((addr<<1)&0x7e)|0x80;
	RCC522_NSS(0);
	SPI2_RW_Byte(AddrByte);
	data = SPI2_RW_Byte(0x00); //0xff类似读个东西
	RCC522_NSS(1);
	return data;
}

//设置寄存器的位
void MFRC_SetBitMask(uint8_t addr,uint8_t mask)
{
	uint8_t temp;
	temp=MFRC_ReadReg(addr);
	MFRC_WriteReg(addr,temp|mask);
}

void MFRC_ClrBitMask(uint8_t addr,uint8_t mask)
{
	uint8_t temp;
	temp=MFRC_ReadReg(addr);
	MFRC_WriteReg(addr,temp& ~mask);
}

//计算CRC
void MFRC_CalulateCRC(uint8_t *pInData,uint8_t len,uint8_t *pOutData)
{
	uint8_t temp;
	uint32_t i;
	
	MFRC_ClrBitMask(MFRC_DivIrqReg,0x04); //使能CRC中断
	MFRC_WriteReg(MFRC_CommandReg,MFRC_IDLE); //取消当前命令执行
	MFRC_SetBitMask(MFRC_FIFOLevelReg,0x80); //清除FIFO和标志位
	for(i=0;i<len;i++){	//将CRC计算的数据写入FIFO
		MFRC_WriteReg(MFRC_FIFODataReg,*(pInData+i));
	}
	MFRC_WriteReg(MFRC_CommandReg,MFRC_CALCCRC);//执行CRC计算
	i=100000;
	do{
		temp=MFRC_ReadReg(MFRC_DivIrqReg);
		i--;
	}while ((i != 0) && !(temp & 0x04)); 	//等待计算完成
	
	pOutData[0]=MFRC_ReadReg(MFRC_CRCResultRegL);
	pOutData[1]=MFRC_ReadReg(MFRC_CRCResultRegM);
}


/**************************************************************************************
 * 函数名称：MFRC_CmdFrame
 * 功能描述：MFRC522和ISO14443A卡通讯的命令帧函数
 * 入口参数：-cmd：MFRC522命令字
 *           -pIndata：MFRC522发送给MF1卡的数据的缓冲区首地址
 *           -InLenByte：发送数据的字节长度
 *           -pOutdata：用于接收MF1卡片返回数据的缓冲区首地址
 *           -pOutLenBit：MF1卡返回数据的位长度
 * 出口参数：-pOutdata：用于接收MF1卡片返回数据的缓冲区首地址
 *           -pOutLenBit：用于MF1卡返回数据位长度的首地址
 * 返 回 值：-status：错误代码(MFRC_OK、MFRC_NOTAGERR、MFRC_ERR)
 *************************************************************/

char MFRC_CmdFrame(uint8_t cmd, uint8_t *pInData, uint8_t InLenByte, uint8_t *pOutData, uint16_t *pOutLenBit)
{
    uint8_t lastBits;
    uint8_t n;
    uint32_t i;
    char status = MFRC_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitFor = 0x00;
 
    /*根据命令设置标志位*/
    switch (cmd)
    {
    case MFRC_AUTHENT: // Mifare认证
        irqEn = 0x12;
        waitFor = 0x10; // idleIRq中断标志
        break;
    case MFRC_TRANSCEIVE: // 发送并接收数据
        irqEn = 0x77;
        waitFor = 0x30; // RxIRq和idleIRq中断标志
        break;
    }
 
    /*发送命令帧前准备*/
    MFRC_WriteReg(MFRC_ComIEnReg, irqEn | 0x80); // 开中断
    MFRC_ClrBitMask(MFRC_ComIrqReg, 0x80);       // 清除中断标志位SET1
    MFRC_WriteReg(MFRC_CommandReg, MFRC_IDLE);   // 取消当前命令的执行
    MFRC_SetBitMask(MFRC_FIFOLevelReg, 0x80);    // 清除FIFO缓冲区及其标志位
 
    /*发送命令帧*/
    for (i = 0; i < InLenByte; i++) // 写入命令参数
    {
        MFRC_WriteReg(MFRC_FIFODataReg, pInData[i]); // 写数据进 FIFODataReg
    }
    MFRC_WriteReg(MFRC_CommandReg, cmd); // 执行命令
    if (cmd == MFRC_TRANSCEIVE)
    {
        MFRC_SetBitMask(MFRC_BitFramingReg, 0x80); // 启动发送
    }
    i = 10000; // 根据时钟频率调整,操作M1卡最大等待时间25ms
    do          // 认证 与寻卡等待时间
    {
        n = MFRC_ReadReg(MFRC_ComIrqReg); // 查询事件中断
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitFor)); // 等待命令完成
    MFRC_ClrBitMask(MFRC_BitFramingReg, 0x80);           // 停止发送
 
    /*处理接收的数据*/
    if (i != 0)
    {
        // 读错误标志寄存器BufferOfI CollErr ParityErr ProtocolErr
        if (!(MFRC_ReadReg(MFRC_ErrorReg) & 0x1B))
        {
            status = MFRC_OK;
            if (n & irqEn & 0x01) // 是否发生定时器中断
            {
                status = MFRC_NOTAGERR;
            }
            if (cmd == MFRC_TRANSCEIVE)
            {
                // 读FIFO中保存的字节数
                n = MFRC_ReadReg(MFRC_FIFOLevelReg);
                lastBits = MFRC_ReadReg(MFRC_ControlReg) & 0x07; // 最后接收到得字节的有效位数
                if (lastBits)
                {
                    *pOutLenBit = (n - 1) * 8 + lastBits; // N个字节数减去1（最后一个字节）+最后一位的位数 读取到的数据总位数
                }
                else
                {
                    *pOutLenBit = n * 8; // 最后接收到的字节整个字节有效
                }
                if (n == 0)
                {
                    n = 1;
                }
                if (n > MFRC_MAXRLEN)
                {
                    n = MFRC_MAXRLEN;
                }
                for (i = 0; i < n; i++)
                {
                    pOutData[i] = MFRC_ReadReg(MFRC_FIFODataReg);
                }
            }
        }
        else
        {
            status = MFRC_ERR;
        }
    }
 
    MFRC_SetBitMask(MFRC_ControlReg, 0x80);    // 停止定时器运行
    MFRC_WriteReg(MFRC_CommandReg, MFRC_IDLE); // 取消当前命令的执行
 
    return status;
}



//PCD复位
void PCD_Reset(void)
{
	/* 硬复位 */
	RCC522_RST(1);
	delay_ms(2);
	RCC522_RST(0);
	delay_ms(2);
	RCC522_RST(1);
	delay_ms(2);
	
	/* 软复位 */
	MFRC_WriteReg(MFRC_CommandReg,MFRC_RESETPHASE);
	delay_ms(2);
	
	/*复位后的初始化配置*/
	MFRC_WriteReg(MFRC_ModeReg, 0x3D);   // CRC初始值0x6363
	MFRC_WriteReg(MFRC_TReloadRegL, 30); // 定时器重装值
	MFRC_WriteReg(MFRC_TReloadRegH, 0);
	MFRC_WriteReg(MFRC_TModeReg, 0x8D);      // 定义内部定时器的设置
	MFRC_WriteReg(MFRC_TPrescalerReg, 0x3E); // 设置定时器预分频值
	MFRC_WriteReg(MFRC_TxAutoReg, 0x40);     // 调制发送信号为100%ASK
	

	PCD_AntennaOff(); // 关天线 
	delay_ms(2);
	PCD_AntennaOn(); // 开天线		 
}

//开关天线
void PCD_AntennaOn(void)
{
	uint8_t temp;
	temp = MFRC_ReadReg(MFRC_TxControlReg);
	if(!(temp&0x03)){
		MFRC_SetBitMask(MFRC_TxControlReg, 0x03); 
	}
}
void PCD_AntennaOff(void)
{
	MFRC_ClrBitMask(MFRC_TxControlReg,0x03);
}

//PCD读写器初始化
void PCD_Init(void)
{
	MFRC_Init();
	PCD_Reset();
	PCD_AntennaOff(); // 关闭天线
	PCD_AntennaOn();  // 开启天线	
}

//寻卡 得到卡类型
char PCD_Request(uint8_t RequestMode,uint8_t *pCardType)
{
	int status;
	uint16_t unLen;
	uint8_t CmdFrameBuf[MFRC_MAXRLEN];
	
	MFRC_ClrBitMask(MFRC_Status2Reg,0x08); //关闭内部温度传感器
	MFRC_WriteReg(MFRC_BitFramingReg,0x07); 
	MFRC_SetBitMask(MFRC_TxControlReg, 0x03); // 配置调制信号13.56MHZ
	
	CmdFrameBuf[0] = RequestMode; // 存入 卡片命令字
	
	status = MFRC_CmdFrame(MFRC_TRANSCEIVE,CmdFrameBuf,1,CmdFrameBuf,&unLen);
	if ((status == PCD_OK) && (unLen == 0x10)){ // 寻卡成功返回卡类型 
		*pCardType = CmdFrameBuf[0];
		*(pCardType + 1) = CmdFrameBuf[1];
	}
		
	return status;
}

//防冲撞 获取卡号 pSnr：用于保存卡片序列号,4字节
char PCD_Anticoll(uint8_t *pSnr)
{
	char status;
	uint8_t i,snr_check=0;
	uint16_t unLen;
	uint8_t CmdFrameBuf[MFRC_MAXRLEN];
	
	MFRC_ClrBitMask(MFRC_Status2Reg, 0x08);  // 清MFCryptol On位 只有成功执行MFAuthent命令后，该位才能置位
	MFRC_WriteReg(MFRC_BitFramingReg, 0x00); // 清理寄存器 停止收发
	MFRC_ClrBitMask(MFRC_CollReg, 0x80);     // 清ValuesAfterColl所有接收的位在冲突后被清除
	
	CmdFrameBuf[0] = PICC_ANTICOLL1; // 卡片防冲突命令
	CmdFrameBuf[1] = 0x20;
	
	status = MFRC_CmdFrame(MFRC_TRANSCEIVE, CmdFrameBuf, 2, CmdFrameBuf, &unLen); // 与卡片通信
	

    if (status == PCD_OK) // 通信成功
    {
        for (i = 0; i < 4; i++)
        {
            *(pSnr + i) = CmdFrameBuf[i]; // 读出UID
            snr_check ^= CmdFrameBuf[i];
        }
        if (snr_check != CmdFrameBuf[i])
        {
            status = PCD_ERR;
        }
    }
		
    MFRC_SetBitMask(MFRC_CollReg, 0x80);
		return status;
}

//选定卡片
char PCD_Select(uint8_t *pSnr)
{
	char status;
	uint8_t i;
	uint16_t unLen;
	uint8_t CmdFrameBuf[MFRC_MAXRLEN];
	
	CmdFrameBuf[0] = PICC_ANTICOLL1;
	CmdFrameBuf[1] = 0x70;	
	CmdFrameBuf[6] = 0;
	for (i = 0; i < 4; i++){
		CmdFrameBuf[i + 2] = *(pSnr + i);
		CmdFrameBuf[6] ^= *(pSnr + i);
	}
	MFRC_CalulateCRC(CmdFrameBuf, 7, &CmdFrameBuf[7]);
	
	MFRC_ClrBitMask(MFRC_Status2Reg, 0x08);
	status = MFRC_CmdFrame(MFRC_TRANSCEIVE, CmdFrameBuf, 9, CmdFrameBuf, &unLen);	
	

	if ((status == PCD_OK) && (unLen == 0x18))
	{
		status = PCD_OK;
	}
	else
	{
		status = PCD_ERR;
	}
	return status;
}

 
//验证卡片密码 -pKey：密码 -BlockAddr：块地址(0~63) -pSnr：卡片序列号,4字节
char PCD_AuthState(uint8_t AuthMode, uint8_t BlockAddr, uint8_t *pKey, uint8_t *pSnr)
{
	char status;
	uint16_t unLen;
	uint8_t i, CmdFrameBuf[MFRC_MAXRLEN];
	
	CmdFrameBuf[0] = AuthMode;
	CmdFrameBuf[1] = BlockAddr;

	for (i = 0; i < 6; i++)
	{
		CmdFrameBuf[i + 2] = *(pKey + i);
	}
	for (i = 0; i < 4; i++)
	{
		CmdFrameBuf[i + 8] = *(pSnr + i);
	}

	status = MFRC_CmdFrame(MFRC_AUTHENT, CmdFrameBuf, 12, CmdFrameBuf, &unLen);

    if ((status != PCD_OK) || (!(MFRC_ReadReg(MFRC_Status2Reg) & 0x08)))
    {
        status = PCD_ERR;
    }

		return status;
}



/***************************************************************************************
 * 函数名称：PCD_WriteBlock
 * 功能描述：写MF1卡数据块
 * 入口参数：-BlockAddr：块地址。M1卡总共有16个扇区(每个扇区有：3个数据块+1个控制块),共64个块
 *           -pData: 用于保存待写入的数据,16字节
 * 出口参数：无
 * 返 回 值：-status：错误代码(PCD_OK、PCD_NOTAGERR、PCD_ERR)
 * 说    明：无
 ***************************************************************************************/
char PCD_WriteBlock(uint8_t BlockAddr, uint8_t *pData)
{
    char status;
    uint16_t unLen;
    uint8_t i, CmdFrameBuf[MFRC_MAXRLEN];

    CmdFrameBuf[0] = PICC_WRITE;
    CmdFrameBuf[1] = BlockAddr;
    MFRC_CalulateCRC(CmdFrameBuf, 2, &CmdFrameBuf[2]);
	
		status = MFRC_CmdFrame(MFRC_TRANSCEIVE, CmdFrameBuf, 4, CmdFrameBuf, &unLen);

    if ((status != PCD_OK) || (unLen != 4) || ((CmdFrameBuf[0] & 0x0F) != 0x0A))
    {
        status = PCD_ERR;
    }

    if (status == PCD_OK){
        for (i = 0; i < 16; i++)
        {
            CmdFrameBuf[i] = *(pData + i);
        }
			MFRC_CalulateCRC(CmdFrameBuf, 16, &CmdFrameBuf[16]);
			status = MFRC_CmdFrame(MFRC_TRANSCEIVE, CmdFrameBuf, 18, CmdFrameBuf, &unLen);
 
        if ((status != PCD_OK) || (unLen != 4) || ((CmdFrameBuf[0] & 0x0F) != 0x0A))
        {
            status = PCD_ERR;
        }							
		}		
	return status;		
}

//读卡
char PCD_ReadBlock(uint8_t BlockAddr, uint8_t *pData)
{
    char status;
    uint16_t unLen;
    uint8_t i, CmdFrameBuf[MFRC_MAXRLEN];


    CmdFrameBuf[0] = PICC_READ;
    CmdFrameBuf[1] = BlockAddr;
    MFRC_CalulateCRC(CmdFrameBuf, 2, &CmdFrameBuf[2]);

    status = MFRC_CmdFrame(MFRC_TRANSCEIVE, CmdFrameBuf, 4, CmdFrameBuf, &unLen);

    if ((status == PCD_OK) && (unLen == 0x90))
    {
        for (i = 0; i < 16; i++)
        {
            *(pData + i) = CmdFrameBuf[i];
        }
    }
    else
    {
        status = PCD_ERR;
    }

		return status;
}

//对MF1卡数据块增减值操作
char PCD_Value(uint8_t mode, uint8_t BlockAddr, uint8_t *pValue)
{
    char status;
    uint16_t unLen;
    uint8_t i, CmdFrameBuf[MFRC_MAXRLEN];

    CmdFrameBuf[0] = mode;
    CmdFrameBuf[1] = BlockAddr;
    MFRC_CalulateCRC(CmdFrameBuf, 2, &CmdFrameBuf[2]);	
	
		status = MFRC_CmdFrame(MFRC_TRANSCEIVE, CmdFrameBuf, 4, CmdFrameBuf, &unLen);

    if ((status != PCD_OK) || (unLen != 4) || ((CmdFrameBuf[0] & 0x0F) != 0x0A))
    {
        status = PCD_ERR;
    }

    if (status == PCD_OK)
    {
        for (i = 0; i < 16; i++)
        {
            CmdFrameBuf[i] = *(pValue + i);
        }
        MFRC_CalulateCRC(CmdFrameBuf, 4, &CmdFrameBuf[4]);
        unLen = 0;
        status = MFRC_CmdFrame(MFRC_TRANSCEIVE, CmdFrameBuf, 6, CmdFrameBuf, &unLen);
        if (status != PCD_ERR)
        {
            status = PCD_OK;
        }
    }

    if (status == PCD_OK)
    {
        CmdFrameBuf[0] = PICC_TRANSFER;
        CmdFrameBuf[1] = BlockAddr;
        MFRC_CalulateCRC(CmdFrameBuf, 2, &CmdFrameBuf[2]);
 
        status = MFRC_CmdFrame(MFRC_TRANSCEIVE, CmdFrameBuf, 4, CmdFrameBuf, &unLen);
 
        if ((status != PCD_OK) || (unLen != 4) || ((CmdFrameBuf[0] & 0x0F) != 0x0A))
        {
            status = PCD_ERR;
        }
    }
    return status;

}
