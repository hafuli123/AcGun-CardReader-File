#include "rc522.h"
#include "spi.h"

#define RCC522_RST(N)	HAL_GPIO_WritePin(RCC522_RST_GPIO_Port,RCC522_RST_Pin,N == 1 ? GPIO_PIN_SET:GPIO_PIN_RESET)
#define RCC522_NSS(N)	HAL_GPIO_WritePin(RCC522_NSS_GPIO_Port,RCC522_NSS_Pin,N==1?GPIO_PIN_SET:GPIO_PIN_RESET)


void delay_ms(uint16_t ms)
{
	uint32_t temp;
	SysTick->LOAD = 100000000/1000/8*ms;
	SysTick->VAL=0X00;//��ռ�����
	SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ

  do
  {
    temp=SysTick->CTRL;//��ȡ��ǰ������ֵ
  }while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��
  SysTick->CTRL=0x00; //�رռ�����
  SysTick->VAL =0X00; //��ռ�����
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
	data = SPI2_RW_Byte(0x00); //0xff���ƶ�������
	RCC522_NSS(1);
	return data;
}

//���üĴ�����λ
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

//����CRC
void MFRC_CalulateCRC(uint8_t *pInData,uint8_t len,uint8_t *pOutData)
{
	uint8_t temp;
	uint32_t i;
	
	MFRC_ClrBitMask(MFRC_DivIrqReg,0x04); //ʹ��CRC�ж�
	MFRC_WriteReg(MFRC_CommandReg,MFRC_IDLE); //ȡ����ǰ����ִ��
	MFRC_SetBitMask(MFRC_FIFOLevelReg,0x80); //���FIFO�ͱ�־λ
	for(i=0;i<len;i++){	//��CRC���������д��FIFO
		MFRC_WriteReg(MFRC_FIFODataReg,*(pInData+i));
	}
	MFRC_WriteReg(MFRC_CommandReg,MFRC_CALCCRC);//ִ��CRC����
	i=100000;
	do{
		temp=MFRC_ReadReg(MFRC_DivIrqReg);
		i--;
	}while ((i != 0) && !(temp & 0x04)); 	//�ȴ��������
	
	pOutData[0]=MFRC_ReadReg(MFRC_CRCResultRegL);
	pOutData[1]=MFRC_ReadReg(MFRC_CRCResultRegM);
}


/**************************************************************************************
 * �������ƣ�MFRC_CmdFrame
 * ����������MFRC522��ISO14443A��ͨѶ������֡����
 * ��ڲ�����-cmd��MFRC522������
 *           -pIndata��MFRC522���͸�MF1�������ݵĻ������׵�ַ
 *           -InLenByte���������ݵ��ֽڳ���
 *           -pOutdata�����ڽ���MF1��Ƭ�������ݵĻ������׵�ַ
 *           -pOutLenBit��MF1���������ݵ�λ����
 * ���ڲ�����-pOutdata�����ڽ���MF1��Ƭ�������ݵĻ������׵�ַ
 *           -pOutLenBit������MF1����������λ���ȵ��׵�ַ
 * �� �� ֵ��-status���������(MFRC_OK��MFRC_NOTAGERR��MFRC_ERR)
 *************************************************************/

char MFRC_CmdFrame(uint8_t cmd, uint8_t *pInData, uint8_t InLenByte, uint8_t *pOutData, uint16_t *pOutLenBit)
{
    uint8_t lastBits;
    uint8_t n;
    uint32_t i;
    char status = MFRC_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitFor = 0x00;
 
    /*�����������ñ�־λ*/
    switch (cmd)
    {
    case MFRC_AUTHENT: // Mifare��֤
        irqEn = 0x12;
        waitFor = 0x10; // idleIRq�жϱ�־
        break;
    case MFRC_TRANSCEIVE: // ���Ͳ���������
        irqEn = 0x77;
        waitFor = 0x30; // RxIRq��idleIRq�жϱ�־
        break;
    }
 
    /*��������֡ǰ׼��*/
    MFRC_WriteReg(MFRC_ComIEnReg, irqEn | 0x80); // ���ж�
    MFRC_ClrBitMask(MFRC_ComIrqReg, 0x80);       // ����жϱ�־λSET1
    MFRC_WriteReg(MFRC_CommandReg, MFRC_IDLE);   // ȡ����ǰ�����ִ��
    MFRC_SetBitMask(MFRC_FIFOLevelReg, 0x80);    // ���FIFO�����������־λ
 
    /*��������֡*/
    for (i = 0; i < InLenByte; i++) // д���������
    {
        MFRC_WriteReg(MFRC_FIFODataReg, pInData[i]); // д���ݽ� FIFODataReg
    }
    MFRC_WriteReg(MFRC_CommandReg, cmd); // ִ������
    if (cmd == MFRC_TRANSCEIVE)
    {
        MFRC_SetBitMask(MFRC_BitFramingReg, 0x80); // ��������
    }
    i = 10000; // ����ʱ��Ƶ�ʵ���,����M1�����ȴ�ʱ��25ms
    do          // ��֤ ��Ѱ���ȴ�ʱ��
    {
        n = MFRC_ReadReg(MFRC_ComIrqReg); // ��ѯ�¼��ж�
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitFor)); // �ȴ��������
    MFRC_ClrBitMask(MFRC_BitFramingReg, 0x80);           // ֹͣ����
 
    /*������յ�����*/
    if (i != 0)
    {
        // �������־�Ĵ���BufferOfI CollErr ParityErr ProtocolErr
        if (!(MFRC_ReadReg(MFRC_ErrorReg) & 0x1B))
        {
            status = MFRC_OK;
            if (n & irqEn & 0x01) // �Ƿ�����ʱ���ж�
            {
                status = MFRC_NOTAGERR;
            }
            if (cmd == MFRC_TRANSCEIVE)
            {
                // ��FIFO�б�����ֽ���
                n = MFRC_ReadReg(MFRC_FIFOLevelReg);
                lastBits = MFRC_ReadReg(MFRC_ControlReg) & 0x07; // �����յ����ֽڵ���Чλ��
                if (lastBits)
                {
                    *pOutLenBit = (n - 1) * 8 + lastBits; // N���ֽ�����ȥ1�����һ���ֽڣ�+���һλ��λ�� ��ȡ����������λ��
                }
                else
                {
                    *pOutLenBit = n * 8; // �����յ����ֽ������ֽ���Ч
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
 
    MFRC_SetBitMask(MFRC_ControlReg, 0x80);    // ֹͣ��ʱ������
    MFRC_WriteReg(MFRC_CommandReg, MFRC_IDLE); // ȡ����ǰ�����ִ��
 
    return status;
}



//PCD��λ
void PCD_Reset(void)
{
	/* Ӳ��λ */
	RCC522_RST(1);
	delay_ms(2);
	RCC522_RST(0);
	delay_ms(2);
	RCC522_RST(1);
	delay_ms(2);
	
	/* ��λ */
	MFRC_WriteReg(MFRC_CommandReg,MFRC_RESETPHASE);
	delay_ms(2);
	
	/*��λ��ĳ�ʼ������*/
	MFRC_WriteReg(MFRC_ModeReg, 0x3D);   // CRC��ʼֵ0x6363
	MFRC_WriteReg(MFRC_TReloadRegL, 30); // ��ʱ����װֵ
	MFRC_WriteReg(MFRC_TReloadRegH, 0);
	MFRC_WriteReg(MFRC_TModeReg, 0x8D);      // �����ڲ���ʱ��������
	MFRC_WriteReg(MFRC_TPrescalerReg, 0x3E); // ���ö�ʱ��Ԥ��Ƶֵ
	MFRC_WriteReg(MFRC_TxAutoReg, 0x40);     // ���Ʒ����ź�Ϊ100%ASK
	

	PCD_AntennaOff(); // ������ 
	delay_ms(2);
	PCD_AntennaOn(); // ������		 
}

//��������
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

//PCD��д����ʼ��
void PCD_Init(void)
{
	MFRC_Init();
	PCD_Reset();
	PCD_AntennaOff(); // �ر�����
	PCD_AntennaOn();  // ��������	
}

//Ѱ�� �õ�������
char PCD_Request(uint8_t RequestMode,uint8_t *pCardType)
{
	int status;
	uint16_t unLen;
	uint8_t CmdFrameBuf[MFRC_MAXRLEN];
	
	MFRC_ClrBitMask(MFRC_Status2Reg,0x08); //�ر��ڲ��¶ȴ�����
	MFRC_WriteReg(MFRC_BitFramingReg,0x07); 
	MFRC_SetBitMask(MFRC_TxControlReg, 0x03); // ���õ����ź�13.56MHZ
	
	CmdFrameBuf[0] = RequestMode; // ���� ��Ƭ������
	
	status = MFRC_CmdFrame(MFRC_TRANSCEIVE,CmdFrameBuf,1,CmdFrameBuf,&unLen);
	if ((status == PCD_OK) && (unLen == 0x10)){ // Ѱ���ɹ����ؿ����� 
		*pCardType = CmdFrameBuf[0];
		*(pCardType + 1) = CmdFrameBuf[1];
	}
		
	return status;
}

//����ײ ��ȡ���� pSnr�����ڱ��濨Ƭ���к�,4�ֽ�
char PCD_Anticoll(uint8_t *pSnr)
{
	char status;
	uint8_t i,snr_check=0;
	uint16_t unLen;
	uint8_t CmdFrameBuf[MFRC_MAXRLEN];
	
	MFRC_ClrBitMask(MFRC_Status2Reg, 0x08);  // ��MFCryptol Onλ ֻ�гɹ�ִ��MFAuthent����󣬸�λ������λ
	MFRC_WriteReg(MFRC_BitFramingReg, 0x00); // ����Ĵ��� ֹͣ�շ�
	MFRC_ClrBitMask(MFRC_CollReg, 0x80);     // ��ValuesAfterColl���н��յ�λ�ڳ�ͻ�����
	
	CmdFrameBuf[0] = PICC_ANTICOLL1; // ��Ƭ����ͻ����
	CmdFrameBuf[1] = 0x20;
	
	status = MFRC_CmdFrame(MFRC_TRANSCEIVE, CmdFrameBuf, 2, CmdFrameBuf, &unLen); // �뿨Ƭͨ��
	

    if (status == PCD_OK) // ͨ�ųɹ�
    {
        for (i = 0; i < 4; i++)
        {
            *(pSnr + i) = CmdFrameBuf[i]; // ����UID
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

//ѡ����Ƭ
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

 
//��֤��Ƭ���� -pKey������ -BlockAddr�����ַ(0~63) -pSnr����Ƭ���к�,4�ֽ�
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
 * �������ƣ�PCD_WriteBlock
 * ����������дMF1�����ݿ�
 * ��ڲ�����-BlockAddr�����ַ��M1���ܹ���16������(ÿ�������У�3�����ݿ�+1�����ƿ�),��64����
 *           -pData: ���ڱ����д�������,16�ֽ�
 * ���ڲ�������
 * �� �� ֵ��-status���������(PCD_OK��PCD_NOTAGERR��PCD_ERR)
 * ˵    ������
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

//����
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

//��MF1�����ݿ�����ֵ����
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
