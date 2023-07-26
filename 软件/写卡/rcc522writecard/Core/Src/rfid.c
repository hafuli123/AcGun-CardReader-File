#include "rfid.h"
#include "rc522.h"


uint8_t readUid[5];
uint8_t UID[5] = {0x37, 0x7e, 0xbc, 0xfd};							// �Զ���Ŀ��ţ����ڱȽ�
uint8_t DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Ĭ����Կ

void RC522_Init(void)
{
	MFRC_Init();
	PCD_Reset();	
}

//�Ž����� ��ȡ ��ʾ����
uint8_t EntranceGuard(uint8_t *readUid, void (*funCallBack)(void))
{
	uint8_t Temp[5];												 // ���IC�������ͺ�UID(IC�����к�)
	if (PCD_Request(PICC_REQALL, Temp) == PCD_OK) // Ѱ��
	{
		if (PCD_Anticoll(readUid) == PCD_OK) // ����ײ,��ȡ���ţ�����readUid
		{									 // ����ײ�ɹ�
			if (funCallBack != NULL)
				funCallBack(); // ���ù���ִ�к�������ָʾ���ź�
			return 0;
		}

	}
}