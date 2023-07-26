#include "rfid.h"
#include "rc522.h"


uint8_t readUid[5];
uint8_t UID[5] = {0x37, 0x7e, 0xbc, 0xfd};							// 自定义的卡号，用于比较
uint8_t DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 默认秘钥

void RC522_Init(void)
{
	MFRC_Init();
	PCD_Reset();	
}

//门禁开门 读取 显示卡号
uint8_t EntranceGuard(uint8_t *readUid, void (*funCallBack)(void))
{
	uint8_t Temp[5];												 // 存放IC卡的类型和UID(IC卡序列号)
	if (PCD_Request(PICC_REQALL, Temp) == PCD_OK) // 寻卡
	{
		if (PCD_Anticoll(readUid) == PCD_OK) // 防冲撞,获取卡号，存入readUid
		{									 // 防冲撞成功
			if (funCallBack != NULL)
				funCallBack(); // 调用功能执行函数，如指示灯信号
			return 0;
		}

	}
}