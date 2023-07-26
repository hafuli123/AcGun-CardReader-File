#ifndef __RFID_H__
#define __RFID_H__

#include "main.h"

void RC522_Init(void);
uint8_t EntranceGuard(uint8_t *readUid, void (*funCallBack)(void));

#endif
