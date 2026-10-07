#ifndef __KEY_APP_H
#define __KEY_APP_H

#include "main.h"

#define KEY_APP_KEY1   0
#define KEY_APP_KEY2   1
#define KEY_APP_KEY3   2

void KEY_AppInit(void);
void KEY_AppScan(void);
uint8_t KEY_AppGetSingleClick(uint8_t keyIndex);
uint8_t KEY_AppGetDoubleClick(uint8_t keyIndex);

#endif
