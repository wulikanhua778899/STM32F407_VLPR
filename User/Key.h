#ifndef _KEY_H
#define _KEY_H

#include <stdint.h>

#define KeyNone ((uint8_t)0x0)
#define KeyUp		((uint8_t)0x1)
#define Key0		((uint8_t)0x2)
#define Key1		((uint8_t)0x4)
#define Key2		((uint8_t)0x8)

extern uint8_t Key_Value;

void InitKey(void);
void Key_Scan(void);

#endif
