#include "Key.h"
#include "Delay.h"
#include "PreDefine.h"
#include "stm32f4xx.h"

#define Pin_Key0	GPIO_Pin_4
#define Pin_Key1	GPIO_Pin_3
#define Pin_Key2	GPIO_Pin_2
#define Pin_KeyUp	GPIO_Pin_0

#define KeyUP_Value PAin(0)
#define Key0_Value 	PEin(4)
#define Key1_Value 	PEin(3)
#define Key2_Value 	PEin(2)

uint8_t Key_Value;

void InitKey(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
	GPIO_InitStructure.GPIO_Pin = Pin_Key0 | Pin_Key1 | Pin_Key2;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = Pin_KeyUp;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	//下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Key_Scan(void) {	
	Key_Value = 0;
	
	if(KeyUP_Value == 1 || Key0_Value == 0 || Key1_Value == 0 || Key2_Value == 0) {	
		Delay_ms(2);  //消抖
		if(KeyUP_Value == 1)	Key_Value |= KeyUp; 
		if(Key0_Value == 0) 	Key_Value |= Key0; 
		if(Key1_Value == 0) 	Key_Value |= Key1; 
		if(Key2_Value == 0) 	Key_Value |= Key2; 
	}
}
