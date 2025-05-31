#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "Delay.h"
#include "Usart.h"
#include "Lcd.h"
#include "Sdio.h"
#include "Key.h"
#include "MemManager.h"
#include "Fatfs.h"

uint8_t ByteData[512];

int fputc(int ch, FILE *p) {
	USART_SendData(USART2, (uint8_t)ch);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);

	if((ch <= '~') && (ch >= ' ')) {	//判断是不是控制字符       
		LCD_ShowChar(LCD_FputcInfo.x, LCD_FputcInfo.y, (uint8_t)ch, LCD_FputcInfo.size, LCD_FputcInfo.mode);
		LCD_FputcInfo.x += LCD_FputcInfo.size / 2;
	} else if(ch == 10) {							//换行符
		LCD_FputcInfo.y += LCD_FputcInfo.size;
		LCD_FputcInfo.x = 0;
	}
	return ch;
}

void ShowHex(uint8_t *bData, uint16_t size, uint16_t displaySize, bool title, bool columnCount, bool ascii) {
	bool moreFlag = false;
	if(displaySize != 0 && size > displaySize) {
		size = displaySize;
		moreFlag = true;
	}

	unsigned int i = 0, j = 0;
	if(title) {
		if (columnCount)
			printf("---- > ");
    printf("0  1  2  3  4  5  6  7    8  9  A  B  C  D  E  F\r\n");
	}

	for(i = 0; i < size; i++) {
		if(i % 16 == 0 && i != 0) {
				if(ascii) {
					printf("| ");
					for(j = i - 16; j < i; j++) {
						if(isprint(bData[j]))
							printf("%c", bData[j]);
						else
							printf(" ");
					}
				}
				printf("\r\n");
			} else if(i % 8 == 0 && i != 0)
				printf("- ");

		if(i % 16 == 0 && columnCount)
			printf("%04d > ", i);

		printf("%02X ", (unsigned char)bData[i]);
	}

	if(ascii) {
		if (i % 16 != 0) {
			j = i;
			i = i + 16 - (i % 16);
			for(; j < i; j++) {
				if(j % 8 == 0 && j != 0)
				printf("  ");
				printf("   ");
			}
		}

		printf("| ");
		for(j = i - 16; j < size; j++) {
			if(isprint(bData[j]))
				printf("%c", bData[j]);
			else
				printf(" ");
		}
	}

	printf("\r\n");
	if(moreFlag) printf("...more...\r\n");
}

int main() {
	uint16_t page = 1, new_page = 0;
	uint16_t count = 0;

	SD_Error SD_errorstatus;
	bool Init_Error = false;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	InitUsart();
	InitLcd();
	printf("Init USART and LCD OK\n");

	SD_errorstatus = InitSdio();
	if(SD_errorstatus != SD_OK){
		printf("Init SDIO Error : %d\n", SD_errorstatus);
		Init_Error = true;
	}
	else printf("Init SDIO OK\n");

	InitKey();
	printf("Init Key OK\n");
	
	printf("Init Done\n");
	while(Init_Error);

//	uint8_t* Test = (uint8_t*)MemManager_Malloc(MEMSRAM, 2048);

//	printf("0x%p\n", Test);

//	for(count = 0; count < 2048; count++)
//			Test[count] = (uint8_t)count;

//	SD_WriteMultiBlocks(ByteData, 0, 512, 4);
//	SD_WriteBlock(ByteData, 0, 512);	

	while(1) {
		Key_Scan();
		if(Key_Value != KeyNone) {
			switch(Key_Value) {
			case Key0:
				new_page+=1;
				break;
			case Key1:
				if(new_page > 0) new_page-=1;
				break;
			case Key2:
				printf("SD_SendStatus() = 0x%02X\n",SD_GetStatus());
				break;
			}
			while(Key_Value != KeyNone) Key_Scan();
		}

		if(page != new_page) {
			LCD_Clear(WHITE);
			LCD_SetFputcLocation(0, 0, 12, 0);

			page = new_page;
			printf("Page = %d\n", page);			

//			SD_errorstatus = SD_ReadMultiBlocks(ByteData, page, 512, 4);
			SD_errorstatus = SD_ReadBlock(ByteData, page, 512); 		
			ShowHex(ByteData, 512, 512, true, true, true);

			printf("SD_errorstatus = 0x%02X\n", SD_errorstatus);
			printf("Done\n");
		}
	}
}
