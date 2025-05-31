#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>

typedef struct {
	uint16_t LCD_CMD;
	uint16_t LCD_DATA;
}LCD_TypeDef;

typedef struct  
{										    
	uint16_t width;		//LCD 宽度
	uint16_t height;	//LCD 高度
	uint8_t id;				//LCD ID
	uint8_t  dir;			//LCD 方向
}LCD_DataTypeDef;
extern LCD_DataTypeDef LCD_Info;

typedef struct  
{
	uint16_t x;
	uint16_t y;
	uint8_t size;
	uint8_t mode;
}LCD_FputcTypeDef;
extern LCD_FputcTypeDef LCD_FputcInfo;

extern uint16_t LCD_FRONT_COLOR;   
extern uint16_t LCD_BACK_COLOR;

//使用NOR/SRAM的Bank1.sector4,地址位HADDR[27,26]=11,A6作为数据命令区分线 		    
#define LCD_BASE        ((uint32_t)(0x6C000000 | 0x0000007E))
#define LCD             ((LCD_TypeDef *) LCD_BASE)

#define WHITE     	0xFFFF
#define BLACK     	0x0000	  
#define BLUE      	0x001F
#define BRED      	0XF81F
#define GRED 				0XFFE0
#define GBLUE				0X07FF
#define RED       	0xF800
#define MAGENTA   	0xF81F
#define GREEN     	0x07E0
#define CYAN      	0x7FFF
#define YELLOW    	0xFFE0
#define BROWN 			0XBC40
#define BRRED 			0XFC07
#define GRAY  			0X8430
#define DARKBLUE  	0X01CF
#define LIGHTBLUE 	0X7D7C  
#define GRAYBLUE  	0X5458
#define LIGHTGREEN	0X841F
#define LIGHTGRAY 	0XEF5B
#define LGRAY 			0XC618
#define LGRAYBLUE 	0XA651
#define LBBLUE    	0X2B12

void LCD_SetFputcLocation(uint16_t x, uint16_t y, uint8_t size, uint8_t mode);
void LCD_Set_Window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height);
void LCD_WriteCmdData(uint16_t cmd, uint16_t data);
void LCD_Display_Dir(uint8_t dir);
void LCD_Clear(uint16_t color);
void InitLcd(void);
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size, uint8_t mode);
void LCD_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p);
void LCD_Fill(uint16_t xState, uint16_t yState, uint16_t xEnd, uint16_t yEnd, uint16_t color);
uint16_t LCD_ReadPoint(uint16_t x,uint16_t y);

#endif
