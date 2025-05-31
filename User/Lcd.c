#include <stdio.h>
#include "Lcd.h"
#include "Font.h"
#include "Delay.h"
#include "stm32f4xx.h"

LCD_DataTypeDef LCD_Info;
LCD_FputcTypeDef LCD_FputcInfo;

uint16_t LCD_FRONT_COLOR = BLACK;	//字体颜色
uint16_t LCD_BACK_COLOR = WHITE;  //背景颜色

void InitLcd_GPIO(void);
void InitLcd_FSMC(void);
void inline LCD_WriteCmd(uint16_t cmd);
void inline LCD_WriteData(uint16_t data);
uint16_t inline LCD_ReadData(void);

void LCD_SetFputcLocation(uint16_t x, uint16_t y, uint8_t size, uint8_t mode) {
	LCD_FputcInfo.x = x;
	LCD_FputcInfo.y = y;
	LCD_FputcInfo.size = size;
	LCD_FputcInfo.mode = mode;
}

//int fputc(int ch, FILE *p) {
//	if((ch <= '~') && (ch >= ' ')) {	//判断是不是控制字符       
//		LCD_ShowChar(LCD_FputcInfo.x, LCD_FputcInfo.y, (uint8_t)ch, LCD_FputcInfo.size, LCD_FputcInfo.mode);
//		LCD_FputcInfo.x += LCD_FputcInfo.size / 2;
//	} else if(ch == 10) {							//换行符
//		LCD_FputcInfo.y += LCD_FputcInfo.size;
//		LCD_FputcInfo.x = 0;
//	}
//	return ch;
//}

void InitLcd(void) { //Lcd初始化
	InitLcd_GPIO();
	InitLcd_FSMC();

	LCD_WriteCmd(0xB9);		//启用扩展指令集P.217 
	LCD_WriteData(0xFF); 
	LCD_WriteData(0x83); 
	LCD_WriteData(0x57);

	LCD_WriteCmd(0xD0);		//读取ID数据P.233
	LCD_Info.id=LCD_ReadData();
	LCD_Info.id=LCD_ReadData();

	LCD_WriteCmdData(0xE9, 0x20); //设置图像功能P.227
	//启动抖动功能
	
	LCD_WriteCmd(0x11); 					//退出休眠模式P.133 
	Delay_ms(5);

	LCD_WriteCmdData(0x3A, 0x05);	//设置像素数据格式P.164
	//16Bit

	LCD_Display_Dir(0);
	LCD_Clear(0xFFFF);
	
  LCD_WriteCmd(0x29);		//显示开启P.143

	LCD_SetFputcLocation(0, 0, 12, 0);	//设置fputc输出参数
}

void InitLcd_GPIO(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_FSMC);  //FSMC_D0-FSMC_D15
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_FSMC);  
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource10,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource12,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource15,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_FSMC); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource10,GPIO_AF_FSMC);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource4,GPIO_AF_FSMC);		//FSMC_NOE
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_FSMC);		//FSMC_NWE
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource12,GPIO_AF_FSMC);	//FSMC_A6
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource12,GPIO_AF_FSMC);	//FSMC_NE4
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9| GPIO_Pin_10 | GPIO_Pin_14| GPIO_Pin_15;
	//PD0,1,4,5,8,9,10,14,15 
	GPIO_Init(GPIOD, &GPIO_InitStructure);  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12| GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15;
	//PE7~15
	GPIO_Init(GPIOE, &GPIO_InitStructure);  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	//PF12,FSMC_A6
	GPIO_Init(GPIOF, &GPIO_InitStructure);  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	//PG12,FSMC_NE4
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}

void InitLcd_FSMC(void) {
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

	FSMC_NORSRAMTimingInitTypeDef FSMC_ReadTimingInitStructure;
	FSMC_NORSRAMTimingInitTypeDef FSMC_WriteTimingInitStructure;

	FSMC_ReadTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;
	FSMC_ReadTimingInitStructure.FSMC_AddressSetupTime = 0x00;
	FSMC_ReadTimingInitStructure.FSMC_AddressHoldTime = 0x00;
	FSMC_ReadTimingInitStructure.FSMC_DataSetupTime = 0x2F;	
	FSMC_ReadTimingInitStructure.FSMC_BusTurnAroundDuration= 0x00;
	FSMC_ReadTimingInitStructure.FSMC_CLKDivision = 0x00;
	FSMC_ReadTimingInitStructure.FSMC_DataLatency = 0x00;

	FSMC_WriteTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;
	FSMC_ReadTimingInitStructure.FSMC_AddressSetupTime = 0x00;
	FSMC_WriteTimingInitStructure.FSMC_AddressHoldTime = 0x00;
	FSMC_WriteTimingInitStructure.FSMC_DataSetupTime = 0x02;	
	FSMC_WriteTimingInitStructure.FSMC_BusTurnAroundDuration= 0x00;
	FSMC_WriteTimingInitStructure.FSMC_CLKDivision = 0x00;
	FSMC_WriteTimingInitStructure.FSMC_DataLatency = 0x00;

	FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMStructInit(&FSMC_NORSRAMInitStructure);

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_ReadTimingInitStructure;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_WriteTimingInitStructure;

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

void inline LCD_WriteCmd(uint16_t cmd) { 		//写入命令
	LCD->LCD_CMD=cmd;
}

void inline LCD_WriteData(uint16_t data) { 	//写入数据
	LCD->LCD_DATA=data;
}

uint16_t inline LCD_ReadData(void) { 				//读取数据
	return LCD->LCD_DATA;
}

void LCD_WriteCmdData(uint16_t cmd, uint16_t data) { //写入单参数命令
	LCD_WriteCmd(cmd);
	LCD_WriteData(data);
}

void LCD_Set_Window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height) { //设定数据更新范围
	LCD_WriteCmd(0x2A);				//选择列地址P.144
	LCD_WriteData(sx>>8);			//传高8位
	LCD_WriteData(sx&0XFF);		//传低8位
	LCD_WriteData(width>>8);
	LCD_WriteData(width&0XFF);

	LCD_WriteCmd(0x2B);				//选择页地址P.146
	LCD_WriteData(sy>>8);			//传高8位
	LCD_WriteData(sy&0XFF);		//传低8位
	LCD_WriteData(height>>8);
	LCD_WriteData(height&0XFF);

	LCD_WriteCmd(0x2C);				//启用内存写入P.148
}

void LCD_WriteData_Color(uint16_t color) { //写入颜色数据
	LCD_WriteData(color>>8);
	LCD_WriteData(color&0xFF);
}

void LCD_Display_Dir(uint8_t dir) { //定义显示方向
	LCD_Info.dir = dir;
	if(dir==0) {
		LCD_WriteCmdData(0x36, 0x4c); //设置内存访问控制P.157
		//列地址顺序读写,BGR颜色面板,水平方向控制
		LCD_Info.height=480;
		LCD_Info.width=320;
	} else {
		LCD_WriteCmdData(0x36, 0x2c); //设置内存访问控制P.157
		//页/列地址选择读写,BGR颜色面板,水平方向控制 
		LCD_Info.height=320;
		LCD_Info.width=480;
	}
}

void LCD_Clear(uint16_t color) { //清屏
	uint16_t i, j;

	LCD_Set_Window(0, 0, LCD_Info.width-1, LCD_Info.height-1);	 //作用区域
  for(i=0; i<LCD_Info.width; i++)
		for (j=0; j<LCD_Info.height; j++)
			LCD_WriteData_Color(color); 
}

void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color) { //绘制点
	if(x >= LCD_Info.width) return;
	if(y >= LCD_Info.height) return;
	
	LCD_Set_Window(x, y, x, y);
	LCD_WriteData_Color(color);	
}

void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size, uint8_t mode) { //绘制字符 							  
  uint8_t temp, t1, t;
	uint16_t y0 = y;
	uint8_t csize = (size / 8 + ((size % 8) ? 1 : 0 )) * (size / 2);	//得到字体一个字符对应点阵集所占的字节数	
 	num = num - ' ';	//得到偏移后的值(ASCII字库是从空格开始取模，所以减' '就是对应字符的字库)
	for(t = 0; t < csize; t++) {   
		if(size == 12) temp=ascii_1206[num][t]; 	 		//调用1206字体
		else if(size == 16) temp=ascii_1608[num][t];	//调用1608字体
		else if(size == 24) temp=ascii_2412[num][t];	//调用2412字体
		else return;																	//没有的字库

		for(t1 = 0; t1 < 8; t1++) {			    
			if(temp & 0x80) LCD_DrawPoint(x, y, LCD_FRONT_COLOR);
			else if(mode == 0) LCD_DrawPoint(x, y, LCD_BACK_COLOR);
			temp <<= 1;
			y++;
			if((y - y0) == size) {
				y=y0;
				x++;
				break;
			}
		}  	 
	}  	    	   	 	  
} 

void LCD_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p) {	//绘制字符串        
	uint8_t x0 = x;
	width += x;
	height += y;
	while((*p <= '~') && (*p >= ' ')) {	//判断是不是控制字符       
		if(x >= width) {
			x=x0;
			y+=size;
		}
		if(y >= height) break;	//退出
		LCD_ShowChar(x, y, *p, size, 0);
		x += size / 2;
		p++;
	}  
}

void LCD_Fill(uint16_t xState, uint16_t yState, uint16_t xEnd, uint16_t yEnd, uint16_t color) {	//填充颜色          
	uint16_t temp;

	if((xState > xEnd) || (yState > yEnd)) return;
 
	LCD_Set_Window(xState, yState, xEnd, yEnd); 
  xState = xEnd - xState + 1;
	yState = yEnd - yState + 1;

	while(xState--) {
	 	temp = yState;
		while(temp--) LCD_WriteData_Color(color);	
	}	
}

uint16_t LCD_ReadPoint(uint16_t x,uint16_t y) {	//获取点的颜色
	uint16_t r1, r2, r3;
	uint16_t value;
	
	if(x >= LCD_Info.width || y >= LCD_Info.height) return 0;	     
	LCD_Set_Window(x, y, x, y);

	LCD_WriteCmd(0X2E);
	r1 = LCD_ReadData();
	r1 = LCD_ReadData();
	//printf("r1=%x\r\n",r1);
	r2 = LCD_ReadData();
	//printf("r2=%x\r\n",r2);
	r3 = LCD_ReadData();
	//printf("r3=%x\r\n",r3);
	value = ((r1 & 0xf8) << 8) | ((r2 & 0xfc) << 3) | ((r3 & 0xf8) >> 3);

 	return value;						
}
