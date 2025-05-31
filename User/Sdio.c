#include <stdio.h>
#include <string.h>
#include "Delay.h"
#include "Sdio.h"
#include "stm32f4xx.h"

#define ENTER_CRITICAL() __disable_irq() // 关闭中断(进入临界区)
#define EXIT_CRITICAL() __enable_irq() 	 // 打开中断(退出临界区)

//SDIO时钟=SDIOCLK/[clkdiv+2];SDIOCLK固定为48Mhz
//SDIO初始化频率,限制为400KHz
#define SDIO_INIT_CLK_DIV 0x76 //48MHz/120=400KHz
//SDIO传输频率
#define SDIO_TRANSFER_CLK_DIV 0x76 //48MHz/120=400KHz	

#define SDIO_CMD_TIMEOUT 	((uint32_t)0x00010000)	//指令执行超时时间
#define SDIO_DATA_TIMEOUT	((uint32_t)0xFFFFFFFF)
#define SDIO_STATIC_FLAGS ((uint32_t)0x000005FF)	//清除所有标志位

//SDIO工作模式定义
#define SD_POLLING_MODE    	0  	//查询模式
#define SD_DMA_MODE    			1		//DMA模式

static uint8_t CardType = SDIO_STD_CAPACITY_SD_CARD_V1_1;		//SD卡类型(默认为1.x卡)
static uint8_t DeviceMode = SD_DMA_MODE;
static uint32_t SDType = SD_STD_CAPACITY;
static uint32_t CSD_Tab[4], CID_Tab[4], RCA=0;							//SD卡CSD,CID以及相对地址(RCA)数据
SD_CardInfo SDCardInfo;	//SD卡信息

//DMA标志
static uint8_t StopCondition = 0; 								//发送停止传输标志
volatile SD_Error TransferError = SD_OK;					//数据传输错误标志	    
volatile uint8_t TransferEnd = 0;										//传输结束标志

__align(4) uint8_t SDIO_DATA_BUFFER[512];
__align(4) uint32_t *tempbuff;

SD_Error CmdSendError(void);
SD_Error CmdRespError(uint32_t SDIO_CmdIndex, uint8_t Response_Type);
SD_Error SendCmd(uint32_t SDIO_CmdIndex, uint32_t SDIO_Argument, uint32_t SDIO_Response, uint8_t Response_Type);
SD_Error SD_EnWideBus(uint32_t enx);
SD_Error FindSCR(uint32_t *pscr);
uint8_t log2x(uint16_t NumberOfBytes);

SD_Error InitSdio(void) {
	SD_Error errorstatus = SD_OK;
	uint8_t clkdiv = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SDIO);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_SDIO);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);

	SDIO->ICR=0x00C007FF; //SDIO中断清零寄存器P.814

	errorstatus = SD_PowerON();
	if(errorstatus == SD_OK) errorstatus = SD_InitializeCards();															//初始化SD卡
	if(errorstatus == SD_OK) errorstatus = SD_GetCardInfo(&SDCardInfo);												//获取卡信息
	//SD_TestInfo(SDCardInfo);
	if(errorstatus == SD_OK) errorstatus = SD_SelectDeselect((uint32_t)(SDCardInfo.RCA<<16)); //选择卡
	if(errorstatus == SD_OK) errorstatus = SD_EnableWideBusOperation(SDIO_BusWide_4b);				//4位宽度传输模式

	if((errorstatus==SD_OK)||(SDIO_MULTIMEDIA_CARD==CardType)) {  		    
		clkdiv = SDIO_TRANSFER_CLK_DIV;
		SDIO_Clock_Set(clkdiv);									//设置时钟频率

		//errorstatus=SD_SetDeviceMode(SD_DMA_MODE);	//设置为DMA模式
		errorstatus=SD_SetDeviceMode(SD_POLLING_MODE);//设置为查询模式
 	}

	return errorstatus;
}

void SDIO_Clock_Set(uint8_t clkdiv) { //SDIO时钟初始化设置
	//SDIO时钟=SDIOCLK/[clkdiv+2];SDIOCLK固定为48Mhz
	uint32_t tmpreg = SDIO->CLKCR; 
  	tmpreg &= 0XFFFFFF00; 
 	tmpreg |= clkdiv;   
	SDIO->CLKCR = tmpreg;
} 

SD_Error SD_PowerON(void) {
	SD_Error errorstatus = SD_OK;
	uint32_t response = 0, count = 0; 
	uint8_t power_up_status = 0;

	SDIO_InitTypeDef SDIO_InitStructure;
	SDIO_InitStructure.SDIO_ClockDiv = SDIO_INIT_CLK_DIV;	
	SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
	SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;  								//直接用HCLK进行分频得到SDIO_CK
	SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;						//空闲时不关闭时钟电源
	SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;	 														//1位数据线
	SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;	//硬件流
	SDIO_Init(&SDIO_InitStructure);	

	SDIO_SetPowerState(SDIO_PowerState_ON);
	SDIO->CLKCR |= 1<<8;	//SDIOCK使能

	Delay_us(185);
	//发送最少74个脉冲;在400KHz下即185us

	errorstatus = SendCmd(SD_CMD_GO_IDLE_STATE, 0x00, SDIO_Response_No, SD_NO_RESPONSE);									//发送CMD0,进入IDLE STAGE模式命令
	errorstatus = SendCmd(SD_CMD_HS_SEND_EXT_CSD, SD_CHECK_PATTERN, SDIO_Response_Short, SD_R7_RESPONSE);	//发送CMD8

	if(errorstatus==SD_OK) { //SD2.0
		if(SDIO->RESP1 != 0x1AA) return SD_ERROR;	//验证响应
			
		CardType=SDIO_STD_CAPACITY_SD_CARD_V2_0;	//SD 2.0卡
		SDType=SD_HIGH_CAPACITY;
			
		//循环发送CMD55+ACMD41 
		while((!power_up_status)&&(count<SD_MAX_VOLT_TRIAL)) {
			errorstatus = SendCmd(SD_CMD_APP_CMD, 0x00, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD55

			//ACMD41,命令参数由支持的电压范围及HCS位组成,HCS位置一来区分卡是SDSC还是SDHC/SDXC
			//参数0x80100000 | 0x40000000
			errorstatus = SendCmd(SD_CMD_SD_APP_OP_COND, SD_VOLTAGE_WINDOW_SD | SDType, SDIO_Response_Short, SD_R3_RESPONSE);

			response=SDIO->RESP1;			   									//得到响应
			power_up_status=(((response>>31)==1)?1:0);		//判断SD卡上电是否完成
			count++;
		}
		if(count>=SD_MAX_VOLT_TRIAL)
			return SD_INVALID_VOLTRANGE;
		if(response&=SD_HIGH_CAPACITY)
			CardType=SDIO_HIGH_CAPACITY_SD_CARD;
	} else { //电压错误/SD1.X/MMC卡
		//TODO
		return SD_ERROR;
	}

	return errorstatus;
}

SD_Error SD_InitializeCards(void) {
	SD_Error errorstatus = SD_OK;
	uint32_t response = 0;

	if(SDIO_GetPowerState() == SDIO_PowerState_OFF)	//检查电源状态  
    return SD_REQUEST_NOT_APPLICABLE;

	if(SDIO_SECURE_DIGITAL_IO_CARD != CardType) {										//非SECURE_DIGITAL_IO_CARD
		errorstatus = SendCmd(SD_CMD_ALL_SEND_CID, 0x00, SDIO_Response_Long, SD_R2_RESPONSE);     //发送CMD2,取得CID	 
		
		CID_Tab[0]=SDIO->RESP1;
		CID_Tab[1]=SDIO->RESP2;
		CID_Tab[2]=SDIO->RESP3;
		CID_Tab[3]=SDIO->RESP4;
	}

	if((SDIO_STD_CAPACITY_SD_CARD_V1_1==CardType)||(SDIO_STD_CAPACITY_SD_CARD_V2_0==CardType)||(SDIO_SECURE_DIGITAL_IO_COMBO_CARD==CardType)||(SDIO_HIGH_CAPACITY_SD_CARD==CardType)) {	
		errorstatus = SendCmd(SD_CMD_SET_REL_ADDR, 0x00, SDIO_Response_Short, SD_R6_RESPONSE);		//发送CMD3
		response=SDIO->RESP1;	 
		if(SD_ALLZERO==(response&(SD_R6_GENERAL_UNKNOWN_ERROR|SD_R6_ILLEGAL_CMD|SD_R6_COM_CRC_FAILED))) {
			RCA=(uint16_t)(response>>16);	//右移16位得到,rca
		} else {
			if(response&SD_R6_GENERAL_UNKNOWN_ERROR) return SD_GENERAL_UNKNOWN_ERROR;
			if(response&SD_R6_ILLEGAL_CMD) return SD_ILLEGAL_CMD;
			if(response&SD_R6_COM_CRC_FAILED) return SD_COM_CRC_FAILED;
		}
	} else { //其他类型卡
		//TODO
		return SD_ERROR;
	}

	if(SDIO_SECURE_DIGITAL_IO_CARD != CardType) {										//非SECURE_DIGITAL_IO_CAR	
		errorstatus = SendCmd(SD_CMD_SEND_CSD, (uint32_t)(RCA << 16), SDIO_Response_Long, SD_R2_RESPONSE);	//发送CMD9+卡RCA,取得CSD	    
  		
		CSD_Tab[0]=SDIO->RESP1;
	  CSD_Tab[1]=SDIO->RESP2;
		CSD_Tab[2]=SDIO->RESP3;						
		CSD_Tab[3]=SDIO->RESP4;					    
	}
	return errorstatus;
}

SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo) { //解析卡信息
 	SD_Error errorstatus = SD_OK;
	uint8_t tmp=0;
	   
	cardinfo->CardType = (uint8_t)CardType; 				//卡类型
	cardinfo->RCA = (uint16_t)RCA;									//卡RCA值

	tmp=(uint8_t)((CSD_Tab[0]&0xFF000000)>>24);
	cardinfo->SD_csd.CSDStruct=(tmp&0xC0)>>6;				//CSD结构
	cardinfo->SD_csd.SysSpecVersion=(tmp&0x3C)>>2;	//2.0协议还没定义这部分(为保留),应该是后续协议定义的
	cardinfo->SD_csd.Reserved1=tmp&0x03;						//2个保留位
  
	tmp=(uint8_t)((CSD_Tab[0]&0x00FF0000)>>16);			//第1个字节
	cardinfo->SD_csd.TAAC=tmp;				   						//数据读时间1

	tmp=(uint8_t)((CSD_Tab[0]&0x0000FF00)>>8);	  	//第2个字节
	cardinfo->SD_csd.NSAC=tmp;		  								//数据读时间2

	tmp=(uint8_t)(CSD_Tab[0]&0x000000FF);						//第3个字节
	cardinfo->SD_csd.MaxBusClkFrec=tmp;		  				//传输速度
	   
	tmp=(uint8_t)((CSD_Tab[1]&0xFF000000)>>24);			//第4个字节
	cardinfo->SD_csd.CardComdClasses=tmp<<4;    		//卡指令类高四位

	tmp=(uint8_t)((CSD_Tab[1]&0x00FF0000)>>16);	 		//第5个字节
	cardinfo->SD_csd.CardComdClasses|=(tmp&0xF0)>>4;//卡指令类低四位
	cardinfo->SD_csd.RdBlockLen=tmp&0x0F;	    			//最大读取数据长度

	tmp=(uint8_t)((CSD_Tab[1]&0x0000FF00)>>8);			//第6个字节
	cardinfo->SD_csd.PartBlockRead=(tmp&0x80)>>7;		//允许分块读
	cardinfo->SD_csd.WrBlockMisalign=(tmp&0x40)>>6;	//写块错位
	cardinfo->SD_csd.RdBlockMisalign=(tmp&0x20)>>5;	//读块错位
	cardinfo->SD_csd.DSRImpl=(tmp&0x10)>>4;
	cardinfo->SD_csd.Reserved2=0; 									//保留

 	if((CardType==SDIO_STD_CAPACITY_SD_CARD_V1_1)||(CardType==SDIO_STD_CAPACITY_SD_CARD_V2_0)||(SDIO_MULTIMEDIA_CARD==CardType)) {
		//TODO
		return SD_ERROR;
	} else if(CardType==SDIO_HIGH_CAPACITY_SD_CARD) {
 		tmp=(uint8_t)(CSD_Tab[1]&0x000000FF); 				//第7个字节	
		cardinfo->SD_csd.DeviceSize=(tmp&0x3F)<<16;		//C_SIZE

 		tmp=(uint8_t)((CSD_Tab[2]&0xFF000000)>>24); 	//第8个字节	
 		cardinfo->SD_csd.DeviceSize|=(tmp<<8);

 		tmp=(uint8_t)((CSD_Tab[2]&0x00FF0000)>>16);		//第9个字节	
 		cardinfo->SD_csd.DeviceSize|=(tmp);

 		tmp=(uint8_t)((CSD_Tab[2]&0x0000FF00)>>8); 		//第10个字节	
 		cardinfo->CardCapacity=(long long)(cardinfo->SD_csd.DeviceSize+1)*512*1024;	//计算卡容量
		cardinfo->CardBlockSize=512; 									//块大小固定为512字节
	}	  
	cardinfo->SD_csd.EraseGrSize=(tmp&0x40)>>6;
	cardinfo->SD_csd.EraseGrMul=(tmp&0x3F)<<1;
	   
	tmp=(uint8_t)(CSD_Tab[2]&0x000000FF);						//第11个字节	
	cardinfo->SD_csd.EraseGrMul|=(tmp&0x80)>>7;
	cardinfo->SD_csd.WrProtectGrSize=(tmp&0x7F);
 	tmp=(uint8_t)((CSD_Tab[3]&0xFF000000)>>24);			//第12个字节	
	cardinfo->SD_csd.WrProtectGrEnable=(tmp&0x80)>>7;
	cardinfo->SD_csd.ManDeflECC=(tmp&0x60)>>5;
	cardinfo->SD_csd.WrSpeedFact=(tmp&0x1C)>>2;
	cardinfo->SD_csd.MaxWrBlockLen=(tmp&0x03)<<2;	 
	tmp=(uint8_t)((CSD_Tab[3]&0x00FF0000)>>16);			//第13个字节
	cardinfo->SD_csd.MaxWrBlockLen|=(tmp&0xC0)>>6;
	cardinfo->SD_csd.WriteBlockPaPartial=(tmp&0x20)>>5;
	cardinfo->SD_csd.Reserved3=0;
	cardinfo->SD_csd.ContentProtectAppli=(tmp&0x01);  
	tmp=(uint8_t)((CSD_Tab[3]&0x0000FF00)>>8);			//第14个字节
	cardinfo->SD_csd.FileFormatGrouop=(tmp&0x80)>>7;
	cardinfo->SD_csd.CopyFlag=(tmp&0x40)>>6;
	cardinfo->SD_csd.PermWrProtect=(tmp&0x20)>>5;
	cardinfo->SD_csd.TempWrProtect=(tmp&0x10)>>4;
	cardinfo->SD_csd.FileFormat=(tmp&0x0C)>>2;
	cardinfo->SD_csd.ECC=(tmp&0x03);  
	tmp=(uint8_t)(CSD_Tab[3]&0x000000FF);						//第15个字节
	cardinfo->SD_csd.CSD_CRC=(tmp&0xFE)>>1;
	cardinfo->SD_csd.Reserved4=1;
		 
	tmp=(uint8_t)((CID_Tab[0]&0xFF000000)>>24);			//第0个字节
	cardinfo->SD_cid.ManufacturerID=tmp;		
    
	tmp=(uint8_t)((CID_Tab[0]&0x00FF0000)>>16);			//第1个字节
	cardinfo->SD_cid.OEM_AppliID=tmp<<8;	  

	tmp=(uint8_t)((CID_Tab[0]&0x000000FF00)>>8);		//第2个字节
	cardinfo->SD_cid.OEM_AppliID|=tmp;	  
  
	tmp=(uint8_t)(CID_Tab[0]&0x000000FF);						//第3个字节	
	cardinfo->SD_cid.ProdName1=tmp<<24;		
		  
	tmp=(uint8_t)((CID_Tab[1]&0xFF000000)>>24); 		//第4个字节
	cardinfo->SD_cid.ProdName1|=tmp<<16;	  

	tmp=(uint8_t)((CID_Tab[1]&0x00FF0000)>>16);	   	//第5个字节
	cardinfo->SD_cid.ProdName1|=tmp<<8;		 

	tmp=(uint8_t)((CID_Tab[1]&0x0000FF00)>>8);			//第6个字节
	cardinfo->SD_cid.ProdName1|=tmp;		   

	tmp=(uint8_t)(CID_Tab[1]&0x000000FF);	  				//第7个字节
	cardinfo->SD_cid.ProdName2=tmp;			  

	tmp=(uint8_t)((CID_Tab[2]&0xFF000000)>>24); 		//第8个字节
	cardinfo->SD_cid.ProdRev=tmp;		 

	tmp=(uint8_t)((CID_Tab[2]&0x00FF0000)>>16);			//第9个字节
	cardinfo->SD_cid.ProdSN=tmp<<24;	   

	tmp=(uint8_t)((CID_Tab[2]&0x0000FF00)>>8); 			//第10个字节
	cardinfo->SD_cid.ProdSN|=tmp<<16;	   

	tmp=(uint8_t)(CID_Tab[2]&0x000000FF);   				//第11个字节
	cardinfo->SD_cid.ProdSN|=tmp<<8;		   

	tmp=(uint8_t)((CID_Tab[3]&0xFF000000)>>24); 		//第12个字节
	cardinfo->SD_cid.ProdSN|=tmp;			     

	tmp=(uint8_t)((CID_Tab[3]&0x00FF0000)>>16);	 		//第13个字节
	cardinfo->SD_cid.Reserved1|=(tmp&0xF0)>>4;
	cardinfo->SD_cid.ManufactDate=(tmp&0x0F)<<8;   
 
	tmp=(uint8_t)((CID_Tab[3]&0x0000FF00)>>8);			//第14个字节
	cardinfo->SD_cid.ManufactDate|=tmp;		 	  

	tmp=(uint8_t)(CID_Tab[3]&0x000000FF);						//第15个字节
	cardinfo->SD_cid.CID_CRC=(tmp&0xFE)>>1;
	cardinfo->SD_cid.Reserved2=1;	 

	return errorstatus;
}

SD_Error inline SD_SelectDeselect(uint32_t addr) { //选择相对地址(RCA)为addr的卡	
 	return SendCmd(SD_CMD_SEL_DESEL_CARD, addr, SDIO_Response_Short, SD_R1_RESPONSE); //发送CMD7	  
}

SD_Error SD_SetDeviceMode(uint32_t Mode) { //设置SD卡工作模式
	SD_Error errorstatus = SD_OK;
 	if((Mode==SD_DMA_MODE)||(Mode==SD_POLLING_MODE)) DeviceMode=Mode;
	else errorstatus = SD_INVALID_PARAMETER;
	return errorstatus;	    
}

SD_Error SD_EnableWideBusOperation(uint32_t WideMode) { //设置SDIO总线宽度
  SD_Error errorstatus=SD_OK;

  if (SDIO_MULTIMEDIA_CARD == CardType) {	//MMC卡不支持4bit模式
		return SD_UNSUPPORTED_FEATURE;	
 	} else if((SDIO_STD_CAPACITY_SD_CARD_V1_1==CardType)||(SDIO_STD_CAPACITY_SD_CARD_V2_0==CardType)||(SDIO_HIGH_CAPACITY_SD_CARD==CardType)) {
		if (SDIO_BusWide_8b == WideMode)			//SD不支持8bits 
      return SD_UNSUPPORTED_FEATURE;
 		else {
			errorstatus = SD_EnWideBus(WideMode);
 			if(SD_OK == errorstatus) {
				SDIO->CLKCR&=~(3<<11);		//清除之前的位宽设置
				Delay_us(1);
				SDIO->CLKCR|=WideMode;		//1位/4位总线宽度 
				SDIO->CLKCR|=0<<14;				//不开启硬件流控制
			}
		}  	 
	} else {
		//TODO
		return SD_ERROR;
	}
	return errorstatus;
}

SD_Error SD_EnWideBus(uint32_t enx) { //SDIO使能宽总线模式
	SD_Error errorstatus = SD_OK;
 	uint32_t scr[2]={0,0};
	uint8_t arg=0X00;

	if(enx) arg = 0X02;
	else arg = 0X00;

 	if(SDIO->RESP1&SD_CARD_LOCKED) return SD_LOCK_UNLOCK_FAILED;	//SD卡处于LOCKED状态
		    
 	errorstatus = FindSCR(scr);											//得到SCR寄存器数据
 	if(errorstatus!=SD_OK) return errorstatus;

	if((scr[1]&SD_WIDE_BUS_SUPPORT)!=SD_ALLZERO) {	//支持宽总线
		errorstatus = SendCmd(SD_CMD_APP_CMD, (uint32_t)RCA << 16, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD55+RCA
		errorstatus = SendCmd(SD_CMD_APP_SD_SET_BUSWIDTH, arg, SDIO_Response_Short, SD_R1_RESPONSE);			//发送ACMD6
		
		return errorstatus;
	} else 
		return SD_REQUEST_NOT_APPLICABLE;							//不支持宽总线设置 	 
}

SD_Error FindSCR(uint32_t *pscr) { //查找SD卡的SCR寄存器值	 
	SD_Error errorstatus = SD_OK;
	uint32_t index = 0;
	uint32_t tempscr[2] = {0, 0};

	errorstatus = SendCmd(SD_CMD_SET_BLOCKLEN, (uint32_t)8, SDIO_Response_Short, SD_R1_RESPONSE);			//发送CMD16,设置Block Size为8字节 
	errorstatus = SendCmd(SD_CMD_APP_CMD, (uint32_t)RCA << 16, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD55 
	
	SDIO_DataInitTypeDef SDIO_DataInitStructure;
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 8;  													//8个字节长度,block为8字节,SD卡到SDIO.
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_8b;  	//块大小8byte 
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
  SDIO_DataConfig(&SDIO_DataInitStructure);	

	errorstatus = SendCmd(SD_CMD_SD_APP_SEND_SCR, 0x00, SDIO_Response_Short, SD_R1_RESPONSE);	//发送ACMD51
	if(errorstatus != SD_OK) return errorstatus;

	while(!(SDIO->STA&(SDIO_FLAG_RXOVERR|SDIO_FLAG_DCRCFAIL|SDIO_FLAG_DTIMEOUT|SDIO_FLAG_DBCKEND|SDIO_FLAG_STBITERR))) { 
		if(SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET) { //接收FIFO数据可用	
			*(tempscr+index)=SDIO->FIFO;	//读取FIFO内容
			index++;
			if(index >= 2) break;
		}
	}

	if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {					//数据超时错误											   
		SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
		return SD_DATA_TIMEOUT;
	} else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {	//数据块CRC错误		
		SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
		return SD_DATA_CRC_FAIL;		   
	} else if(SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) { 	//接收FIFO上溢错误
		SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
		return SD_RX_OVERRUN;		 
	}else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) { 	//接收起始位错误		
		SDIO_ClearFlag(SDIO_FLAG_STBITERR);
		return SD_START_BIT_ERR;		 
	}  
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);	//清除所有标记

	//把数据顺序按8位为单位倒过来   	
	*(pscr+1) = ((tempscr[0]&SD_0TO7BITS)<<24) | ((tempscr[0]&SD_8TO15BITS)<<8) | ((tempscr[0]&SD_16TO23BITS)>>8) | ((tempscr[0]&SD_24TO31BITS)>>24);
	*(pscr) = ((tempscr[1]&SD_0TO7BITS)<<24) | ((tempscr[1]&SD_8TO15BITS)<<8) | ((tempscr[1]&SD_16TO23BITS)>>8) | ((tempscr[1]&SD_24TO31BITS)>>24);

 	return errorstatus;
}

SD_Error CmdSendError(void) { //检查命令的状态
	uint32_t timeout = SDIO_CMD_TIMEOUT;	
   
	while(timeout--)
		if(SDIO_GetFlagStatus(SDIO_FLAG_CMDSENT) != RESET) break;	//命令已发送(无响应)	    
	if(timeout==0) return SD_CMD_RSP_TIMEOUT;  
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);													//清除所有标记

	return SD_OK;
}

SD_Error CmdRespError(uint32_t SDIO_CmdIndex, uint8_t Response_Type) { //检查响应的状态	 
	uint32_t status; 

	while(1) {
		status = SDIO->STA;
		if(status&((1<<0)|(1<<2)|(1<<6))) break;						//CRC错误/命令响应超时/已经收到响应(CRC校验成功)
	}
	if(SDIO_GetFlagStatus(SDIO_FLAG_CTIMEOUT) != RESET) {
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
			return SD_CMD_RSP_TIMEOUT;	//响应超时
	}
	if(Response_Type==SD_R1B_RESPONSE || Response_Type==SD_R1_RESPONSE || Response_Type==SD_R2_RESPONSE || Response_Type==SD_R6_RESPONSE)																									
		if(SDIO_GetFlagStatus(SDIO_FLAG_CCRCFAIL) != RESET) {
			SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
			return SD_CMD_CRC_FAIL;			//CRC错误
		}
	if(Response_Type==SD_R1B_RESPONSE || Response_Type==SD_R1_RESPONSE || Response_Type==SD_R6_RESPONSE)																									
		if(SDIO->RESPCMD != SDIO_CmdIndex) {
			SDIO_ClearFlag(SDIO_STATIC_FLAGS);
			return SD_ILLEGAL_CMD;			//命令不匹配
		}
 		 		
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);	//清除所有标记
	return SD_OK;	//返回卡响应
}

SD_Error SendCmd(uint32_t SDIO_CmdIndex, uint32_t SDIO_Argument, uint32_t SDIO_Response, uint8_t Response_Type) { //发送命令
	SD_Error errorstatus = SD_OK;

	SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
	
	SDIO_CmdInitStructure.SDIO_Argument = SDIO_Argument;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_CmdIndex;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response;

	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;  				
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	if(SDIO_Response == SDIO_Response_No)
		errorstatus = CmdSendError();
	else
		errorstatus = CmdRespError(SDIO_CmdIndex, Response_Type);

	return errorstatus;
} 

void SD_TestInfo(SD_CardInfo cardinfo) { //调试信息
	uint8_t i;
	
	printf("CID = ");
	for(i = 0; i < 4; i++) printf("0x%08X ", CID_Tab[i]);
	printf("\n");
	printf("CSD = ");
	for(i = 0; i < 4; i++) printf("0x%08X ", CSD_Tab[i]);
	printf("\n");

	printf("----------------\n");

	printf("CSD structure = 0x%02X\n", cardinfo.SD_csd.CSDStruct);
	printf("System specification version = 0x%02X\n", cardinfo.SD_csd.SysSpecVersion);
	printf("Reserved = 0x%02X\n", cardinfo.SD_csd.Reserved1);
	printf("Data read access-time 1 = 0x%02X\n", cardinfo.SD_csd.TAAC);
	printf("Data read access-time 2 in CLK cycles = 0x%02X\n", cardinfo.SD_csd.NSAC);
	printf("Max. bus clock frequency = 0x%02X\n", cardinfo.SD_csd.MaxBusClkFrec);
	printf("Card command classes = 0x%04X\n", cardinfo.SD_csd.CardComdClasses);
	printf("Max. read data block length = 0x%02X\n", cardinfo.SD_csd.RdBlockLen);
	printf("Partial blocks for read allowed = 0x%02X\n", cardinfo.SD_csd.PartBlockRead);
	printf("Write block misalignment = 0x%02X\n", cardinfo.SD_csd.WrBlockMisalign);
	printf("Read block misalignment = 0x%02X\n", cardinfo.SD_csd.RdBlockMisalign);
	printf("DSR implemented = 0x%02X\n", cardinfo.SD_csd.DSRImpl);
	printf("Reserved = 0x%02X\n", cardinfo.SD_csd.Reserved2);
	printf("Device Size = 0x%08X\n", cardinfo.SD_csd.DeviceSize);
	printf("Max. read current @ VDD min = 0x%02X\n", cardinfo.SD_csd.MaxRdCurrentVDDMin);
	printf("Max. read current @ VDD max = 0x%02X\n", cardinfo.SD_csd.MaxRdCurrentVDDMax);
	printf("Max. write current @ VDD min = 0x%02X\n", cardinfo.SD_csd.MaxWrCurrentVDDMin);
	printf("Max. write current @ VDD max = 0x%02X\n", cardinfo.SD_csd.MaxWrCurrentVDDMax);
	printf("Device size multiplier = 0x%02X\n", cardinfo.SD_csd.DeviceSizeMul);
	printf("Erase group size = 0x%02X\n", cardinfo.SD_csd.EraseGrSize);
	printf("Erase group size multiplier = 0x%02X\n", cardinfo.SD_csd.EraseGrMul);
	printf("Write protect group size = 0x%02X\n", cardinfo.SD_csd.WrProtectGrSize);
	printf("Write protect group enable = 0x%02X\n", cardinfo.SD_csd.WrProtectGrEnable);
	printf("Manufacturer default ECC = 0x%02X\n", cardinfo.SD_csd.ManDeflECC);
	printf("Write speed factor = 0x%02X\n", cardinfo.SD_csd.WrSpeedFact);
	printf("Max. write data block length = 0x%02X\n", cardinfo.SD_csd.MaxWrBlockLen);
	printf("Partial blocks for write allowed = 0x%02X\n", cardinfo.SD_csd.WriteBlockPaPartial);
	printf("Reserded = 0x%02X\n", cardinfo.SD_csd.Reserved3);
	printf("Content protection application = 0x%02X\n", cardinfo.SD_csd.ContentProtectAppli);
	printf("File format group = 0x%02X\n", cardinfo.SD_csd.FileFormatGrouop);
	printf("Copy flag (OTP) = 0x%02X\n", cardinfo.SD_csd.CopyFlag);
	printf("Permanent write protection = 0x%02X\n", cardinfo.SD_csd.PermWrProtect);
	printf("Temporary write protection = 0x%02X\n", cardinfo.SD_csd.TempWrProtect);
	printf("File Format = 0x%02X\n", cardinfo.SD_csd.FileFormat);
	printf("ECC code = 0x%02X\n", cardinfo.SD_csd.ECC);
	printf("CSD CRC = 0x%02X\n", cardinfo.SD_csd.CSD_CRC);
	printf("always 1 = 0x%02X\n", cardinfo.SD_csd.Reserved4);

	printf("----------------\n");

	printf("ManufacturerID = 0x%02X\n", cardinfo.SD_cid.ManufacturerID);
	printf("OEM/Application ID = 0x%04X\n", cardinfo.SD_cid.OEM_AppliID);
	printf("Product Name part1 = 0x%08X\n", cardinfo.SD_cid.ProdName1);
	printf("Product Name part2 = 0x%02X\n", cardinfo.SD_cid.ProdName2);
	printf("Product Revision = 0x%02X\n", cardinfo.SD_cid.ProdRev);
	printf("Product Serial Number = 0x%08X\n", cardinfo.SD_cid.ProdSN);
	printf("Reserved1 = 0x%02X\n", cardinfo.SD_cid.Reserved1);
	printf("Manufacturing Date = 0x%04X\n", cardinfo.SD_cid.ManufactDate);
	printf("CID CRC = 0x%02X\n", cardinfo.SD_cid.CID_CRC);
	printf("always 1 = 0x%02X\n", cardinfo.SD_cid.Reserved2);

	printf("----------------\n");

	printf("Card Capacity(Byte) = %lld\n", cardinfo.CardCapacity);
	printf("Card Block Size = 0x%08X\n", cardinfo.CardBlockSize);
	printf("RCA = 0x%04X\n", cardinfo.RCA);
	printf("Card Type = 0x%02X\n", cardinfo.CardType);
}

uint8_t log2x(uint16_t NumberOfBytes) { //返回以2为底的对数(向下取整)
	uint8_t count=0;
	while(NumberOfBytes != 1) {
		NumberOfBytes >>= 1;
		count++;
	}
	return count;
}

void SD_DMA_Config(uint32_t*mbuf, uint32_t bufsize, uint32_t dir) { //配置DMA		 	
	while(DMA_GetCmdStatus(DMA2_Stream3) != DISABLE) {}	//等待DMA可配置 
		
  DMA_DeInit(DMA2_Stream3);	//清空之前该stream3上的所有中断标志
	
	DMA_InitTypeDef  DMA_InitStructure;
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  												//通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SDIO->FIFO;				//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)mbuf;									//DMA存储器0地址
  DMA_InitStructure.DMA_DIR = dir;																				//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = 0;																		//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;									//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;	//外设数据长度:32位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;					//存储器数据长度:32位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;														//使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;									//最高优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;   								//FIFO使能      
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;						//全FIFO
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;								//外设突发4次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;				//存储器突发4次传输
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);	//初始化DMA Stream

	DMA_FlowControllerConfig(DMA2_Stream3,DMA_FlowCtrl_Peripheral);//外设流控制 
	 
  DMA_Cmd(DMA2_Stream3 ,ENABLE);	//开启DMA传输	 
}

SD_Error SD_ReadBlock(uint8_t *buf, uint32_t addr, uint16_t blksize) { //读一个块	  
	SD_Error errorstatus = SD_OK;
	uint8_t power;
	uint32_t timeout = SDIO_DATA_TIMEOUT;
  uint32_t count, *tempbuff = (uint32_t*)buf;
   
  if(NULL==buf) return SD_INVALID_PARAMETER; 

  SDIO->DCTRL = 0x0;	//数据控制寄存器清零(关DMA) 
  
	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD) //大容量卡
		blksize=512;
		
	if(SDIO->RESP1&SD_CARD_LOCKED) return SD_LOCK_UNLOCK_FAILED;

	if((blksize>0) && (blksize<=2048) && ((blksize&(blksize-1)) == 0)) {
		power = log2x(blksize);	
		errorstatus = SendCmd(SD_CMD_SET_BLOCKLEN, blksize, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD16,设置数据长度为blksize 		
		if(errorstatus != SD_OK) return errorstatus;			
	} else return SD_INVALID_PARAMETER;	  	 
	
	SDIO_DataInitTypeDef SDIO_DataInitStructure;
	SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4;
	SDIO_DataInitStructure.SDIO_DataLength = blksize;
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	errorstatus = SendCmd(SD_CMD_READ_SINGLE_BLOCK, addr, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD17,从addr地址出读取数据
	if(errorstatus!=SD_OK) return errorstatus;
	 
 	if(DeviceMode==SD_POLLING_MODE) {	//查询模式,轮询数据	 
 		ENTER_CRITICAL();	//关闭中断

		while(!(SDIO->STA&(SDIO_FLAG_RXOVERR|SDIO_FLAG_DCRCFAIL|SDIO_FLAG_DTIMEOUT|SDIO_FLAG_DBCKEND|SDIO_FLAG_STBITERR))) {		
			if(SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET) {	//接收区半满,表示至少存了8个字			
				for(count = 0; count < 8; count++)
					*(tempbuff+count) = SDIO->FIFO;
				tempbuff += 8;
				timeout = 0X7FFFFF;	//读数据溢出时间
			} else {																						 	//处理超时		
				if(timeout == 0) return SD_DATA_TIMEOUT;
				timeout--;
			}
		}

		while(SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET) {	//FIFO里面,还存在可用数据	
			*tempbuff = SDIO->FIFO;	//循环读取数据
			tempbuff++;
		}
 
		if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {					//数据超时错误											   
			SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
			return SD_DATA_TIMEOUT;
		} else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {	//数据块CRC错误		
			SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
			return SD_DATA_CRC_FAIL;		   
		} else if(SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) { 	//接收FIFO上溢错误
			SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
			return SD_RX_OVERRUN;		 
		} else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) { 	//接收起始位错误		
			SDIO_ClearFlag(SDIO_FLAG_STBITERR);
			return SD_START_BIT_ERR;		 
		} 
 
		EXIT_CRITICAL();	//开启中断
		SDIO_ClearFlag(SDIO_STATIC_FLAGS);//清除所有标记
	 
	} else if(DeviceMode==SD_DMA_MODE) {
 		TransferError = SD_OK;
		StopCondition = 0;			//单块读,不需要发送停止传输指令
		TransferEnd = 0;				//传输结束标置位，在中断服务置1

		SDIO->MASK |= SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | SDIO_MASK_DATAENDIE | SDIO_MASK_RXOVERRIE | SDIO_MASK_STBITERRIE;	//配置需要的中断 
	 	SDIO->DCTRL |= SDIO_DCTRL_DMAEN;		 			//DMA使能 
 	  SD_DMA_Config((uint32_t*)buf, blksize, DMA_DIR_PeripheralToMemory);
 
		while(((DMA2->LISR&(1<<27))==RESET) && timeout) timeout--;	//等待传输完成 
		if(timeout == 0) return SD_DATA_TIMEOUT;	//超时

		timeout = SDIO_DATA_TIMEOUT;
		while((TransferEnd==0) && (TransferError==SD_OK) && timeout) timeout--;
		if(timeout == 0) return SD_DATA_TIMEOUT;	//超时	 
		if(TransferError != SD_OK) return TransferError;   
	}
   
 	return errorstatus; 
}

SD_Error SD_ReadMultiBlocks(uint8_t *buf, long long addr, uint16_t blksize, uint32_t nblks) { //读多个块
  SD_Error errorstatus = SD_OK;
	uint8_t power;
  uint32_t count = 0;
	uint32_t timeout = SDIO_DATA_TIMEOUT;  
	tempbuff = (uint32_t*)buf;
	
	if(NULL==buf) return SD_INVALID_PARAMETER; 

  SDIO->DCTRL = 0x0;	//数据控制寄存器清零(关DMA) 
  
	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD) //大容量卡
		blksize=512;
		
	if(SDIO->RESP1&SD_CARD_LOCKED) return SD_LOCK_UNLOCK_FAILED;

	if((blksize>0) && (blksize<=2048) && ((blksize&(blksize-1)) == 0)) {
		power = log2x(blksize);	
		errorstatus = SendCmd(SD_CMD_SET_BLOCKLEN, blksize, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD16,设置数据长度为blksize 
		if(errorstatus != SD_OK) return errorstatus;			
	} else return SD_INVALID_PARAMETER;	  
												
	if(nblks * blksize > SD_MAX_DATA_LENGTH) return SD_INVALID_PARAMETER;	//判断是否超过最大接收长度 

	SDIO_DataInitTypeDef SDIO_DataInitStructure;
	SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4;
	SDIO_DataInitStructure.SDIO_DataLength = nblks * blksize;
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	errorstatus = SendCmd(SD_CMD_READ_MULT_BLOCK, addr, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD18,从addr地址出读取数据
	if(errorstatus!=SD_OK) return errorstatus;		 
		
	if(DeviceMode==SD_POLLING_MODE) {
		ENTER_CRITICAL();
		while(!(SDIO->STA&(SDIO_FLAG_RXOVERR|SDIO_FLAG_DCRCFAIL|SDIO_FLAG_DTIMEOUT|SDIO_FLAG_DATAEND|SDIO_FLAG_STBITERR))) {		
			if(SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET) {	//接收区半满,表示至少存了8个字			
				for(count = 0; count < 8; count++)
					*(tempbuff+count) = SDIO->FIFO;
				tempbuff += 8;
				timeout = 0X7FFFFF;	//读数据溢出时间
			} else {																						 	//处理超时		
				if(timeout == 0) return SD_DATA_TIMEOUT;
				timeout--;
			}
		}

		while(SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET) {	//FIFO里面,还存在可用数据	
			*tempbuff=SDIO->FIFO;	//循环读取数据
			tempbuff++;
		}
 
		if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {					//数据超时错误											   
			SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
			return SD_DATA_TIMEOUT;
		} else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {	//数据块CRC错误		
			SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
			return SD_DATA_CRC_FAIL;		   
		} else if(SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) { 	//接收FIFO上溢错误
			SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
			return SD_RX_OVERRUN;		 
		} else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) { 	//接收起始位错误		
			SDIO_ClearFlag(SDIO_FLAG_STBITERR);
			return SD_START_BIT_ERR;		 
		}  
			
		if(SDIO_GetFlagStatus(SDIO_FLAG_DATAEND) != RESET) {					//接收结束			
			if((SDIO_STD_CAPACITY_SD_CARD_V1_1==CardType)||(SDIO_STD_CAPACITY_SD_CARD_V2_0==CardType)||(SDIO_HIGH_CAPACITY_SD_CARD==CardType)) {
				errorstatus = SendCmd(SD_CMD_STOP_TRANSMISSION, 0, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD12+结束传输
				if(errorstatus!=SD_OK) return errorstatus;				
			}
		}
		EXIT_CRITICAL();	//开启中断
		SDIO_ClearFlag(SDIO_STATIC_FLAGS);//清除所有标记

	} else if(DeviceMode==SD_DMA_MODE) {
		TransferError = SD_OK;
		StopCondition = 1;			//多块读,需要发送停止传输指令
		TransferEnd = 0;				//传输结束标置位，在中断服务置1
			
		SDIO->MASK |= SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | SDIO_MASK_DATAENDIE | SDIO_MASK_RXOVERRIE | SDIO_MASK_STBITERRIE;	//配置需要的中断 
		SDIO->DCTRL |= SDIO_DCTRL_DMAEN;		 			//DMA使能 

		SD_DMA_Config((uint32_t*)buf, nblks*blksize, DMA_DIR_PeripheralToMemory);

		while(((DMA2->LISR&(1<<27))==RESET) && timeout) timeout--;	//等待传输完成 
		if(timeout == 0) return SD_DATA_TIMEOUT;	//超时

		timeout = SDIO_DATA_TIMEOUT;
		while((TransferEnd==0) && (TransferError==SD_OK) && timeout) timeout--;
		if(timeout == 0) return SD_DATA_TIMEOUT;	//超时	 
		if(TransferError != SD_OK) return TransferError; 
	}		 

	return errorstatus;
}		

SD_Error SD_WriteBlock(uint8_t *buf, long long addr, uint16_t blksize) { //写一个块	
	SD_Error errorstatus = SD_OK;
	uint8_t power = 0, cardstate = 0;	
	uint32_t timeout = SDIO_DATA_TIMEOUT, bytestransferred = 0;
	uint32_t cardstatus = 0, count = 0, restwords = 0;	
	uint32_t tlen=blksize;						//总长度(字节)	
	uint32_t* tempbuff = (uint32_t*)buf;					
	
 	if(NULL==buf) return SD_INVALID_PARAMETER;  
	
	SDIO->DCTRL = 0x0;	//数据控制寄存器清零(关DMA) 
	
	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD) //大容量卡
		blksize=512;
		
	if(SDIO->RESP1&SD_CARD_LOCKED) return SD_LOCK_UNLOCK_FAILED;
    
	if((blksize>0) && (blksize<=2048) && ((blksize&(blksize-1)) == 0)) {
		power = log2x(blksize);	
		errorstatus = SendCmd(SD_CMD_SET_BLOCKLEN, blksize, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD16,设置数据长度为blksize 		
		if(errorstatus != SD_OK) return errorstatus;			
	} else return SD_INVALID_PARAMETER;

	SDIO_DataInitTypeDef SDIO_DataInitStructure;
	SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4;
	SDIO_DataInitStructure.SDIO_DataLength = blksize;
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);
	
	do { 	//检查READY_FOR_DATA位是否置位
		errorstatus = SendCmd(SD_CMD_SEND_STATUS, (uint32_t) RCA << 16, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD13,查询卡的状态  
		if(errorstatus != SD_OK) return errorstatus;

		cardstatus = SDIO->RESP1;													  
		timeout--;
	} while(((cardstatus&0x00000100)==0)&&(timeout>0));
	if(timeout == 0) return SD_ERROR;

	errorstatus = SendCmd(SD_CMD_WRITE_SINGLE_BLOCK, addr, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD24,写单块指令
	if(errorstatus!=SD_OK) return errorstatus; 	 
	
	StopCondition = 0;	//单块写,不需要发送停止传输指令 
	timeout = SDIO_DATA_TIMEOUT;
	
	if(DeviceMode==SD_POLLING_MODE) {
		ENTER_CRITICAL();	//关闭中断

		while(!(SDIO->STA&(SDIO_FLAG_TXUNDERR|SDIO_FLAG_DCRCFAIL|SDIO_FLAG_DTIMEOUT|SDIO_FLAG_DBCKEND|SDIO_FLAG_STBITERR))) {
			if(SDIO_GetFlagStatus(SDIO_FLAG_TXFIFOHE) != RESET)	{	//发送区半空,表示至少存了8个字		
				if((tlen - bytestransferred) < 32) {	
					restwords=((tlen - bytestransferred) % 4 == 0) ? ((tlen - bytestransferred) / 4) : ((tlen - bytestransferred) / 4 + 1);
					for(count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
						SDIO->FIFO = *tempbuff;
				} else {
					for(count = 0; count < 8; count++)
						SDIO->FIFO = *(tempbuff+count);
					tempbuff += 8;
					bytestransferred += 32;
				}
				timeout=0X3FFFFFFF;	//写数据溢出时间
			} else {
				if(timeout == 0) return SD_DATA_TIMEOUT;
				timeout--;
			}
		}

		if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {					//数据超时错误											   
			SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
			return SD_DATA_TIMEOUT;
		} else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {	//数据块CRC错误		
			SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
			return SD_DATA_CRC_FAIL;		   
		} else if(SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET) { 	//接收FIFO上溢错误
			SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);
			return SD_TX_UNDERRUN;		 
		} else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) { 	//接收起始位错误		
			SDIO_ClearFlag(SDIO_FLAG_STBITERR);
			return SD_START_BIT_ERR;		 
		}		
	      
		EXIT_CRITICAL();	//开启中断
		SDIO_ClearFlag(SDIO_STATIC_FLAGS);//清除所有标记  
	} else if(DeviceMode==SD_DMA_MODE) {
		TransferError = SD_OK;
		StopCondition = 0;			//单块写,不需要发送停止传输指令
		TransferEnd = 0;				//传输结束标置位，在中断服务置1
		
		SDIO->MASK |= SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | SDIO_MASK_DATAENDIE | SDIO_MASK_TXUNDERRIE | SDIO_MASK_STBITERRIE;	//配置产生数据接收完成中断	
 	 	SDIO->DCTRL |= SDIO_DCTRL_DMAEN;		 			//DMA使能
		SD_DMA_Config((uint32_t*)buf, blksize, DMA_DIR_MemoryToPeripheral);

 		while(((DMA2->LISR&(1<<27))==RESET) && timeout) timeout--;	//等待传输完成 
		if(timeout == 0) return SD_DATA_TIMEOUT;	//超时

		timeout = SDIO_DATA_TIMEOUT;
		while((TransferEnd==0) && (TransferError==SD_OK) && timeout) timeout--;
 		if(timeout == 0) return SD_DATA_TIMEOUT;	//超时	 
  	if(TransferError != SD_OK) return TransferError;
 	}  
 	SDIO_ClearFlag(SDIO_STATIC_FLAGS);//清除所有标记	
 	
	do {
		cardstate = SD_GetStatus();
	} while((cardstate == SD_CARD_PROGRAMMING) || (cardstate == SD_CARD_RECEIVING)); 
  
	return errorstatus;
}

SD_Error SD_WriteMultiBlocks(uint8_t *buf, long long addr, uint16_t blksize, uint32_t nblks) { //写多个块
	SD_Error errorstatus = SD_OK;
	uint8_t power = 0, cardstate = 0;
	uint32_t timeout = SDIO_DATA_TIMEOUT, bytestransferred = 0;
	uint32_t cardstatus = 0, count = 0, restwords = 0;
	uint32_t tlen = nblks*blksize;				//总长度(字节)
	uint32_t *tempbuff = (uint32_t*)buf;
  
 	if(NULL==buf) return SD_INVALID_PARAMETER;  
	
	SDIO->DCTRL = 0x0;	//数据控制寄存器清零(关DMA) 
	
	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD) //大容量卡
		blksize=512;
		
	if(SDIO->RESP1&SD_CARD_LOCKED) return SD_LOCK_UNLOCK_FAILED;

	if((blksize>0) && (blksize<=2048) && ((blksize&(blksize-1)) == 0)) {
		power = log2x(blksize);	
		errorstatus = SendCmd(SD_CMD_SET_BLOCKLEN, blksize, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD16,设置数据长度为blksize 		
		if(errorstatus != SD_OK) return errorstatus;			
	} else return SD_INVALID_PARAMETER;
	 
	if(nblks * blksize > SD_MAX_DATA_LENGTH) return SD_INVALID_PARAMETER;

	SDIO_DataInitTypeDef SDIO_DataInitStructure;
	SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4;
	SDIO_DataInitStructure.SDIO_DataLength = nblks*blksize;
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);
   
	if((SDIO_STD_CAPACITY_SD_CARD_V1_1==CardType)||(SDIO_STD_CAPACITY_SD_CARD_V2_0==CardType)||(SDIO_HIGH_CAPACITY_SD_CARD==CardType)) { //预先擦除需要写入的块，以提高效率
		SendCmd(SD_CMD_APP_CMD, (uint32_t)RCA << 16, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD55
		if(errorstatus != SD_OK) return errorstatus;		
		SendCmd(SD_CMD_SET_BLOCK_COUNT, nblks, SDIO_Response_Short, SD_R1_RESPONSE);				//发送ACMD23,设置块数量
		if(errorstatus != SD_OK) return errorstatus;	
	} 

	do { 	//检查READY_FOR_DATA位是否置位
		errorstatus = SendCmd(SD_CMD_SEND_STATUS, (uint32_t) RCA << 16, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD13,查询卡的状态  
		if(errorstatus != SD_OK) return errorstatus;

		cardstatus = SDIO->RESP1;													  
		timeout--;
	} while(((cardstatus&0x00000100)==0)&&(timeout>0));
	if(timeout == 0) return SD_ERROR;

	errorstatus = SendCmd(SD_CMD_WRITE_MULT_BLOCK, addr, SDIO_Response_Short, SD_R1_RESPONSE);					//发送CMD25,写指令
	if(errorstatus!=SD_OK) return errorstatus;
 	
	timeout = SDIO_DATA_TIMEOUT;
		
	if(DeviceMode==SD_POLLING_MODE) {
		ENTER_CRITICAL();	//关闭中断

		while(!(SDIO->STA&(SDIO_FLAG_TXUNDERR|SDIO_FLAG_DCRCFAIL|SDIO_FLAG_DTIMEOUT|SDIO_STA_DATAEND|SDIO_FLAG_STBITERR))) {
			if(SDIO_GetFlagStatus(SDIO_FLAG_TXFIFOHE) != RESET)	{	//发送区半空,表示至少存了8个字			  
				if((tlen-bytestransferred) < 32) {
					restwords=((tlen - bytestransferred) % 4 == 0) ? ((tlen - bytestransferred) / 4) : ((tlen - bytestransferred) / 4 + 1);
					for(count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
						SDIO->FIFO=*tempbuff;
				} else {
					for(count = 0; count < 8; count++)
						SDIO->FIFO = *(tempbuff+count);
					tempbuff += 8;
					bytestransferred += 32;
				}
				timeout=0X3FFFFFFF;	//写数据溢出时间
			} else {
				if(timeout == 0) return SD_DATA_TIMEOUT;
				timeout--;
			}
		}

		if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {					//数据超时错误											   
			SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
			return SD_DATA_TIMEOUT;
		} else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {	//数据块CRC错误		
			SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
			return SD_DATA_CRC_FAIL;		   
		} else if(SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET) { 	//接收FIFO上溢错误
			SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);
			return SD_TX_UNDERRUN;		 
		} else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) { 	//接收起始位错误		
			SDIO_ClearFlag(SDIO_FLAG_STBITERR);
			return SD_START_BIT_ERR;		 
		}	

		if(SDIO_GetFlagStatus(SDIO_FLAG_DATAEND) != RESET) {					//接收结束			
			if((SDIO_STD_CAPACITY_SD_CARD_V1_1==CardType)||(SDIO_STD_CAPACITY_SD_CARD_V2_0==CardType)||(SDIO_HIGH_CAPACITY_SD_CARD==CardType)) {
				errorstatus = SendCmd(SD_CMD_STOP_TRANSMISSION, 0, SDIO_Response_Short, SD_R1_RESPONSE);	//发送CMD12+结束传输
				if(errorstatus!=SD_OK) return errorstatus;				
			}
		}
		EXIT_CRITICAL();	//开启中断	
		SDIO_ClearFlag(SDIO_STATIC_FLAGS);//清除所有标记

		} else if(DeviceMode==SD_DMA_MODE) {
			TransferError = SD_OK;
			StopCondition = 1;			//多块写,需要发送停止传输指令
			TransferEnd = 0;				//传输结束标置位，在中断服务置1

			SDIO->MASK |= SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | SDIO_MASK_DATAENDIE | SDIO_MASK_TXUNDERRIE | SDIO_MASK_STBITERRIE;	//配置需要的中断 
			SDIO->DCTRL |= SDIO_DCTRL_DMAEN;		 			//DMA使能 

			SD_DMA_Config((uint32_t*)buf, nblks*blksize, DMA_DIR_MemoryToPeripheral);

			while(((DMA2->LISR&(1<<27))==RESET) && timeout) timeout--;	//等待传输完成 
			if(timeout == 0) return SD_DATA_TIMEOUT;	//超时

			timeout = SDIO_DATA_TIMEOUT;
			while((TransferEnd==0) && (TransferError==SD_OK) && timeout) timeout--;
			if(timeout == 0) return SD_DATA_TIMEOUT;	//超时	 
			if(TransferError != SD_OK) return TransferError; 	 
		}
 	SDIO_ClearFlag(SDIO_STATIC_FLAGS);//清除所有标记	
 	
	do {
		cardstate = SD_GetStatus();
	} while((cardstate == SD_CARD_PROGRAMMING) || (cardstate == SD_CARD_RECEIVING)); 
  
	return errorstatus;
}

SDCardState SD_GetStatus(void) { //读取当前卡状态
	if(SendCmd(SD_CMD_SEND_STATUS, (uint32_t) RCA << 16, SDIO_Response_Short, SD_R1_RESPONSE) != SD_OK) //发送CMD16
		return SD_CARD_ERROR;

	return (SDCardState)((SDIO->RESP1 >> 9) & 0x0F);
}

SD_Error SD_ReadDisk(uint8_t *buf, uint32_t sector, uint8_t cnt) {
	SD_Error sta = SD_OK;
	uint8_t n;

	if((uint32_t)buf % 4 != 0) {	//存储空间4字节非对齐
	 	for(n = 0; n < cnt; n++) {
		 	sta = SD_ReadBlock(SDIO_DATA_BUFFER, sector + 512 * n, 512);
			memcpy(buf, SDIO_DATA_BUFFER, 512);
			buf += 512;
		} 
	} else {											//存储空间4字节对齐
		if (cnt == 1) sta = SD_ReadBlock(buf, sector, 512);    	//单个sector
		else sta = SD_ReadMultiBlocks(buf, sector, 512, cnt);		//多个sector  
	}
	return sta;
} 

SD_Error SD_WriteDisk(uint8_t*buf,uint32_t sector,uint8_t cnt) {
	SD_Error sta = SD_OK;
	uint8_t n;

	if((uint32_t)buf % 4 != 0) {	//存储空间4字节非对齐
	 	for(n = 0; n < cnt; n++) {
			memcpy(SDIO_DATA_BUFFER, buf, 512);
		 	sta = SD_WriteBlock(SDIO_DATA_BUFFER, sector + 512 * n, 512);
			buf += 512;
		} 
	} else {											//存储空间4字节对齐
		if (cnt == 1) sta = SD_WriteBlock(buf, sector, 512);    //单个sector
		else sta = SD_WriteMultiBlocks(buf, sector, 512, cnt);	//多个sector  
	}
	return sta;
}
