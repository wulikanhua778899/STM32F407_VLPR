#include <stdint.h>
#include "MemManager.h"

//内存池,32字节对齐
__align(32) uint8_t MemSRAM_Base[MEMSRAM_MAX_SIZE];																		//SRAM
__align(32) uint8_t MemCCM_Base[MEMCCM_MAX_SIZE]		__attribute__((at(0X10000000)));	//CCM

//内存管理表
uint16_t MemSRAM_MapBase[MEMSRAM_ALLOC_TABLE_SIZE];
uint16_t MemCCM_MapBase[MEMCCM_ALLOC_TABLE_SIZE]		__attribute__((at(0X10000000+MEMCCM_MAX_SIZE)));

//内存管理参数	   
const uint32_t MemTableSize[MEM_BANK] = {MEMSRAM_ALLOC_TABLE_SIZE, MEMCCM_ALLOC_TABLE_SIZE};	//内存表大小
const uint32_t MemBlockSize[MEM_BANK] = {MEMSRAM_BLOCK_SIZE, MEMCCM_BLOCK_SIZE};							//内存分块大小
const uint32_t MemSize[MEM_BANK] = {MEMSRAM_MAX_SIZE, MEMCCM_MAX_SIZE};												//内存总大小

//内存管理控制器
struct malloc_cortol_struct malloc_cortol = {
	InitMemManager,										//内存初始化方法
	MemManager_Perused,								//内存使用率方法
	MemSRAM_Base, MemCCM_Base,				//内存池
	MemSRAM_MapBase, MemCCM_MapBase,	//内存管理状态表
	0, 0  		 												//内存管理未就绪
};

uint32_t _Malloc(uint8_t memx, uint32_t size) { //内存分配(内部调用)
	int32_t offset;
	uint32_t i, xmemb, kmemb = 0; 
 
	if(!malloc_cortol.memrdy[memx]) malloc_cortol.init(memx);	//未初始化,先执行初始化 
	if(size == 0) return 0XFFFFFFFF;

	xmemb = size / MemBlockSize[memx];	//获取需要分配的连续内存块数
	if(size % MemBlockSize[memx]) xmemb++;
  
	for(offset = MemTableSize[memx] - 1; offset >= 0; offset--) { //搜索整个内存控制区  		 
		if(!malloc_cortol.memmap[memx][offset]) kmemb++;						//连续空内存块数
		else kmemb = 0;

		if(kmemb == xmemb) {
			for(i = 0; i < xmemb; i++) malloc_cortol.memmap[memx][offset+i] = xmemb;  

			return (offset * MemBlockSize[memx]);	//返回偏移地址  
		}
	}  
	return 0XFFFFFFFF;//未找到符合分配条件的内存块  
}

uint8_t _Free(uint8_t memx, uint32_t offset) { //内存释放(内部调用) 
	uint32_t i, index, nmemb; 

	if(!malloc_cortol.memrdy[memx])	{ //未初始化,先执行初始化	
		malloc_cortol.init(memx);    
		return 1;	//未初始化  
  }
  
	if(offset < MemSize[memx]) {   
		index = offset / MemBlockSize[memx];				//偏移所在内存块号码  
		nmemb = malloc_cortol.memmap[memx][index];	//内存块数
		for(i = 0; i < nmemb; i++) malloc_cortol.memmap[memx][index+i] = 0;  
 
		return 0;  
	} else return 2;	//越界 
}

void InitMemManager(uint8_t memx) {  
	MemManager_Fill(malloc_cortol.memmap[memx], 0, MemTableSize[memx] * 2);	//内存状态表数据清零  
	MemManager_Fill(malloc_cortol.membase[memx], 0,	MemSize[memx]);					//内存池所有数据清零  
	malloc_cortol.memrdy[memx] = 1; 
}

void MemManager_Fill(void *s, uint8_t c, uint32_t num) { //填充内存 
	uint8_t *xs = s;  
	while(num--) *(xs++) = c;  
}	   

uint8_t MemManager_Perused(uint8_t memx) { //获取内存使用率 
	uint32_t i, used = 0;  

	for(i = 0; i < MemTableSize[memx]; i++)   
		if(malloc_cortol.memmap[memx][i]) used++; 

	return (used * 100) / MemTableSize[memx];  
}

void* MemManager_Malloc(uint8_t memx, uint32_t size) { //分配内存(外部调用)  
	uint32_t offset;   
	offset = _Malloc(memx,size);  	   	 	   
	if(offset == 0XFFFFFFFF) return NULL;  
	else return (void*)((uint32_t)malloc_cortol.membase[memx] + offset);  
}

void MemManager_free(uint8_t memx, void *paddr) { //释放内存(外部调用) 
	uint32_t offset;   
	if(paddr == NULL) return; 
 	offset = (uint32_t)paddr - (uint32_t)malloc_cortol.membase[memx];     
  _Free(memx, offset);      
}
