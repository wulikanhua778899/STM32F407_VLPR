#ifndef _MEM_MANAGER_H
#define _MEM_MANAGER_H

#include <stdint.h>

#ifndef NULL
#define NULL 0
#endif

#define MEM_BANK		2	//定义支持的MEM总数

#define MEMSRAM	0	//内部主要SRAM
#define MEMCCM	1	//内核耦合存储器CCM

//SRAM
#define MEMSRAM_BLOCK_SIZE	32  	  																		//内存块大小 32Byte
#define MEMSRAM_MAX_SIZE		100 * 1024  																//最大管理内存 100KB
#define MEMSRAM_ALLOC_TABLE_SIZE	MEMSRAM_MAX_SIZE / MEMSRAM_BLOCK_SIZE	//内存表大小 3200Byte																																
		 
//CCM,仅CPU可以访问
#define MEMCCM_BLOCK_SIZE		32  	  																		//内存块大小 32Byte
#define MEMCCM_MAX_SIZE			60 * 1024  																	//最大管理内存 60KB
#define MEMCCM_ALLOC_TABLE_SIZE		MEMCCM_MAX_SIZE / MEMCCM_BLOCK_SIZE		//内存表大小 1920Byte

//内存管理控制器
struct malloc_cortol_struct {
	void (*init)(uint8_t);				//内存初始化方法
	uint8_t (*perused)(uint8_t);	//内存使用率方法
	uint8_t *membase[MEM_BANK];		//内存池
	uint16_t *memmap[MEM_BANK];		//内存管理状态表
	uint8_t memrdy[MEM_BANK];			//内存管理是否就绪
};
extern struct malloc_cortol_struct malloc_cortol;
																																	
void InitMemManager(uint8_t memx);
void MemManager_Fill(void *s, uint8_t c, uint32_t num);
uint8_t MemManager_Perused(uint8_t memx);
void* MemManager_Malloc(uint8_t memx, uint32_t size);
void MemManager_free(uint8_t memx, void *paddr);

#endif
