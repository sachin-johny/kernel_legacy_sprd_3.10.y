#ifndef _SIM_SHARE_MEM_H
#define _SIM_SHARE_MEM_H
#include "typedef.h"
#include "share_mem_info.h"
#include "trout_rx.h"
#include "trout_tx.h"

struct share_mem {
	struct rx_mem_struct rx_mem;
	struct tx_mem_struct tx_mem;
};

extern struct share_mem *get_share_mem(void);

static inline UWORD32 offset_of_trout_addr(const void *trout_addr)
{
	return (UWORD32)trout_addr - SHARE_MEM_BEGIN;
}

// 调用者需检查 @trout_addr的有效性
static inline void *__from_trout_addr(void *trout_addr)
{
	struct share_mem *mem = get_share_mem();

	return (void *)mem + offset_of_trout_addr(trout_addr);
}

// 调用者需保证 @addr位于share_mem中,否则返回值为无效trout addr
static inline void *__to_trout_addr(void *addr)
{
	struct share_mem *mem = get_share_mem();

	return (void *)SHARE_MEM_BEGIN + (addr - (void *)mem);
}

static inline int is_valid_trout_addr(void *trout_addr)
{
	UWORD32 addr = (UWORD32)trout_addr;
    
	return (SHARE_MEM_BEGIN <= addr)
		&& (addr < WIFI_MEM_END)
		&& is_multiple_of_4(addr);
}

static inline int is_valid_trout_addr_range(void *trout_addr, UWORD32 len)
{
	len = (len + 3) & ~3U;
	return is_valid_trout_addr(trout_addr)
		&& is_valid_trout_addr(trout_addr + len - 4);
}

extern void *from_trout_addr(void *trout_addr);

extern void struct_checker(void);
#endif

