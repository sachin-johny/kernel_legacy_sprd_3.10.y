#ifndef _CONTINUE_CACHE_H
#define _CONTINUE_CACHE_H
#include <linux/mutex.h>

#define MAC_CBUFFER_LENGTH 32*1024

typedef struct _CCACHE{
	struct mutex lock;
	unsigned int start_addr;//start trout_addr
	unsigned char * buffer;
	unsigned int end_addr;//end trout_addr
	unsigned int max_size;
}ccache_t,*pccache_t;

int ccache_init(void);
void ccache_exit(void);

pccache_t ccache_create(unsigned int start_addr,unsigned int size);
void ccache_destry(pccache_t p);
void ccache_reset(pccache_t p);
unsigned int ccache_write(pccache_t p,void *trout_addr, void *host_addr,unsigned int length);
int ccache_write_flash(pccache_t p);
#endif
