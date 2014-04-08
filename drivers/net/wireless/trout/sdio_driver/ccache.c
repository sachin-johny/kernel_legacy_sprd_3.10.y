#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include "trout2_sdio.h"
#include "ccache.h"

#define CDEBUG 0

static ccache_t ccache;

#if CDEBUG
static void hex_dump(unsigned char *info, unsigned char *str, unsigned int len)
{
	unsigned int  i = 0;

    if(str == NULL || len == 0)
        return;

	printk("[ccache]\tdump %s, len: %d; data:\n",info,len);
	for(i = 0; i<len; i++)
	{
		if(i%16==0){
			printk("[ccache]\t");
		}
		if(((unsigned char *)str+i)==NULL)
			break;
		printk("%02x ",*((unsigned char *)str+i));
		if((i+1)%16 == 0){
			printk("\n");
		}
	}
	printk("\n");
}

static void ccache_print(pccache_t cache)
{
	printk("[ccache]: start_addr = 0x%x,end_addr=0x%x,data_lenth =%d,max_size=%d \n",cache->start_addr ,cache->end_addr,\
		cache->data_lenth,cache->max_size);
	hex_dump("buffer",cache->buffer,cache->data_lenth);
}

#endif


static int _ccache_write_flash(pccache_t cache)
{
	int ret = 0,length = cache->end_addr - cache->start_addr;
#if CDEBUG
	ccache_print(cache);
#endif
	if(length <=0 ) return 0;
	dma_map_single(NULL, (void *)cache->buffer, length, DMA_TO_DEVICE);
	ret = host_write_trout_ram(cache->start_addr, cache->buffer, length);
	ccache_reset(cache);
	return ret;
}


static unsigned int _ccache_write(pccache_t cache,unsigned int trout_addr, unsigned char * host_addr,unsigned int length)
{
	unsigned int offset = trout_addr-cache->start_addr;
	memcpy(cache->buffer+offset,host_addr,length);
	if(cache->end_addr == cache->start_addr || cache->end_addr == 0xffffffff){
		cache->end_addr = trout_addr + length;
	}else if (cache->end_addr<trout_addr + length){
		cache->end_addr = trout_addr + length;
	}
	return length;
}

unsigned int ccache_write(pccache_t cache,void *trout_addr, void *host_addr,unsigned int length)
{
	unsigned ret = 0;
	if(cache == NULL || trout_addr == NULL || host_addr == NULL || length<=0 ){
		printk("[ccache]: ccache_write:wrong param\n");
		return 0;
	}

	if((unsigned int )trout_addr < cache->start_addr || length>cache->max_size){
		return host_write_trout_ram(trout_addr,host_addr,length);
	}
	
	mutex_lock(&cache->lock);
	
	if((unsigned int )trout_addr + length > cache->start_addr+cache->max_size){
		_ccache_write_flash(cache);
		cache->start_addr = (unsigned int ) trout_addr;
		cache->end_addr = cache->start_addr;
	}
	
	ret = _ccache_write(cache,(unsigned int )trout_addr,host_addr,length);
	
	mutex_unlock(&cache->lock);
	return ret ;
}

int ccache_write_flash(pccache_t cache)
{
	int ret = 0;
	mutex_lock(&cache->lock);
	ret = _ccache_write_flash(cache);
	mutex_unlock(&cache->lock);
	return ret;
}

void ccache_reset(pccache_t cache)
{
	cache->start_addr = 0xffffffff;
	cache->end_addr = cache->start_addr;
	memset(cache->buffer,0,cache->max_size);
}

pccache_t ccache_create(unsigned int start_addr,unsigned int size)
{
	if(mutex_trylock(&ccache.monopoly) == 0){
		return NULL;
	}
	mutex_lock(&ccache.lock);
	ccache_reset(&ccache);
	ccache.start_addr = start_addr;
	ccache.end_addr = start_addr;
	mutex_unlock(&ccache.lock);
	return &ccache;
}

void ccache_destry(pccache_t cache)
{
	if(cache){
		ccache_write_flash(cache);
	}
	mutex_unlock(&ccache.monopoly);
}

int ccache_init(void)
{
	int len = MAC_CBUFFER_LENGTH,retry = 10;
	mutex_init(&ccache.lock);
	mutex_init(&ccache.monopoly);
	ccache.buffer = NULL;

	while(retry && !ccache.buffer && len>0){
		ccache.buffer = kmalloc(len,GFP_KERNEL);
		ccache.max_size = len;
		retry--;
		len-=1024*2;
	}

	if(ccache.buffer == NULL){
		printk("[cache]\tcache_init: kmalloc buffer failed. (%d)\n",len);
		ccache.max_size = 0;
		return 0;
	}
	printk("[cache]\tcache_init: kmalloc buffer len = . (%d)\n",ccache.max_size);
	ccache_reset(&ccache);
	return 1;
}

void ccache_exit(void)
{
	if(ccache.buffer){
		kfree(ccache.buffer);
	}
}

EXPORT_SYMBOL(ccache_write);
EXPORT_SYMBOL(ccache_write_flash);
EXPORT_SYMBOL(ccache_create);
EXPORT_SYMBOL(ccache_destry);
EXPORT_SYMBOL(ccache_reset);

