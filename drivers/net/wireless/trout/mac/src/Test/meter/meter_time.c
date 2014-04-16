#ifdef TROUT_METER
#include "meter_time.h"
#include <linux/time.h>
#include <linux/slab.h>

//-------------------private function------------------------------
extern unsigned long volatile __jiffy_data jiffies;
static void * jiffies_malloc(void)
{
	return kzalloc(sizeof(unsigned long),GFP_KERNEL);
}

static void  jiffies_free(void * data)
{
	if(data) kfree(data);
}

static int  jiffies_get(void * data)
{
	unsigned long * time = (unsigned long*)data;
	*time = jiffies;
	return 0;
}

static int jiffies_sub(void * a_data,void * b_data,void *c_data)
{
	unsigned long * a = (unsigned long*)a_data;
	unsigned long * b = (unsigned long*)b_data;
	unsigned long * c = (unsigned long*)c_data;
	*c = *a - *b;
	return 0;
}

static int jiffies_greater(void * a_data,void * b_data)
{
	unsigned long * a = (unsigned long*)a_data;
	unsigned long * b = (unsigned long*)b_data;
	return (*a >= *b);
}

static int jiffies_zero(void * a_data)
{
	unsigned long * a = (unsigned long*)a_data;
	return (*a == 0);
}

static  char * jiffies_print(void * data,char * buf)
{
	unsigned long * a = (unsigned long*)data;
	sprintf(buf,"%5lu (jf)",*a);
	return buf;
}

//-------------s driver--
static void * s_malloc(void)
{
	return kzalloc(sizeof( unsigned long),GFP_KERNEL);
}

static void  s_free(void * data)
{
	if(data) kfree(data);
}

static int  s_get(void * data)
{
	 unsigned long * time = (unsigned long*)data;
	*time = jiffies/HZ;
	return 0;
}

static int s_sub(void * a_data,void * b_data,void *c_data)
{
	unsigned long * a = (unsigned long*)a_data;
	unsigned long * b = (unsigned long*)b_data;
	unsigned long * c = (unsigned long*)c_data;
	*c = (*a - *b);
	return 0;
}

static int s_greater(void * a_data,void * b_data)
{
	unsigned long * a = (unsigned long*)a_data;
	unsigned long * b = (unsigned long*)b_data;
	return  (*a >=*b);
}

static int s_zero(void * a_data)
{
	unsigned long * a = (unsigned long*)a_data;
	return  (*a ==0);
}

static  char * s_print(void * data,char * buf)
{
	unsigned long * a = (unsigned long*)data;
	sprintf(buf,"%5lu (s)",*a);
	return buf;
}

//-------------ms driver--
static void * us_malloc(void)
{
	return kzalloc(sizeof(struct timeval),GFP_KERNEL);
}

static void  us_free(void * data)
{
	if(data) kfree(data);
}

static int  us_get(void * data)
{
	struct timeval * time = (struct timeval*) data;
	do_gettimeofday(time);
	return 0;
}

static int us_sub(void * a_data,void * b_data,void * c_data)
{
	struct timeval * a = (struct timeval*) a_data;
	struct timeval * b = (struct timeval*) b_data;
	struct timeval * c = (struct timeval*) c_data;
	c->tv_usec = a->tv_sec*1000000 + a->tv_usec - b->tv_sec*1000000-b->tv_usec;
	return 0;
}

static int us_zero(void * a_data)
{
	struct timeval * a = (struct timeval*) a_data;
	return (a->tv_sec*1000000 + a->tv_usec) == 0;
}

static  char * us_print(void * data,char * buf)
{
	struct timeval * time = (struct timeval*) data;
	sprintf(buf,"%5lu (us)",time->tv_sec*1000000+time->tv_usec);
	return buf;
}

static int us_greater(void * a_data,void * b_data)
{
	struct timeval * a = (struct timeval*) a_data;
	struct timeval * b = (struct timeval*) b_data;
	return (a->tv_sec*1000000000 + a->tv_usec) >= (b->tv_sec*1000000000-b->tv_usec);
}

//-------------us driver---
static void * ns_malloc(void)
{
	return kzalloc(sizeof(struct timespec),GFP_KERNEL);
}

static void  ns_free(void * data)
{
	kfree(data);
}

static int  ns_get(void * data)
{
	struct timespec time = current_kernel_time();
	*(struct timespec*)data = time;
	return 0;
}

static int  ns_zero(void * data)
{
	struct timespec * a = (struct timespec*) data;
	return (a->tv_sec*1000000000 + a->tv_nsec ) == 0;
}

static int ns_sub(void * a_data,void * b_data,void * c_data)
{
	struct timespec * a = (struct timespec*) a_data;
	struct timespec * b = (struct timespec*) b_data;
	struct timespec * c = (struct timespec*) c_data;
	c->tv_nsec = a->tv_sec*1000000000 + a->tv_nsec - b->tv_sec*1000000000-b->tv_nsec;
	return 0;
}

static int ns_greater(void * a_data,void * b_data)
{
	struct timespec * a = (struct timespec*) a_data;
	struct timespec * b = (struct timespec*) b_data;
	return (a->tv_sec*1000000000 + a->tv_nsec) >= (b->tv_sec*1000000000-b->tv_nsec);
}

static  char * ns_print(void * data,char * buf)
{
	struct timespec * time = (struct timespec*) data;
	sprintf(buf,"%5lu (ns)",time->tv_sec*1000000000+time->tv_nsec);
	return buf;
}

//-------------------interface api---------------------------------
static meter_time_driver driver_list[]={
		{"jiffies",jiffies_malloc,jiffies_free,jiffies_get,jiffies_sub,jiffies_print,jiffies_greater,jiffies_zero},
		{"s",s_malloc,s_free,s_get,s_sub,s_print,s_greater,s_zero},
		{"us",us_malloc,us_free,us_get,us_sub,us_print,us_greater,us_zero},
		{"ns",ns_malloc,ns_free,ns_get,ns_sub,ns_print,ns_greater,ns_zero},
		{"none",NULL,NULL,NULL,NULL,NULL,NULL,NULL,},
};
pmeter_time meter_time_malloc(enum METER_PRECISION precision)
{
	pmeter_time time= kzalloc(sizeof(meter_time),GFP_ATOMIC);
	if(time == NULL || precision >= MAX_PRECISION) return NULL ;
	time->precision = precision;
	time->driver = &driver_list[precision];
	if(time->driver->malloc){
		time->private = time->driver->malloc();
	}else{
		time->private = NULL;
	}
	return time;
}

void meter_time_free(pmeter_time time)
{
	if(time == NULL) return ;
	if(time->driver &&  time->driver->free){
		time->driver->free(time->private);
	}
	kfree(time);
}

int meter_time_get(pmeter_time time)
{
	if(time && time->driver && time->driver->get) {
		return time->driver->get(time->private);
	}
	return -1;
}

int meter_time_greater(pmeter_time a,pmeter_time b)
{
	if(a &&  b && a->driver->greater) {
		return a->driver->greater(a->private,b->private);
	}
	return 0;
}

int meter_time_zero(pmeter_time a)
{
	if(a && a->driver->zero) {
		return a->driver->zero(a->private);
	}
	return 0;
}

int meter_time_sub(pmeter_time a,pmeter_time b,pmeter_time * c)
{
	if(a->driver==NULL || b->driver==NULL || (a->driver != b->driver) || (a->driver->sub==NULL)) return -1;
	*c = meter_time_malloc(a->precision);
	if(*c){
		 return a->driver->sub(a->private,b->private,(*c)->private);
	}else{
		return -1;
	}
}

char * meter_time_print(pmeter_time time,char * buf)
{
	if(time && time->driver && time->driver->print && buf) {
		return time->driver->print(time->private,buf);
	}
	return NULL;
}
#endif