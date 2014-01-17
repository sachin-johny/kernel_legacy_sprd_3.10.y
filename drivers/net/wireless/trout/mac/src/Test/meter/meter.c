#ifdef TROUT_METER
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/slab.h>  
#include <linux/list.h>  
#include<linux/semaphore.h>


#include "meter.h"
#include "meter_time.h"
#include "meter_define.h"

static struct list_head meter_list;
static struct mutex    meter_list_lock;
static struct mutex     meter_get_lock;
static  int g_meter_init = 0;
struct semaphore sem;


//--------------private function------------

//-----------step function-----------------

static pmeter_step malloc_step(const char * name,enum METER_PRECISION percision)
{
	pmeter_step step= kzalloc(sizeof(meter_step_t),GFP_ATOMIC);
	if(step == NULL ) return NULL;
	step->time = meter_time_malloc(percision);
	if(step->time == NULL){
		kfree(step);
		return NULL;
	}
	INIT_LIST_HEAD(&step->node);
	strcpy(step->name,name);
	meter_time_get(step->time);
	return step;
}

static void free_step(pmeter_step step)
{
	if(step == NULL) return;
	meter_time_free(step->time);
	kfree(step);
}

static void add_step(pmeter_t meter,pmeter_step step)
{
	if(step == NULL || meter ==NULL) return;
	
	//mutex_lock(&meter->list_lock);
	list_add_tail(&step->node,&meter->step_list);
	//mutex_unlock(&meter->list_lock);
}

static void del_step(pmeter_t meter,pmeter_step step)
{
	if(step == NULL || meter ==NULL) return;
	
	//mutex_lock(&meter->list_lock);
	list_del(&step->node);
	//mutex_unlock(&meter->list_lock);
}

//---------------meter funtion---------

static void _add_meter(pmeter_t meter)
{
	if(meter == NULL) return;
	list_add_tail(&meter->node,&meter_list);
}

static void add_meter(pmeter_t meter)
{
	if(meter == NULL) return;
	
	//mutex_lock(&meter_list_lock);
	_add_meter(meter);
	//mutex_unlock(&meter_list_lock);
}

static void del_meter(pmeter_t meter)
{
	if(meter == NULL) return;
	
	//mutex_lock(&meter_list_lock);
	list_del(&meter->node);
	//mutex_unlock(&meter_list_lock);
}

static pmeter_t _find_meter(const char * name)
{
	if (name == NULL) return NULL;
	pmeter_t meter = NULL;
	list_for_each_entry(meter, &meter_list, node) {
		if(strcmp(meter->name, name) == 0){
			return meter;
		}
	}
	return NULL;
}

static pmeter_t find_meter(const char * name)
{
	pmeter_t meter = NULL;
	if (name == NULL) return NULL;
	//mutex_lock(&meter_list_lock);
	meter = _find_meter(name);
	//mutex_unlock(&meter_list_lock);
	return meter;
}

static pmeter_t get_meter(const char * name)
{
	//mutex_lock(&meter_get_lock);
	pmeter_t meter = find_meter(name);
	if(meter == NULL) {
		//mutex_unlock(&meter_get_lock);
		return NULL;
	}
	//mutex_lock(&meter->lock);
	//mutex_unlock(&meter_get_lock);
	return meter;
}

static void put_meter(pmeter_t meter)
{
	if(meter == NULL) return NULL;
	//mutex_unlock(&meter->lock);
}

static pmeter_t malloc_meter(const char * name,enum METER_PRECISION percision,enum METER_TYPE type)
{
	pmeter_t meter= kzalloc(sizeof(meter_t),GFP_ATOMIC);
	if(meter == NULL ) return NULL ;
	mutex_init(&meter->list_lock);
	mutex_init(&meter->lock);
	INIT_LIST_HEAD(&meter->step_list);
	meter->start_time = meter_time_malloc(percision);
	meter->end_time = meter_time_malloc(percision);
	INIT_LIST_HEAD(&meter->node);
	strcpy(meter->name,name);
	meter->precision = percision;
	meter->type = type;
	meter_time_get(meter->start_time);
	meter->pre_step = malloc_step("meter_start",meter->precision);
	return meter;
}

static void free_meter(pmeter_t meter)
{
	if(meter == NULL) return;
	pmeter_step step,n = NULL;
	//mutex_lock(&meter->list_lock);
	list_for_each_entry_safe(step,n, &meter->step_list, node) {
		free_step(step);
	}
	//mutex_unlock(&meter->list_lock);
	meter_time_free(meter->start_time);
	meter_time_free(meter->end_time);
	if(meter->pre_step){
		free_step(meter->pre_step);
	}
	kfree(meter);
}

static void count_meter(pmeter_t meter)
{
	if(meter == NULL) return;
	char buf[128] ={0,},buf2[128] ={0,};
	pmeter_time start_time = meter->start_time;
	pmeter_time pre_time = start_time;
	pmeter_step step = NULL;
	pmeter_time step_time=NULL,all_time=NULL;
	int i =1;
	printk("[libing]:========meter : %s ==========\n",meter->name);
	
	//mutex_lock(&meter->list_lock);
	
	list_for_each_entry(step, &meter->step_list, node) {
		meter_time_sub(step->time,pre_time,&step_time);
		meter_time_sub(step->time,start_time,&all_time);
		printk("[libing]:%d:\t%s\t(%s):\t%s\n",i++,meter_time_print(step_time,buf),meter_time_print(all_time,buf2),step->name);
		meter_time_free(step_time);
		meter_time_free(all_time);
		pre_time = step->time;
	}
	
	if(meter_time_sub(meter->end_time,meter->start_time, &all_time) ==0 ){
		printk("[libing]:all time:\t %s\n",meter_time_print(all_time,buf));
	}
	meter_time_free(all_time);
	
	//mutex_unlock(&meter->list_lock);

	printk("[libing]:=========================\n");
}
//--------------public function------------

void  meter_init(void)
{
	mutex_init(&meter_list_lock);
	mutex_init(&meter_get_lock);
	INIT_LIST_HEAD(&meter_list);
	g_meter_init = 1;
	sema_init(&sem,1);
}

void  meter_exit(void)
{
	g_meter_init = 0;
}

int meter_start(const char * name, enum METER_PRECISION percision,enum METER_TYPE type)
{
	int ret = 0;
	if(g_meter_init == 0 || name==NULL) return -1;
	mutex_lock(&meter_list_lock);
	pmeter_t meter = _find_meter(name);
	if(meter == NULL){
		meter = malloc_meter(name,percision,type);
		if(meter){
			_add_meter(meter);
		}else{
			ret = -2;
		}
	}else{
		ret = -3;
	}
	mutex_unlock(&meter_list_lock);
	return ret;
}

int meter_step(const char * mname,const char * fmt,...)
{
	int ret = 0;
	pmeter_step step = NULL;
	pmeter_time step_time=NULL,all_time=NULL;
	char buf[128] ={0,},buf2[128] ={0,};
	static int i=0;
	char sname[256]={0,};
	va_list args;
	va_start(args, fmt);
	vsnprintf(sname, 255, fmt, args);
	va_end(args);
	if(g_meter_init == 0 || mname==NULL || sname==NULL) return -1;
   	 if (down_trylock(&sem))  
    	{  
    		//printk("[libing] -----------------------------down_trylock return--------------------------- \n");
       	 return -1;  
    	}  
	//mutex_lock(&g_lock);
	pmeter_t meter = get_meter(mname);
	if(meter && meter->type == METER_ALL){
		step = malloc_step(sname,meter->precision);
		add_step(meter,step);
	}else if(meter && meter->type == METER_EVERY){
		step = malloc_step(sname,meter->precision);
		if(step){
			meter_time_sub(step->time,meter->pre_step->time,&step_time);
			meter_time_sub(step->time,meter->start_time,&all_time);
			//if(!meter_time_zero(step_time)){
				printk("[libing]:%d:\t%s\t(%s):\t%s\n",i++,meter_time_print(step_time,buf),meter_time_print(all_time,buf2),step->name);
			//}
			meter_time_free(step_time);
			meter_time_free(all_time);
			free_step(meter->pre_step);
			meter->pre_step = step;
		}
	}else{
		printk("libing:not't find meter:%s\n",mname);
		ret = -2;
	}
	put_meter(meter);
	//mutex_unlock(&g_lock);
	up(&sem);
	return ret;
}

int meter_end(const char * name)
{
	int ret = 0;
	if(g_meter_init == 0 || name==NULL) return -1;
	pmeter_t meter = get_meter(name);
	if(meter){
		meter_time_get(meter->end_time);
		if(meter->type == METER_ALL){
			count_meter(meter);
		}
		del_meter(meter);
		put_meter(meter);
		free_meter(meter);
	}else{
		ret = -2;
	}
	return ret;
}

//module_init(meter_init);
//module_exit(meter_exit);
#endif