#ifdef  TROUT_DEBUG_RW_PROC
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "rw_register.h"
#include "rw_interface.h"
#include "itypes.h"
#include "common.h"
#include "iconfig.h"

#define BUFFER_SIZE 1024
static struct proc_dir_entry *proc_trout_rw_dir;
static struct trout_rw_config rw_config;
static UWORD32 data_buffer[BUFFER_SIZE/sizeof(UWORD32)]={0,};


static int config_mode = BIN_MODE;

module_param(config_mode, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(config_mode, "set the config mode , if mode =1 ,then ,use bin type,if mode=2,then ,use asic type");

extern unsigned int host_read_trout_reg(unsigned int reg_addr);
extern unsigned int host_write_trout_reg(unsigned int val, unsigned int reg_addr);

extern unsigned int host_read_trout_ram(void *host_addr, void *trout_addr,unsigned int length);
extern unsigned int host_write_trout_ram(void *host_addr, void *trout_addr,unsigned int length);

extern unsigned int host_read_bt_ram(void *host_addr, void *bt_addr,unsigned int length);
extern unsigned int host_write_bt_ram(void *bt_addr, void *host_addr,unsigned int length);

extern bool Set_Trout_PowerOn( unsigned int  MODE_ID );
extern bool Set_Trout_PowerOff( unsigned int  MODE_ID );

extern void dump_allregs(unsigned long ix, unsigned long iy);


static void rw_config_init(struct trout_rw_config * conf)
{
	conf->module = REG;
	conf->debug_level = DEBUG_ERROR;
	conf->addr = 0;
	conf->flag = 0;
}

static int rw_config_sprintf(char *s,struct trout_rw_config * conf)
{
	return sprintf(s,"module=%d,"  "addr=%x," "debug_level=%d," "flag=%d\n" ,\
			 conf->module,conf->addr,conf->debug_level,conf->flag );
}

static int config_proc_write(struct file *file, const char *buf,
			    unsigned long count, void *data)
{
	struct trout_rw_config config;
	if(config_mode == BIN_MODE){
		if(count != sizeof(struct trout_rw_config)){
			printk("%s:wrong param\n",__FUNCTION__);
			return  - EINVAL;
		}
		if (copy_from_user(&config, buf, count) != 0)
			return -EFAULT;
		rw_config.flag = config.flag;
		if(config.flag & MODULE_MASK){
			rw_config.module = config.module;
		}
		if(config.flag & ADDR_MASK){
			rw_config.addr = config.addr;
		}
		if(config.flag & DEBUG_MASK){
			rw_config.debug_level = config.debug_level;
		}
		//printk("config_proc_wirte addr  = %d ,module = %d\n",rw_config.addr,rw_config.module);
		return count;
	}else if(config_mode == ASIC_MODE){
		return 0;
	}
	printk("error config mode =%d \n",config_mode);
	return - EINVAL;
}

static int config_proc_read(char *page,
			   char **start,
			   off_t offset, int count, int *eof, void *data)
{
	int ret = - EINVAL;
 	if (offset > 0){
		printk("can't support off read,off must equal to 0 \n");
		*eof = 1;
		return - EINVAL;
	}

	if(BIN_MODE == config_mode){
		memcpy(page,&rw_config,sizeof(rw_config));
		ret = sizeof(rw_config);		
	}else if(ASIC_MODE == config_mode){
		ret =  rw_config_sprintf(page,&rw_config);
	}else{
		printk("error config mode =%d \n",config_mode);
	}
	return ret;
}

static int data_proc_write(struct file *file, const char *buf,
			    unsigned long count, void *data)
{

	if(count > BUFFER_SIZE){
		return -ENOMEM;
	}
	
	if (copy_from_user(data_buffer, buf, count) != 0)
			return -EFAULT;

	switch(rw_config.module){
		case  REG:
			host_write_trout_reg(convert_to_le(data_buffer[0]),rw_config.addr);
			break;
		case WIFI_RAM:
			host_write_trout_ram((void*)data_buffer,(void*)rw_config.addr,count);
			break;
		case BT_RAM:
			host_write_bt_ram((void*)data_buffer,(void*)rw_config.addr,count);
			break;
		case POWER:
			if(data_buffer[0] == 1){
				Set_Trout_PowerOn(rw_config.addr);
			}else if(data_buffer[0] == 0){
				Set_Trout_PowerOff(rw_config.addr);
			}
			break;
		default:
			printk("wrong module type. (%d)\n",rw_config.module);
			return -EFAULT;
	}
	return count;
}

static int data_proc_read(char *page,
			   char **start,
			   off_t offset, int count, int *eof, void *data)
{
	UWORD32 * p = (UWORD32 *)page;
	//printk("data_proc_read : count = %d,offset = %d \n",count,offset);
	//printk("addr = %0x,module=%d,flag=%d\n",rw_config.addr,rw_config.module,rw_config.flag);
	if(count > BUFFER_SIZE){
		return -ENOMEM;
	}
	
	switch(rw_config.module){
		case  REG:
			*p = host_read_trout_reg(rw_config.addr);
			count = sizeof(UWORD32);
			break;
		case WIFI_RAM:
			host_read_trout_ram((void*)page,(void*)rw_config.addr,count);
			break;
		case BT_RAM:
			host_read_bt_ram((void*)page,(void*)rw_config.addr,count);
			break;
		case POWER:
			*page = -1;
			printk("don't support\n");
			break;
		case LEGACY:
			if(rw_config.addr == 8){
				dump_allregs(0,0);
			}else if(rw_config.addr == 2){
				print_test_stats(0);
			}
			*page = 0;
			count = sizeof(UWORD32);
			break;
		default:
			printk("wrong module type. (%d)\n",rw_config.module);
			return -EFAULT;
	}
	return count;
}

static struct proc_dir_entry * config_node,*data_node;

int  __init trout_rw_init(void)
{
	int ret=0;
       
	rw_config_init(&rw_config);
	
	proc_trout_rw_dir = proc_mkdir(TROUT_DIR, NULL);
	if (proc_trout_rw_dir == NULL){
		 ret = -EACCES;
		 printk("make %s failed\n",TROUT_DIR);
		 goto err_dir;
	}
	
	/* Install the config entries */
	config_node= create_proc_entry(TROUT_CONFIG_NODE,
					 S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP, proc_trout_rw_dir);

	if (config_node) {
		config_node->write_proc = config_proc_write;
		config_node->read_proc = config_proc_read;
		config_node->data = NULL;
	} else {
		printk("make %s failed\n",TROUT_CONFIG_NODE);
		ret =  -ENOMEM;
		goto err_conf;
        }

	/* Install the data entries */
	data_node= create_proc_entry(TROUT_DATA_NODE,
					 S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP, proc_trout_rw_dir);

	if (data_node) {
		data_node->write_proc = data_proc_write;
		data_node->read_proc = data_proc_read;
		data_node->data = NULL;
	} else {
		printk("make %s failed\n",TROUT_DATA_NODE);
		ret =  -ENOMEM;
		goto err_data;
        }

	return ret;
	
err_data:
	remove_proc_entry(TROUT_CONFIG_NODE, proc_trout_rw_dir);
err_conf:
	remove_proc_entry(TROUT_DIR,NULL);
err_dir:
	return ret;
}

void  __exit trout_rw_exit(void)
{
	remove_proc_entry(TROUT_CONFIG_NODE, proc_trout_rw_dir);
	remove_proc_entry(TROUT_DATA_NODE, proc_trout_rw_dir);
	remove_proc_entry(TROUT_DIR,NULL);
}

#ifdef TROUT_DEBUG_RW_PROC_MODULE
module_init(trout_rw_init);
module_exit(trout_rw_exit);


MODULE_DESCRIPTION("Trout: debug tool ,read and write register .");
MODULE_AUTHOR("bing.li <bing.li@spreadstrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("trout_rw");
MODULE_VERSION("V1.0");

#endif

#endif
