#ifdef  TROUT_RUNMODE_PROC
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include "runmode.h"
#include <linux/cpu.h>


static struct proc_dir_entry *proc_trout_dir;
static struct proc_dir_entry * runmode_node;
static int run_mode = SMP_MODE;
static int lock=0;


#define TAG "[runmode]"

static void mode_deal(int mode)
{
	 if(mode == SMP_MODE){
        	while(!cpu_up(1));
	 }else if(mode==SINGLE_CPU_MODE){
	 	while(!cpu_down(1));
	 }
}
/*
 *  ======== atoi ========
 *  Purpose:
 *      This function converts strings in decimal or hex format to integers.
 */
static s32 atoi(char *psz_buf)
{
	char *pch = psz_buf;
	s32 base = 10;

	while (isspace(*pch))
		pch++;

	if (*pch == '-' || *pch == '+') {
		base = 10;
		pch++;
	} else if (*pch && tolower(pch[strlen(pch) - 1]) == 'h') {
		base = 16;
	}

	return simple_strtoul(pch, NULL, base);
}

static int runmode_node_write(struct file *file, const char *buf,
			    unsigned long count, void *data)
{
	char temp_buf[32]={0,};
	int mode = 0 ;
	if (count <= 0 || count >32  || copy_from_user(temp_buf, buf, count) != 0){
			printk(TAG "%s:wrong param\n",__FUNCTION__);
			return -EFAULT;
	}

	mode = atoi(temp_buf);

	if(mode > MODE_START && mode < MODE_END && run_mode!=mode){
		//mode_deal(mode);
		run_mode = mode;
	}else{
		printk(TAG "%s:wrong param mode =%d\n",__FUNCTION__,mode);
		return -EFAULT;
	}
	//printk(TAG "run_mode = %d\n",run_mode);
	
	return count;
}

static int runmode_node_read(char *page,
			   char **start,
			   off_t offset, int count, int *eof, void *data)
{
	int ret = - EINVAL;
	ret = sprintf(page,"%d\n",run_mode);
	 *eof = 1;
	return ret;
}

int  trout_get_runmode(void)
{
	return run_mode;
}

void trout_runmode_lock(void)
{
	while(lock != 0) ;
	lock++;
}

void trout_runmode_unlock(void)
{
	lock--;
}

int  __init trout_runmode_init(void)
{
	int ret=0;
	printk(TAG " trout_runmode_init\n");
	/*proc_trout_dir = proc_mkdir(TROUT_RUNMODE_DIR, NULL);
	if (proc_trout_dir == NULL){
		 ret = -EACCES;
		 printk(TAG "make %s failed\n",TROUT_RUNMODE_DIR);
		 goto err_dir;
	}*/
	
	/* Install the config entries */
	runmode_node= create_proc_entry(TROUT_RUNMODE_NODE,
					  S_IFREG | S_IRUGO | S_IWUSR  | S_IROTH, NULL);

	if (runmode_node) {
		runmode_node->write_proc = runmode_node_write;
		runmode_node->read_proc = runmode_node_read;
		runmode_node->data = NULL;
	} else {
		printk(TAG " make %s failed\n",TROUT_RUNMODE_NODE);
		ret =  -ENOMEM;
		goto err_conf;
        }

	return ret;

err_conf:
	//remove_proc_entry(TROUT_RUNMODE_DIR,NULL);
err_dir:
	return ret;
}

void  __exit trout_runmode_exit(void)
{
	remove_proc_entry(TROUT_RUNMODE_NODE, NULL);
	//remove_proc_entry(TROUT_RUNMODE_DIR,NULL);
	printk(TAG " trout_runmode_exit\n");
}

#ifdef TROUT_RUNMODE_PROC_MODULE
module_init(trout_runmode_init);
module_exit(trout_runmode_exit);


MODULE_DESCRIPTION("Trout: run mode ");
MODULE_AUTHOR("bing.li <bing.li@spreadstrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("trout_runmode");
MODULE_VERSION("V1.0");

#endif

#endif
