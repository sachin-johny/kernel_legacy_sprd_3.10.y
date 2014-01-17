/*******************************************************************************
 ********************************************************************************
 * Revison
  initial version
 *******************************************************************************/
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/file.h>
#include <linux/fs.h>

#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <linux/kthread.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include "trout_bt_debug.h"
#include <net/bluetooth/bluetooth.h>

#define TROUT_COMMON_BASE   (0x4000U << 2)
#define TCBA	TROUT_COMMON_BASE
#define TCBE	(TCBA + (0xFF << 2))

#define TROUT_SYS_BASE		(0x0)

extern unsigned int host_read_trout_reg(unsigned int reg_addr);
extern unsigned int host_write_trout_reg(unsigned int val, unsigned int reg_addr);
extern unsigned int host_read_bt_ram(void *host_addr, void *bt_addr, unsigned int length);

static char debug_mask = DEBUG_PRINT;

/* This function swaps the byte order in the given 32-bit word in BIG_ENDIAN */
/* mode to convert to Little Endian format. For LITTLE_ENDIAN mode no change */
/* is required.                                                              */
INLINE UWORD32 convert_to_le(UWORD32 val)
{
#ifdef BIG_ENDIAN
	return SWAP_BYTE_ORDER_WORD(val);
#else
	return val;  //trout is little_endian
#endif /* LITTLE_ENDIAN */

}


void dump_comm_regs(struct file *filp)
{
   unsigned int i,v;

	if(NULL == filp)
	{
		BT_DBG("*********************COMMON REGS**************************\n");
		for(i = 0; i < 0xff; i++){
			v = convert_to_le(host_read_trout_reg((UWORD32)(TCBA + (i << 2))));
		if(i % 8 == 0)
			BT_DBG("\n");
			BT_DBG("[%04X], %08X, \n", i, v);
		}
		BT_DBG("\n");
	}
	else
	{
		int size = 0;
		char buff[100]={'\0'};
		int  write_len=0;
		BT_DBG("*********************COMMON REGS**************************\n");
		sprintf(buff,"%s","************COMMON REGS*****************\n");
		write_len=strlen(buff);
		size = filp->f_op->write(filp,buff,write_len,&filp->f_pos);

		for(i = 0; i < 0xff; i++){

			v = convert_to_le(host_read_trout_reg((UWORD32)(TCBA + (i << 2))));

			if(i % 8 == 0)
				BT_DBG("\n");
			BT_DBG("[%04X], %08X, ", i, v);

			mem_set(buff, '\0', 100);
			sprintf(buff,"0x%04X, 0x%08X, \n",i,v);
			write_len=strlen(buff);
			size = filp->f_op->write(filp,buff,write_len,&filp->f_pos);
		}
		BT_DBG("\n");

	}
}

void dump_sys_regs(struct file *filp)
{
   unsigned int i,v;

	if(NULL == filp)
	{
		BT_DBG("***********************SYS REGS***************************\n");
		for(i = 0; i < 0xff; i++){
			v = convert_to_le(host_read_trout_reg((UWORD32)((i << 2))));
			if(i % 8 == 0)
				BT_DBG("\n");
		   BT_DBG("[%04X], %08X,\n", i, v);
		}
	   BT_DBG("\n");
	}
	else
	{
		int size = 0;
		char buff[100]={'\0'};
		int  write_len=0;

		BT_DBG("*********************** SYS REGS***************************\n");
	   sprintf(buff,"%s","************ SYS REGS*****************\n");

		write_len=strlen(buff);
		size = filp->f_op->write(filp,buff,write_len,&filp->f_pos);

		for(i = 0; i < 0xff; i++){
			v = convert_to_le(host_read_trout_reg((UWORD32)((i << 2))));
			if(i % 8 == 0)
				BT_DBG("\n");
			BT_DBG("DDD[%04X], %08X", i, v);

			mem_set(buff, 0, 100);
			sprintf(buff,"0x%04X, 0x%08X, \n",i,v);
			write_len=strlen(buff);
			size = filp->f_op->write(filp,buff,write_len,&filp->f_pos);
		}
	}
}

void dump_rf_regs(struct file *filp)
{
	unsigned int i, v;

	if(NULL == filp)
	{
		int size = 0;
		char buff[100]={'\0'};

		BT_DBG("***********************RF REGS***************************\n");

		for(i = 0; i < 0x4EF; i++){
			v = convert_to_le(host_read_trout_reg((UWORD32)((0x1000 + i) << 2)));
			if(i % 8 == 0)
				BT_DBG("\n");
			BT_DBG("[%04X], %08X, \n", i, v);
		}
		BT_DBG("\n");
	}
	else
	{

		int size = 0;
		char buff[100]={'\0'};
		int  write_len=0;

		BT_DBG("***********************RF REGS***************************\n");
		sprintf(buff,"%s","************RF REGS*****************\n");
		write_len=strlen(buff);

		size = filp->f_op->write(filp,buff,write_len,&filp->f_pos);

		for(i = 0; i < 0x4EF; i++){
			v = convert_to_le(host_read_trout_reg((UWORD32)((0x1000 + i) << 2)));

			if(i % 8 == 0)
				BT_DBG("\n");
			BT_DBG("[%04X], %08X, ", i, v);

			mem_set(buff, 0, 100);
			sprintf(buff,"0x%04X, 0x%08X, \n",i,v);
			write_len=strlen(buff);
			size = filp->f_op->write(filp,buff,write_len,&filp->f_pos);
		}
		BT_DBG("\n");
	}
}

void dump_phy_regs()
{
	BT_DBG("*********errrrrrrrrrrrrrrrrrrrrr***************************\n");
	BT_DBG("\n");
   //Trout_Print_Sys_Common_Reg();
}

static void dump_allregs(void)
{

	struct file *filp;
	mm_segment_t fs;

	char* filename = "/data/bt_regsdata.txt"; // "data/bt_regsdata.txt";

	static int count = 0;
	int size = 0;
	char buff[100]={'\0'};
	int write_len=0;


	BT_DBG("dump_allregs: open times =%d...\n",count);

	if(count==0 || count > 10)
	{
		count=0;
		filp = filp_open(filename, O_CREAT |O_RDWR|O_TRUNC, 0644);
		BT_DBG("dump_allregs: open times ==%d, clear file...\n",count);
	}
   else
	{
		filp = filp_open(filename, O_CREAT |O_RDWR|O_APPEND, 0644);
	}

	if(IS_ERR(filp))
	{
		BT_DBG("dump_allregs: open error...\n");
		return;
	}

	sprintf(buff,"dump_allregs: open times = %d \n",count++);
	write_len=strlen(buff);

	fs=get_fs();
	set_fs(KERNEL_DS);
	size = filp->f_op->write(filp,buff,write_len,&filp->f_pos);

	//dump_phy_regs(filp); //wifionly

	dump_comm_regs(filp);
	dump_sys_regs(filp);
	dump_rf_regs(filp);
	set_fs(fs);
	filp_close(filp,NULL);

}

static void print_allregs(void)
{

	BT_DBG("print_allregs: open times ...\n");
	//dump_phy_regs(filp); //wifionly
	dump_comm_regs(NULL);
	dump_sys_regs(NULL);
	dump_rf_regs(NULL);

}

// enable trout bt ram
static WORD32 trout_bt_ram_en(void)
{
	WORD32 ret = 0;

	ret = host_write_trout_reg(convert_to_le((UWORD32)0x0001), (UWORD32)COM_REG_ARM7_CTRL);
	if(ret != 0)
		BT_DBG("dumpcode trout_bt_ram_en COM_REG_ARM7_CTRL write error: %d\n",ret);
	mdelay(1);

    //caisf 20121116, 0x008c
	ret = host_write_trout_reg(convert_to_le((UWORD32)0x018c), (UWORD32)SYS_REG_CLK_CTRL1);
	//ret = host_write_trout_reg(convert_to_le((UWORD32)0x1241c), (UWORD32)SYS_REG_CLK_CTRL1);
	if(ret != 0)
		BT_DBG("dumpcode trout_bt_ram_en SYS_REG_CLK_CTRL1 write error: %d\n",ret);
	mdelay(1);

	ret = host_write_trout_reg(convert_to_le((UWORD32)0x007f), (UWORD32)COM_REG_BT_IRAM_EN);
	if(ret != 0)
		BT_DBG("dumpcode trout_bt_ram_en COM_REG_BT_IRAM_EN write error: %d\n",ret);
	mdelay(1);

	return ret;
}

// disable trout bt ram
static WORD32 trout_bt_ram_dis(void)
{
    WORD32 ret = 0;

    //caisf 20121116, 0x0113
    //ret = host_write_trout_reg(convert_to_le((UWORD32)0x0114), (UWORD32)SYS_REG_CLK_CTRL1);
    ret = host_write_trout_reg(convert_to_le((UWORD32)0x1241c), (UWORD32)SYS_REG_CLK_CTRL1);
    if(ret != 0)
        BT_DBG("dumpcode trout_bt_ram_dis SYS_REG_CLK_CTRL1 write error: %d\n",ret);
    mdelay(1);

    ret = host_write_trout_reg(convert_to_le((UWORD32)0x0000), (UWORD32)COM_REG_BT_IRAM_EN);
    if(ret != 0)
        BT_DBG("dumpcode trout_bt_ram_dis COM_REG_BT_IRAM_EN write error: %d\n",ret);
    mdelay(1);

    ret = host_write_trout_reg(convert_to_le((UWORD32)0x0002), (UWORD32)COM_REG_ARM7_CTRL);
    if(ret != 0)
        BT_DBG("dumpcode trout_bt_ram_dis COM_REG_ARM7_CTRL write error: %d\n",ret);
    mdelay(1);

    return ret;
}


INLINE void hex_dump(UWORD8 *info, UWORD8 *str, UWORD32 len)
{
	if(str == NULL || len == 0)
		return;

	  UWORD32  i = 0;
	  BT_DBG("dump %s, len: %d; data:\n",info,len);
	  for(i = 0; i<len; i++)
		{
			if(((UWORD8 *)str+i)==NULL)
				break;
			BT_DBG("%02x ",*((UWORD8 *)str+i));
			if((i+1)%16 == 0)
				BT_DBG("\n");
		}
		BT_DBG("\n");
}

static void dump_ram_code(struct file *filp)
{
   UWORD8 *ramcode_buffer = NULL;
	int size = 0;

   BT_DBG("=============dump BT firmware code to file  start=============\n");

   trout_bt_ram_en();

   ramcode_buffer = (unsigned char *)kmalloc(DUMP_BT_CODE_LEN, GFP_KERNEL);

   if(!ramcode_buffer)
	{
		BT_DBG("trout %s: unable to alloc bt_ram_code!\n", __FUNCTION__);
		return;
	}

   memset(ramcode_buffer, 0, DUMP_BT_CODE_LEN);

   host_read_bt_ram((void *)ramcode_buffer,(void *)rBT_RAM_CODE_SPACE, DUMP_BT_CODE_LEN);

   //hex_dump("dump bt_code", ramcode_buffer, 512);

   size = filp->f_op->write(filp,ramcode_buffer,DUMP_BT_CODE_LEN,&filp->f_pos);

   trout_bt_ram_dis();

   kfree(ramcode_buffer);

   BT_DBG("=============dump BT firmware code to file  end: write size =%d =============\n", size);
}


static void dump_ramcode(void)
{

	struct file *filp;
	mm_segment_t fs;

	char* filename = "/data/bt_ramcode.bin";

	BT_DBG("dump_ramcode: open times =1...\n");

	filp = filp_open(filename, O_CREAT |O_RDWR|O_TRUNC, 0644);

	if(IS_ERR(filp))
	{
		BT_DBG("dump_ramcode: open error...\n");
		return;
	}

	fs=get_fs();
	set_fs(KERNEL_DS);

	dump_ram_code(filp);

	set_fs(fs);
	filp_close(filp,NULL);

}

/******************************down trout all bt reg**********************************/
static ssize_t trout_downreg_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "trout_debug help:  \n 1: dump_allregs; other: print_allregs.\n");
}

static ssize_t trout_downreg_store(struct kobject *kobj, struct kobj_attribute *attr,
      const char *buf, size_t count)
{
      BT_DBG("trout_debug_store start buf= %s \n", buf);

     debug_mask=buf[0];

     if (debug_mask & DEBUG_PRINT)
        print_allregs();

     if (debug_mask & DEBUG_DUMPBTREG)
        dump_allregs();

     //if (debug_mask & DEBUG_OPENHCIDUMPALL)
     //   open_hcidump_all();

     BT_DBG("trout_debug_store finish.\n");
     return count;
}

/******************************down bt rom code and ram data**********************************/
static ssize_t trout_downmap_show(struct kobject *kobj, struct kobj_attribute *attr,
    char *buf)
{
      return sprintf(buf, "trout_downmap help:  \n 1: dump_ram code.\n");
}

static ssize_t trout_downmap_store(struct kobject *kobj, struct kobj_attribute *attr,
      const char *buf, size_t count)
{
      BT_DBG("trout_dump_ramcode_store start buf= %s \n", buf);

      if (buf[0] & DEBUG_DUMPBTREG)
        dump_ramcode();

      BT_DBG("trout_dump_ramcode_store finish.\n");
      return count;
}

/******************************open hcidump all**********************************/
static bool open_hci_flag=0;
struct semaphore m_hci_sema;

static void open_hcidump_all(bool open_flag)
{
     open_hci_flag=open_flag;
     BT_DBG("trout_hcidump : open_hcidump_all open_flag=%d, open_hci_flag=%d\n",open_flag, open_hci_flag);
}

static ssize_t trout_hcidump_show(struct kobject *kobj, struct kobj_attribute *attr,
    char *buf)
{
       BT_DBG("trout_hcidump: open_hci_flag=%d\n",open_hci_flag);

       down_interruptible(&m_hci_sema);
       return sprintf(buf, "%d\n",open_hci_flag);
}

static ssize_t trout_hcidump_store(struct kobject *kobj, struct kobj_attribute *attr,
      const char *buf, size_t count)
{
      BT_DBG("trout_hcidump store start buf= %s \n", buf);

      if (buf[0] & 0x01)
        open_hcidump_all(true);
      else if(buf[0] & 0x02)
        open_hcidump_all(false);

      BT_DBG("trout_hcidump store finish.\n");

      up(&m_hci_sema);

      return count;
}
/****************************************************************/

static struct kobj_attribute trout_btdebug_attribute =
      __ATTR(trout_btdebug_cmd, 0644, trout_downreg_show, trout_downreg_store);


static struct kobj_attribute trout_downmap_attribute =
      __ATTR(trout_downmap_cmd, 0644, trout_downmap_show, trout_downmap_store);

static struct kobj_attribute trout_hcidump_attribute =
      __ATTR(trout_hcidump_cmd, 0644, trout_hcidump_show, trout_hcidump_store);
/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *attrs[] = {
      &trout_btdebug_attribute.attr,
      &trout_downmap_attribute.attr,
      &trout_hcidump_attribute.attr,
      NULL,  /* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory.  If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
static struct attribute_group attr_group = {
      .attrs = attrs,
};

static struct kobject *trout_btdebug_kobj;


static int __init trout_btdebug_init(void)
{
     int retval;
     /*
      * Create a simple kobject with the name of "kobject_example",
      * located under /sys/kernel/
      *
      * As this is a simple directory, no uevent will be sent to
      * userspace.  That is why this function should not be used for
      * any type of dynamic kobjects, where the name and number are
      * not known ahead of time.
      */
     trout_btdebug_kobj = kobject_create_and_add("trout_debug", kernel_kobj);
     if (!trout_btdebug_kobj)
          return -ENOMEM;

     /* Create the files associated with this kobject */
     retval = sysfs_create_group(trout_btdebug_kobj, &attr_group);
     sema_init(&m_hci_sema,0);

     if (retval)
          kobject_put(trout_btdebug_kobj);

     return retval;
}

static void __exit trout_btdebug_exit(void)
{
     kobject_put(trout_btdebug_kobj);
}

module_init(trout_btdebug_init);
module_exit(trout_btdebug_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("haili");
