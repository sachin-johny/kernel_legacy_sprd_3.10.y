#include "bt_ram_code.h"
#include <linux/kernel.h>

#include <linux/stat.h>
#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/syscalls.h>


#define BT_MIDI_CODE_LEN 21208

#define BT_CODE_MEM_LEN  (42*1024)   // bt ram code size

#define MAX_BT_TMP_PSKEY_FILE_LEN 3072  // pskey_bt.txt file size


// BT ram enable/disable regs
#define COM_REG_ARM7_CTRL           (0x4015 << 2)
#define COM_REG_BT_IRAM_EN          (0x4013 << 2)

#define BT_RAM_CODE_FILE_PATH       "/system/lib/modules/bt_ram_code.bin"
#define BT_PSKEY_PARAM_DEFAULT_FILE_PATH    "/system/lib/modules/pskey.txt"  //sprd,zhaolongting,20130924
#define BT_PSKEY_PARAM_FILE_PATH    "/data/pskey.txt"


#if 1
#if 0
#include <linux/module.h> 
#include <linux/init.h> 
#include <linux/fs.h> 
#include <linux/uaccess.h> 

static char * argv_init[5] = {"init", NULL};
char * envp_init[5] = {"HOME=/", "TEMP=linux", NULL};
static void run_init_process(char *init_filename)
{
	argv_init[0] = init_filename;
	kernel_execve(init_filename, argv_init, envp_init);
}


// loff_t == long long
static loff_t bt_drv_write_file(char *file_path, char *str, size_t str_len, loff_t usr_pos) 
{
    struct file *fp; 
    mm_segment_t fs; 
    loff_t pos = usr_pos; 
	
    printk("%s: enter\n", __FUNCTION__); 
    if(file_path == NULL || str == NULL || str_len <= 0)
    {
        printk("%s: param error! return.\n", __FUNCTION__); 
        return 0;
    }
    
    fp = filp_open(file_path, O_RDWR | O_CREAT, 0644); 
    if (IS_ERR(fp)) {
        printk("%s: create file error\n", __FUNCTION__); 
        return 0; 
    }
    fs = get_fs(); 
    set_fs(KERNEL_DS); 
	
    //pos = 0;
    vfs_write(fp, str, str_len, &pos); 
    printk("%s: pos=%lld\n", __FUNCTION__, pos); 
    //hex_dump(file_path, str, pos);
	

    filp_close(fp, NULL); 
    set_fs(fs); 
    
    return pos; 
}
#endif

// loff_t == long long
static loff_t bt_drv_read_file(char *file_path, char *str, size_t str_len, loff_t usr_pos) 
{
    struct file *fp; 
    mm_segment_t fs; 
    loff_t pos = usr_pos; 
    long error_code = 0;

    printk("trout read file %s: enter, pos=%d\n", __FUNCTION__, pos); 
    if(file_path == NULL || str == NULL || str_len <= 0)
    {
        printk("trout %s: param error! return.\n", __FUNCTION__); 
        return 0;
    }
    printk("trout file is : %s\n", file_path);
  
    fp = filp_open(file_path, O_RDONLY , 0644);
    error_code = IS_ERR(fp);
    printk("trout error code=0x%x\n", error_code);
    
    if(error_code){
    //if (IS_ERR(fp)) {
        printk("trout %s: create file error\n", __FUNCTION__); 
        return 0; 
    }
	printk("trout open file success\n");
	fs = get_fs(); 
    set_fs(KERNEL_DS); 
	
    //pos = 0;
    vfs_read(fp, str, str_len, &pos); 
    printk("trout %s: pos=%lld\n", __FUNCTION__, pos); 
    //hex_dump(file_path, str, pos);
	
	printk("trout close file\n");
    filp_close(fp, NULL); 
    set_fs(fs); 
printk("trout return pos=%d\n", pos);
    return pos;
}
#endif

typedef unsigned char    uint8;
typedef unsigned short   uint16;
typedef unsigned int     uint32;

typedef int            WORD32;
typedef short          WORD16;
typedef char           WORD8;
typedef unsigned int   UWORD32;
typedef unsigned short UWORD16;
typedef unsigned char  UWORD8;
typedef signed char    SWORD8;

#if 0
typedef struct 
{
    uint8   g_dbg_source_sink_syn_test_data;
    uint8   g_sys_sleep_in_standby_supported;
    uint8   g_sys_sleep_master_supported;
    uint8   g_sys_sleep_slave_supported;
    uint32  default_baud_rate;
    uint32  device_class;
    uint32  win_ext;
    uint32  g_aGainValue[6];
    uint32  g_aPowerValue[5];
    uint8   feature_set[16];
    uint8   device_addr[6];   
    uint8   g_sys_sco_by_uart_supported; // true transmit by uart, otherwise by share memory
    uint8   g_sys_uart0_communication_supported; //true use uart0, otherwise use uart1 for debug
    uint8   edr_tx_edr_delay;
    uint8   edr_rx_edr_delay;
    uint8   abcsp_rxcrc_supported;
    uint8   abcsp_txcrc_supported;
    uint32  share_memo_rx_base_addr;
    uint32  share_memo_tx_base_addr;
    uint32  share_memo_tx_packet_num_addr;
    uint32  share_memo_tx_data_base_addr;
    uint32  g_PrintLevel;
    uint16  share_memo_tx_block_length;
    uint16  share_memo_rx_block_length;
    uint16  share_memo_tx_water_mark;
    uint16  share_memo_tx_timeout_value;
    uint16  uart_rx_watermark;
    uint16  uart_flow_control_thld;
    uint32  comp_id;
    uint16  uart_clk_divd;
    uint16  pcm_clk_divd;
}BT_PSKEY_CONFIG_INFO_T;
#endif

typedef struct{
	    uint8   g_dbg_source_sink_syn_test_data;
    uint8   g_sys_sleep_in_standby_supported;
    uint8   g_sys_sleep_master_supported;
    uint8   g_sys_sleep_slave_supported;
    uint32  default_ahb_clk;
    uint32  device_class;
    uint32  win_ext;
    uint32  g_aGainValue[6];
    uint32  g_aPowerValue[5];
    uint8   feature_set[16];
    uint8   device_addr[6];   
    uint8  g_sys_sco_transmit_mode; //0: DMA 1: UART 2:SHAREMEM
    uint8  g_sys_uart0_communication_supported; //true use uart0, otherwise use uart1 for debug
    uint8 edr_tx_edr_delay;
    uint8 edr_rx_edr_delay;
    uint8 abcsp_rxcrc_supported;//true:supported; otherwise not supported
    uint8 abcsp_txcrc_supported;//true:supported; otherwise not supported
    uint32 share_memo_rx_base_addr;
    uint32 share_memo_tx_base_addr;
    uint32 share_memo_tx_packet_num_addr;
    uint32 share_memo_tx_data_base_addr;
    uint32 g_PrintLevel;
    uint16 share_memo_tx_block_length;
    uint16 share_memo_rx_block_length;
    uint16 share_memo_tx_water_mark;
    uint16 share_memo_tx_timeout_value;
    uint16 uart_rx_watermark;
    uint16 uart_flow_control_thld;
    uint32 comp_id;
    uint16  uart_clk_divd;
    uint16  pcm_clk_divd;
     uint16 debug_bb_tx_gain;
     uint16 debug_tx_power;
     uint32 tx_channel_compensate;
     uint16 ctrl_reserved;
     uint16 reserved16;
    uint32  reserved[10];
}BT_PSKEY_CONFIG_INFO_T;

/* bluetooth ram addr, bytes  */
#define rBT_RAM_CODE_SPACE     (0x0)
#define rBT_RAM_PARAMS_SPACE   (0x4050U)


#define down_bt_is_space(c)	(((c) == '\n') || ((c) == ',') || ((c) == '\r') || ((c) == ' ') || ((c) == '{') || ((c) == '}'))
#define down_bt_is_comma(c)	(((c) == ','))
#define down_bt_is_endc(c)	(((c) == '}')) // indicate end of data

/* Macros to swap byte order */
#define SWAP_BYTE_ORDER_WORD(val) ((((val) & 0x000000FF) << 24) + \
                                   (((val) & 0x0000FF00) << 8)  + \
                                   (((val) & 0x00FF0000) >> 8)   + \
                                   (((val) & 0xFF000000) >> 24))
#define INLINE static __inline

#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN
#endif

INLINE UWORD32 convert_to_le(UWORD32 val)
{
#ifdef BIG_ENDIAN
    return SWAP_BYTE_ORDER_WORD(val);
#endif /* BIG_ENDIAN */

#ifdef LITTLE_ENDIAN
    return val;
#endif /* LITTLE_ENDIAN */
}


INLINE void hex_dump(UWORD8 *info, UWORD8 *str, UWORD32 len)
{
    if(str == NULL || len == 0)
        return;
    
    if(1)
    {
		UWORD32  i = 0;
		printk("dump %s, len: %d; data:\n",info,len);
		for(i = 0; i<len; i++)
		{
			if(((UWORD8 *)str+i)==NULL)
				break;
			printk("%02x ",*((UWORD8 *)str+i));
			if((i+1)%16 == 0)
				printk("\n");
		}
		printk("\n");
	}
}


extern unsigned int host_read_trout_reg(unsigned int reg_addr);
extern unsigned int host_write_trout_reg(unsigned int val, unsigned int reg_addr);

extern unsigned int host_write_bt_ram(void *bt_addr, void *host_addr, UWORD32 length);
extern unsigned int host_read_bt_ram(void *host_addr, void *bt_addr, unsigned int length);

// to auto adapte to bt ram code file size
unsigned long get_file_size(const char *path)
{
    struct kstat statbuff;

    mm_segment_t old_fs=get_fs();  // backup fs
    set_fs(KERNEL_DS);
    vfs_stat(path,&statbuff);
    set_fs(old_fs);  // restore fs
    printk("get_file_size .block_size=%d, .size=%d \n", statbuff.blksize, statbuff.size);
    return statbuff.size;
}


static char *down_bt_strstrip(char *s)
{
    size_t size;
    
    size = strlen(s);
    if (!size)
        return s;

    while (*s && down_bt_is_space(*s))
        s++;
    
    return s;
}

// find first comma(char ',') then return the next comma'next space ptr.
static char *down_bt_skip_comma(char *s)
{
    size_t size;
    
    size = strlen(s);
    if (!size)
        return s;

    while(*s)
    {
        if(down_bt_is_comma(*s))
        {
            
            return ++s;
        }
        if(down_bt_is_endc(*s))
        {
            printk("end of buff, str=%s\n",s);
            return s;
        }
        s++;
    }
    
    return s;
}

void trout_reset_arm_ctrl_reg(void)
{
    int ret = 0;
    
    ret = host_write_trout_reg(convert_to_le((UWORD32)0x0003), (UWORD32)COM_REG_ARM7_CTRL);
    if(ret != 0)
        printk("%s: COM_REG_ARM7_CTRL write 0x3 error: %d\n",__FUNCTION__,ret);

    printk("%s: write: 0x3\n",__FUNCTION__);
    mdelay(1);

    ret = host_write_trout_reg(convert_to_le((UWORD32)0x0002), (UWORD32)COM_REG_ARM7_CTRL);
    if(ret != 0)
        printk("%s: COM_REG_ARM7_CTRL write 0x2 error: %d\n",__FUNCTION__,ret);

    printk("%s: write: 0x2\n",__FUNCTION__);
    mdelay(1);
}

/*wanl add for bt*/
const unsigned char bt_boot_code[]=
{
	0x06,0x00,0x00,0xea,0x08,0x00,0x00,0xea,0x07,0x00,0x00,0xea,0x06,0x00,0x00,0xea,
    0x05,0x00,0x00,0xea,0x04,0x00,0x00,0xea,0x03,0x00,0x00,0xea,0x02,0x00,0x00,0xea,
    0x28,0x10,0x9f,0xe5,0x28,0x20,0x9f,0xe5,0x00,0x20,0x81,0xe5,0xff,0xff,0xff,0xea,
    0x20,0x40,0x9f,0xe5,0x20,0x50,0x9f,0xe5,0x00,0x60,0x95,0xe5,0x00,0x00,0x56,0xe3,
    0xfc,0xff,0xff,0x1a,0x00,0x40,0x85,0xe5,0x01,0x40,0x84,0xe2,0xf9,0xff,0xff,0xea,
    0x10,0x01,0x50,0x50,0x82,0x14,0x00,0x00,0x44,0x33,0x22,0x11,0x00,0x01,0x01,0x20,
};
/*wanl add end*/

// enable trout bt ram 
WORD32 trout_bt_ram_en(void)
{
    WORD32 ret = 0;

    ret = host_write_trout_reg(convert_to_le((UWORD32)0x0001), (UWORD32)COM_REG_ARM7_CTRL);
    if(ret != 0)
        printk("trout_bt_ram_en COM_REG_ARM7_CTRL write error: %d\n",ret);
    mdelay(1);
#if 0    
    //caisf 20121116, 0x008c
    ret = host_write_trout_reg(convert_to_le((UWORD32)0x018c), (UWORD32)SYS_REG_CLK_CTRL1);
    if(ret != 0)
        printk("trout_bt_ram_en SYS_REG_CLK_CTRL1 write error: %d\n",ret);
    mdelay(1);
#endif
    ret = host_write_trout_reg(convert_to_le((UWORD32)0x007f), (UWORD32)COM_REG_BT_IRAM_EN);
    if(ret != 0)
        printk("trout_bt_ram_en COM_REG_BT_IRAM_EN write error: %d\n",ret);
    mdelay(1);

    return ret;
}

// disable trout bt ram 
WORD32 trout_bt_ram_dis(void)
{
    WORD32 ret = 0;
#if 0    
    //caisf 20121116, 0x0113
    ret = host_write_trout_reg(convert_to_le((UWORD32)0x0114), (UWORD32)SYS_REG_CLK_CTRL1);
    if(ret != 0)
        printk("trout_bt_ram_dis SYS_REG_CLK_CTRL1 write error: %d\n",ret);
    mdelay(1);
#endif
    ret = host_write_trout_reg(convert_to_le((UWORD32)0x0000), (UWORD32)COM_REG_BT_IRAM_EN);
    if(ret != 0)
        printk("trout_bt_ram_dis COM_REG_BT_IRAM_EN write error: %d\n",ret);
    mdelay(1);

    ret = host_write_trout_reg(convert_to_le((UWORD32)0x0002), (UWORD32)COM_REG_ARM7_CTRL);
    if(ret != 0)
        printk("trout_bt_ram_dis COM_REG_ARM7_CTRL write error: %d\n",ret);
    mdelay(1);

    return ret;
}

static int count_g_u32 = 0;
static int count_g_u16 = 0;
static int count_g_u8 = 0;

#define get_one_item(buff)  (simple_strtoul(buff, NULL, 0))

static int get_one_digit(UWORD8 *ptr, UWORD8 *data)
{
    int count = 0;

    //hex_dump("get_one_digit 10 ptr",ptr,10);
    if(*ptr=='0' && (*(ptr+1)=='x' || *(ptr+1)=='X'))
    {
        memcpy(data, "0x", 2);
        ptr += 2;
        data += 2;
        
        while(1)
        {
            if((*ptr >= '0' && *ptr <= '9') || (*ptr >= 'a' && *ptr <= 'f') || (*ptr >= 'A' && *ptr <= 'F'))
            {
                *data = *ptr;
                printk("char:%c, %c\n", *ptr, *data);
                data++;
                ptr++;
                count++;
            }
            else
            {
                //printk("break 1; ptr=%s\n",ptr);
                break;
            }
        }
    }
    else
    {
        while(1)
        {
            if(*ptr >= '0' && *ptr <= '9')
            {
                *data = *ptr;
                printk("char:%c, %c\n", *ptr, *data);
                data++;
                ptr++;
                count++;
            }
            else
            {
                //printk("break 2; ptr=%s\n",ptr);
                break;
            }
        }
    }
    
    //printk("%s: data=%s, len=%d\n",__FUNCTION__,data,count);
    return count;
}

static UWORD32 get_next_u32(UWORD8 **p_buff)
{
    UWORD32  val_u32  = 0;
    UWORD8 *ptr = 0;
    UWORD8 data[10];
    UWORD32 data_len = 0;
    ++count_g_u32;
    
    if(p_buff == NULL || *p_buff == NULL)
    {
        if(p_buff)
            printk("%s: Error occur! *p_buff == null!\n",__FUNCTION__);
        else
            printk("%s: Error occur! p_buff == null!\n",__FUNCTION__);
        return 0;
    }
    ptr = *p_buff;
    
    ptr = down_bt_strstrip(ptr);

    //printk("before parser:%s;\n",ptr);

    memset(data,'\0',sizeof(data));
    data_len = get_one_digit(ptr, data);

    //hex_dump("get_one_digit ok", data, data_len);
/*
    if(!data_len)
    {
        printk("get_one_digit error! ptr=%s\n",ptr);
        BUG_ON(1);
    }
    */
#if 1
    printk("get_one_item(data) = 0x%x\n",(UWORD32)get_one_item(data));
    val_u32 = get_one_item(data);
#else   
    if(*data=='0' && (*(data+1)=='x' || *(data+1)=='X'))
    {
        UWORD8 tmp_val[4];
        
        data_len -= 2;
        memcpy(tmp_val, data+2, data_len);
        printk("tmp_val=%s\n", tmp_val);
        
        str_2_hex(tmp_val, data_len);
        val_u32 = (unsigned int)tmp_val;
        printk(" tmp_val=0x%x\n", (UWORD32)tmp_val);
        printk(" 1 val_u32=0x%x\n",val_u32);
    }
    else
    {
        UWORD32 count = data_len;
        int i = 0;
        int tmp_data = 1;
        int val_digit = 0;
        UWORD8 *p_data = data;
        
        printk("count=%d, data=%s",count,p_data);

        while(count > 0)
        {
            tmp_data = 1;
            if(count-1 > 0)
            {
                //printk("if 1 count=%d\n",count);
                for(i=0; i<(count-1); i++)
                {
                    tmp_data *= 10;
                    //printk("for 1 tmp_data=%d\n",tmp_data);
                }
            }
            
            val_digit = char_2_digit(*p_data);
            printk("val digit=%d, quan=%d\n",val_digit,tmp_data);
            val_u32 += val_digit*tmp_data;
            printk("tmp val = %d\n",val_u32);
            
            p_data++;
            count--;
        }

        printk(" 2 val_u32=%d\n",val_u32);
    }
#endif

    printk("%s: here:%x\n",__FUNCTION__,val_u32);

    ptr = down_bt_skip_comma(ptr);
    *p_buff = ptr;
    //printk("after parser:%s;\n",ptr);

    return val_u32;
}

static UWORD16 get_next_u16(UWORD8 **p_buff)
{
    UWORD32  val_u16  = 0;
    UWORD8 *ptr = 0;
    UWORD8 data[10];
    UWORD32 data_len = 0;
    ++count_g_u16;
    
    if(p_buff == NULL || *p_buff == NULL)
    {
        if(p_buff)
            printk("%s: Error occur! *p_buff == null!\n",__FUNCTION__);
        else
            printk("%s: Error occur! p_buff == null!\n",__FUNCTION__);
        return 0;
    }
    ptr = *p_buff;
    
    ptr = down_bt_strstrip(ptr);

    //printk("before parser:%s;\n",ptr);

    memset(data,'\0',sizeof(data));
    data_len = get_one_digit(ptr, data);

    //hex_dump("get_one_digit ok", data, data_len);
/*
    if(!data_len)
    {
        printk("get_one_digit error! ptr=%s\n",ptr);
        BUG_ON(1);
    }
    */

    printk("get_one_item(data) = 0x%x\n",(UWORD16)get_one_item(data));
    val_u16 = get_one_item(data);

    printk("%s: here:%x\n",__FUNCTION__,val_u16);

    ptr = down_bt_skip_comma(ptr);
    *p_buff = ptr;
    //printk("after parser:%s;\n",ptr);

    return val_u16;
}

#if 0
static UWORD8 get_next_u8(UWORD8 **p_buff)
{
    UWORD8  val_u8  = 0;
    UWORD8 *ptr = 0;

    if(p_buff == NULL || *p_buff == NULL)
    {
        printk("%s: Error occur! p_buff == null!\n",__FUNCTION__);
        return 0;
    }
    ptr = *p_buff;
    
    ptr = down_bt_strstrip(ptr);
    
    val_u8 = get_one_item(ptr);
    printk("%s: here:%x\n",__FUNCTION__,val_u8);

    ptr = down_bt_skip_comma(ptr);
    *p_buff = ptr;
    
    return val_u8;
}
#else
static UWORD8 get_next_u8(UWORD8 **p_buff)
{
    UWORD32  val_u8  = 0;
    UWORD8 *ptr = 0;
    UWORD8 data[10];
    UWORD32 data_len = 0;
    ++count_g_u8;
    
    if(p_buff == NULL || *p_buff == NULL)
    {
        if(p_buff)
            printk("%s: Error occur! *p_buff == null!\n",__FUNCTION__);
        else
            printk("%s: Error occur! p_buff == null!\n",__FUNCTION__);
        return 0;
    }
    ptr = *p_buff;
    
    ptr = down_bt_strstrip(ptr);

    //printk("before parser:%s;\n",ptr);

    memset(data,'\0',sizeof(data));
    data_len = get_one_digit(ptr, data);

    //hex_dump("get_one_digit ok", data, data_len);
    /*
    if(!data_len)
    {
        printk("get_one_digit error! ptr=%s\n",ptr);
        BUG_ON(1);
    }
    */

    printk("get_one_item(data) = 0x%x\n",(UWORD8)get_one_item(data));
    val_u8 = get_one_item(data);

    printk("%s: here:%x\n",__FUNCTION__,val_u8);

    ptr = down_bt_skip_comma(ptr);
    *p_buff = ptr;
    //printk("after parser:%s;\n",ptr);

    return val_u8;
}
#endif

#if 0
int parser_pskey_info(UWORD8 *buff, UWORD32 buff_len, 
                            BT_PSKEY_CONFIG_INFO_T *p_params)
{
    UWORD8 *tmp_buff = buff;
    int i = 0;
    
    tmp_buff = strstr(tmp_buff, "}");
    //printk("read 1 this: %s", tmp_buff);
    tmp_buff++;
    tmp_buff = strstr(tmp_buff, "{");
    //printk("read 2 this: %s", tmp_buff);
    tmp_buff++;

    p_params->g_dbg_source_sink_syn_test_data = get_next_u8(&tmp_buff);
    p_params->g_sys_sleep_in_standby_supported = get_next_u8(&tmp_buff);
    p_params->g_sys_sleep_master_supported = get_next_u8(&tmp_buff);
    p_params->g_sys_sleep_slave_supported = get_next_u8(&tmp_buff);
    p_params->default_baud_rate = get_next_u32(&tmp_buff); 
    p_params->device_class = get_next_u32(&tmp_buff); 
    p_params->win_ext = get_next_u32(&tmp_buff); 
    
    for(i=0; i<6; i++)
    {
        p_params->g_aGainValue[i] = get_next_u32(&tmp_buff);
    }
    
    for(i=0; i<5; i++)
    {
        p_params->g_aPowerValue[i] = get_next_u32(&tmp_buff);
    }
    
    for(i=0; i<16; i++)
    {
        p_params->feature_set[i] = get_next_u8(&tmp_buff);
    }
    
    for(i=0; i<6; i++)
    {
        p_params->device_addr[i] = get_next_u8(&tmp_buff);
    }
    
    p_params->g_sys_sco_by_uart_supported = get_next_u8(&tmp_buff); 
    p_params->g_sys_uart0_communication_supported = get_next_u8(&tmp_buff); 
    p_params->edr_tx_edr_delay = get_next_u8(&tmp_buff); 
    p_params->edr_rx_edr_delay = get_next_u8(&tmp_buff); 
    p_params->abcsp_rxcrc_supported = get_next_u8(&tmp_buff); 
    p_params->abcsp_txcrc_supported = get_next_u8(&tmp_buff); 
    p_params->share_memo_rx_base_addr = get_next_u32(&tmp_buff); 
    p_params->share_memo_tx_base_addr = get_next_u32(&tmp_buff); 
    p_params->share_memo_tx_packet_num_addr = get_next_u32(&tmp_buff); 
    p_params->share_memo_tx_data_base_addr = get_next_u32(&tmp_buff); 
    p_params->g_PrintLevel = get_next_u32(&tmp_buff); 
    p_params->share_memo_tx_block_length = get_next_u16(&tmp_buff); 
    p_params->share_memo_rx_block_length = get_next_u16(&tmp_buff); 
    p_params->share_memo_tx_water_mark = get_next_u16(&tmp_buff); 
    p_params->share_memo_tx_timeout_value = get_next_u16(&tmp_buff); 
    p_params->uart_rx_watermark = get_next_u16(&tmp_buff); 
    p_params->uart_flow_control_thld = get_next_u16(&tmp_buff); 
    p_params->comp_id = get_next_u32(&tmp_buff); 
    p_params->uart_clk_divd = get_next_u16(&tmp_buff); 
    p_params->pcm_clk_divd = get_next_u16(&tmp_buff); 

    printk("leave out tmp_buff=%s\n", tmp_buff);
    hex_dump("p_params",(void *)p_params,sizeof(BT_PSKEY_CONFIG_INFO_T));

    return 0;
}
#endif

#if 0
/*wanl add for bt*/
const unsigned char wl_bt_boot_code[]=
{
	0x06,0x00,0x00,0xea,0x08,0x00,0x00,0xea,0x07,0x00,0x00,0xea,0x06,0x00,0x00,0xea,
    0x05,0x00,0x00,0xea,0x04,0x00,0x00,0xea,0x03,0x00,0x00,0xea,0x02,0x00,0x00,0xea,
    0x28,0x10,0x9f,0xe5,0x28,0x20,0x9f,0xe5,0x00,0x20,0x81,0xe5,0xff,0xff,0xff,0xea,
    0x20,0x40,0x9f,0xe5,0x20,0x50,0x9f,0xe5,0x00,0x60,0x95,0xe5,0x00,0x00,0x56,0xe3,
    0xfc,0xff,0xff,0x1a,0x00,0x40,0x85,0xe5,0x01,0x40,0x84,0xe2,0xf9,0xff,0xff,0xea,
    0x10,0x01,0x50,0x50,0x82,0x14,0x00,0x00,0x44,0x33,0x22,0x11,0x00,0x01,0x01,0x20,
};

static void wl_trout_bt_ram_en(void)
{
	host_write_trout_reg(1, (0x4015)<<2);
	host_write_trout_reg(0x18c, (0x21)<<2);
	host_write_trout_reg(0x7f, (0x4013)<<2);
}
static void wl_trout_bt_ram_dis(void)
{
	host_write_trout_reg(0x114, (0x21)<<2);
	host_write_trout_reg(0, (0x4013)<<2);
	host_write_trout_reg(0x2, (0x4015)<<2);
}
/*wanl add end*/

void wanl_down_bt_code(void)
{
    int ret, i;
    
	/*wanl add for bt*/
    wl_trout_bt_ram_en();
	for(i=0;i<sizeof(wl_bt_boot_code)/4;i++)
	{
		host_write_trout_reg(*((unsigned int *)(wl_bt_boot_code+i*4)), (0x5000+i)<<2);	
	}
	mdelay(10);
	i = host_read_trout_reg((0x5000)<<2);
	printk("[trout fm] 0x5000 %x\n",i);
	i = host_read_trout_reg((0x5001)<<2);
	printk("[trout fm] 0x5001 %x\n",i);
	for(i=0;i<sizeof(wl_bt_boot_code)/4;i++)
	{
		ret = host_read_trout_reg((0x5000+i)<<2);	
	    printk("%08x ",ret);
		if (!(i%8))
			printk("\n");
	}
	wl_trout_bt_ram_dis();
	mdelay(100);
	i = host_read_trout_reg((0x4040)<<2);
	printk("[trout fm] 0x4040 %x\n",i);
	/*wanl add end*/
}
#endif

int download_bt_code(void)
{
    static unsigned char *bt_ram_code = NULL;
    //unsigned char *bt_ram_params = NULL;
    WORD32 ret = 0;
    WORD32 i = 0;
    WORD32 tmplen = 0;

    loff_t bt_code_len = 0;
    loff_t bt_code_pos = 0;

    host_write_trout_reg((0x4b2), (0x7<<2));
    loff_t bt_pskey_len = 0;
    loff_t bt_pskey_pos = 0;
    UWORD8 *tmp_buff = 0;
     int  code_size = 0;
    int use_def_psky = 0;

    ret = host_write_trout_reg(convert_to_le((UWORD32)0x102f304f), (UWORD32)(0x4040<<2));
    printk("=====1 reread 0x4040 = 0x%x ====\n",host_read_trout_reg((UWORD32)(0x4040<<2)));
    printk("=============upload BT firmware code to trout enable ram=============\n");
    trout_bt_ram_en();

    ret = host_write_trout_reg(convert_to_le((UWORD32)0x1a2a3a4a), (UWORD32)(0x4040<<2));
    printk("=====2 reread 0x4040 = 0x%x ====\n",host_read_trout_reg((UWORD32)(0x4040<<2)));

    // download bt ram code
    printk("=============upload BT firmware code to trout start=============\n");
#if 0
    bt_ram_code = (unsigned char *)kmalloc(BT_MIDI_CODE_LEN+8, GFP_KERNEL);
	if(!bt_ram_code)
	{
	    printk("trout unable to alloc bt_ram_code! \n");
        return -1;
    }
    
    memset(bt_ram_code, 0, 8*sizeof(unsigned char));
    memcpy((bt_ram_code+8), (void *)midi, BT_MIDI_CODE_LEN);

    ret = host_write_bt_ram((void *)rBT_RAM_CODE_SPACE, (void *)(bt_ram_code+8), 
                BT_MIDI_CODE_LEN);
    if(ret < 0)
        printk("trout download_bt_code rBT_RAM_CODE_SPACE write error: %d\n",ret);
    mdelay(1);
#else
    code_size = get_file_size(BT_RAM_CODE_FILE_PATH);
    printk("test code_size = %d\n", code_size);
    if(NULL == bt_ram_code){
        printk("bt_ram_code == NULL \n");
        //bt_ram_code = (unsigned char *)kmalloc(BT_CODE_MEM_LEN+8, GFP_KERNEL);
        bt_ram_code = (unsigned char *)kmalloc(code_size+8, GFP_KERNEL);
	    if(!bt_ram_code){
	        printk("trout %s: unable to alloc bt_ram_code!\n", __FUNCTION__);
            goto down_ram_code_error;
        }
        printk("bt_ram_code kmalloc success \n");
    }
    
    //memset(bt_ram_code, 0x0, BT_CODE_MEM_LEN+8);
    memset(bt_ram_code, 0x0, code_size+8);

    //bt_code_len = bt_drv_read_file(BT_RAM_CODE_FILE_PATH, bt_ram_code+8, BT_CODE_MEM_LEN, bt_code_pos);
    bt_code_len = bt_drv_read_file(BT_RAM_CODE_FILE_PATH, bt_ram_code+8, code_size, bt_code_pos);
    printk("trout bt_code_len=%d\n", bt_code_len);
    if(bt_code_len <= 0)
    {
        printk("trout %s-%d: read file:%s ERROR!\n",__FUNCTION__,__LINE__,BT_RAM_CODE_FILE_PATH);
        goto down_ram_code_error; 
    }

    ret = host_write_bt_ram((void *)rBT_RAM_CODE_SPACE, (void *)(bt_ram_code+8), bt_code_len);
    if(ret < 0){
        printk("trout download_bt_code rBT_RAM_CODE_SPACE write error: %d\n",ret);
        goto down_ram_code_error;
    }
//    mdelay(10);
    mdelay(1);
#endif


    // download bt pskey config parameters
    printk("=============upload BT pskey config parameters to trout start=============\n");
#if 0
    bt_ram_params = (unsigned char *)kmalloc(sizeof(BT_PSKEY_CONFIG_INFO_T)+8, GFP_KERNEL);
	if(!bt_ram_params)
	{
	    printk("unable to alloc bt_ram_params! \n");
        return -1;
    }
    
    memset(bt_ram_params, 0, 8*sizeof(unsigned char));
    memcpy((bt_ram_params+8), (void *)(&g_bt_pskey_param), sizeof(BT_PSKEY_CONFIG_INFO_T));

    ret = host_write_bt_ram((void *)rBT_RAM_PARAMS_SPACE, (void *)(bt_ram_params+8), 
                sizeof(BT_PSKEY_CONFIG_INFO_T));
    if(ret < 0)
        printk("download_bt_code rBT_RAM_PARAMS_SPACE write error: %d\n",ret);
    mdelay(1);
#else
    //bt_ram_params = (unsigned char *)kmalloc(sizeof(BT_PSKEY_CONFIG_INFO_T)+8, GFP_KERNEL);
	//if(!bt_ram_params)
	//{
	    //printk("%s: unable to alloc bt_ram_params! \n", __FUNCTION__);
        //goto down_ram_code_error;
    //}
    
    //memset(bt_ram_params, 0, 8*sizeof(unsigned char));
    //p_bt_ram_params = (BT_PSKEY_CONFIG_INFO_T *)(bt_ram_params + 8);

    // alloc tmp buff for parser file struct
#if 0
    tmp_buff = (unsigned char *)kmalloc(MAX_BT_TMP_PSKEY_FILE_LEN+1, GFP_KERNEL);
	if(!tmp_buff)
	{
	    printk("trout %s: unable to alloc tmp_buff! \n", __FUNCTION__);
        goto down_pskey_error;
    }
#endif

    // read pskey from file
    tmp_buff = bt_ram_code; 
    memset(tmp_buff, 0x0, MAX_BT_TMP_PSKEY_FILE_LEN+1);

    // read pskey from file
    bt_pskey_len = bt_drv_read_file(BT_PSKEY_PARAM_FILE_PATH, tmp_buff, 
            MAX_BT_TMP_PSKEY_FILE_LEN, bt_pskey_pos);
    printk("trout tmp_buff:%s\n",tmp_buff);
    printk("trout pskey_len=%d\n", bt_pskey_len);
    if(bt_pskey_len <= 0)
    {
        printk("%s-%d: read file:%s ERROR!\n",__FUNCTION__,__LINE__,
            BT_PSKEY_PARAM_FILE_PATH);
#if 0
        goto down_ram_code_done; //  no pskey file. 
#else 
	      
	      //  no pskey file.
        bt_pskey_len = bt_drv_read_file(BT_PSKEY_PARAM_DEFAULT_FILE_PATH, tmp_buff,
            MAX_BT_TMP_PSKEY_FILE_LEN, bt_pskey_pos);
        use_def_psky = 1;
        if(bt_pskey_len <= 0)
        {
            printk("%s-%d: read default file:%s ERROR!\n",__FUNCTION__,__LINE__,
            BT_PSKEY_PARAM_FILE_PATH);
            goto down_pskey_error;
        }
#endif
    }
    
#if 0    
    // parser file struct
    ret = parser_pskey_info(tmp_buff, bt_pskey_len, p_bt_ram_params);
    if(ret < 0)
    {
        printk("%s-%d: parser_pskey_info ERROR!\n",__FUNCTION__,__LINE__);
        goto down_pskey_error;
    }

    printk("%s: count call func: 32:%d, 16:%d, 8:%d, all:%d\n",__FUNCTION__, 
        count_g_u32,count_g_u16,count_g_u8,count_g_u32+count_g_u16+count_g_u8);
#endif


#if 1
	ret = host_write_bt_ram((void *)rBT_RAM_PARAMS_SPACE, (void *)tmp_buff, sizeof(BT_PSKEY_CONFIG_INFO_T));
    if(ret < 0)
        printk("trout download_bt_code rBT_RAM_PARAMS_SPACE write error: %d\n",ret);
    mdelay(10);
#else
	tmplen = sizeof(BT_PSKEY_CONFIG_INFO_T)/4;
	printk("p_bt_ram_params tmplen = %d (%d)\n",tmplen,sizeof(BT_PSKEY_CONFIG_INFO_T)/4);
	for(i=0;i<tmplen;i++)
	{
		host_write_trout_reg(*((UWORD32 *)((void *)p_bt_ram_params+i*4)), (void *)((rBT_RAM_PARAMS_SPACE+i)<<2));	
	}
#endif

#if 0
    if(tmp_buff)
        kfree(tmp_buff);
    tmp_buff = NULL;
#endif

#endif


down_ram_code_done:
    // it has a bug, so not free the memory.
   #if 0
    if(bt_ram_code)
        kfree(bt_ram_code);
    bt_ram_code = NULL;
   #endif
    printk("=============upload BT firmware code to trout free 1=============\n");

    //if(bt_ram_params)
        //kfree(bt_ram_params);
    //bt_ram_params = NULL;
    printk("=============upload BT firmware code to trout free 2=============\n");

    printk("=============upload BT firmware code to trout disable ram=============\n");
    trout_bt_ram_dis();

    printk("=============upload BT code to trout end=============\n");
#if 1//sprd,zhaolongting,20130924
    if(1 == use_def_psky)
    {
        printk("download the default pskey!!!");
        return -1;
    }

#endif
    return 0;

down_ram_code_error:
    //trout_bt_ram_dis();
 #if 0 
    if(bt_ram_code)
        kfree(bt_ram_code);
    bt_ram_code = NULL; 
#endif
    printk("=============upload BT firmware code down_ram_code_error over=============\n");
    return -1;

down_pskey_error:
    //trout_bt_ram_dis();
    
#if 0
    if(bt_ram_code)
        kfree(bt_ram_code);
    bt_ram_code = NULL;

    //if(bt_ram_params)
        //kfree(bt_ram_params);
    //bt_ram_params = NULL;
    
    if(tmp_buff)
        kfree(tmp_buff);
    tmp_buff = NULL;
#endif
    printk("=============upload BT firmware code down_pskey_error over=============\n");  
    return -1;
    
}






