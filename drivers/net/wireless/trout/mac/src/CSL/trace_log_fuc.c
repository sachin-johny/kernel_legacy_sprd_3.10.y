#ifdef TROUT_TRACE_DBG

#include <linux/string.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/linkage.h>
#include <linux/types.h>
#include <linux/timer.h> //内核定时器
#include <linux/slab.h>

#include <stdarg.h>

#include "trace_log_fuc.h"
#include "common.h"
#include "csl_linux.h"

#define LOG_DELAY 2*60 //120s = 2m
#define LOG_BUF_SIZE 8*1024
#define LOG_A_TRACE_MAX_SIZE 1024
#define LOG_BUF_CNT 4
#define LOG_FILE_PATH "/mnt/asec/itm_trace.log"
#define LOG_MAX_SIZE 100 * 1024 * 1024

/*时间戳 | 进程号 | log等级 | 保留 | 文件编号 | 行号 | 参数长度*/
/*4byte    2byte    1byte    1byte  2byte       2byte  2byte   */
#define LOG_HEAD_LEN 14

#define TIME_OFFSET 0
#define PROCESSID_OFFSET 4
#define DEBUGLEVEL_OFFSET 6
#define RESERVED_OFFSET 7
#define FILENUM_OFFSET 8
#define LINENUM_OFFSET 10
#define LENGTH_OFFSET 12

#define CHAR_TYPE 1
#define INT_TYPE  4
#define MULT_TYPE 0

#if 0
#define TRACE_LOG_PRINT(args...) \
	    printk("%s-%d:",__FUNCTION__,__LINE__); \
	    printk(args)
#else
#define	TRACE_LOG_PRINT(args...)
#endif

struct timer_list log_timer; 
struct mutex buf_wr_lock;
struct mutex print2file_lock;

struct buf_dscr
{
	atomic_t buf_flag;

	unsigned char * buf_addr; //buf 首地址
	unsigned char * data; //当前位置
	//chenq add 2012-09-28
	int id;

	struct buf_dscr * pnext;
};

struct buf_dscr * g_buf_head    = NULL;
struct buf_dscr * p_w_file      = NULL;
struct buf_dscr * p_w_buf       = NULL;
struct task_struct *g_pthread_t = NULL;

atomic_t g_empt_buf_cnt = ATOMIC_INIT(LOG_BUF_CNT - 1);

atomic_t g_wbuf_cnt = ATOMIC_INIT(0);
atomic_t g_rbuf_cnt = ATOMIC_INIT(0);
atomic_t g_wall_flag = ATOMIC_INIT(0);

int g_log_bufsize     = 0;
int g_a_trace_size    = 0;
int g_log_delay       = 0;
int g_total_wfile_len = 0;
int g_wfile_over_size = 0;

//chenq add 2012-09-28
static int wbuf_cnt = 0;
static int rbuf_cnt = 1;

struct file * fp = 0; 
mm_segment_t fs; 
loff_t *pos;

extern UWORD32 trout_dbg_mask_get(void); 

int openLogFile(void)
{
	//fp = filp_open(LOG_FILE_PATH, O_WRONLY | O_CREAT | O_APPEND, 0);
	fp = filp_open(LOG_FILE_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0);//re write
	if (IS_ERR(fp)) 
	{ 
		TRACE_LOG_PRINT("create file error\n"); 
		return -1; 
	}
	fs = get_fs(); 
	set_fs(KERNEL_DS); 
	return 0;
}

int closeLogFile(void)
{
	int ret = 0;
	
	ret = filp_close(fp, NULL); //0:失败
	set_fs(fs);
	if(ret == 0)
	{
		TRACE_LOG_PRINT("close fial !!!\n");
	}

	return ret;
}

int getFormatType(unsigned char format)
{
	int type = -1;
	
	switch(format)
	{
		case 's':
		case 'S':
			type = MULT_TYPE;
			break;
        case 'c':
			type = CHAR_TYPE;
			break;
		case 'd':
		case 'i':
		case 'o':	
		case 'u':	
		case 'x':
		case 'X':
		case 'p':
			type = INT_TYPE;
			break;
		default:
			break;
	}
	
	return type;
}


/*时间戳 | 进程号 | log等级 | 保留 | 文件编号 | 行号 | 参数长度*/
/*4byte    2byte    1byte    1byte  2byte       2byte  2byte   */

INLINE void setPrintTime(unsigned char * buf_head)
{
    struct timespec time;
	unsigned int printtime = 0;
	
    //getnstimeofday(&time);
    sprd_nstime_get(&time);
	printtime = (unsigned int)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
	buf_head[TIME_OFFSET] = printtime >> 24;
	buf_head[TIME_OFFSET + 1] = printtime >> 16;
	buf_head[TIME_OFFSET + 2] = printtime >> 8;
	buf_head[TIME_OFFSET + 3] = printtime;
    //TRACE_LOG_PRINT("setPtintTime = %u\n",printtime);
}

INLINE void setProcessID(unsigned char * buf_head)
{
    unsigned int printpid = 0;

	/*printk("current->pid:%u,current->tgid:%u,current->real_parent:%u,current->parent:%u\n",
			(unsigned int)(current->pid),(unsigned int)(current->tgid),(unsigned int)(current->real_parent),(unsigned int)(current->parent));
	*/
	printpid = (unsigned int)(current->pid);
	if( printpid == 0 )
	{
		printpid = (unsigned int)(current->tgid);
		//printk("current->pid == 0,tgid:%d\n",printpid);
	}
	
	buf_head[PROCESSID_OFFSET]     = printpid >> 8;
	buf_head[PROCESSID_OFFSET + 1] = printpid;
	//TRACE_LOG_PRINT("setProcessID = %u\n",printpid);
}

INLINE void setDebugLevel(unsigned char * buf_head,int debuglevel)
{
	buf_head[DEBUGLEVEL_OFFSET] = debuglevel;
	//TRACE_LOG_PRINT("setDebugLevel = %u\n",debuglevel & 0xFF);
}

INLINE void setReserved(unsigned char * buf_head,unsigned char reserved)
{
	buf_head[RESERVED_OFFSET] = reserved;
	//TRACE_LOG_PRINT("setReserved = %u\n",reserved & 0xFF);
}

INLINE void setFileNum(unsigned char * buf_head,int filenum)
{
	buf_head[FILENUM_OFFSET]     = filenum >> 8;
	buf_head[FILENUM_OFFSET + 1] = filenum;
    //TRACE_LOG_PRINT("setFileNum = %u\n",filenum);
}

INLINE void setLineNum(unsigned char * buf_head,int linenum)
{
	buf_head[LINENUM_OFFSET]     = linenum >> 8;
	buf_head[LINENUM_OFFSET + 1] = linenum;
    //TRACE_LOG_PRINT("setLineNum = %u\n",linenum);
}

INLINE void setLogLen(unsigned char * buf_head,int len)
{
	buf_head[LENGTH_OFFSET]     = len >> 8;
	buf_head[LENGTH_OFFSET + 1] = len;
    //TRACE_LOG_PRINT("setLogLen = %u\n",len);
}


INLINE struct buf_dscr * saveLog2Buf(struct buf_dscr * cur_buf,int len)
{
	//static int buf_id = 0;

	if(!atomic_inc_and_test(&cur_buf->buf_flag)) //该buf已被占用，只可能是printLog2File_kthread占用
	{
		atomic_dec(&cur_buf->buf_flag);
		if( atomic_read(&g_empt_buf_cnt) <= 0 )
		{
			TRACE_LOG_PRINT("empt buf cnt < 0, this case impossible!!!\n");
			return NULL;
		}

		if(!atomic_inc_and_test(&cur_buf->pnext->buf_flag))//下一块buf也被占用，不可能发生的事情
		{
			TRACE_LOG_PRINT("empt buf cnt > 0,but next buf is lock,impossible!!!\n");
			atomic_dec(&cur_buf->pnext->buf_flag);
			return NULL;
		}

		#if 0
		if(buf_id == 3)
		{
			buf_id = 0;
		}
		else
		{
			buf_id++;
		}
		printk("use buf_id:%d\n",buf_id);
		#endif

		//chenq add 2012-09-28
		wbuf_cnt++;
		cur_buf->pnext->id = wbuf_cnt;
		atomic_set(&g_wbuf_cnt,wbuf_cnt);
		//printk("will write buf id (case next)%d\n",wbuf_cnt);
		
		atomic_dec(&g_empt_buf_cnt);
		return cur_buf->pnext;
	}
	else if( (cur_buf->data + len) > (cur_buf->buf_addr + LOG_BUF_SIZE) ) //buf 存不下
	{
		atomic_dec(&cur_buf->buf_flag);
		if( atomic_read(&g_empt_buf_cnt) <= 0 )
		{
			TRACE_LOG_PRINT("empt buf cnt < 0, buf use up !!!\n");
			return NULL;
		}

		if(!atomic_inc_and_test(&cur_buf->pnext->buf_flag))//下一块buf也被占用，不可能发生的事情
		{
			TRACE_LOG_PRINT("empt buf cnt > 0,but next buf is lock,impossible!!!\n");
			atomic_dec(&cur_buf->pnext->buf_flag);
			return NULL;
		}

		#if 0
		if(buf_id == 3)
		{
			buf_id = 0;
		}
		else
		{
			buf_id++;
		}
		//printk("use buf_id:%d\n",buf_id);
		#endif

		//chenq add 2012-09-28
		wbuf_cnt++;
		cur_buf->pnext->id = wbuf_cnt;
		atomic_set(&g_wbuf_cnt,wbuf_cnt);
		//printk("will write buf id (case too long)%d\n",wbuf_cnt);
		
		atomic_dec(&g_empt_buf_cnt);
		return cur_buf->pnext;
	}
	else
	{
		//chenq add 2012-09-28
		if(cur_buf->data == cur_buf->buf_addr)
		{
			wbuf_cnt++;
			cur_buf->id = wbuf_cnt;
			atomic_set(&g_wbuf_cnt,wbuf_cnt);
			//printk("will write buf id (normal)%d\n",wbuf_cnt);
		}
		//printk("use buf_id:%d\n",buf_id);
		return cur_buf;
	}
}

void saveLog2BufOver(struct buf_dscr * cur_buf)
{
	if(cur_buf != NULL)
	{
		atomic_dec(&cur_buf->buf_flag);
	}
}

INLINE int inc_in_size(int cur_value,int size)
{
	if( (cur_value + 1) >= size )
	{
		cur_value = 0;
	}
	else
	{
		cur_value++;
	}
	return cur_value;
}


static void time2file(unsigned long data)
{
	/*
	printk("time2file\n");
	printk("current->pid:%u,current->tgid:%u,current->real_parent:%u,current->parent:%u\n",
			(unsigned int)(current->pid),(unsigned int)(current->tgid),(unsigned int)(current->real_parent),(unsigned int)(current->parent));
	*/
	
	//TRACE_LOG_PRINT("time2file\n");
	if(IS_ERR(g_pthread_t))
	{
		TRACE_LOG_PRINT("kthread fuc is err ! Impossible !!!\n");
	}
	else
	{
		TRACE_LOG_PRINT("will wake thread!!!\n");
		//printk("timer will wake thread!!!\n");
		atomic_set(&g_wall_flag,1);
		wake_up_process(g_pthread_t);
	}

	//激活timer
	mod_timer(&log_timer, jiffies+ LOG_DELAY * HZ);
	//TRACE_LOG_PRINT("time2file end\n");
	
	return;
}

INLINE void print2file(void)
{
	static int buf_id = 0;
	static int time_cnt = 0;
	static int max_time_cnt = (LOG_DELAY > 30 * 60) ? 30 * 60 : 30*60/LOG_DELAY;
	int i = 0;

	mutex_lock(&print2file_lock);
	
ACTION:
		//chenq add
		for( i = 0 ; i < LOG_BUF_CNT ; i++ )
		{
			if(p_w_file->id == rbuf_cnt)
			{
				break;
			}
			else
			{
				p_w_file = p_w_file->pnext;
			}
		}
		if(p_w_file->id != rbuf_cnt)
		{
			goto SLEEP;
		}

		if(atomic_read(&g_wall_flag) != 0)
		{
			if( atomic_read(&g_rbuf_cnt) > atomic_read(&g_wbuf_cnt) )
			{
				goto SLEEP;
			}
			else if( atomic_read(&g_rbuf_cnt) == atomic_read(&g_wbuf_cnt) )
			{
				atomic_set(&g_wall_flag,0);
			}
		}
		else
		{
			if( atomic_read(&g_rbuf_cnt) >= atomic_read(&g_wbuf_cnt) )
			{
				goto SLEEP;
			}
		}
		
	
		if(!atomic_inc_and_test(&p_w_file->buf_flag)) //当前被占用
		{
			atomic_dec(&p_w_file->buf_flag);
			TRACE_LOG_PRINT("buf id:%d is in log in,sleep...\n",buf_id);
			goto SLEEP;
		}
		else if(p_w_file->data - p_w_file->buf_addr == 0)
		{
			atomic_dec(&p_w_file->buf_flag);
			time_cnt++;
			TRACE_LOG_PRINT("buf id:%d is empty,sleep...\n",buf_id);
			goto SLEEP;
		}

		time_cnt = 0;

		TRACE_LOG_PRINT("will log out buf id:%d,data_len:%d\n",buf_id,p_w_file->data - p_w_file->buf_addr);
		pos = &(fp->f_pos); 
		vfs_write(fp, p_w_file->buf_addr,p_w_file->data - p_w_file->buf_addr, pos);

		//chenq add for debug
		{
			unsigned char tmp_buf[LOG_HEAD_LEN] = {0};

			setPrintTime(tmp_buf);
			setLogLen(tmp_buf,p_w_file->data - p_w_file->buf_addr);
			
			pos = &(fp->f_pos); 
			vfs_write(fp, tmp_buf,sizeof(tmp_buf), pos);
		}


		g_total_wfile_len += p_w_file->data - p_w_file->buf_addr;
		if( g_total_wfile_len > LOG_MAX_SIZE )
		{
			g_wfile_over_size = 1;
		}

		#if 0
		{
			unsigned char * tmp_p = p_w_file->buf_addr;
			int len = p_w_file->data - p_w_file->buf_addr;
			int i = 0;
			printk("will log out buf id:%d,len:%d\n",buf_id,len);

			while(tmp_p < p_w_file->data)
			{
				printk("%02x %02x %02x %02x",tmp_p[0],tmp_p[1],tmp_p[2],tmp_p[3]);
				tmp_p += 4;
				printk("    %02x %02x",tmp_p[0],tmp_p[1]);
				tmp_p += 2;
				printk("    %02x",tmp_p[0]);
				tmp_p += 1;
				printk("       %02x",tmp_p[0]);
				tmp_p += 1;
				printk("       %02x %02x",tmp_p[0],tmp_p[1]);
				tmp_p += 2;
				printk("     %02x %02x",tmp_p[0],tmp_p[1]);
				tmp_p += 2;
				printk("   %02x %02x",tmp_p[0],tmp_p[1]);
				len = (tmp_p[0] << 8) + tmp_p[1] ;
				tmp_p += 2;
				printk("      ");
				for(i = 0 ; i < len ; i++ )
				{
					printk("%02x ",tmp_p[i] );
				}
				printk("\n");
				tmp_p += len;
			}	
		}
		#endif

		//chenq mod 2012-09-28
		#if 0
		p_w_file->data = p_w_file->buf_addr;
		
		atomic_dec(&p_w_file->buf_flag);

		if( atomic_read(&g_empt_buf_cnt) < LOG_BUF_CNT - 1 )
		{
			buf_id = inc_in_size(buf_id,LOG_BUF_CNT);
			
			atomic_inc(&g_empt_buf_cnt);
			p_w_file = p_w_file->pnext;
		}
		else
		{
			goto SLEEP;
		}
		
		goto ACTION;
		#else
		if( atomic_read(&g_empt_buf_cnt) < LOG_BUF_CNT - 1 )
		{
			buf_id = inc_in_size(buf_id,LOG_BUF_CNT);
			
			atomic_inc(&g_empt_buf_cnt);
		}
		
		atomic_dec(&p_w_file->buf_flag);
		rbuf_cnt++;
		atomic_set(&g_rbuf_cnt,rbuf_cnt);
		goto ACTION;
		#endif

SLEEP:
		if(time_cnt >= max_time_cnt)
		{
			unsigned char tmp_buf[LOG_HEAD_LEN] = {0};
			time_cnt = 0;

			setPrintTime(tmp_buf);
			pos = &(fp->f_pos); 
			vfs_write(fp, tmp_buf,sizeof(tmp_buf), pos);
		}

		mutex_unlock(&print2file_lock);

}

static int printLog2File_kthread(void *data)
{


	/*
	printk("printLog2File_kthread current->pid:%d,current->tgid:%d,current->real_parent:%d,current->parent:%d\n",
			(int)(current->pid),(int)(current->tgid),(int)(current->real_parent),(int)(current->parent));
	*/
	
	while(!kthread_should_stop())
	{
		TRACE_LOG_PRINT("printLog2File_kthread wake\n");
		//printk("printLog2File_kthread wake\n");
		set_current_state(TASK_INTERRUPTIBLE);

		print2file();
				
		schedule();
	}
	return 0; 
}

int trout_trace_init(int buf_size,int element_size,int log_delay)
{
	int i=0;
	struct buf_dscr * tmp_buf_dscr = NULL;
	struct buf_dscr * cur_buf_dscr = NULL;

	if( (element_size != 0) && (buf_size != 0) && (element_size + LOG_HEAD_LEN > buf_size) )
	{
		TRACE_LOG_PRINT("element_size + LOG_HEAD_LEN > buf_size\n");
		return -1;
	}

	/* user config some flag */
	g_log_delay    = (log_delay == 0) ? LOG_DELAY : log_delay;
	g_log_bufsize  = (buf_size == 0) ? LOG_BUF_SIZE : buf_size;
	g_a_trace_size = (element_size == 0) ? LOG_A_TRACE_MAX_SIZE : element_size;

	g_total_wfile_len = 0;
    g_wfile_over_size = 0;

	/* init timer */
	init_timer(&log_timer);
	log_timer.data     = 0;
	log_timer.function = time2file;

	/* open log file*/
	if( openLogFile() < 0 )
	{
		return -1;
	}

	/* init buf dscr*/
	for( i = 0 ; i < LOG_BUF_CNT ; i++ )
	{
		if( NULL ==
			(tmp_buf_dscr = (struct buf_dscr * )kmalloc(sizeof(struct buf_dscr),GFP_KERNEL)))
		{
			break;
		}

		memset(tmp_buf_dscr,0x00,sizeof(struct buf_dscr));
		if(g_buf_head == NULL)
		{
			g_buf_head   = tmp_buf_dscr;
			cur_buf_dscr = tmp_buf_dscr;
		}
		else
		{
			cur_buf_dscr->pnext = tmp_buf_dscr;
			cur_buf_dscr        = cur_buf_dscr->pnext;
		}
		
		atomic_set(&(tmp_buf_dscr->buf_flag),-1);
	}
	cur_buf_dscr->pnext = g_buf_head;// around

	if(i < LOG_BUF_CNT)// alloc mem fail
	{
		for( ;i>=0;i--)// free mem
		{
			tmp_buf_dscr = g_buf_head->pnext;
			kfree(g_buf_head);
			g_buf_head = tmp_buf_dscr;
		}
		g_buf_head = NULL;
		closeLogFile();
		return -1;
	}

	/* init buf space*/
	cur_buf_dscr = g_buf_head;
	for( i = 0 ; i < LOG_BUF_CNT ; i++ )
	{
		if( NULL ==
			(cur_buf_dscr->buf_addr = (unsigned char * )kmalloc(g_log_bufsize,GFP_KERNEL)))
		{
			break;
		}
		cur_buf_dscr->data = cur_buf_dscr->buf_addr;
		cur_buf_dscr       = cur_buf_dscr->pnext;
	}

	if(i < LOG_BUF_CNT)// alloc mem fail
	{
		cur_buf_dscr = g_buf_head;
		for( ;i>=0;i--)// free buf mem
		{
			tmp_buf_dscr = cur_buf_dscr->pnext;
			kfree(cur_buf_dscr->buf_addr);
			cur_buf_dscr = tmp_buf_dscr;
		}

		cur_buf_dscr = g_buf_head;
		for( i = 0 ; i < LOG_BUF_CNT ; i++ )// free dscr mem
		{
			tmp_buf_dscr = cur_buf_dscr->pnext;
			kfree(cur_buf_dscr);
			cur_buf_dscr = tmp_buf_dscr;
		}
		closeLogFile();
		return -1;
	}

	p_w_buf  = g_buf_head;
	p_w_file = g_buf_head;
	
	mutex_init(&buf_wr_lock);
	mutex_init(&print2file_lock);
	
	g_pthread_t = kthread_run(printLog2File_kthread,NULL,"printLog2File_kthread");
	mod_timer(&log_timer, jiffies+ LOG_DELAY * HZ);
	
	return 0;
	
}

void trout_trace_exit(void)
{
	int i = 0 ;
	struct buf_dscr * tmp_buf_dscr = NULL;

	/* del timer */
	del_timer(&log_timer);

	atomic_set(&g_wall_flag,1);
	/* print log from buf to file */
	print2file();

	/* del thread */
	kthread_stop(g_pthread_t);

	/* close log file*/
	closeLogFile();

    /* free mem*/
	for( i = 0 ; i < LOG_BUF_CNT ;i++ )
	{
		tmp_buf_dscr = g_buf_head->pnext;
		kfree(g_buf_head->buf_addr);
		kfree(g_buf_head);
		g_buf_head = tmp_buf_dscr;
	}
	g_buf_head = NULL;
}

int  trout_trace_reinit(void)
{
	trout_trace_exit();

	if( trout_trace_init(g_log_bufsize,g_a_trace_size,g_log_delay) < 0 )
	{
		return -1;
	}

	return 0;
}

void trout_trace_hex_dump(int debuglevel,int filenum,int linenum,
	                            unsigned char * ptr, int len)
{
	struct buf_dscr * tmp_pbuf = NULL;
	int i=0;
	int tigger_writefile = 0;
	int total_len = LOG_HEAD_LEN;
	//加锁
	//mutex_lock(&buf_wr_lock);
	if( !mutex_trylock(&buf_wr_lock) )
	{
		return;
	}
	
	//取消timer
	del_timer(&log_timer);

	if( g_wfile_over_size )
	{
		goto END2;
	}

	if( ptr == NULL )
	{
		len = 0;
	}
	
	//限制长度
	if(len > LOG_BUF_SIZE)
	{
		len = LOG_BUF_SIZE;
	}
	
	//先判断bug的剩余空间是否足够
	total_len += len;

	//占用buf
	if( ( tmp_pbuf = saveLog2Buf(p_w_buf,total_len) ) == NULL )
	{
		tigger_writefile = 1;
		goto END1;
	}

	if(p_w_buf != tmp_pbuf)
	{
		p_w_buf = tmp_pbuf;
		p_w_buf->data = p_w_buf->buf_addr;
		tigger_writefile = 1;
	}

	if( (p_w_buf->data + total_len) > (p_w_buf->buf_addr + LOG_BUF_SIZE - LOG_HEAD_LEN) )
	{
		tigger_writefile = 1;
	}
	
	setPrintTime(p_w_buf->data);
	setProcessID(p_w_buf->data);
	setDebugLevel(p_w_buf->data,debuglevel);
	setReserved(p_w_buf->data,0);
	setFileNum(p_w_buf->data,filenum);
	setLineNum(p_w_buf->data,linenum);
	setLogLen(p_w_buf->data,len);
	p_w_buf->data += LOG_HEAD_LEN;

	if(total_len == LOG_HEAD_LEN)
	{
		goto END;
	}
	
	for( i = 0; i < len ; i++)
	{
		*(p_w_buf->data++) = ptr[i];
	}

	END:	
	//放弃占用
	saveLog2BufOver(p_w_buf);

	END1:	
	if(tigger_writefile == 1)
	{
		if(IS_ERR(g_pthread_t))
		{
			TRACE_LOG_PRINT("kthread fuc is err ! Impossible %s,%d!!!\n",__FUNCTION__,__LINE__);
		}
		else
		{
			TRACE_LOG_PRINT("will wake thread!!!\n");
			wake_up_process(g_pthread_t);
		}
	}

	//激活timer
	mod_timer(&log_timer, jiffies+ LOG_DELAY * HZ);

	END2:
	//解锁
	mutex_unlock(&buf_wr_lock);
	return;
	
}

void trout_trace_print_log(int debuglevel,int filenum,int linenum,
	                          char * format,...)
{	
	struct buf_dscr * tmp_pbuf = NULL;
	va_list argp = {0}; /* 定义保存函数参数的结构 */
	unsigned char * tmp_p = 0;
	unsigned char * pmult_arg = 0;
	unsigned char   char_arg = 0;
	unsigned int    int_arg  = 0;

	int cur_format = 0;
	int tmp_len = 0;
	int total_len = LOG_HEAD_LEN;

	int tigger_writefile = 0;

	//加锁
	//mutex_lock(&buf_wr_lock);
	if( !mutex_trylock(&buf_wr_lock) )
	{
		//printk("chenq buf lock !!!!\n");
		return;
	}

	if( !(trout_dbg_mask_get() & (1 << debuglevel)))
	{
		goto END2;
	}

	//取消timer
	del_timer(&log_timer);

	if( g_wfile_over_size )
	{
		goto END2;
	}

	//计算size
	tmp_p = format;
	va_start( argp, format);
	while(tmp_p != NULL)
	{
		if( *tmp_p == 0x00 )
		{
			break;
		}	
		cur_format = getFormatType((unsigned char)(*tmp_p));
		if(cur_format == CHAR_TYPE)
		{
			total_len += CHAR_TYPE;
			va_arg( argp, unsigned int);
		}
		else if(cur_format == INT_TYPE)
		{
			total_len += INT_TYPE;
			va_arg( argp, unsigned int);
		}
		else if(cur_format < 0)
		{
			TRACE_LOG_PRINT("input a err format = %c\n",(char)(*tmp_p));
			goto END1;
		}
		else
		{
			pmult_arg = va_arg( argp, unsigned char *);
			if( pmult_arg == NULL)
			{
				tmp_len = 1;
			}
			else
			{
				tmp_len = strlen(pmult_arg) + 1;
			}
			
			if(tmp_len > g_a_trace_size)
			{
				tmp_len = g_a_trace_size;
			}
			total_len +=tmp_len;
		}
		tmp_p++;
	}

	//根据该条trace长度选择buf
	if(total_len > g_log_bufsize)//超过一块buf大小的log 被 丢弃
	{
		TRACE_LOG_PRINT("the trace too long(len = %d),will not print\n",total_len);
		goto END1;
	}
	else if( ( tmp_pbuf = saveLog2Buf(p_w_buf,total_len) ) == NULL )
	{
		//printk("tigger_writefile == NULL\n");
		tigger_writefile = 1;
		goto END1;
	}

	if(p_w_buf != tmp_pbuf)
	{
		p_w_buf = tmp_pbuf;
		p_w_buf->data = p_w_buf->buf_addr;
		//printk("tigger_writefile == addr diff\n");
		tigger_writefile = 1;
	}

	if( (p_w_buf->data + total_len) > (p_w_buf->buf_addr + LOG_BUF_SIZE - LOG_HEAD_LEN) )
	{
		//printk("tigger_writefile == space not do\n");
		tigger_writefile = 1;
	}

	setPrintTime(p_w_buf->data);
	setProcessID(p_w_buf->data);
	setDebugLevel(p_w_buf->data,debuglevel);
	setReserved(p_w_buf->data,0);
	setFileNum(p_w_buf->data,filenum);
	setLineNum(p_w_buf->data,linenum);
	setLogLen(p_w_buf->data,total_len - LOG_HEAD_LEN);
	p_w_buf->data += LOG_HEAD_LEN;

	if(total_len == LOG_HEAD_LEN)//如果用户给的是空打印，则只有trace头
	{
		goto END;
	}

	// 使用宏va_start, 使argp指向传入的第一个可选参数，
	// 注意 msg是参数表中最后一个确定的参数，并非参数表中第一个参数
	va_start( argp, format );
	while( (cur_format = *(format++)) != 0x00 )
	{
		TRACE_LOG_PRINT("find format:%c\n",(char)cur_format);
		cur_format = getFormatType((unsigned char)cur_format);

		switch(cur_format)
		{
			case CHAR_TYPE:
				
				char_arg = va_arg( argp, unsigned int );
				*(p_w_buf->data++) = char_arg;
				
				break;
			case INT_TYPE:
				
				int_arg = va_arg( argp, unsigned int);
				
				*(p_w_buf->data++) = int_arg >> 24;
				*(p_w_buf->data++) = int_arg >> 16;
				*(p_w_buf->data++) = int_arg >> 8;
				*(p_w_buf->data++) = int_arg;
				
				break;
			case MULT_TYPE:
				pmult_arg = va_arg( argp, unsigned char *);
				if(pmult_arg == NULL)
				{
					*(p_w_buf->data++) = 0x00;
					break;
				}
				
				tmp_len = strlen(pmult_arg) + 1;
				if( tmp_len >= g_a_trace_size )
				{
					tmp_len = g_a_trace_size;
					pmult_arg[g_a_trace_size-1] = 0x00;
				}
				memcpy(p_w_buf->data,pmult_arg,tmp_len);
				p_w_buf->data += tmp_len;
				break;
			default:
				/*不会进来*/
				break;
		}
	}

	END:
	//放弃占用
	saveLog2BufOver(p_w_buf);

	END1:
	va_end( argp ); /* 将argp置为NULL */
	

	if( tigger_writefile == 1 )
	{
		if(IS_ERR(g_pthread_t))
		{
			TRACE_LOG_PRINT("kthread fuc is err ! Impossible !!!\n");
		}
		else
		{
			TRACE_LOG_PRINT("will wake thread!!!\n");
			//printk("print will wake thread!!!\n");
			wake_up_process(g_pthread_t);
		}
	}

	//激活timer
	mod_timer(&log_timer, jiffies + LOG_DELAY * HZ);

	END2:
	//解锁
	mutex_unlock(&buf_wr_lock);
	return;
}

#endif	/* TROUT_TRACE_DBG */

