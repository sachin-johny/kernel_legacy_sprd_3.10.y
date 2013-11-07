#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/namei.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include "wifi_nvm.h"

#define _WIFI_NVM_DEBUG_

typedef struct
{
	char itm[64];
	int  par[32];
	int  num;
} nvm_cali_cmd;

static char *WIFI_NVM_FILE =  "/productinfo/2351_wifi_configure.ini";
static WIFI_nvm_data   g_wifi_nvm_data;

extern long simple_strtol(const char *cp, char **endp, unsigned int base);

WIFI_nvm_data *get_gWIFI_nvm_data(void )
{
	return &g_wifi_nvm_data;
}

static int find_type(char key)
{
	if( (key >= 'a' && key <= 'w') || (key >= 'y' && key <= 'z') || (key >= 'A' && key <= 'W') || (key >= 'Y' && key <= 'Z') || ('_' == key) )
		return 1;
	if( (key >= '0' && key <= '9') || ('-' == key) )
		return 2;
	if( ('x' == key) || ('X' == key) || ('.' == key) )
		return 3;
	if( (key == '\0') || ('\r' == key) || ('\n' == key) || ('#' == key) )
		return 4;
	return 0;
}

static void get_cmd_par(unsigned char *str, nvm_cali_cmd *cmd)
{
	int i, j, bufType, cType, flag;
	char tmp[128];
	char c;
	bufType = -1;
	cType = 0;
	flag = 0;
	memset( cmd, 0, sizeof(nvm_cali_cmd) );
	for(i = 0, j = 0; ; i++)
	{
		c = str[i];
		cType = find_type(c);
		if( (1 == cType) || ( 2 == cType) || (3 == cType)  )
		{
			tmp[j] = c;
			j++;
			if(-1 == bufType)
			{
				if(2 == cType)
					bufType = 2;
				else
					bufType = 1;
			}
			else if(2 == bufType)
			{
				if(1 == cType)
					bufType = 1;
			}
			continue;
		}
		if(-1 != bufType)
		{
			tmp[j] = '\0';

			if((1 == bufType) && (0 == flag) )
			{
				strcpy(cmd->itm, tmp);
				flag = 1;
			}
			else
			{
				cmd->par[cmd->num] = simple_strtol(tmp, NULL, 0);
				cmd->num++;
			}
			bufType = -1;
			j = 0;
		}
		if(0 == cType )
			continue;
		if(4 == cType)
			return;
	}
	return;
}

static int wifi_nvm_set_cmd(nvm_name_table *pTable, nvm_cali_cmd *cmd)
{
	int i;
	unsigned char  *p;
	if( (1 != pTable->type)  && (2 != pTable->type) && (4 != pTable->type) )
		return -1;
	p = (unsigned char *)(&g_wifi_nvm_data) + pTable->mem_offset;
#ifdef _WIFI_NVM_DEBUG_
	printk("###[g_table]%s, offset:%d, num:%d, value: %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d\n", pTable->itm, pTable->mem_offset, cmd->num, cmd->par[0], cmd->par[1], cmd->par[2], cmd->par[3], cmd->par[4], cmd->par[5], cmd->par[6], cmd->par[7], cmd->par[8], cmd->par[9], cmd->par[10], cmd->par[11] );
#endif
	for(i = 0; i < cmd->num;  i++)
	{
		if(1 == pTable->type)
			*((unsigned char *)p + i) = (unsigned char)(cmd->par[i]);
		else if(2 == pTable->type)
			*((unsigned short *)p + i) = (unsigned short)(cmd->par[i]);
		else if(4 == pTable->type)
			*( (unsigned int *)p + i) = (unsigned int)(cmd->par[i]);
		else
			printk("%s, type err\n", __func__);
	}
	return 0;
}

static nvm_name_table *wifi_nvm_table_match(nvm_cali_cmd *cmd)
{
	int i;
	nvm_name_table *pTable = NULL;
	int len = sizeof(g_nvm_table) / sizeof(nvm_name_table);
	if(NULL == cmd->itm)
		return NULL;
	for(i = 0; i < len; i++)
	{
		if(NULL == g_nvm_table[i].itm)
			continue;
		if( 0 != strcmp( g_nvm_table[i].itm, cmd->itm ) )
			continue;
		pTable = &g_nvm_table[i];
		break;
	}
	return pTable;
}

static int wifi_nvm_buf_operate(unsigned char *pBuf, int file_len)
{
	int i, p;
	nvm_cali_cmd cmd;
	nvm_name_table *pTable = NULL;
	if((NULL == pBuf) || (0 == file_len) )
		return -1;
	for(i = 0, p = 0; i < file_len; i++)
	{
		if( ('\n' == *(pBuf + i)) || ( '\r' == *(pBuf + i)) || ( '\0' == *(pBuf + i) )   )
		{
			if(5 <= (i - p) )
			{
				get_cmd_par((pBuf + p), &cmd);
				pTable = wifi_nvm_table_match(&cmd);
				if(NULL != pTable)
				{
					wifi_nvm_set_cmd(pTable, &cmd);
				}
			}
			p = i + 1;
		}

	}
	g_wifi_nvm_data.data_init_ok = 1;
	return 0;
}

void ittiam_nvm_init(void )
{
	struct file    *filp = NULL;
	struct inode *inode = NULL;
	unsigned char *pBuf = NULL;
	unsigned short len = 0;
	mm_segment_t oldfs;
	printk("%s()...\n", __func__);
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	filp = filp_open(WIFI_NVM_FILE, O_RDONLY, S_IRUSR);
	if (IS_ERR(filp))
	{
		printk("%s: file %s filp_open error\n", __FUNCTION__, WIFI_NVM_FILE);
		goto nvm_read_err;
	}
	if (!filp->f_op)
	{
		printk("%s: File Operation Method Error\n", __FUNCTION__);
		goto nvm_read_err;
	}
	inode = filp->f_path.dentry->d_inode;
	if (!inode)
	{
		printk("%s: Get inode from filp failed\n", __FUNCTION__);
		goto nvm_read_err;
	}
	len = i_size_read(inode->i_mapping->host);
	if (!( len > 0 && len < 62 * 1024 ))
	{
		printk("%s file size error\n", __FUNCTION__);
		goto file_close;
	}
	pBuf = kmalloc(len, GFP_KERNEL);
	if (!pBuf)
	{
		printk("%s alloctation memory failed\n", __FUNCTION__);
		goto file_close;
	}
	if (len  != filp->f_op->read(filp, pBuf, len, &filp->f_pos) )
	{
		printk("%s file read error\n", __FUNCTION__);
		goto free_buf;
	}
	filp_close(filp, NULL);
	set_fs(oldfs);

	printk("%s read %s data_len:0x%x \n", __func__, WIFI_NVM_FILE, len);
	wifi_nvm_buf_operate(pBuf, len);
	kfree(pBuf);
	printk("%s(), ok!\n",  __func__);
	return;

free_buf:
	kfree(pBuf);
file_close:
	filp_close(filp, NULL);
nvm_read_err:
	set_fs(oldfs);
	printk("%s(), err!\n",  __func__);
	return;
}



