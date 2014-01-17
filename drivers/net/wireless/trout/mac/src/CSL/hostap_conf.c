#include "hostap_conf.h"
#include "mac_string.h"
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/fs_struct.h>
#include <linux/path.h>
#include <linux/uaccess.h>
#include <linux/slab.h>


static phostap_conf_t hostap_conf_create()
{
     phostap_conf_t conf = NULL;
     conf =  kzalloc(sizeof(hostap_conf_t),GFP_KERNEL);
     return conf;
}


static int fp_size(struct file * f)
{
	int error = -EBADF;
        struct kstat stat;
	error = vfs_getattr(f->f_path.mnt, f->f_path.dentry, &stat);
	if(error == 0){
		return stat.size;
	}else{
		printk("get  hostapd conf file stat error\n");
		return error;
	}
}

static int  hostap_conf_read(char * filename,char * * buf)
{
	struct file *fp;
	mm_segment_t fs;
	int size = 0;
	fp = filp_open(filename, O_RDONLY , 0);
	
    	if (IS_ERR(fp)) {
        	printk("open %s file error\n",filename);
        	goto end;
    	}
	
	fs = get_fs();
    	set_fs(KERNEL_DS);
		
	size = fp_size(fp);

	if(size <= 0){
		printk("load file:%s error\n",filename);
		goto error;
	}
	
	loff_t pos = 0;
	*buf = kzalloc(size+1,GFP_KERNEL);
	
	 vfs_read(fp, *buf, size, &pos);

error:
	 filp_close(fp, NULL);
	 set_fs(fs);
end:	 
	 return size;
}

static char * get_line(char *buf,char * end)
{
	if(buf == NULL ||  buf>= end ) return NULL;
	while(buf !=end && *buf !='\n'){
		buf++;
	}
	return buf;
}

static unsigned char hostap_conf_parse(char * buf,int size,phostap_conf_t conf)
{
	unsigned char ret = 0;
	if(!buf || !conf) return 0;
	char * spos=buf,*epos= NULL,*pos=NULL;
	int line = 0,errors=0;
	
	for(;epos = get_line(spos,buf+size);spos=epos+1){
		line++;
		if (spos[0] == '#')
			continue;
		pos = spos;
		while (*pos != '\0' && pos <= epos) {
			if (*pos == '\n') {
				*pos = '\0';
				break;
			}
			pos++;
		}
		if (spos[0] == '\0')
			continue;

		pos = strchr(spos, '=');
		if (pos == NULL) {
			printk("Line %d: invalid line '%s'",
				   line, spos);
			errors++;
			continue;
		}
		*pos = '\0';
		pos++;
		if ( strcmp(spos,"wpa_psk") == 0 ){
			strlcpy(conf->wpa_psk,pos,sizeof(conf->wpa_psk));
			conf->len = strlen(pos);
		}else{
			continue;
		}
		
	}
	
	
	return ret;
}

//-----------------------interface api-----------------------
void hostap_conf_destry(phostap_conf_t conf)
{

	if(conf == NULL) return;
	kfree(conf);
}


phostap_conf_t hostap_conf_load(char * filename)
{
	
	phostap_conf_t conf = NULL;
	char * buf = NULL;
	int size=0;
		
	if(filename == NULL) filename = HOSTAP_CONF_FILE_NAME;

	size = hostap_conf_read(filename,&buf);
	
	if(size>0){
		conf = hostap_conf_create();
		if(conf == NULL){
			kfree(buf);
			printk("create hostap_conf struct error.\n");
			return NULL;
		}
		hostap_conf_parse(buf,size,conf);
		kfree(buf);
	}
    	return conf;
}
