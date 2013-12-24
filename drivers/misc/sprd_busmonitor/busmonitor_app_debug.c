#include "busmonitor_app_debug.h"


static int bm_ddr_open(struct inode* ind,struct file* filp)
{
	try_module_get(THIS_MODULE);
	BUSMON_PRINT("BUSMON_LOG,%s,open busmon success\n",__func__);
	return 0;
}

static ssize_t bm_ddr_read(struct file *filp, char *buf, size_t size,loff_t *fpos)
{
	return 0;
}

static ssize_t bm_ddr_write(struct file *filp,const char *buf,size_t size,loff_t *fpos)
{
	return 0;
}

static int bm_ddr_release(struct inode *ind,struct file* filp)
{
	module_put(THIS_MODULE);
	BUSMON_PRINT("BUSMON_LOG,%s,close busmon success\n",__func__);
	return 0;
}

static unsigned int virtual_to_physics_addr(unsigned int vir_addr,struct mm_struct * mm)
{
	unsigned int real_addr;
	struct task_struct *task = NULL;
	pgd_t *pgd;
	pud_t * pud;
	pmd_t *pmd;
	pte_t *pte;

	pgd = pgd_offset(mm, vir_addr);
	if ( pgd_none(*pgd) || pgd_bad(*pgd) )
	{
		BUSMON_PRINT("BUSMON_LOG,%s, invalid pgd\n",__func__);
		goto out;
	}

	pud = pud_offset(pgd, vir_addr);
	if ( pud_none(*pud) || pud_bad(*pud) )
	{
		BUSMON_PRINT("BUSMON_LOG,%s, invalid pud\n",__func__);
		goto out;
	}

	pmd = pmd_offset(pgd, vir_addr);
	if ( pmd_none(*pmd) || pmd_bad(*pmd) )
	{
		BUSMON_PRINT("BUSMON_LOG,%s, invalid pmd\n",__func__);
		goto out;
	}

	pte = pte_offset_kernel(pmd, vir_addr);
	if ( pte_none(*pte) )
	{
		BUSMON_PRINT("BUSMON_LOG,%s, bad pte va: %X pte: %p pteval: %lX\n",__func__, vir_addr, pte, pte_val(*pte));
		goto out;
	}
	real_addr = (pte_val(*pte) & PAGE_MASK) |(vir_addr & ~PAGE_MASK);
	return real_addr;
out:
	BUSMON_PRINT("BUSMON_LOG,%s, no map addr=%x, reflect\n",__func__,  vir_addr);
	return 0;
}

static void save_thread_info()
{
	thread_info.u_id = (current->tgid & 0xffff) << 16 | (current->pid & 0xffff);
	memcpy(thread_info.name,current->comm,16);
}

static void export_thread_info(struct saved_thread_info * st)
{
	st->u_id = thread_info.u_id;
	memcpy(st->name,thread_info.name,16);
}

extern void msleep(unsigned int msecs);
static long bm_ddr_ioctl(struct file *filp, unsigned int cmd,unsigned long arg)
{
	int res;
	unsigned long v_addr, v_addr2;
	char *p_data;
	unsigned int loop_value;

	switch (cmd)
	{
		case BUSMON_WRITE_ARRAY:
		{
			unsigned int buf[3];//0-dest_addr 1-sour_addr 2-lenth
			unsigned int valid_lenth;
			res = copy_from_user(buf, arg, 3 * sizeof(unsigned int));//get the parmeters
			BUSMON_PRINT("BUSMON_LOG,%s,  the dest_address is 0x%.8x, the sour_addr is 0x%.8x,the lenth is %.8x\n",\
				__func__,buf[0],buf[1],buf[2]);
			save_thread_info();
			valid_lenth = ((PAGE_SIZE - ((PAGE_SIZE -1)&buf[0]))>buf[2]) ? buf[2] : \
			((PAGE_SIZE - (PAGE_SIZE -1)&buf[0]));
			p_data = (char *)kmalloc(valid_lenth,GFP_KERNEL);
			res = copy_from_user(p_data,(char *)(buf[1]),valid_lenth);
			v_addr = virtual_to_physics_addr(buf[0],current->mm);//transform user addr to kernel addr
			BUSMON_PRINT("BUSMON_LOG,%s, the valid_lenth is 0x%.8x,the v_addr is 0x%.8x\n",__func__, valid_lenth, v_addr);
			for(loop_value = 0; loop_value < valid_lenth/4; loop_value ++)
			{
				change_flag = true;
				v_addr2 = (unsigned int)ioremap_nocache(v_addr, PAGE_SIZE);
				__raw_writel(*((unsigned int *)(p_data + loop_value * 4)), v_addr2 + loop_value * 4);
				iounmap(v_addr2);
				msleep(20);
			}
			kfree(p_data);
		}
		break;

		case BUSMON_WRITE_VARIABLE:
		{
			unsigned int buf[2];//buf[0]: addr buf[1]: value
			res = copy_from_user(buf, arg, 2 * sizeof(unsigned int));//get the parmeters
			BUSMON_PRINT("BUSMON_LOG,%s,  the dest_address is 0x%x, the value is %x\n",__func__,buf[0],buf[1]);
			save_thread_info();
			v_addr = virtual_to_physics_addr(buf[0],current->mm);//transform user addr to kernel addr
			BUSMON_PRINT("BUSMON_LOG,%s, the v_addr is 0x%x\n",__func__,v_addr);

			change_flag = true;
			v_addr2 = (unsigned int)ioremap_nocache(v_addr, PAGE_SIZE);
			__raw_writel(buf[1], v_addr2);
			iounmap(v_addr2);
		}
		break;

		case BUSMON_CLR:
			BUSMON_PRINT("BUSMON_LOG,%s, clear the axi_base register\n",__func__);
			//clear the axi_busmon register
			sci_bm_unset_point(AXI_BM3);
			kfree(g_data);

		break;
		case BUSMON_SET:
		{
			unsigned int buf[3];//buf[0]: addr buf[1]: lenth
			struct sci_bm_cfg bm_cfg;
			unsigned int valid_lenth;
			res = copy_from_user(buf,arg,3 * sizeof(unsigned int));//get the parmeters
			BUSMON_PRINT("BUSMON_LOG,%s,  the dest_address is 0x%x, the lenth is %x\n",__func__,buf[0],buf[1]);

			if(buf[2] == 0) {//application use mmap to get memory in user space
				valid_lenth = buf[1];
				v_addr = user_buf;
			}
			else if(buf[2] == 1) {//application use malloc to get memory in user space
				valid_lenth = ((PAGE_SIZE - ((PAGE_SIZE -1)&buf[0])) > buf[1])? buf[1] : \
				(PAGE_SIZE - ((PAGE_SIZE -1)&buf[0]));
				v_addr = virtual_to_physics_addr(buf[0],current->mm);//transform user addr to kernel addr
			}

			monitor_lenth = (valid_lenth > MONITOR_LENTH) ? MONITOR_LENTH : valid_lenth;

			g_data = kmalloc(sizeof(struct busmon_match) * monitor_lenth/4,GFP_KERNEL);
			if (!g_data) {
				BUSMON_PRINT("BUSMON_LOG,%s,malloc g_data failed\n",__func__);
				return -1;
			}
			BUSMON_PRINT("BUSMON_LOG,%s, the g_data's address is 0x%.8x\n",__func__,(unsigned int)g_data);
			memset(g_data,0,sizeof(struct busmon_match) * monitor_lenth/4);

			for(loop_value = 0; loop_value < monitor_lenth/4; loop_value ++)
			{
				g_data[loop_value].addr = v_addr+ loop_value * 0x4;
			}
			//set the axi_busmon register
			bm_cfg.addr_min = v_addr;
			if (valid_lenth < MONITOR_LENTH)
				bm_cfg.addr_max = v_addr + valid_lenth - 1;
			else
				bm_cfg.addr_max = v_addr + MONITOR_LENTH - 1;
			bm_cfg.bm_mode = W_MODE;

			res = sci_bm_set_point(AXI_BM3, CHN0, &bm_cfg, bm_ddr_callback);

		}
		break;

		default:
			BUSMON_PRINT("BUSMON_LOG,%s,undefined cmd!\n",__func__);
		break;
	}
}

static int bm_ddr_mmap(struct file *filp,struct vm_area_struct *vma)
{

	BUSMON_PRINT("BUSMON_LOG,%s,0x%.8x,0x%.8x,0x%.8x,0x%.8x\n",__func__,\
	vma->vm_start,vma->vm_end,vma->vm_page_prot,vma->vm_pgoff);

	int result;

	vma->vm_flags|=VM_RESERVED|VM_SHARED;

	vma->vm_flags|=VM_IO;

	result=remap_pfn_range(vma,
		vma->vm_start,
		((unsigned int)user_buf)>>PAGE_SHIFT,
		BUSMON_BUF_SIZE,
		vma->vm_page_prot);

	if(result)
		return -EAGAIN;

    return 0;
}


struct file_operations busmon_fops ={
	.open = bm_ddr_open,
	.release = bm_ddr_release,
	.read = bm_ddr_read,
	.write = bm_ddr_write,
	.unlocked_ioctl = bm_ddr_ioctl,
	.mmap = bm_ddr_mmap,
};

static void save_busmon_data(struct work_struct *work)
{
	int res;
	unsigned int index;
	short times;
	unsigned int vaddr;
	vaddr = (unsigned int)ioremap_nocache(g_match_data.read_addr, PAGE_SIZE);
	g_match_data.read_data = __raw_readl(vaddr);
	iounmap(vaddr);

	if(monitor_lenth >= (g_match_data.read_addr - g_match_data.base_addr)) {
		index = (g_match_data.read_addr- g_match_data.base_addr)/4;
		times = g_data[index].cur_times;
		g_data[index].match_data[times].data = g_match_data.read_data;
		memcpy(&(g_data[index].match_data[times].thread_info),&(g_match_data.read_thread_info),sizeof(struct saved_thread_info));
		BUSMON_PRINT("BUSMON_LOG, %s, addr_of_data:0x%x,addr:0x%x,data:0x%x,times:%d,u_id:%x,name:%s\n",__func__,\
		&g_data[index],g_match_data.read_addr,g_match_data.read_data,times,g_match_data.read_thread_info.u_id,g_match_data.read_thread_info.name);
		g_data[index].cur_times = g_data[index].cur_times == (MONITOR_TIMES - 1) ? 0 :  g_data[index].cur_times + 1;
		if(g_data[index].cur_times == 0)
			g_data[index].all_times ++;
	} else {
		BUSMON_PRINT("BUSMON_LOG,%s,out of the monitor range!\n",__func__);
	}
	return;
}

static void bm_ddr_callback()
{
	static int count = 0;
	unsigned int reg_match = SPRD_AXIBM3_BASE;//busmon_3
	BUSMON_PRINT("BUSMON_LOG, %s, count is:%d\n", __func__, count++);
	memset(&g_match_data,0,sizeof(struct busmon_read_data));
	if(change_flag) {
		export_thread_info(&g_match_data.read_thread_info);
		change_flag = false;
	} else {
		g_match_data.read_thread_info.u_id = (current->tgid & 0xffff) << 16 | (current->pid & 0xffff);//get thread_info from current
		memcpy(g_match_data.read_thread_info.name,current->comm,16);
	}
	BUSMON_PRINT("BUSMON_LOG, %s, after export_thread_info,the u_id is %x, the name is %s\n",\
		__func__,g_match_data.read_thread_info.u_id,g_match_data.read_thread_info.name);

	g_match_data.base_addr = __raw_readl(reg_match + 0x08);
	g_match_data.read_addr =  __raw_readl(reg_match + 0x34);
	g_match_data.max_addr = __raw_readl(reg_match + 0x0c);
	schedule_work(&read_match_data_wq);
	BUSMON_PRINT("BUSMON_LOG,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",\
		__raw_readl(reg_match+0x4),__raw_readl(reg_match+0x8),__raw_readl(reg_match+0xc),\
		__raw_readl(reg_match+0x10),__raw_readl(reg_match+0x14),__raw_readl(reg_match+0x18),\
		__raw_readl(reg_match+0x1c),__raw_readl(reg_match+0x20),__raw_readl(reg_match+0x24),\
		__raw_readl(reg_match+0x28),__raw_readl(reg_match+0x2c),__raw_readl(reg_match+0x30),\
		__raw_readl(reg_match+0x34),__raw_readl(reg_match+0x38),__raw_readl(reg_match+0x3c),\
		__raw_readl(reg_match+0x40),__raw_readl(reg_match+0x44),__raw_readl(reg_match+0x48),\
		__raw_readl(reg_match+0x4c),__raw_readl(reg_match+0x50),__raw_readl(reg_match+0x54),\
		__raw_readl(reg_match+0x58),__raw_readl(reg_match+0x5c));
}

static int __init bm_ddr_init(void)
{
	int res;
	int devno = MKDEV(BUSMON_MAJOR,BUSMON_MINOR);

	asm volatile("mrc p15, 0, %0, c1, c0, 0" : "=r" (res));//read the register of cp15, to judge whether the cache has been disabled
	BUSMON_PRINT("BUSMON_LOG,%s,the value of cp15's register is %x\n",__func__,res);
	sci_glb_clr(REG_PUB_APB_BUSMON_CNT_START, BIT(0));//disable count
	user_buf = (char *)__get_free_pages(GFP_KERNEL,2);//alloc 4pages for userspace
	if( user_buf == NULL) {
		BUSMON_PRINT("BUSMON_LOG,%s,malloc user_buf failed\n",__func__);
		return -1;
	}
	BUSMON_PRINT("BUSMON_LOG,%s,the user_buf is 0x%.8x\n",__func__,(unsigned int)user_buf);
	memset(user_buf,0,BUSMON_BUF_SIZE);

	busmon_cdev = cdev_alloc();
	if (busmon_cdev == NULL)
	{
		BUSMON_PRINT("BUSMON_LOG,%s,cdev_alloc failed!\n",__func__);
		kfree(user_buf);
		return 0;
	}
	BUSMON_PRINT("BUSMON_LOG,%s,cdev_alloc sucess\n",__func__);

	res = register_chrdev_region(devno,1,"busmon_char_mem");
	if (res < 0)
		BUSMON_PRINT("BUSMON_LOG, %s,register_chrdev_region failed,the return value is %d\n",__func__,res);

	cdev_init(busmon_cdev,&busmon_fops);
	busmon_cdev->owner = THIS_MODULE;
	res = cdev_add(busmon_cdev, devno,1);
	if (res) {
		cdev_del(busmon_cdev);
		busmon_cdev = NULL;
		kfree(user_buf);
		BUSMON_PRINT("BUSMON_LOG,%s,cdev_add error\n",__func__);
	}
	else {
		BUSMON_PRINT("BUSMON_LOG,%s,cdev_add ok\n",__func__);
	}

	busmon_class = class_create(THIS_MODULE, "busmon_char_class");
	if (IS_ERR(busmon_class)) {
		BUSMON_PRINT("BUSMON_LOG,%s,failed in creating class.\n",__func__);
		res = PTR_ERR(busmon_class);
		return -1;
	}

	device_create(busmon_class, NULL, MKDEV(BUSMON_MAJOR, BUSMON_MINOR), NULL, "busmon_char_dev");

	INIT_WORK(&read_match_data_wq, save_busmon_data);//init workqueue

	return 0;
}

static void __exit bm_ddr_exit(void)
{
	if (busmon_cdev != NULL)
		cdev_del(busmon_cdev);
	BUSMON_PRINT("BUSMON_LOG,%s,cdev_del ok\n",__func__);
	device_destroy(busmon_class, MKDEV(BUSMON_MAJOR,BUSMON_MINOR));
	class_destroy(busmon_class);

	unregister_chrdev_region(MKDEV(BUSMON_MAJOR, BUSMON_MINOR),1);

	free_pages(user_buf, 2);
	BUSMON_PRINT("BUSMON_LOG,%s,over\n", __func__);
}

module_init(bm_ddr_init);
module_exit(bm_ddr_exit);

MODULE_LICENSE("GPL");
