#include <linux/module.h>

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/nmi.h>
#include <linux/quicklist.h>
#include <asm/setup.h>


#define K(x) ((x) << (PAGE_SHIFT - 10))

extern void show_mem(void);
extern struct meminfo meminfo;

static struct proc_dir_entry* phymem;

static int _phymem_map_proc_show(struct seq_file *m, void *v)
{
		int slab = 0, node, i, totalsize = 0;
			struct meminfo *mi = &meminfo;

				for_each_online_node(node) {
							for_each_nodebank (i,mi,node) {
											struct membank *bank = &mi->bank[i];
														unsigned int pfn1, pfn2;

																	pfn1 = bank_pfn_start(bank);
																				pfn2 = bank_pfn_end(bank);
																							totalsize += bank->size;
																										seq_printf(m, "0x%08x - 0x%08x    %dkB\n", bank->start, bank->start + bank->size, bank->size/1024);
																												}
								}

					seq_printf(m, "linux total memory: %dkB\n", totalsize/1024);

						return 0;
}

static int _phymem_pages_proc_show(struct seq_file *m, void *v)
{
		int free = 0, total = 0, reserved = 0;
			int other = 0, shared = 0, cached = 0, slab = 0, node, i;

				struct meminfo * mi = &meminfo;

					for_each_online_node(node) {
								for_each_nodebank (i,mi,node) {
												struct membank *bank = &mi->bank[i];
															unsigned int pfn1, pfn2;
																		struct page *page, *end;

																					pfn1 = bank_pfn_start(bank);
																								pfn2 = bank_pfn_end(bank);

																											page = pfn_to_page(pfn1);
																														end  = pfn_to_page(pfn2 - 1) + 1;

																																	do {
																																						total++;
																																										if (PageReserved(page))
																																																reserved++;
																																														else if (PageSwapCache(page))
																																																				cached++;
																																																		else if (PageSlab(page))
																																																								slab++;
																																																						else if (page_count(page) > 1)
																																																												shared++;
																																																										else if (!page_count(page))
																																																																free++;
																																																														else
																																																																				other++;
																																																																			/*shared += page_count(page) - 1;*/
																																																																		page++;
																																																																					} while (page < end);
																																			}
									}

						seq_printf(m, "pages of RAM       %d\n", total);
							seq_printf(m, "free pages         %d\n", free);
								seq_printf(m, "reserved pages     %d\n", reserved);
									seq_printf(m, "slab pages         %d\n", slab);
										seq_printf(m, "pages shared       %d\n", shared);
											seq_printf(m, "pages swap cached  %d\n", cached);
												seq_printf(m, "other pages        %d\n", other);

													return 0;
}

#define MLK_ROUNDUP(b, t) b, t, DIV_ROUND_UP(((t) - (b)), SZ_1K)

static int _phymem_dist_proc_show(struct seq_file *m, void *v)
{ 
		seq_printf(m, "stack      0x%d kB\n", global_page_state(NR_KERNEL_STACK) * THREAD_SIZE / 1024);
			seq_printf(m, "pagetable  0x%d kB\n", K(global_page_state(NR_PAGETABLE)));
				seq_printf(m, "slab       0x%d kB\n", K(global_page_state(NR_SLAB_RECLAIMABLE) +global_page_state(NR_SLAB_UNRECLAIMABLE)));
					seq_printf(m, "free       0x%d kB\n", K(global_page_state(NR_FREE_PAGES)));

						return 0;
}


static int _phymem_map_proc_open(struct inode* inode, struct file*  file)
{
	    single_open(file, _phymem_map_proc_show, NULL);

		    return 0;
}

static int _phymem_pages_proc_open(struct inode* inode, struct file*  file)
{
	    single_open(file, _phymem_pages_proc_show, NULL);

		    return 0;
}

static int _phymem_dist_proc_open(struct inode* inode, struct file*  file)
{
	    single_open(file, _phymem_dist_proc_show, NULL);

		    return 0;
}

static int _phymem_showmem_proc_show(char *buf, char **start, off_t offset,
		                        int len, int *unused_i, void *unused_v)
{
		show_mem();

		    return 0;
}

static struct file_operations _phymem_map_proc_fops = {
	    .open    = _phymem_map_proc_open,
		    .read    = seq_read,
				.llseek  = seq_lseek,
				    .release = single_release,
};

static struct file_operations _phymem_pages_proc_fops = {
	    .open    = _phymem_pages_proc_open,
		    .read    = seq_read,
				.llseek  = seq_lseek,
				    .release = single_release,
};

static struct file_operations _phymem_dist_proc_fops = {
	    .open    = _phymem_dist_proc_open,
		    .read    = seq_read,
				.llseek  = seq_lseek,
				    .release = single_release,
};

static struct file_operations _phymem_showmem_proc_fops = {
	    .open    = NULL,
			.write   = _phymem_showmem_proc_show,
				.llseek  = NULL,
				    .release = NULL,
};

static struct proc_dir_entry* _phymem_proc_create(struct proc_dir_entry* parent, 
														const char* name, 
																										struct file_operation* fops)
{
		struct proc_dir_entry* file;

			file = create_proc_entry(name, (S_IFREG|S_IRUGO|S_IWUSR), parent);
				if (!file)
						{
									printk("phymem: error -- create_proc_entry(%s) failed\n", name);

											return NULL;
												}

					file->proc_fops = fops;

						return file;
}

static int _phymem_module_init(void)
{
		phymem = proc_mkdir("phymem", NULL);
			if(!phymem)
					{
								printk("phymem: error -- proc_mkdir(/proc/phymem) failed\n");

										return 0;
											}

				_phymem_proc_create(phymem, "map", &_phymem_map_proc_fops);
					_phymem_proc_create(phymem, "pages", &_phymem_pages_proc_fops);
						_phymem_proc_create(phymem, "dist", &_phymem_dist_proc_fops);
							_phymem_proc_create(phymem, "showmem-trigger", &_phymem_showmem_proc_fops);

								return 0;
}

module_init(_phymem_module_init);

