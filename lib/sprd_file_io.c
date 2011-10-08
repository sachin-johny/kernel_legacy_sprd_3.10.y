#include <linux/module.h>
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/sprd_file_io.h>

/* Opening a file, similar to open(): */
struct file* sprd_file_open(const char* path, int flags, int rights)
{
	struct file* filp = NULL;
	mm_segment_t oldfs;
	int err = 0;

	oldfs = get_fs();
	set_fs(get_ds());
	filp = filp_open(path, flags, rights);
	set_fs(oldfs);
	if(IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}
	return filp;
}
 
EXPORT_SYMBOL_GPL(sprd_file_open);

/* Close a file, similar to close() */

void sprd_file_close(struct file* file)
{
	filp_close(file, NULL);
}

EXPORT_SYMBOL_GPL(sprd_file_close);

/* Reading data from a file, similar to pread() */
int sprd_file_read(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size)
{
	mm_segment_t oldfs;
	int ret;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_read(file, data, size, &offset);

	set_fs(oldfs);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_file_read);

/* Writing data to a file, similar to pwrite() */
int sprd_file_write(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size)
{
	mm_segment_t oldfs;
	int ret;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_write(file, data, size, &offset);

	set_fs(oldfs);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_file_write);

/* Syncing changes a file, similar to fsync() */
int sprd_file_sync(struct file* file)
{
	file_fsync(file, file->f_dentry, 0);
	return 0;
}

EXPORT_SYMBOL_GPL(sprd_file_sync);

