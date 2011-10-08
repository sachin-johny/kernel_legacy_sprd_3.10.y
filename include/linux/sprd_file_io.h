#ifndef _LINUX_SPRD_FILE_IO_H_
#define _LINUX_SPRD_FILE_IO_H_

extern struct file* sprd_file_open(const char* path, int flags, int rights);
extern void sprd_file_close(struct file* file);
extern int sprd_file_read(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size);
extern int sprd_file_write(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size);
extern int sprd_file_sync(struct file* file);

#endif
