/*
 * Copyright (C) 2006-2008 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; see the file COPYING. If not, write to the Free Software
 * Foundation, 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Test OOB read and write on MTD device.
 *
 * Author: Adrian Hunter <ext-adrian.hunter@nokia.com>
 */

#include <asm/div64.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/mtd/mtd.h>
#include <linux/sched.h>

#define PRINT_PREF KERN_INFO "mtd_oobtestwrite: "

static int dev;
module_param(dev, int, S_IRUGO);
MODULE_PARM_DESC(dev, "MTD device number to use");

static struct mtd_info *mtd;
static unsigned char *readbuf;
static unsigned char *writebuf;
static unsigned char *bbt;

static int ebcnt;
static int pgcnt;
static int errcnt;
static int use_offset;
static int use_len;
static int use_len_max;
static int vary_offset;
static unsigned long next = 1;

static inline unsigned int simple_rand(void)
{
	next = next * 1103515245 + 12345;
	return (unsigned int)((next / 65536) % 32768);
}

static inline void simple_srand(unsigned long seed)
{
	next = seed;
}

static void set_random_data(unsigned char *buf, size_t len)
{
	size_t i;

	for (i = 0; i < len; ++i)
		buf[i] = simple_rand();
}

static int erase_eraseblock(int ebnum)
{
	int err;
	struct erase_info ei;
	loff_t addr = ebnum * mtd->erasesize;

	memset(&ei, 0, sizeof(struct erase_info));
	ei.mtd  = mtd;
	ei.addr = addr;
	ei.len  = mtd->erasesize;

	err = mtd->erase(mtd, &ei);
	if (err) {
		printk(PRINT_PREF "error %d while erasing EB %d\n", err, ebnum);
		return err;
	}

	if (ei.state == MTD_ERASE_FAILED) {
		printk(PRINT_PREF "some erase error occurred at EB %d\n",
		       ebnum);
		return -EIO;
	}

	return 0;
}

static int erase_whole_device(void)
{
	int err;
	unsigned int i;

	printk(PRINT_PREF "erasing whole device\n");
	for (i = 0; i < ebcnt; ++i) {
		if (bbt[i])
			continue;
		err = erase_eraseblock(i);
		if (err)
			return err;
		cond_resched();
	}
	printk(PRINT_PREF "erased %u eraseblocks\n", i);
	return 0;
}

static void do_vary_offset(void)
{
	use_len -= 1;
	if (use_len < 1) {
		use_offset += 1;
		if (use_offset >= use_len_max)
			use_offset = 0;
		use_len = use_len_max - use_offset;
	}
}

static int write_eraseblock(int ebnum)
{
	int i,aaa;
	struct mtd_oob_ops ops;
	int err = 0;
	loff_t addr = ebnum * mtd->erasesize;
	//printk("pgcnt=%d  use_len=%d\n", pgcnt, use_len);
	for (i = 0; i < pgcnt; ++i, addr += mtd->writesize) {
		set_random_data(writebuf, use_len);
		/*for (aaa = 0; aaa < use_len; aaa++)
			printk("writebuf1[%d]=0x%02x  ", aaa, writebuf[aaa]);*/
		ops.mode      = MTD_OOB_AUTO;
		ops.len       = 0;
		ops.retlen    = 0;
		ops.ooblen    = use_len;
		ops.oobretlen = 0;
		ops.ooboffs   = use_offset;
		ops.datbuf    = NULL;
		ops.oobbuf    = writebuf;
		err = mtd->write_oob(mtd, addr, &ops);
		//printk("err=%d  use_len=%d  oobretlen=%d\n", err, use_len, ops.oobretlen);
		if (err || ops.oobretlen != use_len) {
			printk(PRINT_PREF "error: writeoob failed at %#llx\n",
			       (long long)addr);
			printk(PRINT_PREF "error: use_len %d, use_offset %d\n",
			       use_len, use_offset);
			errcnt += 1;
			return err ? err : -1;
		}
		if (vary_offset)
			do_vary_offset();
	}

	return err;
}

static int write_whole_device(void)
{
	int err;
	unsigned int i;

	printk(PRINT_PREF "writing OOBs of whole device\n");
	for (i = 0; i < ebcnt; ++i) {
		if (bbt[i]) {
			continue;
		}

		err = write_eraseblock(i);
		if (err)
			return err;
		if (i % 256 == 0)
			printk(PRINT_PREF "written up to eraseblock %u\n", i);
		cond_resched();
	}
	printk(PRINT_PREF "written %u eraseblocks\n", i);
	return 0;
}

static int is_block_bad(int ebnum)
{
	int ret;
	loff_t addr = ebnum * mtd->erasesize;

	ret = mtd->block_isbad(mtd, addr);
	if (ret)
		printk(PRINT_PREF "block %d is bad\n", ebnum);
	return ret;
}

static int scan_for_bad_eraseblocks(void)
{
	int i, bad = 0;

	bbt = kmalloc(ebcnt, GFP_KERNEL);
	if (!bbt) {
		printk(PRINT_PREF "error: cannot allocate memory\n");
		return -ENOMEM;
	}
	memset(bbt, 0 , ebcnt);

	printk(PRINT_PREF "scanning for bad eraseblocks\n");
	for (i = 0; i < ebcnt; ++i) {
		bbt[i] = is_block_bad(i) ? 1 : 0;
		if (bbt[i])
			bad += 1;
		cond_resched();
	}
	printk(PRINT_PREF "scanned %d eraseblocks, %d are bad\n", i, bad);
	return 0;
}

static int __init mtd_oobtestwrite_init(void)
{
	int err = 0;
	unsigned int i;
	uint64_t tmp;
	struct mtd_oob_ops ops;
	loff_t addr = 0, addr0;

	printk(KERN_INFO "\n");
	printk(KERN_INFO "=================================================\n");
	printk(PRINT_PREF "MTD device: %d\n", dev);

	mtd = get_mtd_device(NULL, dev);
	if (IS_ERR(mtd)) {
		err = PTR_ERR(mtd);
		printk(PRINT_PREF "error: cannot get MTD device\n");
		return err;
	}

	if (mtd->type != MTD_NANDFLASH) {
		printk(PRINT_PREF "this test requires NAND flash\n");
		goto out;
	}

	tmp = mtd->size;
	do_div(tmp, mtd->erasesize);
	ebcnt = tmp;
	pgcnt = mtd->erasesize / mtd->writesize;

	printk(PRINT_PREF "MTD device size %llu, eraseblock size %u, "
	       "page size %u, count of eraseblocks %u, pages per "
	       "eraseblock %u, OOB size %u\n",
	       (unsigned long long)mtd->size, mtd->erasesize,
	       mtd->writesize, ebcnt, pgcnt, mtd->oobsize);

	ebcnt = 1;//fcjadd

	err = -ENOMEM;
	mtd->erasesize = mtd->erasesize;
	readbuf = kmalloc(mtd->erasesize, GFP_KERNEL);
	if (!readbuf) {
		printk(PRINT_PREF "error: cannot allocate memory\n");
		goto out;
	}
	writebuf = kmalloc(mtd->erasesize, GFP_KERNEL);
	if (!writebuf) {
		printk(PRINT_PREF "error: cannot allocate memory\n");
		goto out;
	}

	err = scan_for_bad_eraseblocks();
	if (err)
		goto out;

	use_offset = 0;
	use_len = mtd->ecclayout->oobavail;
	use_len_max = mtd->ecclayout->oobavail;
	vary_offset = 0;

	/* First test: write all OOB, read it back and verify */
	printk(PRINT_PREF "test 1 of 5\n");

	simple_srand(1);
	err = write_whole_device();
	if (bbt)
		kfree(bbt);
	if (writebuf)
		kfree(writebuf);
	if (readbuf)
		kfree(readbuf);
	put_mtd_device(mtd);
out:
	printk(KERN_INFO "=================================================\n");
	return err;
}
module_init(mtd_oobtestwrite_init);

static void __exit mtd_oobtestwrite_exit(void)
{
	return;
}
module_exit(mtd_oobtestwrite_exit);

MODULE_DESCRIPTION("Out-of-band test module");
MODULE_AUTHOR("Adrian Hunter");
MODULE_LICENSE("GPL");
