/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2011 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * edreproc.c -- EDR Enhanced DAQ proc interface
 *
 * edreproc skeleton from drivers/rtc/rts-proc.c
 *
 *******************************************************************************/

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "edrecore.h"

static int edre_proc_show(struct seq_file *seq, void *offset)
{
//	int err;
//	struct edre_device *edre = seq->private;

	seq_printf(seq, "class driver version\t: %s\n", DRIVER_VERSION);

//	if (edre->fops->proc)
//		ops->proc(edre->dev.parent, seq);

	return 0;
}

static int edre_proc_open(struct inode *inode, struct file *file)
{
	struct edre_device *edre = PDE(inode)->data;

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	return single_open(file, edre_proc_show, edre);
}

static int edre_proc_release(struct inode *inode, struct file *file)
{
	int res = single_release(inode, file);
	module_put(THIS_MODULE);
	return res;
}

static const struct file_operations edre_proc_fops = {
	.open		= edre_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= edre_proc_release,
};

void edre_proc_add_device(struct edre_device *edre)
{
	if (edre->id == 0) {
		struct proc_dir_entry *ent;

		ent = proc_create_data("driver/edredaq", 0, NULL,
				       &edre_proc_fops, edre);
		//if (ent)
			//ent->owner = edre->owner;
	}
}

void edre_proc_del_device(struct edre_device *edre)
{
	if (edre->id == 0)
		remove_proc_entry("driver/edredaq", NULL);
}


/**
 * End of File
 */

