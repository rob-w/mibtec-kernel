/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * edredev.c -- EDR Enhanced DAQ Device Interface Driver
 *
 * edredev skeleton from drivers/rtc/rts-dev.c
 *
 *******************************************************************************/

#include <linux/module.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include "edrecore.h"

static DEFINE_SEMAPHORE(open_sem);
static dev_t edre_devt;

#define EDRE_DEV_MAX 16

static int edre_open(struct inode *inode, struct file *file)
{
	struct edre_device *edre = container_of(inode->i_cdev, struct edre_device, char_dev);
	const struct file_operations *old_fops, *new_fops = NULL;
	int err = -ENODEV;

	if(down_interruptible(&open_sem))
		return -ERESTARTSYS;

	new_fops = fops_get(edre->fops);
	old_fops = file->f_op;
	file->f_op = new_fops;
	file->private_data = edre->data;
	if (file->f_op->open) {
		err = file->f_op->open(inode, file);
	}
	if (err) {
		fops_put(file->f_op);
		file->f_op = fops_get(old_fops);
	}
	fops_put(old_fops);

	up(&open_sem);
	return err;
}

static const struct file_operations edre_dev_fops = {
	.owner =	THIS_MODULE,
	.open =		edre_open,
};

void edre_dev_prepare( struct edre_device *edre )
{
	if (!edre_devt)
		return;

	if (edre->id >= EDRE_DEV_MAX) {
		printk(KERN_ERR"%s: Too many EDR Enhanced devices registered.\n", MODULE_NAME);
		return;
	}

	edre->dev.devt = MKDEV(MAJOR(edre_devt), edre->id);

	cdev_init(&edre->char_dev, &edre_dev_fops);
	edre->char_dev.owner = edre->owner;
}

void edre_dev_add_device( struct edre_device *edre )
{
	if (cdev_add(&edre->char_dev, edre->dev.devt, 1))
		printk(KERN_WARNING "%s: failed to add char device %d:%d\n",
			MODULE_NAME, MAJOR(edre_devt), edre->id);
	else
		printk(KERN_INFO"%s: dev (%d:%d)\n", edre->name,
			MAJOR(edre_devt), edre->id);
}

void edre_dev_del_device(struct edre_device *edre)
{
	if (edre->dev.devt)
		cdev_del(&edre->char_dev);
}

int __init edre_dev_init(void)
{
	int err;

	err = alloc_chrdev_region(&edre_devt, 0, EDRE_DEV_MAX, EDRE_CLASS);
	if (err < 0) {
		printk(KERN_ERR"%s: failed to allocate char dev region\n", MODULE_NAME);
		return -ENOMEM;
	}

	printk(KERN_INFO"%s: major number = %d\n", MODULE_NAME, MAJOR(edre_devt));

	return 0;
}

void __exit edre_dev_exit(void)
{
	if (edre_devt)
		unregister_chrdev_region(edre_devt, EDRE_DEV_MAX);
}

/**
 * End of File
 */

