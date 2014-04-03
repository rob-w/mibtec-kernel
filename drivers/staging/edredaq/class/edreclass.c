/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2011 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * edreclass.c -- EDR Enhanced DAQ class driver
 *
 * class skeleton from drivers/rtc/class.c
 *
 *******************************************************************************/

#define EXPORT_SYMTAB
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kdev_t.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/idr.h>
#include <linux/err.h>
#include "edrecore.h"
#include "edredef.h"

static DEFINE_IDR(edre_idr);
static DEFINE_MUTEX(edre_lock);
static struct class *edre_class;

static void edre_device_release( struct device *dev )
{
	struct edre_device *edre = to_edre_device(dev);
	mutex_lock(&edre_lock);
	idr_remove(&edre_idr, edre->id);
	mutex_unlock(&edre_lock);
	kfree(edre);
}

static int edredaq_register_dev( struct device *device,
			  struct file_operations *fops,
			  void *data)
{
	struct edre_device *edre;
	int id, err;

	if (idr_pre_get(&edre_idr, GFP_KERNEL) == 0) {
		err = -ENOMEM;
		goto exit;
	}


	mutex_lock(&edre_lock);
	err = idr_get_new(&edre_idr, NULL, &id);
	mutex_unlock(&edre_lock);

	if (err < 0)
		goto exit;

	id = id & MAX_ID_MASK;

	edre = kzalloc(sizeof(struct edre_device), GFP_KERNEL);
	if (edre == NULL) {
		err = -ENOMEM;
		goto exit_idr;
	}

	edre->id = id;
	edre->fops = fops;
	edre->data = data;
	edre->dev.parent = device;
	edre->dev.class = edre_class;
	edre->dev.release = edre_device_release;

	mutex_init(&edre->ops_lock);
	
	snprintf(edre->name, EDRE_DEVICE_NAME_SIZE, "device%d", id);
	
	dev_set_name(&edre->dev, "edredaq%d", id);
	
	edre_dev_prepare(edre);

	err = device_register(&edre->dev);
	if (err)
		goto exit_kfree;

	edre_dev_add_device(edre);
	edre_sysfs_add_device(edre);
	edre_proc_add_device(edre);

	mutex_lock(&edre_lock);
	idr_replace(&edre_idr, edre, id);
	mutex_unlock(&edre_lock);

	printk(KERN_INFO "%s: registered as %s\n", MODULE_NAME, dev_name(&edre->dev));	

	return id;

exit_kfree:
	kfree(edre);

exit_idr:
	mutex_lock(&edre_lock);
	idr_remove(&edre_idr, id);
	mutex_unlock(&edre_lock);

exit:
	printk(KERN_ERR "%s: unable to register device err = %d\n", MODULE_NAME, err);
	return err;
}
EXPORT_SYMBOL_GPL(edredaq_register_dev);

static void edredaq_deregister_dev(int id)
{
	struct edre_device *edre = idr_find(&edre_idr, id);

	printk(KERN_ERR "%s: edredaq_deregister_dev - trying to de-register %d\n", MODULE_NAME, id);

	if(edre != NULL) {
		printk(KERN_ERR "%s: found and removing device: %s\n", MODULE_NAME, edre->name);
		mutex_lock(&edre->ops_lock);
		edre_sysfs_del_device(edre);
		edre_dev_del_device(edre);
		edre_proc_del_device(edre);
		device_unregister(&edre->dev);
		edre->fops = NULL;
		mutex_unlock(&edre->ops_lock);
		put_device(&edre->dev);
	}
}
EXPORT_SYMBOL_GPL(edredaq_deregister_dev);

static int __init edre_init(void)
{
	printk(KERN_INFO"%s:%s : %s\n",MODULE_NAME, DRIVER_VERSION, DRIVER_DESC);

	edre_class = class_create(THIS_MODULE, EDRE_CLASS);
	if (IS_ERR(edre_class)) {
		printk(KERN_ERR"%s: Failed to register EDR Enhanced Class.\n", MODULE_NAME);
		return PTR_ERR(edre_class);
	}
	
	edre_class->dev_release = edre_device_release;
	edre_dev_init();
	edre_sysfs_init(edre_class);

	return 0;
}

static void __exit edre_cleanup(void)
{
	printk(KERN_INFO"%s: driver unloading...\n",MODULE_NAME);
	edre_dev_exit();
	class_destroy(edre_class);
}

module_init(edre_init);
module_exit(edre_cleanup);
MODULE_AUTHOR ("Jan Zwiegers - Eagle Technology, jan@eagle.co.za");
MODULE_DESCRIPTION ("EDR Enhanced DAQ Class Driver (C) 2004-2011 \
Eagle Technology");
MODULE_LICENSE("GPL");

/**
 * End of File
 */

