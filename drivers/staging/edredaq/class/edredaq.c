/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2011 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * edreclass.c -- EDR Enhanced DAQ class driver
 *
 *******************************************************************************/

#define DRIVER_VERSION "v2.0.2"

#ifndef LINUX_VERSION_CODE
#  include <linux/version.h>
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
# error "This driver needs kernel 2.6.30 and later"
#endif

#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif

#ifndef LINUX_VERSION_CODE
#  include <linux/version.h>
#endif

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>

/* module info */
#define DRIVER_AUTHOR "Jan Zwiegers, jan@eagle.co.za"
#define DRIVER_DESC "EDR Enhanced DAQ Class Driver"
#define MODULE_NAME "edredaq_core"
#define EDRE_CLASS "edredaq"
#define EDRE_MAX_DEVICES 16

struct device_class
{
	struct file_operations *fops;
	struct class_device *class_dev;
	void *data;
};

static spinlock_t minor_lock = SPIN_LOCK_UNLOCKED;
static int major;
static struct device_class minors[EDRE_MAX_DEVICES];

static int edre_open(struct inode * inode, struct file * file)
{
	int minor = iminor(inode);
	struct file_operations *c;
	int err = -ENODEV;
	const struct file_operations *old_fops, *new_fops = NULL;

	spin_lock (&minor_lock);
	c = minors[minor].fops;

	if (!c || !(new_fops = fops_get(c))) {
		spin_unlock(&minor_lock);
		return err;
	}
	spin_unlock(&minor_lock);

	old_fops = file->f_op;
	file->f_op = new_fops;
	file->private_data = minors[minor].data;
	if (file->f_op->open) {
		err = file->f_op->open(inode,file);
	}
	if (err) {
		fops_put(file->f_op);
		file->f_op = fops_get(old_fops);
	}
	fops_put(old_fops);
	return err;
}

static ssize_t show_devicename(struct class_device *class_dev, char *buf)
{
	char s[10];
	int minor = (int)(long)class_get_devdata(class_dev);
	sprintf(s,"device%d\n", minor);
	strcpy(buf,s);
	return strlen(s) + 1;
}
static CLASS_DEVICE_ATTR(edredaq_nodename, S_IRUGO, show_devicename, NULL);


static void edre_device_release(struct class_device *class_dev)
{
	int minor = (int)(long)class_get_devdata(class_dev);
	printk(KERN_INFO"%s: Released node %d\n", MODULE_NAME, minor);
	kfree(class_dev);
}

static ssize_t class_show_version(struct class *class, char * buf)
{
	strcpy(buf, DRIVER_VERSION);
	return strlen(DRIVER_VERSION) + 1;
}

static CLASS_ATTR(version, S_IRUGO, class_show_version, NULL);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,9)
static ssize_t show_dev(struct class_device *class_dev, char *buf)
{
	int minor = (int)(long)class_get_devdata(class_dev);
	return print_dev_t(buf, MKDEV(major, minor));
}
static CLASS_DEVICE_ATTR(dev, S_IRUGO, show_dev, NULL);
#endif

static struct class edre_class = {
	.name		= EDRE_CLASS,
	.release	= &edre_device_release,
};


int edredaq_find_next_index(void)
{
	int i;
	for(i=0; i<EDRE_MAX_DEVICES; i++) {
		if(minors[i].fops == NULL) {
			return i;
		}
	}
	return -1;
}

static int edredaq_register_dev( struct device *device,
			  struct file_operations *fops,
			  void *data)
{
	struct class_device *class_dev;
	int newminor;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)	
	int rc;
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,9)	
	char name[40];
#endif
	newminor = edredaq_find_next_index();
	if(newminor<0)
		return -ENOMEM;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,9)


	class_dev = class_device_create( &edre_class, NULL, MKDEV(major,newminor), device, 
											"device%d", newminor);
	if(class_dev == NULL)
		return -ENOMEM;

	spin_lock (&minor_lock);
	minors[newminor].fops = fops;
	minors[newminor].data = data;
	minors[newminor].class_dev = class_dev;
	spin_unlock (&minor_lock);

#else

	class_dev = kmalloc(sizeof(*class_dev), GFP_KERNEL);

	if(!class_dev) {
		printk(KERN_ERR"%s: No memory left.\n", MODULE_NAME);
		return -ENOMEM;
	}

	spin_lock (&minor_lock);
	minors[newminor].fops = fops;
	minors[newminor].data = data;
	minors[newminor].class_dev = class_dev;
	spin_unlock (&minor_lock);

	memset(class_dev, 0x00, sizeof(struct class_device));
	class_dev->class = &edre_class;
	class_dev->dev = device;

	sprintf(name,"device%d",newminor);
	snprintf(class_dev->class_id, BUS_ID_SIZE, "%s", name);
	class_set_devdata(class_dev, (void *)(long)newminor);
	class_device_register(class_dev);
	class_device_create_file(class_dev, &class_device_attr_dev);
	
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)	
	class_device_create_file(class_dev, &class_device_attr_edredaq_nodename);
#else
	rc = class_device_create_file(class_dev, &class_device_attr_edredaq_nodename);
	if(rc) 
	    printk(KERN_WARNING"%s: Failed to create device file.\n", MODULE_NAME);
#endif

	printk(KERN_INFO"%s: Registered node %d\n", MODULE_NAME, newminor);
	return newminor;
}
EXPORT_SYMBOL_GPL(edredaq_register_dev);

static void edredaq_deregister_dev(int minor)
{
	if(!minors[minor].class_dev) {
		printk(KERN_ERR"%s: Trying to deregister invalid class\n", MODULE_NAME);
		return;
	}
	spin_lock (&minor_lock);
	minors[minor].fops = NULL;
	minors[minor].data = NULL;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,9)
	class_device_destroy(&edre_class, MKDEV(major,minor));
#else
	class_device_unregister(minors[minor].class_dev);
#endif

	spin_unlock (&minor_lock);
}
EXPORT_SYMBOL_GPL(edredaq_deregister_dev);

static struct file_operations edre_fops = {
	.owner =	THIS_MODULE,
	.open =		edre_open,
};

static int __init edre_init(void)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)
	int rc;
#endif	
	printk(KERN_INFO"%s : %s\n",DRIVER_VERSION,DRIVER_DESC);
	major = register_chrdev(0, MODULE_NAME, &edre_fops);
	if(major<0) {
		printk(KERN_ERR"%s: Unable to get major number\n", MODULE_NAME);
		return -ENOMEM;
	}
	printk(KERN_INFO"%s: major number = %d\n", MODULE_NAME, major);
	if(class_register(&edre_class)) {
		printk(KERN_ERR"%s: Failed to register EDR Enhanced Class.\n", MODULE_NAME);
		unregister_chrdev(major, MODULE_NAME);
		return -ENOMEM;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)	
	class_create_file(&edre_class, &class_attr_version);
#else
	rc = class_create_file(&edre_class, &class_attr_version);
#endif
	return 0;
}

static void __exit edre_cleanup(void)
{
	printk(KERN_INFO"%s: driver unloading...\n",MODULE_NAME);
	class_remove_file(&edre_class, &class_attr_version);
	class_unregister(&edre_class);
	unregister_chrdev(major, MODULE_NAME);
}

module_init(edre_init);
module_exit(edre_cleanup);
MODULE_AUTHOR ("Jan Zwiegers - Eagle Technology, jan@eagle.co.za");
MODULE_DESCRIPTION ("EDR Enhanced DAQ Class Driver (C) 2004-2009 \
Eagle Technology");
MODULE_LICENSE("GPL");

/**
 * End of File
 */

