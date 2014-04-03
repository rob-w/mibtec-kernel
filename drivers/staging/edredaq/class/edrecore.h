/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2011 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * edre.h -- EDR Enhanced DAQ Class Driver Header File
 *
 *******************************************************************************/
#include <linux/device.h>
#include <linux/cdev.h>

#define DRIVER_VERSION "v2.0.2"
#define MODULE_NAME "edredaq_core"
#define DRIVER_DESC "EDR Enhanced DAQ Class Driver"
#define EDRE_CLASS "edredaq"
#define EDRE_DEVICE_NAME_SIZE 20

struct edre_device {
	struct device dev;
	char name[EDRE_DEVICE_NAME_SIZE];
	struct module *owner;
	struct mutex ops_lock;
	struct file_operations *fops;
	unsigned long flags;
	struct cdev char_dev;
	int id;
	void *data;
};


#define to_edre_device(d) container_of(d, struct edre_device, dev)

/**
 * End of File
 */

