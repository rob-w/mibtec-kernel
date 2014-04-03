/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2011 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * edresysfs.c -- EDR Enhanced DAQ class driver
 * 
 * edresysfs skeleton from drivers/rtc/rts-sysfs.c
 *
 *******************************************************************************/
#include <linux/module.h>
#include "edrecore.h"

static ssize_t edre_sysfs_edredaq_nodename(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int minor = MINOR(dev->devt);
	return sprintf(buf, "device%d\n", minor);
}
static DEVICE_ATTR(edredaq_nodename, S_IRUGO, edre_sysfs_edredaq_nodename, NULL);


static ssize_t edre_sysfs_dev(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return print_dev_t(buf, dev->devt);
}
static DEVICE_ATTR(show_device, S_IRUGO, edre_sysfs_dev, NULL);


void edre_sysfs_add_device(struct edre_device *edre)
{
	int err;

	err = device_create_file(&edre->dev, &dev_attr_show_device);
	if (err)
		dev_err(edre->dev.parent,
			"failed to create device attribute, %d\n", err);

	err = device_create_file(&edre->dev, &dev_attr_edredaq_nodename);
	if (err) {
		device_remove_file(&edre->dev, &dev_attr_show_device);
		dev_err(edre->dev.parent,
			"failed to create device name attribute, %d\n", err);
	}
}

void edre_sysfs_del_device(struct edre_device *edre)
{
	device_remove_file(&edre->dev, &dev_attr_show_device);
	device_remove_file(&edre->dev, &dev_attr_edredaq_nodename);
}

static ssize_t edre_sysfs_show_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_VERSION);
}

static struct device_attribute edre_attrs[] = {
	__ATTR(version, S_IRUGO, edre_sysfs_show_version, NULL),
	{ },
};

void __init edre_sysfs_init(struct class *edre_class)
{
	edre_class->dev_attrs = edre_attrs;
}


/**
 * End of File
 */

