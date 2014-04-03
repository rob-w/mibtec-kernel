/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 * udaqlite.c -- uDAQLite main source file
 ******************************************************************************/
#include "stdinc.h"
#include "version.h"
#include "udaqlite.h"
#include "usbio.h"
#include "deviceio.h"
#include "ioctl.h"
#include "buffer.h"
#include "fxloader.h"

/* module info */
#define DRIVER_AUTHOR "Jan Zwiegers, jan@eagle.co.za"
#define DRIVER_DESC "MicroDAQLite Device Driver (C) 2005 - 2011"
#define MODULE_NAME "udaqlitedrv" 

#define to_udaqlite_dev(d) container_of(d, struct uDAQLiteDevice_t, kref)

/* global varables */
static struct usb_driver udaqlite_driver;

static void udaqlite_delete( struct kref *kref )
{
	struct uDAQLiteDevice_t *dev = to_udaqlite_dev(kref);

	usb_put_dev(dev->usbdev);
	kfree(dev);
}

/* file operations */
static loff_t udaqlite_llseek(struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}

static int udaqlite_read(struct file *file, char *buf, size_t length, loff_t *
			offset)
{
	struct uDAQLiteDevice_t *dev;
	struct ADC_t *adc;
	struct DblBuffer_t *dblbuffer;
	unsigned long samples, tocopy, copied, transfered, size;
	int retval = 0;

	dev = (struct uDAQLiteDevice_t *)file->private_data;
	adc = &dev->adc;
	dblbuffer = &adc->dblbuffer;
	size = sizeof(long);

	mutex_lock(&dev->io_mutex);
	if (!dev->usbif) {		/* disconnect() was called */
		retval = -ENODEV;
		goto udaqlite_read_exit;
	}

	samples = length / size;
	transfered = 0;
	do {
		tocopy = samples;
		if(tocopy > (dblbuffer->size / size))
			tocopy = dblbuffer->size / size;
		copied = udaqlite_buf_getdata(adc, (long *)dblbuffer->data, tocopy);
		samples -= copied;
		retval = tocopy - copied;
		do {
			tocopy = copied;
			copied = copy_to_user( (void *)(buf + transfered * size),
						(void *)dblbuffer->data, tocopy * size);
			if(copied)
				copied /= size;
			transfered += tocopy - copied;
		}while(copied > 0);
	}while(samples > 0 || retval);
	transfered -= samples;

	if(retval)
		retval = -EFAULT;
	else
		retval = transfered * size;

udaqlite_read_exit:
	mutex_unlock(&dev->io_mutex);
	return retval;

}

static int udaqlite_write(struct file *file, const char *buf, size_t length,
			loff_t * offset)
{
	return -ESPIPE;
}

static int udaqlite_release( struct inode *inode, struct file *file )
{

	struct uDAQLiteDevice_t *dev;

	dev = (struct uDAQLiteDevice_t *)file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* allow the device to be autosuspended */
	mutex_lock(&dev->io_mutex);
	if (!--dev->open_count && dev->usbif)
		usb_autopm_put_interface(dev->usbif);
	mutex_unlock(&dev->io_mutex);

	/* decrement the count on our device */
	kref_put(&dev->kref, udaqlite_delete);
	return 0;
}

static int udaqlite_open(struct inode *inode, struct file *file)
{
	struct uDAQLiteDevice_t *dev;

	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&udaqlite_driver, subminor);
	if (!interface) {
		drv_err("%s: error, can't find device for minor %d", MODULE_NAME, subminor);
		retval = -ENODEV;
		goto udaqlite_open_exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto udaqlite_open_exit;
	}

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* lock the device to allow correctly handling errors
	 * in resumption */
	mutex_lock(&dev->io_mutex);

	if (!dev->open_count++) {
		retval = usb_autopm_get_interface(interface);
			if (retval) {
				dev->open_count--;
				mutex_unlock(&dev->io_mutex);
				kref_put(&dev->kref, udaqlite_delete);
				goto udaqlite_open_exit;
			}
	}

	/* save our object in the file's private structure */
	file->private_data = dev;
	mutex_unlock(&dev->io_mutex);

udaqlite_open_exit:
	return retval;
}

static int udaqlite_ioctl(struct file *file, unsigned int
			cmd, unsigned long arg)
{
	struct uDAQLiteDevice_t *dev = (struct uDAQLiteDevice_t *)file->private_data;
	int retval = 0;

	if(_IOC_TYPE(cmd) != DEV_IOC_MAGIC)
		return -EINVAL;
	if(_IOC_NR(cmd) > DEV_IOC_MAXNR)
		return -EINVAL;
	if (dev == NULL)
		return -ENODEV;

	switch(cmd) {
	case DEV_IOC_QUERY:
		retval = udaqlite_ioctl_query(dev, arg);
		break;
	case DEV_IOC_DIOWRITE:
		retval = udaqlite_dio_write(dev, arg);
		break;
	case DEV_IOC_DIOREAD:
		retval = udaqlite_dio_read(dev, arg);
		break;
	case DEV_IOC_CTREAD:
		retval = udaqlite_cnt_read(dev, arg);
		break;
	case DEV_IOC_CTWRITE:
		retval = udaqlite_cnt_write(dev, arg);
		break;
	case DEV_IOC_CTCFG:
		retval = udaqlite_cnt_config(dev, arg);
		break;
	case DEV_IOC_CTSOFTGATE:
		retval = udaqlite_cnt_softgate(dev, arg);
		break;
	case DEV_IOC_DAWRITE:
		retval = udaqlite_dac_write(dev, arg);
		break;
	case DEV_IOC_ADSINGLE:
		retval = udaqlite_adc_single(dev, arg);
		break;
	case DEV_IOC_ADCFG:
		retval = udaqlite_adc_config(dev, arg);
		break;
	case DEV_IOC_ADCLKSRC:
		retval = udaqlite_adc_clksrc(dev, arg);
		break;
	case DEV_IOC_ADCHLST:
		retval = udaqlite_adc_chlst(dev, arg);
		break;
	case DEV_IOC_ADFREQ:
		retval = udaqlite_adc_freq(dev, arg);
		break;
	case DEV_IOC_ADSTART:
		retval = udaqlite_adc_start(dev, arg);
		break;
	case DEV_IOC_ADSTOP:
		retval = udaqlite_adc_stop(dev, arg);
		break;
	default:
		retval = -EINVAL;
	}

	return retval;
}

static ssize_t dev_show_serial( struct device *device, 
				struct device_attribute *attr,
				char *buf )
{
	struct uDAQLiteDevice_t *dev = (struct uDAQLiteDevice_t *)dev_get_drvdata(device);	
	return sprintf(buf,"%ld\n",dev->eeinfo.serialnumber);
}
static DEVICE_ATTR(serial, S_IRUGO, dev_show_serial, NULL);

static struct file_operations udaqlite_fops = {
	.owner = THIS_MODULE,
	.open = udaqlite_open,
	.release = udaqlite_release,
	.unlocked_ioctl = udaqlite_ioctl,
	.read = udaqlite_read,
	.write = udaqlite_write,
	.llseek = udaqlite_llseek,
};

static int udaqlite_probe(struct usb_interface *interface,
			const struct usb_device_id *id)
{
	struct uDAQLiteDevice_t *dev;
	int retval = -ENOMEM;

	/* see if firmware is loaded */
	if(id->idProduct == 0x0101l) {
		drv_info("MicroDAQLite attached to %d. Loading firmware and rebooting...", interface->minor);		
		if(fxldr_load_firmware(usb_get_dev(interface_to_usbdev(interface)))) {			
			drv_err("%s: Failed to download firmware.", MODULE_NAME);
		}
		return -ENODEV;
	}

	drv_info("MicroDAQLite attached to %d. Now booted and ready.", interface->minor);

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {		
		drv_err("%s: Failed to allocate memory.", MODULE_NAME);
		goto udaqlite_probe_error;
	}
	kref_init(&dev->kref);
	mutex_init(&dev->io_mutex);

	dev->usbdev = usb_get_dev(interface_to_usbdev(interface));
	dev->usbif = interface;

	if(udaqlite_init_endpoints(dev)) {		
		drv_err("%s: Failed to initialize MicroDAQLite endpoints..", MODULE_NAME);
		goto udaqlite_probe_error;
	}

	/* make sure CPU is running */
	fxldr_run(dev->usbdev);

	/* get device information */
	if(udaqlite_get_deviceinfo(dev)) {		
		drv_err("%s: Failed to retrieve MicroDAQLite device information.", MODULE_NAME);
		goto udaqlite_probe_error;
	}

	/* initialize the device properties */
	udaqlite_set_devicetype(dev);
	
	/* initialize the hardware sub-system */
	udaqlite_init_device(dev);

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	device_create_file(&interface->dev, &dev_attr_serial);

	/* register device with DAQ class driver */
	interface->minor = edredaq_register_dev( &interface->dev,
					    &udaqlite_fops,
					    (void *)dev );
	return 0;

udaqlite_probe_error:
	if(dev)
		/* this frees allocated memory */
		kref_put(&dev->kref, udaqlite_delete);
	return retval;
}

static void udaqlite_disconnect (struct usb_interface *interface)
{
	struct uDAQLiteDevice_t *dev;
	int minor = interface->minor;

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);
	device_remove_file(&interface->dev, &dev_attr_serial);

	udaqlite_uninit_device(dev);

	/* give back our minor */	
	edredaq_deregister_dev(minor);

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->usbif = NULL;
	mutex_unlock(&dev->io_mutex);

	/* decrement our usage count */
	kref_put(&dev->kref, udaqlite_delete);
	drv_info("MicroDAQLite #%ld now disconnected", dev->eeinfo.serialnumber);
	
}


static struct usb_device_id udaqlite_ids_tbl[] = {
	{ USB_DEVICE(0xea01l, 0x0100l) }, /* USB MicroDAQLite Raw*/
	{ USB_DEVICE(0xea01l, 0x0101l) }, /* USB MicroDAQLite Firmware Loaded*/
	{ } /*nothing*/
};

MODULE_DEVICE_TABLE(usb,udaqlite_ids_tbl);

static struct usb_driver udaqlite_driver = {
	.name = MODULE_NAME,
	.probe = udaqlite_probe,
	.disconnect = udaqlite_disconnect,
	.id_table = udaqlite_ids_tbl,
};

static ssize_t drv_show_version(struct device_driver *drv, char *buf)
{
	strcpy(buf, DRIVER_VERSION);
	return strlen(DRIVER_VERSION) + 1;
}

static DRIVER_ATTR(version, S_IRUGO, drv_show_version, NULL);

static int __init udaqlite_init(void)
{
	int result;

	drv_info("%s (%s)", DRIVER_DESC, DRIVER_VERSION);
	
	/* register this driver with the USB subsystem */
	result = usb_register(&udaqlite_driver);

	if(result) {		
		drv_err("%s: Failed to register driver with USB sub-system.", MODULE_NAME);
		return -ENODEV;
	}
	
	driver_create_file(&udaqlite_driver.drvwrap.driver, &driver_attr_version);
	return result;
}

static void __exit udaqlite_cleanup(void)
{	
	driver_remove_file(&udaqlite_driver.drvwrap.driver, &driver_attr_version);

	/* deregister this driver with the USB subsystem */
	usb_deregister(&udaqlite_driver);
}

module_init(udaqlite_init);
module_exit(udaqlite_cleanup);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/**
 * End of File
 */
