/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 * usbio.c -- USB I/O source file
 ******************************************************************************/
#ifndef LINUX_VERSION_CODE
#  include <linux/version.h>
#endif
#include "stdinc.h"
#include "udaqlite.h"
#include "buffer.h"
#include "usbio.h"

int udaqlite_init_endpoints( struct uDAQLiteDevice_t *dev )
{
	struct ADC_t *adc = &dev->adc;
	struct usb_device *ud;
	struct usb_interface *ui;
	struct usb_host_interface *if_desc;
	struct usb_endpoint_descriptor *ep_desc;
	int i;

	ud = dev->usbdev;
	ui = dev->usbif;
	if_desc = ui->cur_altsetting;

	adc->isourb_ep = 0;
	dev->ctrl_ep = 0;

	for(i=0; i<if_desc->desc.bNumEndpoints; i++) {
		ep_desc = &if_desc->endpoint[i].desc;
		if( !dev->ctrl_ep &&
		    (ep_desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
		    == USB_ENDPOINT_XFER_BULK) {
			dev->ctrl_ep = ep_desc->bEndpointAddress & 0xf;
		}
		else if( !adc->isourb_ep &&
			(ep_desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
			== USB_ENDPOINT_XFER_ISOC) {
			adc->isourb_ep = ep_desc->bEndpointAddress & 0xf;
			adc->isourb_ep_size = ep_desc->wMaxPacketSize;
			adc->isourb_ep_interval = ep_desc->bInterval;
		}
	}

	if(!adc->isourb_ep && !dev->ctrl_ep)
		return -ENOMEM;
	return 0;
}

int udaqlite_bulkxfr( struct uDAQLiteDevice_t *dev, int dir, unsigned char *buffer,
		    int *size )
{
	int ret;
	unsigned int pipe;
	int ln;

	if(dir == PACKET_DIR_OUT)
		pipe = usb_sndbulkpipe(dev->usbdev, dev->ctrl_ep);
	else
		pipe = usb_rcvbulkpipe(dev->usbdev, dev->ctrl_ep);

	mutex_lock(&dev->io_mutex);
	ret = usb_bulk_msg(dev->usbdev, pipe, buffer, *size, &ln, HZ*10);
	mutex_unlock(&dev->io_mutex);

	if(ret == -EPIPE) {
		drv_err("usbupclite: usb_bulk_msg stalled");
		if(usb_clear_halt(dev->usbdev,usb_pipeendpoint(pipe)))
			drv_err("usbupclite: failed to clear stalled endpoint");
	}

	*size = ln;
	return ret;
}

int udaqlite_bulkpacket( struct uDAQLiteDevice_t *dev, unsigned char *buffer,
		       unsigned int size )
{
	int retval;
	int sz;
	sz = size;

	retval = udaqlite_bulkxfr(dev, PACKET_DIR_OUT, buffer, &sz);
	if(retval) {
		err("%s - bulk message failed on write: %d",
			    __func__, retval);
		return retval;
	}

	sz = size;
	memset(buffer, 0, size);
	retval = udaqlite_bulkxfr(dev, PACKET_DIR_IN, buffer, &sz);
	if(retval)
		err("%s - bulk message failed on read: %d",
		    __func__, retval);

	return retval;
}

void udaqlite_isourb_waitforurbs( struct uDAQLiteDevice_t *dev )
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->adc.iso_submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->adc.iso_submitted);
}

/* iso streaming callback */
static void udaqlite_iso_callback( struct urb *urb )
{
	struct ADC_t *adc = urb->context;
	void *buf = urb->transfer_buffer;
	int i, len, status;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if(!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN || 
		    urb->status == -ENODEV
		))
			err("%s - nonzero read ISO status received: %d",
			    __func__, urb->status);

		/* signal a stop */
		clear_bit(1, &adc->status);

	}
	else {
		for(i=0; i<urb->number_of_packets; i++) {
			if((!urb->iso_frame_desc[i].status)) {
				len = urb->iso_frame_desc[i].actual_length;
				udaqlite_buf_adddata(&adc->buffer,
				  (void*)(buf + urb->iso_frame_desc[i].offset), len);
			}
		}
	}

	/* Check if we need to continue or start cleaning up.
	 * We might be stopping due to an error (URB unlink or unplugged) or adc_stop was called.
	 * All URB buffers will be freed here
	 */
	if(test_bit(1, &adc->status)) {
		/* re-queue the URB */
		status = usb_submit_urb (urb, GFP_ATOMIC);
	}
}

/* allocates an ISO URB */
int udaqlite_isourb_alloc( struct uDAQLiteDevice_t *dev )
{
	struct usb_device *usbdev = dev->usbdev;
	struct ADC_t *adc = &dev->adc;
	struct urb *urb = NULL;
	char *buf = NULL;
	unsigned int pipe = usb_rcvisocpipe(usbdev, adc->isourb_ep);
	int maxp;
	int pipesize = usb_maxpacket(usbdev, pipe, usb_pipeout(pipe));
	int packets = ISO_PIPESIZE / pipesize;
	int buffer_length = packets * pipesize;
	int i;
	int retval;

	maxp = 0x7ff & pipesize;
	maxp *= 1 + (0x3 & (pipesize >> 11));
	packets = (buffer_length + maxp - 1) / maxp;

	/* create a urb, and a buffer for it, and submit it  */
	urb = usb_alloc_urb(packets, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto udaqlite_isourb_alloc_error;
	}

	//buf = usb_buffer_alloc(usbdev, buffer_length, GFP_KERNEL, &urb->transfer_dma);
	buf = usb_alloc_coherent(usbdev, buffer_length, GFP_KERNEL, &urb->transfer_dma);	
	if (!buf) {
		retval = -ENOMEM;
		goto udaqlite_isourb_alloc_error;
	}

	/* setup iso buffer descriptors */
	for (i = 0; i < packets; i++) {
		urb->iso_frame_desc[i].length = pipesize;
		urb->iso_frame_desc[i].offset = pipesize * i;
	}

	/* this lock makes sure we don't submit URBs to gone devices */
	mutex_lock(&dev->io_mutex);
	if (!dev->usbif) {		/* disconnect() was called */
		mutex_unlock(&dev->io_mutex);
		retval = -ENODEV;
		goto udaqlite_isourb_alloc_error;
	}

	/* initialize the urb properly */
	usb_fill_iso_urb( urb, dev->usbdev, pipe, 
				buf, buffer_length, 
				udaqlite_iso_callback, adc,
				packets, adc->isourb_ep_interval,
				URB_ISO_ASAP | URB_NO_TRANSFER_DMA_MAP );

	/* add to URB list */
	usb_anchor_urb(urb, &adc->iso_submitted);

	/* submit URB to queue list */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	mutex_unlock(&dev->io_mutex);

	if (retval) {
		err("%s - failed submitting write urb, error %d", __func__, retval);
		goto udaqlite_isourb_alloc_error_submit;
	}

	return 0;

udaqlite_isourb_alloc_error_submit:
	usb_unanchor_urb(urb);

udaqlite_isourb_alloc_error:
	if(buf) {
		//usb_buffer_free(dev->usbdev, buffer_length, buf, urb->transfer_dma);
		usb_free_coherent(dev->usbdev, buffer_length, buf, urb->transfer_dma);		
	}
	if (urb) {
		usb_free_urb(urb);
	}

	return retval;
}


/**
 * End of File
 */
