/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 * usbio.h -- USB I/O header file
 ******************************************************************************/
#ifndef _USBIO_H_
#define _USBIO_H_
int udaqlite_init_endpoints( struct uDAQLiteDevice_t *dev );
int udaqlite_bulkxfr( struct uDAQLiteDevice_t *dev, int dir, 
			unsigned char *buffer, int *size );
int udaqlite_bulkpacket( struct uDAQLiteDevice_t *dev, 
			unsigned char *buffer, unsigned int size );
void udaqlite_isourb_waitforurbs( struct uDAQLiteDevice_t *dev );
int udaqlite_isourb_alloc( struct uDAQLiteDevice_t *dev );
static inline void usb_fill_iso_urb( struct urb *urb,
				     struct usb_device *dev,
				     unsigned int pipe,
				     void *transfer_buffer,
				     int buffer_length,
				     usb_complete_t complete,
				     void *context,
				     int number_of_packets,
			             int interval,
				     unsigned int transfer_flags )
{

	urb->dev = dev;
	urb->pipe = pipe;
	urb->transfer_buffer = transfer_buffer;
	urb->transfer_buffer_length = buffer_length;
	urb->actual_length = 0;
	urb->complete = complete;
	urb->context = context;
	urb->number_of_packets = number_of_packets;
	urb->interval = 1 << (interval - 1);
	urb->transfer_flags = transfer_flags;
}
#endif /*_USBIO_H_*/

/**
 * End of File
 */
