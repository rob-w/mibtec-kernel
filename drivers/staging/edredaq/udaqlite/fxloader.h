/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 * fxloader.h -- Loads firmware into EZ-USB device
 ******************************************************************************/
 
#ifndef _FXLOADER_H_
#define _FXLOADER_H_
int fxldr_reset( struct usb_device *device );
int fxldr_run( struct usb_device *device );
int fxldr_load_firmware( struct usb_device *ud );
#endif /* _FXLOADER_H_ */
