/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2003 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * edreusb.h - EDR Enhanced USB related constants
 *
 *******************************************************************************
 *
 * GNU Lesser General Public License, version 2.1
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 ******************************************************************************/

/*! \file edreusb.h
    \brief EDR Enhanced USB include file

    This file defines all constants common to the USB embedded system.
*/

#ifndef __EDRE_USB__
#define __EDRE_USB__

/**
 * USB uPc Command Codes
 */
#define USB_QUERY 0x000a
#define USB_DIOWRITE 0x000b
#define USB_DIOREAD 0x000c
#define USB_ADCREAD 0x000d
#define USB_ADCSTART 0x000e
#define USB_ADCSTOP 0x000f
#define USB_ADCFREQ 0x0010
#define USB_ADCGETDATA 0x0012
#define USB_CTWRITE 0x0013
#define USB_CTREAD 0x0014
#define USB_CTCONFIG 0x0015
#define USB_CTSOFTGATE 0x0016
#define USB_ADCADDCH 0x0017
#define USB_ADCCHLSRST 0x0018
#define USB_IRQCFG 0x0019
#define USB_IRQSTAT 0x001a
#define USB_IRQEN 0x001b
#define USB_IRQDIS 0x001c
#define USB_IRQGEN 0x001d
#define USB_ADCLKSRC 0x001e
#define USB_ADCADDCH16 0x0020
#define USB_DACWRITE16 0x0021
#define USB_EEWRITE 0x0030
#define USB_EEREAD 0x0031
#define USB_DACWRITE 0x0040
#define USB_CALDACWRITE 0x0041

/**
 * USB Specific Query Codes
 */
#define USB_QRY_EEWRITE 2000
#define USB_QRY_EEREAD 2100
#define USB_QRY_TMP_EEWRITE 2200
#define USB_QRY_CALWRITE 1000
#define USB_QRY_IRQGEN 1100
#define USB_QRY_TMPOFFSET 1200
#define USB_QRY_TMPGAIN 1300
#define USB_QRY_TMPSAVE 1400

#endif /*__EDRE_USB__*/

/**
 * End of File
 */
