/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * edredaq.h -- EDR Enhanced DAQ class driver exported functions.
 *
 *******************************************************************************
 *                              LICENSE AGREEMENT
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ******************************************************************************/
#ifndef _EDREDAQ_H_
#define _EDREDAQ_H_

extern int edredaq_register_dev( struct device *device,
			  struct file_operations *fops,
			  void *data);
extern int edredaq_deregister_dev( int minor );


#ifdef DEBUG
#define drv_dbg(format, arg...) printk(KERN_DEBUG "%s: - %s - " format "\n" , __FILE__ , __FUNCTION__ , ## arg)
#else
#define drv_dbg(format, arg...) do {} while (0)
#endif

#ifndef drv_err	
#define drv_err(format, arg...) printk(KERN_ERR "%s - %s - " format "\n" , __FILE__ , __FUNCTION__ , ## arg)
#endif
#ifndef drv_info
#define drv_info(format, arg...) printk(KERN_INFO format "\n", ## arg)
#endif
#ifndef drv_warn
#define drv_warn(format, arg...) printk(KERN_WARNING format "\n" ## arg)
#endif


#endif /*_EDREDAQ_H_*/

