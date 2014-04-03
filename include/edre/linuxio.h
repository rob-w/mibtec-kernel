/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2003 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * linuxio.h - EDR Enhanced Linux Module I/O Control Codes
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
/*! \file linuxio.h
    \brief EDR Enhanced Linux device driver I/O control definition file.

    This file defines all the Linux I/O control macros and contants.
*/

#ifndef __EDRE_IOCTL__
#define __EDRE_IOCTL__

/** @defgroup PCIBOARDS EDR Enhanced PCI Boards
 *  @{
 */

/*!\def DEV_IOC_MAGIC The driver magic character. */
#define DEV_IOC_MAGIC 'e'
#define parg unsigned long
#define DATA_MODE_RAW 0
#define DATA_MODE_VOLT 1


#define DEV_IOC_RESET _IOWR(DEV_IOC_MAGIC,0,parg)
#define DEV_IOC_QUERY _IOWR(DEV_IOC_MAGIC,1,parg)
#define DEV_IOC_DIOCFG _IOWR(DEV_IOC_MAGIC,10,parg)
#define DEV_IOC_DIOREAD _IOWR(DEV_IOC_MAGIC,11,parg)
#define DEV_IOC_DIOWRITE _IOWR(DEV_IOC_MAGIC,12,parg)
#define DEV_IOC_MIOCFG _IOWR(DEV_IOC_MAGIC,13,parg)
#define DEV_IOC_INTCFG _IOWR(DEV_IOC_MAGIC,20,parg)
#define DEV_IOC_INTEN _IOWR(DEV_IOC_MAGIC,21,parg)
#define DEV_IOC_INTDIS _IOWR(DEV_IOC_MAGIC,22,parg)
#define DEV_IOC_INTWAIT _IOWR(DEV_IOC_MAGIC,23,parg)
#define DEV_IOC_INTRELEASE _IOWR(DEV_IOC_MAGIC,24,parg)
#define DEV_IOC_CTCFG _IOWR(DEV_IOC_MAGIC,30,parg)
#define DEV_IOC_CTREAD _IOWR(DEV_IOC_MAGIC,31,parg)
#define DEV_IOC_CTWRITE _IOWR(DEV_IOC_MAGIC,32,parg)
#define DEV_IOC_CTSOFTGATE _IOWR(DEV_IOC_MAGIC,33,parg)
#define DEV_IOC_DACFG _IOWR(DEV_IOC_MAGIC,40,parg)
#define DEV_IOC_DAWRITE _IOWR(DEV_IOC_MAGIC,41,parg)
#define DEV_IOC_DACONTROL _IOWR(DEV_IOC_MAGIC,42,parg)
#define DEV_IOC_DABLOCK _IOWR(DEV_IOC_MAGIC,43,parg)
#define DEV_IOC_ADSINGLE _IOWR(DEV_IOC_MAGIC,50,parg)
#define DEV_IOC_ADCFG _IOWR(DEV_IOC_MAGIC,51,parg)
#define DEV_IOC_ADCLKSRC _IOWR(DEV_IOC_MAGIC,52,parg)
#define DEV_IOC_ADCHLST _IOWR(DEV_IOC_MAGIC,53,parg)
#define DEV_IOC_ADFREQ _IOWR(DEV_IOC_MAGIC,54,parg)
#define DEV_IOC_ADSTART _IOWR(DEV_IOC_MAGIC,55,parg)
#define DEV_IOC_ADSTOP _IOWR(DEV_IOC_MAGIC,56,parg)
#define DEV_IOC_ADGETDATA _IOWR(DEV_IOC_MAGIC,57,parg)
#define DEV_IOC_ADGETDATARAW _IOWR(DEV_IOC_MAGIC,58,parg)
#define DEV_IOC_MAXNR 59

/** @} */

#endif /*__EDRE_IOCTL__*/
/**
 * End of File
 */
