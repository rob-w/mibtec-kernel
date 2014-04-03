/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 * deviceio.c -- Device I/O source file
 ******************************************************************************/
#ifndef LINUX_VERSION_CODE
#  include <linux/version.h>
#endif
#include "stdinc.h"
#include "udaqlite.h"
#include "usbio.h"
#include "buffer.h"
#include "ioctl.h"
#include "deviceio.h"

void udaqlite_eeinfo_dump( pEepromInfo_t ee )
{
#ifdef DEBUG
	err("udaqlite: udaqlite_eeinfo_dump.");
	err("udaqlite: serialnumber: %ld", ee->serialnumber);
	err("udaqlite: boardtype: %d", ee->boardtype);
	err("udaqlite: revison: %d", ee->revision);
	err("udaqlite: man date: %d-%d-%d", ee->day, ee->month, ee->year);
	err("udaqlite: firmware version: %d.%d.%d",
	ee->firmmajor,ee->firmminor,ee->firmbuild);
#endif /*DEBUG*/
}

int udaqlite_query_device( struct uDAQLiteDevice_t *dev, long code, long param,
			 unsigned long *value )
{
	unsigned char buf[64];
	unsigned long val;
	buf[0] = USB_QUERY>>8;
	buf[1] = USB_QUERY;
	buf[2] = code>>8;
	buf[3] = code;
	buf[4] = param>>8;
	buf[5] = param;
	udaqlite_bulkpacket(dev, buf, 64);
	if(buf[1] != 0)
		return -EIO;
	val = 0;
	val = buf[2]<<24 | buf[3]<<16 | buf[4]<<8 | buf[5];
	*value = val;
	return 0;
}

int udaqlite_eeprom_read( struct uDAQLiteDevice_t *dev, unsigned char offset )
{
	unsigned char buf[64];
	unsigned short qry = USB_EEREAD;
	unsigned short val;
	
	buf[0] = qry>>8;
	buf[1] = qry;
	buf[2] = 0;
	buf[3] = offset;
	udaqlite_bulkpacket(dev, buf, 64);
	
	if(buf[1] != 0)
		return -EIO;

	val = buf[5];
	
	return val;
}

int udaqlite_get_adcconfig( struct uDAQLiteDevice_t *dev )
{
	int i, ret;
	
	for(i=0; i<12; i++) {
		ret = udaqlite_eeprom_read(dev, 40 + i);
		if(ret < 0)
			return ret;
		dev->adc.offset[i] = ret;
	}
	
	ret = udaqlite_eeprom_read(dev, 52);
	if(ret < 0)
			return ret;
	
	dev->adc.gain_error = ret;
	return 0;
}

int udaqlite_get_dacconfig( struct uDAQLiteDevice_t *dev )
{
	int ret;

	ret = udaqlite_eeprom_read(dev, 30);
	if(ret < 0)
			return ret;
	dev->eeinfo.dac_offset[0] = ret;

	ret = udaqlite_eeprom_read(dev, 31);
	if(ret < 0)
			return ret;
	dev->eeinfo.dac_offset[1] = ret;

	ret = udaqlite_eeprom_read(dev, 32);
	if(ret < 0)
			return ret;
	dev->eeinfo.dac_gain_error[0] = ret;

	ret = udaqlite_eeprom_read(dev, 33);
	if(ret < 0)
			return ret;
	dev->eeinfo.dac_gain_error[1] = ret;

	return 0;
}

void udaqlite_set_devicetype( struct uDAQLiteDevice_t *dev )
{	
	pEepromInfo_t eeinfo=&dev->eeinfo;

	eeinfo->diochan = 2;
	eeinfo->cntchan = 1;
	eeinfo->adcchan = 8;
	eeinfo->dacchan = 2;
	
	eeinfo->adc_freq = 49000;
	eeinfo->adc_numranges = 8;
	eeinfo->adc_ranges[0] = 40000;
	eeinfo->adc_ranges[1] = 20000;
	eeinfo->adc_ranges[2] = 10000;
	eeinfo->adc_ranges[3] = 8000;
	eeinfo->adc_ranges[4] = 5000;
	eeinfo->adc_ranges[5] = 4000;
	eeinfo->adc_ranges[6] = 2500;
	eeinfo->adc_ranges[7] = 2000;
	
	eeinfo->adc_nummodes = 2;
	eeinfo->adc_modes[0] = BI_SINGLE;
	eeinfo->adc_modes[1] = BI_DIFF;

	eeinfo->adc_numtrig = 2;
	eeinfo->adc_trig[0] = AD_TRIG_INTERNAL;
	eeinfo->adc_trig[1] = AD_TRIG_EXTERNAL;
	
	eeinfo->adc_numtrigmode = 1;
	eeinfo->adc_trigmode[0] = AD_TRIG_POS;
	
	eeinfo->adc_numclk = 2;
	eeinfo->adc_clk[0] = AD_CLK_INTERNAL;
	eeinfo->adc_clk[1] = AD_CLK_EXTERNAL;
	
	eeinfo->dac_freq = 1;
	eeinfo->dac_speed = 1;
	eeinfo->dac_power = 0;

}

int udaqlite_get_deviceinfo( struct uDAQLiteDevice_t *dev )
{
	unsigned long val;
	if(udaqlite_query_device(dev, BRDSERIALNO, 0, &val) != 0)
		return -EIO;
	dev->eeinfo.serialnumber = val;
	if(udaqlite_query_device(dev, BRDTYPE, 0, &val) != 0)
		return -EIO;
	dev->eeinfo.boardtype = 0x2100;
	if(udaqlite_query_device(dev, BRDREV, 0, &val) != 0)
		return -EIO;
	dev->eeinfo.revision = val;
	if(udaqlite_query_device(dev, BRDDAY, 0, &val) != 0)
		return -EIO;
	dev->eeinfo.day = val;
	if(udaqlite_query_device(dev, BRDMONTH, 0, &val) != 0)
		return -EIO;
	dev->eeinfo.month = val;
	if(udaqlite_query_device(dev, BRDYEAR, 0, &val) != 0)
		return -EIO;
	dev->eeinfo.year = val;
	if(udaqlite_query_device(dev, FRMMAJOR, 0, &val) != 0)
		return -EIO;
	dev->eeinfo.firmmajor = val;
	if(udaqlite_query_device(dev, FRMMINOR, 0, &val) != 0)
		return -EIO;
	dev->eeinfo.firmminor = val;
	if(udaqlite_query_device(dev, FRMBUILD, 0, &val) != 0)
		return -EIO;
	dev->eeinfo.firmbuild = val;
	
	udaqlite_get_adcconfig(dev);
	udaqlite_get_dacconfig(dev);
	
	udaqlite_eeinfo_dump(&dev->eeinfo);

	return 0;
}

int udaqlite_init_device( struct uDAQLiteDevice_t *dev )
{
	/* initialize A/D sub-system */
	init_usb_anchor(&dev->adc.iso_submitted);
	spin_lock_init(&dev->adc.lock);

	/* allocate double buffer */
	dev->adc.dblbuffer.data = kmalloc(DOUBLE_BUFFER_SIZE, GFP_KERNEL);
	if(dev->adc.dblbuffer.data == NULL)
		dev->adc.dblbuffer.size = 0;
	else
		dev->adc.dblbuffer.size = DOUBLE_BUFFER_SIZE;

	/* allocate circular buffer */
	udaqlite_buf_init(&dev->adc.buffer, ADC_BUFFER_SIZE);

	/* initialize A/D status */
	dev->adc.status = 0;
	return 0;
}

void udaqlite_uninit_device( struct uDAQLiteDevice_t *dev )
{
	/* call A/D to stop by default */
	udaqlite_adc_stop(dev, 0);

	/* release buffers */
	udaqlite_buf_uninit(&dev->adc.buffer);
	if(dev->adc.dblbuffer.size > 0) 
		kfree(dev->adc.dblbuffer.data);
}

/**
 * End of File
 */
