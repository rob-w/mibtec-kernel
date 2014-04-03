/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 * ioctl.c -- I/O Control Source File
 ******************************************************************************/
#include "stdinc.h"
#include "version.h"
#include "udaqlite.h"
#include "usbio.h"
#include "buffer.h"
#include "ioctl.h"
#include "fxloader.h"

int udaqlite_ioctl_query( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned long param[2];
	if(copy_from_user(param, (void *)arg, 2*sizeof(unsigned long)) != 0)
		return -EFAULT;

	switch(param[0]) {
	case BRDSERIALNO:
		param[0] = dev->eeinfo.serialnumber;
		break;
	case BRDREV:
		param[0] = dev->eeinfo.revision;
		break;
	case BRDTYPE:
		param[0] = 0x2100;
		break;
	case BRDYEAR:
		param[0] = dev->eeinfo.year;
		break;
	case BRDMONTH:
		param[0] = dev->eeinfo.month;
		break;
	case BRDDAY:
		param[0] = dev->eeinfo.day;
		break;
	case BRDBUSTYPE:
		param[0] = PC_BUS_USB;
		break;
	case DRVMAJOR:
		param[0] = (UDAQLITEDRV_VERSION>>16)&0xff;
		break;
	case DRVMINOR:
		param[0] = (UDAQLITEDRV_VERSION>>8)&0xff;
		break;
	case DRVBUILD:
		param[0] = UDAQLITEDRV_VERSION&0xff;
		break;
	case FRMMAJOR:
		param[0] = dev->eeinfo.firmmajor;
		break;
	case FRMMINOR:
		param[0] = dev->eeinfo.firmminor;
		break;
	case FRMBUILD:
		param[0] = dev->eeinfo.firmbuild;
		break;
	case ADNUMCHAN:
		param[0] = dev->eeinfo.adcchan;
		break;
	case ADMAXFREQ:
		param[0] = dev->eeinfo.adc_freq;
		break;
	case ADNUMTMP:
		param[0] = 0;
		break;
	case ADBUSY:
		param[0] = dev->adc.status;
		break;
	case ADUNREAD:
		param[0] = udaqlite_buf_getunread(&dev->adc.buffer);
		break;
	case ADBUFFSIZE:
		param[0] = dev->adc.buffer.size;
		break;
	case ADFIFOSIZE:
		param[0] = 512;
		break;
	case ADFIFOOVER:
	case ADBUFFOVER:
		param[0] = dev->adc.buffer.overrun;
		break;
	case ADBUFFALLOC:
		if(dev->adc.buffer.size > 0)
			param[0] = 1;
		else
			param[0] = 0;
		break;
	case ADNUMRANGES:
		param[0] = dev->eeinfo.adc_numranges;
		break;
	case ADRANGES:
		if(param[1] >= dev->eeinfo.adc_numranges)
			return -EINVAL;
		param[0] = dev->eeinfo.adc_ranges[param[1]];
		break;
	case ADNUMMODES:
		param[0] = dev->eeinfo.adc_nummodes;
		break;
	case ADMODES:
		if(param[1] >= dev->eeinfo.adc_nummodes)
			return -EINVAL;
		param[0] = dev->eeinfo.adc_modes[param[1]];
		break;
	case ADNUMTRIG:
		param[0] = dev->eeinfo.adc_numtrig;
		break;
	case ADTRIG:
		if(param[1] >= dev->eeinfo.adc_numtrig)
			return -EINVAL;
		param[0] = dev->eeinfo.adc_trig[param[1]];
		break;
	case ADNUMTRIGMODE:
		param[0] = dev->eeinfo.adc_numtrigmode;
		break;
	case ADTRIGMODE:
		if(param[1] >= dev->eeinfo.adc_numtrigmode)
			return -EINVAL;
		param[0] = dev->eeinfo.adc_trig[param[1]];
		break;
	case ADNUMCLK:
		param[0] = dev->eeinfo.adc_numclk;
		break;
	case ADCLK:
		if(param[1] >= dev->eeinfo.adc_numclk)
			return -EINVAL;
		param[0] = dev->eeinfo.adc_clk[param[1]];
		break;
	case ADSETDATAMODE:
		dev->adc.datamode = param[1] & 0x1l;
		if(param[1] > 1)
			return -EINVAL;
		break;
	case DANUMCHAN:
		param[0] = dev->eeinfo.dacchan;
		break;
	case CTNUM:
		param[0] = dev->eeinfo.cntchan;
		break;
	case DIONUMPORT:
		param[0] = dev->eeinfo.diochan;
		break;
	case DIOPORTWIDTH:
		param[0] = 8;
		break;
	case DIOQRYPORT:
		param[0] = DIOINANDOUT;
		break;
	case BRDRESET:
		fxldr_reset(dev->usbdev);
		param[0] = 0;
		break;
	default:
		param[0] = 0;
		param[1] = 0;
	}
	
	if(copy_to_user((void *)arg, param, sizeof(unsigned long)) != 0)
		return -EFAULT;
	return 0;
}

int udaqlite_dio_write( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned long param[2], port, value;
	unsigned char buf[64];

	if(copy_from_user(param, (void *)arg, 2*sizeof(unsigned long)) != 0)
		return -EFAULT;

	port = param[0];
	value = param[1];
	buf[0] = USB_DIOWRITE>>8;
	buf[1] = USB_DIOWRITE;
	buf[2] = 0;
	buf[3] = 0;
	buf[4] = value>>8;
	buf[5] = value;

	if(udaqlite_bulkpacket(dev, buf, 64))
		return -EFAULT;
	if(buf[1] != 0)
		return -EIO;
	return 0;
}

int udaqlite_dio_read( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned long param[1],port;
	unsigned char buf[64];

	if(copy_from_user(param, (void *)arg, sizeof(unsigned long)) != 0)
		return -EFAULT;

	port = param[0];
	buf[0] = USB_DIOREAD>>8;
	buf[1] = USB_DIOREAD;
	buf[2] = 0;
	buf[3] = 0;

	if(udaqlite_bulkpacket(dev, buf, 64))
		return -EFAULT;
	if(buf[1] != 0)
		return -EIO;

	param[0] = buf[2]<<8 | buf[3];
	if(copy_to_user((void *)arg, param, sizeof(unsigned long)) != 0)
		return -EFAULT;

	return 0;
}

int udaqlite_cnt_write( struct uDAQLiteDevice_t *dev,unsigned long arg )
{
	unsigned long param[2], port, value;
	unsigned char buf[64];

	if(copy_from_user(param, (void *)arg, 2*sizeof(unsigned long)) != 0)
		return -EFAULT;

	port = param[0];
	value = param[1];

	if(port >= dev->eeinfo.cntchan)
		return -EINVAL;

	buf[0] = USB_CTWRITE>>8;
	buf[1] = USB_CTWRITE;
	buf[2] = port>>8;
	buf[3] = port;
	buf[4] = value>>8;
	buf[5] = value;

	if(udaqlite_bulkpacket(dev, buf, 64))
		return -EFAULT;
	if(buf[1] != 0)
		return -EIO;
	return 0;
}

int udaqlite_cnt_read( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned long param[1], port;
	unsigned char buf[64];

	if(copy_from_user(param, (void *)arg, sizeof(unsigned long)) != 0)
		return -EFAULT;

	port = param[0];
	if(port >= dev->eeinfo.cntchan)
		return -EINVAL;

	buf[0] = USB_CTREAD>>8;
	buf[1] = USB_CTREAD;
	buf[2] = port>>8;
	buf[3] = port;

	if(udaqlite_bulkpacket(dev, buf, 64))
		return -EFAULT;
	if(buf[1] != 0)
		return -EIO;

	param[0] = buf[4]<<8 | buf[5];
	if(copy_to_user((void *)arg, param, sizeof(unsigned long)) != 0)
		return -EFAULT;

	return 0;
}

int udaqlite_cnt_config( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned long param[5], port, mode, bcd, clksrc, gatesrc;
	unsigned char buf[64];

	if(copy_from_user(param, (void *)arg, 5*sizeof(unsigned long)) != 0)
		return -EFAULT;

	port = param[0];
	mode = param[1];
	bcd = param[2];
	clksrc = param[3];
	gatesrc = param[4];

	if(port >= dev->eeinfo.cntchan)
		return -EINVAL;

	buf[0] = USB_CTCONFIG>>8;
	buf[1] = USB_CTCONFIG;
	buf[2] = 0;
	buf[3] = clksrc;
	buf[4] = 0;
	buf[5] = gatesrc;
	buf[6] = 0;
	buf[7] = 0;

	if(udaqlite_bulkpacket(dev, buf, 64))
		return -EFAULT;
	if(buf[1] != 0)
		return -EIO;
	return 0;
}

int udaqlite_cnt_softgate( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned long param[2], port, mode;
	unsigned char buf[64];

	if(copy_from_user(param, (void *)arg, 2*sizeof(unsigned long)) != 0)
		return -EFAULT;

	port = param[0];
	mode = param[1];
	if(port>= dev->eeinfo.cntchan)
		return -EFAULT;

	buf[0] = USB_CTSOFTGATE>>8;
	buf[1] = USB_CTSOFTGATE;
	buf[2] = port>>8;
	buf[3] = port;
	buf[4] = mode>>8;
	buf[5] = mode;

	if(udaqlite_bulkpacket(dev, buf, 64))
		return -EFAULT;
	if(buf[1] != 0)
		return -EIO;
	return 0;
}

int udaqlite_dac_write( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned long param[2], port;
	long value;
	unsigned short bin;
	unsigned char buf[BULKDATAMAX];
	
	if(copy_from_user(param, (void *)arg, 2*sizeof(unsigned long)) != 0)
		return -EFAULT;

	port = param[0];
	value = (long)param[1];
	
	if(port >= dev->eeinfo.dacchan)
		return -EINVAL;

	if((value > 10000000) || (value < -10000000))
		return -EINVAL;
		
	bin = udaqlite_dac_volttobin( value, dev->eeinfo.dac_offset[port],
					dev->eeinfo.dac_gain_error[port]);
	if(port == 0)
		port = 0x8000 | (dev->eeinfo.dac_speed & 0x01) << 14 | (dev->eeinfo.dac_power & 0x01) << 13;
	else
		port = 0x0000 | (dev->eeinfo.dac_speed & 0x01) << 14 | (dev->eeinfo.dac_power & 0x01) << 13;
		
	bin |= port;
	
	buf[0] = USB_DACWRITE>>8;
	buf[1] = USB_DACWRITE;
	buf[2] = bin>>8;
	buf[3] = bin;
		
	if(udaqlite_bulkpacket(dev, buf, 64))
		return -EFAULT;
	if(buf[1] != 0)
		return -EIO;
		
	return 0;
}
 
int udaqlite_adc_single( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	struct ADC_t *adc = &dev->adc;
	unsigned long param[2], gain, chan;
	unsigned char offset;
	unsigned short bin;
	long uv;
	int retval = 0;

	if(test_bit(1, &adc->status)) {
		err("MicroDAQLite A/D sub-system is busy: #%ld", dev->eeinfo.serialnumber);
		return -EBUSY;
	}

	spin_lock(&adc->lock);

	if(copy_from_user(param, (void *)arg, 2*sizeof(unsigned long))) {
		retval = -EFAULT;
		goto udaqlite_adc_single_done;
	}

	chan = param[0];
	gain = param[1];
	if(chan >= dev->eeinfo.adcchan || gain > 9 || dev->adc.range > 2) {
		retval = -EINVAL;
		goto udaqlite_adc_single_done;
	}

	if(dev->adc.range == 0) {
		offset = dev->adc.offset[chan];
		chan |= 0x08;
		gain = 0;
	}
	else {
		offset = dev->adc.offset[chan + 8];
	}
		
	retval = udaqlite_adc_read(dev, chan, gain, dev->adc.range, &bin);
	
	if(retval == -25 || retval == -35) {
		retval = -EIO;
		goto udaqlite_adc_single_done;
	}

	uv = udaqlite_adc_bintovolt(bin, chan, gain, offset, dev->adc.gain_error);
	
	if(copy_to_user((void *)arg, &uv, sizeof(long)) != 0 )
		retval = -EFAULT;

udaqlite_adc_single_done:
	spin_unlock(&adc->lock);
	return retval;
}

int udaqlite_adc_config( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned long param[2];
	int retval = 0;
		
	if(test_bit(1, &dev->adc.status) != 0) {
		err("MicroDAQLite A/D sub-system is busy: #%ld", dev->eeinfo.serialnumber);
		return -EBUSY;
	}
	
	spin_lock(&dev->adc.lock);

	if(copy_from_user(param, (void *)arg, 2*sizeof(unsigned long)) != 0) {
		goto udaqlite_adc_config_done;
		return -EFAULT;
	}

	if(param[0] > 1)
		param[0] = 1;

	dev->adc.range = param[0];

udaqlite_adc_config_done:
	spin_unlock(&dev->adc.lock);
	return retval;
}

int udaqlite_adc_stop( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned char buf[BULKDATAMAX];


	buf[0] = USB_ADCSTOP>>8;
	buf[1] = USB_ADCSTOP;

	spin_lock(&dev->adc.lock);
	
	/* is busy? */
	if(test_bit(1, &dev->adc.status)) {
		/* send stop command to USB firmware */
		udaqlite_bulkpacket(dev, buf, 64);
		/* tell USB host to unqueue and release our URBs */
		/* the USB sub-system will release all allocated buffers */
		usb_unlink_anchored_urbs(&dev->adc.iso_submitted);
		udaqlite_isourb_waitforurbs(dev);
		/* clear status bit */
		clear_bit(1, &dev->adc.status);
	}

	spin_unlock(&dev->adc.lock);

	return 0;
}

int udaqlite_adc_start( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned char buf[BULKDATAMAX];
	int retval = 0;
	int i;

	if(test_bit(1, &dev->adc.status)) {
		return -EBUSY;
	}

	spin_lock(&dev->adc.lock);

	/* start queuing ISO URBs to be able to receive the data from the device */
	for(i=0; i<ISO_NUM_URB; i++) {
		if(udaqlite_isourb_alloc(dev)) {
			retval = -EFAULT;
			usb_unlink_anchored_urbs(&dev->adc.iso_submitted);
			err("%s - failed allocating streaming ISO URBs", __func__);
			goto udaqlite_adc_start_exit;
		}
	}

	/* tell device to start sending us data */
	buf[0] = USB_ADCSTART>>8;
	buf[1] = USB_ADCSTART;
	if(udaqlite_bulkpacket(dev, buf, 64)) {
		retval = -EIO;
		usb_unlink_anchored_urbs(&dev->adc.iso_submitted);
		goto udaqlite_adc_start_exit;
	}
	
	/* let's go busy */
	set_bit(1, &dev->adc.status);

udaqlite_adc_start_exit:
	spin_unlock(&dev->adc.lock);
	return retval;
}

int udaqlite_adc_clksrc( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned char buf[BULKDATAMAX];
	unsigned long param[1], clksrc, trigsrc;
	int retval = 0;

	if(test_bit(1, &dev->adc.status)) {
		return -EBUSY;
	}

	spin_lock(&dev->adc.lock);

	if(copy_from_user(param, (void *)arg, sizeof(unsigned long)) != 0) {
		retval = -EFAULT;
		goto udaqlite_adc_clksrc_exit;
	}

	clksrc = param[0] & 0xff;
	trigsrc = (param[0]>>8) & 0xff;
	if(clksrc > 1) clksrc = 1;
	if(trigsrc>1) trigsrc = 1;

	buf[0] = USB_ADCLKSRC>>8;
	buf[1] = USB_ADCLKSRC;
	buf[2] = clksrc>>8;
	buf[3] = clksrc;
	buf[4] = trigsrc>>8;
	buf[5] = trigsrc;
	if(udaqlite_bulkpacket(dev, buf, 64)) {
		retval = -EIO;
		goto udaqlite_adc_clksrc_exit;
	}

	if(buf[1] != 0)
		retval = -EIO;	

udaqlite_adc_clksrc_exit:
	spin_unlock(&dev->adc.lock);
	return retval;
}

int udaqlite_adc_freq( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned long param[1], freq, val, div;
	int retval = 0;

	if(test_bit(1, &dev->adc.status)) {
		return -EBUSY;
	}

	spin_lock(&dev->adc.lock);

	if(copy_from_user(param, (void *)arg, sizeof(unsigned long)) != 0) {
		goto udaqlite_adc_freq_exit;
		retval = -EIO;
	}

	freq = param[0];
	if(freq > 49000 || freq < 100) {
		retval = -EINVAL;
		goto udaqlite_adc_freq_exit;
	}

	if(freq > 35000)
		freq += 1000;
		
	div = 6000000 / freq;
	val = 65536 - div;
	
	if(udaqlite_adc_setfrequency(dev, val)) {
		retval = -EIO;
		goto udaqlite_adc_freq_exit;
	}
		
	freq = 6000000 / div;
	if(freq > 35000)
		freq -= 1000;

	dev->adc.frequency = freq;
	if(copy_to_user((void *)arg, &freq, sizeof(unsigned long)) != 0)
		retval = -EIO;

udaqlite_adc_freq_exit:
	spin_unlock(&dev->adc.lock);
	return retval;
}

int udaqlite_adc_chlst( struct uDAQLiteDevice_t *dev, unsigned long arg )
{
	unsigned long param[33], ch;
	unsigned char chan, gain, range, chlstsz;
	int retval = 0;
	
	if(test_bit(1, &dev->adc.status)) {
		return -EBUSY;
	}

	spin_lock(&dev->adc.lock);
		
	if(copy_from_user(param, (void *)arg, sizeof(unsigned long)) != 0) {
		retval = -EFAULT;
		goto udaqlite_adc_chlst_exit;
	}
	
	chlstsz = param[0];
	if(chlstsz > 8 /*dev->eeinfo.adcchan*/) {
		retval = -EINVAL;
		goto udaqlite_adc_chlst_exit;
	}

	if(copy_from_user(param, (void*)arg, (chlstsz * 2 + 1) * sizeof(unsigned long)) != 0) {
		retval = -EIO;
		goto udaqlite_adc_chlst_exit;
	}

	udaqlite_adc_reset(&dev->adc);
	udaqlite_buf_reset(&dev->adc.buffer);
	
	if(udaqlite_adc_chanlstrst(dev)) {
		retval = -EIO;
		goto udaqlite_adc_chlst_exit;

	}
		
	for(ch=0; ch<chlstsz; ch++) {
	
		chan = (unsigned char)param[ch * 2 + 1];
		gain = (unsigned char)(param[ch * 2 + 2] & 0x0f);
		range = (unsigned char)((param[ch * 2 + 2] >> 8) & 0x0f);
		
		dev->adc.chanlst[ch] = chan;
		dev->adc.gainlst[ch] = gain;
		dev->adc.rangelst[ch] = range;
		
		if(range == 0)
			gain = 0;
		if(gain > 7)
			gain = 7;
		
		if(udaqlite_adc_addchan(dev, chan, gain, range)) {
			retval = -EIO;
			goto udaqlite_adc_chlst_exit;
		}
	}
	
	dev->adc.chanlstsz = chlstsz;
	udaqlite_adc_chanlstdump(&dev->adc);

udaqlite_adc_chlst_exit:
	spin_unlock(&dev->adc.lock);
	return retval;
}

/* private helper functions */
int udaqlite_adc_chanlstrst( struct uDAQLiteDevice_t *dev )
{
	unsigned char buf[BULKDATAMAX];
	buf[0] = USB_ADCCHLSRST>>8;
	buf[1] = USB_ADCCHLSRST;
	if(udaqlite_bulkpacket(dev, buf, 64))
		return -EIO;
	if(buf[1] != EDRE_OK)
		return -EIO;
	return 0;
}

unsigned short udaqlite_dac_volttobin( long value, unsigned char offset, 
				unsigned char gain_error )
{
	unsigned short bin;	
	value = (value / (900 + gain_error)) * 1000;
	bin = (value + 10000000) / 4884;
	bin += (offset - 30);	
	return bin;
}

int udaqlite_adc_setfrequency( struct uDAQLiteDevice_t *dev, unsigned short value )
{
	unsigned char buf[BULKDATAMAX];
	unsigned short val;
	val = value - 1;
	buf[0] = USB_ADCFREQ>>8;
	buf[1] = USB_ADCFREQ;
	buf[2] = 0;
	buf[3] = 0;
	buf[4] = val>>8;
	buf[5] = val;
	if(udaqlite_bulkpacket(dev, buf, 64))
		return -EIO;
	if(buf[1])
		return -EIO;
	return 0;
}

void udaqlite_adc_reset(pADC_t adc)
{
	adc->chanlstsz = 0;
	adc->chanidx = 0;
}

int udaqlite_adc_addchan( struct uDAQLiteDevice_t *dev, unsigned char channel,
			unsigned char gain, unsigned char range )
{
#define RANGE_SINGLE_ENDED 0
	unsigned char buf[BULKDATAMAX];
	
	if(range == RANGE_SINGLE_ENDED) {
		channel |= 0x08;
	}

	buf[0] = USB_ADCADDCH>>8;
	buf[1] = USB_ADCADDCH;
	buf[2] = channel;
	buf[3] = gain << 4;
	if(udaqlite_bulkpacket(dev, buf, 64))
		return -EIO;
	if(buf[1] != EDRE_OK)
		return -EFAULT;
	return 0;
}

int udaqlite_adc_read( struct uDAQLiteDevice_t *dev, unsigned char channel,
		     unsigned char gain, unsigned char range, unsigned short *bin)
{
	unsigned char buf[BULKDATAMAX];
	buf[0] = USB_ADCREAD>>8;
	buf[1] = USB_ADCREAD;
	buf[2] = channel;
	buf[3] = gain << 4;
	if(udaqlite_bulkpacket(dev, buf, 64) )
		return -EIO;
	if(buf[1]) {
		err("A/D Error returned from firmware.");
		return -EIO;
	}
	*bin = (buf[5]<<8) | buf[4];
	return 0;
}

void udaqlite_adc_chanlstdump(pADC_t adc)
{
	int i;
	dbg("udaqlite_adc_chanlstdump: start(%d)", adc->chanlstsz);
	for(i=0; i<adc->chanlstsz; i++) {
		dbg( "udaqlite_adc_chanlstdump: chan: %d, gain: %d, range: %d",
			adc->chanlst[i], adc->gainlst[i], adc->rangelst[i]);
	}
	dbg("udaqlite_adc_chanlstdump: end");
}

/**
 * End of File
 */
