/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 * udaqlite.h -- uDAQLite main header file
 ******************************************************************************/
#ifndef _USBUPC_H__
#define _USBUPC_H__

/*General Constans*/
#define BULKDATAMAX 64                  /**/
#define PACKET_DIR_OUT 1
#define PACKET_DIR_IN 0
#define URB_TIMEOUT 100
#define STOPPED 0
#define STARTED 1
#define MAX_UDAQLITE_DEVICES 16
#define EMPTY_ENDPOINT 0xffff
#define ISO_NUM_URB 4
#define ISO_PIPESIZE 5120
#define DOUBLE_BUFFER_SIZE 1024
#define ADC_BUFFER_SIZE 131072

/*macros*/
#define TYPE(dev)   (MINOR(dev) >> 4)  /* high nibble */
#define NUM(dev)    (MINOR(dev) & 0xf) /* low  nibble */

#include <asm/atomic.h>

struct EepromInfo_t {
	unsigned long serialnumber;
	unsigned short month, day, year;
	unsigned short revision;
	unsigned short boardtype;
	unsigned short firmmajor, firmminor, firmbuild;
	unsigned char diochan, adcchan, dacchan, cntchan;
	unsigned long adc_freq;
	unsigned long dac_freq;
	unsigned char dac_speed;
	unsigned char dac_power;
	unsigned char dac_offset[2];
	unsigned char dac_gain_error[2];
	unsigned short adc_numranges;
	unsigned short adc_ranges[20];
	unsigned short adc_nummodes;
	unsigned short adc_modes[20];
	unsigned short adc_numtrig;
	unsigned short adc_trig[20];
	unsigned short adc_numtrigmode;
	unsigned short adc_trigmode[10];
	unsigned short adc_numclk;
	unsigned short adc_clk[10];
};
#define pEepromInfo_t struct EepromInfo_t*

struct ADCBuffer_t {
	unsigned short *data;
	unsigned long size;
	unsigned long samples;
	unsigned long head;
	unsigned long tail;
	unsigned long wrap;
	unsigned char overrun;
	spinlock_t lock;
};
#define pADCBuffer_t struct ADCBuffer_t*

struct DblBuffer_t {
	void *data;
	unsigned short size;
};
#define pDblBuffer_t struct DblBuffer_t*

struct ADC_t {
	spinlock_t lock;
	unsigned long status;
	unsigned int isourb_ep;
	int isourb_ep_size;
	int isourb_ep_interval;
	struct usb_anchor iso_submitted;		/* all our submitted ISO URBs */
	unsigned char range;
	unsigned char chanlst[8];
	unsigned char gainlst[8];
	unsigned char rangelst[8];
	unsigned char chanlstsz;
	unsigned long frequency;
	unsigned long minfreq;
	unsigned long oversample;
	unsigned char chanidx;
	unsigned char datamode;
	unsigned char offset[12];
	unsigned char gain_error;
	struct ADCBuffer_t buffer;
	struct DblBuffer_t dblbuffer;
};
#define pADC_t struct ADC_t*

struct uDAQLiteDevice_t {
	struct usb_device	*usbdev;		/* the usb device for this device */
	struct usb_interface	*usbif;			/* the interface for this device */
	struct semaphore	io_sem;			/* used for I/O blocking */
	int state;
	int got_mem;
	int total_buffer_size;
	unsigned int ctrl_ep;
	int readptr;
	struct ADC_t adc;
	struct EepromInfo_t eeinfo;
	int			open_count;		/* count the number of openers */
	spinlock_t		err_lock;		/* lock for errors */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */

};

#endif /* _USBUPC_H_ */

/**
 * End of File
 */
