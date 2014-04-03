/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 * buffer.c -- Buffer manager source file
 ******************************************************************************/
#include "stdinc.h"
#include <linux/vmalloc.h>
#include <asm/div64.h>
#include "udaqlite.h"
#include "buffer.h"

int udaqlite_buf_init( pADCBuffer_t buf, unsigned long length )
{
	spin_lock_init(&buf->lock);
	buf->data = (unsigned short *)vmalloc(length * sizeof(unsigned short));
	if(buf->data == NULL)
		buf->size = 0;
	else
		buf->size = length;
	udaqlite_buf_reset(buf);
	return 0;
}

void udaqlite_buf_uninit( pADCBuffer_t buf )
{
	if(buf->size > 0)
		vfree(buf->data);
}

void udaqlite_buf_reset( pADCBuffer_t buf )
{
	unsigned long flags;
	spin_lock_irqsave(&buf->lock, flags);
	buf->samples = 0;
	buf->head = 0;
	buf->tail = 0;
	buf->wrap = 0;
	buf->overrun = 0;
	spin_unlock_irqrestore(&buf->lock, flags);
}

int udaqlite_buf_adddata( pADCBuffer_t buf, unsigned char *buffer, unsigned short bytes )
{
	unsigned short *data = buf->data;
	int eerr = EDRE_OK;
	unsigned long samples, i;
	unsigned long head, size, unread;
	unsigned long flags;
	
	samples = bytes / 2;
	spin_lock_irqsave(&buf->lock, flags);
	head = buf->head;
	size = buf->size;
	unread = buf->samples;
	spin_unlock_irqrestore(&buf->lock, flags);
	
	if(samples > (size - unread)) {
		samples = size - unread;
		eerr = EDRE_FAIL;
	}

	for(i=0; i<samples; i++) {
		data[head++] = (unsigned short)((buffer[i * 2 + 1] << 8) | buffer[i * 2]);
		if(head == size) {
			head = 0;
		}
	}

	spin_lock_irqsave(&buf->lock, flags);
	buf->head = head;
	buf->samples += samples;
	if(eerr)
		buf->overrun = 1;
	spin_unlock_irqrestore(&buf->lock, flags);
	return eerr;
}

unsigned long udaqlite_buf_getdata( pADC_t adc, long *buffer,
				  unsigned long samples )
{
	pADCBuffer_t buf = &adc->buffer;
	unsigned short *data = buf->data;
	unsigned long i, tail, size;
	int range, gain, chan;
	unsigned char offset;
	unsigned long flags;

	spin_lock_irqsave(&buf->lock, flags);

	if(samples > buf->samples)
		samples = buf->samples;
	tail = buf->tail;
	size = buf->size;
	spin_unlock_irqrestore(&buf->lock, flags);

	for(i=0; i<samples; i++) {
		range = adc->rangelst[adc->chanidx];
		gain = adc->gainlst[adc->chanidx];
		chan = adc->chanlst[adc->chanidx];
		offset = adc->offset[chan + range * 8];
		
		buffer[i] = udaqlite_adc_bintovolt(data[tail], chan, gain, offset, adc->gain_error);
		
		if(++adc->chanidx == adc->chanlstsz)
			adc->chanidx = 0;
		if(++tail == size)
			tail = 0;
	}

	spin_lock_irqsave(&buf->lock, flags);
	buf->samples -= samples;
	spin_unlock_irqrestore(&buf->lock, flags);
	
	return samples;
}

unsigned long udaqlite_buf_getunread( pADCBuffer_t buf )
{
	unsigned long samples;
	unsigned long flags;
	spin_lock_irqsave(&buf->lock, flags);
	samples = buf->samples;
	spin_unlock_irqrestore(&buf->lock, flags);
	return samples;
}

long udaqlite_adc_bintovolt( unsigned short bin, unsigned char channel, 
				unsigned char gain, unsigned char offset, 
				unsigned char gain_error )
{
	unsigned char GAIN[8] = {1, 2, 4, 5, 8, 10, 16, 20};
	long uv, sign;
	unsigned short bin2, i;
	unsigned long long uuv, duv;
	
	
	/* swop bits */
	bin <<= 3;
	bin2 = 0;
	
	for(i=0; i<16; i++) {
		bin2 |= (((bin >> (15 - i)) & 0x01) << i);
	}
	
	bin2 &= 0x0fff;
	sign = 1;
	
	if(channel > 0x07) {
		uv = (bin2 * 9765) - 9999360;
	}
	else {
	
		if(bin2 > 0x07ff) {
			bin2 = 0x1000 - bin2;
			uv = (bin2 * 9765) / GAIN[gain];
			uv = -uv;
		}
		else {
			bin2 &= 0x07ff;
			uv = (bin2 * 9765) / GAIN[gain];
		}
		
	}
	
	uv += (long)( ((80 - offset) * 9765) / GAIN[7]);
	if(uv < 0) {
		sign = -1;
		uv = -uv;
	}
	
	uuv = uv;
	uuv = (uuv * 1000);
	duv = (900 + gain_error);
	uv = do_div(uuv, duv);
	uv = (long)uuv;
	uv *= sign;
	return uv;
}


/**
 * End of File
 */
