/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 * ioctl.h -- I/O Control header file
 ******************************************************************************/
#ifndef _IOCTL_H_
#define _IOCTL_H_
int udaqlite_ioctl_query( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_dio_write( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_dio_read( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_cnt_write( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_cnt_read( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_cnt_config( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_cnt_softgate( struct uDAQLiteDevice_t *dev ,unsigned long arg );
int udaqlite_dac_write( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_adc_single( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_adc_config( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_adc_stop( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_adc_start( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_adc_clksrc( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_adc_freq( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_adc_chlst( struct uDAQLiteDevice_t *dev, unsigned long arg );
int udaqlite_adc_isbusy( pADC_t adc );
int udaqlite_adc_gobusy( pADC_t adc );
void udaqlite_adc_goidle( pADC_t adc );
unsigned short udaqlite_dac_volttobin( long value, unsigned char offset, 
																			unsigned char gain_error );
int udaqlite_adc_setfrequency( struct uDAQLiteDevice_t *dev, unsigned short value );
void udaqlite_adc_reset( pADC_t adc );
int udaqlite_adc_addchan( struct uDAQLiteDevice_t *dev, unsigned char channel,
			unsigned char gain, unsigned char range );
int udaqlite_adc_read( struct uDAQLiteDevice_t *dev, unsigned char channel,
		     unsigned char gain, unsigned char range, unsigned short *bin );
void udaqlite_adc_chanlstdump( pADC_t adc );
int udaqlite_adc_chanlstrst( struct uDAQLiteDevice_t *dev );
#endif /* _IOCTL_H_ */
/**
 * End of File
 */
