/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 * buffer.h -- Buffer manager header file
 ******************************************************************************/
#ifndef _BUFFER_H_
#define _BUFFER_H_
int udaqlite_buf_init( pADCBuffer_t buf, unsigned long length );
void udaqlite_buf_uninit( pADCBuffer_t buf );
void udaqlite_buf_reset( pADCBuffer_t buf );
int udaqlite_buf_adddata( pADCBuffer_t buf, unsigned char *buffer, unsigned short bytes );
unsigned long udaqlite_buf_getdata( pADC_t adc, long *buffer,
				  unsigned long samples );
unsigned long udaqlite_buf_getunread( pADCBuffer_t buf );
long udaqlite_adc_bintovolt( unsigned short bin, unsigned char channel, 
													unsigned char gain, unsigned char offset, 
													unsigned char gain_error );
#endif /*_BUFFER_H_*/
/**
 * End of File
 */
