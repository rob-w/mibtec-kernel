/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 * deviceio.h -- Device I/O header file
 ******************************************************************************/
#ifndef _DEVICEIO_H_
#define _DEVICEIO_H_
int udaqlite_query_device( struct uDAQLiteDevice_t *dev, long code, long param,
			 unsigned long *value);
int udaqlite_get_deviceinfo( struct uDAQLiteDevice_t *devv );
int udaqlite_init_device( struct uDAQLiteDevice_t *dev );
int udaqlite_eeprom_read( struct uDAQLiteDevice_t *dev, unsigned char offset );
void udaqlite_eeinfo_dump( pEepromInfo_t ee );
void udaqlite_set_devicetype( struct uDAQLiteDevice_t *dev );
int udaqlite_init_device( struct uDAQLiteDevice_t *dev );
void udaqlite_uninit_device( struct uDAQLiteDevice_t *devv );
#endif /*_DEVICEIO_H_*/
/**
 * End of File
 */
