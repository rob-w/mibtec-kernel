/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 * fxloader.c -- Source file to load firmware
 ******************************************************************************/
#include "stdinc.h" 
#include "udaqlite.h"
#include "fxloader.h"
#include "firmware.h"

#define CPUCS_RESET 0x01
#define CPUCS_RUN 0x00
#define CPUCS_REG 0x7f92

#define EZUSB_INTERNAL_RAM_LOAD 0xa0
#define EZUSB_EXTERNAL_RAM_LOAD 0xa3
#define VEN_REQUEST_OUT 0x40
#define VEN_REQUEST_IN 0xc0
#define MAX_INTERNAL_ADDRESS 0x2000
#define MAX_INTEL_RECORD_LENGTH 16

#define INTERNAL_RAM(address) ((address <= MAX_INTERNAL_ADDRESS) ? 1 : 0)

struct Intel_Hex_Record_t {
 unsigned char Length;
 unsigned short Address;
 unsigned char Type;
 unsigned char Data[MAX_INTEL_RECORD_LENGTH];
};

int fxldr_reset( struct usb_device *device )
{
	char cmd;
	cmd = CPUCS_RESET;
	if(usb_control_msg( device, usb_sndctrlpipe(device, 0),
									EZUSB_INTERNAL_RAM_LOAD, VEN_REQUEST_OUT,
									CPUCS_REG, 0, &cmd, 1, 300) != 1) {
		drv_err("usbupclite: Failed to put processor into reset.");
		return -1;
	}

	cmd = CPUCS_RUN;
	if(usb_control_msg( device, usb_sndctrlpipe(device, 0),
									EZUSB_INTERNAL_RAM_LOAD, VEN_REQUEST_OUT,
									CPUCS_REG, 0, &cmd, 1, 300) != 1) {
		drv_err("usbupclite: Failed to put processor into run mode.");
		return -1;
	}

	return 0;
}

int fxldr_run( struct usb_device *device )
{
	char cmd;
	cmd = CPUCS_RUN;
	if(usb_control_msg( device, usb_sndctrlpipe(device, 0),
									EZUSB_INTERNAL_RAM_LOAD, VEN_REQUEST_OUT,
									CPUCS_REG, 0, &cmd, 1, 300) != 1) {
		drv_err("usbupclite: Failed to put processor into run mode.");
		return -1;
	}

	return 0;
}

int fxldr_verify( struct usb_device *device, struct Intel_Hex_Record_t *record )
{
  unsigned char data[16];
  int i, status;
  status = usb_control_msg( device, usb_rcvctrlpipe(device, 0), 
									 EZUSB_INTERNAL_RAM_LOAD, 
									 VEN_REQUEST_IN, 
									 record->Address, 0, data, record->Length, 300);
  if(status != record->Length) {
	 drv_err("usbupclite: usb_control_msg verify error.");
	 return -1;
  }
  
  for(i=0; i<record->Length; i++) {
   if(data[i] != record->Data[i])
	 return -1;
  }
  return 0;

}

int fxldr_load_firmware( struct usb_device *ud )
{
	unsigned short r, i;
	const unsigned char *rec;
	const unsigned char *pch;
	struct Intel_Hex_Record_t ihr;
	char cmd;
	char str[6];
	int status;
	
	cmd = CPUCS_RESET;
	if(usb_control_msg( ud, usb_sndctrlpipe(ud, 0),
									EZUSB_INTERNAL_RAM_LOAD, VEN_REQUEST_OUT, 
									CPUCS_REG, 0, &cmd, 1, 300) != 1) {
	 drv_err("usbupclite: Failed to put processor into reset.");
	 return -1;
	}
		
	for(r=0; r < FW_NUM_RECORDS - 1; r++) {
		rec = firmware_source[r];
		
		strncpy(str, rec, 2);
		str[2] = '\n';
		ihr.Length = (unsigned char)simple_strtoul(str, (char**)0, 16);
		
		strncpy(str, &rec[2], 4);
		str[4] = '\n';
		ihr.Address = (unsigned short)simple_strtoul(str, (char**)0, 16);

		strncpy(str, &rec[6], 2);
		str[2] = '\n';
		ihr.Type = (unsigned char)simple_strtoul(str, (char**)0, 16);

		pch = &rec[8];
		
		for(i=0; i<ihr.Length; i++) {
			strncpy(str, pch, 2);
			str[2] = '\n';
			ihr.Data[i] = (unsigned char)simple_strtoul(str, (char**)0, 16);
			pch += 2;
		}
		
		status = usb_control_msg( ud, usb_sndctrlpipe(ud, 0), 
															EZUSB_INTERNAL_RAM_LOAD, 
															VEN_REQUEST_OUT, 
															ihr.Address, 0, ihr.Data, ihr.Length, 300);
		if(status != ihr.Length) {
		 drv_err("usbupclite: usb_control_msg error.");
		 return -1;
		}
		
		if(fxldr_verify( ud, &ihr)) {
		 drv_err("usbupclite: Failed to verify data.");
		 return -1;
		}
	}
	
	cmd = CPUCS_RUN;
	if(usb_control_msg( ud, usb_sndctrlpipe(ud, 0),
									EZUSB_INTERNAL_RAM_LOAD, VEN_REQUEST_OUT, CPUCS_REG, 0, &cmd, 1, 300) != 1) {
	 drv_err("usbupclite: Failed to start processor.");
	 return -1;
	}
		
	return 0;
 }
 
/**
 * End of File
 */
