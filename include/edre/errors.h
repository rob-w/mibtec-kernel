/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2003 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * errors.h -- EDR Enhanced Errors
 *
 *******************************************************************************
 * 2005-02-24  Jan Zwiegers  <jan@eage.co.za>
 * - Add doxygen style comments
 *******************************************************************************/

 /*! \file errors.h
    \brief EDR Enhanced Errors File.

    This defines all the EDR Enhanced error codes.
*/

#ifndef _EDRE_ERRORS_H_
#define _EDRE_ERRORS_H_

/** @defgroup APIERRORCODES EDR Enhanced Error Codes
 *  @{
 */
/*!\def EDRE_OK No error. */
#define EDRE_OK 0
/*!\def EDRE_FAIL General failure. */
#define EDRE_FAIL -1
/*!\def EDRE_BAD_FN Invalid function call. Maybe not supported by the spcific OS. */
#define EDRE_BAD_FN -2
/*!\def EDRE_BAD_SN Invalid device and serial number. */
#define EDRE_BAD_SN -3
/*!\def EDRE_BAD_DEVICE Device does not exist. */
#define EDRE_BAD_DEVICE -4
/*!\def EDRE_BAD_OS Operating system is not supported. */
#define EDRE_BAD_OS -5
/*!\def EDRE_EVENT_FAILED Wait on event failed. */
#define EDRE_EVENT_FAILED -6
/*!\def EDRE_EVENT_TIMEOUT Wait on event timed out. */
#define EDRE_EVENT_TIMEOUT -7
/*!\def EDRE_INT_SET Already waiting for an interrupt. */
#define EDRE_INT_SET -8
/*!\def EDRE_DA_BAD_RANGE Invalid D/A range. */
#define EDRE_DA_BAD_RANGE -9
/*!\def EDRE_AD_BAD_CHANLIST Channel list contain an invalid channel or gain value or is to long. */
#define EDRE_AD_BAD_CHANLIST -10
/*!\def EDRE_BAD_FREQUENCY Frequency value is to high or to low. */
#define EDRE_BAD_FREQUENCY -11
/*!\def EDRE_BAD_BUFFER_SIZE Buffers size is of incorrect length. */
#define EDRE_BAD_BUFFER_SIZE -12
/*!\def EDRE_BAD_PORT Invalid port reference. */
#define EDRE_BAD_PORT -13
/*!\def EDRE_BAD_PARAMETER Error in parameter list. */
#define EDRE_BAD_PARAMETER -14
/*!\def EDRE_BUSY Device or subs-system is busy. */
#define EDRE_BUSY -15
/*!\def EDRE_IO_FAIL Hardware I/O failure. */
#define EDRE_IO_FAIL -16
/*!\def EDRE_BAD_ADGAIN A/D gain value invalid. */
#define EDRE_BAD_ADGAIN -17
/*!\def EDRE_BAD_QUERY Query code not supported. */
#define EDRE_BAD_QUERY -18
/*!\def EDRE_BAD_CHAN Invalid channel. */
#define EDRE_BAD_CHAN -19
/*!\def EDRE_BAD_VALUE Invalid parameter value. */
#define EDRE_BAD_VALUE -20
/*!\def EDRE_BAD_CT Invalid counter-timer channel. */
#define EDRE_BAD_CT -21
/*!\def EDRE_BAD_CHANLIST Invalid channel list. */
#define EDRE_BAD_CHANLIST -22
/*!\def EDRE_BAD_CONFIG Invalid configuration. */
#define EDRE_BAD_CONFIG -23
/*!\def EDRE_BAD_MODE Invalid operating mode. */
#define EDRE_BAD_MODE -24
/*!\def EDRE_HW_ERROR General hardware error. */
#define EDRE_HW_ERROR -25
/*!\def EDRE_HW_BUSY Hardware is busy. */
#define EDRE_HW_BUSY -26
/*!\def EDRE_BAD_BUFFER Buffer is invalid. */
#define EDRE_BAD_BUFFER -27
/*!\def EDRE_REG_ERROR Windows registery error. */
#define EDRE_REG_ERROR -28
/*!\def EDRE_OUT_RES Ran out of resources. */
#define EDRE_OUT_RES -29
/*!\def EDRE_IO_PENDING I/O call is pending. */
#define EDRE_IO_PENDING -30
/*!\def EDRE_NET_ERROR A network sub-system error occured. */
#define EDRE_NET_ERROR -31
/*!\def EDRE_FILE_ERROR A file error occured. */
#define EDRE_FILE_ERROR -32
/*!\def EDRE_FILE_OPEN File already open. */
#define EDRE_FILE_OPEN -33
/*!\def EDRE_PACKET_ERR A communications packet error occured . */
#define EDRE_PACKET_ERR -34
/*!\def EDRE_FIFO_ERR A FIFO error occured. */
#define EDRE_FIFO_ERR -35
/*!\def EDRE_NumErrorCodes Number of error codes. */
#define EDRE_NumErrorCodes 36
/** @} */
#endif /*_EDRE_ERRORS_H_*/
