/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2006 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * edrapiobj.h -- EDR Enhanced Public Object Definition File
 *
 *******************************************************************************
 * 2006-01-01  Jan Zwiegers  <jan@eage.co.za>
 * - Created
 *******************************************************************************/

#ifndef _EDRE_OBJECT_PUBLIC_H_
#define _EDRE_OBJECT_PUBLIC_H_
/*! \file edrapiobj.h
    \brief EDR Enhanced Object and Device Object definition file

    This file defines the EDR Enhanced main object and EDR Enhanced device object. Currently only supported by Linux.
*/

/** @defgroup EDREOBJECT EDR Enhanced Main Object
 *  @{
 */

/*!
  \brief EDR Enhanced Main Public Object

  The EDREObject is the main controller object. It has access to all device objects. A serial number needs to be supplied for each function to tell the object controller which device to talk too (Linux Only).
*/
class EDREObject
{
public:
/*! \fn long Query(unsigned long Sn, unsigned long QueryCode, unsigned long Param)
    \brief Query command
    \param Sn The device serial number to query if relevant.
    \param QueryCode The query code that must be executed.
  \param Param An extra query parameter.
  \return Result of the query code.

  Use this command to query a device property or API property.

  \sa query.h
*/
  long Query(unsigned long Sn, unsigned long QueryCode, unsigned long Param);

/*! \fn long DioWrite(unsigned long Sn, unsigned long Port, unsigned long Value)
    \brief Digital I/O write command
    \param Sn The device serial number to execute the digital I/O write.
    \param Port The port to write too.
  \param Value The value that must be written to the port.
  \return Success if 0, else error code.

  Use this command command to output a digital value.
*/
  long DioWrite(unsigned long Sn, unsigned long Port, unsigned long Value);

/*! \fn long DioRead(unsigned long Sn, unsigned long Port, unsigned long *Value)
    \brief Digital I/O read command
    \param Sn The device serial number to execute the digital I/O read.
    \param Port The port to read from.
  \param *Value A pointer to an 32-bit unsigned value.
  \return Success if 0, else error code

  Use this command command to output a digital value.
*/
  long DioRead(unsigned long Sn, unsigned long Port, unsigned long *Value);

/*! \fn long MioConfig(unsigned long Sn, unsigned long Port, unsigned long Value)
    \brief Digital I/O configure command - PCI14B only
    \param Sn The serial number of the device to configure.
    \param Port The port to configure.
  \param Value Configuration value.
  \return Success if 0, else error code.

  Use this command to configure the multi-I/O port on the PCI14B device.
*/
  long MioConfig(unsigned long Sn, unsigned long Port, unsigned long Value);

/*! \fn long WaitOnInterrupt(unsigned long Sn)
    \brief Wait for an interrupt event
    \param Sn The device serial number to generate the interrupt
  \return Interrupt source. Each bit that is set represents a source.

  This command is used to wait for an interrupt to occur. Make sure to use a
  separate thread to wait, otherwise the user application will simply hang.
*/
  long WaitOnInterrupt(unsigned long Sn);

/*! \fn long ReleaseInterrupt(unsigned long Sn)
    \brief Release WaitOnInterrupt prematurely.
    \param Sn The device that is currently waiting for an interrupt.
  \return Success if 0, else error code.

  This command is used to wait for an interrupt to occur. Make sure to use a
  separate thread to wait, otherwise the user application will simply hang.
*/
  long ReleaseInterrupt(unsigned long Sn);

/*! \fn long IntConfigure(unsigned long Sn, unsigned long Src, unsigned long Mode, unsigned long Type)
    \brief Configure interrupt source
    \param Sn The serial number of the device to configure.
  \param Src The interrupt source number.
  \param Mode The interrupt mode. Refer to device manual.
  \param Type The interrupt type. Refer to device manual.
  \return Success if 0, else error code.

  This command is used to configure an interrupt source. It can be used to disable or
  enable a specific interrupt.
*/
  long IntConfigure(unsigned long Sn, unsigned long Src, unsigned long Mode, unsigned long Type);

/*! \fn long IntEnable(unsigned long Sn)
    \brief Enable interrupts
    \param Sn The serial number of the device.
  \return Success if 0, else error code.

  This command is used to enable interrupt globally.
*/
  long IntEnable(unsigned long Sn);

/*! \fn long IntDisable(unsigned long Sn)
    \brief Disable Interrupts
    \param Sn The serial number of the device.
  \return Success if 0, else error code.

  This command is used to disable interrupt globally. Make sure to do this before releasing a device.
*/
  long IntDisable(unsigned long Sn);

/*! \fn long CTWrite(unsigned long Sn, unsigned long Port, unsigned long Value)
    \brief Write to a counter-timer
    \param Sn The serial number of the device.
  \param Port The counter to write too.
  \param Value The value to write to the counter-timer.
  \return Success if 0, else error code.

  This command is used to write an initial value to a counter-timer.
*/
  long CTWrite(unsigned long Sn, unsigned long Port, unsigned long Value);

/*! \fn long CTRead(unsigned long Sn, unsigned long Port, unsigned long *Value)
    \brief Read from a counter-timer
    \param Sn The serial number of the device.
  \param Port The counter to read from.
  \param *Value A pointer to a 32-bit unsigned value.
  \return Success if 0, else error code.

  This command is used to read a counter-timer.
*/
  long CTRead(unsigned long Sn, unsigned long Port, unsigned long *Value);

/*! \fn long CTConfig(unsigned long Sn, unsigned long Ct, unsigned long Mode, unsigned long Type, unsigned long ClkSrc, unsigned long GateSrc)
    \brief Configure a counter-timer
    \param Sn The serial number of the device.
  \param Ct The counter to configure.
  \param Mode The counter-timer mode.
  \param Type The counter-timer type.
  \param ClkSrc The clock source.
  \param GateSrc The gate source.
  \return Success if 0, else error code.

  This command is used to configure a specific counter-timer.
*/
  long CTConfig(unsigned long Sn, unsigned long Ct, unsigned long Mode, unsigned long Type, unsigned long ClkSrc, unsigned long GateSrc);

/*! \fn long CTSoftGate(unsigned long Sn, unsigned long Port, unsigned long Gate)
    \brief Control the counter-timer gate
    \param Sn The serial number of the device.
  \param Port The counter to control.
  \param Gate The gate value. 1 is on and 0 is off.
  \return Success if 0, else error code.

  This command is used to control the gate of a counter-timer via software. Make sure the gate is not set for external.
*/
  long CTSoftGate(unsigned long Sn, unsigned long Port, unsigned long Gate);

/*! \fn long DAWrite(unsigned long Sn, unsigned long Channel, long uVoltage)
    \brief Write a voltage to a D/A channel
    \param Sn The serial number of the device.
  \param Channel The channel to write to.
  \param uVoltage The voltage to write in microvolts.
  \return Success if 0, else error code.

  This command is used to write a single votlage to a specific D/A channel. Make sure the channel is not occupied a streaming.
*/
  long DAWrite(unsigned long Sn, unsigned long Channel, long uVoltage);

/*! \fn long DAConfig(unsigned long Sn, unsigned long Channel, unsigned long Frequency, unsigned long ClkSrc, unsigned long GateSrc, unsigned long Continuous, unsigned long Length, long *uVoltage)
    \brief Configure a D/A channel for streaming
    \param Sn The serial number of the device.
  \param Channel The channel to configure.
  \param Frequency The update frequency.
  \param ClkSrc The clock source for the channel. Some devices has a common clock for all channels.
  \param GateSrc The gate source for the channel.
  \param Continuous Set to 1 for continuous waveform generation.
  \param Length The length of the microvolt buffer.
  \param *uVoltage A pointer to a 32-bit wide microvolt buffer. This buffer contains the
  \return Success if 0, else error code.

  This command is used to write a single votlage to a specific D/A channel. Make sure the channel is not occupied a streaming.
*/
  long DAConfig(unsigned long Sn, unsigned long Channel, unsigned long Frequency, unsigned long ClkSrc, unsigned long GateSrc, unsigned long Continuous, unsigned long Length, long *uVoltage);

/*! \fn long DAControl(unsigned long Sn, unsigned long Channel, unsigned long Command)
    \brief Control a D/A channel
  \param Sn The serial number of the device.
  \param Channel The channel to control
  \param Command The command to execute \sa DACCOMMANDS
  \return Success if 0, else error code.

  This command is used to control the D/A streaming process, e.g. stop, start etc.
*/
  long DAControl(unsigned long Sn, unsigned long Channel, unsigned long Command);

/*! \fn long DAUpdateData(unsigned long Sn, unsigned long Channel, unsigned long Length, long *uVoltage)
    \brief Update D/A data buffer
  \param Sn The serial number of the device.
  \param Channel The channel to update
  \param Length The length of the buffer in samples.
  \param *uVoltage A pointer a 32-bit signed buffer.
  \return Success if 0, else error code.

  This command is used to update the D/A buffer while streaming data.
*/
  long DAUpdateData(unsigned long Sn, unsigned long Channel, unsigned long Length, long *uVoltage);

/*! \fn long ADSingle(unsigned long Sn, unsigned long Channel, unsigned long Gain, unsigned long Range, long *uVoltage)
    \brief Read a A/D voltage
  \param Sn The serial number of the device.
  \param Channel The channel to read
  \param Gain The voltage gain. See device manual for reference.
  \param Range The voltage range. See device manual for further reference.
  \param *uVoltage A pointer a 32-bit signed buffer. This will hold the the value read.
  \return Success if 0, else error code.

  This command is used to read a single channel from an A/D device.
*/
  long ADSingle(unsigned long Sn, unsigned long Channel, unsigned long Gain, unsigned long Range, long *uVoltage);

/*! \fn long ADConfig(unsigned long Sn, unsigned long *Freq, unsigned long ClkSrc, unsigned long Burst, unsigned long Range, unsigned long *ChanList, unsigned long *GainList, unsigned long ListSize)
    \brief Configure A/D sub-system
  \param Sn The serial number of the device.
  \param *Freq A pointer to a frequency variable. Set it to a frequency. The returned value will be the actual frequency.
  \param ClkSrc The A/D clock source.
  \param Burst Set to 1 for burst mode. PCI30FG and PC104-30FG only.
  \param Range The voltage range. See device manual for further reference.
  \param *ChanList A pointer to an array that has the channel list to sample.
  \param *GainList A pointer to an array the same length as the channel list. this is a list of gains and ranges for each channel.
  \param ListSize The length of the channel list.
  \return Success if 0, else error code.

  ADConfig is used to configure the A/D sub-system for steaming. Each channel in the channel list will be scanned and configured to the corresponding value in the gain list.
*/
  long ADConfig(unsigned long Sn, unsigned long *Freq, unsigned long ClkSrc, unsigned long Burst, unsigned long Range, unsigned long *ChanList, unsigned long *GainList, unsigned long ListSize);

/*! \fn long ADStart(unsigned long Sn)
    \brief Start A/D streaming
  \param Sn The serial number of the device.
  \return Success if 0, else error code.

  ADStart will start the A/D streaming process.
*/
  long ADStart(unsigned long Sn);

/*! \fn long ADStop(unsigned long Sn)
    \brief Stop A/D streaming
  \param Sn The serial number of the device.
  \return Success if 0, else error code.

  ADStop will stop the A/D streaming process.
*/
  long ADStop(unsigned long Sn);

/*! \fn long ADGetData(unsigned long Sn, long *Buf, unsigned long *BufSize)
    \brief Retrieve A/D data
  \param Sn The serial number of the device.
  \param *Buf A pointer to a buffer to copy the requested data too (32-bit signed).
  \param *BufSize A pointer to a value of buffer size or requested number of samples. Check this value upon returned for the actual number  of samples copied to the buffer.
  \return Success if 0, else error code.

  ADGetData is used to retrieve data from the driver buffer. The data will then be removed from the driver buffer. Use the ADUNREAD query   to determine the number of available samples in the driver buffer.

  \sa ADGetDataRaw
*/
  long ADGetData(unsigned long Sn, long *Buf, unsigned long *BufSize);

/*! \fn long ADGetDataRaw(unsigned long Sn, unsigned short *Buf, unsigned long *BufSize)
    \brief Retrieve raw A/D data
  \param Sn The serial number of the device.
  \param *Buf A pointer to a buffer to copy the requested data too (16-bit unsigned).
  \param *BufSize A pointer to a value of buffer size or requested number of samples. Check this value upon returned for the actual number
  of samples copied to the buffer.
  \return Success if 0, else error code.

  ADGetDataRaw is used to retrieve raw unconverted data from the driver buffer. The data will then be removed from the driver buffer. Use the ADUNREAD query to determine the number of available samples in the driver buffer.

  \sa ADGetData
*/
  long ADGetDataRaw(unsigned long Sn, unsigned short *Buf, unsigned long *BufSize);

/*! \fn long StrBoardName(unsigned long Sn, char *StrBoardName)
    \brief Get device name
  \param Sn The serial number of the device.
  \param *StrBoardName A pointer to an character array of 40.
  \return Success if 0, else error code.

  StrBoardName is used to get the decsiptive name of a device.

  \sa StrError
*/
  long StrBoardName(unsigned long Sn, char *StrBoardName);

/*! \fn long StrError(long Error, char *StrError)
    \brief Convert error code to string
  \param Error Error code.
  \param *StrError A pointer to an character array of 40.
  \return Success if 0, else error code.

  StrBoardName is used to get the decsiptive name of a device.

  \sa StrBoardName
*/
  long StrError(long Error, char *StrError);
};

/*!\def pEDREObject Defines an object reference */
#define pEDREObject EDREObject*
/** @} */


/** @defgroup EDREDEVICE EDR Enhanced Device Object
 *  @{
 */

/*!
  \brief EDR Enhanced Device Object

  The EDREDevice object is used to control a specific device. Because the object is bound to specific device no serial number is needed as first parameter for each of the functions. (Linux Only)
*/
class EDREDevice
{
public:
/*! \fn ~EDREDevice
    \brief Desctructor
  Device destructor
*/
	virtual ~EDREDevice();

/*! \fn long StrBoardName(char *StrBoardName)
    \brief Get device name
  \param *StrBoardName A pointer to an character array of 40.
  \return Success if 0, else error code.

  StrBoardName is used to get the decsiptive name of a device.

  \sa StrError
*/
  long StrBoardName(char *StrBoardName);

/*! \fn long StrError(long Error, char *StrError)
    \brief Convert error code to string
  \param Error Error code.
  \param *StrError A pointer to an character array of 40.
  \return Success if 0, else error code.

  StrBoardName is used to get the decsiptive name of a device.

  \sa StrBoardName
*/
  long StrError(long Error, char *StrError);

/*! \fn long Query(unsigned long QueryCode, unsigned long Param)
    \brief Query command
    \param QueryCode The query code that must be executed.
  \param Param An extra query parameter.
  \return Result of the query code.

  Use this command to query a device property or API property.

  \sa query.h
*/
  virtual long Query(unsigned long QueryCode, unsigned long Param);

/*! \fn long DioWrite(unsigned long Port, unsigned long Value)
    \brief Digital I/O write command
    \param Port The port to write too.
  \param Value The value that must be written to the port.
  \return Success if 0, else error code.

  Use this command command to output a digital value.
*/
  virtual long DioWrite(unsigned long Port, unsigned long Value);

/*! \fn long DioRead(unsigned long Port, unsigned long *Value)
    \brief Digital I/O read command
    \param Port The port to read from.
  \param *Value A pointer to an 32-bit unsigned value.
  \return Success if 0, else error code

  Use this command command to output a digital value.
*/
  virtual long DioRead(unsigned long Port, unsigned long *Value);

/*! \fn long MioConfig(unsigned long Port, unsigned long Value)
    \brief Digital I/O configure command - PCI14B only
    \param Port The port to configure.
  \param Value Configuration value.
  \return Success if 0, else error code.

  Use this command to configure the multi-I/O port on the PCI14B device.
*/
  virtual long MioConfig(unsigned long Port, unsigned long Value);

/*! \fn long WaitOnInterrupt(void)
    \brief Wait for an interrupt event
  \return Interrupt source. Each bit that is set represents a source.

  This command is used to wait for an interrupt to occur. Make sure to use a
  separate thread to wait, otherwise the user application will simply hang.
*/
  virtual long WaitOnInterrupt(void);

/*! \fn long ReleaseInterrupt(void)
    \brief Release WaitOnInterrupt prematurely.
  \return Success if 0, else error code.

  This command is used to wait for an interrupt to occur. Make sure to use a
  separate thread to wait, otherwise the user application will simply hang.
*/
  virtual long ReleaseInterrupt(void);

/*! \fn long IntConfigure(unsigned long Src, unsigned long Mode, unsigned long Type)
    \brief Configure interrupt source
  \param Src The interrupt source number.
  \param Mode The interrupt mode. Refer to device manual.
  \param Type The interrupt type. Refer to device manual.
  \return Success if 0, else error code.

  This command is used to configure an interrupt source. It can be used to disable or
  enable a specific interrupt.
*/
  virtual long IntConfigure(unsigned long Src, unsigned long Mode, unsigned long Type);

/*! \fn long IntEnable(void)
    \brief Enable interrupts
  \return Success if 0, else error code.

  This command is used to enable interrupt globally.
*/
  virtual long IntEnable(void);

/*! \fn long IntDisable(void)
    \brief Disable interrupts
  \return Success if 0, else error code.

  This command is used to disable interrupt globally.
*/
  virtual long IntDisable(void);

/*! \fn long CTWrite(unsigned long Port, unsigned long Value)
    \brief Write to a counter-timer
  \param Port The counter to write too.
  \param Value The value to write to the counter-timer.
  \return Success if 0, else error code.

  This command is used to write an initial value to a counter-timer.
*/
  virtual long CTWrite(unsigned long Port, unsigned long Value);

/*! \fn long CTRead(unsigned long Port, unsigned long *Value)
    \brief Read from a counter-timer
  \param Port The counter to read from.
  \param *Value A pointer to a 32-bit unsigned value.
  \return Success if 0, else error code.

  This command is used to read a counter-timer.
*/
  virtual long CTRead(unsigned long Port, unsigned long *Value);

/*! \fn long CTConfig(unsigned long Ct, unsigned long Mode, unsigned long Type, unsigned long ClkSrc, unsigned long GateSrc)
    \brief Configure a counter-timer
  \param Ct The counter to configure.
  \param Mode The counter-timer mode.
  \param Type The counter-timer type.
  \param ClkSrc The clock source.
  \param GateSrc The gate source.
  \return Success if 0, else error code.

  This command is used to configure a specific counter-timer.
*/
  virtual long CTConfig(unsigned long Ct, unsigned long Mode, unsigned long Type, unsigned long ClkSrc, unsigned long GateSrc);

/*! \fn long CTSoftGate(unsigned long Port, unsigned long Gate)
    \brief Control the counter-timer gate
  \param Port The counter to control.
  \param Gate The gate value. 1 is on and 0 is off.
  \return Success if 0, else error code.

  This command is used to control the gate of a counter-timer via software. Make sure the gate is not set for external.
*/
  virtual long CTSoftGate(unsigned long Port, unsigned long Gate);

/*! \fn long DAWrite(unsigned long Channel, long uVoltage)
    \brief Write a voltage to a D/A channel
  \param Channel The channel to write to.
  \param uVoltage The voltage to write in microvolts.
  \return Success if 0, else error code.

  This command is used to write a single votlage to a specific D/A channel. Make sure the channel is not occupied a streaming.
*/
  virtual long DAWrite(unsigned long Channel, long uVoltage);

/*! \fn long DAConfig(unsigned long Channel, unsigned long Frequency, unsigned long ClkSrc, unsigned long GateSrc, unsigned long Continuous, unsigned long Length, long *uVoltage)
    \brief Configure a D/A channel for streaming
  \param Channel The channel to configure.
  \param Frequency The update frequency.
  \param ClkSrc The clock source for the channel. Some devices has a common clock for all channels.
  \param GateSrc The gate source for the channel.
  \param Continuous Set to 1 for continuous waveform generation.
  \param Length The length of the microvolt buffer.
  \param *uVoltage A pointer to a 32-bit wide microvolt buffer. This buffer contains the
  \return Success if 0, else error code.

  This command is used to write a single votlage to a specific D/A channel. Make sure the channel is not occupied a streaming.
*/
  virtual long DAConfig(unsigned long Channel, unsigned long Frequency, unsigned long ClkSrc, unsigned long GateSrc, unsigned long Continuous, unsigned long Length, long *uVoltage);

/*! \fn long DAControl(unsigned long Channel, unsigned long Command)
    \brief Control a D/A channel
  \param Channel The channel to control
  \param Command The command to execute \sa DACCOMMANDS
  \return Success if 0, else error code.

  This command is used to control the D/A streaming process, e.g. stop, start etc.
*/
  virtual long DAControl(unsigned long Channel, unsigned long Command);

/*! \fn long DAUpdateData(unsigned long Channel, unsigned long Length, long *uVoltage)
    \brief Update D/A data buffer
  \param Channel The channel to update
  \param Length The length of the buffer in samples.
  \param *uVoltage A pointer a 32-bit signed buffer.
  \return Success if 0, else error code.

  This command is used to update the D/A buffer while streaming data.
*/
  virtual long DAUpdateData(unsigned long Channel, unsigned long Length, long *uVoltage);

/*! \fn long ADSingle(unsigned long Channel, unsigned long Gain, unsigned long Range, long *uVoltage)
    \brief Read a A/D voltage
  \param Channel The channel to read
  \param Gain The voltage gain. See device manual for reference.
  \param Range The voltage range. See device manual for further reference.
  \param *uVoltage A pointer a 32-bit signed buffer. This will hold the the value read.
  \return Success if 0, else error code.

  This command is used to read a single channel from an A/D device.
*/
  virtual long ADSingle(unsigned long Channel, unsigned long Gain, unsigned long Range, long *uVoltage);

/*! \fn long ADConfig(unsigned long *Freq, unsigned long ClkSrc, unsigned long Burst, unsigned long Range, unsigned long *ChanList, unsigned long *GainList, unsigned long ListSize)
    \brief Configure A/D sub-system
  \param *Freq A pointer to a frequency variable. Set it to a frequency. The returned value will be the actual frequency.
  \param ClkSrc The A/D clock source.
  \param Burst Set to 1 for burst mode. PCI30FG and PC104-30FG only.
  \param Range The voltage range. See device manual for further reference.
  \param *ChanList A pointer to an array that has the channel list to sample.
  \param *GainList A pointer to an array the same length as the channel list. this is a list of gains and ranges for each channel.
  \param ListSize The length of the channel list.
  \return Success if 0, else error code.

  ADConfig is used to configure the A/D sub-system for steaming. Each channel in the channel list will be scanned and configured to the corresponding value in the gain list.
*/
  virtual long ADConfig(unsigned long *Freq, unsigned long ClkSrc, unsigned long Burst, unsigned long Range, unsigned long *ChanList, unsigned long *GainList, unsigned long ListSize);

/*! \fn long ADStart(void)
    \brief Start A/D streaming
  \return Success if 0, else error code.

  ADStart will start the A/D streaming process.
*/
  virtual long ADStart(void);

/*! \fn long ADStop(void)
    \brief Stop A/D streaming
  \return Success if 0, else error code.

  ADStop will stop the A/D streaming process.
*/
  virtual long ADStop(void);

/*! \fn long ADGetData(long *Buf, unsigned long *BufSize)
    \brief Retrieve A/D data
  \param *Buf A pointer to a buffer to copy the requested data too (32-bit signed).
  \param *BufSize A pointer to a value of buffer size or requested number of samples. Check this value upon returned for the actual number  of samples copied to the buffer.
  \return Success if 0, else error code.

  ADGetData is used to retrieve data from the driver buffer. The data will then be removed from the driver buffer. Use the ADUNREAD query   to determine the number of available samples in the driver buffer.

  \sa ADGetDataRaw
*/
  virtual long ADGetData(long *Buf, unsigned long *BufSize);

/*! \fn long ADGetDataRaw(unsigned short *Buf, unsigned long *BufSize)
    \brief Retrieve raw A/D data
  \param *Buf A pointer to a buffer to copy the requested data too (16-bit unsigned).
  \param *BufSize A pointer to a value of buffer size or requested number of samples. Check this value upon returned for the actual number
  of samples copied to the buffer.
  \return Success if 0, else error code.

  ADGetDataRaw is used to retrieve raw unconverted data from the driver buffer. The data will then be removed from the driver buffer. Use the ADUNREAD query to determine the number of available samples in the driver buffer.

  \sa ADGetData
*/
  virtual long ADGetDataRaw(unsigned short *Buf, unsigned long *BufSize);
};

/*!\def pEDREDevice Defines an object reference */
#define pEDREDevice EDREDevice*
/** @} */

#endif /*_EDRE_OBJECT_PUBLIC_H_*/
/**
 * End of File
 */
