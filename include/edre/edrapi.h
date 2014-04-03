/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2003 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * edrapi.h -- EDR Enhanced API header file
 *
 *******************************************************************************
 * 2005-02-24  Jan Zwiegers  <jan@eage.co.za>
 * - Add doxygen style comments
 *******************************************************************************/

/*! \file edrapi.h
    \brief EDR Enhanced API Definition File

    This is the main EDR Enhanced include file. It defines all API functions.
*/

#ifndef __EDRE_API__
#define __EDRE_API__

#ifdef __cplusplus
extern "C"{
#endif

/*!\def EDRE_MAXDEVICES The maximum devices supported by the EDR Enhanced API. */
#define EDRE_MAXDEVICES 255
/*!\def EDRE_MAXBOARDS The maximum boards supported by the EDR Enhanced API. */
#define EDRE_MAXBOARDS 20
/*!\def APISERIALNO The EDR Enhanced generic serial number. Obsolete. */
#define APISERIALNO 0x3b9aca00
/*!\def EDRE_MAXDIOCHANNELS The maximum digital I/O channels per device. */
#define EDRE_MAXDIOCHANNELS 1000

#ifdef __linux__
#define EDRE_API
#else
/*!\def EDRE_API The Windows API interface. */
#define EDRE_API __stdcall
#endif /*LINUX*/

/*! \brief EDR Enhanced Platforms & Interfaces */
enum EDRE_PLATFORMS
{
 W95 = 1, /*!< Windows 95 */
 WNT = 2, /*!< Windows NT 4.0 */
 WDM98 = 3, /*!< Windows 98 WDM */
 WDM2000 = 4, /*!< Windows 2000, XP */
 ETHER = 5, /*!< Ethernet */
 RHLINUX = 6, /*!< Linux type platform */
 SERIAL = 7, /*!< Serial Interface  */
 USB = 8, /*!< USB Interface  */
};


/** @defgroup EDREAPI EDR Enhanced Application Programming Interface functions
 *  @{
 */
/*! \fn long EDRE_API EDRE_DioWrite(unsigned long Sn, unsigned long Port, unsigned long Value)
    \brief Digital I/O write command
    \param Sn The device serial number to execute the digital I/O write.
    \param Port The port to write too.
    \param Value The value that must be written to the port.
    \return Success if 0, else error code.

    Use this command command to output a digital value.
*/
long EDRE_API EDRE_DioWrite(unsigned long Sn, unsigned long Port, unsigned long Value);

/*! \fn long EDRE_API EDRE_DioRead(unsigned long Sn, unsigned long Port, unsigned long *Value)
    \brief Digital I/O read command
    \param Sn The device serial number to execute the digital I/O read.
    \param Port The port to read from.
    \param *Value A pointer to an 32-bit unsigned value.
    \return Success if 0, else error code

    Use this command command to output a digital value.
*/
long EDRE_API EDRE_DioRead(unsigned long Sn, unsigned long Port, unsigned long *Value);

/*! \fn long EDRE_API EDRE_MioConfig(unsigned long Sn, unsigned long Port, unsigned long Value)
    \brief Digital I/O configure command - PCI14B only
    \param Sn The serial number of the device to configure.
    \param Port The port to configure.
    \param Value Configuration value.
    \return Success if 0, else error code.

    Use this command to configure the multi-I/O port on the PCI14B device.
*/
long EDRE_API EDRE_MioConfig(unsigned long Sn, unsigned long Port, unsigned long Value);

/*! \fn long EDRE_API EDRE_WaitOnInterrupt(unsigned long Sn)
    \brief Wait for an interrupt event
    \param Sn The device serial number to generate the interrupt
    \return Interrupt source. Each bit that is set represents a source.

    This command is used to wait for an interrupt to occur. Make sure to use a
    separate thread to wait, otherwise the user application will simply hang.
*/
long EDRE_API EDRE_WaitOnInterrupt(unsigned long Sn);

/*! \fn long EDRE_API EDRE_ReleaseInterrupt(unsigned long Sn)
    \brief Release WaitOnInterrupt prematurely.
    \param Sn The device that is currently waiting for an interrupt.
    \return Success if 0, else error code.

    This command is used to wait for an interrupt to occur. Make sure to use a
    separate thread to wait, otherwise the user application will simply hang.
*/
long EDRE_API EDRE_ReleaseInterrupt(unsigned long Sn);

/*! \fn long EDRE_API EDRE_IntConfigure(unsigned long Sn, unsigned long Src, unsigned long Mode, unsigned long Type)
    \brief Configure interrupt source
    \param Sn The serial number of the device to configure.
    \param Src The interrupt source number.
    \param Mode The interrupt mode. Refer to device manual.
    \param Type The interrupt type. Refer to device manual.
    \return Success if 0, else error code.

    This command is used to configure an interrupt source. It can be used to disable or
    enable a specific interrupt.
*/
long EDRE_API EDRE_IntConfigure(unsigned long Sn, unsigned long Src, unsigned long Mode, unsigned long Type);

/*! \fn long EDRE_API EDRE_IntEnable(unsigned long Sn)
    \brief Enable interrupts
    \param Sn The serial number of the device.
    \return Success if 0, else error code.

    This command is used to enable interrupt globally.
*/
long EDRE_API EDRE_IntEnable(unsigned long Sn);

/*! \fn long EDRE_API EDRE_IntDisable(unsigned long Sn)
    \brief Disable Interrupts
    \param Sn The serial number of the device.
    \return Success if 0, else error code.

    This command is used to disable interrupt globally. Make sure to do this before releasing a device.
*/
long EDRE_API EDRE_IntDisable(unsigned long Sn);

/*! \fn long EDRE_API EDRE_CTWrite(unsigned long Sn, unsigned long Port, unsigned long Value)
    \brief Write to a counter-timer
    \param Sn The serial number of the device.
    \param Port The counter to write too.
    \param Value The value to write to the counter-timer.
    \return Success if 0, else error code.

    This command is used to write an initial value to a counter-timer.
*/
long EDRE_API EDRE_CTWrite(unsigned long Sn, unsigned long Ct, unsigned long Value);

/*! \fn long EDRE_API EDRE_CTRead(unsigned long Sn, unsigned long Port, unsigned long *Value)
    \brief Read from a counter-timer
    \param Sn The serial number of the device.
    \param Port The counter to read from.
    \param *Value A pointer to a 32-bit unsigned value.
    \return Success if 0, else error code.

    This command is used to read a counter-timer.
*/
long EDRE_API EDRE_CTRead(unsigned long Sn, unsigned long Ct, unsigned long *Value);


/*! \fn long EDRE_API EDRE_CTSoftGate(unsigned long Sn, unsigned long Port, unsigned long Gate)
    \brief Control the counter-timer gate
    \param Sn The serial number of the device.
    \param Port The counter to control.
    \param Gate The gate value. 1 is on and 0 is off.
    \return Success if 0, else error code.

    This command is used to control the gate of a counter-timer via software. Make sure the gate is not set for external.
*/
long EDRE_API EDRE_CTSoftGate(unsigned long Sn, unsigned long Ct, unsigned long Gate);

/*! \fn long EDRE_API EDRE_CTCfg(unsigned long Sn, unsigned long Ct, unsigned long Mode, unsigned long BCD)
    \brief Counter-Timer configuration function - obsolete
    \sa EDRE_CTConfig
*/
long EDRE_API EDRE_CTCfg(unsigned long Sn, unsigned long Ct, unsigned long Mode, unsigned long BCD);

/*! \fn long EDRE_API EDRE_CTConfig(unsigned long Sn, unsigned long Ct, unsigned long Mode, unsigned long Type, unsigned long ClkSrc, unsigned long GateSrc)
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
long EDRE_API EDRE_CTConfig(unsigned long Sn, unsigned long Ct, unsigned long Mode, unsigned long Type, unsigned long ClkSrc, unsigned long GateSrc);

/*! \fn long EDRE_API EDRE_StrBoardName(unsigned long Sn, char *StrBoardName)
    \brief Get device name
    \param Sn The serial number of the device.
    \param *StrBoardName A pointer to an character array of 40.
    \return Success if 0, else error code.

    StrBoardName is used to get the decsiptive name of a device.

    \sa StrError
*/
long EDRE_API EDRE_StrBoardName(unsigned long Sn, char *StrBoardName);

/*! \fn long EDRE_API EDRE_StrError(long Error, char *StrError)
    \brief Convert error code to string
    \param Error Error code.
    \param *StrError A pointer to an character array of 40.
    \return Success if 0, else error code.

    StrBoardName is used to get the decsiptive name of a device.

    \sa StrBoardName
*/
long EDRE_API EDRE_StrError(long Error, char *StrError);

/*! \fn long EDRE_API EDRE_DAWrite(unsigned long Sn, unsigned long Channel, long uVoltage)
    \brief Write a voltage to a D/A channel
    \param Sn The serial number of the device.
    \param Channel The channel to write to.
    \param uVoltage The voltage to write in microvolts.
    \return Success if 0, else error code.

    This command is used to write a single votlage to a specific D/A channel. Make sure the channel is not occupied a streaming.
*/
long EDRE_API EDRE_DAWrite(unsigned long Sn, unsigned long Channel, long uVoltage);

/*! \fn long EDRE_API EDRE_DAConfig(unsigned long Sn, unsigned long Channel, unsigned long Frequency, unsigned long ClkSrc, unsigned long GateSrc, unsigned long Continuous, unsigned long Length, long *uVoltage)
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
long EDRE_API EDRE_DAConfig(unsigned long Sn, unsigned long Channel, unsigned long Frequency, unsigned long ClkSrc, unsigned long GateSrc, unsigned long Continuous, unsigned long Length, long *uVoltage);

/*! \fn long EDRE_API EDRE_DAControl(unsigned long Sn, unsigned long Channel, unsigned long Command)
    \brief Control a D/A channel
    \param Sn The serial number of the device.
    \param Channel The channel to control
    \param Command The command to execute \sa DACCOMMANDS
    \return Success if 0, else error code.

    This command is used to control the D/A streaming process, e.g. stop, start etc.
*/
long EDRE_API EDRE_DAControl(unsigned long Sn, unsigned long Channel, unsigned long Command);

/*! \fn long EDRE_API EDRE_DAUpdateData(unsigned long Sn, unsigned long Channel, unsigned long Length, long *uVoltage)
    \brief Update D/A data buffer
    \param Sn The serial number of the device.
    \param Channel The channel to update
    \param Length The length of the buffer in samples.
    \param *uVoltage A pointer a 32-bit signed buffer.
    \return Success if 0, else error code.

    This command is used to update the D/A buffer while streaming data.
*/
long EDRE_API EDRE_DAUpdateData(unsigned long Sn, unsigned long Channel, unsigned long Length, long *uVoltage);

/*! \fn long EDRE_API EDRE_ADSingle(unsigned long Sn, unsigned long Channel, unsigned long Gain, unsigned long Range, long *uVoltage)
    \brief Read a A/D voltage
    \param Sn The serial number of the device.
    \param Channel The channel to read
    \param Gain The voltage gain. See device manual for reference.
    \param Range The voltage range. See device manual for further reference.
    \param *uVoltage A pointer a 32-bit signed buffer. This will hold the the value read.
    \return Success if 0, else error code.

    This command is used to read a single channel from an A/D device.
*/
long EDRE_API EDRE_ADSingle(unsigned long Sn, unsigned long Channel, unsigned long Gain, unsigned long Range, long *uVoltage);

/*! \fn long EDRE_API EDRE_ADConfig(unsigned long Sn, unsigned long *Freq, unsigned long ClkSrc, unsigned long Burst, unsigned long Range, unsigned long *ChanList, unsigned long *GainList, unsigned long ListSize)
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
long EDRE_API EDRE_ADConfig(unsigned long Sn, unsigned long *Freq, unsigned long ClkSrc, unsigned long Burst, unsigned long Range, unsigned long *ChanList, unsigned long *GainList, unsigned long ListSize);
/*! \fn long EDRE_API EDRE_ADStart(unsigned long Sn)
    \brief Start A/D streaming
    \param Sn The serial number of the device.
    \return Success if 0, else error code.

    ADStart will start the A/D streaming process.
*/
long EDRE_API EDRE_ADStart(unsigned long Sn);

/*! \fn long EDRE_API EDRE_ADStop(unsigned long Sn)
    \brief Stop A/D streaming
    \param Sn The serial number of the device.
    \return Success if 0, else error code.

    ADStop will stop the A/D streaming process.
*/
long EDRE_API EDRE_ADStop(unsigned long Sn);

/*! \fn long EDRE_API EDRE_ADGetData(unsigned long Sn, long *Buf, unsigned long *BufSize)
    \brief Retrieve A/D data
    \param Sn The serial number of the device.
    \param *Buf A pointer to a buffer to copy the requested data too (32-bit signed).
    \param *BufSize A pointer to a value of buffer size or requested number of samples. Check this value upon returned for the actual number    of samples copied to the buffer.
    \return Success if 0, else error code.

    ADGetData is used to retrieve data from the driver buffer. The data will then be removed from the driver buffer. Use the ADUNREAD query     to determine the number of available samples in the driver buffer.

    \sa EDRE_ADGetDataRaw
*/
long EDRE_API EDRE_ADGetData(unsigned long Sn, long *Buf, unsigned long *BufSize);

/*! \fn long EDRE_API EDRE_ADGetDataFromFile(unsigned long Sn, unsigned long Start, long *Buf, unsigned long *BufSize)
    \brief Retrieve A/D data
    \param Sn The serial number of the device.
    \param Start Start position the file.
    \param *Buf A pointer to a buffer to copy the requested data too (32-bit signed).
    \param *BufSize A pointer to a value of buffer size or requested number of samples. Check this value upon returned for the actual number    of samples copied to the buffer.
    \return Success if 0, else error code.

    ADGetData is used to retrieve data from file.

    \sa EDRE_ADGetDataRawFromFile
*/
long EDRE_API EDRE_ADGetDataFromFile(unsigned long Sn, unsigned long Start, long *Buf, unsigned long *BufSize);

/*! \fn long EDRE_API EDRE_ADGetDataRaw(unsigned long Sn, unsigned short *Buf, unsigned long *BufSize)
    \brief Retrieve raw A/D data
    \param Sn The serial number of the device.
    \param *Buf A pointer to a buffer to copy the requested data too (16-bit unsigned).
    \param *BufSize A pointer to a value of buffer size or requested number of samples. Check this value upon returned for the actual number
    of samples copied to the buffer.
    \return Success if 0, else error code.

    ADGetDataRaw is used to retrieve raw unconverted data from the driver buffer. The data will then be removed from the driver buffer. Use the ADUNREAD query to determine the number of available samples in the driver buffer.

    \sa EDRE_ADGetData
*/
long EDRE_API EDRE_ADGetDataRaw(unsigned long Sn, unsigned short *Buf, unsigned long *BufSize);

/*! \fn long EDRE_API EDRE_ADGetDataRawFromFile(unsigned long Sn, unsigned long Start, unsigned short *Buf, unsigned long *BufSize)
    \brief Retrieve raw A/D data
    \param Sn The serial number of the device.
    \param Start Start position in the file.
    \param *Buf A pointer to a buffer to copy the requested data too (16-bit unsigned).
    \param *BufSize A pointer to a value of buffer size or requested number of samples. Check this value upon returned for the actual number
    of samples copied to the buffer.
    \return Success if 0, else error code.

    ADGetDataRawFromFile is used to retrieve raw unconverted data from s file.

    \sa EDRE_ADGetDataFromFile
*/
long EDRE_API EDRE_ADGetDataRawFromFile(unsigned long Sn, unsigned long Start, unsigned short *Buf, unsigned long *BufSize);

/*! \fn long EDRE_API EDRE_ADOpenStreamFile(unsigned long Sn, unsigned long Mode, char *FileName)
    \brief Open an A/D streaming file
    \param Sn The device serial number to query if relevant.
    \param Mode File mode.
    \param FileName File name.
    \return Result of the query code.

    Use this command to open a file for streaming data. When calling the EDRE_ADGetData command data will automatically be written to file.

    \sa EDRE_ADCloseStreamFile
*/
long EDRE_API EDRE_ADOpenStreamFile(unsigned long Sn, unsigned long Mode, char *FileName);

/*! \fn void EDRE_API EDRE_ADCloseStreamFile(unsigned Sn)
    \brief Close an A/D streaming file
    \param Sn The device serial number to query if relevant.

    Close a file previously opened.

    \sa EDRE_ADOpenStreamFile
*/
void EDRE_API EDRE_ADCloseStreamFile(unsigned Sn);

/*! \fn long EDRE_API EDRE_Query(unsigned long Sn, unsigned long QueryCode, unsigned long Param)
    \brief Query command
    \param Sn The device serial number to query if relevant.
    \param QueryCode The query code that must be executed.
    \param Param An extra query parameter.
    \return Result of the query code.

    Use this command to query a device property or API property.

    \sa query.h
*/
long EDRE_API EDRE_Query(unsigned long Sn, unsigned long QueryCode, unsigned long Param);

/*! \fn long EDRE_API EDRE_CalcCJCmC(long cjcuv)
    \brief Convert millivolt to millidegrees for CJC
    \param cjcuv CJC channel microvolt.
    \return millidegrees centigrade.

    Convert a millivolt value to milli degrees centigrade to calculate ambient temperature.
*/
long EDRE_API EDRE_CalcCJCmC(long cjcuv);

/*! \fn long EDRE_API EDRE_CalcRTDmC(long rtduv)
    \brief Convert millivolt to millidegrees for RTD
    \param rtduv RTD microvolt.
    \return millidegrees centigrade.

    Convert a millivolt value to milli degrees centigrade to calculate RTD temperature.
*/
long EDRE_API EDRE_CalcRTDmC(long rtduv);

/*! \fn long EDRE_API EDRE_CalcTCmC(long tctype, long tcuv, long ambientmc)
    \brief Convert millivolt to millidegrees fot T/C
    \param tctype Thermocouple type.
    \param tcuv Thermocouple microvotlage.
    \param ambientmc Ambient temperature.
    \return millidegrees centigrade.

    Convert a millivolt value to milli degrees centigrade to calculate thermocouple temperature.
*/
long EDRE_API EDRE_CalcTCmC(long tctype, long tcuv, long ambientmc);
/** @} */

#ifdef __cplusplus
}
#endif

#endif /*__EDRE_API__*/
