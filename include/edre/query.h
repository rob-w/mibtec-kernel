/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2003 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * query.h -- EDR Enhanced Query Codes. Part of EDR Enhanced SDK
 *
 *******************************************************************************
 * 2009-05-05  Jan Zwiegers  <jan@eage.co.za>
 * - Add query codes to program pull-up and pull-down resistors
 *
 * 2009-01-22  Jan Zwiegers  <jan@eage.co.za>
 * - Add BRDRESET query code
 *
 * 2005-02-25  Jan Zwiegers  <jan@eage.co.za>
 * - Add APILOADDEV code
 * - Fix minor mistakes 
 * - Add doxygen style comments
 *******************************************************************************/

/*! \file query.h
    \brief EDR Enhanced Query Code File.

    This defines all the EDR Enhanced Query codes. These codes are used by the EDRE_Query function.
    Include this file to have all the EDR Enhanced Query constants defined.
*/

#ifndef _EDRE_QUERY_H_
#define _EDRE_QUERY_H_

/** @defgroup APICODES API releated query codes
 *  @{
 */
/*!\def APIMAJOR Query the API major version number. */
/*!\def APIMINOR Query the API minor version number. */
/*!\def APIBUILD Query the API build version number. */
/*!\def APIOS Query the operating system platform. */
/*!\def APINUMDEV Query the number of installed or detected devices on the system. */
/*!\def APIERROR Retrieve the last known API or devcie error. (EXPERIMENTAL) */
/*!\def APIOBJECT Get the API main object pointer. */
/*!\def BRDOBJECT Get a specific device's pointer. */
/*!\def APILOADDEV Reload the device list. */
/** @} */

#define APIMAJOR 1
#define APIMINOR 2
#define APIBUILD 3
#define APIOS 4
#define APINUMDEV 5
#define APIERROR 6
#define APIOBJECT 7
#define BRDOBJECT 8
#define APILOADDEV 9


/** @defgroup DEVICECODES Device releated query codes
 *  @{
 */
/*!\def BRDTYPE Query the device type. */
/*!\def BRDREV Query the device revison. */
/*!\def BRDYEAR Query the year the device was manufactured. */
/*!\def BRDMONTH Query the month the device was manufactured. */
/*!\def BRDDAY Query the day the device was manufactured. */
/*!\def BRDSERIALNO Query the device serial number. */
/*!\def BRDBUSTYPE Query the device bus type. */
/*!\def BRDDRVTYPE Query the device driver type. */
/*!\def BRDBASEADDR Query the device base address. Only related to ISA type devices. */
/*!\def BRDINDEX Query the device index in the current device list. */
/** @} */

#define BRDTYPE 10
#define BRDREV 11
#define BRDYEAR 12
#define BRDMONTH 13
#define BRDDAY 14
#define BRDSERIALNO 15
#define BRDBUSTYPE 16
#define BRDDRVTYPE 17
#define BRDBASEADDR 18
#define BRDINDEX 19

/** @defgroup DRIVERCODES Device driver and firmware releated query codes
 *  @{
 */
/*!\def DRVMAJOR Query driver major version number. */
/*!\def DRVMINOR Query driver minor version number. */
/*!\def DRVBUILD Query driver build version number. */
/*!\def FRMMAJOR Query the firmware major version number. Only related to embedded devices. */
/*!\def FRMMINOR Query the firmware minor version number. Only related to embedded devices. */
/*!\def FRMBUILD Query the firmware build version number. Only related to embedded devices. */
/*!\def BRDRESET Reset device. Only device that supports it. */
/** @} */
#define DRVMAJOR 20
#define DRVMINOR 21
#define DRVBUILD 22
#define FRMMAJOR 23
#define FRMMINOR 24
#define FRMBUILD 25
#define BRDRESET 26

/** @defgroup ADCCODES Analog input related query codes
 *  @{
 */
/*!\def ADNUMCHAN Query number of A/D channels. */
/*!\def ADNUMSH Query number of simultaneous sample-and-hold A/D channels. */
/*!\def ADMAXFREQ Query the maximum streaming collective frequency. */
/*!\def ADBUSY Query if the A/D sub-system is busy. Anything other than 0 means busy. */
/*!\def ADFIFOSIZE Query the A/D hardware FIFO size in samples. */
/*!\def ADFIFOOVER Check for FIFO overrun by supported hardware. */
/*!\def ADBUFFSIZE Query the A/D driver/software buffer size in samples. */
/*!\def ADBUFFOVER Check for buffer overrun. A non-zero number means a overrun. */
/*!\def ADBUFFALLOC Check if the A/D buffer was allocated. Zero means no. */
/*!\def ADUNREAD Query the number of unread samples in the A/D buffer. */
/*!\def ADEXTCLK Query the state of the external clock. PCI30FG only. */
/*!\def ADEXTTRIG Query the state of the external trigger line. PCI30FG only. */
/*!\def ADBURST Check for burst mode. PCI30FG only. */
/*!\def ADRANGE Query the current A/D range. PCI30FG only. */
/*!\def ADSETCLKSRC Set the current clock source. Obolete. */
/*!\def ADCLRBUF Clear the A/D buffer. Not supported by all hardware. */
/*!\def ADNUMMA Query the number of mill-amp channels. */
/*!\def ADNUMTMP Query the number of temperature channels. */
/*!\def ADAMBCHAN Query the number of ambient temperature channels. */
/*!\def ADIRQLEVEL Set the FIFO interrupt level. Supported by PCI703, PCI730 & PC104P-30. */
/*!\def ADRELGETDATA Release a EDRE_ADGetData prematurely. */
/*!\def ADNUMRANGES Query the number of A/D voltage ranges. Negative means not supported. \sa ADRANGES */
/*!\def ADRANGES Query each A/D range. Param specify the index. The value is the A/D range in millivolts. \sa ADMODES*/
/*!\def ADNUMMODES Query the number of A/D modes. Negative means not supported. \sa ADMODES ADCMODES */
/*!\def ADMODES Query each A/D mode. Param specify the index. The returned code is an ADCMODE. \sa ADCMODES*/
/*!\def ADNUMGAINS Query the number of A/D gains. Negative means not supported. \sa ADGAINS */
/*!\def ADGAINS Query each A/D gain. Param specify the index. */
/*!\def ADNUMTRIG Query the number of trigger sources. Negative means not supported. \sa ADTRIG ADCTRIGGERS */
/*!\def ADTRIG Query each trigger source. Param specify the index. \sa ADCTRIGGERS */
/*!\def ADNUMTRIGMODE Query the number of trigger modes. Negative means not supported. \sa ADTRIGMODE ADCTRIGMODES */
/*!\def ADTRIGMODE Query each trigger mode. Param specify the index. \sa ADCTRIGMODES */
/*!\def ADNUMCLK Query the number of clock sources. Negative means not supported. \sa ADCLK ADCCLKS */
/*!\def ADCLK Query each clock source. Param specify the index. \sa ADCCLKS */
/*!\def ADNUMGATES Query the number of gate sources. Negative means not supported. \sa ADGATE ADCGATES */
/*!\def ADGATE Query each gate source. Param specify the index. \sa ADCGATES */
/*!\def ADREFGNDPIN Query if the device has a separate selectable analog reference ground pin. 1 = TRUE, 0 = FALSE */
/*!\def ADSETDATAMODE (Internal Linux Object Only) Set the data mode when reading data data from the driver.  */
/*!\def ADDMATEST Run a DMA test on the A/D FIFOs.  */

/** @} */
#define ADNUMCHAN 100
#define ADNUMSH 101
#define ADMAXFREQ 102
#define ADBUSY 103
#define ADFIFOSIZE 104
#define ADFIFOOVER 105
#define ADBUFFSIZE 106
#define ADBUFFOVER 107
#define ADBUFFALLOC 108
#define ADUNREAD 109
#define ADEXTCLK 110
#define ADEXTTRIG 111
#define ADBURST 112
#define ADRANGE 113
#define ADSETCLKSRC 114
#define ADCLRBUF 115
#define ADNUMMA 130
#define ADNUMTMP 140
#define ADAMBCHAN 141
#define ADIRQLEVEL 142
#define ADRELGETDATA 143
#define ADNUMRANGES 144
#define ADRANGES 145
#define ADNUMMODES 146
#define ADMODES 147
#define ADNUMGAINS 148
#define ADGAINS 149
#define ADNUMTRIG 150
#define ADTRIG 151
#define ADNUMTRIGMODE 152
#define ADTRIGMODE 153
#define ADNUMCLK 154
#define ADCLK 155
#define ADNUMGATES 156
#define ADGATE 157
#define ADREFGNDPIN 158
#define ADSETDATAMODE 159
#define ADDMATEST 160

/** @defgroup DACCODES Analog output related query codes
 *  @{
 */
/*!\def DANUMCHAN Query the number of D/A channels. */
/*!\def DAMAXFREQ Query the maximum update rate for waveform generation. */
/*!\def DABUSY Query if D/A sub-system busy. A non-zero value means busy. */
/*!\def DAFIFOSZ Query the D/A FIFO size in samples. Only supported by certain hardware. */
/*!\def DABUFSZ Query the D/A buffer size in samples. Only supported by certain hardware. */
/*!\def DABUFSPACE Query the D/A driver/software buffer size in samples. */
/*!\def DABUFUNDER Check for a buffer underrun. A zero means negative. */
/** @} */
#define DANUMCHAN 200
#define DAMAXFREQ 201
#define DABUSY 202
#define DAFIFOSZ 203
#define DABUFSZ 204
#define DABUFSPACE 205
#define DABUFUNDER 206

/** @defgroup CNTCODES Counter-Timer related query codes
 *  @{
 */
/*!\def CTNUM Query the number of counter-timer channels. */
/*!\def CTBUSY Check if a counter is busy. Param specify the channel. */
/*!\def CTLATCHALL Latch all counters. Supported by 8255 compatible devices.*/
/** @} */
#define CTNUM 300
#define CTBUSY 301
#define CTLATCHALL 302

/** @defgroup DIOCODES Digital I/O related query codes
 *  @{
 */
/*!\def DIONUMPORT Query the number of digital I/O ports. */
/*!\def DIOQRYPORT Query the port type. Param specify the port. \sa DIOPORTCODES */
/*!\def DIOPORTWIDTH Query the port width. Param specify the the port. The width is returned in bits. */
/*!\def DIOCFG Configure an 8255 port. Only supported by certain models. */
/*!\def DIOPULLUPDOWN Program pull-up & pull down resistors \sa DIOPULLCODES. */
/** @} */
#define DIONUMPORT 400
#define DIOQRYPORT 401
#define DIOPORTWIDTH 402
#define DIOCFG 407
#define DIOPULLUPDOWN 408

/** @defgroup DIOPULLCODES Digital I/O related query codes
 *  @{
 */
/*!\def DIOPULLDN Port will be pulled to digital ground via a 100K resistor. */
/*!\def DIOPULLHZ Port will be pulled to 5 volts via a 100K resistor. */
/*!\def DIOPULLHZ Port will be high impedance. */
/** @} */
#define DIOPULLDN 0
#define DIOPULLUP 1
#define DIOPULLHZ 2

/** @defgroup DIOSYNCHROCODES Digital to Synchro and Synchro Resolver related query codes
 *  @{
 */
/*!\def DSRPSU Control the power supply. */
/*!\def DSRPSUERR Query the power supply status. */
/** @} */
#define DSRPSU 450
#define DSRPSUERR 451

/** @defgroup INTCODES Interrupt related query codes
 *  @{
 */
/*!\def INTNUMSRC Query number of interrupt sources. */
/*!\def INTSTATUS Query the interrupt status. */
/*!\def INTBUSCONNECT Connect/disconnect the device to the physical bus. Only supported by certain models. */
/*!\def INTISAVAILABLE Query if interrupt was triggered. */
/*!\def INTNUMTRIG Query number of interrupt that passed through the system since last start. */
/** @} */
#define INTNUMSRC 500
#define INTSTATUS 501
#define INTBUSCONNECT 502
#define INTISAVAILABLE 503
#define INTNUMTRIG 504

/** @defgroup NETCODES Network related query codes
 *  @{
 */
/*!\def NETIPADDRESS Query the system IP address. */
/*!\def NETDISCON Disconnect all client prematurely. */
/*!\def NETPROGOPEN Open connection to update firmware. */
/*!\def NETPROGCLOSE Close programming connection. */
/*!\def NETSETIPADDRESS Set ip address of remote device. */
/*!\def NETSETMASK Set network mask of remote device. */
/*!\def NETSETGATEWAY Set gateway of remote device. */
/** @} */
#define NETIPADDRESS 600
#define NETDISCON 601
#define NETPROGOPEN 602
#define NETPROGCLOSE 603
#define NETSETIPADDRESS 604
#define NETSETMASK 605
#define NETSETGATEWAY 606
#define NETGETMACADDRESS 607

/** @defgroup SERIALCODES Serial communications related query codes
 *  @{
 */
/*!\def SRLGETBAUD Query the device BAUD rate. */
/*!\def SRLSETBAUD Set a new device BAUD rate. */
/*!\def SRLGETSEND Query the number of packets sent on a comm port. Param is the index. */
/*!\def SRLGETRECV Query the number of packets received on a comm port. Param is the index. */
/*!\def SRLGETERRS Query the number of packets errors on a comm port. Param is the index. */
/** @} */
#define SRLGETBAUD 700
#define SRLSETBAUD 701
#define SRLGETSEND 710
#define SRLGETRECV 711
#define SRLGETERRS 712

/** @defgroup ADCMODES A/D mode returned codes
 *  @{
 */
/*!\def UNI_SINGLE Mode is unipolar single ended. */
/*!\def BI_SINGLE Mode is bipolar single ended. */
/*!\def UNI_DIFF Mode is unipolar differential. */
/*!\def BI_DIFF Mode is bipolar differential. */
/** @} */
#define UNI_SINGLE 0
#define BI_SINGLE 1
#define UNI_DIFF 2
#define BI_DIFF 3

/** @defgroup ADCTRIGGERS A/D trigger returned codes
 *  @{
 */
/*!\def AD_TRIG_INTERNAL A/D trigger is internal or always enabled. */
/*!\def AD_TRIG_EXTERNAL A/D trigger is externally controled. */
/*!\def AD_TRIG_REF A/D trigger is controlled by analog reference system. /\sa ADCTRIGMODES*/
/** @} */
#define AD_TRIG_INTERNAL 0
#define AD_TRIG_EXTERNAL 1
#define AD_TRIG_REF 2

/** @defgroup ADCTRIGMODES A/D trigger mode returned codes
 *  @{
 */
/*!\def AD_TRIG_POS A/D trigger on positve signal. */
/*!\def AD_TRIG_NEG A/D trigger on negative signal. */
/*!\def AD_TRIG_RIS A/D trigger on rising edge. */
/*!\def AD_TRIG_FAL A/D trigger on failling edge. */
/** @} */
#define AD_TRIG_POS 0
#define AD_TRIG_NEG 1
#define AD_TRIG_RIS 2
#define AD_TRIG_FAL 3

/** @defgroup ADCCLKS A/D clock source returned codes
 *  @{
 */
/*!\def AD_CLK_INTERNAL Clock source is internal. */
/*!\def AD_CLK_EXTERNAL Clock source is external. */
/*!\def AD_CLK_UNDEFINED Clock source is undefined. PCI703 only */
/** @} */
#define AD_CLK_INTERNAL 0
#define AD_CLK_EXTERNAL 1
#define AD_CLK_UNDEFINED 2

/** @defgroup ADCGATES A/D gate returned codes
 *  @{
 */
/*!\def GATESRC_INTERNAL Gate is enabled. */
/*!\def GATESRC_EXTERNAL Gate is externally controlled. */
/** @} */
#define GATESRC_INTERNAL 0
#define GATESRC_EXTERNAL 1

/** @defgroup DACCLKS D/A clock sources
 *  @{
 */
/*!\def DA_CLK_NONE Clock is disabled. */
/*!\def DA_CLK_INTERNAL1 Clock is internal no 1. */
/*!\def DA_CLK_INTERNAL2 Clock is internal no 2. */
/*!\def DA_CLK_INTERNAL3 Clock is internal no 3. */
/*!\def DA_CLK_INTERNAL4 Clock is internal no 4. */
/*!\def DA_CLK_SOFT Clock is software controlled. */
/*!\def DA_CLK_EXTERNAL  Clock is externally controlled. */
/** @} */
#define DA_CLK_NONE 0
#define DA_CLK_INTERNAL1 1
#define DA_CLK_INTERNAL2 2
#define DA_CLK_INTERNAL3 3
#define DA_CLK_INTERNAL4 4
#define DA_CLK_SOFT 5
#define DA_CLK_EXTERNAL 6

/** @defgroup DACGATES D/A gates
 *  @{
 */
/*!\def DA_GATE_SOFT Gate is software controlled. */
/*!\def DA_GATE_EXT Gate is externally cotnrolled. */
/** @} */
#define DA_GATE_SOFT 0
#define DA_GATE_EXT 1

/** @defgroup DACCOMMANDS D/A commands
 *  @{
 */
/*!\def DA_COMMAND_NULL No command. */
/*!\def DA_COMMAND_START Start operation. */
/*!\def DA_COMMAND_STOP Stop operation. */
/*!\def DA_COMMAND_PAUSE Pause operation. */
/*!\def DA_COMMAND_CONT Continue operation. */
/*!\def DA_COMMAND_CLOCK Software clock once. */
/** @} */
#define DA_COMMAND_NULL 0
#define DA_COMMAND_START 1
#define DA_COMMAND_STOP 2
#define DA_COMMAND_PAUSE 3
#define DA_COMMAND_CONT 4
#define DA_COMMAND_CLOCK 5

/** @defgroup CNTMODES Counter-Timer Modes
 *  @{
 */
/*!\def CT_MODE_0 8254 Mode 0. */
/*!\def CT_MODE_1 8254 Mode 1. */
/*!\def CT_MODE_2 8254 Mode 2. */
/*!\def CT_MODE_3 8254 Mode 3. */
/*!\def CT_MODE_4 8254 Mode 4. */
/*!\def CT_MODE_5 8254 Mode 5. */
/** @} */
#define CT_MODE_0 0
#define CT_MODE_1 1
#define CT_MODE_2 2
#define CT_MODE_3 3
#define CT_MODE_4 4
#define CT_MODE_5 5


/** @defgroup DIOPORTCODES Digital I/O port type return codes
 *  @{
 */
/*!\def DIOOUT Port is output. */
/*!\def DIOIN Port is input. */
/*!\def DIOINOROUT Port is input or output. */
/*!\def DIOINANDOUT Port is input and output. */
/*!\def DIONOTUSED Port is not used. */
/** @} */
#define DIOOUT 0
#define DIOIN 1
#define DIOINOROUT 2
#define DIOINANDOUT 3
#define DIONOTUSED 4

/** @defgroup ADCSTREAMMODE A/D file stream mode types
 *  @{
 */
/*!\def ADC_STREAM_MODE_NEW Overwrite data file. */
/*!\def ADC_STREAM_MODE_APPEND Append data to current file. */
/** @} */
#define ADC_STREAM_MODE_NEW 0
#define ADC_STREAM_MODE_APPEND 1
#define ADC_STREAM_MODE_NUM 2

/** @defgroup DIOCFG Digital I/O configuration constants
 *  @{
 */
/*!\def CFGDIOOUT Configure port as output. */
/*!\def CFGDIOIN Configure port as input. */
/*!\def CFGDIOINANDOUT Configure port as in and out. This is possible on port C of the 8255 compatible devices. */
/*!\def CFGDIOAUTO Enable automatic configuration. */
/** @} */
#define CFGDIOOUT 0
#define CFGDIOIN 1
#define CFGDIOINANDOUT 2
#define CFGDIOAUTO 3

/** @defgroup ADCSETDATAMODE A/D data mode
 * @{
 */
/*!\def ADC_DATA_MODE_RAW A/D data mode is raw. */
/*!\def ADC_DATA_MODE_VOLT A/D data mode is volts. */
/** @} */
#define ADC_DATA_MODE_RAW 0
#define ADC_DATA_MODE_VOLT 1

/** @defgroup TCTYPES Thermocouple types
 * @{
 */
/*!\def TC_TYPE_J Thermocouple type J. */
/*!\def TC_TYPE_K Thermocouple type K. */
/*!\def TC_TYPE_E Thermocouple type E. */
/*!\def TC_TYPE_T Thermocouple type T. */
/*!\def TC_TYPE_S Thermocouple type S. */
/*!\def TC_TYPE_R Thermocouple type R. */
/*!\def TC_TYPE_B Thermocouple type B. */
/*!\def TC_TYPE_N Thermocouple type N. */
/*!\def TC_TYPE_C Thermocouple type C. */

/** @} */
#define TC_TYPE_J 0
#define TC_TYPE_K 1
#define TC_TYPE_E 2
#define TC_TYPE_T 3
#define TC_TYPE_S 4
#define TC_TYPE_R 5
#define TC_TYPE_B 6
#define TC_TYPE_N 7
#define TC_TYPE_C 8

#endif /* _EDRE_QUERY_H_ */
