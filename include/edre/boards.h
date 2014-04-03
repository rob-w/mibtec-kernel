/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2003 - 2009 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * boards.h -- EDR Enhanced Device Definition File
 *
 *******************************************************************************
 * 2005-08-19  Jan Zwiegers  <jan@eage.co.za>
 * - Add Ethernet family of devices
 *
 * 2005-07-28  Jan Zwiegers  <jan@eage.co.za>
 * - Add uDAQlite device
 *
 * 2005-02-24  Jan Zwiegers  <jan@eage.co.za>
 * - Add/Change new USB devices
 * - Add/Change new serial devices
 * - Change file structure
 * - Add doxygen style comments 
 *******************************************************************************/

/*! \file boards.h
    \brief EDR Enhanced Device Definition File

    This define all the devices supported by EDR Enhanced.
*/

#ifndef __EDRE_BOARDS__
#define __EDRE_BOARDS__

/** @defgroup PCIBOARDS EDR Enhanced PCI Boards
 *  @{
 */
#define NOBOARD 0
#define PCI36C 1
#define PCI36C_ADV 2
#define PCI63C16 3
#define PCI63C32 4
#define PCI62C16 5
#define PCI62C32 6
#define PCI62C48 7
#define PCI773T16 10
#define PCI773R16 11
#define PCI30G 20
#define PCI30GA 21
#define PCI30G32 22
#define PCI30GA32 23
#define PCI30GS4 24
#define PCI30GS16 25
#define PCI30GAS4 26
#define PCI30GAS16 27
#define PCI30F 28
#define PCI30FA 29
#define PCI30F32 30
#define PCI30FA32 31
#define PCI30FS4 32
#define PCI30FS16 33
#define PCI30FAS4 34
#define PCI30FAS16 35
#define PCI26 36
#define PCI725 50
#define PCI726 51
#define PCI730 52
#define PCI14B 60
#define PCI14C 61
#define PCI762_16 62
#define PCI762_32 63
#define PCI762_48 64
#define PCI763_16 65
#define PCI763_32 66
#define PCI769 67
#define PCI766A8 70
#define PCI766A16 71
#define PCI766A24 72
#define PCI70316 80
#define PCI70316A 81
#define PCI70364A 82
#define PCI70364 83
#define PCI703S8 84
#define PCI703S8A 85
#define PCI703S16 86
#define PCI703S16A 87
#define PCI70332 88
#define PCI70332A 89
/** @} */


#ifndef __EDR__

/** @defgroup PC104BOARDS EDR Enhanced PC104 Boards
 *  @{
 */
#define PC104_30G 90
#define PC104_30GA 91
#define PC104_30F 92
#define PC104_30FA 93
#define PC104_72A 94
#define PC104_30H 99
/** @} */


/** @defgroup ISABOARDS EDR Enhanced ISA Boards
 *  @{
 */
#define PC36C 100
#define PC14B 101
#define PC62C 102
#define PC192A 103
#define PC166 104
#define PC61 105
#define PC60 106
#define PC40 107
#define PC58 108
#define PC73C 109
#define PCDEMO 150
/** @} */

/** @defgroup PC104PBOARDS EDR Enhanced PC104PLUS Boards
 *  @{
 */
#define PC104PLUS_OFFSET 160
#define PC104P69 160
#define PC104P25 170
#define PC104P26 171
#define PC104P30 172
/** @} */

#endif /* END EDR */

/** @defgroup PCI800BOARDS EDR Enhanced PCI800 Family Boards
 *  @{
 */
#define PCI800_Offset 200
#define PCI836A 201
#define PCI836C 202
#define PCI848A 210
#define PCI848C 211
#define PCI896A 220
#define PCI896C 221
#define PCI8192A 230
#define PCI8192C 231
#define PCBOARD_ERR 232
#define EDRE_NumOfBoards 233
/** @} */

/** @defgroup PCIe800BOARDS EDR Enhanced PCIe800 Family Boards
 *  @{
 */
#define PCIe800_Offset 300
#define PCIe836A 301
#define PCIe836C 302
#define PCIe848A 310
#define PCIe848C 311
#define PCIe896A 320
#define PCIe896C 321
#define PCIe8192A 330
#define PCIe8192C 331
#define PCIe_NumOfBoards 32
/** @} */

/** @defgroup SRLDEVICES EDR Enhanced Serial Family of Devices
 *  @{
 */
#define EDRE_SRL_DEVOFFSET 0x1000
#define SRL24A 0x1001
#define SRL24C 0x1002
#define SRL48A 0x1003
#define SRL48C 0x1004
#define SRL72A 0x1005
#define SRL72C 0x1006
#define SRL96A 0x1007
#define SRL96C 0x1008
#define SRL120A 0x1009
#define SRL120C 0x100A
#define SRL62_16 0x100B
#define SRL62_32 0x100C
#define SRL63_16 0x100D
#define SRL63_32 0x100E
#define SRL69_16 0x100F
#define SRL69_32 0x1010
#define SRL26 0x1013
#define SRL26A16 0x1013
#define SRL26A32 0x1014
#define SRL30 0x1015
#define SRL30A16 0x1015
#define SRL30A32 0x1016
#define SRL26C16 0x101B
#define SRL26C32 0x101C
#define SRL30C16 0x101D
#define SRL30C32 0x101E
#define SRL73T8 0x1020
#define SRL73T16 0x1021
#define SRL73T32 0x1022
#define SRL73R8 0x1023
#define SRL73R16 0x1024
#define SRL73R32 0x1025
#define SRLRUGDAQ 0x1030
#define EDRE_NumOfSerialDevices 49
/** @} */


/** @defgroup USBDEVICES EDR Enhanced Universal Serial Bus Family of Devices
 *  @{
 */
#define EDRE_USB_DEVOFFSET 0x2000
#define USB24A 0x2001
#define USB24C 0x2002
#define USB48A 0x2003
#define USB48C 0x2004
#define USB72A 0x2005
#define USB72C 0x2006
#define USB96A 0x2007
#define USB96C 0x2008
#define USB120A 0x2009
#define USB120C 0x200A
#define USB62_16 0x200B
#define USB62_32 0x200C
#define USB63_16 0x200D
#define USB63_32 0x200E
#define USB69_16 0x200F
#define USB69_32 0x2010
#define USB26 0x2013
#define USB26A16 0x2013
#define USB26A32 0x2014
#define USB30 0x2015
#define USB30A16 0x2015
#define USB30A32 0x2016
#define USB26B16 0x2017
#define USB26B32 0x2018
#define USB30B16 0x2019
#define USB30B32 0x201A
#define USB26C16 0x201B
#define USB26C32 0x201C
#define USB30C16 0x201D
#define USB30C32 0x201E
#define USB73T8 0x2020
#define USB73T16 0x2021
#define USB73T32 0x2022
#define USB73R8 0x2023
#define USB73R16 0x2024
#define USB73R32 0x2025

#define USB26D16 0x2027
#define USB26D32 0x2028
#define USB30D16 0x2029
#define USB30D32 0x202A

#define USBRUGDAQ 0x2030
#define UDAQLITE 0x2100
#define EDRE_NumOfUSBDevices 257
/** @} */

/** @defgroup NETDEVICES EDR Enhanced Ethernet Family of Devices
 *  @{
 */
#define EDRE_NET_DEVOFFSET 0x3000
#define NET24A 0x3001
#define NET24C 0x3002
#define NET48A 0x3003
#define NET48C 0x3004
#define NET72A 0x3005
#define NET72C 0x3006
#define NET96A 0x3007
#define NET96C 0x3008
#define NET120A 0x3009
#define NET120C 0x300A
#define NET62_16 0x300B
#define NET62_32 0x300C
#define NET63_16 0x300D
#define NET63_32 0x300E
#define NET69_16 0x300F
#define NET69_32 0x3010
#define NET26 0x3013
#define NET26A16 0x3013
#define NET26A32 0x3014
#define NET30 0x3015
#define NET30A16 0x3015
#define NET30A32 0x3016
#define NET26B16 0x3017
#define NET26B32 0x3018
#define NET30B16 0x3019
#define NET30B32 0x301A
#define NET26C16 0x301B
#define NET26C32 0x301C
#define NET30C16 0x301D
#define NET30C32 0x301E
#define NET73T8 0x3020
#define NET73T16 0x3021
#define NET73T32 0x3022
#define NET73R8 0x3023
#define NET73R16 0x3024
#define NET73R32 0x3025
#define NET30C16_73R16 0x302C
#define NET30C32_73R16 0x302D
#define NET30C16_73T16 0x302E
#define NET30C32_73T16 0x302F
#define NETRUGDAQ 0x3030
#define EDRE_NumOfNETDevices 49
/** @} */

/** @defgroup SERIALOFFSETS Default serial number offset values
 *  @{
 */
#define PCI_SN_OFFSET 1000000000
#define ISA_SN_OFFSET 100000000
#define PC104_SN_OFFSET 10000000
/** @} */

/** @defgroup DEVICOFFSETS Default devices offsets for devices without eeproms. We use this to simulate a serial number.
 *  @{
 */
#define ISA_PCDEMO_OFFSET 100
#define ISA_PC14B_OFFSET 200
#define ISA_PC36C_OFFSET 300
#define ISA_PC192A_OFFSET 400
#define ISA_PC60_OFFSET 420
#define ISA_PC61_OFFSET 440
#define ISA_PC40_OFFSET 460
#define ISA_PC58_OFFSET 480
#define ISA_PC73C_OFFSET 500
#define ISA_PC10430FG_OFFSET 600
#define ISA_PC10472A_OFFSET 700
/** @} */

/** @defgroup MAXNUMDEVICES Maximum Installed Devices - Obsolete.
 *  @{
 */
#define DIO_ISA_NumOfDev 10
#define DA_ISA_NumOfDev 10
#define AD_ISA_NumOfDev 10
#define DIO_PCI_NumOfDev 4
#define DA_PCI_NumOfDev 4
#define AD_PCI_NumOfDev 4
#define DIO_104_NumOfDev 10
#define DA_104_NumOfDev 4
#define AD_104_NumOfDev 4
/** @} */

/** @defgroup BUSARCHITECTURE EDR Enhanced Bus Architectures.
 *  @{
 */
#define PC_BUS_NONE 0
#define PC_BUS_ISA 1
#define PC_BUS_PCI 2
#define PC_BUS_104 3
#define PC_BUS_USB 4
#define PC_BUS_SER 5
#define PC_BUS_ETH 6
#define PC_BUS_WRL 7
/** @} */

#endif /* __EDRE_BOARDS__ */

/**
 * End of File
 */
