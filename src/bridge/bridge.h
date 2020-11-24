/**
  ******************************************************************************
  * @file    bridge.h
  * @author  MCD Application Team
  * @brief   Header for bridge.cpp module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/** @addtogroup BRIDGE
 * @{
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BRIDGE_H
#define _BRIDGE_H
/* Includes ------------------------------------------------------------------*/
#include "stlink_device.h"
#include "stlink_fw_const_bridge.h"


/* Exported types and constants ----------------------------------------------*/
/** @addtogroup GENERAL
 * @{
 */
/// Bridge Error and Status
typedef enum {
	BRG_NO_ERR = 0,           ///< OK (no error)
	BRG_CONNECT_ERR,          ///< USB Connection error
	BRG_DLL_ERR,              ///< USB DLL error
	BRG_USB_COMM_ERR,         ///< USB Communication error
	BRG_NO_DEVICE,            ///< No Bridge device target found error
	BRG_OLD_FIRMWARE_WARNING, ///< Warning: current bridge firmware is not the last one available
	BRG_TARGET_CMD_ERR,       ///< Target communication or command error
	BRG_PARAM_ERR,            ///< Wrong parameters error
	BRG_CMD_NOT_SUPPORTED,    ///< Firmware command not supported by the current firmware version
	BRG_GET_INFO_ERR,         ///< Error getting STLink Bridge device information
	BRG_STLINK_SN_NOT_FOUND,  ///< Required STLink serial number not found error
	BRG_NO_STLINK,            ///< STLink Bridge device not opened error
	BRG_NOT_SUPPORTED,        ///< Parameter error
	BRG_PERMISSION_ERR,       ///< STLink Bridge device already in use by another program error
	BRG_ENUM_ERR,             ///< USB enumeration error
	BRG_COM_FREQ_MODIFIED,    ///< Warning: required frequency is not exactely the one applied
	BRG_COM_FREQ_NOT_SUPPORTED,///< Required frequency cannot be applied error
	BRG_SPI_ERR,              ///< SPI communication error
	BRG_I2C_ERR,              ///< I2C communication error
	BRG_CAN_ERR,              ///< CAN communication error
	BRG_TARGET_CMD_TIMEOUT,   ///< Timeout error during Bridge communication
	BRG_COM_INIT_NOT_DONE,    ///< Bridge Init function not called error
	BRG_COM_CMD_ORDER_ERR,    ///< Sequencial Bridge function order error
	BRG_BL_NACK_ERR,          ///< Bootloader NACK error
	BRG_VERIF_ERR,            ///< Data verification error
	BRG_MEM_ALLOC_ERR,        ///< Memory allocation error
	BRG_GPIO_ERR,             ///< GPIO communication error
	BRG_OVERRUN_ERR,          ///< Overrun error during bridge communication
	BRG_CMD_BUSY,             ///< Command busy: only Brg::GetLastReadWriteStatus() command allowed in that case
	BRG_CLOSE_ERR,            ///< Error during device Close
	BRG_INTERFACE_ERR         ///< Unknown default error returned by STLinkInterface
} Brg_StatusT;

#define COM_SPI STLINK_SPI_COM   ///< 0x2 SPI Bridge communication parameter
#define COM_I2C STLINK_I2C_COM   ///< 0x3 I2C Bridge communication parameter
#define COM_CAN STLINK_CAN_COM   ///< 0x4 CAN Bridge communication parameter
#define COM_GPIO STLINK_GPIO_COM ///< 0x6 GPIO Bridge communication parameter
#define COM_UNDEF_ALL 0xFF       ///< 0xFF All or Undefined Bridge communication parameter

#define DEFAULT_CMD_TIMEOUT 0  ///< 0x0 Parameter to use default firmware timeout
// end group doxygen GENERAL
/** @} */
// -------------------------------- SPI ------------------------------------ //
/** @addtogroup SPI
 * @{
 */
/// Add or not a SPI delay between bytes (or words)
typedef enum {
	DEFAULT_NO_DELAY = 0, ///< Default case: no delay is inserted
	DELAY_FEW_MICROSEC = 1  ///< At least 4us delay is inserted
} Brg_DelayT;

// SPI Init parameters, see InitSPI() for the currently supported SPI mode
/// SPI Init: lines connection configuration see also \ref LIMITATION
typedef enum {
	SPI_DIRECTION_2LINES_FULLDUPLEX = 0, ///< Fullduplex: master and slave MISO + MOSI used
	SPI_DIRECTION_2LINES_RXONLY = 1, ///< Halfduplex: master MOSI and slave MISO used
	SPI_DIRECTION_1LINE_RX = 2, ///< Simplex: master and slave MISO used
	SPI_DIRECTION_1LINE_TX = 3  ///< Simplex: master and slave MOSI used
} Brg_SpiDirT;
/// SPI Init: mode configuration see also \ref LIMITATION
typedef enum {
	SPI_MODE_SLAVE = 0, ///< SPI Slave mode
	SPI_MODE_MASTER = 1 ///< SPI master mode
} Brg_SpiModeT;
/// SPI Init: data size configuration
typedef enum {
	SPI_DATASIZE_16B = 0, ///< 16 Bit data word
	SPI_DATASIZE_8B = 1 ///< 8 Bit data byte
} Brg_SpiDataSizeT;
/// SPI Init: SCK polarity configuration (see also #Brg_SpiCphaT)
typedef enum {
	SPI_CPOL_LOW = 0, ///< SCK low polarity
	SPI_CPOL_HIGH = 1 ///< SCK High polarity
} Brg_SpiCpolT;
/// SPI Init: SCK phase configuration (see also #Brg_SpiCpolT)
typedef enum {
	SPI_CPHA_1EDGE = 0, ///< SCK 1st edge: data captured on 1st CLK edge (rising if CPOL low or falling if CPOL high),
						///<                data output on 2nd edge (falling if CPOL low)
	SPI_CPHA_2EDGE = 1  ///< SCK 2nd edge: data captured on 2nd CLK edge, data output on 3rd edge
} Brg_SpiCphaT;
/// SPI Init: data bit transmission order configuration
typedef enum {
	SPI_FIRSTBIT_LSB = 0, ///< LSB is output first
	SPI_FIRSTBIT_MSB = 1 ///< MSB is output first
} Brg_SpiFirstBitT;
/// SPI Init: frame format configuration see also \ref LIMITATION
typedef enum {
	SPI_FRF_MOTOROLA = 0, ///< Motorola format (default)
	SPI_FRF_TI = 1 ///< TI format, CPHA, CPOL, NSS are forced to TI mode (whatever their value)
} Brg_SpiFrfT;
/// SPI Init: NSS (Slave Select) management
typedef enum {
	SPI_NSS_SOFT = 0, ///< NSS manage by software (see Brg::SetSPIpinCS())
	SPI_NSS_HARD = 1 ///< NSS manage by hardware
} Brg_SpiNssT;
/// SPI Init: NSS configuration for SPI_NSS_HARD case in master mode
typedef enum {
	SPI_NSS_NO_PULSE = 0, ///< SPI NSS not pulsed between each data
	SPI_NSS_PULSE = 1 ///< SPI NSS pulse generated between 2 data
} Brg_SpiNssPulseT;
/// Brg::SetSPIpinCS() parameter: SPI NSS level for #SPI_NSS_SOFT case in master mode
typedef enum {
	SPI_NSS_LOW = 0, ///< Set SPI NSS low
	SPI_NSS_HIGH = 1 ///< Set SPI NSS high
} Brg_SpiNssLevelT;
/// SPI Init: baudrate, SPI IP input CLK prescaler, use Brg::GetSPIbaudratePrescal() 
/// to get the correct prescaler according to required SCK frequency.
typedef enum {
	SPI_BAUDRATEPRESCALER_2 = 0,  ///< Prescaler = 2
	SPI_BAUDRATEPRESCALER_4 = 1,  ///< Prescaler = 4
	SPI_BAUDRATEPRESCALER_8 = 2,  ///< Prescaler = 8
	SPI_BAUDRATEPRESCALER_16 = 3, ///< Prescaler = 16
	SPI_BAUDRATEPRESCALER_32 = 4, ///< Prescaler = 32
	SPI_BAUDRATEPRESCALER_64 = 5, ///< Prescaler = 64
	SPI_BAUDRATEPRESCALER_128 = 6,///< Prescaler = 128
	SPI_BAUDRATEPRESCALER_256 = 7 ///< Prescaler = 256
} Brg_SpiBaudrateT;
/// SPI Init: CRC configuration see also \ref LIMITATION
typedef enum {
	SPI_CRC_DISABLE = 0, ///< CRC disabled (default)
	SPI_CRC_ENABLE = 1 ///< CRC enabled
} Brg_SpiCrcT;
/// SPI Init parameter structure, see also \ref LIMITATION
typedef struct {
	Brg_SpiDirT Direction;      ///< Data Line direction
	Brg_SpiModeT Mode;          ///< SPI mode
	Brg_SpiDataSizeT DataSize;  ///< Data size
	Brg_SpiCpolT Cpol;          ///< SCK Polarity
	Brg_SpiCphaT Cpha;          ///< SCK Phase
	Brg_SpiFirstBitT FirstBit;  ///< Data bit transmission order
	Brg_SpiFrfT FrameFormat;    ///< Frame format
	Brg_SpiNssT Nss;            ///< NSS configuration
	Brg_SpiNssPulseT NssPulse;  ///< NSS hardware pulse configuration
	Brg_SpiBaudrateT Baudrate;  ///< Prescaler for SCK
	Brg_SpiCrcT Crc;            ///< CRC configuration
	uint16_t CrcPoly;           ///< 0 or if #SPI_CRC_ENABLE: Odd value only
	Brg_DelayT SpiDelay;
	///< SpiDelay to be used to insert a delay of several microseconds between bytes (or words) 
	///< when calling Read/Write functions.\n
	///< Note for smaller delay: NSS mode hardware pulse insert one SPI CLK cycle (when NSS is pulsed)\n
	///< Note for larger delay: call N time Read/Write function transmitting 1 byte instead of 1 time 
	///<                        Read/write function transmitting N bytes (~200us delay)
} Brg_SpiInitT;
// end group doxygen SPI
/** @} */
// -------------------------------- I2C ------------------------------------ //
/** @addtogroup I2C
 * @{
 */
// I2C Init parameters
/// I2C Init: enable or disable filter configuration
typedef enum {
	I2C_FILTER_DISABLE = 0, ///< Filter disabled
	I2C_FILTER_ENABLE = 1   ///< Filter enabled
} Brg_I2cFilterT;

/// AddressingMode : for slave address for master and Own address for slave mode, see also \ref LIMITATION
typedef enum {
	I2C_ADDR_7BIT = 0, ///< 7 bit addressing
	I2C_ADDR_10BIT = 1 ///< 10 bit addressing
} Brg_I2cAddrModeT;
/// I2C Init parameter structure, see also \ref LIMITATION
typedef struct {
	uint32_t TimingReg;        ///< Call Brg::GetI2cTiming() to get TimingReg parameter according to required
						       ///< I2C speed and time configuration parameters.
	uint16_t  OwnAddr;         ///< OwnAddress1 <= 0X3FF (for slave mode)
	Brg_I2cAddrModeT AddrMode; ///< Address mode (for slave mode)
	Brg_I2cFilterT AnFilterEn; ///< Enable or disable Analog filter
	Brg_I2cFilterT DigitalFilterEn; ///< Enable or disable Digital filter
	uint8_t Dnf;               ///< Digital filter configuration: DNF (<=15) or 0 if digital filter OFF
} Brg_I2cInitT;
/// I2C Init: I2C mode parameter for Brg::GetI2cTiming()
typedef enum {
	I2C_STANDARD,   ///< I2C standard mode, 1-100KHz
	I2C_FAST,       ///< I2C fast mode, 1-400KHz
	I2C_FAST_PLUS   ///< I2C fast mode plus, 1-1000KHz
} I2cModeT;

#define I2C_10B_ADDR_FLAG  0x8000   ///< I2C 10bit addressing flag
#define I2C_10B_ADDR(_addr) (_addr|I2C_10B_ADDR_FLAG) ///< I2C 10bit addressing macro
#define I2C_7B_ADDR(_addr) (_addr)  ///< I2C 7bit addressing macro

// Private low level struct for partial transfer management
typedef enum {
	I2C_FULL_RW_TRANS = 0, // Default do full I2C transaction: START-slave ADDR-DATA-STOP
	// Partial I2C transaction: user needs to call Start, Cont (optional) and Stop in this order
	I2C_START_RW_TRANS = 1, // Do partial I2C transaction: START-slave ADDR-DATA 
	I2C_CONT_RW_TRANS = 2, // Do partial I2C transaction: DATA 
	I2C_STOP_RW_TRANS = 3 // Do partial I2C transaction: DATA-STOP
} Brg_I2cRWTransfer; // Used in case I2C transfer is split over several RW cmd

typedef enum {
	I2C_TRANS_IDLE, // Default no partial transaction ongoing
	// Partial I2C transaction: user needs to call Start, Cont (optional) and Stop in this order
	I2C_TRANS_R_ONGOING, // Partial read I2C transaction ongoing
	I2C_TRANS_W_ONGOING // Partial write I2C transaction ongoing
} Brg_I2cPartialTransT;
// end group doxygen I2C
/** @} */
// -------------------------------- CAN ------------------------------------ //
/** @addtogroup CAN
 * @{
 */
/// CAN Init: Initialization type
typedef enum {
	BRG_INIT_FULL, ///< Default: complete IP initialization (Filter reseted)
	BRG_REINIT,    ///< Reinitialization (only modify configuration parameters)
} Brg_InitTypeT;

/// CAN Init: CAN Bit Time configuration used in Brg::GetCANbaudratePrescal() and InitCAN()
typedef struct {
	uint8_t PropSegInTq;   ///< 1 to 8 time quantum
	uint8_t PhaseSeg1InTq; ///< 1 to 8 time quantum
	uint8_t PhaseSeg2InTq; ///< 1 to 8 time quantum
	uint8_t SjwInTq;       ///< 1 to 4 time quantum (may be less than PhaseSeg1InTq and PhaseSeg2InTq)
} Brg_CanBitTimeConfT;

/// CAN Init: possible CAN mode parameter for Brg::InitCAN(), see also \ref LIMITATION .\n
/// #CAN_MODE_NORMAL is the mode to be used by default.\n\n
///
///	In Loop Back Mode, the CAN Bridge treats its own transmitted messages as received
///	messages and stores them (if they pass acceptance filtering) in a Receive mailbox.
///	This mode is provided for self-test functions. To be independent of external events, the CAN
///	Core ignores acknowledge errors (no dominant bit sampled in the acknowledge slot of a
///	data / remote frame) in Loop Back Mode. In this mode, the CAN performs an internal
///	feedback from its Tx output to its Rx input. The actual value of the CANRX input pin is
///	disregarded by the bxCAN. The transmitted messages can be monitored on the CANTX pin.\n\n
///
///	In Silent mode, the CAN Bridge is able to receive valid data frames and valid remote frames, but
///	it sends only recessive bits on the CAN bus and it cannot start a transmission. If the CAN
///	has to send a dominant bit (ACK bit, overload flag, active error flag), the bit is rerouted
///	internally so that the CAN Core monitors this dominant bit, although the CAN bus may
///	remain in recessive state. Silent mode can be used to analyze the traffic on a CAN bus
///	without affecting it by the transmission of dominant bits (Acknowledge Bits, Error Frames).
typedef enum {
	CAN_MODE_NORMAL = 0,         ///< Normal mode
	CAN_MODE_LOOPBACK = 1,       ///< Loopback mode
	CAN_MODE_SILENT = 2,         ///< Silent mode
	CAN_MODE_SILENT_LOOPBACK = 3 ///< Silent Loopback mode
} Brg_CanModeT;

/// CAN init parameters for InitCAN(), see also \ref LIMITATION
typedef struct {
	Brg_CanBitTimeConfT BitTimeConf; ///< Bit time management
	Brg_CanModeT Mode;               ///< CAN mode (normal, loopback ...)
	uint32_t Prescaler;              ///< 1 to 1024 (use Brg::GetCANbaudratePrescal())
	bool bIsAbomEn;                  ///< Enable or disable automatic bus-off management
	bool bIsAwumEn;                  ///< Enable or disable automatic wake-up mode
	bool bIsNartEn;                  ///< Enable or disable no-automatic retransmission mode
	bool bIsRflmEn;                  ///< Enable or disable Receive FIFO Locked mode
	bool bIsTxfpEn;                  ///< Enable or disable transmit FIFO priority
} Brg_CanInitT;

/// CAN message format types
typedef enum {
	CAN_ID_STANDARD = 0, ///< Standard CAN message
	CAN_ID_EXTENDED = 1  ///< Extended CAN message
} Brg_CanMsgIdT;
/// CAN message data types, see also \ref LIMITATION
typedef enum {
	CAN_DATA_FRAME = 0, ///< Data (default)
	CAN_REMOTE_FRAME = 1 ///< Remote data
} Brg_CanMsgRtrT;
/// CAN receive fifo
typedef enum {
	CAN_MSG_RX_FIFO0 = 0, ///< Received CAN message in FIFO0
	CAN_MSG_RX_FIFO1 = 1  ///< Received CAN message in FIFO1
} Brg_CanRxFifoT;
/// CAN Rx message overrun status
typedef enum {
	CAN_RX_NO_OVERRUN = 0,   ///< No error
	CAN_RX_FIFO_OVERRUN = 1, ///< STLink CAN HW fifo overrun
	CAN_RX_BUFF_OVERRUN = 2  ///< STLink CAN Rx buffer overrun
} Brg_CanRxOverrunT;

/// Structure for Rx CAN messages (except data field), see also \ref LIMITATION
typedef struct {
	Brg_CanMsgIdT IDE;  ///< Specifies if ID is standard (11-bit) or extended (29-bit) identifier.
	uint32_t ID;        ///< Identifier of the message (11bit or 29bit according to IDE).
	Brg_CanMsgRtrT RTR; ///< Remote Frame Request or data frame message type.
	uint8_t DLC;        ///< Data Length Code is the number of data bytes in the received message 
	                    ///< or number of data bytes requested by RTR.
	Brg_CanRxFifoT Fifo; ///< Fifo in which the message was received (according to Filter initialization done
	                     ///< with Brg::InitFilterCAN(): AssignedFifo in #Brg_CanFilterConfT)
	Brg_CanRxOverrunT Overrun; ///< Indicate if overrun has occurred before this message.
	uint16_t TimeStamp;  ///< unused
} Brg_CanRxMsgT;

/// Structure for Tx CAN messages (except data field), see also \ref LIMITATION
typedef struct {
	Brg_CanMsgIdT IDE; ///< Specifies if ID is standard (11-bit) or extended (29-bit) identifier.
	uint32_t ID;       ///< Identifier of the message (11bit or 29bit according to IDE).
	                   ///< DLC: max 8. Data Length Code of requested data bytes when sending RTR .
	Brg_CanMsgRtrT RTR; ///< RTR: Specifies if Remote Transmission Request should be sent (DLC is used for
	                    ///< number of requested bytes), otherwise the data message will be sent.
	uint8_t DLC;        ///< Data Length Code, max 8. Number of requested data for RTR, unused for data Frame. 
	                    ///< (for data frame Size parameter of Brg::WriteMsgCAN() is used as DLC)
} Brg_CanTxMsgT;

/// Filter mode \n
/// In mask mode the identifier is associated with mask to specify which
/// bits of the identifier are handled as "must match" or as "don't care".
/// i.e.: message is accepted if ID_msg & MASK_filter = ID_filter.\n
/// In identifier list mode, instead of defining an identifier and a mask, 
/// two identifiers are specified, doubling the number of single identifiers.
/// All bits of the incoming identifier must match the bits specified in the filter.
/// i.e.: message is accepted if ID_msg = ID_filter.
typedef enum {
	CAN_FILTER_ID_MASK = 0, ///< Mask mode
	CAN_FILTER_ID_LIST = 1  ///< Identifier list mode
}Brg_CanFilterModeT;

/// Filter scale \n
/// To optimize and adapt the filters to the application needs, each filter bank
/// can be scaled independently. Depending on the filter scale a filter bank provides:\n
/// One 32-bit filter for the STDID[10:0], EXTID[17:0], IDE and RTR bits.\n
/// Two 16-bit filters for the STDID[10:0], RTR, IDE and EXTID[17:15] bits.
typedef enum {
	CAN_FILTER_16BIT = 0, ///< 16 bit filter
	CAN_FILTER_32BIT = 1  ///< 32 bit filter
}Brg_CanFilterScaleT;

/// Filter identifier or mask
typedef struct {
	Brg_CanMsgRtrT RTR; ///< Data type
	Brg_CanMsgIdT IDE;  ///< ID Format type
	uint32_t ID;        ///< Standard (max 0x7FF) or extended ID (max 0x1FFFFFFF)
}Brg_FilterBitsT;

/// Structure used to configure a given CAN filter with Brg::InitFilterCAN().\n
/// For 16-bit filters and extended ID, only ID[17:15,10:0] bits are used
/// in #Brg_FilterBitsT fields (ID[28-18,14:11] are unused).
/// According to FilterMode and FilterScale some #Brg_FilterBitsT fields
/// are used or not.\n See table below (x = used, - = unused)\n
///     <table>
///     <tr><th>Scale              <th>Mode    <th>Id[0] <th>Mask[0] <th>Id[1] <th>Mask[1] <th>Id[2] <th>Id[3]
///     <tr><td rowspan="2">32bit  <td>ID_MASK <td>x     <td>x       <td>-     <td>-       <td>-     <td>-
///     <tr>                       <td>ID_LIST <td>x     <td>-       <td>x     <td>-       <td>-     <td>-
///     <tr><td rowspan="2">16bit  <td>ID_MASK <td>x     <td>x       <td>x     <td>x       <td>-     <td>-
///     <tr>                       <td>ID_LIST <td>x     <td>-       <td>x     <td>-       <td>x     <td>x
///     </table>
// -------------------------------------------------------------------
// | Scale |  Mode   | Id[0] |Mask[0]| Id[1] |Mask[1]| Id[2] | Id[3] |
// -------------------------------------------------------------------
// |       | ID_MASK |   x   |   x   |   -   |   -   |   -   |   -   |
// | 32bit |----------------------------------------------------------
// |       | ID_LIST |   x   |   -   |   x   |   -   |   -   |   -   |
// -------------------------------------------------------------------
// |       | ID_MASK |   x   |   x   |   x   |   x   |   -   |   -   |
// | 16bit |----------------------------------------------------------
// |       | ID_LIST |   x   |   -   |   x   |   -   |   x   |   x   |
// -------------------------------------------------------------------
typedef struct {
	uint8_t FilterBankNb;    ///< Filter bank to configure: 0 to 13
	bool bIsFilterEn;        ///< FilterActivation: Enable or disable the filter
	Brg_CanFilterModeT FilterMode;   ///< ID_LIST or ID_MASK
	Brg_CanFilterScaleT FilterScale; ///< 16bit or 32bit
	Brg_FilterBitsT Id[4];   ///< Identifiers, Id[0] used in all cases,
	                         ///< Id[1] used only if 16bit (LIST or MASK) or 32bit and ID_LIST,
	                         ///< Id[2] used only if 16bit and ID_LIST,
	                         ///< Id[3] used only if 16bit and ID_LIST.
	Brg_FilterBitsT Mask[2]; ///< Masks, Mask[0] used only if ID_MASK (16bit or 32bit)
	                         ///< Mask[1] used only if 16bit and ID_MASK.
	Brg_CanRxFifoT AssignedFifo;     ///< Rx FIFO in which message is received
}Brg_CanFilterConfT;
/** @} */
// -------------------------------- GPIO ----------------------------------- //
/** @addtogroup GPIO
 * @{
 */
/// GPIO mask
typedef enum {
	BRG_GPIO_0 = 0x01,  ///< GPIO Bridge I00
	BRG_GPIO_1 = 0x02,  ///< GPIO Bridge I01
	BRG_GPIO_2 = 0x04,  ///< GPIO Bridge I02
	BRG_GPIO_3 = 0x08,  ///< GPIO Bridge I03
	BRG_GPIO_ALL = 0x0F ///< All GPIO Bridge: I00 I01 I02 I03
}Brg_GpioMaskT;

#define BRG_GPIO_MAX_NB 4  ///< Max number of used GPIO Bridge

/// GPIO port mode configuration, see also \ref LIMITATION
typedef enum {
	GPIO_MODE_INPUT = 0,  ///< Input mode
	GPIO_MODE_OUTPUT = 1, ///< Ouptput mode
	GPIO_MODE_ANALOG = 3  ///< Analog mode
}Brg_GpioModeT;

/// GPIO port output speed configuration
typedef enum {
	GPIO_SPEED_LOW = 0,      ///< Low speed
	GPIO_SPEED_MEDIUM = 1,   ///< Medium speed
	GPIO_SPEED_HIGH = 2,     ///< High speed
	GPIO_SPEED_VERY_HIGH = 3 ///< Very high speed
}Brg_GpioSpeedT;

/// GPIO port pull-up/pull-down configuration
typedef enum {
	GPIO_NO_PULL = 0,  ///< No pull-up, no pull-down
	GPIO_PULL_UP = 1,  ///< Pull-up
	GPIO_PULL_DOWN = 2 ///< Pull-down
}Brg_GpioPullT;

/// GPIO port output type configuration
typedef enum {
	GPIO_OUTPUT_PUSHPULL = 0, ///< Output push-pull
	GPIO_OUTPUT_OPENDRAIN = 1 ///< Output open-drain
}Brg_GpioOutputT;

/// GPIO init configuration, see also \ref LIMITATION
typedef struct {
	Brg_GpioModeT Mode;         ///< GPIO port mode
	Brg_GpioSpeedT Speed;       ///< GPIO port output speed
	Brg_GpioPullT Pull;         ///< GPIO port pull-up/pull-down
	Brg_GpioOutputT OutputType; ///< GPIO port output type
}Brg_GpioConfT;

/// GPIO init parameters for Brg::InitGPIO(), see also \ref LIMITATION
typedef struct {
	uint8_t GpioMask; ///< GPIO(s) to be configured (one or several value of #Brg_GpioMaskT)
	uint8_t ConfigNb; ///< Number of #Brg_GpioConfT pointed by pGpioConf:\n
	                  ///< must be #BRG_GPIO_MAX_NB or 1 (if 1 pGpioConf[0] used for all gpios)
	Brg_GpioConfT *pGpioConf;  ///< Table of ConfigNb init configuration.\n
	                  ///< If #BRG_GPIO_MAX_NB, pGpioConf[0] for GPIO_0, .., pGpioConf[3] for GPIO_3. \n
	                  ///< GPIO(s) that are not present in GpioMask are not configured.
} Brg_GpioInitT;

/// GPIO Level for Brg::ReadGPIO() and Brg::SetResetGPIO() functions
typedef enum {
	GPIO_RESET = 0, ///< GPIO Low level
	GPIO_SET = 1    ///< GPIO High level
}Brg_GpioValT;
/** @} */
// ------------------------------------------------------------------------- //
/* Class -------------------------------------------------------------------- */
/// Bridge Class
class Brg : public StlinkDevice
{
public:
	Brg(STLinkInterface &StlinkIf);

	virtual ~Brg(void);

	/**
	 * @ingroup DEVICE
	 * @retval Current Bridge C++ API version.
	 */
	int GetBridgeApiVersion(void) const {
		return 1; // Current Bridge C++ API version v1
	}

	Brg_StatusT OpenStlink(int StlinkInstId=0);
	Brg_StatusT OpenStlink(const char *pSerialNumber, bool bStrict);
	Brg_StatusT CloseStlink(void);

	Brg_StatusT ST_GetVersionExt(Stlk_VersionExtT* pVersion);
	Brg_StatusT GetTargetVoltage(float *pVoltage);

	Brg_StatusT GetLastReadWriteStatus(uint16_t *pBytesWithoutError=NULL, uint32_t *pErrorInfo=NULL);
	Brg_StatusT CloseBridge(uint8_t BrgCom);
	Brg_StatusT GetClk(uint8_t BrgCom, uint32_t *pBrgInputClk, uint32_t *pStlHClk);

	Brg_StatusT InitSPI(const Brg_SpiInitT *pInitParams);
	Brg_StatusT GetSPIbaudratePrescal(uint32_t ReqSpiFreqKHz, Brg_SpiBaudrateT *pBaudrate, uint32_t *pFinalSpiFreqKHz);
	Brg_StatusT SetSPIpinCS(Brg_SpiNssLevelT NssLevel);
	Brg_StatusT ReadSPI(uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeRead);
	Brg_StatusT WriteSPI(const uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeWritten);

	Brg_StatusT InitI2C(const Brg_I2cInitT *pInitParams);
	Brg_StatusT GetI2cTiming(I2cModeT I2CSpeedMode, int SpeedFrequency, int DNFn, int RiseTime,
	                         int FallTime, bool bAF, uint32_t *pTimingReg);
	Brg_StatusT ReadI2C(uint8_t *pBuffer, uint16_t Addr,
	                    uint16_t SizeInBytes, uint16_t *pSizeRead);
	Brg_StatusT ReadI2C(uint8_t *pBuffer, uint16_t Addr, Brg_I2cAddrModeT AddrMode,
	                    uint16_t SizeInBytes, uint16_t *pSizeRead);
	Brg_StatusT StartReadI2C(uint8_t *pBuffer, uint16_t Addr,
	                         uint16_t SizeInBytes, uint16_t *pSizeRead);
	Brg_StatusT StartReadI2C(uint8_t *pBuffer, uint16_t Addr, Brg_I2cAddrModeT AddrMode,
	                         uint16_t SizeInBytes, uint16_t *pSizeRead);
	Brg_StatusT ContReadI2C(uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeRead);
	Brg_StatusT StopReadI2C(uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeRead);
	Brg_StatusT ReadNoWaitI2C(uint16_t Addr, uint16_t SizeInBytes,
	                          uint16_t *pSizeRead, uint16_t CmdTimeoutMs);
	Brg_StatusT ReadNoWaitI2C(uint16_t Addr, Brg_I2cAddrModeT AddrMode, uint16_t SizeInBytes,
	                          uint16_t *pSizeRead, uint16_t CmdTimeoutMs);
	Brg_StatusT GetReadDataI2C(uint8_t *pBuffer, uint16_t SizeInBytes);
	Brg_StatusT WriteI2C(const uint8_t *pBuffer, uint16_t Addr,
	                     uint16_t SizeInBytes, uint16_t *pSizeWritten);
	Brg_StatusT WriteI2C(const uint8_t *pBuffer, uint16_t Addr, Brg_I2cAddrModeT AddrMode,
	                     uint16_t SizeInBytes, uint16_t *pSizeWritten);
	Brg_StatusT StartWriteI2C(const uint8_t *pBuffer, uint16_t Addr,
	                          uint16_t SizeInBytes, uint16_t *pSizeWritten);
	Brg_StatusT StartWriteI2C(const uint8_t *pBuffer, uint16_t Addr, Brg_I2cAddrModeT AddrMode,
	                          uint16_t SizeInBytes, uint16_t *pSizeWritten);
	Brg_StatusT ContWriteI2C(const uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeWritten);
	Brg_StatusT StopWriteI2C(const uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeWritten);

	Brg_StatusT InitCAN(const Brg_CanInitT *pInitParams, Brg_InitTypeT InitType);
	Brg_StatusT GetCANbaudratePrescal(const Brg_CanBitTimeConfT *pBitTimeConf, uint32_t ReqBaudrate,
	                                  uint32_t *pPrescal, uint32_t *pFinalBaudrate);
	Brg_StatusT InitFilterCAN(const Brg_CanFilterConfT *pInitParams);
	Brg_StatusT StartMsgReceptionCAN(void);
	Brg_StatusT StopMsgReceptionCAN(void);
	Brg_StatusT GetRxMsgNbCAN(uint16_t *pMsgNb);
	Brg_StatusT GetRxMsgCAN(Brg_CanRxMsgT *pCanMsg, uint16_t MsgNb, uint8_t *pBuffer,
	                        uint16_t BufSizeInBytes, uint16_t *pDataSizeInBytes);
	Brg_StatusT WriteMsgCAN(const Brg_CanTxMsgT *pCanMsg, const uint8_t *pBuffer, uint8_t SizeInBytes);

	Brg_StatusT InitGPIO(const Brg_GpioInitT *pInitParams);
	Brg_StatusT ReadGPIO(uint8_t GpioMask, Brg_GpioValT *pGpioVal, uint8_t *pGpioErrorMask);
	Brg_StatusT SetResetGPIO(uint8_t GpioMask, const Brg_GpioValT *pGpioVal, uint8_t *pGpioErrorMask);

	static Brg_StatusT ConvSTLinkIfToBrgStatus(STLinkIf_StatusT IfStat);

	bool IsCanSupport(void) const;
	bool IsReadNoWaitI2CSupport(void) const;
	bool IsOldBrgFwVersion(void) const;

private:

	Brg_StatusT AnalyzeStatus(const uint16_t *pStatus);

	Brg_StatusT SendRequestAndAnalyzeStatus(STLink_DeviceRequestT *pDevReq,
	                                        const uint16_t *pStatus,
	                                        const uint16_t UsbTimeoutMs=0);


	Brg_StatusT WriteI2Ccmd(const uint8_t *pBuffer, uint16_t Addr, uint16_t Size,
	                        Brg_I2cRWTransfer RwTransType, uint16_t *pSizeWritten, uint32_t *pErrorInfo);
	Brg_StatusT ReadI2Ccmd(uint8_t *pBuffer, uint16_t Addr, uint16_t SizeInBytes,
	                       Brg_I2cRWTransfer RwTransType, uint16_t *pSizeRead, uint32_t *pErrorInfo);

	uint8_t GpioConfField(Brg_GpioConfT GpioConfParam);

	// Global to manage I2C partial transaction (START, STOP, CONT)
	uint16_t m_slaveAddrPartialI2cTrans;

	Brg_StatusT CalculateI2cTimingReg(I2cModeT I2CSpeedMode, int SpeedFrequency, double ClockSource,
	                                  int DNFn, int RiseTime, int FallTime, bool bAF, uint32_t *pTimingReg);
	Brg_StatusT FormatFilter32bitCAN(const Brg_FilterBitsT *pInConf, uint8_t *pOutConf);
	Brg_StatusT FormatFilter16bitCAN(const Brg_FilterBitsT *pInConf, uint8_t *pOutConf);
};

#endif //_BRIDGE_H
/** @} */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/