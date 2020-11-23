/**
  ******************************************************************************
  * @file    bridge.cpp
  * @author  MCD Application Team
  * @brief   This module contains the Brg bridge class to instanciate in order to 
  *          use the STLink Bridge functionalities. \n
  *          STLinkInterface to managed USB communication needs to be instanciated
  *          and initialized before being used by Brg class. \n
  *          Brg is derived of its parent class StlinkDevice.
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
/*******************************************************************************
                            How to use this driver
 *******************************************************************************
    See examples in doxygen_bridge_example.h file or doxygen chm documentation.

********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#if defined(_MSC_VER) &&  (_MSC_VER >= 1000)
#include "stdafx.h"  // first include for windows visual studio
#endif

#include <math.h>
#include "bridge.h"

/* Private typedef -----------------------------------------------------------*/
// I2C structure for timing calculation
typedef struct {
    int prescaler;
    int SDLDEL;
    int SCLDEL;
    double resultat;
} Brg_I2cModelT;

/* Private defines -----------------------------------------------------------*/
// Size in bytes of a USB bridge command
#define STLINK_BRIDGE_CMD_SIZE_16   STLINK_CMD_SIZE_16

// Define for answer to STLINK_BRIDGE_GET_RWCMD_STATUS command
#define BRIDGE_RW_STATUS_LEN_WORD 4
#define BRIDGE_RW_STATUS_LEN_BYTE 8

// I2C define for timing calculation
#define SCLL_LENGTH    256
#define SCLH_LENGTH    256
#define SCLDEL_LENGTH  16
#define SCLDEL_LENGTH  16
#define PRESC_LENGTH   16
#define MODE_NUMBER    3

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// I2C constants for timing calculation
const double TFALL_MAX_T0 = (double)(300 / pow((double)10, 9));
const double TFALL_MAX_T1 = (double)(300 / pow((double)10, 9));
const double TFALL_MAX_T2 = (double)(120 / pow((double)10, 9));

const double TRISE_MAX_T0 = (double)(1000 / pow((double)10, 9));
const double TRISE_MAX_T1 = (double)(300 / pow((double)10, 9));
const double TRISE_MAX_T2 = (double)(120 / pow((double)10, 9));

const double THDDAT_MAX_T0 = (double)(3450 / pow((double)10, 9));
const double THDDAT_MAX_T1 = (double)(900 / pow((double)10, 9));
const double THDDAT_MAX_T2 = (double)(450 / pow((double)10, 9));

const double TSUDAT_MIN_T0 = (double)(250 / pow((double)10, 9));
const double TSUDAT_MIN_T1 = (double)(100 / pow((double)10, 9));
const double TSUDAT_MIN_T2 = (double)(50 / pow((double)10, 9));

const double TLOW_MIN0 = (double)(4.7 / pow((double)10, 6));
const double TLOW_MIN1 = (double)(1.3 / pow((double)10, 6));
const double TLOW_MIN2 = (double)(0.5 / pow((double)10, 6));

const double THIGH_MIN0 = (double)(4 / pow((double)10, 6));
const double THIGH_MIN1 = (double)(0.6 / pow((double)10, 6));
const double THIGH_MIN2 = (double)(0.26 / pow((double)10, 6));

/* Global variables ----------------------------------------------------------*/

/* Class Functions Definition ------------------------------------------------*/

/**
 * @ingroup DEVICE
 * @brief Brg constructor
 * @param[in]  StlinkIf  reference to USB STLink Bridge interface: STLinkInterface(STLINK_BRIDGE)
 */
Brg::Brg(STLinkInterface &StlinkIf): StlinkDevice(StlinkIf), m_slaveAddrPartialI2cTrans(0)
{
	this->SetOpenModeExclusive(true);
}
/**
 * @ingroup DEVICE
 * @brief Brg destructor
 */
Brg::~Brg(void)
{
	// Close device if necessary
	CloseBridge(COM_UNDEF_ALL);
	// Close STLink is done by ~StlinkDevice
}

/**
 * @ingroup DEVICE
 * @brief Convert status returned by STLinkInterface to Bridge status
 * @param[in] IfStat STLinkInterface status
 * @return  Corresponding #Brg_StatusT
 */
Brg_StatusT Brg::ConvSTLinkIfToBrgStatus(STLinkIf_StatusT IfStat)
{
	Brg_StatusT  brgStat;
	switch(IfStat){
		case STLINKIF_NO_ERR:
			brgStat = BRG_NO_ERR;
			break;
		case STLINKIF_CONNECT_ERR:
			brgStat = BRG_CONNECT_ERR;
			break;
		case STLINKIF_DLL_ERR:
			brgStat = BRG_DLL_ERR;
			break;
		case STLINKIF_USB_COMM_ERR:
			brgStat = BRG_USB_COMM_ERR;
			break;
		case STLINKIF_PARAM_ERR:
			brgStat = BRG_PARAM_ERR;
			break;
		case STLINKIF_NO_STLINK:
			brgStat = BRG_NO_STLINK;
			break;
		case STLINKIF_NOT_SUPPORTED:
			brgStat = BRG_NOT_SUPPORTED;
			break;
		case STLINKIF_PERMISSION_ERR:
			brgStat = BRG_PERMISSION_ERR;
			break;
		case STLINKIF_ENUM_ERR:
			brgStat = BRG_ENUM_ERR;
			break;
		case STLINKIF_GET_INFO_ERR:
			brgStat = BRG_GET_INFO_ERR;
			break;
		case STLINKIF_STLINK_SN_NOT_FOUND:
			brgStat = BRG_STLINK_SN_NOT_FOUND;
			break;
		case STLINKIF_CLOSE_ERR:
			brgStat = BRG_CLOSE_ERR;
			break;
		default:
			brgStat = BRG_INTERFACE_ERR;
			break;
	}
	return brgStat;
}

/**
 * @ingroup DEVICE
 * @retval true If not the last Bridge Firmware version.
 * @note To be called after OpenStlink().
 */
bool Brg::IsOldBrgFwVersion(void) const {
	// Return true if current Bridge FW version is not the last one for given STLink
	if( ((m_Version.Major_Ver==3) && (m_Version.Bridge_Ver<FIRMWARE_BRIDGE_STLINK_V3_LAST_VERSION)) ) {
		return true;
	} else {
		return false;
	}
}

/**
 * @ingroup DEVICE
 * @brief Open BRIDGE USB connection with the STLink device. \n
 * This routine use the USB STLink Bridge interface so it is required to have previously called
 * successfully STLinkInterface::LoadStlinkLibrary() to load STLinkUSBDriver library. \n
 * User can call STLinkInterface::EnumDevices() and STLinkInterface::GetDeviceInfo2() to get the
 * list of available USB STLink Bridge interface devices and choose StlinkInstId. 
 * If not already done, this routine will enumerate the USB STLink Bridge interface anyway. \n
 * It also checks firmware version. \n
 * StlinkDevice::SetOpenModeExclusive() is used before opening the device to choose the opening mode.
 *
 * @param[in]  StlinkInstId  Instance ID in the list of enumerated STLink devices.
 *                           from 0 to *pNumDevices-1 (value returned by STLinkInterface::EnumDevices()).
 * @warning #BRG_OLD_FIRMWARE_WARNING is not a fatal error.
 * @return Brg::ConvSTLinkIfToBrgStatus(STLinkInterface::EnumDevices()) errors
 * @retval #BRG_CONNECT_ERR Wrong StlinkInstId or USB error
 * @retval #BRG_OLD_FIRMWARE_WARNING Warning that current Bridge Firmware version is not the last one
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::OpenStlink(int StlinkInstId)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;
	Brg_StatusT brgStatus;

	ifStatus = StlinkDevice::PrivOpenStlink(StlinkInstId);

	brgStatus = ConvSTLinkIfToBrgStatus(ifStatus);
	if( brgStatus == BRG_NO_ERR ) {
		// All is OK but send a warning in case of old firmware
		if( IsOldBrgFwVersion() == true )
		{
			LogTrace("The detected STLink firmware BRIDGE version (V%d.B%d) is compatible with PC software but is not the most recent one",
				(int)m_Version.Major_Ver, (int)m_Version.Bridge_Ver);
			brgStatus = BRG_OLD_FIRMWARE_WARNING;
		}
	}
	return brgStatus;
}
/**
 * @ingroup DEVICE
 * @brief Same as Brg::OpenStlink(int StlinkInstId) but opening routine based on the serial number of the STLink. \n
 * To be used in case StlinkInstId is no more reliable (devices have been reenumerated between STLink instance
 * selection and opening) or to choose a specific STLink by its serial number.
 * 
 * @param[in]  pSerialNumber  STLink serial number (ASCII) (can be retrieved with STLinkInterface::GetDeviceInfo2()).
 * @param[in]  bStrict 	 Used if STLink with required serial number is not found, but one other STLink is found:
 *                       choose if we open or not the other STLink. \n
 *                       If bStrict is true, the command will return #BRG_STLINK_SN_NOT_FOUND if the
 *                       given SerialNumber is not found.\n
 *                       If bStrict is false and the given serial number is not found and one (and only one)
 *                       STLink is found, the command will attempt to open the STLink even if
 *                       the serial number does not match.
 * @warning #BRG_OLD_FIRMWARE_WARNING is not a fatal error.
 * @return Brg::ConvSTLinkIfToBrgStatus(STLinkInterface::EnumDevices()) errors
 * @retval #BRG_CONNECT_ERR USB error
 * @retval #BRG_OLD_FIRMWARE_WARNING Warning that current Bridge Firmware version is not the last one
 * @retval #BRG_STLINK_SN_NOT_FOUND Serial number not found
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::OpenStlink(const char *pSerialNumber, bool bStrict) {
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;
	Brg_StatusT brgStatus;

	ifStatus = StlinkDevice::PrivOpenStlink(pSerialNumber, bStrict);

	brgStatus = ConvSTLinkIfToBrgStatus(ifStatus);
	if( brgStatus == BRG_NO_ERR ) {
		// All is OK but send a warning in case of old firmware
		if( IsOldBrgFwVersion() == true )
		{
			LogTrace("The detected STLink firmware BRIDGE version (V%d.B%d) is compatible with PC software but is not the most recent one",
				(int)m_Version.Major_Ver, (int)m_Version.Bridge_Ver);
			brgStatus = BRG_OLD_FIRMWARE_WARNING;
		}
	}
	return brgStatus;
}
/**
 * @ingroup DEVICE
 * @brief Close STLink USB communication with the device instance that was opened by Brg::OpenStlink().
 *
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::CloseStlink(void)
{
	StlinkDevice::PrivCloseStlink();
	return BRG_NO_ERR;
}
/**
 * @ingroup DEVICE
 * @brief This routine closes given or all Bridge communication(s), need to call again Init function after if
 * new bridge communication is required.
 * @param[in]  BrgCom Communication(s) to be closed: One of #COM_I2C, #COM_SPI, #COM_CAN, #COM_GPIO \n
 *                    or #COM_UNDEF_ALL for all.
 *
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::CloseBridge(uint8_t BrgCom)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint32_t answer = 0;
	uint8_t closeCom;

	if( (BrgCom != COM_SPI)&&(BrgCom != COM_I2C)&&(BrgCom != COM_CAN)
		 &&(BrgCom != COM_GPIO)&&(BrgCom != COM_UNDEF_ALL) ) {
		return BRG_PARAM_ERR;
	}
	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}

	if( BrgCom == COM_UNDEF_ALL ) { // Close all bridge interfaces
		closeCom = 0;
	} else { // Close only the given interface
		closeCom = BrgCom;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_CLOSE;
	// Bridge interface
	pRq->CDBByte[2] = closeCom;

	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = &answer;
	pRq->BufferLength = 2;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, (uint16_t*)&answer);

	delete pRq;

	return brgStat;
}
/**
 * @ingroup DEVICE
 * @brief This routine gets USB VID and PID, and firmware version of the STLink Bridge device.
 * @param[out] pVersion Pointer filled with version information.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::ST_GetVersionExt(Stlk_VersionExtT* pVersion)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;
	ifStatus = StlinkDevice::PrivGetVersionExt(pVersion);
	return ConvSTLinkIfToBrgStatus(ifStatus);
}
/* Send a command over the USB, wait for answer, and analyze the returned status if pStatus!=NULL
 * UsbTimeoutMs if 0 use default (5s) else use UsbTimeoutMs.
 * Requires the STLink to be already connected (m_bStlinkConnected==true)
*/
Brg_StatusT Brg::SendRequestAndAnalyzeStatus(STLink_DeviceRequestT *pDevReq,
                                             const uint16_t *pStatus,
											 const uint16_t UsbTimeoutMs)
{
	Brg_StatusT brgStat = BRG_PARAM_ERR;
	STLinkIf_StatusT ifStatus;

	ifStatus = StlinkDevice::SendRequest(pDevReq, UsbTimeoutMs);
	if( ifStatus != STLINKIF_NO_ERR) {
		return BRG_USB_COMM_ERR;
	}
	// Analyse status
	brgStat = AnalyzeStatus(pStatus);
	if( brgStat == BRG_TARGET_CMD_ERR ) {
		// Default error
		// If useful, one can add some error codes in Brg_StatusT corresponding
		// to firmware error codes from stlink_firmware_const.h.
		// It may be useful also to trace the error here, before loosing
		// the exact value of the firmware error.
		LogTrace("BRIDGE Error (0x%hx) after BRIDGE cmd %02hX %02hX %02hX %02hX %02hX %02hX %02hX %02hX %02hX %02hX",
		//printf("Error (0x%hx) after target cmd %02hX %02hX %02hX %02hX %02hX %02hX %02hX %02hX %02hX %02hX",
			(unsigned short)*pStatus,
			(unsigned short)pDevReq->CDBByte[0], (unsigned short)pDevReq->CDBByte[1], (unsigned short)pDevReq->CDBByte[2],
			(unsigned short)pDevReq->CDBByte[3], (unsigned short)pDevReq->CDBByte[4], (unsigned short)pDevReq->CDBByte[5], 
			(unsigned short)pDevReq->CDBByte[6], (unsigned short)pDevReq->CDBByte[7], (unsigned short)pDevReq->CDBByte[8],
			(unsigned short)pDevReq->CDBByte[9]);
	}

	return brgStat;
}
/*
 * Analyze the STLink returned status if pStatus!=NULL and convert it to Bridge status
 */
Brg_StatusT Brg::AnalyzeStatus(const uint16_t *pStatus)
{
	if( pStatus != NULL )
	{
		if( *pStatus != STLINK_BRIDGE_OK )
		{
			// Precise some cases for upper layers
			if( *pStatus == STLINK_BRIDGE_UNKNOWN_CMD) {
				LogTrace("BRIDGE Command not supported");
				return BRG_CMD_NOT_SUPPORTED;
			}
			if( *pStatus == STLINK_BRIDGE_BAD_PARAM) {
				LogTrace("BRIDGE Bad command parameter");
				return BRG_PARAM_ERR;
			}
			if( *pStatus == STLINK_BRIDGE_SPI_ERROR) {
				LogTrace("BRIDGE SPI issue");
				return BRG_SPI_ERR;
			}
			if( *pStatus == STLINK_BRIDGE_I2C_ERROR) {
				LogTrace("BRIDGE I2C issue");
				return BRG_I2C_ERR;
			}
			if( *pStatus == STLINK_BRIDGE_CAN_ERROR) {
				LogTrace("BRIDGE CAN issue");
				return BRG_CAN_ERR;
			}
			if( *pStatus == STLINK_BRIDGE_INIT_NOT_DONE) {
				LogTrace("This BRIDGE command requires the com to be initialized: call Init function");
				return BRG_COM_INIT_NOT_DONE;
			}
			if( *pStatus == STLINK_BRIDGE_ABORT_TRANS) {
				LogTrace("BRIDGE Incorrect command order in partial (I2C) transaction, current transaction aborted");
				return BRG_COM_CMD_ORDER_ERR;
			}
			if( *pStatus == STLINK_BRIDGE_TIMEOUT_ERR) {
				LogTrace("BRIDGE Timeout waiting for command execution");
				return BRG_TARGET_CMD_TIMEOUT;
			}
			if( *pStatus == STLINK_BRIDGE_CMD_BUSY) {
				LogTrace("BRIDGE Command busy (only GET_RWCMD_STATUS allowed in this state)");
				return BRG_CMD_BUSY;
			}
			// All other errors will be seen as "target command" error.
			return BRG_TARGET_CMD_ERR;
		}
	}
	return BRG_NO_ERR;
}
/**
 * @ingroup BRIDGE
 * @brief This routine gets some current frequencies, useful to choose bridge initialization parameters.
 * @warning: Frequency parameters are valid while STLink frequency is not changed on 
 *           debug interface by using STLINK_SWITCH_STLINK_FREQ.
 * @param[in]  BrgCom Bridge com: one of #COM_I2C, #COM_SPI, #COM_CAN, #COM_GPIO
 * @param[out] pBrgInputClk Current input frequency in KHz of the given com.
 * @param[out] pStlHClk Current STLink HCLK in KHz.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_PARAM_ERR Bad BrgCom parameters or null pointer
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::GetClk(uint8_t BrgCom, uint32_t *pBrgInputClk, uint32_t *pStlHClk)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint8_t answer[12]={0,0,0,0,0,0,0,0,0,0,0,0};

	if( (pBrgInputClk == NULL) || (pStlHClk == NULL) ) {
		return BRG_PARAM_ERR;
	}
	if( (BrgCom != COM_SPI)&&(BrgCom != COM_I2C)&&(BrgCom != COM_CAN)&&(BrgCom != COM_GPIO) ) {
		return BRG_PARAM_ERR;
	}

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_GET_CLOCK;
	// Bridge interface
	pRq->CDBByte[2] = (uint8_t) (BrgCom);


	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = &answer;
	pRq->BufferLength = 12;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, (uint16_t*)answer);

	*pBrgInputClk = (uint32_t)answer[4] | (uint32_t)answer[5]<<8 | (uint32_t)answer[6]<<16 | (uint32_t)answer[7]<<24;
	*pStlHClk = (uint32_t)answer[8] | (uint32_t)answer[9]<<8 | (uint32_t)answer[10]<<16 | (uint32_t)answer[11]<<24;

	delete pRq;

	return brgStat;
}
/**
 * @ingroup SPI
 * @brief This routine initializes the SPI according to init parameters.\n
 * Refer to STLink firmware upgrade release note (stsw-link007) for supported init configurations.\n
 * Call Brg::GetSPIbaudratePrescal() to get baudrate parameter according to required SPI output frequency.
 * @warning: Baudrate prescaler parameter is valid while STLink frequency is not changed on 
 *           debug interface by using STLINK_SWITCH_STLINK_FREQ.
 * @param[in]  pInitParams SPI initialization parameters see #Brg_SpiInitT and \ref LIMITATION
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_PARAM_ERR Null pointer parameter
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::InitSPI(const Brg_SpiInitT *pInitParams)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint16_t status;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( pInitParams == NULL ) {
		return BRG_PARAM_ERR;
	}
	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_INIT_SPI;
	// Data direction
	pRq->CDBByte[2] = (uint8_t) (pInitParams->Direction);
	// Mode (Mode/Phase/Polarity/Transmission/FrameFormat)
	pRq->CDBByte[3] = ((uint8_t)pInitParams->Mode & 0x01) | ((((uint8_t)pInitParams->Cpha) << 1) & 0x02)
	                | ((((uint8_t)pInitParams->Cpol) << 2) & 0x04) | ((((uint8_t)pInitParams->FirstBit) << 3) & 0x08)
					| ((((uint8_t)pInitParams->FrameFormat) << 4) & 0x10);
	// Data size
	pRq->CDBByte[4] = (uint8_t) (pInitParams->DataSize);
	// Slave Select management
	pRq->CDBByte[5] = ((uint8_t)pInitParams->Nss & 0x1) | ((((uint8_t)pInitParams->NssPulse) << 1) & 0x2);
	// BaudRate Prescaler
	pRq->CDBByte[6] = (uint8_t) (pInitParams->Baudrate);
	// CRC
	if( pInitParams->Crc == SPI_CRC_DISABLE ) {
		pRq->CDBByte[7] = 0; // CRC disable (0)
		pRq->CDBByte[8] = 0;
	} else {
		if( ((pInitParams->CrcPoly & 0x1) == 0x1) && (pInitParams->CrcPoly <= 0xFFFF) ) {
			// CRC polynomial >= 0x1 (odd value only)
			pRq->CDBByte[7] = (uint8_t)(pInitParams->CrcPoly&0xFF);
			pRq->CDBByte[8] = (uint8_t)((pInitParams->CrcPoly>>8)&0xFF);
		} else {
			delete pRq;
			return BRG_PARAM_ERR;
		}
	}

	if( pInitParams->SpiDelay == DELAY_FEW_MICROSEC ) {
		pRq->CDBByte[9] = 1;
	} else {
		pRq->CDBByte[9] = 0; // normal case no delay
	}

	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = &status;
	pRq->BufferLength = 2;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, &status);
	delete pRq;

	return brgStat;
}
/**
 * @ingroup SPI
 * @brief This routine gets the SPI baudrate prescaler needed for Brg::InitSPI() according to required SPI CLK.
 * @param[in]  ReqSpiFreqKHz Required SPI CLK (SCK) in KHz
 * @param[out] pBaudrate Baudrate parameter to use (in Brg_SpiInitT Brg::InitSPI())
 * @param[out] pFinalSpiFreqKHz Final SPI CLK (SCK) in KHz corresponding to returned baudrate
 *                              (may be less than ReqSpiFreqKHz)
 * @warning: Baudrate parameter is valid while STLink frequency is not changed on debug interface 
 *           by using STLINK_SWITCH_STLINK_FREQ.\n
 *           #BRG_COM_FREQ_MODIFIED is not a fatal error.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_FREQ_MODIFIED If ReqSpiFreqKHz is different from pFinalSpiFreqKHz
 * @retval #BRG_PARAM_ERR In case of wrong parameter
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::GetSPIbaudratePrescal(uint32_t ReqSpiFreqKHz, Brg_SpiBaudrateT *pBaudrate, uint32_t *pFinalSpiFreqKHz)
{
	Brg_StatusT brgStat;
	uint32_t spiInputClkKHz=0;
	uint32_t stlHClkKHz=0;
	uint32_t calcBaudrate=1, finalBaudrate=1;

	if( (ReqSpiFreqKHz == 0) || (pBaudrate == NULL) || (pFinalSpiFreqKHz == NULL) ) {
		return BRG_PARAM_ERR;
	}
	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	// Get the current SPI input Clk
	brgStat = GetClk(COM_SPI, &spiInputClkKHz, &stlHClkKHz);
	if( brgStat == BRG_NO_ERR ) {
		calcBaudrate = spiInputClkKHz/ReqSpiFreqKHz;
		// Apply a smaller frequency if not exact
		if( calcBaudrate <= 2 ) {
			finalBaudrate = 2;
			*pBaudrate = SPI_BAUDRATEPRESCALER_2;
		} else if( calcBaudrate <= 4 ) {
			finalBaudrate = 4;
			*pBaudrate = SPI_BAUDRATEPRESCALER_4;
		} else if( calcBaudrate <= 8 ) {
			finalBaudrate = 8;
			*pBaudrate = SPI_BAUDRATEPRESCALER_8;
		} else if( calcBaudrate <= 16 ) {
			finalBaudrate = 16;
			*pBaudrate = SPI_BAUDRATEPRESCALER_16;
		} else if( calcBaudrate <= 32 ) {
			finalBaudrate = 32;
			*pBaudrate = SPI_BAUDRATEPRESCALER_32;
		} else if( calcBaudrate <= 64 ) {
			finalBaudrate = 64;
			*pBaudrate = SPI_BAUDRATEPRESCALER_64;
		} else if( calcBaudrate <= 128 ) {
			finalBaudrate = 128;
			*pBaudrate = SPI_BAUDRATEPRESCALER_128;
		} else if( calcBaudrate <= 256 ) {
			finalBaudrate = 256;
			*pBaudrate = SPI_BAUDRATEPRESCALER_256;
		} else {
			// smaller frequency not possible use a different error code
			brgStat = BRG_COM_FREQ_NOT_SUPPORTED; 
			finalBaudrate = 256;
			*pBaudrate = SPI_BAUDRATEPRESCALER_256;
		}
	}
	*pFinalSpiFreqKHz = spiInputClkKHz/finalBaudrate;
	if( (brgStat == BRG_NO_ERR) && ( *pFinalSpiFreqKHz != ReqSpiFreqKHz) ) {
		brgStat = BRG_COM_FREQ_MODIFIED;
	}
	return brgStat;
}
/**
 * @ingroup SPI
 * @brief This routine allows to manage SPI NSS (nCS) pin when SPI has been initialized with #SPI_NSS_SOFT.
 * @param[in]  NssLevel Level to apply on NSS pin.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If SPI is not initialized
 * @retval #BRG_PARAM_ERR In case of #SPI_NSS_HARD
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::SetSPIpinCS(Brg_SpiNssLevelT NssLevel)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint16_t status;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_CS_SPI;
	pRq->CDBByte[2] = (uint8_t) (NssLevel);

	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = &status;
	pRq->BufferLength = 2;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, &status);
	delete pRq;

	return brgStat;
}
/**
 * @ingroup SPI
 * @brief This routine allows to receive bytes on the SPI interface, in the mode initialized by Brg::InitSPI().\n
 * In #SPI_DIRECTION_2LINES_FULLDUPLEX dummy data are sent while receiving.
 * @param[out] pBuffer Pointer on data buffer filled with read data.
 * @param[in]  SizeInBytes Data size to be read in bytes (min 1, max data buffer size)
 * @param[out] pSizeRead If not NULL and in case of error, pSizeRead returns the number of bytes
 *             received before the error.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If SPI is not initialized
 * @retval #BRG_SPI_ERR In case of SPI read error
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::ReadSPI(uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeRead)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( pBuffer == NULL ) {
		return BRG_PARAM_ERR;
	}
	if( SizeInBytes==0 ) {
		return BRG_NO_ERR;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_READ_SPI;
	pRq->CDBByte[2] = (uint8_t)SizeInBytes;
	pRq->CDBByte[3] = (uint8_t)(SizeInBytes>>8);

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->BufferLength = SizeInBytes;
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = pBuffer;

	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, NULL);

	delete pRq;

	if( brgStat == BRG_NO_ERR )
	{	// pErrorInfo currently unused
		brgStat = GetLastReadWriteStatus(pSizeRead, NULL);
	}

	if( brgStat != BRG_NO_ERR ) {
		LogTrace("SPI Error (%d) in ReadSPI (%d bytes)", (int)brgStat,(int)SizeInBytes);
		if( pSizeRead != NULL ) {
			LogTrace("SPI Only %d bytes read without error",(int)*pSizeRead);
		}
	}

#ifdef USING_TRACELOG
	else {
		if(SizeInBytes==4) {
			LogTrace("SPI R: 0x%08lx", (unsigned long)*pBuffer);
		} else {
			LogTrace("SPI R %d bytes", (int)SizeInBytes);
		}
	}
#endif

	return brgStat;
}
/**
 * @ingroup SPI
 * @brief This routine allows to transmit bytes on the SPI interface, in the mode initialized by Brg::InitSPI().
 * @param[in]  pBuffer Pointer on data buffer with data to be sent.
 * @param[in]  SizeInBytes Data size to be sent in bytes (min 1, max data buffer size)
 * @param[out] pSizeWritten If not NULL and in case of error, pSizeWritten returns the number of bytes
 *             transmitted before the error.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If SPI is not initialized
 * @retval #BRG_SPI_ERR In case of SPI write error
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::WriteSPI(const uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeWritten)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( pBuffer == NULL ) {
		return BRG_PARAM_ERR;
	}
	if( SizeInBytes==0 ) {
		return BRG_NO_ERR;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));
	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_WRITE_SPI;
	pRq->CDBByte[2] = (uint8_t)SizeInBytes;
	pRq->CDBByte[3] = (uint8_t)(SizeInBytes>>8);
	if( SizeInBytes > 8 ) {	
		// First bytes to transfer to target
		for( int i = 0; i<8; i++ ) {
			pRq->CDBByte[4+i] = pBuffer[i];
		}
		pRq->BufferLength = SizeInBytes - 8;
		pRq->InputRequest = REQUEST_WRITE_1ST_EPOUT;
		pRq->Buffer = (void *)&pBuffer[8];
	} else {
		// First bytes to transfer to target
		for( int i = 0; i<SizeInBytes; i++ ) {
			pRq->CDBByte[4+i] = pBuffer[i];
		}
		// If less than 8 bytes all data are sent inside the cmd
		// Just send the cmd (no data follows)
		pRq->BufferLength = 0;
		pRq->InputRequest = REQUEST_READ_1ST_EPIN;
		pRq->Buffer = NULL;
	}

	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, NULL);

	delete pRq;

	if( brgStat == BRG_NO_ERR )
	{	// pErrorInfo currently unused
		brgStat = GetLastReadWriteStatus(pSizeWritten,NULL);
	}

	if( brgStat != BRG_NO_ERR ) {
		LogTrace("SPI Error (%d) in WriteSPI (%d bytes)", (int)brgStat,(int)SizeInBytes);
		if( pSizeWritten != NULL ) {
			LogTrace("SPI Only %d bytes written without error",(int)*pSizeWritten);
		}
	}
	return brgStat;
}
/**
 * @ingroup I2C
 * @brief This routine initializes the I2C according to init parameters.\n
 * Refer to STLink firmware upgrade release note (stsw-link007) for supported init configurations.\n
 * Call Brg::GetI2cTiming() to get timing parameter according to required I2C speed and time configuration parameters.
 * @param[in]  pInitParams I2C initialization parameters see #Brg_I2cInitT and \ref LIMITATION
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_PARAM_ERR Null pointer parameter
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::InitI2C(const Brg_I2cInitT *pInitParams)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint16_t status;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( pInitParams == NULL ) {
		return BRG_PARAM_ERR;
	}
	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_INIT_I2C;
	// Timing register: I2C TIMINGR 
	pRq->CDBByte[2] = (uint8_t) (pInitParams->TimingReg);
	pRq->CDBByte[3] = (uint8_t) (pInitParams->TimingReg>>8);
	pRq->CDBByte[4] = (uint8_t) (pInitParams->TimingReg>>16);
	pRq->CDBByte[5] = (uint8_t) (pInitParams->TimingReg>>24);
	// OwnAddress1 <= 0X3FF (for slave mode)
	if( pInitParams->OwnAddr <= 0x3FF ) {
		pRq->CDBByte[6] = (uint8_t) (pInitParams->OwnAddr);
		pRq->CDBByte[7] = (uint8_t) (pInitParams->OwnAddr>>8);
	} else {
		delete pRq;
		return BRG_PARAM_ERR;
	}
	// AddressingMode
	pRq->CDBByte[8] = (uint8_t) pInitParams->AddrMode;
	// Filters : analog and digital filters config : Bit3-0: DNF (<=15)  Bit7: analog filter 0 OFF, 1 ON
	//  Bit6-4: 0 (reserved)
	if( pInitParams->DigitalFilterEn == I2C_FILTER_DISABLE ) {
		// Bit3-0: DNF = 0 if digital filter OFF
		pRq->CDBByte[9] = ((((uint8_t)pInitParams->AnFilterEn) << 7) & 0x80);
	} else {
		if( pInitParams->Dnf <= 15 ) {
			pRq->CDBByte[9] = ((uint8_t)pInitParams->Dnf & 0x0F) | ((((uint8_t)pInitParams->AnFilterEn) << 7) & 0x80);
		} else {
			delete pRq;
			return BRG_PARAM_ERR;
		}		
	}
	// Reset partial I2C transaction global
	m_slaveAddrPartialI2cTrans = 0;

	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = &status;
	pRq->BufferLength = 2;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, &status);
	delete pRq;

	return brgStat;
}
/**
 * @ingroup I2C
 * @brief This routines can be used to get timing parameter (pTimingReg) required by Brg::InitI2C().\n
 * It set timing parameter according to required I2C speed, mode, rise/fall time and filter configuration.
 * @param[in]  I2CSpeedMode  #I2C_STANDARD, #I2C_FAST, #I2C_FAST_PLUS
 * @param[in]  SpeedFrequency  In KHz, 1-100KHz (STANDARD), 1-400KHz (FAST), 1-1000KHz (FAST PLUS)
 * @param[in]  DNFn  0 (no digital filter) up to 15, noise digital filter (delay = DNFn/SpeedFrequency)
 * @param[in]  RiseTime In ns, 0-1000ns (STANDARD), 0-300ns (FAST), 0-120ns (FAST PLUS)
 * @param[in]  FallTime In ns, 0-300ns (STANDARD), 0-300ns (FAST), 0-120ns (FAST PLUS)
 * @param[in]  bAF  Use true for Analog Filter ON or false for Analog Filter OFF
 * @param[out] pTimingReg  Filled with timing parameter required by Brg::InitI2C().
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_PARAM_ERR Null pointer parameter
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::GetI2cTiming(I2cModeT I2CSpeedMode, int SpeedFrequency, int DNFn,
                              int RiseTime, int FallTime, bool bAF, uint32_t *pTimingReg)
{
	Brg_StatusT brgStatus=BRG_NO_ERR;
	double clockSource;
	uint32_t stlHClkKHz, i2cInputClkKHz;

	if( pTimingReg == NULL ) {
		return BRG_PARAM_ERR;
	}

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}

	*pTimingReg = 0; // Default if error
	if( (SpeedFrequency < 1) || (RiseTime < 0) || (FallTime < 0)|| (DNFn>15) || (DNFn<0)
		|| ((I2CSpeedMode == I2C_STANDARD) && ((SpeedFrequency >100) || (RiseTime > 1000) || (FallTime > 300)))
		|| ((I2CSpeedMode == I2C_FAST) && ((SpeedFrequency >400) || (RiseTime > 300) || (FallTime > 300)))
		|| ((I2CSpeedMode == I2C_FAST_PLUS) && ((SpeedFrequency >1000) || (RiseTime > 120) || (FallTime > 120)))
	     ) {
		return BRG_PARAM_ERR;
	}

	// Get the current I2C input Clk
	brgStatus = GetClk(COM_I2C, &i2cInputClkKHz, &stlHClkKHz);
	if( brgStatus == BRG_NO_ERR ) {
		clockSource = (double)i2cInputClkKHz;
		brgStatus = CalculateI2cTimingReg(I2CSpeedMode, SpeedFrequency, clockSource, DNFn,
		                                  RiseTime, FallTime, bAF, pTimingReg);
	}

	return brgStatus;
}
/*
 * Internal function used by GetI2cTiming()
 */
Brg_StatusT Brg::CalculateI2cTimingReg(I2cModeT I2CSpeedMode, int SpeedFrequency, double ClockSource, int DNFn,
                                       int RiseTime, int FallTime, bool bAF, uint32_t *pTimingReg)
{
#define MAX_I2C_MODEL_NB 40
	Brg_StatusT brgStatus=BRG_NO_ERR;

	if( (SpeedFrequency == 0) || (ClockSource == 0) || (pTimingReg == NULL) ) {
		return BRG_PARAM_ERR;
	}

    // SCLL & SCLH  settings
    int *pSCLL = new int[SCLL_LENGTH];
    int *pSCLH = new int[SCLH_LENGTH];

    //SCLDEL & SDLDEL settings
    int *pSCLDEL = new int[SCLDEL_LENGTH];
    int *pSDADEL = new int[SCLDEL_LENGTH];
    // PRESC settings
    int *pPRESC = new int[PRESC_LENGTH];
    int *pPRESCvalid = new int[16];

    // Variables for solution  (SCLDEL , SDLDEL  and PRESC)
    Brg_I2cModelT *pPrescSDLDELSCLDELpossComb = new Brg_I2cModelT[MAX_I2C_MODEL_NB]; // possible combination table
    int prescSDLDELSCLDELpossCombSize = 0;
    int tabIndex = 0;

    // Variables for Boundaries
    double *pTFallMax = new double[MODE_NUMBER];
    double *pTRiseMax = new double[MODE_NUMBER];
    double *pTHdDatMax = new double[MODE_NUMBER];
    double *pTSuDatMin = new double[MODE_NUMBER];
    double *pTLowMin = new double[MODE_NUMBER];
    double *pTHighMin = new double[MODE_NUMBER];

    double targetFreqI2C;   // Target frequency
    double targetPeriodI2C; // Target period of clock
    double tmpSCLDEL;       // Calculate SCLDEL Time from  SCLDEL Bit
    double tmpSDADEL;       // Calculate SDLDEL Time from  SDLDEL Bit
    double clkMax;
    double clkMin;
    int idx=0;              // Get I2c mode speed
    double clkI2C;          // Clock frequency
    double clkPeriodI2C;    // Period of clock

    double delayAF;
    double delayDNF;
    double delayFilter;
    double riseTimeCalc;   // Rise time
    double fallTimeCalc;   // Fall time
    double tmpMinSDADEL;   // SDADEL range
    double tmpMaxSDADEL;   // SDADEL range
    double tmpMinSCLDEL;   // SCLDEL range
    int nbValid;           // Number of valid solution
    int tmpPRESC;          // Current prescaler

	// Timing handling
	int i;
	for (i = 0; i < SCLL_LENGTH; i++) {
		pSCLL[i] = i;//current SCLL value
		pSCLH[i] = i;//cuirrent SCLH value
	}
	for (i = 0; i < SCLDEL_LENGTH; i++) {
		pPRESC[i] = i;//current Presc value
		pSCLDEL[i] = i;
		pSDADEL[i] = i;
		pPRESCvalid[i] = 99;
	}
	pTFallMax[0] = TFALL_MAX_T0;
	pTFallMax[1] = TFALL_MAX_T1;
	pTFallMax[2] = TFALL_MAX_T2;

	pTRiseMax[0] = TRISE_MAX_T0;
	pTRiseMax[1] = TRISE_MAX_T1;
	pTRiseMax[2] = TRISE_MAX_T2;

	pTHdDatMax[0] = THDDAT_MAX_T0;
	pTHdDatMax[1] = THDDAT_MAX_T1;
	pTHdDatMax[2] = THDDAT_MAX_T2;

	pTSuDatMin[0] = TSUDAT_MIN_T0;
	pTSuDatMin[1] = TSUDAT_MIN_T1;
	pTSuDatMin[2] = TSUDAT_MIN_T2;

	pTLowMin[0] = TLOW_MIN0;
	pTLowMin[1] = TLOW_MIN1;
	pTLowMin[2] = TLOW_MIN2;

	pTHighMin[0] = THIGH_MIN0;
	pTHighMin[1] = THIGH_MIN1;
	pTHighMin[2] = THIGH_MIN2;

	targetFreqI2C = SpeedFrequency * 1000;   // Speed frequency
	targetPeriodI2C = (1 / (targetFreqI2C)); // Speed period

	clkMax = targetFreqI2C + targetFreqI2C * 0.2;
	clkMin = targetFreqI2C - targetFreqI2C * 0.2;

	clkI2C = ClockSource;
	clkPeriodI2C = (double)(1 / (double)(clkI2C * 1000));  // Clock period

	if( I2CSpeedMode == I2C_STANDARD ) {
		idx = 0;
	}
	if( I2CSpeedMode == I2C_FAST ) {
		idx = 1;
	}
	if( I2CSpeedMode == I2C_FAST_PLUS ) {
		idx = 2;
	}
	if( bAF == true ) {
		delayAF = (50 / (double)pow((double)10, 9)) * 1;
	} else {
		delayAF = 0; // (50 / (double)pow((double)10, 9)) * 0;
	}

	delayDNF = (DNFn * clkPeriodI2C);
	delayFilter = delayAF + delayDNF;

	riseTimeCalc = (double)(RiseTime / (double)pow((double)10, 9));
	fallTimeCalc = (double)(FallTime / (double)pow((double)10, 9));

	tmpMinSDADEL = (double)((double)fallTimeCalc - (50 / (double)pow((double)10, 9))
	                - (double)((DNFn + 3) * clkPeriodI2C));
	tmpMaxSDADEL = (double)((double)pTHdDatMax[idx] - riseTimeCalc - (1 * (260 / (double)pow((double)10, 9)))
		            - (double)((DNFn + 4) * clkPeriodI2C));
	if( tmpMaxSDADEL < 0 ) {
		tmpMaxSDADEL = 0;
	}
	if( tmpMinSDADEL < 0 ) {
		tmpMinSDADEL = 0;
	}

	tmpMinSCLDEL = (double)((double)riseTimeCalc + pTSuDatMin[idx]);
	if( tmpMinSCLDEL < 0 ) {
		tmpMinSCLDEL = 0;
	}

	nbValid = 0;

	tmpPRESC = 99;
	// End of timing handling

	// SDLDEL and SCLDEL calculation
	for( int i1 = 0; i1 < PRESC_LENGTH; i1++ ) {

		for( int i3 = 0; i3 < SCLDEL_LENGTH; i3++ ) {
			tmpSCLDEL = (double)((pSCLDEL[i3] + 1) * (double)((pPRESC[i1] + 1) * clkPeriodI2C));

			for( int i2 = 0; i2 < SCLDEL_LENGTH; i2++ ) {
				tmpSDADEL = (double)(pSDADEL[i2] * (double)((pPRESC[i1] + 1) * clkPeriodI2C));
				if( (tmpSDADEL >= tmpMinSDADEL) && (tmpSDADEL <= tmpMaxSDADEL) && (tmpSCLDEL >= tmpMinSCLDEL) ) {
					if( pPRESC[i1] != tmpPRESC ) {
						pPRESCvalid[nbValid] = pPRESC[i1];
						tmpPRESC = pPRESCvalid[nbValid];
						nbValid = nbValid + 1;
						if( tabIndex < MAX_I2C_MODEL_NB ) {
							pPrescSDLDELSCLDELpossComb[tabIndex].prescaler = i1;
							pPrescSDLDELSCLDELpossComb[tabIndex].SDLDEL=i2;
							pPrescSDLDELSCLDELpossComb[tabIndex].SCLDEL=i3;
							pPrescSDLDELSCLDELpossComb[tabIndex].resultat=1;
							prescSDLDELSCLDELpossCombSize++;
						}
						// else MAX_I2C_MODEL_NB should be incremented or use dynamic memory
						tabIndex++;
					}
				}
			}
		}
	}

	// SCLL SCLH calculation
	int l = 0;
	double solution = 0;
	double prescR = 99;
	int sdadel = 0;
	int scldel = 0;
	int i1Sel = 0;
	int i2Sel = 0;
	int i3Sel = 0;
	double errorTarget = 0.2;
	double tsync = (double)delayFilter + (2 * clkPeriodI2C);
	Brg_I2cModelT s;

	if( nbValid != 0 ) {
		for( int i3 = nbValid - 1; i3 >= 0; i3-- ) {
			for( int i1 = 0; i1 < SCLL_LENGTH; i1++ ) {
				for( int i2 = 0; i2 < SCLH_LENGTH; i2++ ) {
					double presc = (double)(pPRESCvalid[i3] + 1) * (double)clkPeriodI2C;
					double tSclLow = (double)(pSCLL[i1] + 1) * (double)presc;
					double tSclHigh = (double)(pSCLH[i2] + 1) * (double)presc;

					tSclLow = (double)tSclLow + (double)tsync;
					tSclHigh = (double)tSclHigh + (double)tsync;
					double tScl = (double)tSclLow + (double)tSclHigh + (double)riseTimeCalc + (double)fallTimeCalc;
					double speed = 1 / (double)tScl;

					if( (speed >= clkMin) && (speed <= clkMax) && (tSclLow >= pTLowMin[idx]) &&
					    (tSclHigh >= pTHighMin[idx]) && (clkPeriodI2C < ((tSclLow - delayFilter) / 4)) &&
					    (clkPeriodI2C < tSclHigh) ) {
						double errorTmp = (double)(speed - targetFreqI2C) / (double)(targetFreqI2C);
						if( errorTmp < 0 ) {
							double x = (double)(0 - (double)errorTmp);
							s.prescaler = i1;
							s.SDLDEL=i2;
							s.SCLDEL=i3;
							s.resultat=x;
						} else {
							s.prescaler = i1;
							s.SDLDEL=i2;
							s.SCLDEL=i3;
							s.resultat=errorTmp;
						}
					} else {
						s.prescaler = i1;
						s.SDLDEL=i2;
						s.SCLDEL=i3;
						s.resultat=1;
					}

					if( (s.resultat <= errorTarget) && (pPRESCvalid[i3] <= prescR) ) {
						prescR = pPRESCvalid[i3];
						solution = 1;
						errorTarget = (double)s.resultat;
						i1Sel = i1;
						i2Sel = i2;
						i3Sel = i3;
					}
				}
			}
		}
		int x = 0;
		for( int i2 = 0; i2 < 16; i2++ ) {
			for( int i3 = 0; i3 < 16; i3++ ) {
				for( int j = 0; j < prescSDLDELSCLDELpossCombSize; j++ ) {
					if( (pPrescSDLDELSCLDELpossComb[j].resultat == 1) &&
					    (pPrescSDLDELSCLDELpossComb[j].prescaler == pPRESCvalid[i3Sel]) &&
					    (pPrescSDLDELSCLDELpossComb[j].SDLDEL == i2) &&
					    (pPrescSDLDELSCLDELpossComb[j].SCLDEL == i3) &&
					    (x == 0) ) {
						l = pPRESCvalid[i3Sel];
						sdadel = i2;
						scldel = i3;
						x++;
					}
				}
			}
		}
	}
	// Get results
	*pTimingReg=0;
	if( solution==1 ) {
		*pTimingReg = (uint32_t) ((uint32_t)pPRESCvalid[i3Sel]<<28 | // Bits 31:28 PRESC[3:0]: Timing prescaler
					(uint32_t)scldel<<20 | // Bits 27:24 Reserved, must be kept at reset value.
		                                   // Bits 23:20 SCLDEL[3:0]: Data setup time
					(uint32_t)sdadel<<16 | // Bits 19:16 SDADEL[3:0]: Data hold time
					(uint32_t)i2Sel<<8 | // Bits 15:8 SCLH[7:0]: SCL high period (master mode)
					(uint32_t)i1Sel); // Bits 7:0 SCLL[7:0]: SCL low period (master mode)
		brgStatus = BRG_NO_ERR;
	} else {
		brgStatus = BRG_PARAM_ERR;
	}

    delete [] pSCLL;
    delete [] pSCLH;
	delete [] pSCLDEL;
    delete [] pSDADEL;
    delete [] pPRESC;
    delete [] pPRESCvalid;
	delete [] pPrescSDLDELSCLDELpossComb;
    delete [] pTFallMax;
    delete [] pTRiseMax;
    delete [] pTHdDatMax;
    delete [] pTSuDatMin;
    delete [] pTLowMin;
    delete [] pTHighMin;

	return brgStatus;
}
/**
 * @ingroup I2C
 * @brief This routine allows to receive bytes on the I2C interface, in the mode initialized by Brg::InitI2C().\n
 * In master mode I2C, the size bytes are received in one I2C transaction ((re)start- addr- data -stop)
 * even if data may be split into several USB packets.
 * @param[in]  Addr  I2C slave address used in master mode (default 7bit): 
 *             use #I2C_10B_ADDR(Addr) if it is a 10bit address.
 * @param[in]  pBuffer Pointer on data buffer with data to be read.
 * @param[in]  SizeInBytes Data size to be read in bytes (min 1, max data buffer size)
 * @param[out] pSizeRead If not NULL and in case of error, pSizeRead returns the number of bytes
 *             received before the error.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If I2C is not initialized
 * @retval #BRG_I2C_ERR In case of I2C error
 * @retval #BRG_COM_CMD_ORDER_ERR If low level I2C function call order is not consitent (Start, Cont, Stop)
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::ReadI2C(uint8_t *pBuffer, uint16_t Addr, uint16_t SizeInBytes, uint16_t *pSizeRead)
{
	return ReadI2Ccmd(pBuffer, Addr, SizeInBytes, I2C_FULL_RW_TRANS, pSizeRead, NULL);
}
/**
 * @ingroup I2C
 * @brief Same as Brg::ReadI2C(uint8_t *pBuffer, uint16_t Addr, uint16_t SizeInBytes, uint16_t *pSizeRead)
 *        except for Addr parameter
 * @param[in]  Addr Is the I2C slave address used in master mode.
 * @param[in]  AddrMode Indicate if it is a 10bit or 7bit address.
 * @param[in]  pBuffer     See Brg::ReadI2C() above
 * @param[in]  SizeInBytes See Brg::ReadI2C() above
 * @param[out] pSizeRead   See Brg::ReadI2C() above
 */
Brg_StatusT Brg::ReadI2C(uint8_t *pBuffer, uint16_t Addr, Brg_I2cAddrModeT AddrMode,
                         uint16_t SizeInBytes, uint16_t *pSizeRead)
{
	 uint16_t slaveAddr = Addr; // default bit15=0 7b
	 if( AddrMode == I2C_ADDR_10BIT ) {
		slaveAddr = I2C_10B_ADDR(Addr); // set bit15 to 1 for 10b
	 }
	 return ReadI2C(pBuffer, slaveAddr, SizeInBytes, pSizeRead);
}
/*
 * See Brg::ReadI2C header
 * RwTransType gives the type of I2C transaction to perform
 * - if not NULL, pErrorInfo currently unused
 */
Brg_StatusT Brg::ReadI2Ccmd(uint8_t *pBuffer, uint16_t Addr,
                            uint16_t SizeInBytes, Brg_I2cRWTransfer RwTransType,
                            uint16_t *pSizeRead, uint32_t *pErrorInfo)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( (pBuffer == NULL) || (SizeInBytes==0) ) {
		return BRG_PARAM_ERR;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_READ_I2C;
	pRq->CDBByte[2] = (uint8_t)SizeInBytes;
	pRq->CDBByte[3] = (uint8_t)(SizeInBytes>>8);
	pRq->CDBByte[4] = (uint8_t)Addr;
	pRq->CDBByte[5] = (uint8_t)(Addr>>8);
	pRq->CDBByte[6] = (uint8_t)RwTransType;

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->BufferLength = SizeInBytes;
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = pBuffer;

	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, NULL, DEFAULT_TIMEOUT);

	delete pRq;

	if( brgStat == BRG_NO_ERR )
	{
		brgStat = GetLastReadWriteStatus(pSizeRead, pErrorInfo);
	}

	if( brgStat != BRG_NO_ERR ) {
		LogTrace("I2C Error (%d) in ReadI2C (%d bytes)", (int)brgStat,(int)SizeInBytes);
		if( pSizeRead != NULL ) {
			LogTrace("I2C Only %d bytes read without error",(int)*pSizeRead);
		}
	}
#ifdef USING_TRACELOG
	else {
		if(SizeInBytes==4) {
			LogTrace("I2C R: 0x%08lx", (unsigned long)*pBuffer);
		} else {
			LogTrace("I2C R %d bytes", (int)SizeInBytes);
		}
	}
#endif

	return brgStat;
}
/**
 * @ingroup I2C
 * @brief This low level routine allows to split I2C transaction and to receive bytes on the I2C interface,
 * in the mode initialized by Brg::InitI2C().\n 
 * For a full I2C transaction use Brg::ReadI2C() instead.\n
 * Master mode I2C transaction is splitted: only (re)start, Addr are sent and data are received (no stop sent).
 * @warning After Brg::StartReadI2C() it is mandatory to call Brg::StopReadI2C() (optionally Brg::ContReadI2C()).
 * @param[in]  Addr  I2C slave address used in master mode (default 7bit): 
 *             use #I2C_10B_ADDR(Addr) if it is a 10bit address.
 * @param[in]  pBuffer Pointer on data buffer with data to be read.
 * @param[in]  SizeInBytes Data size to be read in bytes (min 1, max data buffer size)
 * @param[out] pSizeRead If not NULL and in case of error, pSizeRead returns the number of bytes
 *             received before the error.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If I2C is not initialized
 * @retval #BRG_I2C_ERR In case of I2C error
 * @retval #BRG_COM_CMD_ORDER_ERR If low level I2C function call order is not consitent (Start, Cont, Stop)
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::StartReadI2C(uint8_t *pBuffer, uint16_t Addr, uint16_t SizeInBytes, uint16_t *pSizeRead)
{
	Brg_StatusT status;
	m_slaveAddrPartialI2cTrans = Addr;
	status = ReadI2Ccmd(pBuffer, Addr, SizeInBytes, I2C_START_RW_TRANS, pSizeRead, NULL);
	return status;
}
/**
 * @ingroup I2C
 * @brief Same as Brg::StartReadI2C(uint8_t *pBuffer, uint16_t Addr, uint16_t SizeInBytes, uint16_t *pSizeRead)
 *        except for Addr parameter
 * @param[in]  Addr Is the I2C slave address used in master mode.
 * @param[in]  AddrMode Indicate if it is a 10bit or 7bit address.
 * @param[in]  pBuffer     See Brg::StartReadI2C() above
 * @param[in]  SizeInBytes See Brg::StartReadI2C() above
 * @param[out] pSizeRead   See Brg::StartReadI2C() above
 */
Brg_StatusT Brg::StartReadI2C(uint8_t *pBuffer, uint16_t Addr, Brg_I2cAddrModeT AddrMode,
                              uint16_t SizeInBytes, uint16_t *pSizeRead)
{
	 uint16_t slaveAddr = Addr; // default bit15=0 7b
	 if( AddrMode == I2C_ADDR_10BIT ) {
		slaveAddr = I2C_10B_ADDR(Addr); // set bit15 to 1 for 10b
	 }
	 return StartReadI2C(pBuffer, slaveAddr, SizeInBytes, pSizeRead);
}
/**
 * @ingroup I2C
 * @brief This low level routine allows to split I2C transaction and to receive bytes on the I2C interface,
 * in the mode initialized by Brg::InitI2C().\n 
 * For a full I2C transaction use Brg::ReadI2C() instead.\n
 * Master mode I2C transaction is splitted: only data are received (no start-address or stop sent).
 * @warning Before Brg::ContReadI2C() it is mandatory to call Brg::StartReadI2C().\n
 *          After Brg::ContReadI2C() it is mandatory to call Brg::StopReadI2C().
 * @param[in]  pBuffer Pointer on data buffer with data to be read.
 * @param[in]  SizeInBytes Data size to be read in bytes (min 1, max data buffer size)
 * @param[out] pSizeRead If not NULL and in case of error, pSizeRead returns the number of bytes
 *             received before the error.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If I2C is not initialized
 * @retval #BRG_I2C_ERR In case of I2C error
 * @retval #BRG_COM_CMD_ORDER_ERR If low level I2C function call order is not consitent (Start, Cont, Stop)
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::ContReadI2C(uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeRead)
{
	Brg_StatusT status;
	status = ReadI2Ccmd(pBuffer, m_slaveAddrPartialI2cTrans, SizeInBytes, I2C_CONT_RW_TRANS, pSizeRead, NULL);
	return status;
}
/**
 * @ingroup I2C
 * @brief This low level routine allows to split I2C transaction and to receive bytes on the I2C interface,
 * in the mode initialized by Brg::InitI2C().\n 
 * For a full I2C transaction use Brg::ReadI2C() instead.\n
 * Master mode I2C transaction is splitted: only data are received and stop sent (no start-address sent).
 * @warning Before Brg::StopReadI2C() it is mandatory to call Brg::StartReadI2C() (optionally Brg::ContReadI2C()).
 * @param[in]  pBuffer Pointer on data buffer with data to be read.
 * @param[in]  SizeInBytes Data size to be read in bytes (min 1, max data buffer size)
 * @param[out] pSizeRead If not NULL and in case of error, pSizeRead returns the number of bytes
 *             received before the error.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If I2C is not initialized
 * @retval #BRG_I2C_ERR In case of I2C error
 * @retval #BRG_COM_CMD_ORDER_ERR If low level I2C function call order is not consitent (Start, Cont, Stop)
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::StopReadI2C(uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeRead)
{
	Brg_StatusT status;
	status = ReadI2Ccmd(pBuffer, m_slaveAddrPartialI2cTrans, SizeInBytes, I2C_STOP_RW_TRANS, pSizeRead, NULL);
	return status;
}
/**
 * @ingroup I2C
 * @brief This routine allows to receive bytes on the I2C interface without blocking the
 *        USB for I2C transaction completion.\n
 * Main use case if for I2C transaction requiring timeout higher than 200ms to complete.
 * Difference with Brg::ReadI2C() are:
 * - Command answer is a status not the data.
 * - I2C transaction at target side is not completed when receiving the status answer.
 * - BUSY is returned in the status while I2C transaction at target side is not completed,
 * in BUSY state only Brg:GetLastReadWriteStatus() command can be sent to the firmware.
 * - Once status is no more BUSY, data can be retrieved using Brg:GetReadDataI2C().
 * - Data size limited to 512 bytes.
 * @warning If other commands than STLINK_BRIDGE_GET_RWCMD_STATUS are sent to the STLink firmware
 *  while it is in BUSY state this can break the USB communication.
 *
 * @param[in]  Addr  I2C slave address used in master mode (default 7bit): 
 *             use #I2C_10B_ADDR(Addr) if it is a 10bit address.
 * @param[in]  SizeInBytes Data size to be read in bytes: min 1, max 512 bytes.
 * @param[out] pSizeRead If not NULL and in case of error, pSizeRead returns the number of bytes
 *             received before the error.
 * @param[in]  CmdTimeoutMs 0: #DEFAULT_CMD_TIMEOUT (~200ms), val>0: timeout to wait for
 *             the command completion, value in ms (must be a multiple of 200ms, up to 50s).
 *             While command is not completed or CmdTimeoutMs not elapsed STLink firmware is BUSY.
 *
 * @retval #BRG_CMD_BUSY If I2C transaction is not completed, use Brg:GetLastReadWriteStatus() until no more BUSY
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If I2C is not initialized
 * @retval #BRG_I2C_ERR In case of I2C error
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::ReadNoWaitI2C(uint16_t Addr, uint16_t SizeInBytes, uint16_t *pSizeRead, uint16_t CmdTimeoutMs)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint8_t targetCmdTimeout = 0; // Default timeout
	uint16_t answer[BRIDGE_RW_STATUS_LEN_WORD]={0,0,0,0};

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( IsReadNoWaitI2CSupport() == false ) {
		// Command supported starting firmware version B3
		return BRG_CMD_NOT_SUPPORTED;
	}
	if( (SizeInBytes > 512) || (SizeInBytes==0) ) {
		return BRG_PARAM_ERR;
	}

	// Timeout param: timeout for BUSY state at target side
	if( CmdTimeoutMs != DEFAULT_CMD_TIMEOUT ) {
		if(CmdTimeoutMs > 50000) { // trunk to 50s (max uint8_t 0xFF = 256)
			targetCmdTimeout = 250; // 50s (250*200ms)
		} else {
			targetCmdTimeout = (uint8_t)((CmdTimeoutMs + 199)/200); // CmdTimeoutMs = targetCmdTimeout*200ms
			                                                        // (rounded to 200ms superior)
		}
	}
	if( SizeInBytes==0 ) {
		return BRG_NO_ERR;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_READ_NO_WAIT_I2C;
	pRq->CDBByte[2] = (uint8_t)SizeInBytes;
	pRq->CDBByte[3] = (uint8_t)(SizeInBytes>>8);
	pRq->CDBByte[4] = (uint8_t)Addr;
	pRq->CDBByte[5] = (uint8_t)(Addr>>8);
	pRq->CDBByte[6] = (uint8_t)I2C_FULL_RW_TRANS;
	pRq->CDBByte[7] = (uint8_t)targetCmdTimeout;

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->BufferLength = BRIDGE_RW_STATUS_LEN_BYTE; // 8
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = answer;

	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, NULL, DEFAULT_TIMEOUT);

	delete pRq;

	if( brgStat == BRG_NO_ERR ) // answer is same format as GetLastReadWriteStatus()
	{
		brgStat = AnalyzeStatus(&answer[0]);
		if( pSizeRead != NULL ) {
			*pSizeRead = answer[1];
		}
		// unused (uint32_t)answer[2] | (uint32_t)answer[3]<<16;
	}

	if( brgStat == BRG_CMD_BUSY ) {
		LogTrace("I2C (Busy) (%d) in ReadNoWaitI2C (%d bytes)", (int)brgStat,(int)SizeInBytes);
	} else if( brgStat != BRG_NO_ERR ) {
		LogTrace("I2C Error (%d) in ReadNoWaitI2C (%d bytes)", (int)brgStat,(int)SizeInBytes);
		if( pSizeRead != NULL ) {
			LogTrace("I2C Only %d bytes read without error",(int)*pSizeRead);
		}
	}
#ifdef USING_TRACELOG
	else {
		LogTrace("I2C R no wait %d bytes", (int)SizeInBytes);
	}
#endif

	return brgStat;
}
/**
 * @ingroup I2C
 * @brief Same as Brg::ReadNoWaitI2C(uint8_t *pBuffer, uint16_t Addr, uint16_t SizeInBytes, uint16_t *pSizeRead)
 *        except for Addr parameter
 * @param[in]  Addr Is the I2C slave address used in master mode.
 * @param[in]  AddrMode Indicate if it is a 10bit or 7bit address.
 * @param[in]  SizeInBytes see Brg::ReadNoWaitI2C() above
 * @param[out] pSizeRead  see Brg::ReadNoWaitI2C() above
 * @param[in]  CmdTimeoutMs  see Brg::ReadNoWaitI2C() above
 */
Brg_StatusT Brg::ReadNoWaitI2C(uint16_t Addr, Brg_I2cAddrModeT AddrMode,
                               uint16_t SizeInBytes, uint16_t *pSizeRead, uint16_t CmdTimeoutMs)
{
	 uint16_t slaveAddr = Addr; // Default bit15=0 7b
	 if( AddrMode == I2C_ADDR_10BIT ) {
		slaveAddr = I2C_10B_ADDR(Addr); // Set bit15 to 1 for 10b
	 }
	 return ReadNoWaitI2C(slaveAddr, SizeInBytes, pSizeRead, CmdTimeoutMs);
}
/**
 * @ingroup I2C
 * @brief This routine allows to get the data read on I2C with Brg::ReadNoWaitI2C() once status is no more BUSY.
 * Status is retrieved using Brg::GetLastReadWriteStatus() (#BRG_CMD_BUSY is answered in BUSY state).
 * @warning If other commands than STLINK_BRIDGE_GET_RWCMD_STATUS are sent to the STLink firmware
 *  while it is in BUSY state this can break the USB communication.
 *
 * @param[in]  pBuffer Pointer on data buffer with data to be read.
 * @param[in]  SizeInBytes Data size to be read in bytes must be same as the one used in Brg::ReadNoWaitI2C()
 *             (min 1, max data buffer size).
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If I2C is not initialized
 * @retval #BRG_I2C_ERR In case of I2C error
 * @retval #BRG_NO_ERR If no error
	 */
Brg_StatusT Brg::GetReadDataI2C(uint8_t *pBuffer, uint16_t SizeInBytes)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( IsReadNoWaitI2CSupport() == false ) {
		// Command supported starting firmware version B3
		return BRG_CMD_NOT_SUPPORTED;
	}
	if( (pBuffer == NULL) || (SizeInBytes > 512) ) {
		return BRG_PARAM_ERR;
	}
	if( SizeInBytes==0 ) {
		return BRG_NO_ERR;
	}
	// send get status to be sure we are not in busy must be done outside this command
	// using: brgStat = GetLastReadWriteStatus(NULL, NULL)
	brgStat = BRG_NO_ERR;

	if( brgStat == BRG_NO_ERR )
	{
		pRq = new STLink_DeviceRequestT;
		memset(pRq, 0, sizeof(STLink_DeviceRequestT));

		pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
		pRq->CDBByte[1] = STLINK_BRIDGE_GET_READ_DATA_I2C;
		pRq->CDBByte[2] = (uint8_t)SizeInBytes;
		pRq->CDBByte[3] = (uint8_t)(SizeInBytes>>8);

		pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
		pRq->BufferLength = SizeInBytes;
		pRq->InputRequest = REQUEST_READ_1ST_EPIN;
		pRq->Buffer = pBuffer;

		pRq->SenseLength=DEFAULT_SENSE_LEN;

		brgStat = SendRequestAndAnalyzeStatus(pRq, NULL, DEFAULT_TIMEOUT);

		delete pRq;

		if( brgStat != BRG_NO_ERR ) {
			LogTrace("I2C Error (%d) in ReadI2C (%d bytes)", (int)brgStat,(int)SizeInBytes);
		}
#ifdef USING_TRACELOG
		else {
			if(SizeInBytes==4) {
				LogTrace("I2C R: 0x%08lx", (unsigned long)*pBuffer);
			} else {
				LogTrace("I2C R %d bytes", (int)SizeInBytes);
			}
		}
#endif
	}

	return brgStat;
}
/**
 * @ingroup I2C
 * @brief This routine allows to transmit bytes on the I2C interface, in the mode initialized by Brg::InitI2C().\n
 * In master mode I2C the size bytes are transmitted in one I2C transaction ((re)start- addr- data -stop)
 * even if data may be split into several USB packets.
 * @param[in]  Addr  I2C slave address used in master mode (default 7bit): 
 *             use #I2C_10B_ADDR(Addr) if it is a 10bit address.
 * @param[in]  pBuffer Pointer on data buffer with data to be read.
 * @param[in]  SizeInBytes Data size to be read in bytes (min 1, max data buffer size).
 * @param[out] pSizeWritten If not NULL and in case of error, pSizeWritten returns the number of bytes
 *             transmitted before the error.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If I2C is not initialized
 * @retval #BRG_I2C_ERR In case of I2C error
 * @retval #BRG_COM_CMD_ORDER_ERR If low level I2C function call order is not consitent (Start, Cont, Stop)
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::WriteI2C(const uint8_t *pBuffer, uint16_t Addr, uint16_t SizeInBytes, uint16_t *pSizeWritten)
{
	return WriteI2Ccmd(pBuffer, Addr, SizeInBytes, I2C_FULL_RW_TRANS, pSizeWritten, NULL);
}
/**
 * @ingroup I2C
 * @brief Same as WriteI2C(uint8_t *pBuffer, uint16_t Addr, uint16_t Size, uint16_t *pSizeWritten)
 *        except for Addr parameter
 * @param[in]  Addr Is the I2C slave address used in master mode.
 * @param[in]  AddrMode Indicate if it is a 10bit or 7bit address.
 * @param[in]  pBuffer     See Brg::WriteI2C() above
 * @param[in]  SizeInBytes See Brg::WriteI2C() above
 * @param[out] pSizeWritten See Brg::WriteI2C() above
 */
Brg_StatusT Brg::WriteI2C(const uint8_t *pBuffer, uint16_t Addr, Brg_I2cAddrModeT AddrMode,
						  uint16_t SizeInBytes, uint16_t *pSizeWritten)
{
	 uint16_t slaveAddr = Addr; // default bit15=0 7b
	 if( AddrMode == I2C_ADDR_10BIT ) {
		slaveAddr = I2C_10B_ADDR(Addr); // set bit15 to 1 for 10b
	 }
	 return WriteI2C(pBuffer, slaveAddr, SizeInBytes, pSizeWritten);
}
/*
 * See Brg::WriteI2C header
 * RwTransType gives the type of I2C transaction to perform
 * - if not NULL, pErrorInfo currently unused
 */
Brg_StatusT Brg::WriteI2Ccmd(const uint8_t *pBuffer, uint16_t Addr,
                             uint16_t Size, Brg_I2cRWTransfer RwTransType,
                             uint16_t *pSizeWritten, uint32_t *pErrorInfo)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( (pBuffer == NULL) || (Size == 0) ) {
		return BRG_PARAM_ERR;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));
	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_WRITE_I2C;
	pRq->CDBByte[2] = (uint8_t)Size;
	pRq->CDBByte[3] = (uint8_t)(Size>>8);
	pRq->CDBByte[4] = (uint8_t)Addr;
	pRq->CDBByte[5] = (uint8_t)(Addr>>8);
	pRq->CDBByte[6] = (uint8_t)RwTransType;
	// 	pRq->CDBByte[7] unused 0
	if( Size > 4 ) {
		// First bytes to transfer to target
		for( int i = 0; i<4; i++ ) {
			pRq->CDBByte[8+i] = pBuffer[i];
		}
		pRq->BufferLength = Size - 4;
		pRq->InputRequest = REQUEST_WRITE_1ST_EPOUT;
		pRq->Buffer = (void *)&pBuffer[4];
	} else {
		// First bytes to transfer to target
		for( int i = 0; i<Size; i++ ) {
			pRq->CDBByte[8+i] = pBuffer[i];
		}
		// If less than 4 bytes all data are sent inside the cmd
		// Just send the cmd (no data follows)
		pRq->BufferLength = 0;
		pRq->InputRequest = REQUEST_READ_1ST_EPIN;
		pRq->Buffer = NULL;
	}

	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, NULL, DEFAULT_TIMEOUT);

	delete pRq;

	if( brgStat == BRG_NO_ERR )
	{
		brgStat = GetLastReadWriteStatus(pSizeWritten, pErrorInfo);
	}

	if( brgStat != BRG_NO_ERR ) {
		LogTrace("I2C Error (%d) in WriteI2C (%d bytes)", (int)brgStat,(int)Size);
		if( pSizeWritten != NULL ) {
			LogTrace("I2C Only %d bytes written without error",(int)*pSizeWritten);
		}
	}
	return brgStat;
}
/**
 * @ingroup I2C
 * @brief This low level routine allows to split I2C transaction and to send bytes on the I2C interface,
 * in the mode initialized by Brg::InitI2C().\n
 * For a full I2C transaction use Brg::WriteI2C() instead.\n
 * Master mode I2C transaction is splitted: only (re)start- addr- data are sent (no stop).
 * @param[in]  Addr  I2C slave address used in master mode (default 7bit): 
 *             use #I2C_10B_ADDR(Addr) if it is a 10bit address.
 * @param[in]  pBuffer Pointer on data buffer with data to be read.
 * @param[in]  SizeInBytes Data size to be read in bytes (min 1, max data buffer size).
 * @param[out] pSizeWritten If not NULL and in case of error, pSizeWritten returns the number of bytes
 *             transmitted before the error.
 * @retval #BRG_NO_ERR If no error
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If I2C is not initialized
 * @retval #BRG_I2C_ERR In case of I2C error
 * @retval #BRG_COM_CMD_ORDER_ERR If low level I2C function call order is not consitent (Start, Cont, Stop)
 */
Brg_StatusT Brg::StartWriteI2C(const uint8_t *pBuffer, uint16_t Addr, uint16_t SizeInBytes, uint16_t *pSizeWritten)
{
	Brg_StatusT status;
	m_slaveAddrPartialI2cTrans = Addr;
	status = WriteI2Ccmd(pBuffer, Addr, SizeInBytes, I2C_START_RW_TRANS, pSizeWritten, NULL);
	return status;
}
/**
 * @ingroup I2C
 * @brief Same as Brg::StartWriteI2C(uint8_t *pBuffer, uint16_t Addr, uint16_t SizeInBytes, uint16_t *pSizeWritten)
 *        except for Addr parameter
 * @param[in]  Addr Is the I2C slave address used in master mode.
 * @param[in]  AddrMode Indicate if it is a 10bit or 7bit address.
 * @param[in]  pBuffer      See Brg::StartWriteI2C() above
 * @param[in]  SizeInBytes  See Brg::StartWriteI2C() above
 * @param[out] pSizeWritten See Brg::StartWriteI2C() above
 */
Brg_StatusT Brg::StartWriteI2C(const uint8_t *pBuffer, uint16_t Addr, Brg_I2cAddrModeT AddrMode,
                            uint16_t SizeInBytes, uint16_t *pSizeWritten)
{
	 uint16_t slaveAddr = Addr; // default bit15=0 7b
	 if( AddrMode == I2C_ADDR_10BIT ) {
		slaveAddr = I2C_10B_ADDR(Addr); // set bit15 to 1 for 10b
	 }
	 return StartWriteI2C(pBuffer, slaveAddr, SizeInBytes, pSizeWritten);
}
/**
 * @ingroup I2C
 * @brief This low level routine allows to split I2C transaction and to send bytes on the I2C interface,
 * in the mode initialized by Brg::InitI2C().\n
 * For a full I2C transaction use Brg::WriteI2C() instead.\n
 * Master mode I2C transaction is splitted: only (re)start- addr- data are sent (no stop).
 * @warning Before Brg::ContWriteI2C() it is mandatory to call Brg::StartWriteI2C().\n
 *          After Brg::ContWriteI2C() it is mandatory to call Brg::StopWriteI2C().
 * @param[in]  pBuffer Pointer on data buffer with data to be read.
 * @param[in]  SizeInBytes Data size to be read in bytes (min 1, max data buffer size).
 * @param[out] pSizeWritten If not NULL and in case of error, pSizeWritten returns the number of bytes
 *             transmitted before the error.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If I2C is not initialized
 * @retval #BRG_I2C_ERR In case of I2C error
 * @retval #BRG_COM_CMD_ORDER_ERR If low level I2C function call order is not consitent (Start, Cont, Stop)
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::ContWriteI2C(const uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeWritten)
{
	Brg_StatusT status;
	status = WriteI2Ccmd(pBuffer, m_slaveAddrPartialI2cTrans, SizeInBytes, I2C_CONT_RW_TRANS, pSizeWritten, NULL);
	return status;
}
/**
 * @ingroup I2C
 * @brief This low level routine allows to split I2C transaction and to send bytes on the I2C interface,
 * in the mode initialized by Brg::InitI2C().\n
 * For a full I2C transaction use Brg::WriteI2C() instead.\n
 * Master mode I2C transaction is splitted: only data and stop are sent (no start-address).
 * @warning Before Brg::StopWriteI2C() it is mandatory to call Brg::StartWriteI2C() (optionally Brg::ContWriteI2C()).
 * @param[in]  pBuffer Pointer on data buffer with data to be read.
 * @param[in]  SizeInBytes Data size to be read in bytes (min 1, max data buffer size).
 * @param[out] pSizeWritten If not NULL and in case of error, pSizeWritten returns the number of bytes
 *             transmitted before the error.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If I2C is not initialized
 * @retval #BRG_I2C_ERR In case of I2C error
 * @retval #BRG_COM_CMD_ORDER_ERR If low level I2C function call order is not consitent (Start, Cont, Stop)
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::StopWriteI2C(const uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeWritten)
{
	Brg_StatusT status;
	status = WriteI2Ccmd(pBuffer, m_slaveAddrPartialI2cTrans, SizeInBytes, I2C_STOP_RW_TRANS, pSizeWritten, NULL);
	return status;
}
/**
 * @ingroup CAN
 * @brief This routine initializes the CAN according to init parameters.\n
 * Refer to STLink firmware upgrade release note (stsw-link007) for supported init configurations.\n
 * Call Brg::GetCANbaudratePrescal() to get prescaler parameter according to required CAN bitrate.
 * @warning Prescaler parameter is valid while STLink frequency is not changed on debug interface
 *          by using STLINK_SWITCH_STLINK_FREQ.
 * @param[in]  pInitParams CAN initialization parameters see #Brg_CanInitT and \ref LIMITATION
 * @param[in]  InitType  #BRG_INIT_FULL for normal case (CLK, reset, Filter deactivated) or \n
 *                       #BRG_REINIT for configuration change only.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_PARAM_ERR Null pointer parameter
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::InitCAN(const Brg_CanInitT *pInitParams, Brg_InitTypeT InitType)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint16_t status;
	const Brg_CanBitTimeConfT* pBitTimeConf;
	uint8_t conf;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( pInitParams == NULL ) {
		return BRG_PARAM_ERR;
	}
	pBitTimeConf = &pInitParams->BitTimeConf;
	// Check BitTimeConf parameters
	if( (pBitTimeConf->PropSegInTq < 1) || (pBitTimeConf->PropSegInTq > 8) || // PROP_SEG = 1 to 8 time quantum
		(pBitTimeConf->PhaseSeg1InTq < 1) || (pBitTimeConf->PhaseSeg1InTq > 8) || // PHASE_SEG1 = 1 to 8 time quantum
		(pBitTimeConf->PhaseSeg2InTq < 1) || (pBitTimeConf->PhaseSeg2InTq > 8) || // PHASE_SEG2 = 1 to 8 time quantum
		(pBitTimeConf->SjwInTq < 1) || (pBitTimeConf->SjwInTq > 4) ) {// Synchro Jump Width = 1 to 4 time quantum
		return BRG_PARAM_ERR;
	}
	// Check prescaler
	if( (pInitParams->Prescaler < 1) || (pInitParams->Prescaler > 1024) ) {
		return BRG_PARAM_ERR;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_INIT_CAN;
	// Can Mode
	pRq->CDBByte[2] = (uint8_t) pInitParams->Mode;
	// Bit Time segment1 BS1 (PROP_SEG + PHASE_SEG1)  CAN_1_time_quantum = 0
	// Bit 2-0: segment1 (PHASE_SEG1)
	// Bit 5-3: propagation (PROP_SEG)
	pRq->CDBByte[3] = (uint8_t)(((pBitTimeConf->PhaseSeg1InTq -1)&0x07) | (((pBitTimeConf->PropSegInTq-1)<<3)&0x38));
	// Bit Time segment2 BS2 (PHASE_SEG2) and SJW (Synchronisation Jump Width) CAN_1_time_quantum = 0
	// Bit 2-0:  segment2 (BS2)
	// Bit 4-3: SJW
	pRq->CDBByte[4] = (uint8_t)(((pBitTimeConf->PhaseSeg2InTq-1)&0x07) | (((pBitTimeConf->SjwInTq-1)<<3)&0x18));
	// Configuration
	// Bit0 reserved: 0
	// Bit1 Abom : Enable (1) or disable (0) the automatic bus-off management
	// Bit2 Awum : Enable (1) or disable (0) the automatic wake-up mode
	// Bit3 Nart : Enable (1) or disable (0) the no-automatic retransmission mode
	// Bit4 Rflm : Enable (1) or disable (0) the Receive FIFO Locked mode
	// Bit5 Txfp : Enable (1) or disable (0) the transmit FIFO priority
	conf = 0;
	if( pInitParams->bIsAbomEn == true) {
		conf |= 1<<1;
	}
	if( pInitParams->bIsAwumEn == true) {
		conf |= 1<<2;
	}
	if( pInitParams->bIsNartEn == true) {
		conf |= 1<<3;
	}
	if( pInitParams->bIsRflmEn == true) {
		conf |= 1<<4;
	}
	if( pInitParams->bIsTxfpEn == true) {
		conf |= 1<<5;
	}
	pRq->CDBByte[5] = conf;
	// BaudRate Prescaler
	pRq->CDBByte[6] = (uint8_t) (pInitParams->Prescaler)&0xFF;
	pRq->CDBByte[7] = (uint8_t) (pInitParams->Prescaler>>8)&0xFF;
	// Init Type
	pRq->CDBByte[8] = (uint8_t) InitType;

	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = &status;
	pRq->BufferLength = 2;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, &status);
	delete pRq;

	return brgStat;
}
/**
 * @ingroup CAN
 * @brief This routine gets the CAN  prescaler needed for Brg::InitCAN() according to required CAN bitrate.\n
 * Refer to STLink firmware upgrade release note (stsw-link007) for supported init configurations.
 * @warning pPrescal parameter is valid while STLink frequency is not changed on debug interface
 *          by using STLINK_SWITCH_STLINK_FREQ.
 * @param[in]  pBitTimeConf  CAN Bit Time configuration that will also be used in Brg::CAN_Init().
 * @param[in]  ReqBaudrate  Requested baudRate (max 1000000 1Mbps).
 * @param[out] pPrescal  Prescaler to be used in Brg::InitCAN() parameters for the given bitrate and 
 *                       bit Time configuration.\n
 *                       If found prescaler is <1 baudrate or N need to be decreased,
 *                       if if found prescaler is >1024 baudrate or N need to be increased.\n
 *    Tq = time quanta \n
 *    BaudRate = 1/NominalBitTime \n
 *    <table>
 *    <tr><td colspan="4">Nominal bit Time
 *    <tr><td>sync_seg      <td>prop_seg      <td>phase_seg1    <td>phase_seg2  
 *    <tr><td>1xTq          <td>1 to 8 xTq    <td>1 to 8 xTq    <td>1 to 8 xTq   
 *    <tr><td>              <td colspan="2">Tbs1                <td>Tbs2
 *    </table>
 *    NominalBitTime = Tsync_seg + Tprop_seg + Tphase_seg1 + Tphase_seg2 \n
 *                   = N x Tq \n
 *    Tq = Prescaler x TcanClk \n
 *    Prescaler = 1/ (N x BaudRate x TcanClk) = FcanClk / (N x BaudRate) \n
 * @param[out] pFinalBaudrate  Found baudRate if #BRG_COM_FREQ_MODIFIED
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_PARAM_ERR If null pointer parameter or incorrect #Brg_CanBitTimeConfT
 * @retval #BRG_COM_FREQ_MODIFIED If ReqBaudrate is different from pFinalBaudrate
 * @retval #BRG_COM_FREQ_NOT_SUPPORTED If found prescaler is not in the correct range (1-1024)
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::GetCANbaudratePrescal(const Brg_CanBitTimeConfT *pBitTimeConf, uint32_t ReqBaudrate,
                                       uint32_t *pPrescal, uint32_t *pFinalBaudrate) 
{
	Brg_StatusT brgStat;
	uint32_t canInputClkKHz=0;
	uint32_t stlHClkKHz=0;
	uint32_t n=1, calcPrescal=0; // 0 if error

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( (pBitTimeConf == NULL) || (pPrescal == NULL) || (pFinalBaudrate == NULL) ) {
		return BRG_PARAM_ERR;
	}
	// Check Brg_CanBitTimeConfT parameters
	if( (pBitTimeConf->PropSegInTq < 1) || (pBitTimeConf->PropSegInTq > 8) || // PROP_SEG = 1 to 8 time quantum
		(pBitTimeConf->PhaseSeg1InTq < 1) || (pBitTimeConf->PhaseSeg1InTq > 8) || // PHASE_SEG1 = 1 to 8 time quantum
		(pBitTimeConf->PhaseSeg2InTq < 1) || (pBitTimeConf->PhaseSeg2InTq > 8) || // PHASE_SEG2 = 1 to 8 time quantum
		(pBitTimeConf->SjwInTq < 1) || (pBitTimeConf->SjwInTq > 4) ) {// Synchro Jump Width = 1 to 4 time quantum
		return BRG_PARAM_ERR;
	}
	// Check requested BaudRate (max 1Mb/s)
	if( (ReqBaudrate < 1) || (ReqBaudrate > 1000000) ) {
		return BRG_PARAM_ERR;
	}
	// Get the current CAN input Clk
	brgStat = GetClk(COM_CAN, &canInputClkKHz, &stlHClkKHz);
	if( brgStat == BRG_NO_ERR ) {
		/* Bit timing
		  The bit timing logic monitors the serial bus-line and performs sampling and adjustment of
		  the sample point by synchronizing on the start-bit edge and resynchronizing on the following edges.
		  Its operation may be explained simply by splitting nominal bit time into three segments as follows:
		  - Synchronization segment (SYNC_SEG): a bit change is expected to occur within this
		  time segment. It has a fixed length of one time quantum (1 x tq).
		  - Bit segment 1 (BS1): defines the location of the sample point. It includes the
		  PROP_SEG and PHASE_SEG1 of the CAN standard. Its duration is programmable
		  between 1 and 16 time quanta but may be automatically lengthened to compensate for
		  positive phase drifts due to differences in the frequency of the various nodes of the network.
		  - Bit segment 2 (BS2): defines the location of the transmit point. It represents the
		  PHASE_SEG2 of the CAN standard. Its duration is programmable between 1 and 8
		  time quanta but may also be automatically shortened to compensate for negative phase drifts.
		  The resynchronization Jump Width (SJW) defines an upper bound to the amount of
		  lengthening or shortening of the bit segments. It is programmable between 1 and 4 time quanta
		  Tq = time quanta
		  BaudRate = 1/NominalBitTime
		  <--------------- Nominal bit Time --------------------------------->
		  | sync_seg |    prop_seg     |    phase_seg1    |    phase_seg2    |
		  <-- 1xTq --><- 1 to 8 x Tq -><-- 1 to 8 x Tq  --><-- 1 to 8 x Tq -->
					  <-------- Tbs1 ---------------------><----- Tbs2 ------>
		  NominalBitTime = Tsync_seg + Tprop_seg + Tphase_seg1 + Tphase_seg2
						 = 1xTq + Tbs1 +Tbs2
						 = NxTq
		  Tq = Prescaler x TcanClk
		  Prescaler = 1/ (N x BaudRate x TcanClk) = FcanClk / (N x BaudRate)
		*/
		// Calculate Prescaler = 1/ (N x BaudRate x TcanClk) = FcanClk / (N x BaudRate)
		// N = 1 + PROP_SEG + PHASE_SEG1 + PHASE_SEG2
		n = 1 + pBitTimeConf->PropSegInTq + pBitTimeConf->PhaseSeg1InTq + pBitTimeConf->PhaseSeg2InTq;
		calcPrescal = canInputClkKHz * 1000 / (n * ReqBaudrate);
		*pFinalBaudrate = canInputClkKHz * 1000 / (n * calcPrescal);
		if( ReqBaudrate < *pFinalBaudrate ) {
			calcPrescal ++;
			*pFinalBaudrate = canInputClkKHz * 1000 / (n * calcPrescal);
		}
		// prescaler must be between 1 and 1024
		if( (calcPrescal < 1)||(calcPrescal > 1024) ) {
			// if found prescaler is <1 baudrate or N need to be decreased,
			// if found prescaler is >1024 baudrate or N need to be increased.
			brgStat = BRG_COM_FREQ_NOT_SUPPORTED;
		} else if( ReqBaudrate != *pFinalBaudrate ) {
			brgStat = BRG_COM_FREQ_MODIFIED; // ReqBaudrate is different from pFinalBaudrate
		}
	}

	*pPrescal = calcPrescal;
	return brgStat;
}
/*
 * Internal: Fill Filter or Mask Id pOutConf[3:0] according to pInConf as follow
 * 32bit: [31:21] = Id[10:0], [20:3]= Id[28:11], [2]=IDE, [1]=RTR, [0]=0
 * pOutConf must be initialized to 0 before
 * return BRG_PARAM_ERR if input ID is not coherennt with IDE bit
 */
Brg_StatusT Brg::FormatFilter32bitCAN(const Brg_FilterBitsT *pInConf, uint8_t *pOutConf)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
	// FilterId/Mask format default 0: Id=0, CAN_ID_STANDARD, CAN_DATA_FRAME
	if( pInConf->RTR == CAN_REMOTE_FRAME ) {
		pOutConf[0] |= 1<<1; //bit1 RTR
	}
	if( pInConf->IDE == CAN_ID_EXTENDED ) {
		pOutConf[0] |= 1<<2; //bit2 IDE
		if( pInConf->ID > 0x1FFFFFFF ) {
			brgStat = BRG_PARAM_ERR;
		}
	} else if( pInConf->ID > 0x7FF ) { // CAN_ID_STANDARD
		brgStat = BRG_PARAM_ERR;
	}
	pOutConf[0] |= (uint8_t)(((pInConf->ID>>11)<<3)&0xF8); // [7:3]= Id[15:11]
	pOutConf[1] |= (uint8_t)((pInConf->ID>>16)&0xFF); // [15:8]= Id[23:16]
	pOutConf[2] |= (uint8_t)((pInConf->ID>>24)&0x1F); // [20:16]= Id[28:24]
	pOutConf[2] |= (uint8_t)(((pInConf->ID)<<5)&0xE0); // [23:21] = Id[2:0]
	pOutConf[3] |= (uint8_t)((pInConf->ID>>3)&0xFF); // [31:24] = Id[10:3]

	return brgStat;
}
/*
 * Internal: Fill Filter or Mask Id pOutConf[1:0] according to pInConf as follow
 * 16bit: [15:5] = Id[10:0], [4]=IDE, [3]=RTR, [2:0]= Id[28:26]
 * pOutConf must be initialized to 0 before
 * return BRG_PARAM_ERR if input ID is not coherennt with IDE bit
 */
Brg_StatusT Brg::FormatFilter16bitCAN(const Brg_FilterBitsT *pInConf, uint8_t *pOutConf)
{
	// filterId/Mask format default 0: Id=0, CAN_ID_STANDARD, CAN_DATA_FRAME
	Brg_StatusT brgStat = BRG_NO_ERR;
	// filterId/Mask format default 0: Id=0, CAN_ID_STANDARD, CAN_DATA_FRAME
	if( pInConf->RTR == CAN_REMOTE_FRAME ) {
		pOutConf[0] |= 1<<4; //bit4 RTR
	}
	if( pInConf->IDE == CAN_ID_EXTENDED ) {
		pOutConf[0] |= 1<<3; //bit3 IDE
		if( pInConf->ID > 0x1FFFFFFF ) {
			brgStat = BRG_PARAM_ERR;
		}
	} else if( pInConf->ID > 0x7FF ) { // CAN_ID_STANDARD
		brgStat = BRG_PARAM_ERR;
	}
	pOutConf[0] |= (uint8_t)((pInConf->ID>>26)&0x07); // [2:0]= Id[28:26]
	pOutConf[0] |= (uint8_t)(((pInConf->ID)<<5)&0xE0); // [7:5]= Id[2:0]
	pOutConf[1] |= (uint8_t)((pInConf->ID>>3)&0xFF); // [15:8]= Id[10:3]

	return brgStat;	
}
/**
 * @ingroup CAN
 * @brief This routine initializes the CAN Filters according to init parameters.\n
 * 14 filters can be configured.
 * @param[in]  pInitParams CAN filter configuration see #Brg_CanFilterConfT for details.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If CAN is not initialized
 * @retval #BRG_CAN_ERR In case of CAN error
 * @retval #BRG_PARAM_ERR For null pInitParams or incorrect pInitParams fields
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::InitFilterCAN(const Brg_CanFilterConfT *pInitParams)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint16_t status;
	uint8_t filterConf = 0; // Default DISABLED CAN_FILTER_16BIT CAN_FILTER_ID_MASK CAN_MSG_RX_FIFO0
	uint8_t filterId[4] = {0,0,0,0}, filterMask[4] = {0,0,0,0};

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( pInitParams == NULL ) {
		return BRG_PARAM_ERR;
	}
	// Check Filter number
	if( pInitParams->FilterBankNb > 13 ) {
		return BRG_PARAM_ERR;
	}

	// Filter configuration
	// Default 0: DISABLED CAN_FILTER_16BIT CAN_FILTER_ID_MASK CAN_MSG_RX_FIFO0
	if( pInitParams->FilterMode == CAN_FILTER_ID_LIST) {
		filterConf |= 1; //bit0
	}
	if( pInitParams->FilterScale == CAN_FILTER_32BIT) {
		filterConf |= 1<<1; //bit1
	}
	if( pInitParams->bIsFilterEn == true) {
		filterConf |= 1<<2; //bit2
	}
	if( pInitParams->AssignedFifo == CAN_MSG_RX_FIFO1) {
		filterConf |= 1<<3; //bit3
	}
	// FilterId/Mask format default 0: Id=0, CAN_ID_STANDARD, CAN_DATA_FRAME
	// 32bit: [31:21] = Id[10:0], [20:3]= Id[28:11], [2]=IDE, [1]=RTR, [0]=0
	// 16bit: [15:5] = Id[10:0], [4]=IDE, [3]=RTR, [2:0]= Id[28:26]
	if( pInitParams->FilterScale == CAN_FILTER_32BIT) {
		// FilterId/Mask format:
		// 32bit: [31:21] = Id[10:0], [20:3]= Id[28:11], [2]=IDE, [1]=RTR, [0]=0
		// ID0 in FilterIdHigh+Low
		brgStat = FormatFilter32bitCAN(&pInitParams->Id[0], &filterId[0]);
		if( brgStat == BRG_NO_ERR ) {
			if( pInitParams->FilterMode == CAN_FILTER_ID_MASK ) { // Mask0 in FilterMaskHigh+Low
				brgStat = FormatFilter32bitCAN(&pInitParams->Mask[0], &filterMask[0]);
			} else { //CAN_FILTER_ID_LIST, ID1 in FilterMaskHigh+Low
				brgStat = FormatFilter32bitCAN(&pInitParams->Id[1], &filterMask[0]);
			}
		}
	} else { // CAN_FILTER_16BIT
		// FilterId/Mask format:
		// 16bit: [15:5] = Id[10:0], [4]=IDE, [3]=RTR, [2:0]= Id[28:26]
		// ID0 in FilterIdHigh
		brgStat = FormatFilter16bitCAN(&pInitParams->Id[0], &filterId[2]);
		if( pInitParams->FilterMode == CAN_FILTER_ID_MASK ) {
			if( brgStat == BRG_NO_ERR ) { // Mask0 in MaskIdHigh
				brgStat = FormatFilter16bitCAN(&pInitParams->Mask[0], &filterMask[2]);
			}
			if( brgStat == BRG_NO_ERR ) { // ID1 in FilterIdLow
				brgStat = FormatFilter16bitCAN(&pInitParams->Id[1], &filterId[0]);
			}
			if( brgStat == BRG_NO_ERR ) { // Mask1 in MaskIdLow
				brgStat = FormatFilter16bitCAN(&pInitParams->Mask[1], &filterMask[0]);
			}
		} else { //CAN_FILTER_ID_LIST
			if( brgStat == BRG_NO_ERR ) { // ID1 in FilterIdLow
				brgStat = FormatFilter16bitCAN(&pInitParams->Id[1], &filterId[0]);
			}
			if( brgStat == BRG_NO_ERR ) { // ID2 in MaskIdHigh
				brgStat = FormatFilter16bitCAN(&pInitParams->Id[2], &filterMask[2]);
			}
			if( brgStat == BRG_NO_ERR ) { // ID3 in MaskIdLow
				brgStat = FormatFilter16bitCAN(&pInitParams->Id[3], &filterMask[0]);
			}
		}
	}
	if( brgStat != BRG_NO_ERR ) {
		return brgStat;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_INIT_FILTER_CAN;

	// Filter configuration
	pRq->CDBByte[2] = filterConf;
	// FilterIdLow
	pRq->CDBByte[3] = filterId[0];
	pRq->CDBByte[4] = filterId[1];
	// FilterIdHigh
	pRq->CDBByte[5] = filterId[2];
	pRq->CDBByte[6] = filterId[3];
	// FilterMaskLow
	pRq->CDBByte[7] = filterMask[0];
	pRq->CDBByte[8] = filterMask[1];
	// FilterMaskHigh
	pRq->CDBByte[9] = filterMask[2];
	pRq->CDBByte[10] = filterMask[3];
	// Filter Bank number
	pRq->CDBByte[11] = pInitParams->FilterBankNb;

	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = &status;
	pRq->BufferLength = 2;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, &status);
	delete pRq;

	return brgStat;
}
/**
 * @ingroup CAN
 * @brief This routine activates the CAN RX messages reception storage at STLink side.\n
 * Brg::StartMsgReceptionCAN() must be called before Brg::GetRxMsgNbCAN() and Brg::GetRxMsgCAN().\n
 * CAN init must have been called before and CAN filter activation is needed 
 * to be able to receive CAN messages. 
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If CAN is not initialized
 * @retval #BRG_CAN_ERR In case of CAN error
 * @retval #BRG_PARAM_ERR If message format is different between host and firmware
 * @retval #BRG_CMD_NOT_SUPPORTED If firmware is too old for this command
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::StartMsgReceptionCAN(void)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint8_t answer[4];

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( IsCanSupport() == false ) {
		// Command not supported on first FW bridge version (B1)
		return BRG_CMD_NOT_SUPPORTED;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_START_MSG_RECEPTION_CAN;
	pRq->CDBByte[2] = CAN_MSG_FORMAT_V1;

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->BufferLength = 4;
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = answer;

	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, (uint16_t*)answer);
	if( (answer[2] != CAN_MSG_FORMAT_V1)&&(brgStat == BRG_NO_ERR) ) { //robustness
		StopMsgReceptionCAN();
		brgStat = BRG_PARAM_ERR;
	}
	if( brgStat != BRG_NO_ERR ) {
		LogTrace("CAN Error (%d) in StartMsgReceptionCAN (firmware msg format: %d, host format: %d)",
		         (int)brgStat, (int)answer[2], (int)CAN_MSG_FORMAT_V1);
	}

	delete pRq;
	return brgStat;
}
/**
 * @ingroup CAN
 * @brief This routine disables the CAN RX messages storage at STLink side.\n
 * CAN messages will still be received by the STLink but won't be stored. 
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If CAN is not initialized
 * @retval #BRG_CAN_ERR In case of CAN error
 * @retval #BRG_CMD_NOT_SUPPORTED If firmware is too old for this command
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::StopMsgReceptionCAN(void)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint16_t status;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( IsCanSupport() == false ) {
		// Command not supported on first FW bridge version (B1)
		return BRG_CMD_NOT_SUPPORTED;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_STOP_MSG_RECEPTION_CAN;

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->BufferLength = 2;
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = &status;

	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, &status);

	delete pRq;
	return brgStat;
}
/**
 * @ingroup CAN
 * @brief This routine gets the number of received CAN messages available 
 * to be retrieved with Brg::GetRxMsgCAN().
 * @param[out]  pMsgNb  Filled with the number of available RX CAN messages at STLink side.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If CAN is not initialized
 * @retval #BRG_CAN_ERR In case of CAN error
 * @retval #BRG_PARAM_ERR If NULL pointer
 * @retval #BRG_CMD_NOT_SUPPORTED If firmware is too old for this command
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::GetRxMsgNbCAN(uint16_t *pMsgNb)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint8_t answer[8];

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( IsCanSupport() == false ) {
		// Command not supported on first FW bridge version (B1)
		return BRG_CMD_NOT_SUPPORTED;
	}
	if( pMsgNb == NULL ) {
		return BRG_PARAM_ERR;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_GET_NB_RXMSG_CAN;

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->BufferLength = 8;
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = answer;

	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, (uint16_t*)answer);
	*pMsgNb = (uint16_t)(answer[2] | (((uint16_t)answer[3]<<8)&0xFF00));
	if( (answer[4] != CAN_MSG_FORMAT_V1)&&(brgStat == BRG_NO_ERR) ) { //robustness
		brgStat = BRG_PARAM_ERR;
	}

	delete pRq;
	return brgStat;
}
/**
 * @ingroup CAN
 * @brief This routine allows to get the available CAN messages receieved through the CAN interface 
 * and stored by the STLink.\n
 * The number of available messages (MsgNb) MUST be retrieved first with Brg::GetRxMsgNbCAN().
 * @param[out]  pCanMsg pointer on array of at least MsgNb message "header": #Brg_CanRxMsgT.
 *                       Each message fields are updated as described in #Brg_CanRxMsgT.
 * @param[in]  MsgNb MUST be the value returned by Brg::GetRxMsgNbCAN(). 
 *                    It may be less than the value returned by Brg::GetRxMsgNbCAN()
 *                    only if the messages are retrieved with several call to Brg::GetRxMsgCAN().\n
 *                    pCanMsg must point on a #Brg_CanRxMsgT array with at least MsgNb element.\n
 * @warning     If MsgNb > Brg::GetRxMsgNbCAN(), firmware will return status error (2bytes) instead of requested 
 *              messages this may break the USB com.
 * @param[out]  pBuffer  Data buffer for read data (filled only for data frame with DLC>0).
 * @param[out]  BufSizeInBytes  Is less or equal to real data buffer size in bytes.
 *                              It indicates pBuffer available size for read data.\n
 *                Note: data size that can be received is less or equal to MsgNb*8 (DLC max =8).\n
 *                If BufSizeInBytes is smaller than the sum of all DLC only BufSizeInBytes are copied into pBuffer.
 * @param[out]  pDataSizeInBytes  Filled with the number of data written in pBuffer (in bytes).
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If CAN is not initialized
 * @retval #BRG_CAN_ERR In case of CAN error
 * @retval #BRG_PARAM_ERR If incorrect MsgNb (0 or greater than available messages) or NULL pointer
 * @retval #BRG_CMD_NOT_SUPPORTED If firmware is too old for this command
 * @retval #BRG_MEM_ALLOC_ERR If memory allocation for message answer failed
 * @retval #BRG_OVERRUN_ERR if overrun is detected in at least 1 message
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::GetRxMsgCAN(Brg_CanRxMsgT *pCanMsg, uint16_t MsgNb, uint8_t *pBuffer,
                             uint16_t BufSizeInBytes, uint16_t *pDataSizeInBytes)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint8_t *pAnswer;
	uint8_t *pReadCanMsg;
	uint16_t msgDataSize, buffDataSize, buffDataOffset;
	uint32_t answerSize, firstErrMsgNb;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( IsCanSupport() == false ) {
		// Command not supported on first FW bridge version (B1)
		return BRG_CMD_NOT_SUPPORTED;
	}
	if( (pCanMsg == NULL) || (pBuffer == NULL) || (pDataSizeInBytes == NULL) || (MsgNb < 1) ) {
		return BRG_PARAM_ERR;
	}

	*pDataSizeInBytes = 0; // Default
	answerSize = MsgNb*CAN_READ_MSG_SIZE_V1;
	pAnswer = new uint8_t[answerSize];
	if( pAnswer == NULL ) {
		return BRG_MEM_ALLOC_ERR;
	}
	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_GET_RXMSG_CAN;
	pRq->CDBByte[2] = (uint8_t)MsgNb;
	pRq->CDBByte[3] = (uint8_t)((MsgNb>>8)&0xFF);

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->BufferLength = answerSize;
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = pAnswer;

	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, NULL);

	delete pRq;

	// Warning if MsgNb is not correct, a 2 bytes error status is received from the FW instead
	// of answerSize bytes, this is a host issue and can lead to USB com err or wrongly
	// interpreted answer
	if( brgStat == BRG_NO_ERR ) {
		uint8_t overrunErr;
		pReadCanMsg = &pAnswer[0]; //First received message
		buffDataSize = BufSizeInBytes;
		buffDataOffset = 0;
		for( int j=0; j<MsgNb; j++ ) {
			// Fill pCanMsg and pBuffer with read data
			pCanMsg[j].ID = (uint32_t)pReadCanMsg[0] | (((uint32_t)pReadCanMsg[1])<<8) |
                            (((uint32_t)pReadCanMsg[2])<<16) | (((uint32_t)pReadCanMsg[3])<<24);
			// byte4 message type
			if( (pReadCanMsg[4]&0x1) == 0) { // byte4 bit0 IDE
				pCanMsg[j].IDE = CAN_ID_STANDARD;
			} else {
				pCanMsg[j].IDE = CAN_ID_EXTENDED;
			}
			if( (pReadCanMsg[4]&(0x1<<2)) == 0) { // byte4 Bit2 FIFONumber
				pCanMsg[j].Fifo = CAN_MSG_RX_FIFO0;
			} else {
				pCanMsg[j].Fifo = CAN_MSG_RX_FIFO1;
			}
			overrunErr = (pReadCanMsg[4]>>3)&0x3; // byte4 Bit3-4 Overrun
			if( overrunErr != 0 ) {
				// Overrun has occurred before this msg
				if( overrunErr == 1 ) { // CAN fifo overrun err (1)
					pCanMsg[j].Overrun = CAN_RX_FIFO_OVERRUN;
				} else { // Buffer overrun error (2)
					pCanMsg[j].Overrun = CAN_RX_BUFF_OVERRUN;
				}
				if( brgStat == BRG_NO_ERR ) {
					brgStat = BRG_OVERRUN_ERR;
					firstErrMsgNb = j;
					LogTrace("CAN Overrun Error in GetRxMsgCAN (first error %d at %d/%d msg)",
                             (int)overrunErr, (int)firstErrMsgNb, (int)MsgNb);
				}
			} else { // Else no overrun error
				pCanMsg[j].Overrun = CAN_RX_NO_OVERRUN;
			}
			// Byte5 DLC
			pCanMsg[j].DLC = pReadCanMsg[5];
			if( (pReadCanMsg[4]&0x2) == 0) { // byte4 bit1 RTR
				pCanMsg[j].RTR = CAN_DATA_FRAME;
				if( buffDataSize >= pCanMsg[j].DLC ) {
					msgDataSize = pCanMsg[j].DLC;
				} else {
					msgDataSize = buffDataSize; // limit copied data to max buffer size
					if( brgStat == BRG_NO_ERR ) {
						brgStat = BRG_OVERRUN_ERR;
						LogTrace("CAN Data Error in GetRxMsgCAN: BufSizeInBytes too small (error at %d/%d msg)",
						         (int)j, (int)MsgNb);
					}
				}
			} else {
				pCanMsg[j].RTR = CAN_REMOTE_FRAME;
				msgDataSize = 0; // no data to copy in case of RTR message
			}
			// Byte6-7: Message time stamp unused
			pCanMsg[j].TimeStamp = 0;
			// Byte 8 to 15: 0 to 8 data bytes
			for( int i=0; i<msgDataSize; i++ ) {
				pBuffer[buffDataOffset+i] = pReadCanMsg[CAN_READ_MSG_HEADER_SIZE_V1+i];
			}
			// Point on next message and update the number of remaining data to copy
			pReadCanMsg += CAN_READ_MSG_SIZE_V1;
			buffDataSize -= msgDataSize;
			buffDataOffset += msgDataSize;
		} // End of read Can msg loop
		*pDataSizeInBytes = buffDataOffset;
	}

	if( brgStat != BRG_NO_ERR ) {
		LogTrace("CAN Error (%d) in GetRxMsgCAN (max %d bytes, %d msg)",
		         (int)brgStat, (int)BufSizeInBytes, (int)MsgNb);
	}

#ifdef USING_TRACELOG
	else {
		if(BufSizeInBytes==4) { //BufSizeInBytes
			LogTrace("CAN R %d msg: 0x%08lx", (int)MsgNb, (unsigned long)*pBuffer);
		} else {
			LogTrace("CAN R %d msg, %d bytes", (int)MsgNb, (int)BufSizeInBytes);
		}
	}
#endif

	delete [] pAnswer;
	return brgStat;
}
/**
 * @ingroup CAN
 * @brief This routine allows to send a message on CAN bus through the CAN interface,
 * in the mode initialized by InitCAN().\n
 * @param[in]   pCanMsg Pointer on a message "header" see #Brg_CanTxMsgT description.
 * @param[in]   pBuffer  Pointer to the data buffer (must be at least size length).
 * @param[in]   SizeInBytes  Number of data bytes to send (max 8 bytes).
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_COM_INIT_NOT_DONE If CAN is not initialized
 * @retval #BRG_CAN_ERR In case of CAN error
 * @retval #BRG_PARAM_ERR If NULL pointer
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::WriteMsgCAN(const Brg_CanTxMsgT *pCanMsg, const uint8_t *pBuffer, uint8_t SizeInBytes)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint8_t msgType, msgDLC;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( (pCanMsg == NULL) || (pBuffer == NULL) ) {
		return BRG_PARAM_ERR;
	}
	if( (pCanMsg->DLC > 8) || (SizeInBytes > 8) ) { // CAN message DATA FIELD = 8 bytes max
			return BRG_PARAM_ERR;
	}

	msgType = 0; // Default CAN_ID_STANDARD CAN_DATA_FRAME, bit2=0 reserved
	if( pCanMsg->IDE == CAN_ID_EXTENDED ) {
		msgType |= 1; // bit0
		if( pCanMsg->ID > 0x1FFFFFFF ) { // 29bits ID
			return BRG_PARAM_ERR;
		}
	} else if( pCanMsg->ID > 0x7FF ) { // CAN_ID_STANDARD 11bits ID
			return BRG_PARAM_ERR;
	}
	if( pCanMsg->RTR == CAN_REMOTE_FRAME ) {
		msgType |= 1<<1; // bit1
		msgDLC = pCanMsg->DLC;
	} else {
		msgDLC = SizeInBytes;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));
	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_WRITE_MSG_CAN;
	// ID
	pRq->CDBByte[2] = (uint8_t)(pCanMsg->ID);
	pRq->CDBByte[3] = (uint8_t)(pCanMsg->ID>>8);
	pRq->CDBByte[4] = (uint8_t)(pCanMsg->ID>>16);
	pRq->CDBByte[5] = (uint8_t)(pCanMsg->ID>>24);
	// IDE, RTR 
	pRq->CDBByte[6] = msgType;
	pRq->CDBByte[7] = (uint8_t)msgDLC;
	if( SizeInBytes > 4 ) {
		// First bytes to transfer to target
		for( int i = 0; i<4; i++ ) {
			pRq->CDBByte[8+i] = pBuffer[i];
		}
		pRq->BufferLength = SizeInBytes - 4;
		pRq->InputRequest = REQUEST_WRITE_1ST_EPOUT;
		pRq->Buffer = (void *)&pBuffer[4];
	} else {
		// First bytes to transfer to target
		for( int i = 0; i<SizeInBytes; i++ ) {
			pRq->CDBByte[8+i] = pBuffer[i];
		}
		// If less than 4 bytes all data are sent inside the cmd
		// Just send the cmd (no data follows)
		pRq->BufferLength = 0;
		pRq->InputRequest = REQUEST_READ_1ST_EPIN;
		pRq->Buffer = NULL;
	}

	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, NULL);

	delete pRq;

	if( brgStat == BRG_NO_ERR )
	{	// pSizeWritten not useful for CAN, pErrorInfo currently unused
		brgStat = GetLastReadWriteStatus(NULL, NULL);
	}

	if( brgStat != BRG_NO_ERR ) {
		LogTrace("CAN Error (%d) in WriteMsgCAN (%d bytes)", (int)brgStat,(int)SizeInBytes);
	}
	return brgStat;
}
/**
 * @ingroup BRIDGE
 * @brief This routine gets status of last (Read/Write)(SPI/I2C/CAN) command.
 * Used internally be classic Read/Write bridge function and need to be called by user
 * in some I2C cases.
 * @warning In #BRG_CMD_BUSY case, this is the only command that can be sent to the STLink.
 *
 * @param[out] pBytesWithoutError If not NULL and in case of error, returns the number of bytes
 *             transmitted/received before the error (status != #BRG_NO_ERR).
 * @param[out] pErrorInfo Currently not significant (=0 if pErrorInfo!=NULL)
 *
 * @return All possible read/write errors
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_CMD_BUSY STLink is BUSY state: need to poll later again until BUSY no more returned
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::GetLastReadWriteStatus(uint16_t *pBytesWithoutError, uint32_t *pErrorInfo)
{
	uint16_t answer[BRIDGE_RW_STATUS_LEN_WORD]={0,0,0,0};
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));
	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_GET_RWCMD_STATUS;
	pRq->BufferLength = BRIDGE_RW_STATUS_LEN_BYTE; // 8
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = answer;

	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, (uint16_t *)answer);

	if( (pBytesWithoutError != NULL) && (brgStat != BRG_NO_ERR) ) {
		*pBytesWithoutError = answer[1];
	}
	if( (pErrorInfo != NULL) && (brgStat != BRG_NO_ERR) ) {
		*pErrorInfo = (uint32_t)answer[2] | (uint32_t)answer[3]<<16;
	}

	delete pRq;

	return brgStat;
}

// -------------------------------- GPIO ----------------------------------- //
/*
 * private: return the gpio configuration field of STLINK_BRIDGE_INIT_GPIO according to init parameter
 */
uint8_t Brg::GpioConfField(Brg_GpioConfT GpioConfParam)
{
	// return gpioConf: Bit1-0 mode, Bit3-2 speed, Bit5-4 pull, Bit6 output type
	return (uint8_t)(((uint8_t)GpioConfParam.Mode & 0x03) | ((((uint8_t)GpioConfParam.Speed) << 2) & 0x0C)
	       | ((((uint8_t)GpioConfParam.Pull) << 4) & 0x30) | ((((uint8_t)GpioConfParam.OutputType) << 6) & 0x40));
}
/**
 * @ingroup GPIO
 * @brief This routine initializes the given GPIO according to init parameters.\n
 * Refer to STLink firmware upgrade release note (stsw-link007) for supported init configurations.\n
 * @param[in]  pInitParams GPIO initialization parameters see #Brg_GpioInitT and \ref LIMITATION
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_PARAM_ERR If NULL pointer or if ConfigNb is not 1 or #BRG_GPIO_MAX_NB
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::InitGPIO(const Brg_GpioInitT *pInitParams)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint16_t status;
	uint8_t gpioConf, i;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}
	if( (pInitParams == NULL) || (pInitParams->pGpioConf == NULL) ) {
		return BRG_PARAM_ERR;
	}
	if( ((pInitParams->ConfigNb != 1) && (pInitParams->ConfigNb != BRG_GPIO_MAX_NB)) ||
		((pInitParams->GpioMask & BRG_GPIO_ALL) == 0) ) {
		return BRG_PARAM_ERR;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_INIT_GPIO;
	// GPIO nb mask (GPIO bit: 1 if used, 0 if not used)
	pRq->CDBByte[2] = pInitParams->GpioMask;
	// GPIO 0, 1, 2, 3 config, Bit1-0 mode, Bit3-2 speed, Bit5-4 pull, Bit6 output type
	if( pInitParams->ConfigNb == 1 ) { // same configuration for all GPIOs 
		gpioConf = GpioConfField(pInitParams->pGpioConf[0]);
		for( i=0; i<BRG_GPIO_MAX_NB; i++) { 
			pRq->CDBByte[3+i] = gpioConf;
		}
	} else {
		for( i=0; i<BRG_GPIO_MAX_NB; i++) {
			gpioConf = GpioConfField(pInitParams->pGpioConf[i]);
			pRq->CDBByte[3+i] = gpioConf;
		}
	}
	// pRq->CDBByte[7-15] = 0

	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = &status;
	pRq->BufferLength = 2;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, &status);
	delete pRq;

	return brgStat;
}
/**
 * @ingroup GPIO
 * @brief This routine read the GPIO(s) requested in GpioMask.
 * @param[in]  GpioMask  GPIO(s) to be read (one or several value of #Brg_GpioMaskT)
 * @param[out] pGpioVal Table of #BRG_GPIO_MAX_NB #Brg_GpioValT filled with gpio read value,
 *                        pGpioVal[0] for GPIO0, ..., pGpioVal[3] for GPIO3
 * @param[out] pGpioErrorMask  0 if no error, else corresponding GPIO bit is set to 1,
 *                    error may occur if GPIO has not been initialized previously in Brg::InitGPIO()
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_PARAM_ERR If NULL pointer or if GpioMask is 0
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::ReadGPIO(uint8_t GpioMask, Brg_GpioValT *pGpioVal, uint8_t *pGpioErrorMask)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint8_t answer[8]={0,0,0,0,0,0,0,0};

	if( (pGpioVal == NULL) || (pGpioErrorMask == NULL) || ((GpioMask & BRG_GPIO_ALL) == 0) ) {
		return BRG_PARAM_ERR;
	}

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_READ_GPIO;
	// GPIO mask
	pRq->CDBByte[2] = GpioMask;


	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = &answer;
	pRq->BufferLength = 8;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, (uint16_t*)answer);

	// Answer byte2 GPIO error mask (0 if no error, 1 if error)
	*pGpioErrorMask = answer[2];
	if( (brgStat == BRG_NO_ERR)&&(*pGpioErrorMask & GpioMask) != 0 ) {
		brgStat = BRG_GPIO_ERR;
	}
	// Answer byte3 GPIO read value (0 or 1), if GPIO is present in the mask, retrieve the read value
	for( int i=0; i<BRG_GPIO_MAX_NB; i++ ) {
		if( (GpioMask & (1<<i)) != 0 ) {
			if( (answer[3] & (1<<i)) != 0 ) { // = 1
				pGpioVal[i] = GPIO_SET;
			} else { // = 0
				pGpioVal[i] = GPIO_RESET;
			}
		}
	}
	delete pRq;

	return brgStat;
}
/**
 * @ingroup GPIO
 * @brief This routine Set or Reset the GPIO(s) requested in GpioMask.
 * @param[in]  GpioMask  GPIO(s) to be set or reset (one or several value of #Brg_GpioMaskT)
 * @param[in]  pGpioVal Table of #BRG_GPIO_MAX_NB #Brg_GpioValT containing value to write,
 *                        pGpioVal[0] for GPIO0, ..., pGpioVal[3] for GPIO3
 * @param[out] pGpioErrorMask  0 if no error, else corresponding GPIO bit is set to 1,
 *                    error may occur if GPIO has not been initialized previously in Brg::InitGPIO()
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_PARAM_ERR If NULL pointer or if GpioMask is 0
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::SetResetGPIO(uint8_t GpioMask, const Brg_GpioValT *pGpioVal, uint8_t *pGpioErrorMask)
{
	STLink_DeviceRequestT *pRq;
	Brg_StatusT brgStat;
	uint8_t answer[8]={0,0,0,0,0,0,0,0};

	if( (pGpioVal == NULL) || (pGpioErrorMask == NULL) || ((GpioMask & BRG_GPIO_ALL) == 0) ) {
		return BRG_PARAM_ERR;
	}

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return BRG_NO_STLINK;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_BRIDGE_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_BRIDGE_COMMAND;
	pRq->CDBByte[1] = STLINK_BRIDGE_SET_RESET_GPIO;
	// GPIO mask
	pRq->CDBByte[2] = GpioMask;
	// GPIO set reset mask (1 if  set, 0 if reset), if GPIO is present in the mask set the value to write.
	for( int i=0; i<BRG_GPIO_MAX_NB; i++ ) {
		if( (GpioMask & (1<<i)) != 0 ) {
			if( pGpioVal[i] == GPIO_SET ) { // = 1
				pRq->CDBByte[3] |= (uint8_t)(1<<i);
			} // If GPIO_RESET let to 0
		}
	}
	// Bytes 4-15 0 unused

	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = &answer;
	pRq->BufferLength = 8;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	brgStat = SendRequestAndAnalyzeStatus(pRq, (uint16_t*)answer);

	// Answer byte2 GPIO error mask (0 if no error, 1 if error)
	// byte3-7 unused
	*pGpioErrorMask = answer[2];
	if( (brgStat == BRG_NO_ERR)&&(*pGpioErrorMask & GpioMask) != 0 ) {
		brgStat = BRG_GPIO_ERR;
	}

	delete pRq;

	return brgStat;
}
/**
 * @ingroup BRIDGE
 * @brief This routine gets target voltage in V, computed from STLink VREFINT value (typically
 * 1.2V at 25C).
 * @warning  Requires to have target voltage connected to T_VCC (not present on
 *           Bridge connector). If T_VCC is not connected return 0V.
 * @param[out] pVoltage  Target volatge in V.
 *
 * @retval #BRG_NO_STLINK If Brg::OpenStlink() not called before
 * @retval #BRG_PARAM_ERR If NULL pointer
 * @retval #BRG_NO_ERR If no error
 */
Brg_StatusT Brg::GetTargetVoltage(float *pVoltage)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;
	ifStatus = StlinkDevice::PrivGetTargetVoltage(pVoltage);
	return ConvSTLinkIfToBrgStatus(ifStatus);
}

/**
 * @ingroup CAN
 * @retval false If FW is too old for full CAN support.
 * @note To be called after Brg::OpenStlink().
 */
bool Brg::IsCanSupport(void) const {
	// Return false if FW is too old for full CAN support
	if( ((m_Version.Major_Ver==3) && (m_Version.Bridge_Ver<FIRMWARE_BRIDGE_MIN_VER_FOR_CAN)) ) {
		return false;
	} else {
		return true;
	}
}
/**
 * @ingroup I2C
 * @retval false If FW is too old for I2C ReadNoWait support.
 * @note To be called after Brg::OpenStlink().
 */
bool Brg::IsReadNoWaitI2CSupport(void) const {
	// Return false if FW is too old for READ_NO_WAIT_I2C command support
	if( ((m_Version.Major_Ver==3) && (m_Version.Bridge_Ver<FIRMWARE_BRIDGE_MIN_VER_FOR_READ_NO_WAIT_I2C)) ) {
		return false;
	} else {
		return true;
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

