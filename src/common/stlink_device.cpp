/**
  ******************************************************************************
  * @file    stlink_device.cpp
  * @author  MCD Application Team
  * @brief   StlinkDevice parent class used for STLink device common functionalities.
  *          Must not be used directly, child class to be instanciated (see bridge.cpp).
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
/* Includes ------------------------------------------------------------------*/
#if defined(_MSC_VER) &&  (_MSC_VER >= 1000)
#include "stdafx.h"  // first include for windows visual studio
#endif

#include "stlink_device.h"

/* Private typedef -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// used for debug trace: interface name according to STLink_EnumStlinkInterfaceT value
const char * LogInterfaceString[STLINK_NB_INTERFACES] = 
{ "DBG", "DBG2", "DBG SERVER", "BRIDGE" };

/* Global variables ----------------------------------------------------------*/

/* Class Functions Definition ------------------------------------------------*/

/*
 * StlinkDevice constructor
 */
StlinkDevice::StlinkDevice(STLinkInterface &StlinkIf): m_bStlinkConnected(false),m_bOpenExclusive(false),
	m_pStlinkInterface(&StlinkIf)
{
	m_handle = NULL;
	m_Version.Major_Ver = 0;
	m_Version.Jtag_Ver = 0;
	m_Version.Swim_Ver = 0;
	m_Version.Msc_Ver = 0;
	m_Version.Bridge_Ver = 0;
	m_Version.Power_Ver = 0; // unused
	m_Version.VID = 0;
	m_Version.PID = 0;

#ifdef USING_ERRORLOG
	// Error log management
	m_pErrLog = NULL;
#endif
}
/*
 * StlinkDevice destructor
 */
StlinkDevice::~StlinkDevice(void)
{
	// Close STLink even if failure
	PrivCloseStlink();

#ifdef WIN32 //Defined for applications for Win32 and Win64.
#ifdef USING_ERRORLOG
	// Flush the trace log system after closing
	if( m_pErrLog != NULL ) {
		m_pErrLog->Dump();
	}
#endif
#endif // WIN32
}

/*
 * Trace logging mechanism, under compilation switch USING_ERRORLOG (requiring ErrLog.h and ErrLog.cpp)
 */
void StlinkDevice::LogTrace(const char *pMessage, ...)
{
#ifdef USING_ERRORLOG
	va_list args; // used to manage the variable argument list
	va_start(args, pMessage);     
	if( m_pErrLog != NULL ) {
		m_pErrLog->LogTrace(pMessage, args);
	}
	va_end(args);     
#endif
}

/**
 * @ingroup DEVICE
 * @brief Mode for open USB connection. Used by OpenStlink().
 * @param[in] bExclusive  false: shared between applications \n
 *                        true: exclusive to 1 application: recommended for Bridge
 */
void StlinkDevice::SetOpenModeExclusive(bool bExclusive)
{
	m_bOpenExclusive = bExclusive;
}
/*
 * @brief Open current USB connection with the STLink.
 * If not already done, STLinkInterface::OpenDevice will build the device list before opening.
 *
 * @param[in]  StlinkInstId  Instance ID in the list of enumerated STLink devices.
 *                           Create one instance of StlinkDevice by device.
 * @return STLinkInterface::OpenDevice() errors
 * @retval #STLINKIF_CONNECT_ERR Wrong StlinkInstId or USB error
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT StlinkDevice::PrivOpenStlink(int StlinkInstId)
{
	STLinkIf_StatusT ifStatus=STLINKIF_NO_ERR;

	if( m_bStlinkConnected == false ) {
		// Open the device
		ifStatus = m_pStlinkInterface->OpenDevice(StlinkInstId, 0, m_bOpenExclusive, &m_handle);
		if( ifStatus != STLINKIF_NO_ERR ) {
			LogTrace("%s STLink device USB connection failure", LogInterfaceString[m_pStlinkInterface->GetIfId()]);
			return STLINKIF_CONNECT_ERR;
		}
		m_bStlinkConnected = true;
		ifStatus = PrivGetVersionExt(&m_Version);
		if( ifStatus != STLINKIF_NO_ERR )
		{
			LogTrace("STLink get Extended version failure");
			PrivCloseStlink();
			return ifStatus;
		}
		LogTrace("STLink with %s interface detected", LogInterfaceString[m_pStlinkInterface->GetIfId()]);
	}

	if( m_bStlinkConnected == false )
	{
		return STLINKIF_CONNECT_ERR;
	}
	return ifStatus;
}
/*
 * @brief Opening routine based on the serial number of the STLink; to use in case StlinkInstId is no more
 * reliable (devices have been reenumerated between STLink instance selection and opening) or
 * to choose a specific STLink by its serial number.
 *
 * @param[in]  pPathOfProcess  Path, in ASCII, where STLinkUSBDriver library
 *                             is searched when not found in current dir, for Windows.
 * @param[in]  pSerialNumber  STLink serial number (ASCII).
 * @param[in]  bStrict 	 Used if STLink with required serial number is not found, but one other STLink is found:
 *                       choose if we open or not the other STLink. \n
 *                       If bStrict is true, the command will return #STLINKIF_STLINK_SN_NOT_FOUND if the
 *                       given SerialNumber is not found.\n
 *                       If bStrict is false and the given serial number is not found and one (and only one)
 *                       STLink is found, the command will attempt to open the STLink even if
 *                       the serial number does not match.
 * @return STLinkInterface::OpenDevice() errors
 * @retval #STLINKIF_CONNECT_ERR USB error
 * @retval #STLINKIF_STLINK_SN_NOT_FOUND Serial number not found
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT StlinkDevice::PrivOpenStlink(const char *pSerialNumber, bool bStrict) {
	STLinkIf_StatusT ifStatus=STLINKIF_NO_ERR;
	STLink_DeviceInfo2T devInfo2;
	char * pEnumUniqueId = devInfo2.EnumUniqueId;

	if( pSerialNumber == NULL ) {
		LogTrace("NULL pointer for pSerialNumber in OpenStlink");
		return STLINKIF_PARAM_ERR;
	}

	if( m_pStlinkInterface == NULL ) {
		return STLINKIF_DLL_ERR;
	}

	if( m_bStlinkConnected == false )
	{
		ifStatus = m_pStlinkInterface->OpenDevice(pSerialNumber, bStrict, 0, m_bOpenExclusive, &m_handle);
		if( ifStatus == STLINKIF_NO_ERR ) {
			m_bStlinkConnected = true;
		}
	}
	return ifStatus;
}
/*
 * @brief Close STLink USB communication, with the device instance that was opened by StlinkDevice::PrivOpenStlink()
 *
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT StlinkDevice::PrivCloseStlink(void)
{
	if( m_bStlinkConnected == true )
	{
		if( (m_handle != NULL) )
		{
			if( m_pStlinkInterface != NULL ) {
				if( m_pStlinkInterface->CloseDevice(m_handle, 0) != STLINKIF_NO_ERR ) {
					LogTrace("Error closing %s USB communication", LogInterfaceString[m_pStlinkInterface->GetIfId()]);
				}
			} // else STLINKIF_DLL_ERR
		}
		// Consider the communication is closed even if error
		m_bStlinkConnected = false;
	}
	return STLINKIF_NO_ERR;
}

/*
 * @brief This routine gets USB VID and PID, and firmware version of the STLink device.
 * @param[out] pVersion Pointer filled with version information.
 *
 * @retval #STLINKIF_NO_STLINK If StlinkDevice::PrivOpenStlink() not called before
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT StlinkDevice::PrivGetVersionExt(Stlk_VersionExtT* pVersion)
{
	STLink_DeviceRequestT *pRq;
	STLinkIf_StatusT ifStatus;
	uint8_t version[12];

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after PrivOpenStlink
		return STLINKIF_NO_STLINK;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_CMD_SIZE_16;
	pRq->CDBByte[0] = ST_GETVERSION_EXT;
	pRq->CDBByte[1] = 0x80;
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = version;
	pRq->BufferLength = 12;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	// PrivGetVersionExt is called after m_bStlinkConnected=true, so we can call SendRequest
	// (preferable for semaphore management and status code analysis)
	ifStatus = SendRequest(pRq);
	delete pRq;

	if( ifStatus == STLINKIF_NO_ERR ) {
		pVersion->Major_Ver = version[0];
		pVersion->Jtag_Ver = version[2];
		pVersion->Swim_Ver = version[1];
		pVersion->Msc_Ver = version[3];
		pVersion->Bridge_Ver = version[4];
		pVersion->Power_Ver = version[5]; // unused currently
		pVersion->VID = (((uint16_t)version[9])<<8) + version[8];
		pVersion->PID = (((uint16_t)version[11])<<8) + version[10];
	}
	return ifStatus;
}
/* 
 * @brief Send a command over the USB and wait for answer.
 * Requires the STLink to be already connected (m_bStlinkConnected==true)
 * @param[in] pDevReq USB request to send.
 * @param[in] UsbTimeoutMs if 0 use default (5s) else use UsbTimeoutMs.
 * 
 * @retval #STLINKIF_USB_COMM_ERR USB error when sending the request
 * @retval #STLINKIF_PARAM_ERR Null pointer
 * @retval #STLINKIF_NO_STLINK If StlinkDevice::PrivOpenStlink() not called before
 * @retval #STLINKIF_DLL_ERR StlinkInterface not initialized
 * @retval #STLINKIF_NO_ERR If no error
*/
STLinkIf_StatusT StlinkDevice::SendRequest(STLink_DeviceRequestT *pDevReq,
											 const uint16_t UsbTimeoutMs)
{
	STLinkIf_StatusT ifStatus;

	if( pDevReq == NULL ) {
		return STLINKIF_PARAM_ERR;
	}

	if( m_bStlinkConnected == false ) {
		return STLINKIF_NO_STLINK;
	}

	if( m_pStlinkInterface == NULL ) {
		return STLINKIF_DLL_ERR;
	}

	ifStatus = m_pStlinkInterface->SendCommand(m_handle, 0, pDevReq, UsbTimeoutMs);
	if( ifStatus != STLINKIF_NO_ERR) {
		ifStatus = STLINKIF_USB_COMM_ERR;
	} else {
		ifStatus = STLINKIF_NO_ERR;
	}

	return ifStatus;
}

/*
 * @brief This routine gets target voltage in V, computed from STLink VREFINT value (typically
 * 1.2V at 25C).
 * @warning  Requires to have target voltage connected to T_VCC (not present on
 *           Bridge connector). If T_VCC is not connected return 0V.
 * @param[out] pVoltage  Target volatge in V.
 *
 * @retval #STLINKIF_NO_STLINK If StlinkDevice::PrivOpenStlink() not called before
 * @retval #STLINKIF_PARAM_ERR If NULL pointer
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT StlinkDevice::PrivGetTargetVoltage(float *pVoltage)
{
	uint32_t adcMeasures[2];
	STLink_DeviceRequestT *pRq;
	STLinkIf_StatusT ifStatus;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after PrivOpenStlink
		return STLINKIF_NO_STLINK;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_GET_TARGET_VOLTAGE;

	pRq->BufferLength = 8;
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = adcMeasures;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	ifStatus = SendRequest(pRq);

	delete pRq;

	if( ifStatus == STLINKIF_NO_ERR )
	{
		// First returned value is the ADC measure for VREFINT (according to datasheet: 1.2V);
		// the second value is Vtarget/2;
		if( (pVoltage != NULL) &&  (adcMeasures[0]!=0) ) {
			*pVoltage = 2*((float)adcMeasures[1])*(float)1.2/adcMeasures[0];
		}
	}

	return ifStatus;
}

#ifdef USING_ERRORLOG
/*
 * Associate files to be used for Error/Trace log (pErrLog must be initialized before)
 */
void StlinkDevice::BindErrLog(cErrLog *pErrLog)
{
	m_pErrLog = pErrLog;
	if( m_pStlinkInterface != NULL ) {
		m_pStlinkInterface->BindErrLog(pErrLog);
	}
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

