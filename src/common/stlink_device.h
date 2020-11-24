/**
  ******************************************************************************
  * @file    stlink_device.h
  * @author  MCD Application Team
  * @brief   Header for stlink_device.cpp module
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
/** @addtogroup DEVICE
 * @{
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _STLINK_DEVICE_H
#define _STLINK_DEVICE_H
/* Includes ------------------------------------------------------------------*/
#include "stlink_type.h"
#include "stlink_if_common.h"
#include "stlink_interface.h"
#include "stlink_fw_api_common.h"

#ifdef USING_ERRORLOG
#include "ErrLog.h"
// Also use trace.log for test
//#define USING_TRACELOG
#endif

/* Exported types and constants ----------------------------------------------*/

// ------------------------------------------------------------------------- //
/* Class -------------------------------------------------------------------- */
/// Device Class (Parent class of Brg)
class StlinkDevice
{
public:

	StlinkDevice(STLinkInterface &StlinkIf);

	virtual ~StlinkDevice(void);

	void SetOpenModeExclusive(bool bExclusive);

#ifdef USING_ERRORLOG
	void BindErrLog(cErrLog *pErrLog);
#endif

	/*
	 * Getter for m_bStlinkConnected: True if connected to an STLink (PrivOpenStlink() done)
	 */
	bool GetIsStlinkConnected(void) const {
		return m_bStlinkConnected;
	}
	/*
	 * Getter for m_Version
	 */
	uint8_t GetMajorVer(void) const {
		return m_Version.Major_Ver;
	}
	uint8_t GetStm32DbgVer(void) const {
		return m_Version.Jtag_Ver;
	}
	uint8_t GetStm8DbgVer(void) const {
		return m_Version.Swim_Ver;
	}
	uint8_t GetMscVcpVer(void) const {
		return m_Version.Msc_Ver;
	}
	uint8_t GetBridgeVer(void) const {
		return m_Version.Bridge_Ver;
	}
	uint16_t GetUsbVid(void) const {
		return m_Version.VID;
	}
	uint16_t GetUsbPid(void) const {
		return m_Version.PID;
	}

	// Flag indicating if a session is opened (public for legacy but use getter, may become private)
	bool m_bStlinkConnected;

	//STLink version (public for legacy but use getter, may become private)
	Stlk_VersionExtT  m_Version;

protected:
	STLinkIf_StatusT PrivOpenStlink(int StlinkInstId=0);
	STLinkIf_StatusT PrivOpenStlink(const char *pSerialNumber, bool bStrict);
	STLinkIf_StatusT PrivCloseStlink(void);
	STLinkIf_StatusT PrivGetVersionExt(Stlk_VersionExtT* pVersion);
	STLinkIf_StatusT PrivGetTargetVoltage(float *pVoltage);

	STLinkIf_StatusT SendRequest(STLink_DeviceRequestT *pDevReq, const uint16_t UsbTimeoutMs=0);
	void LogTrace(const char *pMessage, ...);
private:
	// Opened device handle
	void*   m_handle;

	STLinkInterface * m_pStlinkInterface;

	// Mode for device opening: shared or exclusive
	bool m_bOpenExclusive;

#ifdef USING_ERRORLOG
	// Error log management
	cErrLog *m_pErrLog;
#endif
};

#endif //_STLINK_DEVICE_H
// end group DEVICE
/** @} */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/