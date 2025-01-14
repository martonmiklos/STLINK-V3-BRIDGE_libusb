/**
 ******************************************************************************
 * @file    stlink_interface.h
 * @author  MCD Application Team
 * @brief   Header for stlink_interface.cpp module
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
/** @addtogroup INTERFACE
 * @{
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _STLINK_INTERFACE_H
#define _STLINK_INTERFACE_H
/* Includes ------------------------------------------------------------------*/
#include "STLinkUSBDriver.h"

#include <algorithm>
#include <libusb.h>

#ifdef USING_ERRORLOG
#include "ErrLog.h"
// Also use trace.log for test
// #define USING_TRACELOG
#endif

/* Exported types and constants ----------------------------------------------*/
// Warning if modified update also: ConvSTLinkIfToBrgStatus
/// Interface Error and Status
typedef enum {
    STLINKIF_NO_ERR = 0,          ///< OK (no error)
    STLINKIF_CONNECT_ERR,         ///< USB Connection error
    STLINKIF_DLL_ERR,             ///< USB DLL error
    STLINKIF_USB_COMM_ERR,        ///< USB Communication error
    STLINKIF_PARAM_ERR,           ///< Wrong parameters error
    STLINKIF_NO_STLINK,           ///< STLink device not opened error
    STLINKIF_NOT_SUPPORTED,       ///< Parameter error
    STLINKIF_PERMISSION_ERR,      ///< STLink device already in use by another
                                  ///< program error
    STLINKIF_ENUM_ERR,            ///< USB enumeration error
    STLINKIF_GET_INFO_ERR,        ///< Error getting STLink device information
    STLINKIF_STLINK_SN_NOT_FOUND, ///< Required STLink serial number not found
                                  ///< error
    STLINKIF_CLOSE_ERR            ///< Error during device Close
} STLinkIf_StatusT;

const uint16_t STLINK_V3_VID = 0x0483;
const uint16_t STLINK_V3_PID[] = {
    0x374E, // STLINK-V3E
    0x374F, // STLINK-V3S
    0x3753, // STLINK-V3 Dual VCP
    0x3754  // STLINK-V3 no MSD
};

/* Class -------------------------------------------------------------------- */
/// STLinkInterface Class
class STLinkInterface
{
  public:
    STLinkInterface(STLink_EnumStlinkInterfaceT IfId = STLINK_BRIDGE);

    virtual ~STLinkInterface(void);

    STLink_EnumStlinkInterfaceT
    GetIfId(void) const
    {
        return m_ifId;
    }

    STLinkIf_StatusT
    LoadStlinkLibrary(const char *pPathOfProcess);

    bool
    IsLibraryLoaded();

    STLinkIf_StatusT
    EnumDevices(uint32_t *pNumDevices, bool bClearList);

    STLinkIf_StatusT
    GetDeviceInfo2(int StlinkInstId, STLink_DeviceInfo2T *pInfo,
                   uint32_t InfoSize);

    STLinkIf_StatusT
    OpenDevice(int StlinkInstId, uint32_t StlinkIdTcp, bool bOpenExclusive,
               void **pHandle);

    STLinkIf_StatusT
    OpenDevice(const char *pSerialNumber, bool bStrict, uint32_t StlinkIdTcp,
               bool bOpenExclusive, void **pHandle);

    STLinkIf_StatusT
    CloseDevice(void *pHandle, uint32_t StlinkIdTcp);

    STLinkIf_StatusT
    SendCommand(void *pHandle, uint32_t StlinkIdTcp,
                STLink_DeviceRequestT *pDevReq, const uint16_t UsbTimeoutMs);

#ifdef USING_ERRORLOG
    void
    BindErrLog(cErrLog *pErrLog);
#endif

  private:
    libusb_context *ctx = nullptr; // a libusb session
    libusb_device *devices[256];
    ssize_t cnt;
    STLinkIf_StatusT
    EnumDevicesIfRequired(uint32_t *pNumDevices, bool bForceRenum,
                          bool bClearList);

    void
    LogTrace(const char *pMessage, ...);

    STLink_EnumStlinkInterfaceT m_ifId;
    uint32_t m_nbEnumDevices;

    // Flag for STLinkUSBDriver.dll loaded state
    bool m_bApiDllLoaded;

    // Flag for enumerating the Device (Bridge, ...) interface when required
    bool m_bDevInterfaceEnumerated;

#ifdef USING_ERRORLOG
    // Error log management
    cErrLog *m_pErrLog;
#endif

    uint32_t
    STLink_GetNbDevices(TEnumStlinkInterface IfId);
    uint32_t
    STLink_GetDeviceInfo2(TEnumStlinkInterface IfId, uint8_t DevIdxInList,
                          TDeviceInfo2 *pInfo, uint32_t InfoSize);
    uint32_t
    STLink_OpenDevice(TEnumStlinkInterface IfId, uint8_t DevIdxInList,
                      uint8_t bExclusiveAccess, void **pHandle);
    uint32_t
    STLink_CloseDevice(void *pHandle);
    uint32_t
    STLink_SendCommand(void *pHandle, PDeviceRequest pRequest,
                       uint32_t DwTimeOut);
    uint32_t
    STLink_Reenumerate(TEnumStlinkInterface IfId, uint8_t bClearList);
};

#endif //_STLINK_INTERFACE_H
// end group INTERFACE
/** @} */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
