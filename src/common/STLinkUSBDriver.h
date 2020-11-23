/**
  ******************************************************************************
  * @file    STLinkUSBDriver.h
  * @author  MCD Application Team
  * @brief   API of ST-Link, ST-Link/V2, /V2-1 and STLINK-V3 USB driver interface 
  *         (STLinkUSBDriver.dll for Windows, libSTLinkUSBDriver.so for Linux,
  *          libSTLinkUSBDriver.dylib for Mac).
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
//******************************************************************************
// ST-Link (v1) enumerates as a mass storage device and must be associated to a disk
// drive letter in order to be handled through this driver.
//
// ST-Link/v2 is handled through its unique GUID=DBCE1CD9-A320-4b51-A365-A0C3F3C5FB29,
// assigned during enumeration with .inf file (on Windows).
// ST-Link/v2-1 and v3 are also handled via their GUID (on Windows).
//******************************************************************************
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _STLINKUSBDRIVER_H
#define _STLINKUSBDRIVER_H
/* Includes ------------------------------------------------------------------*/
#include "stlink_type.h"

/* Exported types ------------------------------------------------------------*/
#ifdef WIN32 //Defined for applications for Win32 and Win64.
#define STD_CALL __stdcall
#else
#define STD_CALL
#endif


#define  DEFAULT_SENSE_LEN  14

#define  SS_OK               0x01
#define  SS_ERR              0x04
#define  SS_NO_DEVICE        0x82
#define  SS_INVALID_HA	     0x81
#define  SS_INVALID_SRB      0xE0
#define  SS_FAILED_INIT	     0xE4
#define  SS_SKIP_STEP        0xF1

#define  SS_MEMORY_PROBLEM    0x1000
#define  SS_TIMEOUT           0x1001
#define  SS_BAD_PARAMETER     0x1002
#define  SS_OPEN_ERR          0x1003
#define  SS_TRANSFER_ERR      0x1004
#define  SS_NO_SUCH_INFO      0x1050
#define  SS_BAD_BUFFER_SIZE   0x1051
#define  SS_TRUNCATED_DATA    0x1052
#define  SS_CMD_NOT_AVAILABLE 0x1053
#define  SS_DEVICE_NOT_SUPPORTED 0x1054
#define  SS_PERMISSION_ERR   0x1055

#define  SS_FAILED           0x2000
#define  SS_TCP_ERROR        0x2001  // To be refined later
#define  SS_TCP_CANT_CONNECT 0x2002  // Error when TCP client is not able to connect to the server

#define  SS_SCSI_ERROR       0x5000
#define  SS_WIN32_ERROR      0x10000

// Size in bytes of the TCP command returned status
#define TCP_ERROR_SIZE 4

#define  DEFAULT_TIMEOUT     5000

// Constants defining the type of info to retrieve (STMass_GetDeviceInfo) - Old API required for ST-Link/V1
#define  INFOTYPE_LETTER              1000
#define  INFOTYPE_INQUIRY_STRING      1001
#define  INFOTYPE_VID                 1002
#define  INFOTYPE_PID                 1003
#define  INFOTYPE_RBC_EXTENSION_MAJ   1004
#define  INFOTYPE_RBC_EXTENSION_MIN   1005
#define  INFOTYPE_NATIVE              1006

// Predefined values for STLink_DeviceRequestT::InputRequest
// ----- Legacy defines, (do not use) -----------------------------------------
// (initially: 0 for write, else Read through EP1)
#define REQUEST_WRITE    0 //Through EP2OUT or EP1OUT for DBG/TCP interface, through EP6OUT for BRIDGE interface 
#define REQUEST_READ_EP3 3 //Through EP3IN or EP2IN for DBG/TCP interface only: Compatibility break if that value was used for reading through EP1
// All other values are for read through EP1IN, but a constant is provided anyway for upper layers lisibility
#define REQUEST_READ_EP1 1  //Through EP1IN for DBG interface, through EP6IN for BRIDGE interface 
// ----- Define for STLink_DeviceRequestT::InputRequest -----------------------
#define REQUEST_READ_1ST_EPIN        REQUEST_READ_EP1 // read req (cmd+data) through 1st bulk IN EP of the given interface
#define REQUEST_WRITE_1ST_EPOUT      REQUEST_WRITE    // write req (cmd+data) through 1st bulk OUT EP of the given interface
#define REQUEST_READ_DATA_2ND_EPIN   REQUEST_READ_EP3 // read req (data only) through 2nd bulk IN EP of the given interface (if it exists)

/// Serial number max size
#define SERIAL_NUM_STR_MAX_LEN 32

// Constant defining close type in STLink_CloseDeviceTcp() closeTcp parameter
#define CLOSE_TCP_AUTO   0

// Size in bytes of a STLink_DeviceRequestT CDBByte field
#define STLINK_CMD_SIZE_16   16

// As this structure is part of the DLL API, and is transmitted through
// a pointer, the field alignment in the structure must fit with the
// calling one: and it is a 1-byte alignment
#pragma pack(push,1)

///////////////////////////////////////////////////////////////////////////////
// Type definitions for old API (ascendant compatibility ...)
///////////////////////////////////////////////////////////////////////////////
typedef void * PDevice;
typedef PDevice * PPDevice;

typedef struct {
  uint8_t CDBLength;         // Command header length (CDB on mass storage) - used only on ST-Link/V1
  uint8_t CDBByte[STLINK_CMD_SIZE_16]; // Command header (CDB on mass storage)
  uint8_t InputRequest;      // 0 (output) or input on ST-Link/V1, REQUEST_WRITE, REQUEST_READ_EP1 or REQUEST_READ_EP3 on ST-Link/V2
  void* Buffer;           // Data for data stage (if any)
  uint32_t BufferLength;     // Size of data stage (0 if no data stage)
  uint8_t SenseLength;       // To be initialized: DEFAULT_SENSE_LEN - used only on ST-Link/V1
  uint8_t Sense[DEFAULT_SENSE_LEN+2]; // Used only on ST-Link/V1
} TDeviceRequest, *PDeviceRequest; // TDeviceRequest, *PDeviceRequest: legacy library name, use STLink_DeviceRequestT

typedef TDeviceRequest STLink_DeviceRequestT;

// Note keep TDeviceRequest before STLink_DeviceRequestT because old library only know TDeviceRequest
///////////////////////////////////////////////////////////////////////////////
// Type definitions for new API
///////////////////////////////////////////////////////////////////////////////
//legacy deviceInfo structure before stlink-TCP, use STLink_DeviceInfo2T instead
typedef struct {
	char DevPath[MAX_PATH]; // Device pathname returned by SetupDiGetDeviceInterfaceDetail on ST-Link/V2
	                        // Or disk name like "\\.\A:" on ST-Link/V1
	char EnumUniqueId[SERIAL_NUM_STR_MAX_LEN]; // Unique instance ID from system enumeration (equal to
	                                           // device Serial number in some cases, but not always ...)
	uint16_t VendorId;  // Vendor  ID from USB device descriptor (system enumeration)
	uint16_t ProductId; // Product ID from USB device descriptor (system enumeration)
	uint8_t DeviceUsed; // On windows, equal to 1 if device interface was already opened from externally when enumerating or trying to open
	// NOTE: do not modify the existing fields in the structure. But for any evolution,
	// add a new field at the end in order to keep ascendant compatibility.
} TDeviceInfo; // TDeviceInfo: legacy library name, use STLink_DeviceInfoT

typedef TDeviceInfo STLink_DeviceInfoT;

/** @addtogroup INTERFACE
 * @{
 */

/// #TDeviceInfo2: legacy library name, use #STLink_DeviceInfo2T
typedef struct {
	uint32_t StLinkUsbId; ///< ST-LINK-SERVER Device cookie in little endian format, to use in STLINK_TCP_CMD_OPEN_DEVICE
	char EnumUniqueId[SERIAL_NUM_STR_MAX_LEN]; ///< Unique instance ID from system enumeration (equal to
	                                           ///< device Serial number in some cases, but not always ...)
	uint16_t VendorId;  ///< Vendor  ID from USB device descriptor (system enumeration)
	uint16_t ProductId; ///< Product ID from USB device descriptor (system enumeration)
	uint8_t DeviceUsed; ///< On windows, equal to 1 if device interface was already opened from 
	                    ///< externally when enumerating or trying to open
	// NOTE: do not modify the existing fields in the structure. But for any evolution,
	// add a new field at the end in order to keep ascendant compatibility.
} TDeviceInfo2;

typedef TDeviceInfo2 STLink_DeviceInfo2T; ///< Structure containing information on connected device connected
                                          ///<  (serial number, device in use, USB ProductID, ...).

/// #TEnumStlinkInterface: legacy library name, use #STLink_EnumStlinkInterfaceT
typedef enum {
	STLINK_DBG_INTERFACE =0, // Note that the value is also the interface index in the device (important for implementation on Linux)
	STLINK_RW_INTERFACE,  // Note that the value is also the interface index in the device (important for implementation on Linux)
	STLINK_TCP, // Interface for multi-core debug through TCP client/server model
	STLINK_BRIDGE, ///< Interface for Bridge
	               //on Linux conversion is done by STLinkLibUSB_ConvToUsbIfId()
    STLINK_NB_INTERFACES // Keep this last
} TEnumStlinkInterface;

typedef TEnumStlinkInterface STLink_EnumStlinkInterfaceT;///< Enum to choose the STLink interface to be used

// end group doxygen INTERFACE
/** @} */


// Do not affect the alignment of others
#pragma pack(pop)

#define STLINK_USB_BRIDGE_IF_NB 4 // Bridge interface index in the device (important for implementation on Linux) when mass storage interface is present
#define STLINK_USB_BRIDGE_IF_NB_NO_MSD 3 // Bridge interface index in the device (important for implementation on Linux) when no mass storage interface

/* Exported functions ------------------------------------------------------- */

// CALL FUNCTIONS WITH GETPROCADDRESS

///////////////////////////////////////////////////////////////////////////////
// Routines for old API (ascendant compatibility ...)
///////////////////////////////////////////////////////////////////////////////

//******************************************************************************
// STMass_Enum_Reenumerate: Rebuild a list of mass storage device manageable
//   by this dll. To belong to the list, the dll must have its licence key.
// Returned: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
typedef uint32_t (STD_CALL *pSTMass_Enum_Reenumerate)();

//******************************************************************************
// STMass_Enum_GetNbDevices: returns the number of devices that were seen with
//   a previous call to STMass_Enum_Reenumerate.
// Returned: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
typedef uint32_t (STD_CALL *pSTMass_Enum_GetNbDevices)();

//******************************************************************************
// STMass_Enum_GetDevice: used to obtain a device representation.
// Parameters:
//   Index: nb for device to retrieved. 0-based
//   pDevice: retrieved device
// Returned: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
typedef uint32_t (STD_CALL *pSTMass_Enum_GetDevice)(uint32_t Index, PPDevice pDevice);

//******************************************************************************
// STMass_GetDeviceInfo: Retrieves some information about a device
// Parameters:
//   pDevice: device for which to retrieve information
//   Kind: information to retrieve (see Kind codes)
//   pBuffer: Buffer in which to pur information
//   BufferLength: Maximum size for buffer
// Returned: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
typedef uint32_t (STD_CALL *pSTMass_GetDeviceInfo)(PDevice pDevice, uint32_t Kind, 
						      char * pBuffer, uint32_t BufferLength);

//******************************************************************************
// STMass_OpenDevice: opens a device
// Parameters:
//   pDevice: device
//   pHandle: handle returned if successful
// Returned: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
typedef uint32_t (STD_CALL *pSTMass_OpenDevice)(PDevice pDevice, void ** pHandle);

//******************************************************************************
// STMass_OpenDevice: opens a device in non-shareable mode (recommended)
// Parameters:
//   pDevice: device
//   pHandle: handle returned if successful
// Returned: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
typedef uint32_t (STD_CALL *pSTMass_OpenDeviceExclusive)(PDevice pDevice, void ** pHandle);

//******************************************************************************
// STMass_CloseDevice: opens a device
// Parameters:
//   pDevice: device
//   pHandle: handle to be closed
// Returned: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
typedef uint32_t (STD_CALL *pSTMass_CloseDevice)(PDevice pDevice, void * pHandle);

//******************************************************************************
// STMass_SendCommand: send an RBC command to the device
// Parameters:
//   pHandle: handle for device
//   pRequest: request to be issued
//   DwTimeOut: TimeOut for request completion
// Returned: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
typedef uint32_t (STD_CALL *pSTMass_SendCommand)(PDevice pDevice, void * pHandle, 
                                       PDeviceRequest pRequest, uint32_t DwTimeOut);

///////////////////////////////////////////////////////////////////////////////
// Routines for new API
///////////////////////////////////////////////////////////////////////////////

//******************************************************************************
// STLink_Reenumerate:
// Build the list of presently connected devices providing the given interface.
// The list is kept locally until next call. The list is internally limited to
// MAX_DEVICES (60), exceeding devices (if any) are ignored.
//
// If bClearList == 0:
//   - Opened devices remain in the list if still present on the system (but
//     potentially with a different index in the list => use of handles instead
//     of indexes is required because of that)
//   - Opened devices that are no more present on the system are closed and
//     handles are deleted.
//
// If bClearList != 0:
//   - All opened devices are closed and handles are deleted.
//   - The use of a previously returned handle is UNPREDICTABLE (might either fail
//     because the handle is no more known, or might address a newly renumerated
//     device that has been granted the same handle by chance ... 
//   - Useful for closing all devices is case returned handles have been lost by the
//     caller. In standard case, bClearList == 0 has to be preferred.
//
// Returns: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
#ifdef WIN32
// Export the signature only; the routine will be linked through GetProcAdddress
typedef uint32_t (STD_CALL *pSTLink_Reenumerate)(TEnumStlinkInterface IfId, uint8_t bClearList);
#else
EXPORTED_API uint32_t STD_CALL STLink_Reenumerate(TEnumStlinkInterface IfId, uint8_t bClearList);
#endif

//******************************************************************************
// STLink_GetNbDevices:
// Returns the number of connected devices providing the given interface, that
// have been enumerated with a previous call to STLink_Reenumerate.
//******************************************************************************
#ifdef WIN32
// Export the signature only; the routine will be linked through GetProcAdddress
typedef uint32_t (STD_CALL *pSTLink_GetNbDevices)(TEnumStlinkInterface IfId);
#else
EXPORTED_API uint32_t STD_CALL STLink_GetNbDevices(TEnumStlinkInterface IfId);
#endif

//******************************************************************************
// STLink_GetDeviceInfo:
// Retrieves some information about a device
// Parameters:
//   IfId, DevIdxInList: selects the device in the list of devices providing the
//            considered interface
//   pInfo: Pointer to a TDeviceInfo structure allocated by the caller
//   InfoSize: Size of the TDeviceInfo structure allocated by the caller.
//             Must be provided in order to properly manage future evolutions: new
//             fields might be added at the end of the structure, without
//             compatibility break.
//
// Returns:
//   SS_OK  = SUCCESS
//   SS_TRUNCATED_DATA in case infoSize is greater than the size of the TDeviceInfo
//          managed by the DLL. In that case only the first fields of TDeviceInfo
//          are significant; the other ones are set to 0.
//   SS_BAD_PARAMETER in case of unexpected values for IfId, DevIdxInList or pInfo
//******************************************************************************
#ifdef WIN32
// Export the signature only; the routine will be linked through GetProcAdddress
typedef uint32_t (STD_CALL *pSTLink_GetDeviceInfo)(TEnumStlinkInterface IfId,
                          uint8_t DevIdxInList, TDeviceInfo *pInfo, uint32_t InfoSize);
#else
EXPORTED_API uint32_t STD_CALL STLink_GetDeviceInfo(TEnumStlinkInterface IfId,
                          uint8_t DevIdxInList, TDeviceInfo *pInfo, uint32_t InfoSize);
#endif

//******************************************************************************
// STLink_GetDeviceInfo2:
// Retrieves some information about a device
// replace STLink_GetDeviceInfo
// Parameters:
//   IfId, DevIdxInList: selects the device in the list of devices providing the
//            considered interface
//   pInfo: pointer to a TDeviceInfo2 structure allocated by the caller
//   InfoSize: size of the TDeviceInfo2 structure allocated by the caller.
//             Must be provided in order to properly manage future evolutions:
//             new fields might be added at the end of the structure,
//             without compatibility break.
//
// Returns:
//   SS_OK  = SUCCESS
//   SS_TRUNCATED_DATA in case infoSize is greater than the size of the TDeviceInfo2
//          managed by the DLL. In that case only the first fields of TDeviceInfo2
//          are significant; the other ones are set to 0.
//          Or caller used TDeviceInfo struct and size instead of TDeviceInfo2.
//   SS_BAD_PARAMETER in case of unexpected values for IfId, DevIdxInList or pInfo
//******************************************************************************
#ifdef WIN32
// Export the signature only; the routine will be linked through GetProcAdddress
typedef uint32_t (STD_CALL *pSTLink_GetDeviceInfo2)(TEnumStlinkInterface IfId,
                          uint8_t DevIdxInList, TDeviceInfo2 *pInfo, uint32_t InfoSize);
#else
EXPORTED_API uint32_t STD_CALL STLink_GetDeviceInfo2(TEnumStlinkInterface IfId,
                          uint8_t DevIdxInList, TDeviceInfo2 *pInfo, uint32_t InfoSize);
#endif

//******************************************************************************
// STLink_OpenDevice:
// Opens a previously enumerated device
// Parameters:
//   IfId, DevIdxInList: selects the device in the list of devices providing the
//            considered interface
//   bExclusiveAccess: if 0, the device is opened in shared mode (different
//            applications might share access to the same device);
//            otherwise the device is opened in exclusive mode (fails if already
//            opened by someone else, and if opened successfully, will make fail
//            any attempt to open from others)
//   pHandle: handle returned if successful. Required for accessing the device.
//
// Returns: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
#ifdef WIN32
// Export the signature only; the routine will be linked through GetProcAdddress
typedef uint32_t (STD_CALL *pSTLink_OpenDevice)(TEnumStlinkInterface IfId,
                          uint8_t DevIdxInList, uint8_t bExclusiveAccess, void ** pHandle);
#else
EXPORTED_API uint32_t STD_CALL STLink_OpenDevice(TEnumStlinkInterface IfId,
                          uint8_t DevIdxInList, uint8_t bExclusiveAccess, void ** pHandle);
#endif

//******************************************************************************
// STLink_CloseDevice:
// Closes a previously opened device
// Parameters:
//   pHandle: handle (as returned by STLink_OpenDevice) to be closed
//
// Returns: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
#ifdef WIN32
// Export the signature only; the routine will be linked through GetProcAdddress
typedef uint32_t (STD_CALL *pSTLink_CloseDevice)(void * pHandle);
#else
EXPORTED_API uint32_t STD_CALL STLink_CloseDevice(void * pHandle);
#endif

//******************************************************************************
// STLink_SendCommand:
// Sends a PDeviceRequest command to a previously opened device
// Parameters:
//   pHandle: handle for device
//   pRequest: request to be issued
//   DwTimeOut: TimeOut for request completion
//
// Returns: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
#ifdef WIN32
// Export the signature only; the routine will be linked through GetProcAdddress
typedef uint32_t (STD_CALL *pSTLink_SendCommand)(void * pHandle, 
                                       PDeviceRequest pRequest, uint32_t DwTimeOut);
#else
EXPORTED_API uint32_t STD_CALL STLink_SendCommand(void * pHandle,
                                       PDeviceRequest pRequest, uint32_t DwTimeOut);
#endif

//******************************************************************************
// STLink_ReenumerateTcp:
// Build the list of presently STLink-TCP connected devices.
// The list is kept locally until next call. The list is internally limited to
// MAX_DEVICES (60), exceeding devices (if any) are ignored.
// pConnectParams and pServerCmdLineParams must be ASCII.
//
// If bClearList == 0: currently not used by tcp server.
// connectParams, char *serverCmdLineParams
//
// Returns: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
#ifdef WIN32
// Export the signature only; the routine will be linked through GetProcAdddress
typedef uint32_t (STD_CALL *pSTLink_ReenumerateTcp)(TEnumStlinkInterface IfId, uint8_t bClearList,
											char *pConnectParams, char *pServerCmdLineParams);
#else
EXPORTED_API uint32_t STD_CALL STLink_ReenumerateTcp(TEnumStlinkInterface IfId, uint8_t bClearList,
											char *pConnectParams, char *pServerCmdLineParams);
#endif

//******************************************************************************
// STLink_OpenDeviceTcp:
// Opens a previously enumerated device through TCP, STLink_GetDeviceInfo2()
// must have been called before.
// Parameters:
//   IfId: considered interface, must be STLINK_TCP
//   DevInfoUsbId: Tcp server Device cookie as returned by STLink_GetDeviceInfo2
//                in TDeviceInfo2 (field StLinkUsbId)
//   bExclusiveAccess: if 0, the device is opened in shared mode (different
//            applications might share access to the same device);
//            otherwise the device is opened in exclusive mode (fails if already
//            opened by someone else, and if opened successfully, will make fail
//            any attempt to open from others)
//
// Returns: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
#ifdef WIN32
// Export the signature only; the routine will be linked through GetProcAdddress
typedef uint32_t (STD_CALL *pSTLink_OpenDeviceTcp)(TEnumStlinkInterface IfId,
                          uint32_t DevInfoUsbId, uint8_t bExclusiveAccess);
#else
EXPORTED_API uint32_t STD_CALL STLink_OpenDeviceTcp(TEnumStlinkInterface IfId,
                          uint32_t DevInfoUsbId, uint8_t bExclusiveAccess);
#endif

//******************************************************************************
// STLink_CloseDeviceTcp:
// Closes a previously opened device through TCP
// Parameters:
//   StLinkUsbId: Tcp server Device to be closed (cookie as returned by
//                STLink_GetDeviceInfo2 in TDeviceInfo2)
//   closeTcp: only CLOSE_TCP_AUTO = 0, other values reserved for future use
//             Tcp socket is closed if StLinkUsbId was the last opened device.
//
// Returns: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
#ifdef WIN32
// Export the signature only; the routine will be linked through GetProcAdddress
typedef uint32_t (STD_CALL *pSTLink_CloseDeviceTcp)(uint32_t StLinkUsbId, uint8_t closeTcp);
#else
EXPORTED_API uint32_t STD_CALL STLink_CloseDeviceTcp(uint32_t StLinkUsbId, uint8_t closeTcp);
#endif

//******************************************************************************
// STLink_SendCommandTcp:
// Sends a PDeviceRequest command to a previously opened device through TCP
// Parameters:
//   StLinkUsbId: Tcp server Device cookie as returned by STLink_GetDeviceInfo2
//                in TDeviceInfo2
//   pRequest: request to be issued
//   DwTimeOut: TimeOut for request completion
//
// Returns: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
#ifdef WIN32
// Export the signature only; the routine will be linked through GetProcAdddress
typedef uint32_t (STD_CALL *pSTLink_SendCommandTcp)(uint32_t StLinkUsbId,
                                       PDeviceRequest pRequest, uint32_t DwTimeOut);
#else
EXPORTED_API uint32_t STD_CALL STLink_SendCommandTcp(uint32_t StLinkUsbId,
                                       PDeviceRequest pRequest, uint32_t DwTimeOut);
#endif

//******************************************************************************
// STLink_GetNumOfDeviceClientsTcp:
// Returns the number of connected TCP clients providing the given device, that
// have been enumerated with a previous call to STLink_Reenumerate.
// returns 0 if given device is currently not opened by any TCP client or if
// device is not found.
//******************************************************************************
#ifdef WIN32
// Export the signature only; the routine will be linked through GetProcAdddress
typedef uint32_t (STD_CALL *pSTLink_GetNumOfDeviceClientsTcp)(uint32_t StLinkUsbId);
#else
EXPORTED_API uint32_t STD_CALL STLink_GetNumOfDeviceClientsTcp(uint32_t StLinkUsbId);
#endif

//******************************************************************************
// STLink_FreeLibrary:
// Available on Linux and MacOS only. To call before the application terminates
// in order to free all objects. On Windows similar things are done through
// DllMain(DLL_PROCESS_DETACH)
// Parameters: None
//
// Returns: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
#ifndef WIN32
EXPORTED_API uint32_t STD_CALL STLink_FreeLibrary(void);
#endif

#endif // _STLINKUSBDRIVER_H
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/