/**
  ******************************************************************************
  * @file    stlink_interface.cpp
  * @author  MCD Application Team
  * @brief   Module to access the STLink device library (STLinkUSBDriver.h).
  *          Manage communication interfaces (USB) to connect to the STLink device. \n
  *          STLinkInterface class manages USB enumeration and STLink devices detection.
  *          STLinkInterface object to be initialized before being used by Brg.
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
#include "stlink_interface.h"

#include <string>

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

// used for debug trace: interface name according to STLink_EnumStlinkInterfaceT value
const char * LogIfString[STLINK_NB_INTERFACES] = 
{ "DBG", "DBG2", "DBG SERVER", "BRIDGE" };

/* Class Functions Definition ------------------------------------------------*/

/**
 * @ingroup INTERFACE
 * @brief STLinkInterface constructor
 * @param[in]  IfId  STLink USB interface to be used: #STLINK_BRIDGE for Bridge interface.
 *                   Other interfaces not supported currently.
 */
STLinkInterface::STLinkInterface(STLink_EnumStlinkInterfaceT IfId): m_ifId(IfId), m_nbEnumDevices(0), m_bApiDllLoaded(false),
	m_bDevInterfaceEnumerated(false)
{
	m_pathOfProcess[0]='\0';

#ifdef USING_ERRORLOG
	// Error log management
	m_pErrLog = NULL;
#endif
}
/**
 * @ingroup INTERFACE
 * @brief STLinkInterface destructor
 */
STLinkInterface::~STLinkInterface(void)
{
	// critical section deletion not needed because static mutex

	//STLink_FreeLibrary();
	if (m_bApiDllLoaded) {
		libusb_exit(ctx);
	}
}
/*
 * Trace logging mechanism, under compilation switch USING_ERRORLOG (requiring ErrLog.h and ErrLog.cpp)
 */
void STLinkInterface::LogTrace(const char *pMessage, ...)
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

uint32_t STLinkInterface::STLink_GetNbDevices(TEnumStlinkInterface IfId)
{
	uint32_t deviceCount = 0;
	libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	cnt = libusb_get_device_list(ctx, &devs); //get the list of devices
	if (cnt < 0) {
		return 0;
	}

	ssize_t i; //for iterating through the list
	for(i = 0; i < cnt; i++) {
		libusb_device_descriptor desc = {0};
		int rc = libusb_get_device_descriptor(devs[i], &desc);
		if (rc == 0) {
			if (desc.idVendor == STLINK_V3_VID && 
				std::find(std::begin(STLINK_V3_PID), std::end(STLINK_V3_PID), desc.idProduct) != std::end(STLINK_V3_PID)) {
				deviceCount++;
			}
		}
	}
	libusb_free_device_list(devs, 1); //free the list, unref the devices in it
	return deviceCount;
}

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
uint32_t STLinkInterface::STLink_GetDeviceInfo2(TEnumStlinkInterface IfId, uint8_t DevIdxInList, TDeviceInfo2 *pInfo, uint32_t InfoSize)
{
	if (DevIdxInList >= cnt)
		return SS_BAD_PARAMETER;

	libusb_device *dev = devices[DevIdxInList];
	libusb_device_descriptor desc = {0};
	char string[256];
	int rc = libusb_get_device_descriptor(dev, &desc);
	if (rc == 0) {
		pInfo->VendorId = desc.idVendor;
		pInfo->ProductId = desc.idProduct;
		pInfo->DeviceUsed = 0;
		libusb_device_handle *handle = nullptr;
		int ret = libusb_open(dev, &handle);
		if (LIBUSB_SUCCESS == ret) {
			if (desc.iSerialNumber) {
				ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, (unsigned char*)string, sizeof(string));
				if (ret > 0) {
					#if __GNUC__
					#pragma GCC diagnostic push
					#pragma GCC diagnostic ignored "-Wformat-truncation"
					#endif
					// We are intentionally truncating since for STLink devices,
					// the string descriptor is known to be no more than 31 bytes,
					// despite the fact that a USB string descriptor may up to 255 bytes long.
					ret = snprintf(pInfo->EnumUniqueId, sizeof(pInfo->EnumUniqueId), "%s", string);
					#if __GNUC__
					#pragma GCC diagnostic pop
					#endif
				}
			}
			libusb_close(handle);
		}
	}
	return SS_OK;
}

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
uint32_t STLinkInterface::STLink_OpenDevice(TEnumStlinkInterface IfId, uint8_t DevIdxInList, uint8_t bExclusiveAccess, void **pHandle)
{
	if (DevIdxInList >= cnt)
		return SS_BAD_PARAMETER;

	libusb_device *dev = devices[DevIdxInList];
	libusb_device_handle *handle = nullptr;
	int ret = libusb_open(dev, &handle);
	libusb_claim_interface(handle, 3);
	if (LIBUSB_SUCCESS == ret) {
		*pHandle = handle;
		return  SS_OK;
	}
	return SS_ERR;
}

//******************************************************************************
// STLink_CloseDevice:
// Closes a previously opened device
// Parameters:
//   pHandle: handle (as returned by STLink_OpenDevice) to be closed
//
// Returns: SS_OK  = SUCCESS, Error otherwise (Error Code)
//******************************************************************************
uint32_t STLinkInterface::STLink_CloseDevice(void *pHandle)
{
	libusb_release_interface((libusb_device_handle*)pHandle, 3);
	libusb_close((libusb_device_handle*)pHandle);
	return SS_OK;
}
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
uint32_t STLinkInterface::STLink_SendCommand(void *pHandle, PDeviceRequest pRequest, uint32_t DwTimeOut)
{
	libusb_device_handle *handle = (libusb_device_handle *)pHandle;
	int actualLength = 0;

	// transmit command
	int rc = libusb_bulk_transfer(handle, 0x06, (unsigned char*)pRequest->CDBByte, (int)pRequest->CDBLength, &actualLength, DwTimeOut);
	if (rc != LIBUSB_TRANSFER_COMPLETED || actualLength != (int)pRequest->CDBLength)
		return SS_ERR;
	if (pRequest->BufferLength == 0) // 0 length transfer should be supported, but breaks comms
		return SS_OK;
	// read or write depending on request type
	unsigned char ep = pRequest->InputRequest == REQUEST_READ_1ST_EPIN ? 0x86 : 0x06; // else REQUEST_WRITE_1ST_EPOUT
	rc = libusb_bulk_transfer(handle, ep, (unsigned char*)pRequest->Buffer, (int)pRequest->BufferLength, &actualLength, DwTimeOut);
	if (rc != LIBUSB_TRANSFER_COMPLETED || actualLength != (int)pRequest->BufferLength)
		return SS_ERR;

	return SS_OK;
}

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
uint32_t STLinkInterface::STLink_Reenumerate(TEnumStlinkInterface IfId, uint8_t bClearList)
{
	uint32_t deviceCount = 0;
	libusb_device **devs;
	ssize_t cnt; //holding number of devices in list
	cnt = libusb_get_device_list(ctx, &devs); //get the list of devices
	if (cnt < 0) {
		return 0;
	}

	ssize_t i; //for iterating through the list
	for(i = 0; i < cnt; i++) {
		libusb_device_descriptor desc;
		int rc = libusb_get_device_descriptor(devs[i], &desc);
		if (rc == 0) {
			if (desc.idVendor == STLINK_V3_VID && 
				std::find(std::begin(STLINK_V3_PID), std::end(STLINK_V3_PID), desc.idProduct) != std::end(STLINK_V3_PID)) {
				devices[deviceCount] = devs[i];
				deviceCount++;
			}
		}
	}
	libusb_free_device_list(devs, 1);
	return SS_OK;
}

/**
 * @ingroup INTERFACE
 * @brief If not already done: load the STLinkUSBDriver library (windows only), open log files.
 *
 * @param[in]  pPathOfProcess  Path, in ASCII, where STLinkUSBDriver library is searched
 *                             when not found in current dir, for Windows.
 *
 * @retval #STLINKIF_NOT_SUPPORTED m_ifId not supported yet
 * @retval #STLINKIF_DLL_ERR  STLinkUSBDriver library not loaded
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT  STLinkInterface::LoadStlinkLibrary(const char *pPathOfProcess)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;

	// Only BRIDGE interface supported currently
	if( m_ifId != STLINK_BRIDGE ) {
		return STLINKIF_NOT_SUPPORTED;
	}

	if( m_bApiDllLoaded == false ) {
		int res = libusb_init(&ctx);
		if (res == 0) {
			libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
			m_bApiDllLoaded = true;
		}
	}
	return ifStatus;
}
/*
 * @brief Return true if STLinkInterface::LoadStlinkLibrary() has been called successfully.
 */
bool STLinkInterface::IsLibraryLoaded() {
	return m_bApiDllLoaded;
}
/**
 * @ingroup INTERFACE
 * @brief USB enumeration routine.
 * Enumerate all devices presenting the given USB STLink interface
 * (or refresh device list after change in the system).
 *
 * @param[in]  bClearList  If true:\n
 *               - All opened devices are closed and handles are deleted.\n
 *               - The use of a previously returned handle is UNPREDICTABLE (might either fail
 *               because the handle is no more known, or might address a newly renumerated
 *               device that has been granted the same handle by chance ... \n
 *               - Useful for closing all devices is case returned handles have been lost by the
 *               caller. In standard case, bClearList == 0 has to be preferred.
 * @param[out] pNumDevices Pointer where the function returns the number of connected STLink m_ifId interfaces.
 *                          Can be NULL if not wanted.
 *
 * @retval #STLINKIF_NOT_SUPPORTED If STLinkUSBDriver is too old for given m_ifId or m_ifId not supported yet
 * @retval #STLINKIF_NO_STLINK No STLink with given m_ifId connected.
 * @retval #STLINKIF_PERMISSION_ERR Lack of permission during enumeration
 * @retval #STLINKIF_ENUM_ERR Error during enumeration
 * @retval #STLINKIF_DLL_ERR  STLinkUSBDriver library not loaded
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface:: EnumDevices(uint32_t *pNumDevices, bool bClearList)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NOT_SUPPORTED;
	uint32_t status = SS_OK;

	if( pNumDevices != nullptr ) {
		*pNumDevices = 0; // default if error
	}

	if( IsLibraryLoaded() == true ) {
		if( m_ifId == STLINK_BRIDGE ) {
			status = STLink_Reenumerate(m_ifId, bClearList);
			if( status == SS_BAD_PARAMETER ) {
				// DLL is too old and does not support BRIDGE interface
				m_bApiDllLoaded = false;
				return STLINKIF_DLL_ERR;
			}
			// Note that STLink_Reenumerate might fail because of issue during serial number retrieving
			// which is not a blocking error here;
			m_nbEnumDevices = STLink_GetNbDevices(m_ifId);

			if( m_nbEnumDevices == 0 ) {
				LogTrace("No STLink device with %s interface detected on the USB", LogIfString[m_ifId]);
				return STLINKIF_NO_STLINK;
			}

			if( status == SS_OK ) {
				ifStatus = STLINKIF_NO_ERR;
			} else {
				if( status == SS_PERMISSION_ERR ) {
					LogTrace("STLinkInterface Lack of permission during enumeration");
					ifStatus = STLINKIF_PERMISSION_ERR;
				} else {
					LogTrace("STLinkInterface Error during enumeration");
					ifStatus = STLINKIF_ENUM_ERR;
				}
			}

			if( pNumDevices != NULL ) {
				*pNumDevices=(int)m_nbEnumDevices;
			}
		} else {
			ifStatus = STLINKIF_NOT_SUPPORTED;
		}
	} else { // IsLibraryLoaded()
		ifStatus = STLINKIF_DLL_ERR;
	}
	return ifStatus;
}
/*
 * @brief  Private enumeration routine called by other functions. USB enumeration routine.
 * Enumerate all devices presenting the current USB STLink interface
 * (or refresh device list after change in the system).
 *
 * @param[in]  bForceRenum Set to true in order to force the renumeration (refresh device list after change
 *                         in the system). Otherwise the list remains as it was during the previous call.
 * @param[in]  bClearList  If true:\n
 *               - All opened devices are closed and handles are deleted.\n
 *               - The use of a previously returned handle is UNPREDICTABLE (might either fail
 *               because the handle is no more known, or might address a newly renumerated
 *               device that has been granted the same handle by chance ... \n
 *               - Useful for closing all devices is case returned handles have been lost by the
 *               caller. In standard case, bClearList == 0 has to be preferred.
 * @param[out] pNumDevices Pointer where the function returns the number of connected STLink devices
 *                         with the required interface. Can be NULL if not wanted.
 * @warning #STLINKIF_PERMISSION_ERR can be returned if one device is already opened by another program,
 *          however the device is listed in pNumDevices.
 *
 * @retval #STLINKIF_NOT_SUPPORTED Parameter(s) error
 * @retval #STLINKIF_DLL_ERR Error in loading STLinkUSBDriver library or too old not supporting current Interface
 * @retval #STLINKIF_PERMISSION_ERR Lack of permission during USB enumeration
 * @retval #STLINKIF_ENUM_ERR USB enumeration error
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::EnumDevicesIfRequired(uint32_t *pNumDevices, bool bForceRenum, bool bClearList)
{
	STLinkIf_StatusT ifStatus=STLINKIF_NO_ERR;
	// uint32_t status = SS_OK;

	if( pNumDevices != NULL ) {
		// Will return 0 in case of error
		*pNumDevices=0;
	}
	if( m_ifId != STLINK_BRIDGE ) {
		return STLINKIF_NOT_SUPPORTED;
	}

	if( (m_bDevInterfaceEnumerated == false) || (bForceRenum==true) ) {

		ifStatus = EnumDevices(pNumDevices, bClearList);
		if( m_nbEnumDevices == 0 ) {
			return STLINKIF_NO_STLINK;
		}
		if( ifStatus == STLINKIF_NO_ERR ) {
			// All is OK; no more necessary to do it again
			m_bDevInterfaceEnumerated = true;
		}
	}
	return ifStatus;
}

/**
 * @ingroup INTERFACE
 * @brief Return the #STLink_DeviceInfo2T data of the specified STLink device.
 *
 * @param[in]  StlinkInstId   STLink device index, from 0 to *pNumDevices-1
 *                            (value returned by STLinkInterface::EnumDevices())
 * @param[in]  InfoSize   Size of the allocated #STLink_DeviceInfo2T instance.
 *               Required for ascendant compatibility in case this structure grows in the future.
 *               If the allocated size is smaller than the size managed by STLinkUSBDriver library,
 *               returned data are limited to the given size. In case the allocated size
 *               is greater (caller more recent than STLinkUSBDriver library), exceeding fields are set to 0.
 * @param[out] pInfo  Pointer to the caller-allocated #STLink_DeviceInfo2T instance.
 *
 * @return STLinkInterface::EnumDevices() errors
 * @retval #STLINKIF_NOT_SUPPORTED If STLinkUSBDriver is too old for given m_ifId or m_ifId not supported yet
 * @retval #STLINKIF_GET_INFO_ERR Error in STLinkUSBDriver to retrieve the information
 * @retval #STLINKIF_PARAM_ERR Wrong StlinkInstId or NULL pInfo pointer
 * @retval #STLINKIF_DLL_ERR  STLinkUSBDriver library not loaded
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::GetDeviceInfo2(int StlinkInstId, STLink_DeviceInfo2T *pInfo, uint32_t InfoSize)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;

	if( IsLibraryLoaded() == true ) {

		// Enumerate the current STLink interface if not already done
		ifStatus = EnumDevicesIfRequired(NULL, false, false);
		if( ifStatus != STLINKIF_NO_ERR ) {
			return ifStatus;
		}

		if( m_ifId == STLINK_BRIDGE ) {
			if( (StlinkInstId<0) || (((unsigned int)StlinkInstId) >= m_nbEnumDevices) ) {
				LogTrace("%s Bad STLink instance id (%d > %d)", LogIfString[m_ifId], StlinkInstId, m_nbEnumDevices-1);
				return STLINKIF_PARAM_ERR;
			}
			if( pInfo == NULL ) {
				LogTrace("%s Bad parameter in GetDeviceInfo2 (NULL pointer)", LogIfString[m_ifId]);
				return STLINKIF_PARAM_ERR;
			}

			if( STLink_GetDeviceInfo2(m_ifId, StlinkInstId, pInfo, InfoSize) != SS_OK ) {
				return STLINKIF_GET_INFO_ERR;
			}
		} else {
			ifStatus = STLINKIF_NOT_SUPPORTED;
		}
	} else {
		ifStatus = STLINKIF_DLL_ERR;
	}
	return ifStatus;
}
/*
 * @brief Called by StlinkDevice object, do not use directly.
 * Open USB connection with the STLink for the given USB interface.
 *
 * @param[in]  StlinkInstId   Instance ID in the list of enumerated STLink devices
 * @param[in]  bOpenExclusive false: shared between applications \n
 *                            true: exclusive to 1 application
 * @param[in]  stlinkIdTcp    0 (unused)
 * @param[out] pHandle        Handle of the opened STLink device
 *
 * @return STLinkInterface::EnumDevices() errors
 * @retval #STLINKIF_CONNECT_ERR Wrong StlinkInstId or USB error
 * @retval #STLINKIF_NOT_SUPPORTED m_ifId not supported yet
 * @retval #STLINKIF_DLL_ERR STLinkUSBDriver library not loaded
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::OpenDevice(int StlinkInstId, uint32_t StlinkIdTcp, bool bOpenExclusive, void **pHandle)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;
	uint32_t status;

	if( IsLibraryLoaded() == true ) {
		if( m_ifId == STLINK_BRIDGE ) {
			// Enumerate the STLink interface if not already done
			ifStatus = EnumDevicesIfRequired(NULL, false, false);
			if( ifStatus != STLINKIF_NO_ERR ) {
				return ifStatus;
			}

			if( (StlinkInstId<0) || (((unsigned int)StlinkInstId) >= m_nbEnumDevices) ) {
				LogTrace("%s Bad STLink instance id (%d > %d)", LogIfString[m_ifId], StlinkInstId, m_nbEnumDevices-1);
				return STLINKIF_PARAM_ERR;
			}
			// Open the device
			status = STLink_OpenDevice(m_ifId, StlinkInstId, (bOpenExclusive==true)?1:0, pHandle);
			if( status != SS_OK ) {
				LogTrace("%s STLink device USB connection failure", LogIfString[m_ifId]);
				ifStatus = STLINKIF_CONNECT_ERR;
			}
		} else {
			ifStatus = STLINKIF_NOT_SUPPORTED;
		}
	} else {
		ifStatus = STLINKIF_DLL_ERR;
	}
	return ifStatus;
}
/*
 * @brief Called by StlinkDevice object, do not use directly.
 * Opening routine based on the serial number of the STLink; to use in case StlinkInstId is no more
 * reliable (devices have been reenumerated between STLink instance selection and opening) or
 * to choose a specific STLink by its serial number.
 *
 * @param[in]  pSerialNumber  STLink serial number (ASCII).
 * @param[in]  bStrict 	 Used if STLink with required serial number is not found, but one other STLink is found:
 *                       choose if we open or not the other STLink. \n
 *                       If bStrict is true, the command will return #STLINKIF_STLINK_SN_NOT_FOUND if the
 *                       given SerialNumber is not found.\n
 *                       If bStrict is false and the given serial number is not found and one (and only one)
 *                       STLink is found, the command will attempt to open the STLink even if
 *                       the serial number does not match.
 * @return STLinkInterface::EnumDevices() errors
 * @retval #STLINKIF_CONNECT_ERR USB error
 * @retval #STLINKIF_STLINK_SN_NOT_FOUND Serial number not found
 * @retval #STLINKIF_PARAM_ERR if NULL pointer
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::OpenDevice(const char *pSerialNumber, bool bStrict, uint32_t StlinkIdTcp, bool bOpenExclusive, void **pHandle) {
	STLinkIf_StatusT ifStatus=STLINKIF_NO_ERR;
	int stlinkInstId;
	STLink_DeviceInfo2T devInfo2;
	char * pEnumUniqueId = devInfo2.EnumUniqueId;

	if( pSerialNumber == NULL ) {
		LogTrace("NULL pointer for pSerialNumber in OpenStlink");
		return STLINKIF_PARAM_ERR;
	}

	// Enumerate the current STLink interface if not already done
	ifStatus = EnumDevicesIfRequired(NULL, false, false);
	if( ifStatus != STLINKIF_NO_ERR ) {
		return ifStatus;
	}

	// Look for the given serialNumber
	for( stlinkInstId=0; (uint32_t)stlinkInstId<m_nbEnumDevices; stlinkInstId++ ) {
		ifStatus = GetDeviceInfo2(stlinkInstId, &devInfo2, sizeof(devInfo2));
		if( ifStatus != STLINKIF_NO_ERR ) {
			return ifStatus;
		}
		if( strcmp(pSerialNumber, pEnumUniqueId) == 0 ) {
			// Right STLink found: open it
			return OpenDevice(stlinkInstId, 0, bOpenExclusive, pHandle);
		}
	}
	// If there, the asked serial number was not found
	if( (bStrict == false) && (m_nbEnumDevices==1) ) {
		// There is currently only one device connected, and the caller did not expected a full matching
		LogTrace("STLink serial number (%s) not found; opening the (lonely) connected STLink (SN=%s)",
				 pSerialNumber, pEnumUniqueId);
		return OpenDevice(0, 0, bOpenExclusive, pHandle);
	}
	LogTrace("STLink serial number (%s) not found; can not open.", pSerialNumber);
	return STLINKIF_STLINK_SN_NOT_FOUND;
}
/*
 * @brief Called by StlinkDevice object, do not use directly.
 * Close STLink USB communication, with the device instance that was opened by STLinkInterface::OpenDevice()
 *
 * @param[in]  pHandle        Handle of the opened STLink device (returned by STLinkInterface::OpenDevice())
 * @param[in]  stlinkIdTcp    0 (unused)
 *
 * @retval #STLINKIF_CLOSE_ERR Error at USB side
 * @retval #STLINKIF_NOT_SUPPORTED m_ifId not supported yet
 * @retval #STLINKIF_DLL_ERR
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::CloseDevice(void *pHandle, uint32_t StlinkIdTcp)
{
	uint32_t status=SS_OK;
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;

	if( IsLibraryLoaded() == true ) {
		if( m_ifId == STLINK_BRIDGE ) {
			if( (pHandle != NULL) ) {
				status = STLink_CloseDevice(pHandle);
				if( status != SS_OK ) {
					LogTrace("%s Error closing USB communication", LogIfString[m_ifId]);
					ifStatus = STLINKIF_CLOSE_ERR;
				}
			}
		} else {
			ifStatus = STLINKIF_NOT_SUPPORTED;
		}
	} else {
		ifStatus = STLINKIF_DLL_ERR;
	}
	return ifStatus;
}
/*
 * @brief Called by StlinkDevice object, do not use directly.
 * Send a command over the USB, wait for answer, requires the STLink to be already connected.
 *
 * @param[in]  pHandle        Handle of the opened STLink device (returned by STLinkInterface::OpenDevice())
 * @param[in]  stlinkIdTcp    0 (unused)
 * @param[in,out]  pDevReq    Command to send (contains a pointer to answer buffer if any)
 * @param[in]  UsbTimeoutMs   if 0 use default (5s) else use UsbTimeoutMs.
 *
 * @retval #STLINKIF_PARAM_ERR Null pointer error
 * @retval #STLINKIF_USB_COMM_ERR USB communication error
 * @retval #STLINKIF_NOT_SUPPORTED m_ifId not supported yet
 * @retval #STLINKIF_DLL_ERR
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::SendCommand(void *pHandle,
											  uint32_t StlinkIdTcp, STLink_DeviceRequestT *pDevReq,
											  const uint16_t UsbTimeoutMs)
{
	uint32_t ret;
	uint32_t usbTimeout = DEFAULT_TIMEOUT;
	STLinkIf_StatusT ifStatus = STLINKIF_PARAM_ERR;

	if( pDevReq == NULL ) {
		return STLINKIF_PARAM_ERR;
	}


	if( IsLibraryLoaded() == true ) {
		if( m_ifId == STLINK_BRIDGE ) {
			// UsbTimeoutMs if 0 use default (5s) else use UsbTimeoutMs.
			if( UsbTimeoutMs != 0 ) {
				usbTimeout = (uint32_t) UsbTimeoutMs;
			}
			ret = STLink_SendCommand(pHandle, pDevReq, usbTimeout);

			if( ret != SS_OK ) {
				LogTrace("%s USB communication error (%d) after target cmd %02hX %02hX %02hX %02hX %02hX %02hX %02hX %02hX %02hX %02hX",
						 LogIfString[m_ifId], (int)ret,
						 (unsigned short)pDevReq->CDBByte[0], (unsigned short)pDevReq->CDBByte[1], (unsigned short)pDevReq->CDBByte[2],
						(unsigned short)pDevReq->CDBByte[3], (unsigned short)pDevReq->CDBByte[4], (unsigned short)pDevReq->CDBByte[5],
						(unsigned short)pDevReq->CDBByte[6], (unsigned short)pDevReq->CDBByte[7], (unsigned short)pDevReq->CDBByte[8],
						(unsigned short)pDevReq->CDBByte[9]);
				ifStatus = STLINKIF_USB_COMM_ERR;
			} else {
				ifStatus = STLINKIF_NO_ERR;
			}
		} else {
			ifStatus = STLINKIF_NOT_SUPPORTED;
		}
	} else {
		ifStatus = STLINKIF_DLL_ERR;
	}
	return ifStatus;
}

#ifdef USING_ERRORLOG
/*
 * Associate files to be used for Error/Trace log (pErrLog must be initialized before)
 */
void STLinkInterface::BindErrLog(cErrLog *pErrLog)
{
	m_pErrLog = pErrLog;
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

