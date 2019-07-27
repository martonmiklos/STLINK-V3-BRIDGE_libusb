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
#if defined(_MSC_VER) &&  (_MSC_VER >= 1000)
#include "stdafx.h"  // first include for windows visual studio
#endif

#include "criticalsectionlock.h"
#include "stlink_interface.h"
#ifdef WIN32 //Defined for applications for Win32 and Win64.
#include "shlwapi.h"
#endif // WIN32
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
#ifdef WIN32 //Defined for applications for Win32 and Win64.
// Create a critical section
static CRITICAL_SECTION g_csInterface;
#else
// critical section object (statically allocated)
static pthread_mutex_t g_csInterface =  PTHREAD_MUTEX_INITIALIZER;
#endif // WIN32

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
#ifdef WIN32 //Defined for applications for Win32 and Win64.
	m_hMod = NULL;
#endif
	m_pathOfProcess[0]='\0';

#ifdef WIN32 //Defined for applications for Win32 and Win64.
	// Initialize the critical section
	InitializeCriticalSection(&g_csInterface);
#else
	// nothing to do (statically allocated with PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP macro)
#endif
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
#ifdef WIN32
	// Delete the critical section
	DeleteCriticalSection(&g_csInterface);
	//Defined for applications for Win32 and Win64.
#ifdef USING_ERRORLOG
	// Flush the trace log system after closing
	if( m_pErrLog != NULL ) {
		m_pErrLog->Dump();
	}
#endif

	if( m_hMod != NULL )
	{
		if( FreeLibrary(m_hMod) != 0 )
		{
			// Successful
			m_hMod = NULL;
		}
	}
#else
	// critical section deletion not needed because static mutex

	STLink_FreeLibrary();
#endif // WIN32
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
		// Load the STLinkUSBDriver library only once in a session
		if( pPathOfProcess != NULL ) {
			// Memorize the path of process
#if defined(_MSC_VER) &&  (_MSC_VER >= 1400) /* VC8+ (VS2005) */
			::strncpy_s(m_pathOfProcess, MAX_PATH, pPathOfProcess, MAX_PATH);
#else
			::strncpy(m_pathOfProcess, pPathOfProcess, MAX_PATH);
#endif
		}

#ifdef WIN32 //Defined for applications for Win32 and Win64.
		if( m_hMod == NULL ) {
			// First try from this DLL path
			if( pPathOfProcess != NULL ) {
				char szDllPath[_MAX_PATH];
#if defined(_MSC_VER) &&  (_MSC_VER >= 1400) /* VC8+ (VS2005) */
				::strncpy_s(szDllPath, _MAX_PATH, m_pathOfProcess, _MAX_PATH);
#else
				::strncpy(szDllPath, m_pathOfProcess, _MAX_PATH);
#endif
				// Note: Unicode is not supported (would require T_CHAR szDllPath, to include <tchar.h> 
				// and to use generic function PathAppend(szDllPath, _T("STLinkUSBDriver.dll")) 
				::PathAppendA(szDllPath, "STLinkUSBDriver.dll");

				m_hMod = LoadLibraryA(szDllPath);
			}
		}

		if( m_hMod == NULL ) {
			// Second try using the whole procedure for path resolution (including PATH environment variable)
			m_hMod = LoadLibraryA("STLinkUSBDriver.dll");
		}

		if( m_hMod == NULL ) {
			LogTrace("STLinkInterface Failure loading STLinkUSBDriver.dll");
			ifStatus = STLINKIF_DLL_ERR;
		}

		if( ifStatus == STLINKIF_NO_ERR ) {
			LogTrace("STLinkInterface STLinkUSBDriver.dll loaded");
			if( m_ifId == STLINK_BRIDGE ) {
				// Get the needed API
				STLink_Reenumerate    = (pSTLink_Reenumerate)   GetProcAddress(m_hMod, ("STLink_Reenumerate"));
				STLink_GetNbDevices   = (pSTLink_GetNbDevices)  GetProcAddress(m_hMod, ("STLink_GetNbDevices"));
				STLink_GetDeviceInfo2 = (pSTLink_GetDeviceInfo2)GetProcAddress(m_hMod, ("STLink_GetDeviceInfo2"));
				STLink_OpenDevice     = (pSTLink_OpenDevice)    GetProcAddress(m_hMod, ("STLink_OpenDevice"));
				STLink_CloseDevice    = (pSTLink_CloseDevice)   GetProcAddress(m_hMod, ("STLink_CloseDevice"));
				STLink_SendCommand    = (pSTLink_SendCommand)   GetProcAddress(m_hMod, ("STLink_SendCommand"));

				// Check if DLL supports at least required function, this is mandatory but not enough for BRIDGE support
				// STLink_Reenumerate will return SS_BAD_PARAMETER if DLL is too old and Bridge interface is not supported.
				if( (STLink_Reenumerate == NULL) || (STLink_GetNbDevices == NULL) 
					|| (STLink_GetDeviceInfo2 == NULL) || (STLink_OpenDevice == NULL) || (STLink_CloseDevice == NULL)
					|| (STLink_SendCommand == NULL) ) {
					// TCP routines are required and missing
					ifStatus = STLINKIF_DLL_ERR;
				}
			}
		}
#else // !WIN32
        // nothing to do
#endif
		if( ifStatus == STLINKIF_NO_ERR ) {
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
STLinkIf_StatusT STLinkInterface::EnumDevices(uint32_t *pNumDevices, bool bClearList)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NOT_SUPPORTED;
	uint32_t status = SS_OK;

	if( pNumDevices != NULL ) {
		*pNumDevices=0; // default if error
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
	uint32_t status = SS_OK;

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
#ifdef WIN32
		if( STLink_GetDeviceInfo2 == NULL ) {
			// STLinkUSBDriver is too old 
			return STLINKIF_NOT_SUPPORTED;
		}
#endif

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

	// Create a Mutex to avoid concurrent access to STLink_SendCommand 
	CSLocker locker(g_csInterface);

	if( IsLibraryLoaded() == true ) {
		if( m_ifId == STLINK_BRIDGE ) {
			// UsbTimeoutMs if 0 use default (5s) else use UsbTimeoutMs.
			if( UsbTimeoutMs != 0 ) {
				usbTimeout = (uint32_t) UsbTimeoutMs;
			}
			ret=STLink_SendCommand(pHandle, pDevReq, usbTimeout);

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

