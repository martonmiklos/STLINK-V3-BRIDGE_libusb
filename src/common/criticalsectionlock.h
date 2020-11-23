/**
  ******************************************************************************
  * @file    criticalsectionlock.h
  * @author  MCD Application Team
  * @brief   Critical section header.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CRITICALSECTIONLOCK_H
#define _CRITICALSECTIONLOCK_H


/* Includes ------------------------------------------------------------------*/
#ifdef WIN32 //Defined for applications for Win32 and Win64.
#include "windows.h"
#else
#include <pthread.h>
#endif
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Class -------------------------------------------------------------------- */
/*
 * Critical sections locker class.
 **/
// Critical section or Mutex Object type according to OS platform
#ifdef WIN32 // windows platform
#define CriticalSection_ObjectT CRITICAL_SECTION
#else // other platform
#define CriticalSection_ObjectT pthread_mutex_t
#endif

class CSLocker
{
	public:
	CSLocker(CriticalSection_ObjectT& cs);

	~CSLocker(void);

	private:
	CriticalSection_ObjectT&  m_cs;
};

#endif //_CRITICALSECTIONLOCK_H
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
