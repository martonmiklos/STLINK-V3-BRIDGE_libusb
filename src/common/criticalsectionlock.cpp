/**
  ******************************************************************************
  * @file    criticalsectionlock.cpp
  * @author  MCD Application Team
  * @brief   Critical section implementation.
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
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Class Functions Definition ------------------------------------------------*/
/*
 * Critical sections locker class.
 */
CSLocker::CSLocker(CriticalSection_ObjectT& cs): m_cs(cs)
{
#ifdef WIN32 //Defined for applications for Win32 and Win64.
	EnterCriticalSection(&m_cs);
#else // other platform
	// Enter the critical section -- other threads are locked out
	pthread_mutex_lock(&m_cs);
#endif


}

CSLocker::~CSLocker()
{
#ifdef WIN32 //Defined for applications for Win32 and Win64.
	LeaveCriticalSection(&m_cs);
#else  // other platform
	// Leave the critical section -- other threads can now pthread_mutex_lock()
	pthread_mutex_unlock(&m_cs);
#endif
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/