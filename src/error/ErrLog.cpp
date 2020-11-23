/**
  ******************************************************************************
  * @file    ErrLog.cpp
  * @author  MCD Application Team
  * @brief   All tracing mechanisms (Error, Trace)
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
#include "ErrLog.h"
#include <stdio.h> // fopen... 
#ifdef WIN32
#include <stdlib.h> // for _countof
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// static variables

/* Global variables ----------------------------------------------------------*/
/* Class Functions Definition ------------------------------------------------*/

/*****************************************************************************/    
cErrLog::cErrLog()
{
	// To be completed: implementation defined
}

/*****************************************************************************/    
void cErrLog::Init (const char *pSzFileName, bool bResetFile)
{
	// To be completed: implementation defined
}
/*****************************************************************************/    
void cErrLog::Dump()
{
	// To be completed: implementation defined
	// possible implementation:
	// if records available, open errorLog file
	// copy errorLog buffer in the errorLog file, add timestamp ...
	// close errorLog file
	// reset errorLog buffer
}

void cErrLog::LogTrace(const char *pMessage, ...)
{
	// Trace the specified string into log file

	va_list args; // used to manage the variable argument list
	va_start(args, pMessage);     

	LogTrace(pMessage, args);

	va_end(args);     
}

void cErrLog::LogTrace(const char *pMessage, va_list Args)
{
	// Trace the specified string into log file
	// To be completed: implementation defined
	// possible implementation: write in a buffer or in a file
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/