/**
  ******************************************************************************
  * @file    ErrLog.h
  * @author  MCD Application Team
  * @brief   Header for ErrLog.cpp module. All tracing mechanisms:
	1. ErrorLog: circular buffer always filled in, that can be dumped into
	    a file after an error for instance
	2. TraceLog: immediate trace logging in a file (once activated, the file
	   is opened then closed on each trace action)
	3. TraceFunctionCall: manages automatic indentation for tracing function
	   callings in ErrorLog and/or TraceLog systems. In order to trace a function,
		 just instantiate the object.
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
#ifndef ERRLOG_H
#define ERRLOG_H

#include "stlink_type.h" // for MAX_PATH if required
/* Includes ------------------------------------------------------------------*/
#ifndef WIN32 //Linux MacOS (not Win32 and Win64)
#include <stdarg.h> // for va_list
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/

/* Class -------------------------------------------------------------------- */
class cErrLog 	// To be completed: implementation defined
{
public:
	cErrLog();
	// to be completed: implementation defined
	// bResetFile == true clear the content of the file if it exists
	void Init (const char *pSzFileName, bool bResetFile);
	// Flush the file if not already done in LogTrace: to be completed: implementation defined
	void Dump();

	// General log trace routine: to be completed: implementation defined
	void LogTrace(const char *pMessage, ...);
	void LogTrace(const char *pMessage, va_list Args);

private:

	// To be completed: implementation defined
};

#endif /* ERRLOG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

