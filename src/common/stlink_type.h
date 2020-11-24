/**
  ******************************************************************************
  * @file    stlink_type.h
  * @author  MCD Development tools
  * @brief   Include file for standard typedef according to platform
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
#ifndef _STLINK_TYPE_H
#define _STLINK_TYPE_H

#ifdef WIN32  //Defined for applications for Win32 and Win64.
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Insert your headers here
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

#include <windows.h>
#include <time.h>
//MSVC++ 11.0  _MSC_VER == 1700 (Visual Studio 2012 version 11.0)
//MSVC++ 12.0  _MSC_VER == 1800 (Visual Studio 2013 version 12.0)
#if defined(_MSC_VER) &&  (_MSC_VER >= 1800) // VC++ 12.0 (VS2013)
#include <cstdint>
#else
typedef signed char int8_t;
typedef short int16_t;
typedef int int32_t;

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

// bool; In C++, the type bool is built in (although it is a 1-byte type in C++ mode, rather than a 4-byte type like in C mode).
// BOOL defined in windef.h
#endif

#define EXPORTED_API extern "C"

#else // !WIN32: Linux/MacOS
#include <stdint.h>
#include "limits.h"
#include <string.h>
#include "time.h"

#define EXPORTED_API __attribute__ ((visibility ("default")))
#define MAX_PATH PATH_MAX
#define HANDLE void *
#define PHANDLE void **
#define PVOID  void *
#define BYTE  unsigned char
#define CHAR  char
#define TCHAR char
#define WORD  unsigned short // unsigned 16 
#define DWORD unsigned int  // unsigned 32 bits
#define LONG int  // int for signed 32 bits; Note that "long" is signed 32 bits on Windows, signed 64 bits on Linux/MacOS => dangerous
#define LPSTR char *
#define INVALID_HANDLE_VALUE 0xFFFFFFFF
typedef enum {FALSE=0, TRUE} BOOL;
#endif // WIN32

#endif //_STLINK_TYPE_H
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/