/**
  ******************************************************************************
  * @file    stlink_if_common.h
  * @author  MCD Development tools
  * @brief   Structure/define common to several interfaces (Bridge, Debug)
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
#ifndef _STLINK_IF_COMMON_H
#define _STLINK_IF_COMMON_H
/* Includes ------------------------------------------------------------------*/
#include "stlink_type.h"

/* Exported types ------------------------------------------------------------*/
/// @ingroup GENERAL
/// Extended version structure used to store STLink version returned by the
/// STLink (ST_RBC_CMD, ST_GETVERSION_EXT commands).
/// More detailed information can be found in application note TN1235
/// (Overview of ST-LINK derivatives) on st website.
typedef struct
{
    uint8_t Major_Ver;  ///< Major version ID (Vx)
    uint8_t Swim_Ver;   ///< Version for STM8 debug interface (SWIM protocol, Sx)
    uint8_t Jtag_Ver;   ///< Version for STM32 debug interface (JTAG and SWD protocols, Jx)
    uint8_t Msc_Ver;    ///< Version for mass storage and/or Virtual COM port interfaces (Mx)
    uint8_t Bridge_Ver; ///< Version for bridge interface
    uint8_t Power_Ver;  ///< 0, unused
    uint8_t Res1_Ver;   ///< 0, unused
    uint8_t Res2_Ver;   ///< 0, unused
    uint16_t VID;       ///< STlink USB Vendor ID
    uint16_t PID;       ///< STlink USB Product ID
} Stlk_VersionExtT, VERSION_EXT, *P_VERSION_EXT; // VERSION_EXT and *P_VERSION_EXT for legacy only

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif //_STLINK_IF_COMMON_H
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/