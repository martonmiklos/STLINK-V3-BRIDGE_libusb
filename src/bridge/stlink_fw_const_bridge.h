/**
  ******************************************************************************
  * @file    stlink_fw_const_bridge.h
  * @author  MCD Development tools
  * @brief   Definition of all constants shared between the STLink firmware
  *          and PC application for Bridge interface
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
#ifndef _STLINK_FW_CONST_BRIDGE_H
#define _STLINK_FW_CONST_BRIDGE_H
/* Includes ------------------------------------------------------------------*/
/* API shared with embedded code ---------------------------------------------*/
#include "stlink_fw_api_bridge.h"
#include "stlink_fw_api_common.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Up-to-date firmware version for upgrade message for STLink v3--------------*/
#define FIRMWARE_BRIDGE_STLINK_V3_LAST_VERSION  3

/* Firmware version supporting CAN interface ---------------------------------*/
#define FIRMWARE_BRIDGE_MIN_VER_FOR_CAN  2
/* Firmware version supporting I2C READ_NO_WAIT related commands -------------*/
#define FIRMWARE_BRIDGE_MIN_VER_FOR_READ_NO_WAIT_I2C 3

/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#endif //_STLINK_FW_CONST_BRIDGE_H
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/