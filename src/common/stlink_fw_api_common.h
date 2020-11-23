/**
  ******************************************************************************
  * @file    stlink_fw_api_common.h
  * @author  MCD Development tools
  * @brief   File for firmware Open API common to several interface defines
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
#ifndef _STLINK_FW_API_COMMON_H
#define _STLINK_FW_API_COMMON_H
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/*  Common to several interface API ------------------------------------------*/
#define ST_RBC_CMD                0xF1 // ST_GETVERSION
#define ST_GETVERSION_EXT         0xFB // New in ST-Link/V3 (to be used instead
                                       // ST_GETVERSION to get the full version)
#define STLINK_GET_TARGET_VOLTAGE 0xF7 // New in ST-Link/V2
#define STLINK_DRIVE_TARGET_PWR   0xFD // New in ST-Link/V2 from version V2J34
                                       // and in STLINK-V3  from version V3J4
#define STLINK_DRIVE_TARGET_PWR_DISABLE 0    // Parameter for STLINK_DRIVE_TARGET_PWR
#define STLINK_DRIVE_TARGET_PWR_ENABLE  1    // Parameter for STLINK_DRIVE_TARGET_PWR

/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#endif //_STLINK_FW_API_COMMON_H
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/