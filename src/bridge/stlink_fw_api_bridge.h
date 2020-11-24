/**
  ******************************************************************************
  * @file    stlink_fw_api_bridge.h
  * @author  MCD Development tools
  * @brief   File for firmware Open Bridge API defines
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
#ifndef _STLINK_FW_API_BRIDGE_H
#define _STLINK_FW_API_BRIDGE_H

/*  BRIDGE dedicated API -----------------------------------------------------*/
#define STLINK_BRIDGE_COMMAND            0xFC // New in ST-Link/V3 from version B1

/* BRIDGE Commands codes -----------------------------------------------------*/
#define STLINK_BRIDGE_CLOSE               0x01
#define STLINK_BRIDGE_GET_RWCMD_STATUS    0x02
#define STLINK_BRIDGE_GET_CLOCK           0x03
// SPI
#define STLINK_BRIDGE_INIT_SPI            0x20
#define STLINK_BRIDGE_WRITE_SPI           0x21
#define STLINK_BRIDGE_READ_SPI            0x22
#define STLINK_BRIDGE_CS_SPI              0x23
//I2C
#define STLINK_BRIDGE_INIT_I2C            0x30
#define STLINK_BRIDGE_WRITE_I2C           0x31
#define STLINK_BRIDGE_READ_I2C            0x32
#define STLINK_BRIDGE_READ_NO_WAIT_I2C    0x33 // new in V3B3
#define STLINK_BRIDGE_GET_READ_DATA_I2C   0x34 // new in V3B3

//CAN
#define STLINK_BRIDGE_INIT_CAN             0x40
#define STLINK_BRIDGE_WRITE_MSG_CAN        0x41

#define STLINK_BRIDGE_INIT_FILTER_CAN      0x43
#define STLINK_BRIDGE_START_MSG_RECEPTION_CAN 0x44
#define STLINK_BRIDGE_STOP_MSG_RECEPTION_CAN  0x45
#define STLINK_BRIDGE_GET_NB_RXMSG_CAN     0x46
#define STLINK_BRIDGE_GET_RXMSG_CAN        0x47

//GPIO
#define STLINK_BRIDGE_INIT_GPIO           0x60
#define STLINK_BRIDGE_SET_RESET_GPIO      0x61
#define STLINK_BRIDGE_READ_GPIO           0x62


/* BRIDGE Status codes -------------------------------------------------------*/
#define STLINK_BRIDGE_OK                  0x80 // use same code as STLINK_JTAG_OK 
#define STLINK_BRIDGE_SPI_ERROR           0x02
#define STLINK_BRIDGE_I2C_ERROR           0x03
#define STLINK_BRIDGE_CAN_ERROR           0x04

#define STLINK_BRIDGE_INIT_NOT_DONE       0x07
#define STLINK_BRIDGE_UNKNOWN_CMD         0x08
#define STLINK_BRIDGE_BAD_PARAM           0x09
#define STLINK_BRIDGE_TIMEOUT_ERR         0x0A
#define STLINK_BRIDGE_ABORT_TRANS         0x0B
#define STLINK_BRIDGE_INTERNAL_ERR        0x0C
#define STLINK_BRIDGE_CMD_BUSY            0x0D

// supported BRIDGE interface for com parameter
#define STLINK_SPI_COM 0x02
#define STLINK_I2C_COM 0x03
#define STLINK_CAN_COM 0x04
#define STLINK_GPIO_COM 0x06

// define for READ_MSG_CAN
#define CAN_MSG_FORMAT_V1 1
#define CAN_READ_MSG_HEADER_SIZE_V1 8
#define CAN_READ_MSG_DATA_SIZE_V1 8
#define CAN_READ_MSG_SIZE_V1  (CAN_READ_MSG_HEADER_SIZE_V1+CAN_READ_MSG_DATA_SIZE_V1) // 8 bytes for header(ID/DLC...)+ 8 bytes for data


#endif //_STLINK_FW_API_BRIDGE_H
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/