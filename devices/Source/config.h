/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// Global configuration settings

#ifndef _CONFIG_H
#define _CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// MQTT-SN Section
#define MQTTSN_MSG_SIZE         48      // Size of payload(base for all buffers)
#define MQTTSN_USE_DHCP         1       // Use Automatic address resolution

// Object Dictionary
#define OD_MAX_INDEX_LIST       16      // Size of identificators list
#define OD_DEV_SWVERSH          '2'     // Software Version
#define OD_DEV_SWVERSM          '7'
#define OD_DEV_SWVERSL          'a'     // Alfa

#define configTICK_RATE_HZ      100     // System Tick Period

#define UART_BaudRate           38400
#define RF_BASE_FREQ            868300000UL

//Optional modules
#define EXTDIO_USED             1       // Use DIO

#define DIAG_USED               1       // Enable diagnostic messages

// Atmel
// ATM328P
#if     (defined CFG_A1EN12)
#include "HAL/HWconfig_A1En12.h"
#elif   (defined CFG_A1ES12)
#include "HAL/HWconfig_A1ES12.h"
#elif   (defined CFG_A1Sn12)
#include "HAL/HWconfig_A1Sn12.h"
// ATM1284P
#elif   (defined CFG_A3SC12)
#include "HAL/HWconfig_A3SC12.h"
#elif   (defined CFG_A3Sn12)
#include "HAL/HWconfig_A3Sn12.h"
#else
#error Undefined configuration
#endif  //  Configuration

#include "mqMEM.h"
#include "mqTypes.h"
#include "mqttsn.h"
#include "ObjDict.h"
#include "diag.h"

void StartSheduler(void);
void SystemTick(void);

#ifdef __cplusplus
}
#endif

#endif  //  _CONFIG_H
