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

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// MQTT-SN Section
#define MQTTSN_MSG_SIZE         48                // Size of payload(base for all buffers)

// Object Dictionary
#define OD_MAX_INDEX_LIST       16                // Size of identificators list
#define OD_DEV_SWVERSH          '2'               // Software Version
#define OD_DEV_SWVERSM          '7'
#define OD_DEV_SWVERSL          'a'               // Alfa

#define UART_BaudRate           38400

#include "HAL/HWconfig_A1Sn12.h"

#define configTICK_RATE_HZ      100

#include "memmang.h"
#include "mqTypes.h"
#include "mqttsn.h"
#include "ObjDict.h"

#endif  //  _CONFIG_H
