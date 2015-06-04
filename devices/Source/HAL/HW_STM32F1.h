/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HW_STM32F1_H
#define HW_STM32F1_H

// STM32F10x
#if   (defined CFG_S3En12)
#include "HAL/HWconfig_S3En12.h"    // ENC28J60
#elif   (defined CFG_S3Sn12)
#include "HAL/HWconfig_S3Sn12.h"    // UART
#else
#error Unknown configuration
#endif  //  Configuration

#endif  //  HW_STM32F0_H

