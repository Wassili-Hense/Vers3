/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HW_ATMEGA_H
#define HW_ATMEGA_H

#if     (defined CFG_A1Cn12)        // CC1101
#include "HWconfig_A1Cn12.h"
#elif   (defined CFG_A1En12)        // ENC28J60
#include "HWconfig_A1En12.h"
#elif   (defined CFG_A1ES12)        // ENC28J60 + UART
#include "HWconfig_A1ES12.h"
#elif   (defined CFG_A1SC12)        // UART + CC1101
#include "HWconfig_A1SC12.h"
#elif   (defined CFG_A1Sn12)        // UART
#include "HWconfig_A1Sn12.h"
#elif   (defined CFG_A1SR11)        // RFM12 vers. 1.1
#include "HWconfig_A1SR11.h"
#elif   (defined CFG_A1RN11)        // RFM12 vers. 1.1, node
#include "HWconfig_A1Rn11.h"
// ATM1284P
#elif   (defined CFG_A3SC12)        // UART + CC1101
#include "HWconfig_A3SC12.h"
#elif   (defined CFG_A3Sn12)        // UART
#include "HWconfig_A3Sn12.h"
// ATM2560
#elif   (defined CFG_A4En12)
#include "HWconfig_A4En12.h"        // ENC28J60
#elif   (defined CFG_A4ES12)
#include "HWconfig_A4ES12.h"        // ENC28J60 + UART
#elif   (defined CFG_A4Sn12)
#include "HWconfig_A4Sn12.h"        // UART
#else
#error Unknown configuration
#endif  //  Configuration

#endif  //  HW_ATMEGA_H