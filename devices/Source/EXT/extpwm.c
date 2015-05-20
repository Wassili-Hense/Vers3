/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// Extensions PWM

#include "../config.h"

#if ((defined EXTDIO_USED) && (defined EXTPWM_USED))

#include "extpwm.h"

/////////////////////////////////////////////////////
// PWM Section ,   depended from extdio.c

// Local Variables
static const uint8_t pwm_port2cfg[EXTPWM_MAXPORT_NR] = EXTPWM_PORT2CFG;
static const uint8_t pwm_port2dio[EXTPWM_MAXPORT_NR] = EXTPWM_PORT2DIO;

// dio subroutines
uint8_t dioCheckBase(uint16_t base);
void dioConfigure(uint16_t base, eDIOmode_t Mode);

// hal
void hal_pwm_write(uint8_t Config, uint16_t value);

// Local subroutines

static uint8_t pwmCheckBase(uint16_t base)
{
    if((base >= EXTPWM_MAXPORT_NR) ||
       (pwm_port2cfg[base] == 0xFF))        // Port not exist
        return 2;
        
    return dioCheckBase(pwm_port2dio[base]);
}

// Check index
uint8_t pwmCheckIdx(subidx_t * pSubidx)
{
    if((pSubidx->Type == objPinNPN) || (pSubidx->Type == objPinPNP))
        return pwmCheckBase(pSubidx->Base);

    return 2;
}

static e_MQTTSN_RETURNS_t pwmWriteOD(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf)
{
    uint16_t value = pBuf[1];
    value <<= 8;
    value |= pBuf[0];
    if(pSubidx->Type == objPinNPN)
        value = ~value;

    hal_pwm_write(pwm_port2cfg[pSubidx->Base], value);

    return MQTTSN_RET_ACCEPTED;
}

// Register PWM Object
e_MQTTSN_RETURNS_t pwmRegisterOD(indextable_t *pIdx)
{
    uint16_t base = pIdx->sidx.Base;
    if(pwmCheckBase(base) != 0)
        return MQTTSN_RET_REJ_INV_ID;

    // configure port and mark as busy
    dioConfigure(pwm_port2dio[base], DIO_MODE_PWM);

    uint8_t cfg = pwm_port2cfg[base];

    if(pIdx->sidx.Type == objPinPNP)
        hal_pwm_write(cfg, 0);
    else
        hal_pwm_write(cfg, 0xFFFF);

    pIdx->cbWrite = &pwmWriteOD;

    return MQTTSN_RET_ACCEPTED;
}

void pwmDeleteOD(subidx_t * pSubidx)
{
    uint16_t base = pSubidx->Base;
    
    if(pwmCheckBase(base) != 1)
        return;
    
    hal_pwm_write(pwm_port2cfg[base], 0);
    base = pwm_port2dio[base];

    // Release PIN
    dioConfigure(base, DIO_MODE_IN_FLOAT);
}

#endif  // ((defined EXTDIO_USED) && (defined EXTPWM_USED))


