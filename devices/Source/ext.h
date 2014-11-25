/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _EXT_H_
#define _EXT_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  DIO_MODE_IN_FLOAT = 0,
  DIO_MODE_IN_PD,
  DIO_MODE_IN_PU,
  DIO_MODE_OUT,
  DIO_MODE_PWM,
  DIO_MODE_AIN,
  DIO_MODE_UART
}eDIOmode_t;

void extInit(void);                                         // Initialise extensions
uint8_t extCheckIdx(subidx_t * pSubidx);                    // Check Subindex ->free/busy/invalid
e_MQTTSN_RETURNS_t extRegisterOD(indextable_t * pIdx);      // Register Object
void extDeleteOD(subidx_t * pSubidx);                       // Delete Object
void extProc(void);                                         // Update IO's

#ifdef __cplusplus
}
#endif

#endif  //  _EXT_H_
