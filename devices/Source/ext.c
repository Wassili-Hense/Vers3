/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#include "config.h"
#include "ext.h"

#ifdef EXTDIO_USED
#include "EXT/extdio.h"

#ifdef EXTPWM_USED
#include "EXT/extpwm.h"
#endif  //  EXTPWM_USED

#ifdef EXTAIN_USED
#include "EXT/extain.h"
#endif  //  EXTAIN_USED

#endif  //  EXTDIO_USED

#ifdef EXTTWI_USED
#include "EXT/exttwi.h"
#endif  //  EXTTWI_USED

#ifdef EXTSER_USED
#include "EXT/extser.h"
#endif  //  EXTSER_USED

#ifdef EXTPLC_USED
#include "EXT/extplc.h"
#endif  //  EXTPLC_USED

// Initialise extensions
void extInit(void)
{
#ifdef EXTDIO_USED
    dioInit();
#ifdef EXTAIN_USED
    ainInit();
#endif  //  EXTAIN_USED
#endif  //  EXTDIO_USED

#ifdef EXTTWI_USED
    twiInit();
#endif

#ifdef EXTSER_USED
    serInit();
#endif  //  EXTSER_USED

#ifdef EXTPLC_USED
    plcInit();
#endif  //  EXTPLC_USED
}

// Check Subindex: 0 - free / 1 - busy / 2 - invalid
bool extCheckSubidx(subidx_t * pSubidx)
{
    switch(pSubidx->Place)
    {
#ifdef EXTDIO_USED
        case objDin:
        case objDout:
            return dioCheckSubidx(pSubidx);

#ifdef EXTPWM_USED
        case objPWM:
            return pwmCheckSubidx(pSubidx);
#endif  //  EXTPWM_USED

#ifdef EXTAIN_USED
        case objAin:
            return ainCheckSubidx(pSubidx);
#endif  //  EXTAIN_USED
#endif  //  EXTDIO_USED

#ifdef EXTSER_USED
        case objSer:
            return serCheckSubidx(pSubidx);
#endif  //  EXTSER_USED
#ifdef EXTPLC_USED
        case objMerker:
            return plcCheckSubidx(pSubidx);
#endif  //  EXTPLC_USED
        default:
            break;
  }

  return false;
}

// Register Object
e_MQTTSN_RETURNS_t extRegisterOD(indextable_t * pIdx)
{
    pIdx->cbRead = NULL;
    pIdx->cbWrite = NULL;
    pIdx->cbPoll = NULL;

    switch(pIdx->sidx.Place)
    {
#ifdef EXTDIO_USED
        case objDin:        // Digital(bool) Input's
        case objDout:       // Digital(bool) Output's
            return dioRegisterOD(pIdx);

#ifdef EXTPWM_USED
        case objPWM:        // PWM
            return pwmRegisterOD(pIdx);
#endif  //  EXTPWM_USED

#ifdef EXTAIN_USED
        case objAin:        // Analog(int16_t) Input's
            return ainRegisterOD(pIdx);
#endif  //  EXTAIN_USED

#endif  //  EXTDIO_USED

#ifdef EXTSER_USED
        case objSer:        // User Serial I/O
            return serRegisterOD(pIdx);
#endif  //  EXTSER_USED

#ifdef EXTPLC_USED
        case objMerker:
            return plcRegisterOD(pIdx);
#endif  //  EXTPLC_USED
        default:
            break;
    }
    return MQTTSN_RET_REJ_NOT_SUPP;
}

// Delete Object
void extDeleteOD(subidx_t * pSubidx)
{
    // Delete Objects
    switch(pSubidx->Place)
    {
#ifdef EXTDIO_USED
        case objDin:
        case objDout:
            dioDeleteOD(pSubidx);
            break;

#ifdef EXTPWM_USED
        case objPWM:        // PWM
            pwmDeleteOD(pSubidx);
            break;
#endif  //  EXTPWM_USED
            
#ifdef EXTAIN_USED
        case objAin:
            ainDeleteOD(pSubidx);
            break;
#endif  //  EXTAIN_USED
#endif  //  EXTDIO_USED

#ifdef EXTSER_USED
        case objSer:
            serDeleteOD(pSubidx);
            break;
#endif  //  EXTSER_USED

        default:
            break;
    }
}

void extProc(void)
{
#ifdef EXTDIO_USED
    dioProc();
#ifdef EXTAIN_USED
    ainProc();
#endif  //  EXTAIN_USED

#endif  //  EXTDIO_USED

#ifdef EXTSER_USED
    serProc();
#endif  //  EXTSER_USED

#ifdef EXTPLC_USED
    plcProc();
#endif  // EXTPLC_USED
}
