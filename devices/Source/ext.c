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
#endif  //  EXTDIO_USED

#ifdef EXTAIN_USED
#include "EXT/extain.h"
#endif  //  EXTAIN_USED

// Initialise extensions
void extInit(uint8_t *pBuf)
{
#ifdef EXTDIO_USED
  dioInit(&pBuf[EXTDIO_BASE>>3]);
#endif  //  EXTDIO_USED

#ifdef EXTAIN_USED
  ainInit(&pBuf[EXTAIN_BASE>>3]);
#endif  //  EXTAIN_USED
}

// Check Subindex ->free/busy/invalid
uint16_t extCheckIdx(subidx_t * pSubidx)
{
  switch(pSubidx->Place)
  {
#ifdef EXTDIO_USED
    case objDin:
    case objDout:
      return dioCheckIdx(pSubidx);
#endif  //  EXTDIO_USED
#ifdef EXTAIN_USED
    case objAin:
      return ainCheckIdx(pSubidx);
#endif  //  EXTAIN_USED
    default:
      break;
  }
  return 0xFFFF;
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
#endif  //  EXTDIO_USED
#ifdef EXTAIN_USED
    case objAin:        // Analog(int16_t) Input's
      return dioRegisterOD(pIdx);
#endif  //  EXTAIN_USED
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
#endif  //  EXTDIO_USED
#ifdef EXTAIN_USED
    case objAin:
      ainDeleteOD(pSubidx);
      break;
#endif  //  EXTAIN_USED
    default:
      break;
  }
}

void extProc(void)
{
#ifdef EXTDIO_USED
  dioProc();
#endif  //  EXTDIO_USED
#ifdef EXTAIN_USED
  ainProc();
#endif  //  EXTAIN_USED
}
