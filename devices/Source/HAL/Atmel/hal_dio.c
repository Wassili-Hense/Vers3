#include <avr/pgmspace.h>

const uint16_t dio_portnum2port[]  = EXTDIO_PORTNUM2PORT;

void dioConfigPort(uint8_t PortNr, DIO_PORT_TYPE Mask, eDIOmode_t Mode)
{
  uint16_t base = dio_portnum2port[PortNr];

  uint8_t *pPORT;
  pPORT = (uint8_t *)base;
  uint8_t *pDDR;
  pDDR = (uint8_t *)(base - 1);

  switch(Mode)
  {
    case DIO_MODE_IN_PU:
      *pPORT |= Mask;
      *pDDR  &= ~Mask;
      break;
    case DIO_MODE_OUT:
      *pDDR  |= Mask;
      break;
    default:
      *pPORT &= ~Mask;
      *pDDR  &= ~Mask;
      break;
  }
}

DIO_PORT_TYPE dioReadPort(uint8_t PortNr)
{
  uint8_t *pPIN;
  pPIN = (uint8_t *)(dio_portnum2port[PortNr] - 2);
  return *pPIN;
}

void dioWritePort(uint8_t PortNr, DIO_PORT_TYPE Mask, bool Value)
{
  uint8_t *pPORT;
  pPORT = (uint8_t *)dio_portnum2port[PortNr];
  
  if(Value)
    *pPORT |= Mask;
  else
    *pPORT &= ~Mask;
}
