#include "../../config.h"

#ifdef EXTDIO_USED

static const GPIO_TypeDef * dio_pGPIOx[] = EXTDIO_PORTNUM2PORT;

static GPIO_TypeDef * dioPortNr2GPIOx(uint8_t PortNr)
{
  if(PortNr < EXTDIO_MAXPORT_NR)
    return (GPIO_TypeDef *)dio_pGPIOx[PortNr];

  return NULL;
}

void dioConfigPort(uint8_t PortNr, DIO_PORT_TYPE Mask, eDIOmode_t Mode)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  switch(Mode)
  {
    case DIO_MODE_IN_PD:
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
      GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
      break;
    case DIO_MODE_IN_PU:
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
      GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
      break;
    case DIO_MODE_OUT:
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
      GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
      break;
    case DIO_MODE_AIN:
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
      GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
      break;
    default:
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
      GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
      return;
  }

  GPIO_InitStructure.GPIO_Pin     = Mask;
  GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
  GPIO_Init(dioPortNr2GPIOx(PortNr), &GPIO_InitStructure);
}

DIO_PORT_TYPE dioReadPort(uint8_t PortNr)
{
  return GPIO_ReadInputData(dioPortNr2GPIOx(PortNr));
}

void dioWritePort(uint8_t PortNr, DIO_PORT_TYPE Mask, bool Value)
{
  if(Value)
    GPIO_SetBits(dioPortNr2GPIOx(PortNr), Mask);
  else
    GPIO_ResetBits(dioPortNr2GPIOx(PortNr), Mask);
}

#endif  //  EXTDIO_USED