#ifndef __HAL_H
#define __HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
#include <avr/io.h>
#include <avr/eeprom.h>

#define INIT_SYSTEM()

#define ENTER_CRITICAL_SECTION()    asm volatile ( "in      __tmp_reg__, __SREG__" :: );    \
                                    asm volatile ( "cli" :: );                              \
                                    asm volatile ( "push    __tmp_reg__" :: )

#define LEAVE_CRITICAL_SECTION()    asm volatile ( "pop     __tmp_reg__" :: );              \
                                    asm volatile ( "out     __SREG__, __tmp_reg__" :: )

#define eeprom_init_hw()
#define eeprom_read(pBuf, Addr, Len)  eeprom_read_block((void *)pBuf, (const void *)Addr, (size_t)Len)
#define eeprom_write(pBuf, Addr, Len) eeprom_write_block((const void *)pBuf, (void *)Addr, (size_t)Len)

// AVR Architecture specifics.
#define DIO_PORT_SIZE               8
#define portBYTE_ALIGNMENT          1
#define portPOINTER_SIZE_TYPE       uintptr_t
#define configTOTAL_HEAP_SIZE       1024

// GPIO Types
typedef enum
{
    DIO_MODE_IN_FLOAT   = 0,
    DIO_MODE_IN_PD      = 0,
    DIO_MODE_IN_PU      = 0x01,

    DIO_MODE_OUT_PP     = 0x08,
    
    DIO_MODE_AIN        = 0x18
}DIOmode_e;

/*
// GPIO Types
typedef enum
{
    DIO_MODE_IN_FLOAT = 0,
    DIO_MODE_IN_PD,
    DIO_MODE_IN_PU,
    DIO_MODE_OUT,
    DIO_MODE_PWM,
    DIO_MODE_AIN,
    DIO_MODE_TWI
}eDIOmode_t;

// GPIO Types
typedef enum
{
    DIO_MODE_IN_FLOAT   = 0,
    DIO_MODE_IN_PU      = 0x01,
    DIO_MODE_IN_PD      = 0x02,
    
    DIO_MODE_OUT_PP     = 0x08,
    DIO_MODE_OUT_OD     = 0x0C,
    DIO_MODE_OUT_PP_MS  = 0x28,     // Output, push-pull, medium speed

    DIO_MODE_AF_PP      = 0x10,
    DIO_MODE_AF_PU      = 0x11,
    DIO_MODE_AF_PD      = 0x12,
    DIO_MODE_AF_OD      = 0x14,
    DIO_MODE_AF_PP_MS   = 0x30,     // Alternative function, Push/pull, medium speed

    DIO_MODE_AIN        = 0x18
}DIOmode_e;
*/

uint16_t halRNG();

#ifdef __cplusplus
}
#endif


#endif  //  __HAL_H