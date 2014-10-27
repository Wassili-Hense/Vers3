#ifndef __HAL_H
#define __HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#if   (defined STM32F0XX_MD)
#include "stm32f0xx.h"
#elif ((defined STM32F10X_MD) || (defined STM32F10X_MD_VL))
#include "stm32f10x.h"
#endif  //  uC Familie

void INIT_SYSTEM(void);

void halEnterCritical(void);
void halLeaveCritical(void);
#define ENTER_CRITICAL_SECTION      halEnterCritical
#define LEAVE_CRITICAL_SECTION      halLeaveCritical

#define portBYTE_ALIGNMENT          8
#define configTOTAL_HEAP_SIZE       1024

void StartSheduler(void);

void eeprom_init_hw(void);
void eeprom_read(uint8_t *pBuf, uint32_t Addr, uint32_t Len);
void eeprom_write(uint8_t *pBuf, uint32_t Addr, uint32_t Len);

void _delay_ms(uint32_t ms);
void _delay_us(uint32_t us);

uint16_t halRNG();

#ifdef __cplusplus
}
#endif

#endif  //  __HAL_H