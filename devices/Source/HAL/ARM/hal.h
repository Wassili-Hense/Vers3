#ifndef __HAL_H
#define __HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef STM32F0XX_MD
#include "stm32f0xx.h"
#endif  //  STM32F0XX_MD

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

#ifdef __cplusplus
}
#endif

#endif  //  __HAL_H