#include "../../config.h"

extern uint32_t SystemCoreClock;

static uint32_t CriticalNesting = 0;

void INIT_SYSTEM(void)
{
    __disable_irq();
    
    SystemInit();

    // Enable GPIO clock
#if   (defined __STM32F0XX_H)
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
#elif (defined __STM32F10x_H)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
#elif (defined STM32F4XX)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
#else
    #error unknown uC Family
#endif  //  STM32F0XX_MD

    CriticalNesting = 0;
}

void halEnterCritical(void)
{
    __disable_irq();

    CriticalNesting++;
    
    __DSB();    //  Data Synchronization Barrier
    __ISB();    //  Instruction Synchronization Barrier
}

void halLeaveCritical(void)
{
    if(CriticalNesting == 0)
        while(1);               // Error

    CriticalNesting--;
    if(CriticalNesting == 0 )
        __enable_irq();
}

void StartSheduler(void)
{
    if(SysTick_Config((SystemCoreClock / 1000) - 1UL))
        while(1);

    // Enable Global interrupts
    __enable_irq();
}

// Main program tick procedure
extern void SystemTick(void);

// IRQ handlers.

static uint16_t hal_ms_counter = 0;
static uint32_t hal_sec_counter = 0xFFFFFFF8;

// 1ms Ticks
void SysTick_Handler(void)
{
    hal_ms_counter++;
    if(hal_ms_counter > 999)
    {
        hal_ms_counter = 0;
        hal_sec_counter++;
    }
    
    static uint16_t ticks_counter = 0;

    if(ticks_counter < (const uint16_t)(1000/POLL_TMR_FREQ))
        ticks_counter++;
    else
    {
        SystemTick();
        ticks_counter = 1;
    }
/*
    uint32_t PreviousMask;

    PreviousMask = __get_PRIMASK();
    __disable_irq();

    hal_ticks_counter++;

    __set_PRIMASK(PreviousMask);
*/

}

void HardFault_Handler( void ) __attribute__( ( naked ) );
void HardFault_Handler(void)
{
    __disable_irq();
    while(1);
}

uint32_t hal_get_ms(void)
{
    uint32_t val = hal_sec_counter;
    val *= 1000;
    val += hal_ms_counter;
    return val;
/*
    uint32_t val = SysTick->VAL;
    val /= (const uint32_t)(SystemCoreClock/1000);
    val += hal_ticks_counter * POLL_TMR_FREQ;
*/
}

uint32_t hal_get_sec(void)
{
    return hal_sec_counter;
}

/*
uint32_t hal_get_us(void)
{
    uint32_t val = SysTick->VAL;
    val /= (const uint32_t)(SystemCoreClock/1000000);
    val += hal_ticks_counter * (const uint32_t)(POLL_TMR_FREQ * 1000);
    return val;
}
*/

void _delay_ms(uint32_t ms)
{
    if(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)
    {
        uint32_t new_ms = hal_get_ms() + ms;
        while(hal_get_ms() != new_ms);
    }
    else
    {
        ms *= (SystemCoreClock / 12000UL);
        while(ms > 0)
            ms--;
    }
}

void _delay_us(uint32_t us)
{
  us *= (SystemCoreClock / 12000000UL);
  while(us > 0)
    us--;
}

// Generate pseudo random uint16
uint16_t halRNG()
{
    static uint16_t rand16 = 0xA15E;

    // Galois LFSRs
    if(rand16 & 1)
    {
        rand16 >>= 1;
        rand16 ^= 0xB400;
    }
    else
        rand16 >>= 1;
  
    return rand16;
}
