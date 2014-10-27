#include "../../config.h"

extern uint32_t SystemCoreClock;

static uint32_t CriticalNesting = 0;

void INIT_SYSTEM(void)
{
    SystemInit();

    // Enable GPIO clock
#if   (defined STM32F0XX_MD)
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
#elif (defined STM32F10X_MD)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
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
    if(SysTick_Config((SystemCoreClock / POLL_TMR_FREQ) - 1UL))
        while(1);

    // Enable Global interrupts
    __enable_irq();
}

// Main program tick procedure
void SystemTick(void);

// IRQ handlers.

void SysTick_Handler(void)
{
    uint32_t PreviousMask;

    PreviousMask = __get_PRIMASK();
    __disable_irq();

    SystemTick();

    __set_PRIMASK(PreviousMask);
}

void HardFault_Handler( void ) __attribute__( ( naked ) );
void HardFault_Handler(void)
{
    __disable_irq();
    while(1);
}

void _delay_ms(uint32_t ms)
{
  ms *= (SystemCoreClock / 12000UL);
  while(ms > 0)
    ms--;
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
