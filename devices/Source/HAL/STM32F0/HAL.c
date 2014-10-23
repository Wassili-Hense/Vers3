#include "../../config.h"

#ifdef STM32F0XX_MD

static uint32_t CriticalNesting = 0;

void INIT_SYSTEM(void)
{
    SystemInit();

    // Enable GPIO clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    CriticalNesting = 0;
}


void halEnterCritical(void)
{
    __disable_irq();

    CriticalNesting++;
    __asm volatile  ("dsb");
    __asm volatile  ("isb");
}
/*-----------------------------------------------------------*/

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
    uint32_t ticks = ((configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1UL);
    if(ticks > SysTick_LOAD_RELOAD_Msk)
        while(1);

    SysTick->LOAD = ticks;
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    SysTick->VAL  = 0;
    SysTick->CTRL  = (SysTick_CTRL_CLKSOURCE_Msk |
                      SysTick_CTRL_TICKINT_Msk   |
                      SysTick_CTRL_ENABLE_Msk);

    // Enable Global interrupts
    __enable_irq();
}

/*
void _delay_ms(uint32_t ms)
{
  ms *= (configCPU_CLOCK_HZ / 12000UL);
  while(ms > 0)
    ms--;
}

void _delay_us(uint32_t us)
{
  us *= (configCPU_CLOCK_HZ / 12000000UL);
  while(us > 0)
    us--;
}
*/

uint32_t SetInterruptMaskFromISR(void) __attribute__((naked));
uint32_t SetInterruptMaskFromISR(void)
{
    __asm volatile(
                    " mrs   r0, PRIMASK \n"
                    " cpsid i           \n"
                    " bx    lr"
                  );
    return 0;
}

void ClearInterruptMaskFromISR(uint32_t Mask)  __attribute__((naked));
void ClearInterruptMaskFromISR(uint32_t Mask)
{
    __asm volatile(
                    " msr   PRIMASK, r0 \n"
                    " bx    lr"
                  );
    (void)Mask;
}

// IRQ handlers.

void SysTick_Handler(void)
{
    uint32_t PreviousMask;

    PreviousMask = SetInterruptMaskFromISR();
    {
        SystemTick();
    }
    ClearInterruptMaskFromISR(PreviousMask);
}

void PendSV_Handler(void)
{
}

void SVC_Handler(void)
{
}

void HardFault_Handler( void ) __attribute__( ( naked ) );
void HardFault_Handler(void)
{
    __asm(  ".syntax unified\n"
                "MOVS   R0, #4  \n"
                "MOV    R1, LR  \n"
                "TST    R0, R1  \n"
                "BEQ    _MSP    \n"
                "MRS    R0, PSP \n"
                "B      HardFault_HandlerC      \n"
            "_MSP:  \n"
                "MRS    R0, MSP \n"
                "B      HardFault_HandlerC      \n"
            ".syntax divided\n") ;
}

void HardFault_HandlerC(unsigned long *hardfault_args)
{
    __asm("BKPT #0\n") ; // Break into the debugger
}
#endif  //  STM32F0XX_MD
