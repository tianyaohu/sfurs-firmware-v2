/* Minimal interrupt vectors for the bootloader. The stock startup file
 * leaves every handler weak; without these, the HAL SysTick (enabled by
 * HAL_Init) traps the core in Default_Handler on the first tick.
 * Faults blink fast forever so a dead board still tells its story. */
#include "stm32g4xx_hal.h"

void SysTick_Handler(void)
{
  HAL_IncTick();
}

static void fault_trap(void)
{
  for (;;) {
    GPIOC->ODR ^= GPIO_PIN_6;
    for (volatile unsigned long n = 0; n < 400000UL; n++) {
      __asm__ volatile("nop");
    }
  }
}

void NMI_Handler(void) { fault_trap(); }
void HardFault_Handler(void) { fault_trap(); }
void MemManage_Handler(void) { fault_trap(); }
void BusFault_Handler(void) { fault_trap(); }
void UsageFault_Handler(void) { fault_trap(); }
