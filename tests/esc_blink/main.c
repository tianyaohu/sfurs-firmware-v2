/* Motor-disabled B-G431B-ESC1 CAN-update acceptance payload.
 * Executes at 0x08004000, holds all six PWM inputs LOW, toggles PC6
 * every 500ms forever. Polls CAN only for guarded NE boot-entry requests;
 * no motor-control code, interrupts, or CAN application transmissions. */
#include <stdint.h>
#include "boot_entry.h"
#include "can_boot.h"
#define REG(addr) (*(volatile uint32_t *)(addr))
#define GPIOA 0x48000000U
#define GPIOB 0x48000400U
#define GPIOC 0x48000800U

static void outputs_low(uint32_t gpio, uint32_t pins)
{
  REG(gpio + 0x18U) = pins << 16; /* BSRR, reset all gate signals */
  for (unsigned bit = 0; bit < 16; ++bit) {
    if ((pins & (1U << bit)) != 0U) {
      uint32_t mode = REG(gpio);
      REG(gpio) = (mode & ~(3U << (bit * 2U))) | (1U << (bit * 2U));
    }
  }
}

void Reset_Handler(void)
{
  /* Bootloader restores HSI16 before handoff. */
  REG(0x4002104CU) |= 7U; /* GPIO A/B/C clocks */
  (void)REG(0x4002104CU);
  outputs_low(GPIOA, (1U << 8) | (1U << 9) | (1U << 10) | (1U << 12));
  outputs_low(GPIOB, 1U << 15);
  outputs_low(GPIOC, (1U << 13) | (1U << 6));
  REG(0xE000ED08U) = 0x08004000U; /* VTOR */
  REG(0xE000E014U) = 16000U - 1U; /* SysTick: HSI16, 1ms, polling */
  REG(0xE000E018U) = 0U;
  REG(0xE000E010U) = 5U; /* enable + CPU clock, no interrupt */
  int can_ready = esc_blink_can_init();
  for (;;) {
    for (unsigned tick = 0; tick < 500U; ++tick) {
      while ((REG(0xE000E010U) & (1U << 16)) == 0U) {
        uint32_t id;
        uint8_t payload[8];
        if (can_ready && esc_blink_can_poll(&id, payload) &&
            esc_boot_request_valid(id, payload, 1U)) {
          /* This payload never starts a motor and has no arm/calibration
           * state: its six gate inputs remain LOW throughout reset. */
          esc_boot_mailbox_write(ESC_BOOT_MAILBOX, 1U);
          __asm volatile("dsb" ::: "memory");
          REG(0xE000ED0CU) = 0x05FA0004U; /* SYSRESETREQ */
          __asm volatile("dsb" ::: "memory");
          for (;;) { }
        }
      }
    }
    REG(GPIOC + 0x14U) ^= 1U << 6;
  }
}

static void trap(void) { for (;;) { } }
extern uint32_t _estack;
typedef void (*handler)(void);
__attribute__((section(".isr_vector"), used))
const handler vectors[16] = {
  (handler)&_estack, Reset_Handler, trap, trap, trap, trap, trap, 0,
  0, 0, 0, trap, trap, 0, trap, trap
};
