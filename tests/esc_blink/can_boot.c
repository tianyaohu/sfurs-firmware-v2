/* Minimal polling receiver for G431 FDCAN1 at HSI16/PCLK1 = 16MHz.
 * Register offsets/fixed message-RAM geometry: RM0440 / STM32G431 CMSIS
 * FDCAN_GlobalTypeDef and CubeG4 HAL SRAMCAN_RF0SA/RF0_SIZE.
 * No transmit path, motor timers, interrupts, or heap. */
#include "can_boot.h"
#define REG(address) (*(volatile uint32_t *)(uintptr_t)(address))
#define CAN 0x40006400U
#define GPIOA 0x48000000U
#define GPIOB 0x48000400U

static int wait_init(uint32_t expected)
{
  for (unsigned n = 0; n < 100000U; ++n) {
    if ((REG(CAN + 0x18U) & 1U) == expected) return 1;
  }
  return 0; /* Failed CAN init must not strand the motor-disabled blinker. */
}

static void alternate(uint32_t port, unsigned bit)
{
  REG(port) = (REG(port) & ~(3U << (bit * 2U))) | (2U << (bit * 2U));
  REG(port + 0x0CU) &= ~(3U << (bit * 2U));
  unsigned shift = (bit - 8U) * 4U;
  REG(port + 0x24U) = (REG(port + 0x24U) & ~(15U << shift)) | (9U << shift);
}

int esc_blink_can_init(void)
{
  REG(0x4002104CU) |= 3U; /* GPIOA/B clock */
  (void)REG(0x4002104CU);
  alternate(GPIOA, 11U); /* FDCAN1_RX */
  alternate(GPIOB, 9U);  /* FDCAN1_TX (controller ACK bits only) */
  REG(0x40021088U) = (REG(0x40021088U) & ~(3U << 24)) | (2U << 24);
  REG(0x40021058U) |= 1U << 25;
  (void)REG(0x40021058U);
  REG(0x40021038U) |= 1U << 25;
  REG(0x40021038U) &= ~(1U << 25);
  REG(CAN + 0x18U) |= 1U;
  if (!wait_init(1U)) return 0;
  REG(CAN + 0x18U) |= 2U;
  REG(0x40006500U) = 0U; /* common FDCAN divider /1 */
  REG(CAN + 0x18U) = 0x103U; /* INIT, CCE, FD enabled; BRS off */
  /* 16MHz / 2 / (1+13+2) = 500kbit/s, SJW=2. */
  REG(CAN + 0x1CU) = (1U << 25) | (1U << 16) | (12U << 8) | 1U;
  REG(CAN + 0x0CU) = (1U << 16) | (12U << 8) | (1U << 4) | 1U;
  /* No filter entries: accept standard data into FIFO0, reject extended
   * and remote frames. Validate full frame header before reading payload. */
  REG(CAN + 0x80U) = 3U | (2U << 2);
  REG(CAN + 0x18U) &= ~1U;
  return wait_init(0U);
}

int esc_blink_can_poll(uint32_t *id, uint8_t payload[8])
{
  uint32_t status = REG(CAN + 0x90U);
  if ((status & 0xFU) == 0U) return 0;
  uint32_t index = (status >> 8) & 3U;
  if (index >= 3U) return 0; /* G431 has three 72-byte RX elements. */
  uint32_t address = 0x4000A4B0U + index * 72U;
  uint32_t word0 = REG(address), word1 = REG(address + 4U);
  int valid = (word0 & ((1U << 30) | (1U << 29))) == 0U &&
      (word1 & ((1U << 21) | (1U << 20) | (15U << 16))) ==
      ((1U << 21) | (8U << 16));
  if (valid) {
    *id = (word0 >> 18) & 0x7FFU;
    for (unsigned i = 0; i < 8U; ++i) {
      payload[i] = (uint8_t)(REG(address + 8U + (i / 4U) * 4U) >> ((i % 4U) * 8U));
    }
  }
  REG(CAN + 0x94U) = index; /* release even rejected/oversized frames */
  return valid;
}
