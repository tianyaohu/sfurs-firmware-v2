/* B-G431B-ESC1 CAN bootloader (16K @ 0x08000000).
 *
 * One-time SWD install; every later update arrives over CAN from the Nucleo.
 * Pairs with the app linked at 0x08004000 (STM32G431CBUX_FLASH_APP.ld).
 *
 * Behaviour: wait BOOT_WAIT_MS for ENTER; jump to a valid app on timeout.
 * Once ENTER is accepted the session owns the board until VERIFY+JUMP
 * (jump immediately) or an unrecoverable fault (stay in boot, blink).
 * A half-erased flash never jumps: erased_pages != 0 without VERIFY pins
 * the bootloader until a good image lands. Power-cycling restarts the
 * 3 s window from a clean session state (unless remote entry requests listening).
 *
 * Build with -DMOTOR_NE (or SE/SW/NW); see build_boot.py.
 */
#include "stm32g4xx_hal.h"
#include "can_fwu_protocol.h"
#include "can_fwu_boot.h"
#include "can_fwu_validity.h"
#include "boot_metadata.h"
#include "boot_entry.h"
#include "boot_rx.h"

#define APP_BASE 0x08004000U
#define APP_FIRST_PAGE 8U
#define APP_PAGE_COUNT 55U
#define METADATA_PAGE 63U
#define BOOT_WAIT_MS 3000U
#define BOOT_TX_TIMEOUT_MS 100U
_Static_assert(FLASH_PAGE_SIZE == CAN_FWU_PAGE_SIZE, "Unexpected flash page geometry");
_Static_assert(sizeof(struct can_fwu_validity) == 32U, "Unexpected metadata layout");

#if defined(MOTOR_NE)
#define BOOT_DEV 1U
#define BOOT_TERM_SET 1
#elif defined(MOTOR_SE)
#define BOOT_DEV 2U
#define BOOT_TERM_SET 1
#elif defined(MOTOR_SW)
#define BOOT_DEV 4U
#define BOOT_TERM_SET 0
#elif defined(MOTOR_NW)
#define BOOT_DEV 8U
#define BOOT_TERM_SET 0
#else
#error "Define one of MOTOR_NE, MOTOR_SE, MOTOR_SW, MOTOR_NW"
#endif

static FDCAN_HandleTypeDef hfdcan1;
static struct can_fwu_boot boot;
/* Diagnostics: frames matched to us vs ACKs fully transmitted (TXBTO seen).
 * Read via SWD; never cleared except by reset. */
volatile uint32_t boot_rx_cnt;
volatile uint32_t boot_txok_cnt;

void Error_Handler(void)
{
  /* Stay debuggable: rapid blink, no reset loop. */
  for (;;) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
    HAL_Delay(80);
  }
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }

  HAL_RCC_EnableCSS();
}

static void gpio_init(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /* All six gate-driver PWM inputs held LOW. No motor-control timer runs. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_12,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_PULLDOWN;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &g);
  g.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOC, &g);
  /* PC6 red user LED. */
  g.Pin = GPIO_PIN_6;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &g);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  /* PC14 CAN termination, same rule as the app: ends SET, middles RESET. */
  g.Pin = GPIO_PIN_14;
  HAL_GPIO_Init(GPIOC, &g);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,
                    BOOT_TERM_SET ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void fdcan_init(void)
{
  RCC_PeriphCLKInitTypeDef pc = {0};
  GPIO_InitTypeDef g = {0};
  FDCAN_FilterTypeDef filt = {0};

  pc.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  pc.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&pc) != HAL_OK) {
    Error_Handler();
  }
  __HAL_RCC_FDCAN_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  g.Pin = GPIO_PIN_11;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &g);

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  /* Identical nominal timing to the app: 170 MHz / 2 / 170 = 500 kbit/s. */
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 17;
  hfdcan1.Init.NominalTimeSeg1 = 152;
  hfdcan1.Init.NominalTimeSeg2 = 17;
  hfdcan1.Init.DataPrescaler = 17;
  hfdcan1.Init.DataSyncJumpWidth = 9;
  hfdcan1.Init.DataTimeSeg1 = 10;
  hfdcan1.Init.DataTimeSeg2 = 9;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
  /* Same mask trick as the app: filter=mask=BOOT_DEV passes our odd IDs. */
  filt.IdType = FDCAN_STANDARD_ID;
  filt.FilterIndex = 0U;
  filt.FilterType = FDCAN_FILTER_MASK;
  filt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filt.FilterID1 = BOOT_DEV;
  filt.FilterID2 = BOOT_DEV;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filt) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
}

/* Flash backend: host pages are app-relative, page 0 = 0x08004000. */
#ifndef BOOT_DIAGNOSTIC
static int boot_erase(uint32_t page, uint32_t n, void *ctx)
{
  FLASH_EraseInitTypeDef ei;
  uint32_t err = 0U;
  (void)ctx;
  if (n == 0U || page >= APP_PAGE_COUNT || n > APP_PAGE_COUNT - page) {
    return -1;
  }
  ei.TypeErase = FLASH_TYPEERASE_PAGES;
  ei.Banks = FLASH_BANK_1;
  ei.Page = APP_FIRST_PAGE + page;
  ei.NbPages = n;
  HAL_FLASH_Unlock();
  /* Persistent invalidation BEFORE touching any application page. */
  ei.Page = METADATA_PAGE;
  ei.NbPages = 1U;
  if (HAL_FLASHEx_Erase(&ei, &err) != HAL_OK || err != 0xFFFFFFFFU) {
    HAL_FLASH_Lock();
    return -1;
  }
  ei.Page = APP_FIRST_PAGE + page;
  ei.NbPages = n;
  if (HAL_FLASHEx_Erase(&ei, &err) != HAL_OK || err != 0xFFFFFFFFU) {
    HAL_FLASH_Lock();
    return -1;
  }
  HAL_FLASH_Lock();
  return 0;
}

static int boot_program(uint32_t off, uint64_t dw, void *ctx)
{
  (void)ctx;
  if ((off & 7U) != 0U || off > CAN_FWU_APP_MAX - 8U) {
    return -1;
  }
  HAL_FLASH_Unlock();
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_BASE + off, dw) !=
      HAL_OK) {
    HAL_FLASH_Lock();
    return -1;
  }
  HAL_FLASH_Lock();
  /* Read back what we wrote; G4 programs 0 bits only over erased cells. */
  if (*(volatile uint64_t *)(APP_BASE + off) != dw) {
    return -1;
  }
  return 0;
}

#endif
static int boot_motor_stopped(void *ctx)
{
  (void)ctx;
  return 1; /* Bootloader never starts the motor. */
}

static int app_valid(void)
{
  const struct can_fwu_validity *m =
      (const struct can_fwu_validity *)FWU_METADATA_BASE;
  uint32_t sp = *(uint32_t volatile *)APP_BASE;
  uint32_t pc = *(uint32_t volatile *)(APP_BASE + 4U);
  if (m->size < 8U || m->size > CAN_FWU_APP_MAX) return 0;
  return can_fwu_validity_valid(m, sp, pc,
      can_fwu_crc32((const uint8_t *)APP_BASE, m->size));
}

#ifndef BOOT_DIAGNOSTIC
static int metadata_erase(void *ctx)
{
  FLASH_EraseInitTypeDef ei = {0};
  uint32_t err = 0U;
  (void)ctx;
  ei.TypeErase = FLASH_TYPEERASE_PAGES;
  ei.Banks = FLASH_BANK_1;
  ei.Page = METADATA_PAGE;
  ei.NbPages = 1U;
  HAL_FLASH_Unlock();
  int ok = HAL_FLASHEx_Erase(&ei, &err) == HAL_OK && err == 0xFFFFFFFFU;
  HAL_FLASH_Lock();
  return ok ? 0 : -1;
}

static int metadata_program(unsigned word, uint64_t value, void *ctx)
{
  (void)ctx;
  uint32_t address = FWU_METADATA_BASE + word * 4U;
  HAL_FLASH_Unlock();
  int ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, value) == HAL_OK;
  HAL_FLASH_Lock();
  return ok && *(volatile uint64_t *)address == value ? 0 : -1;
}

static int commit_image(uint32_t size, uint32_t crc)
{
  uint32_t sp = *(uint32_t volatile *)APP_BASE;
  uint32_t pc = *(uint32_t volatile *)(APP_BASE + 4U);
  if (!can_fwu_vectors_valid(sp, pc, size) ||
      can_fwu_crc32((const uint8_t *)APP_BASE, size) != crc) return 0;
  const struct esc_metadata_ops ops = {metadata_erase, metadata_program, 0};
  int ok = esc_metadata_commit((const struct can_fwu_validity *)FWU_METADATA_BASE,
      size, crc, sp, pc, crc, &ops);
  return ok && app_valid();
}
#endif

/* Never run C stack accesses after replacing MSP. Pending interrupts are
 * cleared before unmasking; app Reset_Handler takes over immediately. */
__attribute__((naked, noreturn)) static void branch_app(
    uint32_t sp __attribute__((unused)), uint32_t pc __attribute__((unused)))
{
  __asm volatile("msr msp, r0\nmovs r2, #0\nmsr control, r2\n"
                 "isb\ncpsie i\nbx r1\n");
}

static void jump_to_app(void)
{
  uint32_t sp = *(uint32_t volatile *)APP_BASE;
  uint32_t pc = *(uint32_t volatile *)(APP_BASE + 4U);
  __disable_irq();
  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL = 0U;
  for (unsigned i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); ++i) {
    NVIC->ICER[i] = 0xFFFFFFFFU;
    NVIC->ICPR[i] = 0xFFFFFFFFU;
  }
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
  __HAL_RCC_FDCAN_FORCE_RESET();
  __HAL_RCC_FDCAN_RELEASE_RESET();
  /* RCC deinit uses HAL tick timeout paths, so allow SysTick for it. */
  __enable_irq();
  HAL_InitTick(0U);
  CLEAR_BIT(RCC->CR, RCC_CR_CSSON);
  if (HAL_RCC_DeInit() != HAL_OK) Error_Handler();
  __disable_irq();
  SysTick->CTRL = 0U;
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
  /* The app never touches VTOR (guarded off), so point it at the app now. */
  SCB->VTOR = APP_BASE;
  __DSB();
  __ISB();
  branch_app(sp, pc);
}

static int can_send(uint32_t id, const uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef h = {0};
  uint32_t mask, t0;
  h.Identifier = id;
  h.IdType = FDCAN_STANDARD_ID;
  h.TxFrameType = FDCAN_DATA_FRAME;
  h.DataLength = FDCAN_DLC_BYTES_8;
  h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  h.BitRateSwitch = FDCAN_BRS_OFF;
  h.FDFormat = FDCAN_FD_CAN;
  h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &h, data) != HAL_OK) {
    return 0;
  }
  mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
  t0 = HAL_GetTick();
  while ((hfdcan1.Instance->TXBTO & mask) == 0U) {
    if ((uint32_t)(HAL_GetTick() - t0) >= BOOT_TX_TIMEOUT_MS) {
      (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, mask);
      return 0;
    }
  }
  return 1;
}

static void led_waiting(uint32_t now)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,
                    ((now / 150U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void led_busy(uint32_t now)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,
                    ((now / 50U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int main(void)
{
  FDCAN_RxHeaderTypeDef rxh;
  uint8_t rxd[ESC_BOOT_RX_CAPACITY];
  int remote_entry = esc_boot_mailbox_consume(ESC_BOOT_MAILBOX, BOOT_DEV,
      (RCC->CSR & RCC_CSR_SFTRSTF) != 0U);
  __HAL_RCC_CLEAR_RESET_FLAGS();
  uint32_t ack_id, t0, now;
  uint8_t ack[8];
#ifdef BOOT_DIAGNOSTIC
  /* Defense in depth: diagnostic sessions cannot call flash operations. */
  struct can_fwu_flash_ops ops = {0, 0, boot_motor_stopped, 0};
#else
  struct can_fwu_flash_ops ops = {boot_erase, boot_program,
                                  boot_motor_stopped, 0};
#endif

  HAL_Init();
  SystemClock_Config();
  gpio_init();
  fdcan_init();
  can_fwu_boot_init(&boot, BOOT_DEV, &ops);
  t0 = HAL_GetTick();

  for (;;) {
    uint32_t n, i;
    now = HAL_GetTick();
    for (i = 0, n = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
         i < 8U && i < n; i++) {
      if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxh, rxd) !=
          HAL_OK) {
        break;
      }
      int header_valid = esc_boot_rx_header_valid(
          rxh.IdType == FDCAN_STANDARD_ID,
          rxh.RxFrameType == FDCAN_DATA_FRAME,
          rxh.FDFormat == FDCAN_FD_CAN,
          rxh.BitRateSwitch == FDCAN_BRS_OFF,
          rxh.DataLength == FDCAN_DLC_BYTES_8);
      if (!header_valid) {
        continue;
      }
#ifdef BOOT_DIAGNOSTIC
      /* Only ENTER is serviced. Reject all other addressed update commands
       * without invoking the session or touching application flash. */
      if (rxh.Identifier == can_fwu_id(CAN_FWU_ERASE_BASE, BOOT_DEV) ||
          rxh.Identifier == can_fwu_id(CAN_FWU_DATA_BASE, BOOT_DEV) ||
          rxh.Identifier == can_fwu_id(CAN_FWU_VERIFY_BASE, BOOT_DEV) ||
          rxh.Identifier == can_fwu_id(CAN_FWU_JUMP_BASE, BOOT_DEV)) {
        boot_rx_cnt++;
        uint32_t detail = 0U;
        uint32_t command = rxh.Identifier & ~0xFU;
        if (command == CAN_FWU_ERASE_BASE) detail = can_fwu_get_u32(rxd, 1);
        if (command == CAN_FWU_DATA_BASE) detail = can_fwu_data_seq(rxd);
        if (command == CAN_FWU_VERIFY_BASE) detail = can_fwu_get_u32(rxd, 0);
        can_fwu_ack_reply_pack(ack, CAN_FWU_REJECT, command, boot.session, detail);
        if (can_send(can_fwu_id(CAN_FWU_ACK_BASE, BOOT_DEV), ack)) {
          boot_txok_cnt++;
        }
        continue;
      }
#endif
      if (esc_boot_rx_frame(&boot, header_valid, rxh.Identifier, rxd, now, &ack_id, ack)) {
#ifndef BOOT_DIAGNOSTIC
        if (rxh.Identifier == can_fwu_id(CAN_FWU_VERIFY_BASE, BOOT_DEV) &&
             boot.state == FWU_B_VERIFIED && can_fwu_ack_status(ack) == CAN_FWU_OK) {
          if (!commit_image(boot.img_size, boot.img_crc)) {
            boot.state = FWU_B_FAULT;
            boot.fault = CAN_FWU_FLASH_ERR;
             can_fwu_ack_set_status(ack, CAN_FWU_FLASH_ERR);
          }
        }
#endif
        boot_rx_cnt++;
        if (can_send(ack_id, ack)) {
          boot_txok_cnt++;
        }
      }
    }
    can_fwu_boot_poll(&boot, now);

    if (boot.state == FWU_B_JUMP_PENDING) {
#ifndef BOOT_DIAGNOSTIC
      if (!app_valid()) {
        boot.state = FWU_B_FAULT;
        continue;
      }
#endif
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
      HAL_Delay(500);
#ifndef BOOT_BLINK_ONLY
      jump_to_app();
#endif
    }

    if (boot.state == FWU_B_FAULT) {
      /* Stay in boot after any fault. If pages were erased the flash is
       * half-written and must never jump; otherwise the old app is still
       * intact but an update was clearly attempted, so wait for a retry
       * instead of booting stale firmware under the operator.
       * Solid ON = fault, unmistakable against the blink patterns. */
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
      continue;
    }

    if (boot.state == FWU_B_IDLE || boot.state == FWU_B_ENTERED) {
      led_waiting(now);
#ifndef BOOT_BLINK_ONLY
      if (!remote_entry && boot.state == FWU_B_IDLE &&
          (uint32_t)(now - t0) >= BOOT_WAIT_MS && app_valid()) {
        jump_to_app();
      }
#endif
      continue;
    }

    /* ENTERED with traffic, ERASED, RECEIVING, VERIFIED: session owns us. */
    led_busy(now);
  }
}
