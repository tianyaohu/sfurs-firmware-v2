/* USART3 receive for skynet_link: PD8 TX / PD9 RX, the ST-LINK virtual COM port on the NUCLEO-H755ZI-Q,
 * so one USB cable to the Pi carries both SWD and skynet's commands. Register level because the CubeMX
 * project has no UART: adding one through the .ioc would regenerate files we keep stable. Receive only. */
#include "main.h"
#include "skynet_link.h"
#include "skynet_uart.h"

void skynet_uart_init(void)
{
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
  RCC->APB1LENR |= RCC_APB1LENR_USART3EN;
  (void)RCC->APB1LENR;
  RCC->D2CCIP2R &= ~RCC_D2CCIP2R_USART28SEL; /* kernel clock = PCLK1 */

  /* PD8/PD9 alternate function 7; pull-up on RX so an unplugged line idles high instead of framing noise. */
  GPIOD->MODER = (GPIOD->MODER & ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9)) |
                 (2U << GPIO_MODER_MODE8_Pos) | (2U << GPIO_MODER_MODE9_Pos);
  GPIOD->AFR[1] = (GPIOD->AFR[1] & ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9)) |
                  (7U << GPIO_AFRH_AFSEL8_Pos) | (7U << GPIO_AFRH_AFSEL9_Pos);
  GPIOD->PUPDR = (GPIOD->PUPDR & ~GPIO_PUPDR_PUPD9) | (1U << GPIO_PUPDR_PUPD9_Pos);

  USART3->CR1 = 0U;
  USART3->BRR = (HAL_RCC_GetPCLK1Freq() + SKYNET_LINK_BAUD / 2U) / SKYNET_LINK_BAUD;
  /* The 16-byte RX FIFO lets a slow main-loop pass or a masked interrupt lose nothing at 250 kBd. */
  USART3->CR1 = USART_CR1_FIFOEN | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_RE | USART_CR1_UE;
  NVIC_SetPriority(USART3_IRQn, 5U);
  NVIC_EnableIRQ(USART3_IRQn);
}

void USART3_IRQHandler(void)
{
  uint32_t isr = USART3->ISR;
  if (isr & USART_ISR_ORE) skynet_link_isr_overrun();
  /* A framing/noise/parity byte is still delivered: the record CRC rejects it. */
  USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
  while (isr & USART_ISR_RXNE_RXFNE) {
    skynet_link_isr_byte((uint8_t)USART3->RDR);
    isr = USART3->ISR;
  }
}
