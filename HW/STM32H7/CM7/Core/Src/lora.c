#include "lora.h"

static lora_t lora_instance;

// Create LoRa object
lora_t *lora_create(SPI_HandleTypeDef *spi,
                    GPIO_TypeDef *nss_port, uint16_t nss_pin,
                    GPIO_TypeDef *reset_port, uint16_t reset_pin,
                    GPIO_TypeDef *enable_port, uint16_t enable_pin) {

    lora_t *lora = &lora_instance;

    lora->spi = spi;

    lora->nss_port = nss_port;
    lora->nss_pin  = nss_pin;

    lora->reset_port = reset_port;
    lora->reset_pin  = reset_pin;

    lora->enable_port = enable_port;
    lora->enable_pin  = enable_pin;

    // Default frequency
    lora->frequency = 915E6;

    // Initialize pins high
    HAL_GPIO_WritePin(lora->enable_port, lora->enable_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(lora->nss_port, lora->nss_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(lora->reset_port, lora->reset_pin, GPIO_PIN_SET);

    return lora;
}

// Begin LoRa
bool lora_begin(lora_t *lora, long frequency) {
    // Reset LoRa module
    HAL_GPIO_WritePin(lora->reset_port, lora->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(lora->reset_port, lora->reset_pin, GPIO_PIN_SET);
    HAL_Delay(10);

    uint8_t version = lora_read_register(lora, REG_VERSION);
    if(version != 0x12) return false;

    lora_sleep(lora);
    lora_set_frequency(lora, frequency);

    lora_write_register(lora, REG_FIFO_TX_BASE_ADDR, 0);
    lora_write_register(lora, REG_FIFO_RX_BASE_ADDR, 0);

    uint8_t lna = lora_read_register(lora, REG_LNA);
    lora_write_register(lora, REG_LNA, lna | 0x03);

    lora_idle(lora);

    return true;
}

void lora_end(lora_t *lora) {
    lora_sleep(lora);
}

// Sleep / Idle
void lora_sleep(lora_t *lora) {
    lora_write_register(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void lora_idle(lora_t *lora) {
    lora_write_register(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

// Set frequency
void lora_set_frequency(lora_t *lora, long frequency) {
    lora->frequency = frequency;
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    lora_write_register(lora, REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_register(lora, REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_register(lora, REG_FRF_LSB, (uint8_t)(frf >> 0));
}

// Set TX power
void lora_set_tx_power(lora_t *lora, int level, bool use_pa_boost) {
    if(use_pa_boost){
        if(level < 2) level = 2;
        if(level > 17) level = 17;
        lora_write_register(lora, REG_PA_CONFIG, PA_BOOST | (level - 2));
    } else {
        if(level < 0) level = 0;
        if(level > 14) level = 14;
        lora_write_register(lora, REG_PA_CONFIG, level);
    }
}

// SPI read/write
uint8_t lora_transfer(lora_t *lora, uint8_t address, uint8_t value) {
    uint8_t tx[2] = {address, value};
    uint8_t rx[2] = {0,0};

    HAL_GPIO_WritePin(lora->nss_port, lora->nss_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(lora->spi, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(lora->nss_port, lora->nss_pin, GPIO_PIN_SET);

    return rx[1];
}

uint8_t lora_read_register(lora_t *lora, uint8_t address) {
    return lora_transfer(lora, address & 0x7F, 0x00);
}

void lora_write_register(lora_t *lora, uint8_t address, uint8_t value) {
    lora_transfer(lora, address | 0x80, value);
}

// Send packet
void lora_send_packet(lora_t *lora, const uint8_t *data, uint8_t length) {
    lora_idle(lora);

    lora_write_register(lora, REG_IRQ_FLAGS, 0xFF);
    lora_write_register(lora, REG_FIFO_ADDR_PTR, 0);

    for(uint8_t i=0; i<length; i++){
        lora_write_register(lora, REG_FIFO, data[i]);
    }

    lora_write_register(lora, REG_PAYLOAD_LENGTH, length);
    lora_write_register(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    while((lora_read_register(lora, REG_IRQ_FLAGS) & 0x08) == 0);
    lora_write_register(lora, REG_IRQ_FLAGS, 0xFF);
    lora_idle(lora);
}

void lora_send_packet_non_blocking(lora_t *lora, const uint8_t *data, uint8_t length) {
    lora_idle(lora);
    lora_write_register(lora, REG_IRQ_FLAGS, 0xFF); 
    lora_write_register(lora, REG_FIFO_ADDR_PTR, 0);

    for(uint8_t i=0; i<length; i++){
        lora_write_register(lora, REG_FIFO, data[i]);
    }

    lora_write_register(lora, REG_PAYLOAD_LENGTH, length);
    lora_write_register(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  //don t wait with a while loop
}

// Receive packet
uint8_t lora_receive_packet(uint8_t *buffer) {
    lora_t *lora = &lora_instance;

    lora_write_register(lora, REG_IRQ_FLAGS, 0xFF);
    lora_write_register(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    while((lora_read_register(lora, REG_IRQ_FLAGS) & 0x40) == 0);

    uint8_t fifo_addr = lora_read_register(lora, REG_FIFO_RX_CURRENT_ADDR);
    lora_write_register(lora, REG_FIFO_ADDR_PTR, fifo_addr);

    uint8_t length = lora_read_register(lora, REG_RX_NB_BYTES);
    if(length > MAX_PKT_LENGTH) length = MAX_PKT_LENGTH;

    for(uint8_t i=0;i<length;i++){
        buffer[i] = lora_read_register(lora, REG_FIFO);
    }

    lora_write_register(lora, REG_IRQ_FLAGS, 0xFF);
    return length;
}
