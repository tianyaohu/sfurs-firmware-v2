#include "optical_flow.h"

#include <stdint.h>
#include <string.h>

#define SPI_OPTICAL_FLOW_WRITE_MASK        0x80U
#define SPI_OPTICAL_FLOW_MOTION_VALID_MASK 0x80U

#define SPI_OPTICAL_FLOW_T_NCS_SCLK_US       1U
#define SPI_OPTICAL_FLOW_T_WRITE_HOLD_US     35U
#define SPI_OPTICAL_FLOW_T_WRITE_RECOVERY_US 50U
#define SPI_OPTICAL_FLOW_T_READ_ACCESS_US    35U
#define SPI_OPTICAL_FLOW_T_READ_RECOVERY_US  2U
#define SPI_OPTICAL_FLOW_T_BURST_ACCESS_US   35U
#define SPI_OPTICAL_FLOW_T_BURST_EXIT_US     2U

#define SPI_OPTICAL_FLOW_MIN_VALID_SQUAL   0x19U
#define SPI_OPTICAL_FLOW_BAD_SHUTTER_UPPER 0x1FU

/* Returns the number of elements in a fixed-size array. */
#define ARRAY_COUNT(array) ((uint32_t)(sizeof(array) / sizeof((array)[0])))

/*
 * SPI OPTICAL FLOW REGISTERS
 */
#define SPI_OPTICAL_FLOW_REGISTER_PRODUCT_ID         0x00U
#define SPI_OPTICAL_FLOW_REGISTER_REVISION_ID        0x01U
#define SPI_OPTICAL_FLOW_REGISTER_MOTION             0x02U
#define SPI_OPTICAL_FLOW_REGISTER_DELTA_X_L          0x03U
#define SPI_OPTICAL_FLOW_REGISTER_DELTA_X_H          0x04U
#define SPI_OPTICAL_FLOW_REGISTER_DELTA_Y_L          0x05U
#define SPI_OPTICAL_FLOW_REGISTER_DELTA_Y_H          0x06U
#define SPI_OPTICAL_FLOW_REGISTER_MOTION_BURST       0x16U
#define SPI_OPTICAL_FLOW_REGISTER_POWER_UP_RESET     0x3AU
#define SPI_OPTICAL_FLOW_REGISTER_INVERSE_PRODUCT_ID 0x5FU
#define SPI_OPTICAL_FLOW_BURST_LENGTH                12U
#define PAA5100_PRODUCT_ID_VALUE                     0x49U
#define PAA5100_INVERSE_PRODUCT_ID_VALUE             0xB6U

#define SPI_OPTICAL_FLOW_SPI_TIMEOUT_MS 10U


typedef struct {
    uint8_t reg;
    uint8_t value;
} SPI_OPTICAL_FLOW_REGISTER_WRITE_t;

typedef struct {
    uint8_t product_id;
    uint8_t revision_id;
    uint8_t inverse_product_id;
} SPI_OPTICAL_FLOW_IDENTIFIERS_t;

typedef struct {
    int16_t delta_x;
    int16_t delta_y;
    uint16_t shutter;
    uint8_t motion;
    uint8_t squal;
    uint8_t raw_sum;
    uint8_t valid;
    uint8_t raw_burst[SPI_OPTICAL_FLOW_BURST_LENGTH];
} SPI_OPTICAL_FLOW_SAMPLE_t;

static const SPI_OPTICAL_FLOW_REGISTER_WRITE_t startup_config_initial[] = {
    {0x7FU, 0x00U},
    {0x55U, 0x01U},
    {0x50U, 0x07U},
    {0x7FU, 0x0EU},
    {0x43U, 0x10U},
    {0x7FU, 0x00U},
    {0x51U, 0x7BU},
    {0x50U, 0x00U},
    {0x55U, 0x00U},
    {0x7FU, 0x0EU},
    {0x7FU, 0x00U},
    {0x61U, 0xADU},
    {0x7FU, 0x03U},
    {0x40U, 0x00U},
    {0x7FU, 0x05U},
    {0x41U, 0xB3U},
    {0x43U, 0xF1U},
    {0x45U, 0x14U},
    {0x5FU, 0x34U},
    {0x7BU, 0x08U},
    {0x5EU, 0x34U},
    {0x5BU, 0x11U},
    {0x6DU, 0x11U},
    {0x45U, 0x17U},
    {0x70U, 0xE5U},
    {0x71U, 0xE5U},
    {0x7FU, 0x06U},
    {0x44U, 0x1BU},
    {0x40U, 0xBFU},
    {0x4EU, 0x3FU},
    {0x7FU, 0x08U},
    {0x66U, 0x44U},
    {0x65U, 0x20U},
    {0x6AU, 0x3AU},
    {0x61U, 0x05U},
    {0x62U, 0x05U},
    {0x7FU, 0x09U},
    {0x4FU, 0xAFU},
    {0x5FU, 0x40U},
    {0x48U, 0x80U},
    {0x49U, 0x80U},
    {0x57U, 0x77U},
    {0x60U, 0x78U},
    {0x61U, 0x78U},
    {0x62U, 0x08U},
    {0x63U, 0x50U},
    {0x7FU, 0x0AU},
    {0x45U, 0x60U},
    {0x7FU, 0x00U},
    {0x4DU, 0x11U},
    {0x55U, 0x80U},
    {0x74U, 0x21U},
    {0x75U, 0x1FU},
    {0x4AU, 0x78U},
    {0x4BU, 0x78U},
    {0x44U, 0x08U},
    {0x45U, 0x50U},
    {0x64U, 0xFFU},
    {0x65U, 0x1FU},
    {0x7FU, 0x14U},
    {0x65U, 0x67U},
    {0x66U, 0x08U},
    {0x63U, 0x70U},
    {0x6FU, 0x1CU},
    {0x7FU, 0x15U},
    {0x48U, 0x48U},
    {0x7FU, 0x07U},
    {0x41U, 0x0DU},
    {0x43U, 0x14U},
    {0x4BU, 0x0EU},
    {0x45U, 0x0FU},
    {0x44U, 0x42U},
    {0x4CU, 0x80U},
    {0x7FU, 0x10U},
    {0x5BU, 0x02U},
    {0x7FU, 0x07U},
    {0x40U, 0x41U},
};

static const SPI_OPTICAL_FLOW_REGISTER_WRITE_t startup_config_after_delay[] = {
    {0x7FU, 0x00U},
    {0x32U, 0x00U},
    {0x7FU, 0x07U},
    {0x40U, 0x40U},
    {0x7FU, 0x06U},
    {0x68U, 0xF0U},
    {0x69U, 0x00U},
    {0x7FU, 0x0DU},
    {0x48U, 0xC0U},
    {0x6FU, 0xD5U},
    {0x7FU, 0x00U},
    {0x5BU, 0xA0U},
    {0x4EU, 0xA8U},
    {0x5AU, 0x90U},
    {0x40U, 0x80U},
    {0x73U, 0x1FU},
};

static const SPI_OPTICAL_FLOW_REGISTER_WRITE_t startup_config_final[] = {
    {0x73U, 0x00U},
};


static uint8_t spi_optical_flow_dwt_ready = 0U;

/* Low-level timing / GPIO helpers */
inline static void spi_optical_flow_chip_select(GPIO_PinState pin_state)
{
    HAL_GPIO_WritePin(SPI_OPTICAL_FLOW_CHIP_SELECT_GPIO, SPI_OPTICAL_FLOW_CHIP_SELECT_PIN,
                      pin_state);
}

static void spi_optical_flow_init_delay_timer(void)
{
    if (spi_optical_flow_dwt_ready != 0U) {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT                = 0U;
    spi_optical_flow_dwt_ready = 1U;
}

static void spi_optical_flow_delay_us(uint32_t delay_us)
{
    uint32_t start_tick;
    uint32_t wait_cycles;

    if (delay_us == 0U) {
        return;
    }

    spi_optical_flow_init_delay_timer();

    start_tick  = DWT->CYCCNT;
    wait_cycles = delay_us * (HAL_RCC_GetHCLKFreq() / 1000000U);

    while ((DWT->CYCCNT - start_tick) < wait_cycles) {
    }
}

inline static void spi_optical_flow_begin_transaction(void)
{
    spi_optical_flow_chip_select(GPIO_PIN_RESET);
    spi_optical_flow_delay_us(SPI_OPTICAL_FLOW_T_NCS_SCLK_US);
}

inline static void spi_optical_flow_end_transaction(uint32_t post_assert_delay_us,
                                                    uint32_t recovery_delay_us)
{
    spi_optical_flow_delay_us(post_assert_delay_us);
    spi_optical_flow_chip_select(GPIO_PIN_SET);
    spi_optical_flow_delay_us(recovery_delay_us);
}

static HAL_StatusTypeDef spi_optical_flow_write_register(SPI_HandleTypeDef *spi_pointer,
                                                         uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;
    uint8_t spi_tx_data[2] = {(uint8_t)(reg | SPI_OPTICAL_FLOW_WRITE_MASK), value};

    spi_optical_flow_begin_transaction();
    status
        = HAL_SPI_Transmit(spi_pointer, spi_tx_data, 2U, SPI_OPTICAL_FLOW_SPI_TIMEOUT_MS);
    spi_optical_flow_end_transaction(SPI_OPTICAL_FLOW_T_WRITE_HOLD_US,
                                     SPI_OPTICAL_FLOW_T_WRITE_RECOVERY_US);

    return status;
}

static HAL_StatusTypeDef spi_optical_flow_read_register(SPI_HandleTypeDef *spi_pointer,
                                                        uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;
    uint8_t reg_address = reg;
    uint8_t tx_dummy    = 0U;
    uint8_t reg_value   = 0U;

    if (value == NULL) {
        return HAL_ERROR;
    }

    spi_optical_flow_begin_transaction();
    status = HAL_SPI_Transmit(spi_pointer, &reg_address, 1U,
                              SPI_OPTICAL_FLOW_SPI_TIMEOUT_MS);

    if (status == HAL_OK) {
        spi_optical_flow_delay_us(SPI_OPTICAL_FLOW_T_READ_ACCESS_US);
        status = HAL_SPI_TransmitReceive(spi_pointer, &tx_dummy, &reg_value, 1U,
                                         SPI_OPTICAL_FLOW_SPI_TIMEOUT_MS);
    }

    spi_optical_flow_end_transaction(0U, SPI_OPTICAL_FLOW_T_READ_RECOVERY_US);

    if (status == HAL_OK) {
        *value = reg_value;
    }

    return status;
}

static HAL_StatusTypeDef
spi_optical_flow_bulk_write(SPI_HandleTypeDef *spi_pointer,
                            const SPI_OPTICAL_FLOW_REGISTER_WRITE_t *writes,
                            uint32_t count)
{
    HAL_StatusTypeDef status;
    uint32_t index;

    for (index = 0U; index < count; index++) {
        status = spi_optical_flow_write_register(spi_pointer, writes[index].reg,
                                                 writes[index].value);
        if (status != HAL_OK) {
            return status;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef
spi_optical_flow_read_motion_burst_raw(SPI_HandleTypeDef *spi_pointer,
                                       uint8_t *spi_rx_data)
{
    HAL_StatusTypeDef status;
    uint8_t burst_reg = SPI_OPTICAL_FLOW_REGISTER_MOTION_BURST;
    uint8_t tx_dummy[SPI_OPTICAL_FLOW_BURST_LENGTH] = {0};

    spi_optical_flow_begin_transaction();
    status
        = HAL_SPI_Transmit(spi_pointer, &burst_reg, 1U, SPI_OPTICAL_FLOW_SPI_TIMEOUT_MS);

    if (status == HAL_OK) {
        spi_optical_flow_delay_us(SPI_OPTICAL_FLOW_T_BURST_ACCESS_US);
        status = HAL_SPI_TransmitReceive(spi_pointer, tx_dummy, spi_rx_data,
                                         SPI_OPTICAL_FLOW_BURST_LENGTH,
                                         SPI_OPTICAL_FLOW_SPI_TIMEOUT_MS);
    }

    spi_optical_flow_end_transaction(0U, SPI_OPTICAL_FLOW_T_BURST_EXIT_US);
    return status;
}

/* Identity/Validation Helpers */
static HAL_StatusTypeDef
spi_optical_flow_read_identifiers(SPI_HandleTypeDef *spi_pointer,
                                  SPI_OPTICAL_FLOW_IDENTIFIERS_t *identifiers)
{
    HAL_StatusTypeDef status;

    if (spi_pointer == NULL || identifiers == NULL) {
        return HAL_ERROR;
    }

    status = spi_optical_flow_read_register(
        spi_pointer, SPI_OPTICAL_FLOW_REGISTER_PRODUCT_ID, &identifiers->product_id);
    if (status != HAL_OK) {
        return status;
    }
    status = spi_optical_flow_read_register(
        spi_pointer, SPI_OPTICAL_FLOW_REGISTER_REVISION_ID, &identifiers->revision_id);
    if (status != HAL_OK) {
        return status;
    }
    status = spi_optical_flow_read_register(spi_pointer,
                                            SPI_OPTICAL_FLOW_REGISTER_INVERSE_PRODUCT_ID,
                                            &identifiers->inverse_product_id);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

static uint8_t spi_optical_flow_identifiers_match_expected(
    const SPI_OPTICAL_FLOW_IDENTIFIERS_t *identifiers)
{
    if (identifiers == NULL) {
        return 0U;
    }

    if (identifiers->product_id != PAA5100_PRODUCT_ID_VALUE
        || identifiers->inverse_product_id != PAA5100_INVERSE_PRODUCT_ID_VALUE) {
        return 0U;
    }

    return 1U;
}

/* Startup Helpers */

static HAL_StatusTypeDef
spi_optical_flow_apply_calibration(SPI_HandleTypeDef *spi_pointer)
{
    HAL_StatusTypeDef status;
    uint8_t value = 0U;
    uint8_t c1    = 0U;
    uint8_t c2    = 0U;

    status = spi_optical_flow_read_register(spi_pointer, 0x67U, &value);
    if (status != HAL_OK) {
        return status;
    }

    status = spi_optical_flow_write_register(spi_pointer, 0x48U,
                                             (value & 0x80U) != 0U ? 0x04U : 0x02U);

    if (status != HAL_OK) {
        return status;
    }

    status = spi_optical_flow_read_register(spi_pointer, 0x73U, &value);
    if (status != HAL_OK) {
        return status;
    }

    if (value != 0x00U) {
        return HAL_OK;
    }

    status = spi_optical_flow_read_register(spi_pointer, 0x70U, &c1);
    if (status != HAL_OK) {
        return status;
    }

    status = spi_optical_flow_read_register(spi_pointer, 0x71U, &c2);
    if (status != HAL_OK) {
        return status;
    }

    if (c1 <= 28U) {
        c1 = (uint8_t)(c1 + 14U);
    } else {
        c1 = (uint8_t)(c1 + 11U);
    }

    if (c1 > 0x3FU) {
        c1 = 0x3FU;
    }

    c2 = (uint8_t)((((uint32_t)c2) * 45U) / 100U);

    const SPI_OPTICAL_FLOW_REGISTER_WRITE_t calibration_writes[] = {
        {0x7FU, 0x00U},
        {0x61U, 0xADU},
        {0x51U, 0x70U},
        {0x7FU, 0x0EU},
        {0x70U, c1   },
        {0x71U, c2   },
    };

    return spi_optical_flow_bulk_write(spi_pointer, calibration_writes,
                                       ARRAY_COUNT(calibration_writes));
}

static HAL_StatusTypeDef spi_optical_flow_apply_config(SPI_HandleTypeDef *spi_pointer)
{
    HAL_StatusTypeDef status;

    status = spi_optical_flow_bulk_write(spi_pointer, startup_config_initial,
                                         ARRAY_COUNT(startup_config_initial));

    if (status != HAL_OK) {
        return status;
    }

    status = spi_optical_flow_apply_calibration(spi_pointer);

    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(10U);

    status = spi_optical_flow_bulk_write(spi_pointer, startup_config_after_delay,
                                         ARRAY_COUNT(startup_config_after_delay));

    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(10U);

    return spi_optical_flow_bulk_write(spi_pointer, startup_config_final,
                                       ARRAY_COUNT(startup_config_final));
}

static HAL_StatusTypeDef spi_optical_flow_read_motion(SPI_HandleTypeDef *spi_pointer)
{
    HAL_StatusTypeDef status;
    uint8_t value = 0U;

    status = spi_optical_flow_read_register(spi_pointer, SPI_OPTICAL_FLOW_REGISTER_MOTION,
                                            &value);

    if (status != HAL_OK) {
        return status;
    }

    status = spi_optical_flow_read_register(spi_pointer,
                                            SPI_OPTICAL_FLOW_REGISTER_DELTA_X_L, &value);

    if (status != HAL_OK) {
        return status;
    }

    status = spi_optical_flow_read_register(spi_pointer,
                                            SPI_OPTICAL_FLOW_REGISTER_DELTA_X_H, &value);

    if (status != HAL_OK) {
        return status;
    }

    status = spi_optical_flow_read_register(spi_pointer,
                                            SPI_OPTICAL_FLOW_REGISTER_DELTA_Y_L, &value);

    if (status != HAL_OK) {
        return status;
    }

    status = spi_optical_flow_read_register(spi_pointer,
                                            SPI_OPTICAL_FLOW_REGISTER_DELTA_Y_H, &value);

    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

/* Data read helper */
static HAL_StatusTypeDef spi_optical_flow_read_sample(SPI_HandleTypeDef *spi_pointer,
                                                      SPI_OPTICAL_FLOW_SAMPLE_t *sample)
{
    HAL_StatusTypeDef status;
    uint8_t *burst;

    if (sample == NULL) {
        return HAL_ERROR;
    }

    memset(sample, 0, sizeof(*sample));
    burst = sample->raw_burst;

    status = spi_optical_flow_read_motion_burst_raw(spi_pointer, burst);
    if (status != HAL_OK) {
        return status;
    }

    sample->motion  = burst[0];
    sample->delta_x = (int16_t)((((uint16_t)burst[3]) << 8) | burst[2]);
    sample->delta_y = (int16_t)((((uint16_t)burst[5]) << 8) | burst[4]);
    sample->squal   = burst[6];
    sample->raw_sum = burst[7];
    sample->shutter = (uint16_t)((((uint16_t)burst[10]) << 8) | burst[11]);
    sample->valid   = ((burst[0] & SPI_OPTICAL_FLOW_MOTION_VALID_MASK) != 0U) ? 1U : 0U;

    if (sample->valid != 0U
        && !(sample->squal < SPI_OPTICAL_FLOW_MIN_VALID_SQUAL
             && burst[10] == SPI_OPTICAL_FLOW_BAD_SHUTTER_UPPER)) {
        return HAL_OK;
    }

    sample->valid = 0U;
    return HAL_OK;
}


/* Public APIs */
HAL_StatusTypeDef spi_optical_flow_init(SPI_HandleTypeDef *spi_pointer)
{
    HAL_StatusTypeDef status;
    SPI_OPTICAL_FLOW_IDENTIFIERS_t identifiers;

    if (spi_pointer == NULL) {
        return HAL_ERROR;
    }

    memset(&identifiers, 0, sizeof(identifiers));
    spi_optical_flow_init_delay_timer();
    spi_optical_flow_chip_select(GPIO_PIN_SET);
    HAL_Delay(50U);

    status = spi_optical_flow_write_register(
        spi_pointer, SPI_OPTICAL_FLOW_REGISTER_POWER_UP_RESET, 0x5AU);

    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(20U);

    status = spi_optical_flow_read_motion(spi_pointer);

    if (status != HAL_OK) {
        return status;
    }

    status = spi_optical_flow_apply_config(spi_pointer);

    if (status != HAL_OK) {
        return status;
    }

    status = spi_optical_flow_read_identifiers(spi_pointer, &identifiers);

    if (status != HAL_OK) {
        return status;
    }

    if (spi_optical_flow_identifiers_match_expected(&identifiers) == 0U) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef spi_optical_flow_get_delta(SPI_HandleTypeDef *spi_pointer,
                                             SPI_OPTICAL_FLOW_DELTA_DATA_t *delta_data)
{
    HAL_StatusTypeDef status;
    SPI_OPTICAL_FLOW_SAMPLE_t sample;

    if (delta_data == NULL || spi_pointer == NULL) {
        return HAL_ERROR;
    }
    delta_data->delta_x = 0;
    delta_data->delta_y = 0;

    /* SPI communication failure. */
    status = spi_optical_flow_read_sample(spi_pointer, &sample);
    if (status != HAL_OK) {
        return status;
    }

    /* No motion or unusable sample. */
    if (sample.valid == 0U) {
        return HAL_OK;
    }

    delta_data->delta_x = (int32_t)sample.delta_x;
    delta_data->delta_y = (int32_t)sample.delta_y;
    return HAL_OK;
}
