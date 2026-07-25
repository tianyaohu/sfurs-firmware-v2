#include "optical_flow.h"

#include <stdint.h>

#define PAA5100_WRITE_MASK        0x80U
#define PAA5100_MOTION_READY_MASK 0x80U

#define PAA5100_T_NCS_SCLK_US       1U
#define PAA5100_T_WRITE_HOLD_US     35U
#define PAA5100_T_WRITE_RECOVERY_US 50U
#define PAA5100_T_READ_ACCESS_US    35U
#define PAA5100_T_READ_RECOVERY_US  2U
#define PAA5100_T_BURST_ACCESS_US   35U
#define PAA5100_T_BURST_EXIT_US     2U

#define PAA5100_MIN_VALID_SQUAL   0x19U
#define PAA5100_BAD_SHUTTER_UPPER 0x1FU

#define ARRAY_COUNT(array) ((uint32_t)(sizeof(array) / sizeof((array)[0])))

/*
 * SPI OPTICAL FLOW REGISTERS
 */
#define PAA5100_REGISTER_PRODUCT_ID         0x00U
#define PAA5100_REGISTER_REVISION_ID        0x01U
#define PAA5100_REGISTER_MOTION             0x02U
#define PAA5100_REGISTER_DELTA_X_L          0x03U
#define PAA5100_REGISTER_DELTA_X_H          0x04U
#define PAA5100_REGISTER_DELTA_Y_L          0x05U
#define PAA5100_REGISTER_DELTA_Y_H          0x06U
#define PAA5100_REGISTER_MOTION_BURST       0x16U
#define PAA5100_REGISTER_POWER_UP_RESET     0x3AU
#define PAA5100_REGISTER_INVERSE_PRODUCT_ID 0x5FU
#define PAA5100_BURST_LENGTH                12U
#define PAA5100_PRODUCT_ID_VALUE            0x49U
#define PAA5100_INVERSE_PRODUCT_ID_VALUE    0xB6U
#define PAA5100_REVISION_ID_VALUE           0x00U

#define PAA5100_SPI_TIMEOUT_MS 10U

typedef struct {
    uint8_t reg;
    uint8_t value;
} PAA5100_REGISTER_WRITE_t;

typedef struct {
    uint8_t product_id;
    uint8_t revision_id;
    uint8_t inverse_product_id;
} PAA5100_IDENTIFIERS_t;

typedef struct {
    int16_t delta_x;
    int16_t delta_y;
    uint16_t shutter;
    uint8_t motion;
    uint8_t squal;
    uint8_t raw_sum;
    uint8_t motion_usable;
    uint8_t raw_burst[PAA5100_BURST_LENGTH];
} PAA5100_SAMPLE_t;

/*
 * Proprietary startup configuration based on the Pimoroni
 * PAA5100/PMW3901 reference implementation.
 */
static const PAA5100_REGISTER_WRITE_t startup_config_before_calibration[] = {
    {0x7FU, 0x00U},
    {0x55U, 0x01U},
    {0x50U, 0x07U},
    {0x7FU, 0x0EU},
    {0x43U, 0x10U},
};

static const PAA5100_REGISTER_WRITE_t calibration_setup[] = {
    {0x7FU, 0x00U},
    {0x51U, 0x7BU},
    {0x50U, 0x00U},
    {0x55U, 0x00U},
    {0x7FU, 0x0EU},
};

static const PAA5100_REGISTER_WRITE_t startup_config_after_calibration[] = {
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

static const PAA5100_REGISTER_WRITE_t startup_config_after_delay[] = {
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

static const PAA5100_REGISTER_WRITE_t startup_config_final[] = {
    {0x73U, 0x00U},
};


static uint8_t paa5100_dwt_ready = 0U;

/* Low-level timing / GPIO helpers */
inline static void paa5100_chip_select(GPIO_PinState pin_state)
{
    HAL_GPIO_WritePin(PAA5100_CHIP_SELECT_GPIO, PAA5100_CHIP_SELECT_PIN, pin_state);
}

static void paa5100_init_delay_timer(void)
{
    if (paa5100_dwt_ready != 0U) {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT       = 0U;
    paa5100_dwt_ready = 1U;
}

static void paa5100_delay_us(uint32_t delay_us)
{
    uint32_t start_tick;
    uint32_t wait_cycles;

    if (delay_us == 0U) {
        return;
    }

    paa5100_init_delay_timer();

    start_tick  = DWT->CYCCNT;
    wait_cycles = delay_us * (HAL_RCC_GetHCLKFreq() / 1000000U);

    while ((DWT->CYCCNT - start_tick) < wait_cycles) {
    }
}

inline static void paa5100_begin_transaction(void)
{
    paa5100_chip_select(GPIO_PIN_RESET);
    paa5100_delay_us(PAA5100_T_NCS_SCLK_US);
}

inline static void paa5100_end_transaction(uint32_t post_assert_delay_us,
                                           uint32_t recovery_delay_us)
{
    paa5100_delay_us(post_assert_delay_us);
    paa5100_chip_select(GPIO_PIN_SET);
    paa5100_delay_us(recovery_delay_us);
}

static HAL_StatusTypeDef paa5100_write_register(SPI_HandleTypeDef *spi_handle,
                                                uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;
    uint8_t spi_tx_data[2] = {(uint8_t)(reg | PAA5100_WRITE_MASK), value};

    paa5100_begin_transaction();
    status = HAL_SPI_Transmit(spi_handle, spi_tx_data, 2U, PAA5100_SPI_TIMEOUT_MS);
    paa5100_end_transaction(PAA5100_T_WRITE_HOLD_US, PAA5100_T_WRITE_RECOVERY_US);

    return status;
}

static HAL_StatusTypeDef paa5100_read_register(SPI_HandleTypeDef *spi_handle, uint8_t reg,
                                               uint8_t *value)
{
    HAL_StatusTypeDef status;
    uint8_t reg_address = reg;
    uint8_t tx_dummy    = 0U;
    uint8_t reg_value   = 0U;

    paa5100_begin_transaction();
    status = HAL_SPI_Transmit(spi_handle, &reg_address, 1U, PAA5100_SPI_TIMEOUT_MS);

    if (status == HAL_OK) {
        paa5100_delay_us(PAA5100_T_READ_ACCESS_US);
        status = HAL_SPI_TransmitReceive(spi_handle, &tx_dummy, &reg_value, 1U,
                                         PAA5100_SPI_TIMEOUT_MS);
    }

    paa5100_end_transaction(0U, PAA5100_T_READ_RECOVERY_US);

    if (status == HAL_OK) {
        *value = reg_value;
    }

    return status;
}

static HAL_StatusTypeDef paa5100_bulk_write(SPI_HandleTypeDef *spi_handle,
                                            const PAA5100_REGISTER_WRITE_t *writes,
                                            uint32_t count)
{
    HAL_StatusTypeDef status;
    uint32_t index;

    for (index = 0U; index < count; index++) {
        status
            = paa5100_write_register(spi_handle, writes[index].reg, writes[index].value);
        if (status != HAL_OK) {
            return status;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef paa5100_read_motion_burst_raw(SPI_HandleTypeDef *spi_handle,
                                                       uint8_t *spi_rx_data)
{
    HAL_StatusTypeDef status;
    uint8_t burst_reg                      = PAA5100_REGISTER_MOTION_BURST;
    uint8_t tx_dummy[PAA5100_BURST_LENGTH] = {0};

    paa5100_begin_transaction();
    status = HAL_SPI_Transmit(spi_handle, &burst_reg, 1U, PAA5100_SPI_TIMEOUT_MS);

    if (status == HAL_OK) {
        paa5100_delay_us(PAA5100_T_BURST_ACCESS_US);
        status = HAL_SPI_TransmitReceive(spi_handle, tx_dummy, spi_rx_data,
                                         PAA5100_BURST_LENGTH, PAA5100_SPI_TIMEOUT_MS);
    }

    paa5100_end_transaction(0U, PAA5100_T_BURST_EXIT_US);
    return status;
}

/* Identity/Validation Helpers */
static HAL_StatusTypeDef paa5100_read_identifiers(SPI_HandleTypeDef *spi_handle,
                                                  PAA5100_IDENTIFIERS_t *identifiers)
{
    HAL_StatusTypeDef status;
    status = paa5100_read_register(spi_handle, PAA5100_REGISTER_PRODUCT_ID,
                                   &identifiers->product_id);
    if (status != HAL_OK) {
        return status;
    }
    status = paa5100_read_register(spi_handle, PAA5100_REGISTER_REVISION_ID,
                                   &identifiers->revision_id);
    if (status != HAL_OK) {
        return status;
    }
    status = paa5100_read_register(spi_handle, PAA5100_REGISTER_INVERSE_PRODUCT_ID,
                                   &identifiers->inverse_product_id);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

static uint8_t
paa5100_identifiers_match_expected(const PAA5100_IDENTIFIERS_t *identifiers)
{
    if (identifiers->product_id != PAA5100_PRODUCT_ID_VALUE
        || identifiers->inverse_product_id != PAA5100_INVERSE_PRODUCT_ID_VALUE
        || identifiers->revision_id != PAA5100_REVISION_ID_VALUE) {
        return 0U;
    }

    return 1U;
}

/*
 * Requires register bank 0x0E to be selected before this function is called.
 * The calibration registers below are bank-dependent.
 */
static HAL_StatusTypeDef paa5100_apply_calibration(SPI_HandleTypeDef *spi_handle)
{
    HAL_StatusTypeDef status;
    uint8_t value = 0U;
    uint8_t c1    = 0U;
    uint8_t c2    = 0U;

    status = paa5100_read_register(spi_handle, 0x67U, &value);
    if (status != HAL_OK) {
        return status;
    }

    status = paa5100_write_register(spi_handle, 0x48U,
                                    (value & 0x80U) != 0U ? 0x04U : 0x02U);

    if (status != HAL_OK) {
        return status;
    }

    status = paa5100_bulk_write(spi_handle, calibration_setup,
                                ARRAY_COUNT(calibration_setup));

    if (status != HAL_OK) {
        return status;
    }

    status = paa5100_read_register(spi_handle, 0x73U, &value);
    if (status != HAL_OK) {
        return status;
    }

    if (value != 0x00U) {
        return HAL_OK;
    }

    status = paa5100_read_register(spi_handle, 0x70U, &c1);
    if (status != HAL_OK) {
        return status;
    }

    status = paa5100_read_register(spi_handle, 0x71U, &c2);
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

    const PAA5100_REGISTER_WRITE_t calibration_writes[] = {
        {0x7FU, 0x00U},
        {0x61U, 0xADU},
        {0x51U, 0x70U},
        {0x7FU, 0x0EU},
        {0x70U, c1   },
        {0x71U, c2   },
    };

    return paa5100_bulk_write(spi_handle, calibration_writes,
                              ARRAY_COUNT(calibration_writes));
}

static HAL_StatusTypeDef paa5100_apply_config(SPI_HandleTypeDef *spi_handle)
{
    HAL_StatusTypeDef status;

    status = paa5100_bulk_write(spi_handle, startup_config_before_calibration,
                                ARRAY_COUNT(startup_config_before_calibration));

    if (status != HAL_OK) {
        return status;
    }

    status = paa5100_apply_calibration(spi_handle);

    if (status != HAL_OK) {
        return status;
    }

    status = paa5100_bulk_write(spi_handle, startup_config_after_calibration,
                                ARRAY_COUNT(startup_config_after_calibration));

    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(10U);

    status = paa5100_bulk_write(spi_handle, startup_config_after_delay,
                                ARRAY_COUNT(startup_config_after_delay));

    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(10U);

    return paa5100_bulk_write(spi_handle, startup_config_final,
                              ARRAY_COUNT(startup_config_final));
}

/*
 * Perform the post-reset motion-register reads used by the
 * reference implementation before applying configuration.
 */
static HAL_StatusTypeDef paa5100_perform_startup_reads(SPI_HandleTypeDef *spi_handle)
{
    HAL_StatusTypeDef status;
    uint8_t value = 0U;

    status = paa5100_read_register(spi_handle, PAA5100_REGISTER_MOTION, &value);

    if (status != HAL_OK) {
        return status;
    }

    status = paa5100_read_register(spi_handle, PAA5100_REGISTER_DELTA_X_L, &value);

    if (status != HAL_OK) {
        return status;
    }

    status = paa5100_read_register(spi_handle, PAA5100_REGISTER_DELTA_X_H, &value);

    if (status != HAL_OK) {
        return status;
    }

    status = paa5100_read_register(spi_handle, PAA5100_REGISTER_DELTA_Y_L, &value);

    if (status != HAL_OK) {
        return status;
    }

    status = paa5100_read_register(spi_handle, PAA5100_REGISTER_DELTA_Y_H, &value);

    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef paa5100_read_sample(SPI_HandleTypeDef *spi_handle,
                                             PAA5100_SAMPLE_t *sample)
{
    HAL_StatusTypeDef status;
    uint8_t *burst;

    if (sample == NULL) {
        return HAL_ERROR;
    }

    burst = sample->raw_burst;

    status = paa5100_read_motion_burst_raw(spi_handle, burst);
    if (status != HAL_OK) {
        return status;
    }

    sample->motion        = burst[0];
    sample->delta_x       = (int16_t)((((uint16_t)burst[3]) << 8) | burst[2]);
    sample->delta_y       = (int16_t)((((uint16_t)burst[5]) << 8) | burst[4]);
    sample->squal         = burst[6];
    sample->raw_sum       = burst[7];
    sample->shutter       = (uint16_t)((((uint16_t)burst[10]) << 8) | burst[11]);
    sample->motion_usable = ((burst[0] & PAA5100_MOTION_READY_MASK) != 0U) ? 1U : 0U;

    if (sample->motion_usable == 0U) {
        return HAL_OK;
    }

    if (sample->squal < PAA5100_MIN_VALID_SQUAL
        && burst[10] == PAA5100_BAD_SHUTTER_UPPER) {
        sample->motion_usable = 0U;
    }

    return HAL_OK;
}


/* Public APIs */
HAL_StatusTypeDef paa5100_init(SPI_HandleTypeDef *spi_handle)
{
    HAL_StatusTypeDef status;
    PAA5100_IDENTIFIERS_t identifiers;

    if (spi_handle == NULL) {
        return HAL_ERROR;
    }

    paa5100_init_delay_timer();
    paa5100_chip_select(GPIO_PIN_SET);
    HAL_Delay(50U);

    status = paa5100_write_register(spi_handle, PAA5100_REGISTER_POWER_UP_RESET, 0x5AU);

    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(20U);

    status = paa5100_perform_startup_reads(spi_handle);

    if (status != HAL_OK) {
        return status;
    }

    status = paa5100_apply_config(spi_handle);

    if (status != HAL_OK) {
        return status;
    }

    status = paa5100_read_identifiers(spi_handle, &identifiers);

    if (status != HAL_OK) {
        return status;
    }

    if (paa5100_identifiers_match_expected(&identifiers) == 0U) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef paa5100_get_delta(SPI_HandleTypeDef *spi_handle, PAA5100_DELTA_t *delta)
{
    HAL_StatusTypeDef status;
    PAA5100_SAMPLE_t sample;

    if (delta == NULL || spi_handle == NULL) {
        return HAL_ERROR;
    }
    delta->delta_x = 0;
    delta->delta_y = 0;

    status = paa5100_read_sample(spi_handle, &sample);
    if (status != HAL_OK) {
        return status;
    }

    /* No usable motion measurement is available. */
    if (sample.motion_usable == 0U) {
        return HAL_OK;
    }

    delta->delta_x = (int32_t)sample.delta_x;
    delta->delta_y = (int32_t)sample.delta_y;
    return HAL_OK;
}
