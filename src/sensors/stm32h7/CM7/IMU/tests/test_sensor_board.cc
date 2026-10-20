#include <stdint.h>
#include <gtest/gtest.h>
#include "hal_mock.h"

extern "C" {
    #include "sensor_board.h"
}

using ::testing::_;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Pointee;
using ::testing::Return;
using ::testing::SetArgPointee;
using ::testing::WithArg;

namespace {

constexpr uint16_t CsPin      = 0x0010;
constexpr uint8_t  WhoAmILsm6 = 0x6C;

class SensorBoardTest : public ::testing::Test {
protected:
    void SetUp() override {
        g_hal = &hal;

        // Reset the module state before every test.
        sensor_board_init(nullptr);

        board = {};
        board.spi            = &fake_spi;
        board.imu_cs_port    = &fake_port;
        board.imu_cs_pin     = CsPin;
        board.spi_timeout_ms = 25;
    }

    void TearDown() override {
        g_hal = nullptr;
    }

    void InitAsLsm6() {
        EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
            .WillOnce(DoAll(SetArgPointee<2>(WhoAmILsm6), Return(HAL_OK)))
            .WillRepeatedly(Return(HAL_OK));

        sensor_board_init(&board);
        ASSERT_EQ(sensor_board_imu, IMU_LSM6);

        ::testing::Mock::VerifyAndClearExpectations(&hal);
    }

    NiceMock<MockHal> hal;
    sensor_board_t board;
    int fake_spi = 0;
    int fake_port = 0;
};


TEST_F(SensorBoardTest, InitWithNullKeepsModuleSafe) {
    sensor_board_imu = IMU_LSM6;

    EXPECT_CALL(hal, HAL_GPIO_WritePin(_, _, _)).Times(0);
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, _, _)).Times(0);
    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, _, _)).Times(0);

    sensor_board_init(nullptr);

    EXPECT_EQ(sensor_board_imu, IMU_UNSUPPORTED);
}

// Test initialization with the correct WHO_AM_I value.
TEST_F(SensorBoardTest, ProbeAcceptsLsm6) {
    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
        .WillOnce(DoAll(SetArgPointee<2>(WhoAmILsm6), Return(HAL_OK)))
        .WillRepeatedly(Return(HAL_OK));

    sensor_board_init(&board);

    EXPECT_EQ(sensor_board_imu, IMU_LSM6);
}

// Test initialization with an incorrect WHO_AM_I value.
TEST_F(SensorBoardTest, ProbeRejectsWrongWhoAmIAndSkipsConfiguration) {
    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
        .WillOnce(DoAll(SetArgPointee<2>(0x6A), Return(HAL_OK)));

    // Reads send a one-byte register address. Configuration writes use two
    // bytes, so no two-byte write should happen for an unsupported device.
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, 1, _))
        .WillRepeatedly(Return(HAL_OK));
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, 2, _)).Times(0);

    sensor_board_init(&board);

    EXPECT_EQ(sensor_board_imu, IMU_UNSUPPORTED);
}

// Verify that SPI register reads set the read bit (bit 7).
TEST_F(SensorBoardTest, ReadAddressHasHighBitSet) {
    // WHO_AM_I is register 0x0F, therefore a read must send 0x8F.
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, _, _))
        .WillRepeatedly(Return(HAL_OK));
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, Pointee(0x8F), 1, _))
        .WillOnce(Return(HAL_OK));
    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
        .WillOnce(DoAll(SetArgPointee<2>(WhoAmILsm6), Return(HAL_OK)))
        .WillRepeatedly(Return(HAL_OK));

    sensor_board_init(&board);
}


TEST_F(SensorBoardTest, XlGetXReadsLowThenHighAndCombinesValue) {
    InitAsLsm6();

    const uint8_t low_address =
        static_cast<uint8_t>(LSM6_REG_OUTX_L_A | 0x80U);
    const uint8_t high_address =
        static_cast<uint8_t>((LSM6_REG_OUTX_L_A + 1U) | 0x80U);

    {
        InSequence seq;

        EXPECT_CALL(hal, HAL_GPIO_WritePin(_, CsPin, GPIO_PIN_RESET));
        EXPECT_CALL(hal, HAL_SPI_Transmit(_, Pointee(low_address), 1, _))
            .WillOnce(Return(HAL_OK));
        EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
            .WillOnce(DoAll(SetArgPointee<2>(0x34), Return(HAL_OK)));
        EXPECT_CALL(hal, HAL_GPIO_WritePin(_, CsPin, GPIO_PIN_SET));

        EXPECT_CALL(hal, HAL_GPIO_WritePin(_, CsPin, GPIO_PIN_RESET));
        EXPECT_CALL(hal, HAL_SPI_Transmit(_, Pointee(high_address), 1, _))
            .WillOnce(Return(HAL_OK));
        EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
            .WillOnce(DoAll(SetArgPointee<2>(0x12), Return(HAL_OK)));
        EXPECT_CALL(hal, HAL_GPIO_WritePin(_, CsPin, GPIO_PIN_SET));
    }

    EXPECT_EQ(xl_get_x(), static_cast<int16_t>(0x1234));
}


TEST_F(SensorBoardTest, PairAssemblesSignedLittleEndian) {
    InitAsLsm6();

    // (high << 8) | low = 0x8000, which is -32768 as int16_t.
    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
        .WillOnce(DoAll(SetArgPointee<2>(0x00), Return(HAL_OK)))
        .WillOnce(DoAll(SetArgPointee<2>(0x80), Return(HAL_OK)));

    EXPECT_EQ(xl_get_x(), -32768);
}

// Test full-scale configuration bits written to CTRL1_XL.
TEST_F(SensorBoardTest, SetFullScaleWritesCorrectBits) {
    InitAsLsm6();

    uint8_t sent[2] = {0, 0};

    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
        .WillOnce(DoAll(SetArgPointee<2>(0x00), Return(HAL_OK)));
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, 1, _))
        .WillRepeatedly(Return(HAL_OK));
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, 2, _))
        .WillOnce(DoAll(
            WithArg<1>(Invoke([&](uint8_t *p) {
                sent[0] = p[0];
                sent[1] = p[1];
            })),
            Return(HAL_OK)));

    xl_set_fs(LSM6_XL_FS_8G);

    EXPECT_EQ(sent[0], LSM6_REG_CTRL1_XL);
    EXPECT_EQ(sent[1], 0x0C);
}

// Ported from the old SPI-failure test, using GoogleMock to verify the exact
// CS behavior rather than only counting GPIO calls.
TEST_F(SensorBoardTest, ChipSelectReleasedWhenSpiFails) {
    InitAsLsm6();

    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, _, _)).Times(0);

    {
        InSequence seq;
        EXPECT_CALL(hal, HAL_GPIO_WritePin(_, CsPin, GPIO_PIN_RESET));
        EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, 1, _))
            .WillOnce(Return(HAL_ERROR));
        EXPECT_CALL(hal, HAL_GPIO_WritePin(_, CsPin, GPIO_PIN_SET));
    }

    EXPECT_EQ(xl_get_x(), 0);
}

// Ported from the old test suite: sensor reads must return zero without
// touching the HAL when the IMU has not been initialized/detected.
TEST_F(SensorBoardTest, GyrGetZReturnsZeroWhenNotInitialized) {
    sensor_board_imu = IMU_UNSUPPORTED;

    EXPECT_CALL(hal, HAL_GPIO_WritePin(_, _, _)).Times(0);
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, _, _)).Times(0);
    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, _, _)).Times(0);

    EXPECT_EQ(gyr_get_z(), 0);
}

// A configured timeout of zero should fall back to the module default of 10 ms.
TEST_F(SensorBoardTest, TimeoutBackToDefault) {
    board.spi_timeout_ms = 0;

    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, _, 10))
        .WillRepeatedly(Return(HAL_OK));
    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, _, 10))
        .WillOnce(DoAll(SetArgPointee<2>(WhoAmILsm6), Return(HAL_OK)))
        .WillRepeatedly(Return(HAL_OK));

    sensor_board_init(&board);
}

}  // namespace