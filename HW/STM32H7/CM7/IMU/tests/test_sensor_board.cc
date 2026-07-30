#include <stdint.h>
#include <gtest/gtest.h>
#include "hal_mock.h"

extern "C" {
    #include "sensor_board.h"
}
// Define names inside namespace
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
                sensor_board_init(nullptr);

                board = {};
                board.spi               = &fake_spi;
                board.imu_cs_port       = &fake_port;
                board.imu_cs_pin        = CsPin;
                board.spi_timeout_ms    = 25;
            }

            void TearDown() override {g_hal = nullptr;}


            // Bring up module as detected LSM6
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

    // Test the init with correct WHO_AM_I value
    TEST_F(SensorBoardTest, ProbeAcceptsLsm6) {
        EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
            .WillOnce(DoAll(SetArgPointee<2>(WhoAmILsm6), Return(HAL_OK)))
            .WillRepeatedly(Return(HAL_OK));
        
        sensor_board_init(&board);

        EXPECT_EQ(sensor_board_imu, IMU_LSM6);
    }

    // Test the init with the incorrect WHO_AM_I value
    TEST_F(SensorBoardTest, ProbeRejectsWrongWhoAmIAndSkipsConfiguration) {
    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
        .WillOnce(DoAll(SetArgPointee<2>(0x6A), Return(HAL_OK)));
    
    // Reads send a 1-byte address - allow those, then assert no 2-byte writes.
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, 1, _)).WillRepeatedly(Return(HAL_OK));
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, 2, _)).Times(0);

    sensor_board_init(&board);

    EXPECT_EQ(sensor_board_imu, IMU_UNSUPPORTED);
}

// Test the R/W bit
TEST_F(SensorBoardTest, ReadAddressHasHighBitSet) {
    // WHO_AM_I is 0x0F; a read must put 0x8F on the wire.
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, _, _))
        .WillRepeatedly(Return(HAL_OK));
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, Pointee(0x8F), 1, _))
        .WillOnce(Return(HAL_OK));
    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
        .WillOnce(DoAll(SetArgPointee<2>(WhoAmILsm6), Return(HAL_OK)))
        .WillRepeatedly(Return(HAL_OK));

    sensor_board_init(&board);
}

// Receive and assembly of a 16-bit value from two 1-byte reads
TEST_F(SensorBoardTest, PairAssemblesSignedLittleEndian) {
    InitAsLsm6();

    // assemble as (high << 8) | low = 0x8000, which as int16_t is -32768. */
    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
        .WillOnce(DoAll(SetArgPointee<2>(0x00), Return(HAL_OK)))
        .WillOnce(DoAll(SetArgPointee<2>(0x80), Return(HAL_OK)));

    EXPECT_EQ(xl_get_x(), -32768);
}

// Transmission Test
TEST_F(SensorBoardTest, SetFullScaleWritesCorrectBits) {
    InitAsLsm6();

    uint8_t sent[2] = {0, 0};
    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, 1, _))
        .WillOnce(DoAll(SetArgPointee<2>(0x00), Return(HAL_OK))); // current CTRL1_XL 
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, 1, _)).WillRepeatedly(Return(HAL_OK));
    EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, 2, _))
        .WillOnce(DoAll(
            WithArg<1>(Invoke([&](uint8_t *p) { sent[0] = p[0]; sent[1] = p[1]; })),
            Return(HAL_OK)));

    xl_set_fs(LSM6_XL_FS_8G);   // 0b11, bits [3:2], 0x0C 

    EXPECT_EQ(sent[0], LSM6_REG_CTRL1_XL);
    EXPECT_EQ(sent[1], 0x0C);
}

// Chip select control is released when an error occurs
TEST_F(SensorBoardTest, ChipSelectReleasedWhenSpiFails) {
    InitAsLsm6();

    EXPECT_CALL(hal, HAL_SPI_TransmitReceive(_, _, _, _, _)).Times(0);
    {
        InSequence seq;
        EXPECT_CALL(hal, HAL_GPIO_WritePin(_, CsPin, GPIO_PIN_RESET));
        EXPECT_CALL(hal, HAL_SPI_Transmit(_, _, 1, _)).WillOnce(Return(HAL_ERROR));
        EXPECT_CALL(hal, HAL_GPIO_WritePin(_, CsPin, GPIO_PIN_SET));
    }

    EXPECT_EQ(xl_get_x(), 0);
}

// Timeout test 
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