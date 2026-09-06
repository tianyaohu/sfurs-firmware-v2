#include "hal_mock.h"
#include "wheel_encoder.h"

#include <gtest/gtest.h>

using ::testing::Return;

namespace {
    class WheelEncoderHalTest : public ::testing::Test {
      protected:
        void SetUp() override { g_hal = &hal; }

        void TearDown() override { g_hal = nullptr; }

        MockHal hal;
    };

    TEST(WheelEncoderTest, ReturnsCurrentTimerCount)
    {
        TIM_TypeDef fake_timer {};
        TIM_HandleTypeDef timer_handle {};
        timer_handle.Instance = &fake_timer;
        fake_timer.CNT        = 1234U;
        EXPECT_EQ(Encoder_GetCount(&timer_handle), 1234);
    }

    TEST_F(WheelEncoderHalTest, StartsEncoderOnAllChannels)
    {
        TIM_TypeDef fake_timer {};
        TIM_HandleTypeDef timer_handle {};
        timer_handle.Instance = &fake_timer;

        EXPECT_CALL(hal, HAL_TIM_Encoder_Start(&timer_handle, TIM_CHANNEL_ALL))
            .WillOnce(Return(HAL_OK));

        EXPECT_EQ(Encoder_Start(&timer_handle), HAL_OK);
    }

    TEST_F(WheelEncoderHalTest, ReturnsStartError)
    {
        TIM_HandleTypeDef timer_handle {};

        EXPECT_CALL(hal, HAL_TIM_Encoder_Start(&timer_handle, TIM_CHANNEL_ALL))
            .WillOnce(Return(HAL_ERROR));

        EXPECT_EQ(Encoder_Start(&timer_handle), HAL_ERROR);
    }

    TEST_F(WheelEncoderHalTest, StopsEncoderOnAllChannels)
    {
        TIM_TypeDef fake_timer {};
        TIM_HandleTypeDef timer_handle {};
        timer_handle.Instance = &fake_timer;

        EXPECT_CALL(hal, HAL_TIM_Encoder_Stop(&timer_handle, TIM_CHANNEL_ALL))
            .WillOnce(Return(HAL_OK));

        EXPECT_EQ(Encoder_Stop(&timer_handle), HAL_OK);
    }

    TEST_F(WheelEncoderHalTest, ReturnsStopError)
    {
        TIM_HandleTypeDef timer_handle {};

        EXPECT_CALL(hal, HAL_TIM_Encoder_Stop(&timer_handle, TIM_CHANNEL_ALL))
            .WillOnce(Return(HAL_ERROR));

        EXPECT_EQ(Encoder_Stop(&timer_handle), HAL_ERROR);
    }

} // namespace