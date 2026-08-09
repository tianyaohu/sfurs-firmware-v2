#include <gtest/gtest.h>

#include "optical_flow_internal.h"

namespace {
    constexpr uint8_t MotionReady = 0x80U;

    TEST(OpticalFlowDecodeSampleTest, DecodesSignedLittleEndianFields)
    {
        const uint8_t burst[PAA5100_BURST_LENGTH] = {
            MotionReady, 0x00U, 0x34U, 0x12U, 0x78U, 0x56U,
            0x20U,       0x9AU, 0x00U, 0x00U, 0x2AU, 0xBCU,
        };

        PAA5100_SAMPLE_t sample {};

        paa5100_decode_sample(burst, &sample);

        EXPECT_EQ(sample.motion, MotionReady);
        EXPECT_EQ(sample.delta_x, 0x1234);
        EXPECT_EQ(sample.delta_y, 0x5678);
        EXPECT_EQ(sample.squal, 0x20U);
        EXPECT_EQ(sample.raw_sum, 0x9AU);
        EXPECT_EQ(sample.shutter, 0x2ABCU);
        EXPECT_EQ(sample.motion_usable, 1U);
    }

    TEST(OpticalFlowDecodeSampleTest, ClearsMotionUsableWhenMotionReadyBitIsMissing)
    {
        const uint8_t burst[PAA5100_BURST_LENGTH] = {
            0x00U, 0x00U, 0xAAU, 0x55U, 0x44U, 0x33U,
            0x40U, 0x10U, 0x00U, 0x00U, 0x20U, 0x11U,
        };

        PAA5100_SAMPLE_t sample {};

        paa5100_decode_sample(burst, &sample);

        EXPECT_EQ(sample.motion_usable, 0U);
        EXPECT_EQ(sample.delta_x, 0x55AA);
        EXPECT_EQ(sample.delta_y, 0x3344);
    }

    TEST(OpticalFlowDecodeSampleTest, RejectsLowSqualWithBadShutterUpperByte)
    {
        const uint8_t burst[PAA5100_BURST_LENGTH] = {
            MotionReady, 0x00U, 0x01U, 0x00U, 0x02U, 0x00U,
            0x18U,       0x22U, 0x00U, 0x00U, 0x1FU, 0x99U,
        };

        PAA5100_SAMPLE_t sample {};

        paa5100_decode_sample(burst, &sample);

        EXPECT_EQ(sample.motion_usable, 0U);
        EXPECT_EQ(sample.squal, 0x18U);
        EXPECT_EQ(sample.shutter, 0x1F99U);
    }

    TEST(OpticalFlowDecodeSampleTest, KeepsLowSqualWhenShutterUpperByteDoesNotMatchGate)
    {
        const uint8_t burst[PAA5100_BURST_LENGTH] = {
            MotionReady, 0x00U, 0x01U, 0x00U, 0x02U, 0x00U,
            0x18U,       0x22U, 0x00U, 0x00U, 0x20U, 0x99U,
        };

        PAA5100_SAMPLE_t sample {};

        paa5100_decode_sample(burst, &sample);

        EXPECT_EQ(sample.motion_usable, 1U);
    }

    TEST(OpticalFlowDecodeSampleTest, DecodesNegativeSignedDeltaValues)
    {
        const uint8_t burst[PAA5100_BURST_LENGTH] = {
            MotionReady, 0x00U, 0xFFU, 0xFFU, // delta_x = -1
            0x00U,       0x80U,               // delta_y = -32768
            0x20U,       0x00U, 0x00U, 0x00U, 0x00U, 0x01U,
        };

        PAA5100_SAMPLE_t sample {};

        paa5100_decode_sample(burst, &sample);

        EXPECT_EQ(sample.delta_x, -1);
        EXPECT_EQ(sample.delta_y, INT16_MIN);
    }

} // namespace