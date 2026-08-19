#include <gtest/gtest.h>

#include "optical_flow_internal.h"

namespace {
    constexpr uint8_t MotionReady = 0x80U;

    /**
     * Test inputs:
     * - Motion-ready bit is set.
     * - delta_x bytes encode 0x1234.
     * - delta_y bytes encode 0x5678.
     * - SQUAL = 0x20, raw_sum = 0x9A, shutter = 0x2ABC.
     *
     * Expected outputs:
     * - All burst fields are decoded into the correct sample members.
     * - motion_usable is set to 1.
     *
     * How this tests the function:
     * - Verifies that paa5100_decode_sample() correctly interprets the burst
     *   byte layout, including little-endian 16-bit fields and validity state.
     */
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

    /**
     * Test inputs:
     * - Motion-ready bit is not set.
     * - delta_x bytes encode 0x55AA.
     * - delta_y bytes encode 0x3344.
     *
     * Expected outputs:
     * - motion_usable is cleared to 0.
     * - Motion delta fields are still decoded correctly.
     *
     * How this tests the function:
     * - Verifies that paa5100_decode_sample() uses the motion-ready bit to
     *   determine motion validity independently of decoding the delta fields.
     */
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

    /**
     * Test inputs:
     * - Motion-ready bit is set.
     * - SQUAL = 0x18, which is below the valid threshold.
     * - Shutter upper byte = 0x1F, matching the invalid-motion gate.
     *
     * Expected outputs:
     * - motion_usable is cleared to 0.
     * - SQUAL and shutter values are decoded correctly.
     *
     * How this tests the function:
     * - Exercises the invalid-motion quality check and verifies that motion is
     *   rejected when both low SQUAL and the bad shutter condition are present.
     */
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

    /**
     * Test inputs:
     * - Motion-ready bit is set.
     * - SQUAL = 0x18, which is below the valid threshold.
     * - Shutter upper byte = 0x20, which does not match the invalid-motion gate.
     *
     * Expected outputs:
     * - motion_usable remains set to 1.
     *
     * How this tests the function:
     * - Verifies that low SQUAL alone does not invalidate motion and that both
     *   conditions in the rejection rule must be satisfied.
     */
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

    /**
     * Test inputs:
     * - delta_x bytes = 0xFFFF, representing -1 as a signed 16-bit value.
     * - delta_y bytes = 0x8000, representing INT16_MIN (-32768).
     *
     * Expected outputs:
     * - delta_x = -1.
     * - delta_y = INT16_MIN.
     *
     * How this tests the function:
     * - Verifies that paa5100_decode_sample() correctly combines the burst bytes
     *   and preserves two's-complement signed 16-bit motion values.
     */
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