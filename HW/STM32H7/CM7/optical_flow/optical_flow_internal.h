#ifndef OPTICAL_FLOW_INTERNAL_H
#define OPTICAL_FLOW_INTERNAL_H

#include <stdint.h>

#define PAA5100_BURST_LENGTH      12U
#define PAA5100_MOTION_READY_MASK 0x80U
#define PAA5100_MIN_VALID_SQUAL   0x19U
#define PAA5100_BAD_SHUTTER_UPPER 0x1FU

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

static inline void paa5100_decode_sample(const uint8_t *burst, PAA5100_SAMPLE_t *sample)
{
    sample->motion  = burst[0];
    sample->delta_x = (int16_t)((((uint16_t)burst[3]) << 8) | burst[2]);
    sample->delta_y = (int16_t)((((uint16_t)burst[5]) << 8) | burst[4]);
    sample->squal   = burst[6];
    sample->raw_sum = burst[7];
    sample->shutter = (uint16_t)((((uint16_t)burst[10]) << 8) | burst[11]);

    sample->motion_usable = ((burst[0] & PAA5100_MOTION_READY_MASK) != 0U);

    if ((sample->motion_usable != 0U) && (sample->squal < PAA5100_MIN_VALID_SQUAL)
        && (burst[10] == PAA5100_BAD_SHUTTER_UPPER)) {
        sample->motion_usable = 0U;
    }
}

#endif