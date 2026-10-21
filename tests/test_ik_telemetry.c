#include "ik_telemetry.h"
#include "can_datatypes.h"
#include "check.h"
#include <stddef.h>

static void test_c_decoder_to_python_row(void) {
  uint32_t seen[4 * 17];
  unsigned n = 0;
  for (uint32_t t = 5; n < 4 * 17; ++t) {
    uint32_t id = ik_telemetry_poll_due(t);
    if (id) seen[n++] = id;
  }
  for (unsigned c = 0; c < 4; ++c) {
    for (unsigned i = 0; i < 16; ++i)
      CHECK(seen[17 * c + i] == ((i >= 12 ? DIAG_READ : STATUS_READ) | (1U << (i % 4))));
    CHECK(seen[17 * c + 16] == (IDENT_READ | (1U << c)));
  }
  CHECK(ik_telemetry_poll_due(1000) && !ik_telemetry_poll_due(1004));

  const uint8_t c0[8] = {0x0A, 0x00, 0x20, 0x02, 0xFF, 0xFF, 0x10, 0x27};
  const uint8_t c1[8] = {0x0A, 0x10, 0x9C, 0xFF, 0x60, 0x06, 0x00, 0x00};
  const uint8_t ci[8] = {0x00, 0x30, 0x02, 0x00, 0xD4, 0x04, 0x00, 0x02};
  const uint8_t cbad[8] = {0x00, 0x22, 1, 2, 3, 4, 5, 6};
  const int16_t want[6] = {544, -1, 10000, -100, 1632, 0};
  const uint8_t status[8] = {0x38, 0xff, 0xd2, 0x04, 0x14, 0x05, 6, 0x31};
  const uint8_t diag[8] = {0xd4, 0x04, 0x40, 0x00, 0xb3, 0xff, 1, 42};
  const uint8_t ident[8] = {0xEF, 0xBE, 0xAD, 0xDE, 0x18, 0xB1, 0x00, 0x00};

  for (unsigned wheel = 0; wheel < 4; ++wheel) {
    const uint32_t selector = 1U << wheel;
    CHECK(ik_telemetry_rx(CAPTURE_REPLY | selector, c0, 1));
    CHECK(ik_telemetry_rx(CAPTURE_REPLY | selector, c1, 1));
    CHECK(ik_telemetry_rx(CAPTURE_REPLY | selector, ci, 1));
    CHECK(ik_telemetry_rx(CAPTURE_REPLY | selector, cbad, 1));
    CHECK(!ik_telemetry_rx(CAPTURE_REPLY | 3U, c0, 1));
    CHECK(ik_drive_capture.frames == 3 * (wheel + 1) && ik_drive_capture.source == selector);
    CHECK(ik_drive_capture.status == ESC_CAPTURE_DONE && ik_drive_capture.vbus_centivolts == 1236);
    CHECK(ik_drive_capture.samples == 512);
    for (unsigned k = 0; k < 6; ++k) CHECK(ik_drive_capture.sample[10][k] == want[k]);

    CHECK(ik_telemetry_rx(STATUS_REPLY | selector, status, 77));
    CHECK(ik_telemetry_rx(DIAG_REPLY | selector, diag, 78));
    CHECK(ik_telemetry_rx(IDENT_REPLY | selector, ident, 79));
    CHECK(!ik_telemetry_rx(STATUS_REPLY | 3U, status, 79));
    CHECK(!ik_telemetry_rx(SPEED_WRITE | selector, status, 79));

    /* Check the firmware row directly; host formatting belongs to the tool tests. */
    const volatile struct ik_telemetry_wheel *row = &ik_drive_telem[wheel];
    CHECK_EQ(row->generation, 6);
    CHECK_EQ(row->rpm, -200);
    CHECK_EQ(row->iq_counts, 1234);
    CHECK_EQ(row->iq_ref_counts, 1300);
    CHECK_EQ(row->state, 6);
    CHECK_EQ(row->flags, 0x31);
    CHECK_EQ(row->status_tick, 77);
    CHECK_EQ(row->status_count, 1);
    CHECK_EQ(row->vbus_centivolts, 1236);
    CHECK_EQ(row->faults, 0x40);
    CHECK_EQ(row->hall_shift_deg, -77);
    CHECK_EQ(row->align_result, 1);
    CHECK_EQ(row->i2t_percent, 42);
    CHECK_EQ(row->diag_tick, 78);
    CHECK_EQ(row->diag_count, 1);
    CHECK_EQ(row->app_crc, 0xDEADBEEF);
    CHECK_EQ(row->app_size, 45336);
    CHECK_EQ(row->ident_tick, 79);
    CHECK_EQ(row->ident_count, 1);
  }
}

static void test_row_layout_matches_header(void) {
  CHECK_EQ(IK_TELEMETRY_WORDS, 19);
  CHECK_EQ(sizeof(struct ik_telemetry_wheel), IK_TELEMETRY_WORDS * sizeof(uint32_t));
#define CHECK_WORD(field, index) \
  CHECK_EQ(offsetof(struct ik_telemetry_wheel, field), (index) * sizeof(uint32_t))
  CHECK_WORD(generation, 0);
  CHECK_WORD(rpm, 1);
  CHECK_WORD(iq_counts, 2);
  CHECK_WORD(iq_ref_counts, 3);
  CHECK_WORD(state, 4);
  CHECK_WORD(flags, 5);
  CHECK_WORD(status_tick, 6);
  CHECK_WORD(status_count, 7);
  CHECK_WORD(vbus_centivolts, 8);
  CHECK_WORD(faults, 9);
  CHECK_WORD(hall_shift_deg, 10);
  CHECK_WORD(align_result, 11);
  CHECK_WORD(i2t_percent, 12);
  CHECK_WORD(diag_tick, 13);
  CHECK_WORD(diag_count, 14);
  CHECK_WORD(app_crc, 15);
  CHECK_WORD(app_size, 16);
  CHECK_WORD(ident_tick, 17);
  CHECK_WORD(ident_count, 18);
#undef CHECK_WORD
}

int main(void) {
  test_c_decoder_to_python_row();
  test_row_layout_matches_header();
  return 0;
}
