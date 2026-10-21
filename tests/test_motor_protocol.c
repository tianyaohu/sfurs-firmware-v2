#include "motor_protocol.h"
#include "check.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

#define ARRAY_COUNT(a) (sizeof(a) / sizeof((a)[0]))

static const uint32_t wheel_selects[] = {0x1, 0x2, 0x4, 0x8}; /* FR, BR, BL, FL */
static const int16_t signed_values[] = {0, 1, -1, INT16_MIN, INT16_MAX};
static const uint8_t byte_values[] = {0, 1, 127, 128, 255};

static void check_control_fields(const struct motor_control *actual,
                                 const struct motor_control *expected)
{
  CHECK_EQ(actual->mode, expected->mode);
  CHECK_EQ(actual->flags, expected->flags);
  CHECK_EQ(actual->setpoint, expected->setpoint);
  CHECK_EQ(actual->ff_ma, expected->ff_ma);
  CHECK_EQ(actual->seq, expected->seq);
  CHECK_EQ(actual->session, expected->session);
}

static void check_reply_fields(const struct motor_reply *actual,
                               const struct motor_reply *expected)
{
  CHECK_EQ(actual->request_seq, expected->request_seq);
  CHECK_EQ(actual->applied_seq, expected->applied_seq);
  CHECK_EQ(actual->lifecycle, expected->lifecycle);
  CHECK_EQ(actual->outcome, expected->outcome);
  CHECK_EQ(actual->first_edge, expected->first_edge);
  CHECK_EQ(actual->reason, expected->reason);
  CHECK_EQ(actual->session, expected->session);
  CHECK_EQ(actual->rpm, expected->rpm);
  CHECK_EQ(actual->iq_ma, expected->iq_ma);
}

/* Construct test inputs from A's wire layout, independently of the codec. */
static void control_bytes(const struct motor_control *c, unsigned version,
                          uint8_t data[8])
{
  const uint16_t setpoint = (uint16_t)c->setpoint;
  const uint16_t ff_ma = (uint16_t)c->ff_ma;
  data[0] = (uint8_t)((version << 4) | c->mode);
  data[1] = c->flags;
  data[2] = (uint8_t)setpoint;
  data[3] = (uint8_t)(setpoint >> 8);
  data[4] = (uint8_t)ff_ma;
  data[5] = (uint8_t)(ff_ma >> 8);
  data[6] = c->seq;
  data[7] = c->session;
}

static void check_control_decode(uint32_t id, const uint8_t data[8],
                                 const struct motor_control *expected,
                                 enum motor_reason reason)
{
  struct motor_control decoded;
  memset(&decoded, 0xA5, sizeof(decoded));
  CHECK_EQ(motor_control_decode(id, data, 8, &decoded), reason);
  /* The API promises fields even for bad versions, bad IDs and structural rejects. */
  check_control_fields(&decoded, expected);
}

static void check_control_encode(const struct motor_control *c, uint32_t select,
                                 const uint8_t expected[8], enum motor_reason reason)
{
  uint32_t id = UINT32_C(0xDEADBEEF);
  uint8_t encoded[8];
  memset(encoded, 0xA5, sizeof(encoded));
  CHECK_EQ(motor_control_encode(c, select, &id, encoded), reason);
  if (reason == MOTOR_R_NONE) {
    CHECK_EQ(id, 0x010U | select);
    CHECK(memcmp(encoded, expected, sizeof(encoded)) == 0);
  } else {
    CHECK_EQ(id, UINT32_C(0xDEADBEEF));
    for (size_t i = 0; i < sizeof(encoded); ++i)
      CHECK_EQ(encoded[i], 0xA5);
  }
}

static void check_control_case(const struct motor_control *c, uint32_t select,
                               enum motor_reason reason)
{
  uint8_t data[8];
  control_bytes(c, 1, data);
  CHECK_EQ(motor_control_check(c, select), reason);
  check_control_decode(0x010U | select, data, c, reason);
  check_control_encode(c, select, data, reason);
}

static void check_reply_case(const struct motor_reply *input,
                             const uint8_t expected_bytes[8],
                             const struct motor_reply *expected_fields)
{
  uint8_t encoded[8];
  struct motor_reply decoded;
  memset(encoded, 0xA5, sizeof(encoded));
  motor_reply_encode(input, encoded);
  CHECK(memcmp(encoded, expected_bytes, sizeof(encoded)) == 0);
  memset(&decoded, 0xA5, sizeof(decoded));
  CHECK_EQ(motor_reply_decode(expected_bytes, 8, &decoded), MOTOR_R_NONE);
  check_reply_fields(&decoded, expected_fields);
  memset(&decoded, 0xA5, sizeof(decoded));
  CHECK_EQ(motor_reply_decode(encoded, 8, &decoded), MOTOR_R_NONE);
  check_reply_fields(&decoded, expected_fields);
}

static void test_protocol_constants(void)
{
  static const struct { unsigned actual, expected; } cases[] = {
    {MOTOR_PROTOCOL_VERSION, 1}, {MOTOR_PROTOCOL_DLC, 8},
    {MOTOR_PROTOCOL_PERIOD_MS, 20}, {MOTOR_WATCHDOG_MS, 100}, {MOTOR_SEQ_WINDOW, 16},
    {MOTOR_CAN_ID_CONTROL, 0x010}, {MOTOR_CAN_ID_CONFIG, 0x020},
    {MOTOR_CAN_ID_CONTROL_REPLY, 0x410}, {MOTOR_CAN_ID_CONFIG_REPLY, 0x420},
    {MOTOR_CAN_ID_BASE_MASK, 0x7F0}, {MOTOR_CAN_SELECT_MASK, 0x00F},
    {MOTOR_MODE_DISABLE, 0}, {MOTOR_MODE_SPEED, 1}, {MOTOR_MODE_CURRENT, 2},
    {MOTOR_FLAG_ARM, 1}, {MOTOR_FLAG_CLEAR_FAULT, 2}, {MOTOR_FLAG_RESERVED, 0xFC},
    {MOTOR_LC_DISARMED, 0}, {MOTOR_LC_READY, 1}, {MOTOR_LC_STARTING, 2},
    {MOTOR_LC_RUNNING, 3}, {MOTOR_LC_HOLD, 4}, {MOTOR_LC_FAULTED, 5},
    {MOTOR_OUT_APPLIED, 0}, {MOTOR_OUT_INHIBITED, 1}, {MOTOR_OUT_SUPERSEDED, 2},
    {MOTOR_OUT_REJECTED, 3}, {MOTOR_OUT_DUPLICATE, 4},
    {MOTOR_R_NONE, 0}, {MOTOR_R_BAD_VERSION, 1}, {MOTOR_R_BAD_MODE, 2},
    {MOTOR_R_BAD_FLAGS, 3}, {MOTOR_R_RESERVED_NONZERO, 4}, {MOTOR_R_BAD_SELECT, 5},
    {MOTOR_R_OUT_OF_RANGE, 6}, {MOTOR_R_NOT_ARMED, 7}, {MOTOR_R_BAD_SESSION, 8},
    {MOTOR_R_STALE, 9}, {MOTOR_R_SEQ_AMBIGUOUS, 10}, {MOTOR_R_SEQ_JUMP, 11},
    {MOTOR_R_NOT_ALIGNED, 12}, {MOTOR_R_BUSY, 13}, {MOTOR_R_MODE_CHANGE, 14},
    {MOTOR_R_NOT_IDLE, 15}, {MOTOR_R_LIMITED, 16}, {MOTOR_R_STARTING, 17},
    {MOTOR_R_FF_UNUSED, 18}, {MOTOR_R_CONFLICT, 19}, {MOTOR_R_BAD_LENGTH, 20},
    {MOTOR_R_FAULT_MCSDK, 32}, {MOTOR_R_FAULT_STALL, 33},
    {MOTOR_R_FAULT_HALL_LOST, 34}, {MOTOR_R_TIMEOUT, 35},
    {MOTOR_R_LIFECYCLE_OVERFLOW, 36}
  };
  for (size_t i = 0; i < ARRAY_COUNT(cases); ++i)
    CHECK_EQ(cases[i].actual, cases[i].expected);
}

static void test_golden_control_vectors(void)
{
  static const struct {
    uint32_t id;
    uint8_t bytes[8];
    struct motor_control fields;
    enum motor_reason reason;
  } cases[] = {
    {0x011, {0x11,0x00,0x78,0x00,0x00,0x00,0x05,0x3C},
     {1,0,120,0,5,0x3C}, MOTOR_R_NONE},
    {0x011, {0x11,0x00,0x88,0xFF,0x24,0xFA,0x06,0x3C},
     {1,0,-120,-1500,6,0x3C}, MOTOR_R_NONE},
    {0x018, {0x12,0x00,0xC4,0x09,0x00,0x00,0x07,0x3C},
     {2,0,2500,0,7,0x3C}, MOTOR_R_NONE},
    {0x012, {0x10,0x01,0x00,0x00,0x00,0x00,0x00,0x9D},
     {0,1,0,0,0,0x9D}, MOTOR_R_NONE},
    {0x01F, {0x10,0x00,0x00,0x00,0x00,0x00,0x2A,0x00},
     {0,0,0,0,42,0}, MOTOR_R_NONE},
    {0x014, {0x10,0x02,0x00,0x00,0x00,0x00,0x00,0x3C},
     {0,2,0,0,0,0x3C}, MOTOR_R_NONE},
    {0x011, {0x21,0x00,0x78,0x00,0x00,0x00,0x05,0x3C},
     {1,0,120,0,5,0x3C}, MOTOR_R_BAD_VERSION},
    {0x011, {0x13,0x00,0x78,0x00,0x00,0x00,0x05,0x3C},
     {3,0,120,0,5,0x3C}, MOTOR_R_BAD_MODE},
    {0x011, {0x11,0x01,0x78,0x00,0x00,0x00,0x05,0x3C},
     {1,1,120,0,5,0x3C}, MOTOR_R_BAD_FLAGS},
    {0x011, {0x10,0x03,0x00,0x00,0x00,0x00,0x05,0x3C},
     {0,3,0,0,5,0x3C}, MOTOR_R_BAD_FLAGS},
    {0x011, {0x11,0x04,0x78,0x00,0x00,0x00,0x05,0x3C},
     {1,4,120,0,5,0x3C}, MOTOR_R_RESERVED_NONZERO},
    {0x011, {0x12,0x00,0xC4,0x09,0x01,0x00,0x05,0x3C},
     {2,0,2500,1,5,0x3C}, MOTOR_R_RESERVED_NONZERO},
    {0x011, {0x10,0x00,0x01,0x00,0x00,0x00,0x05,0x3C},
     {0,0,1,0,5,0x3C}, MOTOR_R_RESERVED_NONZERO},
    {0x013, {0x11,0x00,0x78,0x00,0x00,0x00,0x05,0x3C},
     {1,0,120,0,5,0x3C}, MOTOR_R_BAD_SELECT}
  };
  for (size_t i = 0; i < ARRAY_COUNT(cases); ++i) {
    check_control_decode(cases[i].id, cases[i].bytes, &cases[i].fields, cases[i].reason);
    if (cases[i].reason != MOTOR_R_BAD_VERSION) {
      CHECK_EQ(motor_control_check(&cases[i].fields, cases[i].id & 0xF), cases[i].reason);
      check_control_encode(&cases[i].fields, cases[i].id & 0xF,
                           cases[i].bytes, cases[i].reason);
    }
  }
}

static void test_golden_reply_vector(void)
{
  static const uint8_t bytes[8] = {0x05,0x05,0x43,0x00,0x3C,0x76,0x00,0x19};
  const struct motor_reply fields = {5,5,3,0,1,0,0x3C,118,1250};
  /* Reply APIs take only the payload; the golden CAN ID is 0x411. */
  CHECK_EQ(MOTOR_CAN_ID_CONTROL_REPLY | 0x1, 0x411);
  check_reply_case(&fields, bytes, &fields);
}

static void test_control_round_trips(void)
{
  for (uint8_t mode = 0; mode <= 2; ++mode) {
    for (size_t wheel = 0; wheel < ARRAY_COUNT(wheel_selects); ++wheel) {
      for (size_t sp = 0; sp < ARRAY_COUNT(signed_values); ++sp) {
        for (size_t ff = 0; ff < ARRAY_COUNT(signed_values); ++ff) {
          /* Nonzero unused fields have rejection tests instead of round trips. */
          if ((mode == 0 && signed_values[sp] != 0) ||
              (mode != 1 && signed_values[ff] != 0))
            continue;
          for (size_t seq = 0; seq < ARRAY_COUNT(byte_values); ++seq) {
            for (size_t session = 0; session < ARRAY_COUNT(byte_values); ++session) {
              for (uint8_t flags = 0; flags <= (mode == 0 ? 2 : 0); ++flags) {
                const struct motor_control c = {
                  mode, flags, signed_values[sp], signed_values[ff],
                  byte_values[seq], byte_values[session]
                };
                uint8_t bytes[8];
                uint32_t id = 0;
                struct motor_control decoded;
                check_control_case(&c, wheel_selects[wheel], MOTOR_R_NONE);
                CHECK_EQ(motor_control_encode(&c, wheel_selects[wheel], &id, bytes),
                         MOTOR_R_NONE);
                CHECK_EQ(motor_control_decode(id, bytes, 8, &decoded), MOTOR_R_NONE);
                check_control_fields(&decoded, &c);
              }
            }
          }
        }
      }
    }
  }
}

static void test_control_structural_rules(void)
{
  static const struct { uint8_t mode, flags; } bad_flags[] = {
    {1,1}, {1,2}, {2,1}, {2,2}, {0,3}, {1,3}, {2,3}
  };
  for (size_t wheel = 0; wheel < ARRAY_COUNT(wheel_selects); ++wheel) {
    const uint32_t select = wheel_selects[wheel];
    for (unsigned version = 0; version <= 15; ++version) {
      if (version == 1)
        continue;
      const struct motor_control c = {1,0,120,-1500,128,255};
      uint8_t data[8];
      control_bytes(&c, version, data);
      check_control_decode(0x010U | select, data, &c, MOTOR_R_BAD_VERSION);
    }
    for (uint8_t mode = 3; mode <= 15; ++mode) {
      const struct motor_control c = {mode,0,0,0,128,255};
      check_control_case(&c, select, MOTOR_R_BAD_MODE);
    }
    for (unsigned bit = 0x04; bit <= 0x80; bit <<= 1) {
      for (uint8_t mode = 0; mode <= 2; ++mode) {
        const struct motor_control c = {mode,(uint8_t)bit,0,0,128,255};
        check_control_case(&c, select, MOTOR_R_RESERVED_NONZERO);
      }
    }
    for (size_t i = 0; i < ARRAY_COUNT(bad_flags); ++i) {
      const struct motor_control c = {bad_flags[i].mode,bad_flags[i].flags,0,0,128,255};
      check_control_case(&c, select, MOTOR_R_BAD_FLAGS);
    }
    for (size_t i = 1; i < ARRAY_COUNT(signed_values); ++i) {
      const struct motor_control bad_fields[] = {
        {0,0,signed_values[i],0,128,255},
        {0,0,0,signed_values[i],128,255},
        {2,0,0,signed_values[i],128,255}
      };
      for (size_t j = 0; j < ARRAY_COUNT(bad_fields); ++j)
        check_control_case(&bad_fields[j], select, MOTOR_R_RESERVED_NONZERO);
    }
  }
}

static void test_control_selects(void)
{
  /* SPEC-QUESTION: A rule 5 omits select == 0; the header explicitly calls it BAD_SELECT. */
  for (uint8_t mode = 0; mode <= 2; ++mode) {
    for (uint8_t flags = 0; flags <= (mode == 0 ? 2 : 0); ++flags) {
      const struct motor_control c = {mode,flags,0,0,128,255};
      check_control_case(&c, 0, MOTOR_R_BAD_SELECT);
    }
  }
  for (uint32_t select = 1; select <= 0xF; ++select) {
    if ((select & (select - 1)) == 0)
      continue;
    for (uint8_t mode = 0; mode <= 2; ++mode) {
      const struct motor_control c = {mode,0,0,0,128,255};
      check_control_case(&c, select, mode == 0 ? MOTOR_R_NONE : MOTOR_R_BAD_SELECT);
    }
    /* SPEC-QUESTION: rule 5 says mode != DISABLE, but its parenthesis and the
     * header allow only plain DISABLE broadcasts; ARM/CLEAR_FAULT must be unicast. */
    for (uint8_t flags = 1; flags <= 2; ++flags) {
      const struct motor_control c = {0,flags,0,0,128,255};
      check_control_case(&c, select, MOTOR_R_BAD_SELECT);
    }
  }
}

static void test_control_rule_order(void)
{
  static const struct {
    unsigned version;
    struct motor_control fields;
    uint32_t select;
    enum motor_reason reason;
  } cases[] = {
    {2, {15,0xFF,1,1,128,255}, 3, MOTOR_R_BAD_VERSION},
    {1, {15,0xFF,1,1,128,255}, 3, MOTOR_R_BAD_MODE},
    {1, {2,0xFF,1,1,128,255}, 3, MOTOR_R_RESERVED_NONZERO},
    {1, {2,0x03,1,1,128,255}, 3, MOTOR_R_BAD_FLAGS},
    {1, {0,0x03,1,1,128,255}, 3, MOTOR_R_BAD_FLAGS},
    {1, {2,0x00,1,1,128,255}, 3, MOTOR_R_RESERVED_NONZERO},
    {1, {0,0x01,1,0,128,255}, 3, MOTOR_R_RESERVED_NONZERO},
    {1, {0,0x02,0,1,128,255}, 3, MOTOR_R_RESERVED_NONZERO},
    {1, {2,0x00,1,0,128,255}, 3, MOTOR_R_BAD_SELECT},
    {1, {0,0x01,0,0,128,255}, 3, MOTOR_R_BAD_SELECT}
  };
  for (size_t i = 0; i < ARRAY_COUNT(cases); ++i) {
    uint8_t data[8];
    control_bytes(&cases[i].fields, cases[i].version, data);
    check_control_decode(0x010U | cases[i].select, data, &cases[i].fields, cases[i].reason);
    if (cases[i].version == 1)
      check_control_case(&cases[i].fields, cases[i].select, cases[i].reason);
    /* Rule 1 outranks every structural failure, including version. */
    for (uint32_t dlc = 0; dlc <= 64; ++dlc) {
      struct motor_control decoded;
      if (dlc != 8)
        CHECK_EQ(motor_control_decode(0x010U | cases[i].select, data, dlc, &decoded),
                 MOTOR_R_BAD_LENGTH);
    }
  }
}

static void test_control_length_and_id(void)
{
  static const uint32_t wrong_bases[] = {0x000,0x020,0x080,0x410,0x420,0x780};
  const struct motor_control c = {1,0,-120,-1500,128,255};
  uint8_t data[8];
  control_bytes(&c, 1, data);
  for (size_t wheel = 0; wheel < ARRAY_COUNT(wheel_selects); ++wheel) {
    for (uint32_t dlc = 0; dlc <= 64; ++dlc) {
      struct motor_control decoded;
      if (dlc != 8)
        CHECK_EQ(motor_control_decode(0x010U | wheel_selects[wheel], data, dlc, &decoded),
                 MOTOR_R_BAD_LENGTH);
    }
    for (size_t i = 0; i < ARRAY_COUNT(wrong_bases); ++i) {
      /* Not a CONTROL frame: rejected and `out` left untouched (it gets no reply). */
      struct motor_control decoded;
      memset(&decoded, 0xA5, sizeof(decoded));
      const struct motor_control untouched = decoded;
      CHECK_EQ(motor_control_decode(wrong_bases[i] | wheel_selects[wheel], data, 8, &decoded),
               MOTOR_R_BAD_LENGTH);
      CHECK(memcmp(&decoded, &untouched, sizeof(decoded)) == 0);
      /* A wrong ID base must also outrank malformed payload fields. */
      const struct motor_control invalid = {15,0xFF,1,1,128,255};
      uint8_t invalid_data[8];
      control_bytes(&invalid, 2, invalid_data);
      CHECK_EQ(motor_control_decode(wrong_bases[i] | wheel_selects[wheel], invalid_data, 8, &decoded),
               MOTOR_R_BAD_LENGTH);
    }
  }
}

static void test_reply_status_combinations(void)
{
  for (uint8_t lifecycle = 0; lifecycle <= 5; ++lifecycle) {
    for (uint8_t outcome = 0; outcome <= 4; ++outcome) {
      for (uint8_t first_edge = 0; first_edge <= 1; ++first_edge) {
        const struct motor_reply r = {0xA5,0x5A,lifecycle,outcome,first_edge,0,255,-120,-1250};
        const uint8_t bytes[8] = {
          0xA5,0x5A,(uint8_t)(lifecycle + 8 * outcome + 64 * first_edge),
          0,255,0x88,0xFF,0xE7
        };
        check_reply_case(&r, bytes, &r);
      }
    }
  }
}

static void test_reply_scalar_fields(void)
{
  for (size_t request = 0; request < ARRAY_COUNT(byte_values); ++request) {
    for (size_t applied = 0; applied < ARRAY_COUNT(byte_values); ++applied) {
      for (size_t session = 0; session < ARRAY_COUNT(byte_values); ++session) {
        for (size_t speed = 0; speed < ARRAY_COUNT(signed_values); ++speed) {
          const struct motor_reply r = {
            byte_values[request],byte_values[applied],3,0,1,0,
            byte_values[session],signed_values[speed],0
          };
          const uint16_t rpm = (uint16_t)r.rpm;
          const uint8_t bytes[8] = {
            r.request_seq,r.applied_seq,0x43,0,r.session,(uint8_t)rpm,(uint8_t)(rpm >> 8),0
          };
          check_reply_case(&r, bytes, &r);
        }
      }
    }
  }
  /* The API validates reply status bits, not reason semantics; preserve the u8. */
  for (unsigned reason = 0; reason <= 255; ++reason) {
    const struct motor_reply r = {5,4,5,3,0,(uint8_t)reason,0x3C,-1,0};
    const uint8_t bytes[8] = {5,4,0x1D,(uint8_t)reason,0x3C,0xFF,0xFF,0};
    check_reply_case(&r, bytes, &r);
  }
}

static void test_reply_iq_quantization(void)
{
  /* SPEC-QUESTION: neither A nor the header defines exact half-step ties.
   * Use nearest 50 mA, with ties away from zero (+25 -> +50, -25 -> -50). */
  static const struct { int16_t ma; int units; } cases[] = {
    {0,0}, {1,0}, {-1,0}, {24,0}, {-24,0}, {25,1}, {-25,-1}, {26,1}, {-26,-1},
    {49,1}, {-49,-1}, {50,1}, {-50,-1}, {74,1}, {-74,-1},
    {75,2}, {-75,-2}, {76,2}, {-76,-2}, {124,2}, {-124,-2},
    {125,3}, {-125,-3}, {126,3}, {-126,-3},
    {1274,25}, {-1274,-25}, {1275,26}, {-1275,-26}, {1276,26}, {-1276,-26},
    {6324,126}, {-6324,-126}, {6325,127}, {-6325,-127}, {6326,127}, {-6326,-127},
    {6349,127}, {-6349,-127}, {6350,127}, {-6350,-127},
    {6351,127}, {-6351,-127}, {6374,127}, {-6374,-127},
    {6375,127}, {-6375,-127}, {6376,127}, {-6376,-127},
    {6400,127}, {-6400,-127}, {10000,127}, {-10000,-127},
    {INT16_MAX,127}, {INT16_MIN,-127}
  };
  for (size_t i = 0; i < ARRAY_COUNT(cases); ++i) {
    const struct motor_reply input = {5,5,3,0,1,0,0x3C,118,cases[i].ma};
    struct motor_reply expected = input;
    expected.iq_ma = (int16_t)(cases[i].units * 50);
    const uint8_t bytes[8] = {5,5,0x43,0,0x3C,0x76,0,(uint8_t)cases[i].units};
    check_reply_case(&input, bytes, &expected);
  }
}

static void test_reply_validation(void)
{
  static const uint8_t valid[8] = {5,5,0x43,0,0x3C,0x76,0,0x19};
  for (uint32_t dlc = 0; dlc <= 64; ++dlc) {
    struct motor_reply decoded;
    if (dlc != 8)
      CHECK_EQ(motor_reply_decode(valid, dlc, &decoded), MOTOR_R_BAD_LENGTH);
  }
  /* Every byte-2 value checks the three restrictions independently and together. */
  for (unsigned status = 0; status <= 255; ++status) {
    uint8_t data[8];
    struct motor_reply decoded;
    memcpy(data, valid, sizeof(data));
    data[2] = (uint8_t)status;
    const unsigned lifecycle = status & 7;
    const unsigned outcome = (status >> 3) & 7;
    const enum motor_reason expected =
      ((status & 0x80) || lifecycle >= 6 || outcome >= 5) ?
      MOTOR_R_RESERVED_NONZERO : MOTOR_R_NONE;
    CHECK_EQ(motor_reply_decode(data, 8, &decoded), expected);
    if (expected == MOTOR_R_NONE) {
      const struct motor_reply fields = {
        5,5,(uint8_t)lifecycle,(uint8_t)outcome,(uint8_t)((status >> 6) & 1),
        0,0x3C,118,1250
      };
      check_reply_fields(&decoded, &fields);
    } else {
      for (uint32_t dlc = 0; dlc <= 64; ++dlc) {
        if (dlc != 8)
          CHECK_EQ(motor_reply_decode(data, dlc, &decoded), MOTOR_R_BAD_LENGTH);
      }
    }
  }
}

static void test_golden_sequence_cases(void)
{
  static const struct { uint8_t seq; enum motor_seq_class expected; } cases[] = {
    {251,MOTOR_SEQ_NEXT}, {10,MOTOR_SEQ_NEXT}, {11,MOTOR_SEQ_JUMP},
    {250,MOTOR_SEQ_DUPLICATE}, {249,MOTOR_SEQ_STALE}, {122,MOTOR_SEQ_AMBIGUOUS}
  };
  /* Equal-sequence payload conflicts are an ESC check, outside this classifier. */
  for (size_t i = 0; i < ARRAY_COUNT(cases); ++i)
    CHECK_EQ(motor_seq_classify(250, cases[i].seq), cases[i].expected);
}

static void test_sequence_exhaustive(void)
{
  for (unsigned last = 0; last <= 255; ++last) {
    for (unsigned seq = 0; seq <= 255; ++seq) {
      /* Express signed modulo-256 arithmetic without implementation-defined casts. */
      int delta = (int)seq - (int)last;
      if (delta < -128)
        delta += 256;
      else if (delta > 127)
        delta -= 256;
      enum motor_seq_class expected;
      if (delta == 0)
        expected = MOTOR_SEQ_DUPLICATE;
      else if (delta == -128)
        expected = MOTOR_SEQ_AMBIGUOUS;
      else if (delta < 0)
        expected = MOTOR_SEQ_STALE;
      else if (delta <= 16)
        expected = MOTOR_SEQ_NEXT;
      else
        expected = MOTOR_SEQ_JUMP;
      CHECK_EQ(motor_seq_classify((uint8_t)last, (uint8_t)seq), expected);
    }
  }
}

static void check_session(uint8_t prev, uint32_t entropy)
{
  const uint32_t expected = prev == 0 ? 1 + entropy % 255 :
    1 + ((uint32_t)prev + entropy % 254) % 255;
  const uint8_t next = motor_session_next(prev, entropy);
  CHECK_EQ(next, expected);
  CHECK(next != 0);
  CHECK(next != prev);
}

static void test_session_next(void)
{
  static const uint32_t entropy_values[] = {
    0,1,127,128,252,253,254,255,256,257,508,509,510,
    65535,65536,UINT32_C(0x7FFFFFFF),UINT32_C(0x80000000),
    UINT32_C(0xFFFFFFFE),UINT32_C(0xFFFFFFFF)
  };
  for (unsigned prev = 0; prev <= 255; ++prev) {
    for (uint32_t entropy = 0; entropy <= 1023; ++entropy)
      check_session((uint8_t)prev, entropy);
    for (size_t i = 0; i < ARRAY_COUNT(entropy_values); ++i)
      check_session((uint8_t)prev, entropy_values[i]);

    uint8_t reached[256] = {0};
    const unsigned entropy_count = prev == 0 ? 255 : 254;
    for (unsigned entropy = 0; entropy < entropy_count; ++entropy) {
      const uint8_t next = motor_session_next((uint8_t)prev, entropy);
      CHECK_EQ(reached[next], 0);
      reached[next] = 1;
    }
    CHECK_EQ(reached[0], 0);
    for (unsigned session = 1; session <= 255; ++session)
      CHECK_EQ(reached[session], session != prev);
  }
}

int main(void)
{
  test_protocol_constants();
  test_golden_control_vectors();
  test_golden_reply_vector();
  test_control_round_trips();
  test_control_structural_rules();
  test_control_selects();
  test_control_rule_order();
  test_control_length_and_id();
  test_reply_status_combinations();
  test_reply_scalar_fields();
  test_reply_iq_quantization();
  test_reply_validation();
  test_golden_sequence_cases();
  test_sequence_exhaustive();
  test_session_next();
  return 0;
}
