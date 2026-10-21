#include "can_fwu_protocol.h"
#include "can_fwu_validity.h"
#include "check.h"
#include <string.h>

static void test_crc_round_trip(void)
{
  const uint8_t vector[] = "123456789";
  uint8_t motorlike[1024] = {0xF0, 0x7F, 0x00, 0x20, 0x09, 0x40, 0x00, 0x08};
  uint8_t allff[100];
  for (size_t i = 8; i < sizeof(motorlike); ++i)
    motorlike[i] = (uint8_t)((i - 8) * 13 + 7);
  memset(allff, 0xFF, sizeof(allff));

  CHECK(can_fwu_crc32(vector, sizeof(vector) - 1) == 0xCBF43926U);
  /* These independent reference CRCs were calculated by Python zlib. */
  CHECK(can_fwu_crc32(vector, sizeof(vector) - 1) == 0xCBF43926U);
  CHECK(can_fwu_crc32(motorlike, sizeof(motorlike)) == 0xDCA56EAEU);
  CHECK(can_fwu_crc32(allff, sizeof(allff)) == 0x03D28681U);
}

static void test_vector_agreement(void)
{
  static const struct {
    uint32_t sp, reset, size;
    int valid;
  } cases[] = {
    {0x20007FF0U, 0x08004009U, 64, 1},
    {0x20007FF0U, 0x08004009U, 8, 0},
    {0x20007FF0U, 0x08004009U, CAN_FWU_APP_MAX, 1},
    {0x20000000U, 0x08004009U, 64, 0},
    {0x20008000U, 0x08004009U, 64, 0},
    {0x20007FEFU, 0x08004009U, 64, 0},
    {0x20007FF0U, 0x08004008U, 64, 0},
    {0x20007FF0U, 0x08000001U, 64, 0},
    {0x20007FF0U, 0x08004041U, 64, 0},
    {0x20007FF0U, 0x08004009U, 7, 0},
    {0x20007FF0U, 0x08004009U, CAN_FWU_APP_MAX + 1, 0},
  };
  for (size_t i = 0; i < sizeof(cases) / sizeof(*cases); ++i)
    CHECK(!!can_fwu_vectors_valid(cases[i].sp, cases[i].reset, cases[i].size)
          == cases[i].valid);
}

int main(void)
{
  test_crc_round_trip();
  test_vector_agreement();
  return 0;
}
