/* The tools restore exactly the RAM tunables the ESC boots with, and the generated
 * values are the ones in drive_parameters.h. */
#include <string.h>

#include "check.h"
#include "esc_defaults.h"
#include "ik_link.h"

int main(void)
{
  int value;
  CHECK_EQ(esc_default("kp", &value), 0);
  CHECK_EQ(value, 1024); /* PID_SPEED_KP_DEFAULT */
  CHECK_EQ(esc_default("ki", &value), 0);
  CHECK_EQ(value, 553); /* PID_SPEED_KI_DEFAULT */
  CHECK_EQ(esc_default("stall_ramp_ma_per_s", &value), 0);
  CHECK_EQ(value, 10000); /* SPEED_STALL_RAMP_MA_PER_S_DEFAULT */
  CHECK_EQ(esc_default("stall_ms", &value), 0);
  CHECK_EQ(value, 500); /* SPEED_STALL_MS_DEFAULT */
  /* Every default is a settable CONFIG key. */
  const char *const restored[] = {"kp", "ki", "stall_ramp_ma_per_s", "stall_ms"};
  for (size_t k = 0; k < sizeof restored / sizeof restored[0]; k++) {
    uint32_t id, word;
    CHECK(esc_default(restored[k], &value) == 0);
    CHECK_EQ(ik_config_frame(IK_WHEEL_SELECT[0], restored[k], value, &id, &word), 0);
  }
  CHECK(esc_default("current_kp", &value) != 0); /* not RAM-restored by the tools */
  CHECK(esc_default("ff_static_ma", &value) != 0); /* retired */
  printf("ok\n");
  return 0;
}
