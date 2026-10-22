#include "esc_defaults.h"

#include <string.h>

#include "gen_esc_defaults.h"

static const struct { const char *key; int value; } DEFAULTS[] = {
    {"kp", ESC_PID_SPEED_KP_DEFAULT}, {"ki", ESC_PID_SPEED_KI_DEFAULT},
    {"stall_ramp_ma_per_s", ESC_SPEED_STALL_RAMP_MA_PER_S_DEFAULT}, {"stall_ms", ESC_SPEED_STALL_MS_DEFAULT}};

int esc_default(const char *key, int *value)
{
  for (unsigned i = 0; i < sizeof DEFAULTS / sizeof DEFAULTS[0]; i++)
    if (!strcmp(DEFAULTS[i].key, key)) {
      *value = DEFAULTS[i].value;
      return 0;
    }
  return -1;
}
