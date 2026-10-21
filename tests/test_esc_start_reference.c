#include "check.h"
#include <string.h>

static char *read_source(const char *name)
{
  char path[1024];
  int n = snprintf(path, sizeof(path), "%s/%s", ESC_SOURCE_DIR, name);
  CHECK(n > 0 && (size_t)n < sizeof(path));
  FILE *file = fopen(path, "rb");
  CHECK(file != NULL);
  CHECK(fseek(file, 0, SEEK_END) == 0);
  long length = ftell(file);
  CHECK(length >= 0);
  rewind(file);
  char *text = malloc((size_t)length + 1);
  CHECK(text != NULL);
  CHECK(fread(text, 1, (size_t)length, file) == (size_t)length);
  CHECK(fclose(file) == 0);
  text[length] = '\0';
  return text;
}

/* Keep the original source check: HALL_Clear requires the STM32 timer. */
static void test_hall_clear_zeroes_the_average_speed(void)
{
  char *text = read_source("hall_speed_pos_fdbk.c");
  char *signature = strstr(text, "void HALL_Clear(");
  CHECK(signature != NULL);
  char *body = strchr(signature, '{');
  CHECK(body != NULL);
  unsigned depth = 1;
  char *end = body + 1;
  while (*end && depth) {
    if (*end == '{') ++depth;
    if (*end == '}') --depth;
    ++end;
  }
  CHECK(depth == 0);
  end[-1] = '\0';
  const char *fields[] = {
    "pHandle->_Super.hAvrMecSpeedUnit = 0;",
    "pHandle->_Super.hElSpeedDpp = 0;",
    "pHandle->AvrElSpeedDpp = 0;",
  };
  for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i)
    CHECK(strstr(body, fields[i]) != NULL);
  free(text);
}

static void test_clear_runs_before_the_reference_is_forced(void)
{
  char *text = read_source("mc_tasks_foc.c");
  char *clear = strstr(text, "HALL_Clear(&HALL_M1);");
  char *force = strstr(text, "STC_ForceSpeedReferenceToCurrentSpeed(");
  CHECK(clear != NULL && force != NULL);
  CHECK(clear < force);
  *force = '\0';
  CHECK(strstr(clear, "SPD_GetAvrgMecSpeedUnit") == NULL);
  free(text);
}

int main(void)
{
  test_hall_clear_zeroes_the_average_speed();
  test_clear_runs_before_the_reference_is_forced();
  return 0;
}
