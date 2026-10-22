#define _POSIX_C_SOURCE 200809L
#include "check.h"
#include <fcntl.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

static char output[32768];
static int run(const char *tool, const char *const *args)
{
  FILE *f = tmpfile(); CHECK(f);
  pid_t pid = fork(); CHECK(pid >= 0);
  if (!pid) {
    CHECK(dup2(fileno(f), STDOUT_FILENO) >= 0); CHECK(dup2(fileno(f), STDERR_FILENO) >= 0);
    char *argv[64]; size_t n = 0; argv[n++] = (char *)tool;
    for (; *args; args++) argv[n++] = (char *)*args;
    argv[n] = NULL; execv(tool, argv); _exit(127);
  }
  int status; CHECK_EQ(waitpid(pid, &status, 0), pid); CHECK(WIFEXITED(status));
  rewind(f); size_t n = fread(output, 1, sizeof output - 1, f); output[n] = 0; fclose(f);
  return WEXITSTATUS(status);
}

static void friction_arguments(void)
{
  CHECK_EQ(run(FRICTION_TOOL, (const char *const[]){"--help", NULL}), 0);
  CHECK(strstr(output, "MOTION")); CHECK(strstr(output, "--breakaway"));
  const char *bad[][2] = {{"--axis", "bad"}, {"--speeds", "0.1,0.2"}, {"--speeds", "-0.2,nan,0.2"},
    {"--speeds", "-0.2,0.2,"}, {"--speeds", "-0.2,0,0.2"}, {"--speeds", "-0.01,0.01"},
    {"--min-vbus", "7"}, {"--max-rpm", "601"}, {"--rate-ma-s", "0"}, {"--cap-ma", "6001"},
    {"--motion-rpm", "301"}, {"--reps", "0"}, {"--wheel", "nope"}, {"--distance", "1.1"},
    {"--transient", "4"}, {"--pause", "-1"}, {"--distance", "nan"}};
  for (size_t i = 0; i < sizeof bad / sizeof *bad; i++) {
    CHECK_EQ(run(FRICTION_TOOL, (const char *const[]){"--out", "/tmp", bad[i][0], bad[i][1], "--elf", "/nonexistent.elf", NULL}), 1);
    CHECK(!strstr(output, "IK ELF"));
  }
  for (int a = 0; a < 3; a++) {
    CHECK_EQ(run(FRICTION_TOOL, (const char *const[]){"--out", "/tmp", "--axis", (const char *[]){"vx", "vy", "wz"}[a],
      "--speeds=-0.1,-0.2,0.1,0.2", "--check", "--elf", "/nonexistent.elf", NULL}), 1);
    CHECK(strstr(output, "IK ELF not found"));
  }
  const char *wheels[] = {"FR", "BR", "BL", "FL", "NE", "SE", "SW", "NW", "all"};
  for (size_t i = 0; i < sizeof wheels / sizeof *wheels; i++) {
    CHECK_EQ(run(FRICTION_TOOL, (const char *const[]){"--out", "/tmp", "--breakaway", "--wheel", wheels[i],
      "--rate-ma-s", "2000", "--cap-ma", "4000", "--reps", "2", "--elf", "/nonexistent.elf", NULL}), 1);
    CHECK(strstr(output, "IK ELF not found"));
  }
}

static void config_arguments(void)
{
  const char *wheels[] = {"FR", "BR", "BL", "FL", "NE", "SE", "SW", "NW", "all"};
  for (size_t i = 0; i < sizeof wheels / sizeof *wheels; i++) {
    CHECK_EQ(run(ESC_TOOL, (const char *const[]){"--elf", "/nonexistent.elf", "get", wheels[i], NULL}), 1);
    CHECK(strstr(output, "IK ELF not found"));
    CHECK_EQ(run(ESC_TOOL, (const char *const[]){"--elf", "/nonexistent.elf", "get", wheels[i], "all", NULL}), 1);
    CHECK(strstr(output, "IK ELF not found"));
    CHECK_EQ(run(ESC_TOOL, (const char *const[]){"--elf", "/nonexistent.elf", "set", wheels[i], "hall_shift_deg", "-64", NULL}), 1);
    CHECK(strstr(output, "IK ELF not found"));
  }
  const char *keys[] = {"iq_limit_ma", "hall_shift_deg", "kp", "ki", "current_kp", "current_ki", "stall_ramp_ma_per_s", "stall_ms", "0x0100"};
  for (size_t i = 0; i < sizeof keys / sizeof *keys; i++) {
    CHECK_EQ(run(ESC_TOOL, (const char *const[]){"--elf", "/nonexistent.elf", "get", "FR", keys[i], NULL}), 1);
    CHECK(strstr(output, "IK ELF not found"));
  }
  const char *bad[] = {"1x", "32768", "-32769", "", "999999999999999999999"};
  for (size_t i = 0; i < sizeof bad / sizeof *bad; i++) {
    CHECK_EQ(run(ESC_TOOL, (const char *const[]){"--elf", "/nonexistent.elf", "set", "FR", "kp", bad[i], NULL}), 1);
    CHECK(strstr(output, "signed 16-bit"));
  }
  CHECK_EQ(run(ESC_TOOL, (const char *const[]){"--elf", "/nonexistent.elf", "get", "FR", "bad", NULL}), 1);
  CHECK(strstr(output, "unknown CONFIG key"));
  CHECK_EQ(run(ESC_TOOL, (const char *const[]){"--elf", "/nonexistent.elf", "get", "FR", "kp", "ki", NULL}), 1);
  CHECK(strstr(output, "usage"));
}

int main(void)
{
  friction_arguments(); config_arguments();
  puts("friction/config CLI: 2 test groups passed");
  return 0;
}
