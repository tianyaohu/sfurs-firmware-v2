/* sweep.c plan mode (no hardware): step count, limits and the oblique twists.
 * Ports the plan checks of the old rig_sweep.py. */
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "check.h"

int sweep_main(int argc, char **argv);
#undef main

static int run(char *out, size_t cap, int argc, char **argv)
{
  fflush(stdout);
  char path[] = "/tmp/test_sweep_XXXXXX";
  int fd = mkstemp(path);
  CHECK(fd >= 0);
  int saved = dup(1);
  dup2(fd, 1);
  int rc = sweep_main(argc, argv);
  fflush(stdout);
  dup2(saved, 1);
  close(saved);
  lseek(fd, 0, SEEK_SET);
  ssize_t n = read(fd, out, cap - 1);
  out[n < 0 ? 0 : n] = '\0';
  close(fd);
  unlink(path);
  return rc;
}

int main(void)
{
  char out[1 << 16];
  char *a1[] = {"sweep"};
  CHECK_EQ(run(out, sizeof out, 1, a1), 0);
  /* 1 initial pause + 8 angles x 3 speeds x 2 directions x (move + pause) = 97 steps */
  CHECK(strstr(out, "97 steps"));
  CHECK(strstr(out, "duration_s 121.0")); /* 1 + 48 x 2 x (1.5 + 1) / ... = 1 + 48*2.5 */

  char *a2[] = {"sweep", "--angles-deg", "90", "--speeds", "0.2", "--seconds", "1"};
  CHECK_EQ(run(out, sizeof out, 7, a2), 0);
  CHECK(strstr(out, "5 steps"));
  CHECK(strstr(out, "twist 0 200 0"));   /* straight along vy at 90 degrees */
  CHECK(strstr(out, "twist 0 -200 0"));  /* and its opposite */

  char *a3[] = {"sweep", "--speeds", "0.5"};
  CHECK_EQ(run(out, sizeof out, 3, a3), 2); /* above the 0.3 m/s limit */
  char *a4[] = {"sweep", "--seconds", "4"};
  CHECK_EQ(run(out, sizeof out, 3, a4), 2);
  char *a5[] = {"sweep", "--pause", "0.2"};
  CHECK_EQ(run(out, sizeof out, 3, a5), 2);
  char *a6[] = {"sweep", "--execute"};
  CHECK_EQ(run(out, sizeof out, 2, a6), 2); /* --execute needs --out */
  printf("ok\n");
  return 0;
}
