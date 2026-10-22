/* sweep.c plan mode (no hardware): step count, limits and the oblique twists.
 * */
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "check.h"
#include "ik_link.h"

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

static bool mode_selected, disabled, drove;
int __wrap_ik_link_open(ik_link_t *l, const char *sn, const char *elf)
{ (void)sn; (void)elf; memset(l, 0, sizeof *l); return 0; }
void __wrap_ik_link_close(ik_link_t *l) { (void)l; }
int __wrap_ik_read(ik_link_t *l, ik_sym_t sym, uint32_t *value)
{ (void)l; CHECK(sym == IK_STATE || sym == IK_FAULT); *value = 0; return 0; }
int __wrap_ik_write(ik_link_t *l, ik_sym_t sym, uint32_t value)
{ (void)l; CHECK_EQ(sym, IK_ENABLE); CHECK_EQ(value, 0); disabled = true; return 0; }
int __wrap_ik_set_mode(ik_link_t *l, unsigned mode)
{ (void)l; CHECK_EQ(mode, IK_MODE_SPEED); mode_selected = true; return 0; }
int __wrap_ik_check_fault(ik_link_t *l, uint32_t value) { (void)l; CHECK_EQ(value, 0); return 0; }
int __wrap_ik_rows(ik_link_t *l, ik_row_t rows[4])
{
  (void)l;
  for (int w = 0; w < 4; w++) rows[w] = (ik_row_t){.valid = true, .mc_state = "IDLE", .aligned = true};
  return 0;
}
int __wrap_ik_drive(ik_link_t *l, int32_t x, int32_t y, int32_t z, bool enable)
{
  (void)x; (void)y; (void)z; CHECK(enable); CHECK(mode_selected); drove = true;
  strcpy(l->err, "scripted stop before motion"); return -1;
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
  char path[] = "/tmp/sweep-execute-XXXXXX";
  int fd = mkstemp(path); CHECK(fd >= 0); close(fd);
  char *execute[] = {"sweep", "--execute", "--out", path};
  CHECK_EQ(run(out, sizeof out, 4, execute), 1);
  CHECK(mode_selected && drove && disabled); CHECK_EQ(unlink(path), 0);
  printf("ok\n");
  return 0;
}
