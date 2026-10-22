#define _POSIX_C_SOURCE 200809L
/** @file keyboard_drive.c
 * @brief Raw-terminal WASD/QE drive, 50 Hz heartbeat and stop-on-exit cleanup.
 */
#include "drive_keyboard.h"
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static volatile sig_atomic_t interrupted;
static void interrupt_handler(int signal_number) { interrupted = signal_number; }

int main(int argc, char **argv)
{
  const char *elf = IK_DEFAULT_ELF, *sn = IK_DEFAULT_SN, *log_path = NULL;
  int accel_rpmps = 0;
  drive_ff_args_t ff_args = {0};
  drive_keyboard_t controls = {.keys = {.hold_s = 0.6}, .speed_mps = 0.5, .max_speed_mps = 2.5, .yaw_radps = 2};
  drive_option options[] = {
    DRIVE_OPT("elf", STRING, &elf), DRIVE_OPT("sn", STRING, &sn),
    DRIVE_OPT("speed", DOUBLE, &controls.speed_mps), DRIVE_OPT("max-speed", DOUBLE, &controls.max_speed_mps),
    DRIVE_OPT("yaw", DOUBLE, &controls.yaw_radps), DRIVE_OPT("hold", DOUBLE, &controls.keys.hold_s),
    DRIVE_OPT("accel", INT, &accel_rpmps), DRIVE_OPT("log", STRING, &log_path), DRIVE_FF_OPTIONS(&ff_args)
  };
  const size_t n_options = sizeof options / sizeof *options;
  int parsed = drive_parse(argc, argv, options, n_options);
  if (parsed) {
    if (parsed > 0) puts("keyboard_drive [--elf F --sn SERIAL --speed 0.5 --max-speed 2.5 --yaw 2 --hold 0.6 --accel 0 --log F]\n"
                         "W/S forward/back, A/D left/right, Q/E CCW/CW, SPACE zero, +/- speed, X/ESC quit.\n"
                         "Axes combine and expire without repeat. Streams at 50 Hz; Nucleo stops after 0.3 s\n"
                         "without heartbeat, ESCs after 0.1 s without the Nucleo. An existing log file is overwritten.\n"
                         DRIVE_FF_USAGE);
    return parsed < 0;
  }
  if (fabs(controls.speed_mps) > INT32_MAX / 1000.0 || fabs(controls.max_speed_mps) > INT32_MAX / 1000.0 ||
      fabs(controls.yaw_radps) > INT32_MAX / 1000.0) {
    fprintf(stderr, "speed/yaw outside the mailbox int32 range\n"); return 1;
  }
  drive_ff_t ff;
  char ff_error[160];
  if (drive_ff_from_options(options, n_options, &ff, ff_error, sizeof ff_error)) {
    fprintf(stderr, "%s\n", ff_error); return 1;
  }
  struct termios saved;
  if (tcgetattr(STDIN_FILENO, &saved)) { perror("keyboard input needs a terminal"); return 1; }
  FILE *log = log_path ? fopen(log_path, "w") : NULL; /* reruns reuse the name (user, 2026-10-22) */
  if (log_path && !log) { perror(log_path); return 1; }
  struct sigaction action = {.sa_handler = interrupt_handler}, old_int, old_term, old_hup;
  sigemptyset(&action.sa_mask);
  sigaction(SIGINT, &action, &old_int);
  sigaction(SIGTERM, &action, &old_term);
  sigaction(SIGHUP, &action, &old_hup);
  ik_link_t link;
  int rc = ik_link_open(&link, sn, elf);
  bool opened = !rc, raw_enabled = false, ff_written = false;
  if (rc) goto done;
  uint32_t fault, state, mode;
  if (ik_read(&link, IK_FAULT, &fault) || ik_check_fault(&link, fault)) { rc = -1; goto done; }
  if (ik_read(&link, IK_STATE, &state) ||
      (state == 0 ? ik_set_mode(&link, IK_MODE_SPEED) : ik_read(&link, IK_MODE, &mode))) { rc = -1; goto done; }
  if (state != 0 && mode != IK_MODE_SPEED) {
    snprintf(link.err, sizeof link.err, "stop the current-mode session before keyboard drive"); rc = -1; goto done;
  }
  if (ik_write(&link, IK_ACCEL_RPMPS, (uint32_t)accel_rpmps)) { rc = -1; goto done; }
  /* Only a stopped drive takes a new model; without ff options the running one is left alone. */
  if (ff.enable) {
    if (drive_ff_write(&link, &ff)) { rc = -1; goto done; }
    ff_written = true;
  }
  struct termios raw = saved;
  raw.c_lflag &= (tcflag_t)~(ICANON | ECHO);
  raw.c_iflag &= (tcflag_t)~(IXON | ICRNL);
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &raw)) { perror("raw terminal"); rc = -1; goto done; }
  raw_enabled = true;
  fputs("\033[?25l", stdout);
  double t0_s = pulse_now_s(), next_tick_s = t0_s, last_ui_s = 0;
  while (!rc) {
    double now_s = pulse_now_s();
    if (interrupted && !controls.quitting) drive_keyboard_press(&controls, 'x', now_s);
    struct pollfd input = {.fd = STDIN_FILENO, .events = POLLIN};
    int ready = poll(&input, 1, 0);
    if (ready < 0 && errno != EINTR) { perror("keyboard poll"); rc = -1; break; }
    if ((input.revents & (POLLHUP | POLLERR | POLLNVAL)) && !controls.quitting)
      drive_keyboard_press(&controls, 'x', now_s);
    if (input.revents & POLLIN) {
      unsigned char bytes[128];
      ssize_t n;
      while ((n = read(STDIN_FILENO, bytes, sizeof bytes)) > 0)
        for (ssize_t k = 0; k < n; k++) drive_keyboard_press(&controls, bytes[k], now_s);
      if (n < 0 && errno != EINTR && errno != EAGAIN) { perror("keyboard read"); rc = -1; break; }
    }
    int32_t vector[3];
    bool enable = drive_keyboard_vector(&controls, now_s, vector);
    if (ik_read(&link, IK_FAULT, &fault) || (fault && ik_check_fault(&link, fault))) { rc = -1; break; }
    if (ik_drive(&link, vector[0], vector[1], vector[2], enable)) { rc = -1; break; }
    if (now_s - last_ui_s >= 0.1) {
      last_ui_s = now_s;
      ik_status_t status;
      ik_row_t rows[4];
      if (ik_status(&link, &status) || ik_rows(&link, rows)) { rc = -1; break; }
      if (log && drive_keyboard_log(log, now_s, t0_s, vector, &status, rows)) { perror(log_path); rc = -1; break; }
      printf("\033[H\033[Jspeed %.1f m/s yaw %.1f rad/s twist vx=%d vy=%d w=%d nucleo %u tx %u fail %u\n",
             controls.speed_mps, controls.yaw_radps, vector[0], vector[1], vector[2], status.state, status.tx_count, status.tx_fail);
      puts("WASD move, QE rotate, SPACE zero, +/- speed, X quit\n");
      drive_keyboard_show(&status, rows);
      puts("\nflags: A active, D I2T derated, L aligning, H Hall shift known (! refuses drive: esc_tool align)");
      if (controls.quitting) puts("stopping...");
      fflush(stdout);
      if (controls.quitting && (status.state == 0 || now_s > controls.quit_deadline_s)) break;
    }
    next_tick_s += 0.02;
    double delay_s = next_tick_s - pulse_now_s();
    if (delay_s > 0) pulse_sleep_s(delay_s);
  }
done:
  /* Never keep streaming during cleanup: the unchanged firmware heartbeat timeouts remain the fallback. */
  if (opened) {
    if (ik_write(&link, IK_ENABLE, 0)) rc = -1;
    if (ff_written && drive_ff_clear(&link)) rc = -1;
    ik_link_close(&link);
  }
  if (raw_enabled) {
    tcsetattr(STDIN_FILENO, TCSANOW, &saved);
    fputs("\033[?25h\n", stdout);
  }
  if (rc && link.err[0]) fprintf(stderr, "%s\n", link.err);
  if (log && fclose(log)) rc = -1;
  sigaction(SIGINT, &old_int, NULL);
  sigaction(SIGTERM, &old_term, NULL);
  sigaction(SIGHUP, &old_hup, NULL);
  return rc != 0;
}
