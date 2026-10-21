#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "check.h"
#include "drive_ops.h"
#include <fcntl.h>
#include <netinet/in.h>
#include <pty.h>
#include <signal.h>
#include <string.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

static char directory[] = "/tmp/drive-cli-XXXXXX";
static char output_path[256];
static char output[65536];

static pid_t spawn(const char *tool, const char *const *args, int input_fd)
{
  pid_t pid = fork(); CHECK(pid >= 0);
  if (!pid) {
    int fd = open(output_path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0 || dup2(fd, STDOUT_FILENO) < 0 || dup2(fd, STDERR_FILENO) < 0) _exit(127);
    close(fd);
    if (input_fd >= 0 && dup2(input_fd, STDIN_FILENO) < 0) _exit(127);
    char *argv[128]; size_t n = 0;
    argv[n++] = (char *)tool;
    while (args && *args && n < 127) argv[n++] = (char *)*args++;
    argv[n] = NULL;
    execv(tool, argv);
    _exit(127);
  }
  return pid;
}

static int wait_child(pid_t pid, double timeout_s)
{
  int status;
  double deadline_s = pulse_now_s() + timeout_s;
  for (;;) {
    pid_t got = waitpid(pid, &status, WNOHANG);
    CHECK(got >= 0);
    if (got) { CHECK(WIFEXITED(status)); return WEXITSTATUS(status); }
    if (pulse_now_s() > deadline_s) { kill(pid, SIGKILL); waitpid(pid, &status, 0); CHECK(!"child timed out"); }
    pulse_sleep_s(0.01);
  }
}

static int run(const char *tool, const char *const *args)
{
  int rc = wait_child(spawn(tool, args, -1), 10);
  FILE *f = fopen(output_path, "r"); CHECK(f);
  size_t n = fread(output, 1, sizeof output - 1, f); output[n] = 0; fclose(f);
  return rc;
}

static bool file_contains(const char *path, const char *needle)
{
  FILE *f = fopen(path, "r");
  if (!f) return false;
  char text[65536] = "";
  size_t n = fread(text, 1, sizeof text - 1, f); text[n] = 0; fclose(f);
  return strstr(text, needle) != NULL;
}

static void await_text(const char *path, const char *needle)
{
  double deadline_s = pulse_now_s() + 5;
  while (!file_contains(path, needle)) {
    if (pulse_now_s() > deadline_s) {
      fprintf(stderr, "missing %s in %s\n", needle, path);
      CHECK(!"missing log output");
    }
    pulse_sleep_s(0.01);
  }
}

static void test_help_and_invalid_arguments(void)
{
  const char *tools[] = {MOVE_TOOL, TUNE_TOOL, KEYBOARD_TOOL};
  for (size_t i = 0; i < 3; i++) {
    CHECK_EQ(run(tools[i], (const char *const[]){"--help", NULL}), 0);
    CHECK(strstr(output, i == 2 ? "heartbeat" : "Camera"));
    CHECK_EQ(run(tools[i], (const char *const[]){"--speed", "nan", NULL}), 1);
    CHECK(strstr(output, "invalid option"));
    CHECK_EQ(run(tools[i], (const char *const[]){"--sn", NULL}), 1);
    CHECK_EQ(run(tools[i], (const char *const[]){"--unknown", "1", NULL}), 1);
  }
  CHECK_EQ(run(MOVE_TOOL, (const char *const[]){"--out", directory, "--speed", "0", NULL}), 1);
  CHECK(strstr(output, "require --out"));
  CHECK_EQ(run(MOVE_TOOL, (const char *const[]){"--out", directory, "--distance", "1", "--speed", "0.1", NULL}), 1);
  CHECK_EQ(run(MOVE_TOOL, (const char *const[]){"--out", directory, "--reps", "4", NULL}), 1);
  CHECK_EQ(run(MOVE_TOOL, (const char *const[]){"--out", directory, "--axis", "bad", NULL}), 1);
  CHECK_EQ(run(MOVE_TOOL, (const char *const[]){"--out", directory, "--edge-speed", "2", NULL}), 1);
  CHECK_EQ(run(MOVE_TOOL, (const char *const[]){"--out", directory, "--record-pre", "384", NULL}), 1);
  CHECK_EQ(run(MOVE_TOOL, (const char *const[]){"--out", directory, "--dsc-cap", "2000", NULL}), 1);
  CHECK(strstr(output, "need --dsc"));
  CHECK_EQ(run(MOVE_TOOL, (const char *const[]){"--out", directory, "--kp", "40000", NULL}), 1);
  CHECK_EQ(run(TUNE_TOOL, (const char *const[]){"--out", directory, "--speed", "0.31", NULL}), 1);
  CHECK_EQ(run(TUNE_TOOL, (const char *const[]){"--out", directory, "--x-speed", "0.31", NULL}), 1);
  CHECK_EQ(run(TUNE_TOOL, (const char *const[]){"--out", directory, "--seconds", "2.1", NULL}), 1);
  CHECK_EQ(run(TUNE_TOOL, (const char *const[]){"--out", directory, "--axes", "wz", NULL}), 1);
  CHECK_EQ(run(TUNE_TOOL, (const char *const[]){"--out", directory, "--reps", "6", NULL}), 1);
}

static void test_non_camera_options_parse_before_elf_refusal(void)
{
  CHECK_EQ(run(MOVE_TOOL, (const char *const[]){"--out", directory, "--no-camera", "--axis", "wz",
    "--distance", "0.6", "--speed", "0.2", "--reps", "3", "--accel", "900", "--first-back", "--pause", "1.5",
    "--kp", "100", "--ki", "20", "--ff", "200", "--ff-static", "300", "--ff-kinetic", "250", "--edge-speed", "1",
    "--dsc", "--dsc-ff-run", "200", "--dsc-from", "300", "--dsc-cap", "3000", "--dsc-slope", "2",
    "--dsc-pcap", "1000", "--dsc-hold-decay", "3", "--dsc-watch", "10", "--dsc-preload", "50",
    "--record", "--record-pre", "50", "--record-delay", "1000", "--min-vbus", "11", "--check",
    "--elf", "/nonexistent.elf", "--sn", "FAKE", NULL}), 1);
  CHECK(strstr(output, "IK ELF not found"));
  CHECK_EQ(run(TUNE_TOOL, (const char *const[]){"--out", directory, "--no-camera", "--speed", "0.2", "--x-speed", "0.3",
    "--seconds", "0.8", "--pause", "1.5", "--reps", "2", "--axes", "both", "--kp", "100", "--ki", "20",
    "--frac", "0.65", "--max-tries", "4", "--max-spread", "0.2", "--breakaway-ms", "80", "--spread-ms", "30",
    "--overshoot", "1.3", "--min-vbus", "11", "--elf", "/nonexistent.elf", "--sn", "FAKE", NULL}), 1);
  CHECK(strstr(output, "IK ELF not found"));
  for (int i = 0; i < 2; i++) {
    CHECK_EQ(run(i ? TUNE_TOOL : MOVE_TOOL, (const char *const[]){"--out", directory, "--calibration", "gone.yaml", NULL}), 1);
    CHECK(strstr(output, "invalid option"));
  }
}

static void test_wrong_abi_and_dead_preflight(void)
{
  unsetenv("FAKE_OCD_MEM");
  for (int i = 0; i < 2; i++) {
    CHECK_EQ(run(i ? TUNE_TOOL : MOVE_TOOL, (const char *const[]){"--out", directory, "--elf", FAKE_ELF, NULL}), 1);
    CHECK(strstr(output, "ABI"));
  }
  uint32_t addresses[IK_SYM_COUNT]; char err[256];
  CHECK_EQ(ik_resolve(FAKE_ELF, addresses, err, sizeof err), 0);
  char mem[128]; snprintf(mem, sizeof mem, "%x=%x", addresses[IK_ABI_SYM], IK_ABI);
  setenv("FAKE_OCD_MEM", mem, 1);
  for (int i = 0; i < 2; i++) {
    CHECK_EQ(run(i ? TUNE_TOOL : MOVE_TOOL, (const char *const[]){"--out", directory, "--elf", FAKE_ELF, NULL}), 1);
    CHECK(strstr(output, "preflight failed"));
    for (int w = 0; w < 4; w++) { char want[64]; snprintf(want, sizeof want, "%s: no live reply", IK_WHEEL_POS[w]); CHECK(strstr(output, want)); }
  }
  unsetenv("FAKE_OCD_MEM");
}

static void terminal_case(int mode)
{
  uint32_t addresses[IK_SYM_COUNT]; char err[256];
  CHECK_EQ(ik_resolve(FAKE_ELF, addresses, err, sizeof err), 0);
  char mem[8192]; size_t used = 0;
  used += (size_t)snprintf(mem, sizeof mem, "%x=%x,%x=%x", addresses[IK_ABI_SYM], IK_ABI, addresses[IK_STATE], mode == 2 ? 1 : 0);
  /* Static mailbox words exercise the documented RPC boundary; no target dynamics are simulated. */
  for (int w = 0; w < 4; w++) for (int k = 0; k < IK_ROW_WORDS; k++)
    used += (size_t)snprintf(mem + used, sizeof mem - used, ",%x=0", addresses[IK_TELEM] + (w * IK_ROW_WORDS + k) * 4);
  setenv("FAKE_OCD_MEM", mem, 1);
  int master, slave; CHECK_EQ(openpty(&master, &slave, NULL, NULL, NULL), 0);
  struct termios before, after; CHECK_EQ(tcgetattr(slave, &before), 0);
  char log_path[256]; snprintf(log_path, sizeof log_path, "%s/keyboard-%d.jsonl", directory, mode);
  const char *args[] = {"--elf", FAKE_ELF, "--log", log_path, NULL};
  pid_t pid = spawn(KEYBOARD_TOOL, args, slave);
  await_text(log_path, "\"vector\":[0,0,0]");
  CHECK_EQ(write(master, "w", 1), 1);
  await_text(log_path, "\"vector\":[0,500,0]");
  double quit_s = pulse_now_s();
  if (mode == 2) CHECK_EQ(kill(pid, SIGINT), 0);
  else CHECK_EQ(write(master, mode ? "\033" : "x", 1), 1);
  CHECK_EQ(wait_child(pid, 8), 0);
  if (mode == 2) CHECK(pulse_now_s() - quit_s >= 2.9);
  CHECK_EQ(tcgetattr(slave, &after), 0);
  CHECK_EQ(after.c_lflag, before.c_lflag); CHECK_EQ(after.c_iflag, before.c_iflag);
  CHECK_EQ(after.c_cc[VMIN], before.c_cc[VMIN]); CHECK_EQ(after.c_cc[VTIME], before.c_cc[VTIME]);
  CHECK(file_contains(log_path, "\"t0_monotonic_s\":")); CHECK(file_contains(log_path, "\"t_monotonic_s\":"));
  CHECK(file_contains(output_path, "stopping..."));
  close(master); close(slave); unlink(log_path); unsetenv("FAKE_OCD_MEM");
}

static void test_keyboard_x_restores_terminal(void) { terminal_case(0); }
static void test_keyboard_escape_restores_terminal(void) { terminal_case(1); }
static void test_keyboard_signal_waits_three_seconds_and_restores(void) { terminal_case(2); }

static void test_keyboard_log_exclusive_and_nonterminal_refusal(void)
{
  int master, slave; CHECK_EQ(openpty(&master, &slave, NULL, NULL, NULL), 0);
  CHECK_EQ(wait_child(spawn(KEYBOARD_TOOL, (const char *const[]){"--log", output_path, NULL}, slave), 5), 1);
  CHECK(file_contains(output_path, "File exists"));
  close(master); close(slave);
  int fd = open("/dev/null", O_RDONLY); CHECK(fd >= 0);
  CHECK_EQ(wait_child(spawn(KEYBOARD_TOOL, NULL, fd), 5), 1); close(fd);
  CHECK(file_contains(output_path, "needs a terminal"));
}

int main(int argc, char **argv)
{
  CHECK(mkdtemp(directory));
  CHECK_EQ(drive_path(output_path, sizeof output_path, directory, "output.txt"), 0);
  setenv("OPENOCD", FAKE_OCD, 1);
  if (argc > 1 && !strcmp(argv[1], "--rpc")) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
    bool available = fd >= 0 && bind(fd, (struct sockaddr *)&a, sizeof a) == 0;
    if (fd >= 0) close(fd);
    if (!available) {
      CHECK_EQ(rmdir(directory), 0);
      puts("SKIP: sandbox denies loopback sockets needed by OpenOCD RPC"); return 77;
    }
    test_wrong_abi_and_dead_preflight();
    test_keyboard_x_restores_terminal();
    test_keyboard_escape_restores_terminal();
    test_keyboard_signal_waits_three_seconds_and_restores();
    puts("drive CLI RPC: 4 test cases passed");
  } else {
    test_help_and_invalid_arguments();
    test_non_camera_options_parse_before_elf_refusal();
    test_keyboard_log_exclusive_and_nonterminal_refusal();
    puts("drive CLI: 3 test cases passed");
  }
  unlink(output_path); CHECK_EQ(rmdir(directory), 0);
  return 0;
}
