#include "ocd.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define SCS_AIRCR 0xE000ED0CU
#define AIRCR_SYSRESETREQ 0x05FA0004U
#define BLOCK_ITEMS 256 /* keeps each Tcl command short */
#define EOM 0x1a        /* Tcl RPC terminates every command and reply with Ctrl-Z */

struct ocd {
  pid_t pid;
  int sock;
  char log_path[64];
  char err[256];
  char *reply;
  size_t reply_cap;
};

static void set_err(ocd_t *o, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void set_err(ocd_t *o, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(o->err, sizeof o->err, fmt, ap);
  va_end(ap);
}

const char *ocd_error(const ocd_t *o) { return o->err; }

void ocd_serial_command(const char *version_text, const char *serial, char *out, size_t outlen)
{
  int major = 0, minor = 0;
  const char *p = strstr(version_text, "Open On-Chip Debugger ");
  if (p) sscanf(p + strlen("Open On-Chip Debugger "), "%d.%d", &major, &minor);
  const int modern = major > 0 || minor >= 12;
  snprintf(out, outlen, "%s %s", modern ? "adapter serial" : "hla_serial", serial);
}

/* Send one command, leave the reply (without the terminator) in o->reply. */
static int tcl_run(ocd_t *o, const char *cmd)
{
  size_t n = strlen(cmd);
  char *msg = malloc(n + 1);
  if (!msg) return -1;
  memcpy(msg, cmd, n);
  msg[n] = EOM;
  ssize_t sent = send(o->sock, msg, n + 1, MSG_NOSIGNAL);
  free(msg);
  if (sent != (ssize_t)(n + 1)) {
    set_err(o, "OpenOCD connection lost sending '%.40s'", cmd);
    return -1;
  }
  size_t len = 0;
  for (;;) {
    if (len + 4096 > o->reply_cap) {
      size_t cap = o->reply_cap ? o->reply_cap * 2 : 16384;
      char *bigger = realloc(o->reply, cap);
      if (!bigger) return -1;
      o->reply = bigger;
      o->reply_cap = cap;
    }
    ssize_t got = recv(o->sock, o->reply + len, o->reply_cap - len - 1, 0);
    if (got <= 0) {
      set_err(o, "OpenOCD closed the connection during '%.40s'", cmd);
      return -1;
    }
    len += (size_t)got;
    if (o->reply[len - 1] == EOM) break;
  }
  o->reply[len - 1] = '\0';
  return 0;
}

int ocd_read(ocd_t *o, uint32_t addr, unsigned width, size_t count, uint32_t *out)
{
  for (size_t off = 0; off < count; off += BLOCK_ITEMS) {
    size_t n = count - off < BLOCK_ITEMS ? count - off : BLOCK_ITEMS;
    char cmd[256];
    /* mem2array, not read_memory: 0.11 has no read_memory. Values come back in decimal. */
    snprintf(cmd, sizeof cmd,
             "mem2array _m %u 0x%x %zu; set _o {}; foreach i [lsort -integer [array names _m]] "
             "{lappend _o $_m($i)}; set _o",
             width, (unsigned)(addr + off * width / 8), n);
    if (tcl_run(o, cmd)) return -1;
    char *p = o->reply;
    for (size_t i = 0; i < n; i++) {
      char *end;
      errno = 0;
      unsigned long v = strtoul(p, &end, 0);
      if (end == p || errno) {
        set_err(o, "read 0x%x: %.200s", (unsigned)addr, o->reply);
        return -1;
      }
      out[off + i] = (uint32_t)v;
      p = end;
    }
  }
  return 0;
}

int ocd_read32(ocd_t *o, uint32_t addr, uint32_t *value) { return ocd_read(o, addr, 32, 1, value); }

int ocd_read_bytes(ocd_t *o, uint32_t addr, uint8_t *buf, size_t n)
{
  uint32_t *tmp = malloc(n * sizeof *tmp);
  if (!tmp) return -1;
  int rc = ocd_read(o, addr, 8, n, tmp);
  if (!rc)
    for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)tmp[i];
  free(tmp);
  return rc;
}

int ocd_write32(ocd_t *o, uint32_t addr, uint32_t value)
{
  char cmd[64];
  snprintf(cmd, sizeof cmd, "mww 0x%x 0x%x", (unsigned)addr, (unsigned)value);
  if (tcl_run(o, cmd)) return -1;
  if (o->reply[0]) {
    set_err(o, "write 0x%x: %.200s", (unsigned)addr, o->reply);
    return -1;
  }
  return 0;
}

void ocd_reset_sysreq(ocd_t *o) { (void)ocd_write32(o, SCS_AIRCR, AIRCR_SYSRESETREQ); }

static void read_log(const ocd_t *o, char *dst, size_t cap)
{
  FILE *f = fopen(o->log_path, "r");
  size_t n = f ? fread(dst, 1, cap - 1, f) : 0;
  dst[n] = '\0';
  if (f) fclose(f);
}

static int find_port(void)
{
  int s = socket(AF_INET, SOCK_STREAM, 0);
  struct sockaddr_in a = {.sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
  socklen_t len = sizeof a;
  int port = -1;
  if (s >= 0 && bind(s, (struct sockaddr *)&a, sizeof a) == 0 && getsockname(s, (struct sockaddr *)&a, &len) == 0)
    port = ntohs(a.sin_port);
  if (s >= 0) close(s);
  return port;
}

static int connect_port(int port)
{
  int s = socket(AF_INET, SOCK_STREAM, 0);
  struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons((uint16_t)port),
                          .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
  if (s >= 0 && connect(s, (struct sockaddr *)&a, sizeof a) == 0) return s;
  if (s >= 0) close(s);
  return -1;
}

static void sleep_ms(long ms) { nanosleep(&(struct timespec){ms / 1000, (ms % 1000) * 1000000L}, NULL); }

static ocd_t *fail(ocd_t *o, char *err, size_t errlen, const char *fmt, ...) __attribute__((format(printf, 4, 5)));
static ocd_t *fail(ocd_t *o, char *err, size_t errlen, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(err, errlen, fmt, ap);
  va_end(ap);
  ocd_close(o);
  return NULL;
}

ocd_t *ocd_open(const char *binary, const char *serial, const char *chip, char *err, size_t errlen)
{
  const char *script, *speed = NULL;
  if (!strcmp(chip, "stm32h755")) {
    script = "target/stm32h7x.cfg";
  } else if (!strcmp(chip, "stm32g431")) {
    script = "target/stm32g4x.cfg";
    speed = "adapter speed 1000";
  } else {
    snprintf(err, errlen, "chip %s; one of stm32h755, stm32g431", chip);
    return NULL;
  }
  if (!binary) binary = getenv("OPENOCD");
  if (!binary) binary = "openocd";

  ocd_t *o = calloc(1, sizeof *o);
  if (!o) return NULL;
  o->sock = -1;
  o->pid = -1;
  snprintf(o->log_path, sizeof o->log_path, "/tmp/ocd-%d-XXXXXX", (int)getpid());

  char cmd[256], version[512] = "";
  snprintf(cmd, sizeof cmd, "'%s' --version 2>&1", binary);
  FILE *p = popen(cmd, "r");
  size_t n = p ? fread(version, 1, sizeof version - 1, p) : 0;
  version[n] = '\0';
  if (!p || pclose(p) != 0 || !n)
    return fail(o, err, errlen, "%s not found; install OpenOCD (apt install openocd)", binary);

  int port = find_port();
  int logfd = mkstemp(o->log_path);
  if (port < 0 || logfd < 0) {
    if (logfd >= 0) close(logfd);
    return fail(o, err, errlen, "cannot allocate a port or log file");
  }
  char serial_cmd[128], port_cmd[48];
  ocd_serial_command(version, serial, serial_cmd, sizeof serial_cmd);
  snprintf(port_cmd, sizeof port_cmd, "tcl_port %d", port);

  const char *argv[24];
  int k = 0;
  argv[k++] = binary;
  argv[k++] = "-f"; argv[k++] = "interface/stlink.cfg";
  argv[k++] = "-c"; argv[k++] = serial_cmd;
  argv[k++] = "-c"; argv[k++] = port_cmd;
  argv[k++] = "-c"; argv[k++] = "gdb_port disabled";
  argv[k++] = "-c"; argv[k++] = "telnet_port disabled";
  if (speed) { argv[k++] = "-c"; argv[k++] = speed; }
  argv[k++] = "-f"; argv[k++] = script;
  argv[k++] = "-c"; argv[k++] = "reset_config none";
  argv[k] = NULL;

  o->pid = fork();
  if (o->pid == 0) {
    dup2(logfd, 1);
    dup2(logfd, 2);
    execvp(binary, (char *const *)argv);
    _exit(127);
  }
  close(logfd);

  for (int tries = 0;; tries++) {
    if (waitpid(o->pid, NULL, WNOHANG) == o->pid) {
      char text[512];
      o->pid = -1;
      read_log(o, text, sizeof text);
      return fail(o, err, errlen, "probe %s (%s): %s", serial, chip, text);
    }
    if ((o->sock = connect_port(port)) >= 0) break;
    if (tries >= 100) return fail(o, err, errlen, "probe %s: OpenOCD not ready in 10 s", serial);
    sleep_ms(100);
  }
  struct timeval tv = {5, 0};
  setsockopt(o->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  return o;
}

void ocd_close(ocd_t *o)
{
  if (!o) return;
  if (o->sock >= 0) {
    (void)tcl_run(o, "shutdown");
    close(o->sock);
  }
  if (o->pid > 0) {
    for (int i = 0; i < 30 && o->pid > 0; i++) {
      if (waitpid(o->pid, NULL, WNOHANG) == o->pid)
        o->pid = -1;
      else
        sleep_ms(100);
    }
    if (o->pid > 0) {
      kill(o->pid, SIGKILL);
      waitpid(o->pid, NULL, 0);
    }
  }
  unlink(o->log_path);
  free(o->reply);
  free(o);
}
