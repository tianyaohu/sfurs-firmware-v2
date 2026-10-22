/* Stand-in `openocd` for ocd.c tests. Fakes only OpenOCD's documented Tcl RPC
 * boundary (Ctrl-Z framed commands; mem2array, mww, shutdown), not a target.
 * Env: FAKE_OCD_VERSION (default 0.12.0), FAKE_OCD_ARGS (file that receives argv). */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define EOM 0x1a
#define SLOTS 512

static struct { unsigned long addr, value; } mem[SLOTS];
static int used;

static unsigned long load(unsigned long addr, unsigned width)
{
  for (int i = 0; i < used; i++)
    if (mem[i].addr == addr) return mem[i].value;
  return addr & (width == 32 ? 0xFFFFFFFFUL : 0xFFUL); /* unwritten memory reads as its own address */
}

static void store(unsigned long addr, unsigned long value)
{
  for (int i = 0; i < used; i++)
    if (mem[i].addr == addr) {
      mem[i].value = value;
      return;
    }
  if (used < SLOTS) mem[used].addr = addr, mem[used++].value = value;
}

int main(int argc, char **argv)
{
  const char *version = getenv("FAKE_OCD_VERSION");
  if (argc == 2 && !strcmp(argv[1], "--version")) {
    fprintf(stderr, "Open On-Chip Debugger %s\n", version ? version : "0.12.0");
    return 0;
  }
  const char *args_file = getenv("FAKE_OCD_ARGS");
  FILE *af = args_file ? fopen(args_file, "w") : NULL;
  int port = 0;
  for (int i = 1; i < argc; i++) {
    if (af) fprintf(af, "%s\n", argv[i]);
    if (strstr(argv[i], "FAIL")) {
      printf("Error: open failed\n");
      return 1;
    }
    if (!strncmp(argv[i], "tcl_port ", 9)) port = atoi(argv[i] + 9);
  }
  if (af) fclose(af);
  /* FAKE_OCD_MEM="hexaddr=hexvalue,..." preloads memory (an unwritten word reads as its own address). */
  const char *preset = getenv("FAKE_OCD_MEM");
  while (preset && *preset) {
    unsigned long a, v;
    int used_chars;
    if (sscanf(preset, "%lx=%lx%n", &a, &v, &used_chars) != 2) break;
    store(a, v);
    preset += used_chars + (preset[used_chars] == ',');
  }

  int srv = socket(AF_INET, SOCK_STREAM, 0);
  struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons((uint16_t)port),
                          .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
  if (bind(srv, (struct sockaddr *)&a, sizeof a) || listen(srv, 1)) return 2;
  int conn = accept(srv, NULL, NULL);
  char buf[4096];
  size_t len = 0;
  for (;;) {
    ssize_t got = recv(conn, buf + len, sizeof buf - len - 1, 0);
    if (got <= 0) return 0;
    len += (size_t)got;
    char *eom;
    while ((eom = memchr(buf, EOM, len))) {
      *eom = '\0';
      char reply[8192] = "";
      unsigned width;
      unsigned long addr, value;
      int count;
      if (sscanf(buf, "mem2array _m %u %lx %d;", &width, &addr, &count) == 3) {
        for (int i = 0; i < count; i++) {
          char item[16];
          snprintf(item, sizeof item, "%s%lu", i ? " " : "", load(addr + (unsigned long)i * width / 8, width));
          strcat(reply, item);
        }
      } else if (sscanf(buf, "mww %lx %lx", &addr, &value) == 2) {
        store(addr, value);
        /* Script memory side effects at a documented write boundary; no target or plant model.
         * FAKE_OCD_ON_WRITE="trigger_addr>dest_addr=value,dest_addr=value,...". */
        const char *script = getenv("FAKE_OCD_ON_WRITE");
        unsigned long trigger;
        int consumed = 0;
        if (script && sscanf(script, "%lx>%n", &trigger, &consumed) == 1 && consumed > 0 && addr == trigger) {
          script += consumed;
          while (*script) {
            unsigned long dest, data;
            if (sscanf(script, "%lx=%lx%n", &dest, &data, &consumed) != 2) break;
            store(dest, data);
            script += consumed;
            if (*script == ',') ++script; else break;
          }
        }
      } else if (!strcmp(buf, "shutdown")) {
        send(conn, "\x1a", 1, 0);
        return 0;
      } else {
        strcpy(reply, "invalid command");
      }
      size_t rl = strlen(reply);
      reply[rl] = EOM;
      send(conn, reply, rl + 1, 0);
      len -= (size_t)(eom - buf) + 1;
      memmove(buf, eom + 1, len);
    }
  }
}
