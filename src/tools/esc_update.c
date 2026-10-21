#include <stdio.h>
#include "flashimg_update.h"

int main(int argc, char **argv)
{
  flashimg_options_t o;
  char err[1024] = "";
  int rc = flashimg_parse(argc, argv, &o, err, sizeof err);
  if (rc == 1) { flashimg_usage(stdout); return 0; }
  if (!rc) rc = flashimg_install(&o, err, sizeof err);
  if (rc) fprintf(stderr, "ERROR: %s\n", err);
  return rc ? 1 : 0;
}
