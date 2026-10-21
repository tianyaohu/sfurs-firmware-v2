#include "elfsym.h"

#include <elf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned char *slurp(const char *path, size_t *len)
{
  FILE *f = fopen(path, "rb");
  if (!f) return NULL;
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  rewind(f);
  unsigned char *buf = n > 0 ? malloc((size_t)n) : NULL;
  if (buf && fread(buf, 1, (size_t)n, f) != (size_t)n) {
    free(buf);
    buf = NULL;
  }
  fclose(f);
  *len = buf ? (size_t)n : 0;
  return buf;
}

/* ELF32 and ELF64 differ only in field widths; the two lookups share this shape. */
#define LOOKUP(BITS, ELF_EHDR, ELF_SHDR, ELF_SYM)                                              \
  static int lookup##BITS(const unsigned char *b, size_t len, const char *name, uint32_t *addr, \
                          uint32_t *size)                                                      \
  {                                                                                            \
    const ELF_EHDR *eh = (const ELF_EHDR *)b;                                                  \
    if (len < sizeof *eh || eh->e_shoff + (size_t)eh->e_shnum * sizeof(ELF_SHDR) > len)       \
      return -1;                                                                               \
    const ELF_SHDR *sh = (const ELF_SHDR *)(b + eh->e_shoff);                                  \
    for (unsigned i = 0; i < eh->e_shnum; i++) {                                               \
      if (sh[i].sh_type != SHT_SYMTAB || sh[i].sh_link >= eh->e_shnum) continue;               \
      const ELF_SHDR *str = &sh[sh[i].sh_link];                                                \
      if (sh[i].sh_offset + sh[i].sh_size > len || str->sh_offset + str->sh_size > len)        \
        return -1;                                                                             \
      const ELF_SYM *sym = (const ELF_SYM *)(b + sh[i].sh_offset);                             \
      for (size_t k = 0; k < sh[i].sh_size / sizeof *sym; k++) {                               \
        if (sym[k].st_name >= str->sh_size) continue;                                          \
        if (!strcmp((const char *)b + str->sh_offset + sym[k].st_name, name)) {                \
          *addr = (uint32_t)sym[k].st_value;                                                   \
          if (size) *size = (uint32_t)sym[k].st_size;                                          \
          return 0;                                                                            \
        }                                                                                      \
      }                                                                                        \
    }                                                                                          \
    return -1;                                                                                 \
  }

LOOKUP(32, Elf32_Ehdr, Elf32_Shdr, Elf32_Sym)
LOOKUP(64, Elf64_Ehdr, Elf64_Shdr, Elf64_Sym)

int elfsym_lookup(const char *path, const char *name, uint32_t *addr, uint32_t *size)
{
  size_t len;
  unsigned char *b = slurp(path, &len);
  int rc = -1;
  if (b && len > EI_NIDENT && !memcmp(b, ELFMAG, SELFMAG) && b[EI_DATA] == ELFDATA2LSB) {
    if (b[EI_CLASS] == ELFCLASS32)
      rc = lookup32(b, len, name, addr, size);
    else if (b[EI_CLASS] == ELFCLASS64)
      rc = lookup64(b, len, name, addr, size);
  }
  free(b);
  return rc;
}
