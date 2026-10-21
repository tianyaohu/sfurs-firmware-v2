#include "check.h"
#include "sha256.h"
#include <string.h>

static void test_empty(void)
{
  char hash[65]; sha256_hex(NULL, 0, hash);
  CHECK(!strcmp(hash, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"));
}

static void test_fips_abc(void)
{
  char hash[65]; sha256_hex("abc", 3, hash);
  CHECK(!strcmp(hash, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"));
}

static void test_fips_multiblock(void)
{
  const char *s = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  char hash[65]; sha256_hex(s, strlen(s), hash);
  CHECK(!strcmp(hash, "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"));
  s = "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmno"
      "ijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu";
  sha256_hex(s, strlen(s), hash);
  CHECK(!strcmp(hash, "cf5b16a778af8380036ce59e7b0492370b249b11e8f07a51afac45037afee9d1"));
}

static void test_fips_million_a(void)
{
  char *s = malloc(1000000); CHECK(s != NULL); memset(s, 'a', 1000000);
  char hash[65]; sha256_hex(s, 1000000, hash); free(s);
  CHECK(!strcmp(hash, "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0"));
}

int main(void)
{
  test_empty(); test_fips_abc(); test_fips_multiblock(); test_fips_million_a();
  puts("4 SHA-256 cases passed (5 known-answer vectors)");
  return 0;
}
