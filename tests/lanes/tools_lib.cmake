host_test(test_ocd
  SOURCES src/tools/lib/ocd.c INCLUDES src/tools/lib
  DEFS FAKE_OCD="$<TARGET_FILE:fake_openocd>" DEPENDS fake_openocd)
host_test(test_elfsym_crc32
  SOURCES src/tools/lib/elfsym.c src/tools/lib/crc32.c INCLUDES src/tools/lib)
