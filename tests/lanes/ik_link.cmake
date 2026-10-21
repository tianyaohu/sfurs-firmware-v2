add_executable(fake_ik_syms fake_ik_syms.c)
target_compile_options(fake_ik_syms PRIVATE -fno-sanitize=all)
target_link_options(fake_ik_syms PRIVATE -fno-sanitize=all)

host_test(test_ik_link
  SOURCES src/tools/lib/ik_link.c src/tools/lib/ocd.c src/tools/lib/elfsym.c
  INCLUDES src/tools/lib src/shared/can
  DEFS FAKE_OCD="$<TARGET_FILE:fake_openocd>" FAKE_ELF="$<TARGET_FILE:fake_ik_syms>"
  DEPENDS fake_openocd fake_ik_syms)
host_test(test_esc_ops
  SOURCES src/tools/lib/esc_ops.c INCLUDES src/tools/lib LIBS m)
host_test(test_esc_defaults
  SOURCES src/tools/lib/esc_defaults.c src/tools/lib/ik_link.c src/tools/lib/ocd.c src/tools/lib/elfsym.c
  INCLUDES src/tools/lib src/shared/can)
