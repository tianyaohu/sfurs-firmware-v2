add_executable(esc_update_fake_openocd test_esc_update_ocd.c)
host_test(test_sha256 SOURCES src/tools/lib/sha256.c INCLUDES src/tools/lib)
host_test(test_esc_update
  SOURCES src/tools/lib/flashimg_checks.c src/tools/lib/flashimg_install.c
          src/tools/lib/sha256.c src/tools/lib/crc32.c src/tools/lib/elfsym.c
          src/tools/lib/ocd.c src/tools/lib/ik_link.c
  INCLUDES src/tools/lib src/shared/can
  DEFS FAKE_OCD="$<TARGET_FILE:fake_openocd>"
       SCRIPTED_OCD="$<TARGET_FILE:esc_update_fake_openocd>"
  LIBS m DEPENDS fake_openocd esc_update_fake_openocd)
foreach(case flash_matches_sections flash_matches_elf64 installed_app_elf check_battery_off battery_guard
             argument_validation program_nucleo bus_snapshot check_bus verify_flash backup_before_refusal)
  add_test(NAME test_esc_update.${case} COMMAND test_esc_update ${case})
endforeach()
