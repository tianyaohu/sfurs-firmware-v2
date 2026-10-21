if(NOT TARGET fwu_image)
  add_subdirectory(${REPO}/src/tools/fwu_image ${CMAKE_CURRENT_BINARY_DIR}/fwu_image)
endif()
host_test(test_fwu_image_cli
  SOURCES src/shared/src/can_fwu_protocol.c
  INCLUDES src/shared/inc
  DEFS FWU_IMAGE="$<TARGET_FILE:fwu_image>"
  DEPENDS fwu_image)
foreach(case test_image_sizes test_entry_vectors test_generated_c_round_trip)
  add_test(NAME test_fwu_image_cli.${case} COMMAND test_fwu_image_cli ${case})
endforeach()
foreach(case test_elf_round_trip test_blink_artifact)
  add_test(NAME test_fwu_image_cli.${case}
    COMMAND sh ${REPO}/tests/test_fwu_artifacts.sh ${REPO}
      $<TARGET_FILE:test_fwu_image_cli> ${case})
  set_tests_properties(test_fwu_image_cli.${case} PROPERTIES SKIP_RETURN_CODE 77)
endforeach()
add_test(NAME test_build_scripts
  COMMAND sh ${REPO}/tests/test_build_scripts.sh ${REPO})
