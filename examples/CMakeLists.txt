add_executable(minimal_receive minimal_receive.cpp)
target_link_libraries(minimal_receive PRIVATE livox::mid360_core)
livox_mid360_apply_warnings(minimal_receive)
livox_mid360_apply_sanitizers(minimal_receive)

add_executable(collect_firmware_log collect_firmware_log.cpp)
target_link_libraries(collect_firmware_log PRIVATE livox::mid360_core)
livox_mid360_apply_warnings(collect_firmware_log)
livox_mid360_apply_sanitizers(collect_firmware_log)

if(LIVOX_MID360_BUILD_TESTS)
  # Runs each sample for 2 s against the simulator on 127.0.0.1 (skipped without python3).
  add_test(NAME example_minimal_receive
    COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/run_against_sim.sh $<TARGET_FILE:minimal_receive>
      ${PROJECT_SOURCE_DIR}/tools/livox_mid360_sim.py)
  set_tests_properties(example_minimal_receive PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)
  add_test(NAME example_collect_firmware_log
    COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/run_against_sim.sh $<TARGET_FILE:collect_firmware_log>
      ${PROJECT_SOURCE_DIR}/tools/livox_mid360_sim.py
      --lidar-ip 127.0.0.1 --host-ip 127.0.0.1 --duration 2
      --out ${CMAKE_CURRENT_BINARY_DIR}/firmware_log_out)
  set_tests_properties(example_collect_firmware_log PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)
endif()
