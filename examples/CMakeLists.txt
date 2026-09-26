add_executable(minimal_receive minimal_receive.cpp)
target_link_libraries(minimal_receive PRIVATE livox::mid360_core)
livox_mid360_apply_warnings(minimal_receive)
livox_mid360_apply_sanitizers(minimal_receive)

if(LIVOX_MID360_BUILD_TESTS)
  # Runs the sample for 2 s against the simulator on 127.0.0.1 (skipped without python3).
  add_test(NAME example_minimal_receive
    COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/run_against_sim.sh $<TARGET_FILE:minimal_receive>
      ${PROJECT_SOURCE_DIR}/tools/livox_mid360_sim.py)
  set_tests_properties(example_minimal_receive PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)
endif()
