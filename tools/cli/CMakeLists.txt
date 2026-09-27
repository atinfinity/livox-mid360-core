add_executable(livox-mid360-cli main.cpp debug_data.cpp record.cpp replay.cpp)
target_link_libraries(livox-mid360-cli PRIVATE livox::mid360_core)
livox_mid360_apply_warnings(livox-mid360-cli)
livox_mid360_apply_sanitizers(livox-mid360-cli)
install(TARGETS livox-mid360-cli RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(LIVOX_MID360_BUILD_TESTS)
  # record 2 s from the simulator, then replay the file at full speed and compare the counts.
  # Shares the simulator's default ports with the example tests, hence the lock.
  add_test(NAME cli_record_replay
    COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/test_record_replay.sh $<TARGET_FILE:livox-mid360-cli>
      ${PROJECT_SOURCE_DIR}/tools/livox_mid360_sim.py ${CMAKE_CURRENT_BINARY_DIR}/record_replay.lvx2)
  set_tests_properties(cli_record_replay PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60
    RESOURCE_LOCK sim_default_ports)
  # debug-data: 2 s of the simulator's debug raw data, validated by the Python reader.
  add_test(NAME cli_debug_data
    COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/test_debug_data.sh $<TARGET_FILE:livox-mid360-cli>
      ${PROJECT_SOURCE_DIR}/tools/livox_mid360_sim.py
      ${PROJECT_SOURCE_DIR}/tools/livox_mid360_debug_data.py
      ${CMAKE_CURRENT_BINARY_DIR}/debug_data.bin)
  set_tests_properties(cli_debug_data PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60
    RESOURCE_LOCK sim_default_ports)
endif()
