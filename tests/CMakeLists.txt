find_package(Catch2 3 QUIET)
if(NOT Catch2_FOUND)
  include(FetchContent)
  fetchcontent_declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.7.1
    GIT_SHALLOW TRUE)
  fetchcontent_makeavailable(Catch2)
  list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
endif()
include(Catch)

add_executable(mid360_core_tests
  test_crc.cpp
  test_bytes.cpp
  test_command_frame.cpp
  test_data_packet.cpp
  test_key_value.cpp
  test_keys.cpp
  test_hms.cpp
  test_transport.cpp
  test_sim_smoke.cpp
  test_session.cpp
  test_session_detail.cpp
  test_config.cpp
  test_api_skeleton.cpp
  test_frame_assembler.cpp
  test_device.cpp
  test_typed_keys.cpp
  test_lidar_info.cpp
  test_fov.cpp
  test_point_format.cpp
  test_reconnect.cpp
  test_golden_vectors.cpp)
target_link_libraries(mid360_core_tests PRIVATE livox::mid360_core Catch2::Catch2WithMain)
target_include_directories(mid360_core_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src)
find_package(Python3 COMPONENTS Interpreter QUIET)
target_compile_definitions(mid360_core_tests PRIVATE
  LIVOX_MID360_SIM_SCRIPT="${PROJECT_SOURCE_DIR}/tools/livox_mid360_sim.py")
if(Python3_Interpreter_FOUND)
  target_compile_definitions(mid360_core_tests PRIVATE LIVOX_MID360_PYTHON="${Python3_EXECUTABLE}")
endif()
livox_mid360_apply_warnings(mid360_core_tests)
livox_mid360_apply_sanitizers(mid360_core_tests)
livox_mid360_apply_coverage(mid360_core_tests)
catch_discover_tests(mid360_core_tests)

if(LIVOX_MID360_BUILD_FUZZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(fuzz_targets "")
  foreach(fz IN ITEMS command_frame data_packet key_value_list
                      transport_text session_ack discovery_ack session_loopback
                      frame_assembler)
    add_executable(fuzz_${fz} fuzz/fuzz_${fz}.cpp)
    target_link_libraries(fuzz_${fz} PRIVATE livox::mid360_core)
    # session_detail.hpp lives in src/ (internal, not installed)
    target_include_directories(fuzz_${fz} PRIVATE ${PROJECT_SOURCE_DIR}/src fuzz)
    target_compile_options(fuzz_${fz} PRIVATE -fsanitize=fuzzer)
    target_link_options(fuzz_${fz} PRIVATE -fsanitize=fuzzer)
    list(APPEND fuzz_targets fuzz_${fz})
  endforeach()
  add_custom_target(fuzzers DEPENDS ${fuzz_targets})
endif()
