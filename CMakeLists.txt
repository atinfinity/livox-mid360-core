cmake_minimum_required(VERSION 3.28)

project(livox_mid360_core
  VERSION 0.1.0
  DESCRIPTION "Unofficial clean-room SDK for the Livox Mid-360 LiDAR (Ubuntu 24.04+)"
  HOMEPAGE_URL "https://github.com/dandelion/livox-mid360-core"
  LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(LivoxMid360Warnings)
include(LivoxMid360Sanitizers)
include(LivoxMid360Coverage)

if(PROJECT_IS_TOP_LEVEL)
  set(CMAKE_CXX_STANDARD 23)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  # No C++20 modules are used; skip the module dependency scan so that Clang
  # builds do not require clang-scan-deps to be installed.
  set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()
endif()

option(LIVOX_MID360_BUILD_TESTS "Build unit tests" ${PROJECT_IS_TOP_LEVEL})
option(LIVOX_MID360_BUILD_FUZZERS "Build libFuzzer targets (Clang only)" OFF)
option(LIVOX_MID360_WARNINGS_AS_ERRORS "Treat warnings as errors" ${PROJECT_IS_TOP_LEVEL})
option(LIVOX_MID360_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(LIVOX_MID360_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(LIVOX_MID360_ENABLE_TSAN "Enable ThreadSanitizer (not with ASan)" OFF)
option(LIVOX_MID360_ENABLE_COVERAGE "Instrument for gcov/gcovr coverage (--coverage)" OFF)
option(BUILD_SHARED_LIBS "Build shared library" ON)

# ---------------------------------------------------------------------------
# Toolchain sanity check: std::expected must be usable. Clang 18 with
# libstdc++ reports __cpp_concepts < 202002L and libstdc++ then hides
# <expected>; use Clang 19+ (or -stdlib=libc++) in that case.
# ---------------------------------------------------------------------------
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_FLAGS "-std=c++23")
check_cxx_source_compiles("
#include <expected>
int main() { std::expected<int, int> e{1}; return *e - 1; }
" LIVOX_MID360_HAS_STD_EXPECTED)
unset(CMAKE_REQUIRED_FLAGS)
if(NOT LIVOX_MID360_HAS_STD_EXPECTED)
  message(FATAL_ERROR
    "std::expected is not available with this compiler/standard library. "
    "Use GCC 13+, or Clang 19+ with libstdc++ (Clang 18 needs -stdlib=libc++).")
endif()

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(mid360_core
  src/crc.cpp
  src/protocol.cpp
  src/keys.cpp
  src/lidar_info.cpp
  src/hms.cpp
  src/session.cpp
  src/config.cpp
  src/frame_assembler.cpp
  src/event.cpp
  src/log.cpp
  src/context.cpp
  src/device.cpp
  src/transport.cpp)
add_library(livox::mid360_core ALIAS mid360_core)

target_include_directories(mid360_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(mid360_core PUBLIC cxx_std_23)
set_target_properties(mid360_core PROPERTIES
  OUTPUT_NAME livox_mid360_core
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  POSITION_INDEPENDENT_CODE ON)
livox_mid360_apply_warnings(mid360_core)
livox_mid360_apply_sanitizers(mid360_core)
livox_mid360_apply_coverage(mid360_core)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(LIVOX_MID360_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Install / export
# ---------------------------------------------------------------------------
install(TARGETS mid360_core EXPORT livox_mid360_core-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT livox_mid360_core-targets
  NAMESPACE livox::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/livox_mid360_core)

configure_package_config_file(
  cmake/livox_mid360_core-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/livox_mid360_core-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/livox_mid360_core)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/livox_mid360_core-config-version.cmake
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/livox_mid360_core-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/livox_mid360_core-config-version.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/livox_mid360_core)
