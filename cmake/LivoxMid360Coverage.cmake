# Adds gcov-style instrumentation (--coverage) when LIVOX_MID360_ENABLE_COVERAGE is ON.
# Applied to the library and the Catch2 test executable only; see issue #18.
function(livox_mid360_apply_coverage target)
  if(NOT LIVOX_MID360_ENABLE_COVERAGE)
    return()
  endif()
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "LIVOX_MID360_ENABLE_COVERAGE requires GCC or Clang (got ${CMAKE_CXX_COMPILER_ID})")
  endif()
  target_compile_options(${target} PUBLIC --coverage)
  target_link_options(${target} PUBLIC --coverage)
endfunction()
