# Adds gcov-style instrumentation (--coverage) when LIVOX_MID360_ENABLE_COVERAGE is ON.
# Applied to the library and the Catch2 test executable only; see issue #18.
function(livox_mid360_apply_coverage target)
  if(NOT LIVOX_MID360_ENABLE_COVERAGE)
    return()
  endif()
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "LIVOX_MID360_ENABLE_COVERAGE requires GCC or Clang (got ${CMAKE_CXX_COMPILER_ID})")
  endif()
  # The library and the tests are multithreaded; non-atomic counters race and gcov then
  # reports negative branch counts, which gcovr rejects (GCC bug 68080, issue #125).
  target_compile_options(${target} PUBLIC --coverage -fprofile-update=atomic)
  target_link_options(${target} PUBLIC --coverage -fprofile-update=atomic)
endfunction()
