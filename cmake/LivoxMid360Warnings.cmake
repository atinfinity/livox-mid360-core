function(livox_mid360_apply_warnings target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE
      -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
      -Wcast-align -Wold-style-cast -Wnon-virtual-dtor -Woverloaded-virtual
      -Wdouble-promotion -Wimplicit-fallthrough -Wformat=2)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(${target} PRIVATE -Wduplicated-cond -Wlogical-op -Wuseless-cast)
    endif()
    if(LIVOX_MID360_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()
