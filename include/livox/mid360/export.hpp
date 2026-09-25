// SPDX-License-Identifier: Apache-2.0
// Symbol visibility helpers. The library is built with -fvisibility=hidden; public headers wrap
// their declarations in LIVOX_MID360_API_BEGIN / LIVOX_MID360_API_END to export them.
#pragma once

#if defined(__GNUC__) || defined(__clang__)
#define LIVOX_MID360_API_BEGIN _Pragma("GCC visibility push(default)")
#define LIVOX_MID360_API_END _Pragma("GCC visibility pop")
#else
#define LIVOX_MID360_API_BEGIN
#define LIVOX_MID360_API_END
#endif
