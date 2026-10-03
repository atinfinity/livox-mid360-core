// SPDX-License-Identifier: Apache-2.0
// Shared bits of the livox-mid360-cli sub-commands (issues #35, #93).
#pragma once

#include <atomic>
#include <optional>
#include <string>

#include "livox/mid360/keys.hpp"
#include "livox/mid360/session.hpp"

namespace cli
{
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables): set by the SIGINT handler
extern std::atomic<bool> g_stop;
void install_sigint();

std::optional<livox::mid360::Ipv4> parse_ip(const std::string & s);
// `sn=... ip=... cmd_port=... dev_type=... from=...`, as `list` and `info` print it.
std::string describe(const livox::mid360::DiscoveredDevice & d);

// Exit codes: 0 ok, 1 usage, 2 setup / I/O failure, 3 nothing recorded.
int run_debug_data(int argc, char ** argv);
int run_info(int argc, char ** argv);
int run_list(int argc, char ** argv);
int run_record(int argc, char ** argv);
int run_replay(int argc, char ** argv);
}  // namespace cli
