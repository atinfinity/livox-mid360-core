// SPDX-License-Identifier: Apache-2.0
#include <catch2/catch_test_macros.hpp>

#include "livox/mid360/hms.hpp"

using namespace livox::mid360;

TEST_CASE("hms decode example from the wiki", "[hms]")
{
  constexpr auto c = decode_hms(0x01020002);
  STATIC_CHECK(c.abnormal_id == 0x0102);
  STATIC_CHECK(c.level == HmsLevel::kWarning);
  STATIC_CHECK(c.active());
  CHECK(hms_description(c.abnormal_id) == "Environment temperature is slightly high");
  CHECK(!hms_suggestion(c.abnormal_id).empty());
  CHECK(to_string(c.level) == "warning");
}

TEST_CASE("hms table ranges and unknowns", "[hms]")
{
  CHECK_FALSE(decode_hms(0).active());
  CHECK(hms_description(0x0215).starts_with("Scan module is abnormal"));
  CHECK(hms_description(0x0219).starts_with("Scan module is abnormal"));
  CHECK(hms_description(0x021A).empty());
  CHECK(hms_description(0x0112) == "Abnormal temperature of internal components of the device");
  CHECK(hms_description(0x040A).starts_with("The PTP and gPTP"));
  CHECK(hms_description(0xFFFF).empty());
  CHECK(decode_hms(0x02110004).level == HmsLevel::kFatal);
}
