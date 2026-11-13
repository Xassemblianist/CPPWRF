// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Verification tiers of options (data/option_tiers.txt, compiled into the binaries and read by
// tools/wrf2cppwrf.py). Every option and value without a row is "verified": its whole-model output is identical to
// WRF's at tolerance 0 (the suite). A row lowers one option value to
//   column_verified: the scheme's routines match WRF's Fortran bit for bit in a column / unit driver (tests/phys),
//   unverified:      ported and compiling only.
// A run with such a value stops at the configuration check unless cppwrf.allow_unverified = true (namelist.input:
// &cppwrf allow_unverified = .true. /); with the switch the run logs a warning per option and every wrfout / wrfrst
// file carries the global attribute CPPWRF_UNVERIFIED_OPTIONS = "mp_physics=4:column_verified;...".
#include <functional>
#include <string>
#include <vector>

#include "io/config.hpp"

namespace cppwrf::io {

enum class Tier { verified, column_verified, unverified };
const char* tier_name(Tier t);

struct TierRow {
  std::string config_key;               // the CPPWRF configuration key, e.g. phys.mp_physics ("-": none)
  std::string wrf_group, wrf_key;       // the WRF namelist option, e.g. physics / mp_physics (empty: none)
  std::vector<std::string> values;      // integers, ranges lo:hi, words (true / false / text) or * (any value)
  Tier tier = Tier::verified;
  std::string note;                     // the scheme, and where its column driver is
  int line = 0;
};

// The rows of a table text (one row per line; # comments): throws ConfigError on a malformed line.
std::vector<TierRow> parse_option_tiers(const std::string& text, const std::string& source);
// The built-in table, plus the rows of the file named by $CPPWRF_OPTION_TIERS_EXTRA when set (tests; trying a tier).
const std::vector<TierRow>& option_tiers();

// one option value set below "verified"
struct TierHit {
  std::string option;  // the WRF option name (the configuration key without a WRF name)
  std::string key;     // where it was found: the configuration key or &group.key
  std::string value;
  Tier tier = Tier::verified;
  std::string note;
};

// does a configuration / namelist value match one of a row's values
bool tier_value_matches(const std::vector<std::string>& values, const std::string& value);
// the rows' options in a configuration (only keys present: an absent key keeps its default, which is verified);
// reads cfg.entries(), marks no key accessed
std::vector<TierHit> config_tier_hits(const Config& cfg, const std::vector<TierRow>& rows = option_tiers());
// the same for a WRF namelist: value(group, key) gives the value in effect (empty: not set)
std::vector<TierHit> namelist_tier_hits(const std::function<std::string(const std::string&, const std::string&)>& value,
                                        const std::vector<TierRow>& rows = option_tiers());
// "mp_physics=4:column_verified;..." (empty without hits)
std::string tier_attribute(const std::vector<TierHit>& hits);
// The gate: hits without allow -> ConfigError naming each option, value and tier; with allow, one warning per hit on
// stderr ("cppwrf: warning: ..."). context prefixes the messages (e.g. "d02: "), switch_name is the switch to name.
void enforce_option_tiers(const std::vector<TierHit>& hits, bool allow, const std::string& context,
                          const std::string& switch_name);
// enforce_option_tiers on a configuration with its cppwrf.allow_unverified; returns the hits
std::vector<TierHit> check_option_tiers(const Config& cfg, const std::string& context = "",
                                        const std::vector<TierRow>& rows = option_tiers());

}  // namespace cppwrf::io
