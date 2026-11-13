// SPDX-License-Identifier: GPL-3.0-or-later
#include "io/option_tiers.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <set>
#include <sstream>

#include "core/strings.hpp"

namespace cppwrf::io {

namespace {

// data/option_tiers.txt (src/io/option_tiers.cmake writes it as a raw string literal at build time)
const char* const kBuiltinTiers =
#include "option_tiers.inc"
    ;

std::optional<double> number(std::string s) {
  if (s.empty()) return std::nullopt;
  for (char& c : s)
    if (c == 'd' || c == 'D') c = 'e';  // Fortran's 1.0d0
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end != s.c_str() + s.size()) return std::nullopt;
  return v;
}

std::string normalize(const std::string& v) {
  std::string s = trim(v);
  if (s.size() >= 2 && (s.front() == '\'' || s.front() == '"') && s.back() == s.front()) s = trim(s.substr(1, s.size() - 2));
  return to_lower(s);
}

std::optional<bool> boolean(const std::string& s) {  // s normalized
  if (s == "true" || s == ".true." || s == "t" || s == ".t." || s == "yes" || s == "1") return true;
  if (s == "false" || s == ".false." || s == "f" || s == ".f." || s == "no" || s == "0") return false;
  return std::nullopt;
}

std::vector<std::string> split(const std::string& s, char sep) {
  std::vector<std::string> out;
  std::string cur;
  std::istringstream in(s);
  while (std::getline(in, cur, sep)) out.push_back(trim(cur));
  return out;
}

void check_item(const std::string& item, const std::string& where) {
  if (item.empty()) throw ConfigError(where + ": empty value");
  const std::size_t colon = item.find(':');
  if (colon != std::string::npos) {
    const auto lo = number(item.substr(0, colon)), hi = number(item.substr(colon + 1));
    if (!lo || !hi || *lo > *hi) throw ConfigError(where + ": value range '" + item + "' is not lo:hi");
  }
}

}  // namespace

const char* tier_name(Tier t) {
  switch (t) {
    case Tier::verified: return "verified";
    case Tier::column_verified: return "column_verified";
    case Tier::unverified: return "unverified";
  }
  return "?";
}

std::vector<TierRow> parse_option_tiers(const std::string& text, const std::string& source) {
  std::vector<TierRow> rows;
  std::istringstream in(text);
  std::string line;
  int n = 0;
  while (std::getline(in, line)) {
    ++n;
    const std::size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    std::istringstream ls(line);
    std::string ckey, wrf, vals, tier;
    if (!(ls >> ckey)) continue;  // blank or comment
    const std::string where = source + ":" + std::to_string(n);
    if (!(ls >> wrf >> vals >> tier))
      throw ConfigError(where + ": a row is <config key> <wrf group.key> <values> <tier> [note]");
    TierRow r;
    r.line = n;
    r.config_key = ckey == "-" ? std::string() : ckey;
    if (wrf != "-") {
      const std::size_t dot = wrf.find('.');
      if (dot == std::string::npos || dot == 0 || dot + 1 == wrf.size())
        throw ConfigError(where + ": the WRF option '" + wrf + "' is not <group>.<key> (or -)");
      r.wrf_group = to_lower(wrf.substr(0, dot));
      r.wrf_key = to_lower(wrf.substr(dot + 1));
    }
    if (r.config_key.empty() && r.wrf_key.empty()) throw ConfigError(where + ": no configuration key and no WRF option");
    if (!r.config_key.empty() && r.config_key.find('.') == std::string::npos)
      throw ConfigError(where + ": the configuration key '" + ckey + "' is not <section>.<key>");
    for (const std::string& v : split(vals, ',')) {
      check_item(v, where);
      r.values.push_back(to_lower(v));
    }
    if (tier == "column_verified") r.tier = Tier::column_verified;
    else if (tier == "unverified") r.tier = Tier::unverified;
    else  // "verified" needs no row: every option without one is verified
      throw ConfigError(where + ": tier '" + tier + "' is not column_verified or unverified");
    std::getline(ls, r.note);
    r.note = trim(r.note);
    rows.push_back(std::move(r));
  }
  return rows;
}

const std::vector<TierRow>& option_tiers() {
  static const std::vector<TierRow> rows = [] {
    std::vector<TierRow> r = parse_option_tiers(kBuiltinTiers, "data/option_tiers.txt");
    if (const char* extra = std::getenv("CPPWRF_OPTION_TIERS_EXTRA"); extra && *extra) {
      std::ifstream f(extra);
      if (!f) throw ConfigError(std::string("CPPWRF_OPTION_TIERS_EXTRA: cannot read ") + extra);
      std::ostringstream s;
      s << f.rdbuf();
      for (TierRow& x : parse_option_tiers(s.str(), extra)) r.push_back(std::move(x));
    }
    return r;
  }();
  return rows;
}

bool tier_value_matches(const std::vector<std::string>& values, const std::string& value) {
  const std::string v = normalize(value);
  const auto vn = number(v);
  for (const std::string& item : values) {
    if (item == "*") return true;
    if (const std::size_t colon = item.find(':'); colon != std::string::npos) {
      if (vn && *vn >= *number(item.substr(0, colon)) && *vn <= *number(item.substr(colon + 1))) return true;
      continue;
    }
    if (const auto in = number(item)) {
      if (vn && *vn == *in) return true;
      continue;
    }
    if (const auto ib = boolean(item)) {
      const auto vb = boolean(v);
      if (vb && *vb == *ib) return true;
      continue;
    }
    if (v == item) return true;
  }
  return false;
}

std::vector<TierHit> config_tier_hits(const Config& cfg, const std::vector<TierRow>& rows) {
  std::vector<TierHit> hits;
  const auto& e = cfg.entries();
  for (const TierRow& r : rows) {
    if (r.config_key.empty()) continue;
    const auto it = e.find(r.config_key);
    if (it == e.end() || !tier_value_matches(r.values, it->second)) continue;
    hits.push_back({r.wrf_key.empty() ? r.config_key : r.wrf_key, r.config_key, trim(it->second), r.tier, r.note});
  }
  return hits;
}

std::vector<TierHit> namelist_tier_hits(const std::function<std::string(const std::string&, const std::string&)>& value,
                                        const std::vector<TierRow>& rows) {
  std::vector<TierHit> hits;
  for (const TierRow& r : rows) {
    if (r.wrf_key.empty()) continue;
    const std::string v = value(r.wrf_group, r.wrf_key);
    if (v.empty() || !tier_value_matches(r.values, v)) continue;
    hits.push_back({r.wrf_key, "&" + r.wrf_group + "." + r.wrf_key, trim(v), r.tier, r.note});
  }
  return hits;
}

std::string tier_attribute(const std::vector<TierHit>& hits) {
  std::string out;
  std::set<std::string> seen;
  for (const TierHit& h : hits) {
    const std::string s = h.option + "=" + h.value + ":" + tier_name(h.tier);
    if (!seen.insert(s).second) continue;
    if (!out.empty()) out += ";";
    out += s;
  }
  return out;
}

void enforce_option_tiers(const std::vector<TierHit>& hits, bool allow, const std::string& context,
                          const std::string& switch_name) {
  if (hits.empty()) return;
  if (!allow) {
    std::string msg = context + "options below the verified tier (not compared with a whole WRF run):";
    for (const TierHit& h : hits)
      msg += " " + h.key + " = " + h.value + " is " + tier_name(h.tier) + (h.note.empty() ? "" : " (" + h.note + ")") + ";";
    msg += " set " + switch_name + " to run them";
    throw ConfigError(msg);
  }
  for (const TierHit& h : hits)
    std::fprintf(stderr, "cppwrf: warning: %s%s = %s is %s%s%s%s, run because %s is set\n", context.c_str(),
                 h.key.c_str(), h.value.c_str(), tier_name(h.tier), h.note.empty() ? "" : " (", h.note.c_str(),
                 h.note.empty() ? "" : ")", switch_name.c_str());
}

std::vector<TierHit> check_option_tiers(const Config& cfg, const std::string& context, const std::vector<TierRow>& rows) {
  const bool allow = cfg.get_bool("cppwrf.allow_unverified", false);
  std::vector<TierHit> hits = config_tier_hits(cfg, rows);
  enforce_option_tiers(hits, allow, context, "cppwrf.allow_unverified = true");
  return hits;
}

}  // namespace cppwrf::io
