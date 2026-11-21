// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Decimal text to a floating value as gfortran's READ into a variable of that kind: one correctly rounded
// conversion (libgfortran: strtof for real*4, strtod for real*8). strtod and then a cast to float rounds twice and
// can give the float next to gfortran's (1 ulp) for a decimal close to a midpoint of two floats.
// tools/fp32_double_rounding.py checks the source literals the same way.

#include <cstdlib>
#include <string>
#include <type_traits>

#include "core/precision.hpp"

namespace cppwrf {

// strtod / strtof of s into T (double or float)
template <typename T>
inline T strto_kind(const char* s, char** end) {
  static_assert(std::is_same_v<T, double> || std::is_same_v<T, float>);
  if constexpr (std::is_same_v<T, double>) return std::strtod(s, end);
  else return std::strtof(s, end);
}

// std::stod's checks and exceptions (no number: std::invalid_argument, outside double's range:
// std::out_of_range), the value rounded once to T
template <typename T = real>
inline T sto_kind(const std::string& s) {
  const double d = std::stod(s);
  if constexpr (std::is_same_v<T, double>) return d;
  else return strto_kind<T>(s.c_str(), nullptr);
}

}  // namespace cppwrf
