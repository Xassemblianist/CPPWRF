// SPDX-License-Identifier: GPL-3.0-or-later
// NSSL 2-moment microphysics: host-side error exits of the row kernel.
#include <stdexcept>
#include <string>
#include "phys/mp_nssl/nssl_util.hpp"

namespace cppwrf::phys::mp_nssl {
void nssl_stop(int line) {
  throw std::runtime_error("NSSL microphysics: WRF's module_mp_nssl_2mom stops here (kernel line " +
                           std::to_string(line) + ")");
}
void Work::throw_overflow() { throw std::runtime_error("NSSL microphysics: row work space too small"); }
}  // namespace cppwrf::phys::mp_nssl
