// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Ensemble runs (docs/ENSEMBLE.md): N members, each a whole single-domain run of the configuration file with its own
// --set overrides, in one process. Every member's output is byte-identical to that member run alone.
//
//   [ensemble]
//   members = 3                                    ; N >= 1
//   member1 =                                      ; empty: the configuration as it is
//   member2 = phys.mp_physics=6 | moist.species=qv,qc,qr,qi,qs,qg
//   member3 = real.dir=/data/run_b | output.out_interval=30
//   mode = threads                                 ; threads (default) | sequential
//
// memberI lists section.key=value overrides separated by '|' (applied after the command line's --set); a member
// without its key is the configuration as it is. Member I writes to <output.dir>/memberI unless its overrides set
// output.dir. mode = threads: every member on its own host thread, with its own CUDA stream, CUDA graphs and fields
// on the one device (the device's shared tables, e.g. Thompson's, once per process); the host work between the
// time steps (input, output, boundaries) runs one member at a time, the time steps at the same time. mode =
// sequential: one member after the other in this thread. `--set ensemble.member=I` runs member I alone, as a
// single run, into the same directory.
#include <string>
#include <vector>

#include "io/config.hpp"

namespace cppwrf::cases {

// the configuration has an [ensemble] section (ensemble.members)
bool is_ensemble(const io::Config& cfg);
// member i (1-based) of the ensemble of the file `path` with the command line's overrides `ov`: the file with ov, the
// member's output directory and the member's overrides, without the [ensemble] section
io::Config ensemble_member_config(const std::string& path, const std::vector<std::string>& ov, const io::Config& cfg,
                                  int i);
// runs every member of the ensemble; the largest member exit code (an error in one member stops all and is rethrown)
int run_ensemble(const std::string& path, const std::vector<std::string>& ov, const io::Config& cfg);

}  // namespace cppwrf::cases
