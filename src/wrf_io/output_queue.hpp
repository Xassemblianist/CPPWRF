// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// The model's WRF-format output written by a writer thread while the model steps on (docs/PERF.md, IO2).
//
// NetCDF (and HDF5 under it) is not thread-safe: two threads must never be inside the library at the same time. The
// process has one NetCDF lock. A thread that makes NetCDF calls of its own (the model's host thread: its readers of
// wrfinput, wrfbdy, wrflowinp, the restart files) holds it from its first queued record on (nc_hold) and lets go of
// it only where it makes no NetCDF call: the model's time step (NcReleased) and its waits for the writer. The writer
// thread takes the lock for each NetCDF call (NcCall, in every NcOut call), so its calls fall between the model's.
//
// Each file's calls (nc_create, the definitions, put_text, the puts of each record in order, nc_sync, nc_close) are
// those of the synchronous writer in the same order; only the thread that makes them and their interleaving with the
// calls of other files change. HDF5 places a file's chunks and metadata from that file's own calls alone, so every
// file has the same bytes.
//
// output.write_queue_mb (cases/domain_runner.cpp) bounds the memory of the records queued (their values and the
// precompressed chunks); a record waits for the queue to shrink below it (one record is always admitted). 0: every
// record is written in the caller's thread, as before. A decomposed run (tiles, MPI) writes synchronously.
#include <cstddef>
#include <functional>

namespace cppwrf::wrf_io {

// RAII: the NetCDF lock for the calls of this scope, unless the calling thread holds it already (nc_hold)
class NcCall {
 public:
  NcCall();
  ~NcCall();
  NcCall(const NcCall&) = delete;
  NcCall& operator=(const NcCall&) = delete;
};

// the calling thread keeps the NetCDF lock (its own NetCDF calls then need no lock each); nc_unhold lets it go
void nc_hold();
void nc_unhold();
bool nc_held();  // by the calling thread

// RAII: a holding thread lets go of the lock in this scope (no NetCDF call in it) and takes it again at its end;
// no effect in a thread that does not hold it
class NcReleased {
 public:
  NcReleased();
  ~NcReleased();
  NcReleased(const NcReleased&) = delete;
  NcReleased& operator=(const NcReleased&) = delete;

 private:
  bool was_held_ = false;
};

namespace output_queue {

// cap: bytes of the queued records (0: synchronous, the default of the library: cppwrf_real writes synchronously).
// The first call sets it for the process; a later call with another cap throws (every domain and ensemble member of
// a run has the same output.write_queue_mb); no effect after forbid()
void configure(std::size_t cap_bytes);
void forbid();  // synchronous for the rest of the process, whatever configure says (decomposed runs: tiles, MPI)
bool on();
// job runs on the writer thread after the jobs submitted before it (off: here, now, after the jobs still queued;
// its exception propagates).
// Waits (without the NetCDF lock) while the queue holds other records and bytes would exceed the cap. Rethrows the
// error of an earlier job first (check).
void submit(std::size_t bytes, std::function<void()> job);
// the first error of a job, rethrown once in the submitting thread, with the job's message; after an error the
// writer skips the jobs still queued (the synchronous writer would have stopped at it)
void check();
// waits for every queued job (without the NetCDF lock), then check(); the calling thread then lets go of the lock
void drain();
// at the end of the process, also after an error: drain(), an error of a job printed instead of thrown; false when
// one was (the run's exit status is then that of an error, not of a non-finite stop)
bool finish() noexcept;
// before a new file is created (NcOut): an error of an earlier job rethrown (no new file after a known error); a
// file of the same path still open in queued jobs (path_open): those jobs done first, so that it is closed before
// nc_create clobbers it
void before_create(bool path_open);

}  // namespace output_queue
}  // namespace cppwrf::wrf_io
