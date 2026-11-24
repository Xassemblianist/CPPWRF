// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Parallel compression of NetCDF-4 output without changing a byte of the files.
//
// WRF's NetCDF-4 output (external/io_netcdf: shuffle + deflate level 2 per chunk) is compressed by HDF5's deflate
// filter, one chunk after the other in the writing thread. Here the chunks of a record are cut, shuffled and
// compressed beforehand on a pool of host threads, with the same zlib call as HDF5's filter (compress2 at the
// variable's level). HDF5's deflate filter is replaced by one that takes the precomputed result when the chunk it is
// given has the same bytes (compared in full) and otherwise compresses as HDF5 does (H5Zdeflate.c, H5Z__filter_deflate
// of HDF5 1.10 to 1.14), so the file is the same either way; a chunk HDF5 pads differently only loses the speed-up.
// Only zlib runs on the pool threads; every HDF5 / NetCDF call stays in the thread that writes the file (the caller,
// or the output writer thread of wrf_io/output_queue.hpp).
// CPPWRF_NC_THREADS=N: pool size (default: the hardware threads, at most 64); 1 switches the precomputation off (the
// filter compresses each chunk itself); 0 leaves HDF5's own filter in place.
#include <cstddef>
#include <memory>
#include <vector>

namespace cppwrf::wrf_io::deflate {

// one record of a chunked, deflated variable in memory type = file type, row-major over len (Time excluded)
struct Record {
  const unsigned char* data = nullptr;
  std::vector<std::size_t> len, chunk;  // dimension lengths and chunk lengths, slowest first
  std::size_t elsize = 0;
  std::vector<unsigned char> fill;  // the fill value of the type (elsize bytes): HDF5 pads edge chunks with it
  bool shuffle = true;
  int level = 2;
};

// the chunks of the records of one file record (NcOut::Batch): waited for and dropped together, so that the records
// staged for one file while another file's (or the same file's previous record's) puts run keep their own results
struct Group;
std::shared_ptr<Group> new_group();

bool active();  // the filter is installed and the pool has more than one thread
void install();  // replaces HDF5's deflate filter (once; no-op without HDF5 or with an HDF5 version not known here)
void submit(const Record& r, Group& g);  // queues the compression of each chunk of r (r.data must live until wait(g))
void wait(Group& g);  // until every chunk queued in g is compressed
void clear(Group& g);  // drops g's results HDF5 did not take
long precomputed(const Group& g);  // chunks of g compressed on the pool
// the filter takes the precomputed chunks of g first in this scope of the calling thread (NcOut::flush, sync)
class UseGroup {
 public:
  explicit UseGroup(const Group* g);
  ~UseGroup();
  UseGroup(const UseGroup&) = delete;
  UseGroup& operator=(const UseGroup&) = delete;

 private:
  const Group* saved_;
};

struct Stats {
  long precomputed = 0, hits = 0, misses = 0;  // chunks: compressed on the pool, taken by the filter, compressed by it
  double miss_s = 0, wait_s = 0;  // filter's own compression; waiting for the pool
  double download_s = 0;  // device-to-host copies of the output fields (cases/wrf_history.cpp)
};
Stats stats();  // a snapshot of the process's counters
void add_download(double s);
void add_wait(double s);
bool timing();  // CPPWRF_IO_TIMING=1: per-record timings of the output files

}  // namespace cppwrf::wrf_io::deflate
