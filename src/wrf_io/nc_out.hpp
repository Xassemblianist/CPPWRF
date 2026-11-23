// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Minimal NetCDF writer for WRF-format files (one Time record): dimensions, global attributes and
// variables with WRF's per-variable attributes.
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>


namespace cppwrf::wrf_io {

namespace deflate {
struct Group;
}

// Every NetCDF call of an NcOut takes the process's NetCDF lock (wrf_io/output_queue.hpp NcCall), so a file can be
// written by the output writer thread while the model's thread reads other files.
class NcOut {
 public:
  // netcdf4: WRF built with NetCDF-4 features (deflate level 2 and chunking); else 64-bit offset classic
  explicit NcOut(const std::string& path, bool netcdf4 = true);
  ~NcOut();
  NcOut(const NcOut&) = delete;
  NcOut& operator=(const NcOut&) = delete;

  void dim(const std::string& name, std::size_t len);  // len 0: unlimited
  void att(const std::string& name, const std::string& v);
  void att(const std::string& name, int v);
  void att_f(const std::string& name, float v);
  void att_d(const std::string& name, double v);
  // define a variable; dims slowest first, e.g. {"Time","bottom_top","south_north","west_east"}
  void var(const std::string& name, char type /* 'f','d','i','c' */, const std::vector<std::string>& dims,
           const std::map<std::string, std::string>& satts, int field_type);
  void enddef();
  void put(const std::string& name, const std::vector<double>& v, std::size_t rec = 0);  // converted to the type
  void put_int(const std::string& name, const std::vector<int>& v, std::size_t rec = 0);
  void sync();
  void put_text(const std::string& name, const std::string& s, std::size_t rec = 0);
  // put / put_int in two steps: stage converts the values to the variable's type and queues the compression of its
  // chunks (wrf_io/nc_deflate.hpp); flush writes the staged records in their order, as the same puts would
  void stage(const std::string& name, const std::vector<double>& v, std::size_t rec = 0);
  void stage_int(const std::string& name, const std::vector<int>& v, std::size_t rec = 0);
  void flush();

  // the staged records of one file record, owned by whoever writes them (the output writer thread): stage in one
  // thread, flush in another; the buffers go back to the file for its next record
  struct Batch;
  std::unique_ptr<Batch> new_batch();
  void stage(Batch& b, const std::string& name, const std::vector<double>& v, std::size_t rec = 0);
  void stage_int(Batch& b, const std::string& name, const std::vector<int>& v, std::size_t rec = 0);
  // the puts of b in their order (waits for b's chunks first); b's precomputed chunks stay until b is destroyed,
  // which must come after the next sync (HDF5 compresses the chunks left in its cache there)
  void flush(Batch& b);
  static std::size_t bytes(const Batch& b);  // of the staged values

 private:
  struct VarInfo {
    int id = -1;
    char type = 'f';
    std::vector<std::size_t> len, chunk;  // all dimensions (Time first, length 1); chunk empty: not deflated
    int level = 0;
    bool shuffle = false;
  };
  struct Staged {
    const VarInfo* v = nullptr;
    std::string name;
    char mem = 'f';  // 'f' float, 'd' double, 'i' int
    std::size_t rec = 0;
    std::vector<unsigned char> bytes;
  };
  void stage_bytes(Batch& b, const std::string& name, char mem, std::size_t n, std::size_t esize, std::size_t rec,
                   const std::function<void(unsigned char*)>& fill_in);
  int nc_ = -1;
  bool nc4_ = true;
  std::map<std::string, std::size_t> dimlen_;
  std::map<std::string, int> dims_;
  std::map<std::string, VarInfo> vars_;
  std::unique_ptr<Batch> batch_;  // stage / flush without a batch
  std::vector<std::unique_ptr<Batch>> flushed_;  // flushed, kept till the next sync
  std::mutex spare_m_;
  std::vector<std::vector<std::vector<unsigned char>>> spare_;  // the buffers of flushed batches, one set each
 public:
  struct Batch {
    ~Batch();
    std::vector<Staged> staged;
    std::vector<std::vector<unsigned char>> spare;  // buffers of an earlier record, by position
    std::shared_ptr<deflate::Group> group;
  };
};

}  // namespace cppwrf::wrf_io
