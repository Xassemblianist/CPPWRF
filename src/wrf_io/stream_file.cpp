// SPDX-License-Identifier: GPL-3.0-or-later
#include "wrf_io/stream_file.hpp"

#include "wrf_io/nc_deflate.hpp"
#include "wrf_io/output_queue.hpp"

#include <chrono>
#include <cstdio>
#include <map>
#include <stdexcept>

namespace cppwrf::wrf_io {

void define_stream_vars(NcOut& nc, std::vector<OutVar>& vars, int real_bytes) {
  nc.dim("Time", 0);
  nc.dim("DateStrLen", 19);
  std::vector<std::pair<std::string, std::size_t>> defined = {{"Time", 0}, {"DateStrLen", 19}};
  for (OutVar& v : vars)
    for (int k = int(v.dimnames.size()) - 1; k >= 0; --k) {  // Fortran order of each variable
      std::string& n = v.dimnames[std::size_t(k)];
      const std::size_t len = v.dimlens[std::size_t(k)];
      if (n.empty()) {
        for (auto& [dn, dl] : defined)
          if (dl == len && dn != "Time") { n = dn; break; }
        if (n.empty()) {
          char b[16];
          std::snprintf(b, sizeof b, "DIM%04d", int(defined.size()));
          n = b;
        }
      }
      bool found = false;
      for (auto& [dn, dl] : defined) found = found || dn == n;
      if (!found) {
        nc.dim(n, len);
        defined.push_back({n, len});
      }
    }
  nc.var("Times", 'c', {"Time", "DateStrLen"}, {}, 0);
  const int ftr = real_bytes == 8 ? 105 : 104;
  for (const OutVar& v : vars) {
    std::vector<std::string> dims = {"Time"};
    for (const std::string& n : v.dimnames) dims.push_back(n);
    std::map<std::string, std::string> at = {{"MemoryOrder", v.memorder_att}, {"description", v.desc},
                                             {"units", v.units}, {"stagger", v.stagger}};
    if (!v.coordinates.empty()) at["coordinates"] = v.coordinates;
    const char t = v.type == 'l' ? 'i' : v.type;
    const int ft = t == 'f' ? 104 : t == 'd' ? 105 : t == 'i' ? 106 : ftr;
    nc.var(v.name, t, dims, at, ft);
  }
}

StreamFile::StreamFile(const std::string& path, std::vector<OutVar> vars, const WrfConfig& c, const DomainAtts& d,
                       Stream s, bool netcdf4)
    : nc_(std::make_shared<NcOut>(path, netcdf4)), vars_(std::move(vars)), path_(path) {
  define_stream_vars(*nc_, vars_, d.real_bytes);
  write_global_atts(*nc_, c, d, s);
  nc_->enddef();
}

void StreamFile::write(const std::string& date, const StreamValueFn& value) {
  using clk = std::chrono::steady_clock;
  auto secs = [](clk::time_point a, clk::time_point b) { return std::chrono::duration<double>(b - a).count(); };
  const bool timed = deflate::timing();
  const deflate::Stats s0 = timed ? deflate::stats() : deflate::Stats{};
  const auto t0 = clk::now();
  double t_value = 0;
  // the values of every variable first (their chunks compress on the pool meanwhile), then the puts in the same order
  std::shared_ptr<NcOut::Batch> b = nc_->new_batch();
  std::vector<double> data;
  for (const OutVar& v : vars_) {
    std::size_t n = 1;
    for (std::size_t l : v.dimlens) n *= l;
    data.assign(n, 0.0);
    const auto tv = clk::now();
    const bool given = value(v, data);
    t_value += secs(tv, clk::now());
    if (given && data.size() != n)
      throw std::runtime_error("wrf_io: " + v.name + " has " + std::to_string(data.size()) + " values, expected " +
                               std::to_string(n));
    if (v.type == 'i' || v.type == 'l') nc_->stage_int(*b, v.name, std::vector<int>(data.begin(), data.end()), rec_);
    else if (v.type != 'c') nc_->stage(*b, v.name, data, rec_);
  }
  const auto t1 = clk::now();
  const double download = timed ? deflate::stats().download_s - s0.download_s : 0.0;
  const std::size_t rec = rec_++;
  // the record's memory in the queue: its values and about as much in precompressed chunks (their input copies)
  const std::size_t bytes = 2 * NcOut::bytes(*b);
  std::shared_ptr<NcOut> nc = nc_;
  output_queue::submit(bytes, path_, [nc, b, date, rec, t0, t1, t_value, download, timed, secs, path = path_]() mutable {
    const deflate::Stats w0 = timed ? deflate::stats() : deflate::Stats{};
    const auto t2 = clk::now();
    const deflate::UseGroup use(b->group.get());  // the record's own precomputed chunks first (also at the sync)
    nc->put_text("Times", date, rec);
    nc->flush(*b);
    const auto t3 = clk::now();
    nc->sync();
    const auto t4 = clk::now();
    const long pre = deflate::precomputed(*b->group);
    b.reset();  // the chunks HDF5 did not take (after the sync: it compresses the chunks left in its cache there)
    if (timed) {
      const deflate::Stats w1 = deflate::stats();
      std::printf("cppwrf: io %s rec %zu: values %.3f s (device copies %.3f s), staging %.3f s, queued %.3f s, puts "
                  "%.3f s (pool wait %.3f s, filter's own compression %.3f s), sync %.3f s, total %.3f s; chunks "
                  "precompressed %ld, taken %ld, compressed by the filter %ld\n",
                  path.c_str(), rec, t_value, download, secs(t0, t1) - t_value, secs(t1, t2), secs(t2, t3),
                  w1.wait_s - w0.wait_s, w1.miss_s - w0.miss_s, secs(t3, t4), secs(t0, t4), pre, w1.hits - w0.hits,
                  w1.misses - w0.misses);
    }
  });
}

}  // namespace cppwrf::wrf_io
