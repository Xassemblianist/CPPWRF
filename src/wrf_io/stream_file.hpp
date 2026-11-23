// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// A WRF-format output file of one I/O stream (wrfinput, wrfout, wrfrst): WRF's dimensions, variables,
// variable attributes and global attributes for the stream's variable list, records written in order.
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "wrf_io/global_atts.hpp"
#include "wrf_io/nc_out.hpp"
#include "wrf_io/wrf_schema.hpp"

namespace cppwrf::wrf_io {

// Defines the dimensions (in the order WRF defines them; an unnamed dimension reuses an existing one of
// the same length or becomes DIM<nnnn>, as external/io_netcdf does), Times and the variables. Fills in
// the dimension names of `vars`.
void define_stream_vars(NcOut& nc, std::vector<OutVar>& vars, int real_bytes);

// Values of one variable in NetCDF order without Time; false: the variable keeps WRF's initial value 0.
using StreamValueFn = std::function<bool(const OutVar&, std::vector<double>&)>;

class StreamFile {
 public:
  StreamFile(const std::string& path, std::vector<OutVar> vars, const WrfConfig& c, const DomainAtts& d,
             Stream s, bool netcdf4);
  // appends one record: the values are gathered here; the NetCDF calls (Times, the puts in order, nc_sync) run on
  // the output writer thread when it is on (wrf_io/output_queue.hpp), else here. The file stays open until the last
  // of its records is written and the StreamFile is gone.
  void write(const std::string& date, const StreamValueFn& value);
  std::size_t records() const { return rec_; }

 private:
  std::shared_ptr<NcOut> nc_;
  std::vector<OutVar> vars_;
  std::size_t rec_ = 0;
  std::string path_;
};

}  // namespace cppwrf::wrf_io
