// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// A bounded read of one NetCDF variable whose shape the caller states: the reader checks every return code, the
// variable's rank, its dimension names and extents before it allocates anything, and reads one record of a
// record-dimension ("Time") variable with nc_get_vara_double, so a file of another shape (a staggered or 3-D copy,
// more records than one, a variable of rank 2 where one dimension is expected) is an error, never a buffer overrun.

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <netcdf.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace cppwrf::io_netcdf {

struct DimSpec {
  std::string name;     // the dimension's name in the file ("Time": the record dimension, any length > rec)
  std::size_t len = 0;  // its extent (not checked for "Time")
};

// The record rec of variable var (dimensions exactly dims, in this order) of the open file ncid as doubles, X
// fastest. path names the file in the errors. present: when given, a missing variable sets it false and returns {}
// (otherwise it is an error).
inline std::vector<double> read_record(int ncid, const std::string& path, const std::string& var,
                                       const std::vector<DimSpec>& dims, std::size_t rec = 0,
                                       bool* present = nullptr) {
  auto fail = [&](const std::string& why) {
    return std::runtime_error(path + ": " + var + " " + why);
  };
  auto shape = [&]() {
    std::string s;
    for (const DimSpec& d : dims) s += (s.empty() ? "" : ", ") + d.name + (d.name == "Time" ? "" : "=" + std::to_string(d.len));
    return "(" + s + ")";
  };
  int vid = -1;
  const int rc = nc_inq_varid(ncid, var.c_str(), &vid);
  if (rc == NC_ENOTVAR && present) {
    *present = false;
    return {};
  }
  if (rc != NC_NOERR) throw fail(rc == NC_ENOTVAR ? "is missing" : std::string("cannot be found: ") + nc_strerror(rc));
  if (present) *present = true;
  int nd = -1;
  if (nc_inq_varndims(ncid, vid, &nd) != NC_NOERR) throw fail("has no readable rank");
  if (nd != int(dims.size()))
    throw fail("has rank " + std::to_string(nd) + ", expected " + std::to_string(dims.size()) + " " + shape());
  std::vector<int> dids(dims.size() + 1);  // + 1: never an empty buffer for nc_inq_vardimid
  if (nc_inq_vardimid(ncid, vid, dids.data()) != NC_NOERR) throw fail("has no readable dimensions");
  std::vector<std::size_t> st(dims.size(), 0), ct(dims.size(), 0);
  std::size_t n = 1;
  for (std::size_t d = 0; d < dims.size(); ++d) {
    char name[NC_MAX_NAME + 1] = {};
    std::size_t len = 0;
    if (nc_inq_dimname(ncid, dids[d], name) != NC_NOERR || nc_inq_dimlen(ncid, dids[d], &len) != NC_NOERR)
      throw fail("has an unreadable dimension");
    if (dims[d].name != name)
      throw fail("dimension " + std::to_string(d) + " is " + name + ", expected " + dims[d].name + " " + shape());
    if (dims[d].name == "Time") {
      if (len <= rec) throw fail("has " + std::to_string(len) + " records, record " + std::to_string(rec) + " is read");
      st[d] = rec;
      ct[d] = 1;
    } else {
      if (len != dims[d].len)
        throw fail("dimension " + dims[d].name + " has length " + std::to_string(len) + ", expected " +
                   std::to_string(dims[d].len) + " " + shape());
      ct[d] = len;
      n *= len;
    }
  }
  std::vector<double> out(n);
  if (n > 0 && nc_get_vara_double(ncid, vid, st.data(), ct.data(), out.data()) != NC_NOERR) throw fail("cannot be read");
  return out;
}

// a value for an error message (shortest of %.17g that reads back: 5, 0.1, 1e-05)
inline std::string value_str(double x) {
  char b[32];
  for (int p = 1; p <= 17; ++p) {
    std::snprintf(b, sizeof b, "%.*g", p, x);
    if (std::strtod(b, nullptr) == x) break;
  }
  return b;
}

}  // namespace cppwrf::io_netcdf
