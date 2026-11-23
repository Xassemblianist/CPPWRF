// SPDX-License-Identifier: GPL-3.0-or-later
#undef NDEBUG  // the checks are asserts: keep them in Release builds
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "io_netcdf/nc_read.hpp"
#include "io_netcdf/netcdf_common.hpp"
#include "io_netcdf/restart.hpp"

using namespace cppwrf::io_netcdf;

static void test_precision(DType dt) {
  bool is_dp = (dt == DType::F64);
  std::printf("Testing restart bitwise identity for %s...\n", is_dp ? "FP64" : "FP32");

  const int nx = 12, ny = 8, nz = 6;
  const size_t sz_u = (nx + 1) * ny * nz;
  const size_t sz_v = nx * (ny + 1) * nz;
  const size_t sz_w = nx * ny * (nz + 1);
  const size_t sz_m = nx * ny * nz;
  const size_t sz_mu = nx * ny;

  RestartMeta meta_out;
  meta_out.step = 300;
  meta_out.time_sec = 300.0;
  meta_out.nx = nx;
  meta_out.ny = ny;
  meta_out.nz = nz;
  meta_out.dx = 100.0;
  meta_out.commit = "0123abcd";
  meta_out.options = "dyn.khdif = 0\nphys.mp_physics = 1\n";
  meta_out.dy = 100.0;
  meta_out.dt = 1.0;
  meta_out.case_name = "straka";
  meta_out.scalars = {{"NSTEPS", 37.0}, {"DECLIN", -0.12969871234567891}, {"SOLCON", 1386.25}};
  meta_out.date_str = "0001-01-01_00:05:00";

  // Lambda to fill test pattern
  auto fill = [is_dp](void* buf, size_t n, double seed) {
    if (is_dp) {
      double* p = static_cast<double*>(buf);
      for (size_t i = 0; i < n; ++i) {
        p[i] = std::sin(seed + double(i) * 0.123456789) * 1.23456789e4 + std::cos(double(i)) * 1e-7;
      }
    } else {
      float* p = static_cast<float*>(buf);
      for (size_t i = 0; i < n; ++i) {
        p[i] = float(std::sin(seed + double(i) * 0.12345) * 1.234e4 + std::cos(double(i)) * 1e-4);
      }
    }
  };

  const size_t item_size = is_dp ? sizeof(double) : sizeof(float);

  // Allocate original state buffers
  std::vector<char> u1(sz_u * item_size), u2(sz_u * item_size);
  std::vector<char> v1(sz_v * item_size), v2(sz_v * item_size);
  std::vector<char> w1(sz_w * item_size), w2(sz_w * item_size);
  std::vector<char> t1(sz_m * item_size), t2(sz_m * item_size);
  std::vector<char> ph1(sz_w * item_size), ph2(sz_w * item_size);
  std::vector<char> mu1(sz_mu * item_size), mu2(sz_mu * item_size);
  std::vector<char> q1(sz_m * item_size), q2(sz_m * item_size);
  std::vector<char> tke1(sz_m * item_size), tke2(sz_m * item_size);

  fill(u1.data(), sz_u, 1.1); fill(u2.data(), sz_u, 1.2);
  fill(v1.data(), sz_v, 2.1); fill(v2.data(), sz_v, 2.2);
  fill(w1.data(), sz_w, 3.1); fill(w2.data(), sz_w, 3.2);
  fill(t1.data(), sz_m, 4.1); fill(t2.data(), sz_m, 4.2);
  fill(ph1.data(), sz_w, 5.1); fill(ph2.data(), sz_w, 5.2);
  fill(mu1.data(), sz_mu, 6.1); fill(mu2.data(), sz_mu, 6.2);
  fill(q1.data(), sz_m, 7.1); fill(q2.data(), sz_m, 7.2);
  fill(tke1.data(), sz_m, 8.1); fill(tke2.data(), sz_m, 8.2);

  std::vector<VarView> vars_out;
  vars_out.emplace_back("u_1", 'U', nx + 1, ny, nz, u1.data(), dt);
  vars_out.emplace_back("u_2", 'U', nx + 1, ny, nz, u2.data(), dt);
  vars_out.emplace_back("v_1", 'V', nx, ny + 1, nz, v1.data(), dt);
  vars_out.emplace_back("v_2", 'V', nx, ny + 1, nz, v2.data(), dt);
  vars_out.emplace_back("w_1", 'W', nx, ny, nz + 1, w1.data(), dt);
  vars_out.emplace_back("w_2", 'W', nx, ny, nz + 1, w2.data(), dt);
  vars_out.emplace_back("t_1", 'M', nx, ny, nz, t1.data(), dt);
  vars_out.emplace_back("t_2", 'M', nx, ny, nz, t2.data(), dt);
  vars_out.emplace_back("ph_1", 'W', nx, ny, nz + 1, ph1.data(), dt);
  vars_out.emplace_back("ph_2", 'W', nx, ny, nz + 1, ph2.data(), dt);
  vars_out.emplace_back("mu_1", 'M', nx, ny, 1, mu1.data(), dt);
  vars_out.emplace_back("mu_2", 'M', nx, ny, 1, mu2.data(), dt);
  vars_out.emplace_back("q1_qv", 'M', nx, ny, nz, q1.data(), dt);
  vars_out.emplace_back("q2_qv", 'M', nx, ny, nz, q2.data(), dt);
  vars_out.emplace_back("q1_tke", 'M', nx, ny, nz, tke1.data(), dt);
  vars_out.emplace_back("q2_tke", 'M', nx, ny, nz, tke2.data(), dt);

  std::string filename = is_dp ? "tests/io_netcdf/test_out/restart_dp.nc" : "tests/io_netcdf/test_out/restart_sp.nc";
  write_restart(filename, meta_out, vars_out);

  // Allocate fresh read buffers zeroed out
  std::vector<char> ru1(sz_u * item_size, 0), ru2(sz_u * item_size, 0);
  std::vector<char> rv1(sz_v * item_size, 0), rv2(sz_v * item_size, 0);
  std::vector<char> rw1(sz_w * item_size, 0), rw2(sz_w * item_size, 0);
  std::vector<char> rt1(sz_m * item_size, 0), rt2(sz_m * item_size, 0);
  std::vector<char> rph1(sz_w * item_size, 0), rph2(sz_w * item_size, 0);
  std::vector<char> rmu1(sz_mu * item_size, 0), rmu2(sz_mu * item_size, 0);
  std::vector<char> rq1(sz_m * item_size, 0), rq2(sz_m * item_size, 0);
  std::vector<char> rtke1(sz_m * item_size, 0), rtke2(sz_m * item_size, 0);

  std::vector<VarView> vars_in;
  vars_in.emplace_back("u_1", 'U', nx + 1, ny, nz, ru1.data(), dt);
  vars_in.emplace_back("u_2", 'U', nx + 1, ny, nz, ru2.data(), dt);
  vars_in.emplace_back("v_1", 'V', nx, ny + 1, nz, rv1.data(), dt);
  vars_in.emplace_back("v_2", 'V', nx, ny + 1, nz, rv2.data(), dt);
  vars_in.emplace_back("w_1", 'W', nx, ny, nz + 1, rw1.data(), dt);
  vars_in.emplace_back("w_2", 'W', nx, ny, nz + 1, rw2.data(), dt);
  vars_in.emplace_back("t_1", 'M', nx, ny, nz, rt1.data(), dt);
  vars_in.emplace_back("t_2", 'M', nx, ny, nz, rt2.data(), dt);
  vars_in.emplace_back("ph_1", 'W', nx, ny, nz + 1, rph1.data(), dt);
  vars_in.emplace_back("ph_2", 'W', nx, ny, nz + 1, rph2.data(), dt);
  vars_in.emplace_back("mu_1", 'M', nx, ny, 1, rmu1.data(), dt);
  vars_in.emplace_back("mu_2", 'M', nx, ny, 1, rmu2.data(), dt);
  vars_in.emplace_back("q1_qv", 'M', nx, ny, nz, rq1.data(), dt);
  vars_in.emplace_back("q2_qv", 'M', nx, ny, nz, rq2.data(), dt);
  vars_in.emplace_back("q1_tke", 'M', nx, ny, nz, rtke1.data(), dt);
  vars_in.emplace_back("q2_tke", 'M', nx, ny, nz, rtke2.data(), dt);

  RestartMeta meta_in;
  read_restart(filename, meta_in, vars_in);

  // Validate metadata
  assert(meta_in.step == meta_out.step);
  assert(std::fabs(meta_in.time_sec - meta_out.time_sec) < 1e-12);
  assert(meta_in.nx == meta_out.nx);
  assert(meta_in.ny == meta_out.ny);
  assert(meta_in.nz == meta_out.nz);
  assert(std::fabs(meta_in.dx - meta_out.dx) < 1e-12);
  assert(std::fabs(meta_in.dy - meta_out.dy) < 1e-12);
  assert(std::fabs(meta_in.dt - meta_out.dt) < 1e-12);
  assert(meta_in.case_name == meta_out.case_name);
  assert(meta_in.date_str == meta_out.date_str);
  // the format version, commit and options (read_restart_meta reads them alone)
  assert(meta_in.version == kRestartVersion && meta_in.commit == meta_out.commit && meta_in.options == meta_out.options);
  const RestartMeta head = read_restart_meta(filename);
  assert(head.version == kRestartVersion && head.commit == meta_out.commit && head.options == meta_out.options);
  assert(head.step == meta_out.step && head.nx == nx);
  // the scalars (SCALAR_* attributes), bit for bit
  assert(meta_in.scalars.size() == 3 && meta_in.scalar("DECLIN", 0.0) == -0.12969871234567891 &&
         meta_in.scalar("NSTEPS", 0.0) == 37.0 && meta_in.scalar("SOLCON", 0.0) == 1386.25 &&
         meta_in.scalar("NYEAR", -1.0) == -1.0);

  // Validate bitwise exact identity using memcmp
  assert(std::memcmp(u1.data(), ru1.data(), u1.size()) == 0);
  assert(std::memcmp(u2.data(), ru2.data(), u2.size()) == 0);
  assert(std::memcmp(v1.data(), rv1.data(), v1.size()) == 0);
  assert(std::memcmp(v2.data(), rv2.data(), v2.size()) == 0);
  assert(std::memcmp(w1.data(), rw1.data(), w1.size()) == 0);
  assert(std::memcmp(w2.data(), rw2.data(), w2.size()) == 0);
  assert(std::memcmp(t1.data(), rt1.data(), t1.size()) == 0);
  assert(std::memcmp(t2.data(), rt2.data(), t2.size()) == 0);
  assert(std::memcmp(ph1.data(), rph1.data(), ph1.size()) == 0);
  assert(std::memcmp(ph2.data(), rph2.data(), ph2.size()) == 0);
  assert(std::memcmp(mu1.data(), rmu1.data(), mu1.size()) == 0);
  assert(std::memcmp(mu2.data(), rmu2.data(), mu2.size()) == 0);
  assert(std::memcmp(q1.data(), rq1.data(), q1.size()) == 0);
  assert(std::memcmp(q2.data(), rq2.data(), q2.size()) == 0);
  assert(std::memcmp(tke1.data(), rtke1.data(), tke1.size()) == 0);
  assert(std::memcmp(tke2.data(), rtke2.data(), tke2.size()) == 0);

  std::printf("PASS: Bitwise identity verified for %s (memcmp == 0)\n", is_dp ? "FP64" : "FP32");
}

// read_record (the bounded read of HALO_<name> in a CPPWRF restart file and of UOCE, VOCE, VAR, U_FRAME, V_FRAME in a
// wrfinput / wrfrst file): the shape is checked before anything is read, a mismatch is an error naming file and variable
static void test_bounded_read() {
  std::printf("Testing the bounded NetCDF read (read_record)...\n");
  std::filesystem::create_directories("tests/io_netcdf/test_out");
  const std::string f = "tests/io_netcdf/test_out/bounded_read.nc";
  const std::size_t nx = 5, ny = 4, nh = 12;
  {
    ScopedNcFile id;
    NC_CHECK(nc_create(f.c_str(), NC_CLOBBER | NC_64BIT_OFFSET, id.ptr()));
    int t, sn, we, wes, h, h2a, h2b, v;
    NC_CHECK(nc_def_dim(id, "Time", NC_UNLIMITED, &t));
    NC_CHECK(nc_def_dim(id, "south_north", ny, &sn));
    NC_CHECK(nc_def_dim(id, "west_east", nx, &we));
    NC_CHECK(nc_def_dim(id, "west_east_stag", nx + 1, &wes));
    NC_CHECK(nc_def_dim(id, ("halo_" + std::to_string(nh)).c_str(), nh, &h));
    NC_CHECK(nc_def_dim(id, "halo_a", 3, &h2a));
    NC_CHECK(nc_def_dim(id, "halo_b", 4, &h2b));
    const int d_h[1] = {h}, d_h2[2] = {h2a, h2b}, d_t[1] = {t}, d_m[3] = {t, sn, we}, d_u[3] = {t, sn, wes};
    NC_CHECK(nc_def_var(id, "HALO_mu_2", NC_DOUBLE, 1, d_h, &v));       // the writer's shape
    NC_CHECK(nc_def_var(id, "HALO_rank2", NC_DOUBLE, 2, d_h2, &v));     // a repacked rank-2 copy (3 x 4 = 12 values)
    NC_CHECK(nc_def_var(id, "U_FRAME", NC_DOUBLE, 1, d_t, &v));         // two records
    NC_CHECK(nc_def_var(id, "UOCE", NC_DOUBLE, 3, d_m, &v));            // mass grid
    NC_CHECK(nc_def_var(id, "UOCE_STAG", NC_DOUBLE, 3, d_u, &v));       // staggered in X
    NC_CHECK(nc_enddef(id));
    std::vector<double> a(nh), b(2 * ny * (nx + 1));
    for (std::size_t n = 0; n < a.size(); ++n) a[n] = double(n) + 0.5;
    for (std::size_t n = 0; n < b.size(); ++n) b[n] = 100.0 + double(n);
    int vid;
    NC_CHECK(nc_inq_varid(id, "HALO_mu_2", &vid));
    NC_CHECK(nc_put_var_double(id, vid, a.data()));
    NC_CHECK(nc_inq_varid(id, "HALO_rank2", &vid));
    NC_CHECK(nc_put_var_double(id, vid, a.data()));
    const double fr[2] = {0.0, 5.0};
    const std::size_t st1[1] = {0}, ct1[1] = {2};
    NC_CHECK(nc_inq_varid(id, "U_FRAME", &vid));
    NC_CHECK(nc_put_vara_double(id, vid, st1, ct1, fr));
    const std::size_t st3[3] = {0, 0, 0}, cm[3] = {2, ny, nx}, cu[3] = {2, ny, nx + 1};
    NC_CHECK(nc_inq_varid(id, "UOCE", &vid));
    NC_CHECK(nc_put_vara_double(id, vid, st3, cm, b.data()));
    NC_CHECK(nc_inq_varid(id, "UOCE_STAG", &vid));
    NC_CHECK(nc_put_vara_double(id, vid, st3, cu, b.data()));
  }
  ScopedNcFile id;
  NC_CHECK(nc_open(f.c_str(), NC_NOWRITE, id.ptr()));
  auto rejects = [&](const char* var, const std::vector<DimSpec>& dims, const char* what) {
    try {
      (void)read_record(id, f, var, dims);
    } catch (const std::runtime_error& e) {
      const std::string m = e.what();
      assert(m.find(f) != std::string::npos && m.find(var) != std::string::npos && m.find(what) != std::string::npos);
      std::printf("  rejected %s: %s\n", var, m.c_str());
      return;
    }
    std::printf("FAIL: %s was read\n", var);
    assert(false);
  };
  const std::vector<DimSpec> halo = {{"halo_" + std::to_string(nh), nh}};
  const std::vector<double> h = read_record(id, f, "HALO_mu_2", halo);
  assert(h.size() == nh && h[0] == 0.5 && h[nh - 1] == double(nh - 1) + 0.5);
  rejects("HALO_rank2", halo, "has rank 2, expected 1");                              // a rank-2 HALO_ variable
  rejects("HALO_mu_2", {{"halo_" + std::to_string(nh), nh + 1}}, "has length 12, expected 13");  // another count
  rejects("HALO_mu_2", {{"halo_13", 13}}, "is halo_12, expected halo_13");
  rejects("HALO_missing", halo, "is missing");
  bool present = true;
  assert(read_record(id, f, "HALO_missing", halo, 0, &present).empty() && !present);
  // a frame value of two records: record 0 alone (no overrun of the one-value buffer); record 2 does not exist
  assert(read_record(id, f, "U_FRAME", {{"Time", 0}}).size() == 1 && read_record(id, f, "U_FRAME", {{"Time", 0}})[0] == 0.0);
  assert(read_record(id, f, "U_FRAME", {{"Time", 0}}, 1)[0] == 5.0);
  rejects("U_FRAME", {{"Time", 0}, {"south_north", ny}}, "has rank 1, expected 2");
  const std::vector<DimSpec> mass = {{"Time", 0}, {"south_north", ny}, {"west_east", nx}};
  const std::vector<double> u = read_record(id, f, "UOCE", mass);
  assert(u.size() == nx * ny && u[0] == 100.0 && u[nx * ny - 1] == 100.0 + double(nx * ny - 1));
  rejects("UOCE_STAG", mass, "is west_east_stag, expected west_east");  // a staggered copy of the currents
  std::printf("PASS: bounded read\n");
}

int main() {
  std::printf("Running test_restart acceptance test...\n");
  test_precision(DType::F32);
  test_precision(DType::F64);
  test_bounded_read();
  std::printf("RESTART BITWISE TEST PASSED!\n");
  return 0;
}
