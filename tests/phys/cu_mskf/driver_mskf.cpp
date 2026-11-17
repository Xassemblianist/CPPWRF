// SPDX-License-Identifier: GPL-3.0-or-later
// The C++ multi-scale Kain-Fritsch (src/phys/cu_mskf, CPU) on the columns of generate_inputs.py; writes the outputs
// in driver_mskf.f90's order.   driver_mskf_cpu IN OUT
#include <cstdint>
#include <cstdio>
#include <vector>
#include "core/backend.hpp"
#include "core/column_work.hpp"
#include "core/field.hpp"
#include "phys/cu_kf/kf_tables.hpp"
#include "phys/cu_mskf/mskf.hpp"
using namespace cppwrf;

int main(int argc, char** argv) {
  if (argc < 3) return 2;
  FILE* f = std::fopen(argv[1], "rb");
  if (!f) return 1;
  int32_t n, nl, stepcu, ktau, trigger, iwarm, iqi, iqs, iqr;
  double dt, dx, cudt;
  bool ok = std::fread(&n, 4, 1, f) && std::fread(&nl, 4, 1, f) && std::fread(&dt, 8, 1, f) &&
            std::fread(&dx, 8, 1, f) && std::fread(&cudt, 8, 1, f) && std::fread(&stepcu, 4, 1, f) &&
            std::fread(&ktau, 4, 1, f) && std::fread(&trigger, 4, 1, f) && std::fread(&iwarm, 4, 1, f) &&
            std::fread(&iqi, 4, 1, f) && std::fread(&iqs, 4, 1, f) && std::fread(&iqr, 4, 1, f);
  if (!ok) return 1;
  const std::size_t n3 = std::size_t(n) * nl, n3w = std::size_t(n) * (nl + 1);
  auto rd = [&](std::vector<real>& v, std::size_t c) {
    std::vector<double> t(c);
    if (std::fread(t.data(), 8, c, f) != c) { std::fprintf(stderr, "short input\n"); std::exit(1); }
    v.assign(t.begin(), t.end());
  };
  std::vector<real> u, v, th, t, qv, dz8w, pcps, rho, pi, w0avg, w, raincv, pratec, nca, cutop, cubot, hfx, ust, pblh,
      zol, xland;
  for (auto* x : {&u, &v, &th, &t, &qv, &dz8w, &pcps, &rho, &pi, &w0avg}) rd(*x, n3);
  rd(w, n3w);
  for (auto* x : {&raincv, &pratec, &nca, &cutop, &cubot, &hfx, &ust, &pblh, &zol, &xland}) rd(*x, std::size_t(n));
  std::fclose(f);
  std::vector<real> rth(n3, 0), rqv(n3, 0), rqc(n3, 0), rqr(n3, 0), rqi(n3, 0), rqs(n3, 0), ru(n3, 0), rv(n3, 0),
      cdp(n3, 0), csh(n3, 0), qc(n3, 0), qi(n3, 0);
  phys::cu_kf::KFTables<real> tables;
  phys::cu_kf::init_kf_tables(tables);
  auto c3 = [&](std::vector<real>& x) { return View3<const real>{x.data(), int(n3), n}; };
  auto m3 = [&](std::vector<real>& x) { return View3<real>{x.data(), int(n3), n}; };
  auto c2 = [&](std::vector<real>& x) { return View2<const real>{x.data(), n}; };
  auto m2 = [&](std::vector<real>& x) { return View2<real>{x.data(), n}; };
  phys::cu_mskf::MSKFKernel<real> k{};
  k.nx = n; k.ny = 1; k.nz = nl;
  k.dt = real(dt); k.dx = real(dx); k.cudt = real(cudt); k.stepcu = stepcu; k.adapt_step_flag = false;
  k.ktau = ktau; k.trigger = trigger; k.warm_rain = iwarm != 0;
  k.flag_qi = iqi != 0; k.flag_qs = iqs != 0; k.flag_qr = iqr != 0;
  k.xlv0 = real(3.15e6); k.xlv1 = real(2370.0); k.xls0 = real(2.905e6); k.xls1 = real(259.532);
  k.cp = real(1004.5); k.r = real(287.0); k.g = real(9.81);
  k.ep1 = real(461.6 / 287.0 - 1.0); k.ep2 = real(287.0 / 461.6);
  k.svp1 = real(0.6112); k.svp2 = real(17.67); k.svp3 = real(29.65); k.svpt0 = real(273.15);
  k.tables = &tables;
  k.u = c3(u); k.v = c3(v); k.w = View3<const real>{w.data(), int(n3w), n}; k.th = c3(th); k.t = c3(t);
  k.qv = c3(qv); k.dz8w = c3(dz8w); k.pcps = c3(pcps); k.rho = c3(rho); k.pi = c3(pi);
  k.w0avg = m3(w0avg);
  k.raincv = m2(raincv); k.pratec = m2(pratec); k.nca = m2(nca); k.cutop = m2(cutop); k.cubot = m2(cubot);
  k.rthcuten = m3(rth); k.rqvcuten = m3(rqv); k.rqccuten = m3(rqc); k.rqrcuten = m3(rqr); k.rqicuten = m3(rqi);
  k.rqscuten = m3(rqs); k.rucuten = m3(ru); k.rvcuten = m3(rv);
  k.cldfra_dp_kf = m3(cdp); k.cldfra_sh_kf = m3(csh); k.qc_kf = m3(qc); k.qi_kf = m3(qi);
  k.zol = c2(zol); k.hfx = c2(hfx); k.ust = c2(ust); k.pblh = c2(pblh);
  const Range2D range{0, n, 0, 1};
  column_work_reserve(range, phys::cu_mskf::mskf_work_bytes<real>(nl));
  phys::cu_mskf::run_mskf(k, range);
  FILE* o = std::fopen(argv[2], "wb");
  auto wr = [&](const std::vector<real>& x) {
    std::vector<double> d(x.begin(), x.end());
    std::fwrite(d.data(), 8, d.size(), o);
  };
  for (auto* x : {&raincv, &pratec, &nca, &cutop, &cubot, &w0avg, &rth, &rqv, &rqc, &rqr, &rqi, &rqs, &ru, &rv, &cdp,
                  &csh, &qc, &qi})
    wr(*x);
  std::fclose(o);
  return 0;
}
