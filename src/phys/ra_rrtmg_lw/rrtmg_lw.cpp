// SPDX-License-Identifier: GPL-3.0-or-later AND LicenseRef-AER-RRTMG
#include "phys/ra_rrtmg_lw/rrtmg_lw.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"

#include <algorithm>
#include <cmath>

namespace cppwrf::phys {

static constexpr rb PPROF[60] = {
    rb(1000.00), rb(855.47), rb(731.82), rb(626.05), rb(535.57), rb(458.16), rb(391.94), rb(335.29), rb(286.83), rb(245.38),
    rb(209.91), rb(179.57), rb(153.62), rb(131.41), rb(112.42), rb(96.17),  rb(82.27),  rb(70.38),  rb(60.21),  rb(51.51),
    rb(44.06),   rb(37.69),  rb(32.25),  rb(27.59),  rb(23.60),  rb(20.19),  rb(17.27),  rb(14.77),  rb(12.64),  rb(10.81),
    rb(9.25),    rb(7.91),   rb(6.77),   rb(5.79),   rb(4.95),   rb(4.24),   rb(3.63),   rb(3.10),   rb(2.65),   rb(2.27),
    rb(1.94),    rb(1.66),   rb(1.42),   rb(1.22),   rb(1.04),   rb(0.89),   rb(0.76),   rb(0.65),   rb(0.56),   rb(0.48),
    rb(0.41),    rb(0.35),   rb(0.30),   rb(0.26),   rb(0.22),   rb(0.19),   rb(0.16),   rb(0.14),   rb(0.12),   rb(0.10)};

static constexpr rb TPROF[60] = {
    rb(286.96), rb(281.07), rb(275.16), rb(268.11), rb(260.56), rb(253.02), rb(245.62), rb(238.41), rb(231.57), rb(225.91),
    rb(221.72), rb(217.79), rb(215.06), rb(212.74), rb(210.25), rb(210.16), rb(210.69), rb(212.14), rb(213.74), rb(215.37),
    rb(216.82), rb(217.94), rb(219.03), rb(220.18), rb(221.37), rb(222.64), rb(224.16), rb(225.88), rb(227.63), rb(229.51),
    rb(231.50), rb(233.73), rb(236.18), rb(238.78), rb(241.60), rb(244.44), rb(247.35), rb(250.33), rb(253.32), rb(256.30),
    rb(259.22), rb(262.12), rb(264.80), rb(266.50), rb(267.59), rb(268.44), rb(268.69), rb(267.76), rb(266.13), rb(263.96),
    rb(261.54), rb(258.93), rb(256.15), rb(253.23), rb(249.89), rb(246.67), rb(243.48), rb(240.25), rb(236.66), rb(233.86)};

static constexpr rb retab[95] = {
    rb(5.92779), rb(6.26422), rb(6.61973), rb(6.99539), rb(7.39234), rb(7.81177), rb(8.25496), rb(8.72323), rb(9.21800), rb(9.74075),
    rb(10.2930), rb(10.8765), rb(11.4929), rb(12.1440), rb(12.8317), rb(13.5581), rb(14.2319), rb(15.0351), rb(15.8799), rb(16.7674),
    rb(17.6986), rb(18.6744), rb(19.6955), rb(20.7623), rb(21.8757), rb(23.0364), rb(24.2452), rb(25.5034), rb(26.8125), rb(27.7895),
    rb(28.6450), rb(29.4167), rb(30.1088), rb(30.7306), rb(31.2943), rb(31.8151), rb(32.3077), rb(32.7870), rb(33.2657), rb(33.7540),
    rb(34.2601), rb(34.7892), rb(35.3442), rb(35.9255), rb(36.5316), rb(37.1602), rb(37.8078), rb(38.4720), rb(39.1508), rb(39.8442),
    rb(40.5552), rb(41.2912), rb(42.0635), rb(42.8876), rb(43.7863), rb(44.7853), rb(45.9170), rb(47.2165), rb(48.7221), rb(50.4710),
    rb(52.4980), rb(54.8315), rb(57.4898), rb(60.4785), rb(63.7898), rb(65.5604), rb(71.2885), rb(75.4113), rb(79.7368), rb(84.2351),
    rb(88.8833), rb(93.6658), rb(98.5739), rb(103.603), rb(108.752), rb(114.025), rb(119.424), rb(124.954), rb(130.630), rb(136.457),
    rb(142.446), rb(148.608), rb(154.956), rb(161.503), rb(168.262), rb(175.248), rb(182.473), rb(189.952), rb(197.699), rb(205.728),
    rb(214.055), rb(222.694), rb(231.661), rb(240.971), rb(250.639)};

namespace {
// inirad (module_ra_rrtmg_lw.F) climatology
static constexpr rb o3sum[31] = {
    rb(5.297e-8), rb(5.852e-8), rb(6.579e-8), rb(7.505e-8), rb(8.577e-8), rb(9.895e-8), rb(1.175e-7), rb(1.399e-7),
    rb(1.677e-7), rb(2.003e-7), rb(2.571e-7), rb(3.325e-7), rb(4.438e-7), rb(6.255e-7), rb(8.168e-7), rb(1.036e-6),
    rb(1.366e-6), rb(1.855e-6), rb(2.514e-6), rb(3.240e-6), rb(4.033e-6), rb(4.854e-6), rb(5.517e-6), rb(6.089e-6),
    rb(6.689e-6), rb(1.106e-5), rb(1.462e-5), rb(1.321e-5), rb(9.856e-6), rb(5.960e-6), rb(5.960e-6)};
static constexpr rb ppsum[31] = {
    rb(955.890), rb(850.532), rb(754.599), rb(667.742), rb(589.841), rb(519.421), rb(455.480), rb(398.085),
    rb(347.171), rb(301.735), rb(261.310), rb(225.360), rb(193.419), rb(165.490), rb(141.032), rb(120.125),
    rb(102.689), rb(87.829),  rb(75.123),  rb(64.306),  rb(55.086),  rb(47.209),  rb(40.535),  rb(34.795),
    rb(29.865),  rb(19.122),  rb(9.277),   rb(4.660),   rb(2.421),   rb(1.294),   rb(0.647)};
static constexpr rb o3win[31] = {
    rb(4.629e-8), rb(4.686e-8), rb(5.017e-8), rb(5.613e-8), rb(6.871e-8), rb(8.751e-8), rb(1.138e-7), rb(1.516e-7),
    rb(2.161e-7), rb(3.264e-7), rb(4.968e-7), rb(7.338e-7), rb(1.017e-6), rb(1.308e-6), rb(1.625e-6), rb(2.011e-6),
    rb(2.516e-6), rb(3.130e-6), rb(3.840e-6), rb(4.703e-6), rb(5.486e-6), rb(6.289e-6), rb(6.993e-6), rb(7.494e-6),
    rb(8.197e-6), rb(9.632e-6), rb(1.113e-5), rb(1.146e-5), rb(9.389e-6), rb(6.135e-6), rb(6.135e-6)};
static constexpr rb ppwin[31] = {
    rb(955.747), rb(841.783), rb(740.199), rb(649.538), rb(568.404), rb(495.815), rb(431.069), rb(373.464),
    rb(322.354), rb(277.190), rb(237.635), rb(203.433), rb(174.070), rb(148.949), rb(127.408), rb(108.915),
    rb(93.114),  rb(79.551),  rb(67.940),  rb(58.072),  rb(49.593),  rb(42.318),  rb(36.138),  rb(30.907),
    rb(26.362),  rb(16.423),  rb(7.583),   rb(3.620),   rb(1.807),   rb(0.938),   rb(0.469)};


}  // namespace

void rrtmg_lw_fill_consts(RrtmgLwConsts& k) {
  for (int i = 0; i < 60; ++i) { k.pprof[i] = PPROF[i]; k.tprof[i] = TPROF[i]; }
  for (int i = 0; i < 95; ++i) k.retab[i] = retab[i];
  // inirad: annual mean ozone (same arithmetic as WRF, once instead of every column)
  rb* o3ann = k.o3ann;
  o3ann[0] = rb(0.5) * (o3sum[0] + o3win[0]);
  for (int kk = 1; kk < 31; ++kk) {
    o3ann[kk] = o3win[kk - 1] + (o3win[kk] - o3win[kk - 1]) / (ppwin[kk] - ppwin[kk - 1]) * (ppsum[kk] - ppwin[kk - 1]);
  }
  for (int kk = 1; kk < 31; ++kk) {
    o3ann[kk] = rb(0.5) * (o3ann[kk] + o3sum[kk]);
  }
  rb* ppwrkh = k.ppwrkh;
  ppwrkh[0] = rb(1100.0);
  for (int kk = 1; kk < 31; ++kk) {
    ppwrkh[kk] = rb(0.5) * (ppsum[kk] + ppsum[kk - 1]);
  }
  ppwrkh[31] = rb(0.0);
}

const RrtmgLwConsts& rrtmg_lw_consts() {
  static const RrtmgLwConsts k = [] {
    RrtmgLwConsts c{};
    rrtmg_lw_fill_consts(c);
    return c;
  }();
  return k;
}

void rrtmg_default_gases(RrtmgLwConfig& c) {
  c.co2 = (rb(280.) + rb(90.) * libm::exp(rb(0.02) * (c.yr - 2000))) * rb(1.e-6);
  c.ch4 = rb(1774.e-9);
  c.n2o = rb(319.e-9);
  c.cfc11 = rb(0.251e-9);
  c.cfc12 = rb(0.538e-9);
}

RrtmgLwCallConfig rrtmg_lw_call_config(int num_levels, const RrtmgLwConfig& config, bool hrc, bool hri, bool hrs) {
  constexpr rb deltap = rb(4.0);  // mb
  constexpr rb grav = rb(9.8066);
  RrtmgLwCallConfig c{};
  c.num_levels = num_levels;
  // rrtmg_lwinit: NLAYERS = kme + nint(p_top*0.01/deltap) - 1 with kme = nz + 1 (Fortran NINT)
  c.nlayers = num_levels + int(std::lround(config.p_top * rb(0.01) / deltap));
  c.o3input = config.o3input;
  c.co2 = config.co2; c.n2o = config.n2o; c.ch4 = config.ch4; c.cfc11 = config.cfc11; c.cfc12 = config.cfc12;
  c.g = config.g;
  c.heatfac = grav * rb(86400.0) / (config.cp * rb(100.0));
  c.has_reqc = hrc; c.has_reqi = hri; c.has_reqs = hrs;
  c.icld = config.cldovrlp; c.idcor = config.idcor; c.icloud = config.icloud;
  c.juldat = int(config.julian);  // juldat = julian (INTEGER)
  return c;
}


// WRF rrtmg_lw (inatm, cldprmc, setcoef, taumol, rtrnmc) for one column whose McICA subcolumn
// cloud arrays are already in ws (cldfmc, taucmc). Layer arrays have
// nlayers entries, level arrays nlayers + 1 (index 0 = surface); pressures in mb, emis per band.
void rrtmg_lw_core(int nlayers, rb cp, const rb* play, const rb* plev, const rb* tlay,
                   const rb* tlev, rb tsk, const rb* h2ovmr, const rb* o3vmr, const rb* co2vmr,
                   const rb* ch4vmr, const rb* n2ovmr, const rb* o2vmr, const rb* cfc11vmr,
                   const rb* cfc12vmr, const rb* cfc22vmr, const rb* ccl4vmr, const rb* semiss,
                   const rb* rei, const rb* rel, const rb* res, rb* totuflux, rb* totdflux,
                   rb* htr, rb* totuclfl, rb* totdclfl, rb* htrc, RrtmgLwWorkspace& ws,
                   const RrtmgLwTables& tables, int inflag, int iceflag, int liqflag) {
  constexpr rb grav = rb(9.8066);
  LwLayout lay(nlayers);
  ensure_workspace(ws, lay);
  const LwViews v = lay.views(ws.d.data(), ws.i.data(), 0, 1);
  v.pz[0] = plev[0];
  v.tz[0] = tlev[0];
  for (int l = 0; l < nlayers; ++l) {
    v.pavel[l] = play[l];
    v.tavel[l] = tlay[l];
    v.pz[l + 1] = plev[l + 1];
    v.tz[l + 1] = tlev[l + 1];
    v.wkl[0][l] = h2ovmr[l]; v.wkl[1][l] = co2vmr[l]; v.wkl[2][l] = o3vmr[l]; v.wkl[3][l] = n2ovmr[l];
    v.wkl[4][l] = rb(0.0); v.wkl[5][l] = ch4vmr[l]; v.wkl[6][l] = o2vmr[l];
    v.wx[0][l] = ccl4vmr[l]; v.wx[1][l] = cfc11vmr[l]; v.wx[2][l] = cfc12vmr[l]; v.wx[3][l] = cfc22vmr[l];
    v.reicmc[l] = rei[l]; v.relqmc[l] = rel[l]; v.resnmc[l] = res[l];
    // McICA flags from ws; the subcolumn water paths are not used (the core check has no clouds)
    for (int g = 0; g < 140; ++g) { v.cldfmc[g][l] = ws.cldfmc[g][l]; v.taucmc[g][l] = ws.taucmc[g][l]; }
    v.ciwpth[l] = rb(0.0); v.clwpth[l] = rb(0.0); v.cswpth[l] = rb(0.0);
  }
  for (int nb = 0; nb < 16; ++nb) v.semiss[nb] = semiss[nb];
  v.iscal[1] = inflag; v.iscal[2] = iceflag; v.iscal[3] = liqflag;
  rrtmg_lw_icldlyr(v.cldfmc, v.icldlyr, 0, nlayers);
  int top_cld_plus1 = 0;  // iscal[4] as rrtmg_lw_prep sets it (0: a clear column)
  for (int l = 0; l < nlayers; ++l) if (v.icldlyr[l]) top_cld_plus1 = l + 1;
  v.iscal[4] = top_cld_plus1;
  rrtmg_lw_inatm(nlayers, v);
  int laytrop = 0;
  rrtmg_lw_setcoef(nlayers, 1, v.pavel, v.tavel, v.tz, tsk, v.semiss, v.coldry, v.wkl, v.wbrodl, laytrop, v.jp,
                   v.jt, v.jt1, v.planklay, v.planklev, v.plankbnd, v.colh2o, v.colco2, v.colo3, v.coln2o, v.colco,
                   v.colch4, v.colo2, v.colbrd, v.fac00, v.fac01, v.fac10, v.fac11, v.rat_h2oco2, v.rat_h2oco2_1,
                   v.rat_h2oo3, v.rat_h2oo3_1, v.rat_h2on2o, v.rat_h2on2o_1, v.rat_h2och4, v.rat_h2och4_1,
                   v.rat_n2oco2, v.rat_n2oco2_1, v.rat_o3co2, v.rat_o3co2_1, v.selffac, v.selffrac, v.indself,
                   v.forfac, v.forfrac, v.indfor, v.minorfrac, v.scaleminor, v.scaleminorn2, v.indminor, tables);
  v.iscal[0] = laytrop;
  rrtmg_lw_secdiff(v.dscal[0], v.secdiff);
  for (int l = 0; l < nlayers; ++l) rrtmg_lw_layer(nlayers, l, v, tables);
  for (int b = 0; b < 16; ++b) rrtmg_lw_bandstage(nlayers, b, v, tables);
  for (int lev = 0; lev <= nlayers; ++lev) rrtmg_lw_level(lev, v, tables);
  rrtmg_lw_heating(nlayers, v.pz, v.fnet, v.fnetc, v.htr, v.htrc, grav * rb(86400.0) / (cp * rb(100.0)));
  for (int lev = 0; lev <= nlayers; ++lev) {
    totuflux[lev] = v.totuflux[lev]; totdflux[lev] = v.totdflux[lev];
    totuclfl[lev] = v.totuclfl[lev]; totdclfl[lev] = v.totdclfl[lev];
    htr[lev] = v.htr[lev]; htrc[lev] = v.htrc[lev];
  }
}

void rrtmg_lw_column(
    int num_levels,
    const RrtmgLwConfig& config,
    const RrtmgLwTables& tables,
    rb tsk,
    rb emiss,
    rb xland,
    rb xice,
    rb snow,
    rb xlat,
    const rb* p8w,
    const rb* t8w,
    const rb* p3d,
    const rb* pi3d,
    const rb* dz8w,
    const rb* t3d,
    const rb* rho3d,
    const rb* qv3d,
    const rb* qc3d,
    const rb* qr3d,
    const rb* qi3d,
    const rb* qs3d,
    const rb* qg3d,
    const rb* cldfra3d,
    const rb* o33d,
    rb& glw,
    rb& olr,
    rb& lwcf,
    rb& lwupt,
    rb& lwuptc,
    rb& lwdnt,
    rb& lwdntc,
    rb& lwupb,
    rb& lwupbc,
    rb& lwdnb,
    rb& lwdnbc,
    rb* rthratenlw,
    rb* rthratenlwc,
    RrtmgLwWorkspace& ws,
    const rb* re_cloud,
    const rb* re_ice,
    const rb* re_snow,
    rb* o3_out
) {
  (void)qr3d; (void)qg3d; (void)xlat; (void)rho3d; (void)dz8w;
  const RrtmgLwCallConfig c = rrtmg_lw_call_config(num_levels, config, re_cloud, re_ice, re_snow);
  LwLayout lay(c.nlayers);
  ensure_workspace(ws, lay);
  const LwViews v = lay.views(ws.d.data(), ws.i.data(), 0, 1);
  LwColumnIn in{};
  in.tsk = tsk; in.emiss = emiss; in.xland = xland; in.xice = xice; in.snow = snow; in.xlat = xlat;
  in.p8w = sd_const(p8w); in.t8w = sd_const(t8w); in.p3d = sd_const(p3d); in.pi3d = sd_const(pi3d); in.dz8w = sd_const(dz8w); in.t3d = sd_const(t3d);
  in.rho3d = sd_const(rho3d); in.qv3d = sd_const(qv3d); in.qc3d = sd_const(qc3d); in.qi3d = sd_const(qi3d); in.qs3d = sd_const(qs3d);
  in.cldfra3d = sd_const(cldfra3d); in.o33d = sd_const(o33d ? o33d : qv3d);
  in.re_cloud = sd_const(re_cloud); in.re_ice = sd_const(re_ice); in.re_snow = sd_const(re_snow);
  in.has_qc = in.has_qi = in.has_qs = true;  // absent species are passed as zero arrays
  in.o3_out = SD{o3_out, 1};
  rrtmg_lw_prep(c, in, v, tables, rrtmg_lw_consts());
  for (int l = 0; l < c.nlayers; ++l) rrtmg_lw_layer(c.nlayers, l, v, tables);
  for (int b = 0; b < 16; ++b) rrtmg_lw_bandstage(c.nlayers, b, v, tables);
  for (int lev = 0; lev <= c.nlayers; ++lev) rrtmg_lw_level(lev, v, tables);
  const LwColumnOut o = rrtmg_lw_finish(c, v, sd_const(pi3d), SD{rthratenlw, 1}, SD{rthratenlwc, 1});
  glw = o.glw; olr = o.olr; lwcf = o.lwcf; lwupt = o.lwupt; lwuptc = o.lwuptc; lwdnt = o.lwdnt;
  lwdntc = o.lwdntc; lwupb = o.lwupb; lwupbc = o.lwupbc; lwdnb = o.lwdnb; lwdnbc = o.lwdnbc;
}

} // namespace cppwrf::phys
