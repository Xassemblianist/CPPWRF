// SPDX-License-Identifier: GPL-3.0-or-later AND LicenseRef-AER-RRTMG
#pragma once
// WRF rrtmg_lw rtrnmc (McICA radiative transfer), split into independent pieces so that a GPU can
// run every band and every level of a column in parallel with the same arithmetic as WRF:
//   rrtmg_lw_secdiff        per column: diffusivity angles (16 bands) from pwvcm
//   rrtmg_lw_icldlyr        per layer : cloudy layer flag (any McICA subcolumn cloudy)
//   rrtmg_lw_band           per band  : rtrnmc's g-point loop of one band (sweeps, band sums per level)
//   rrtmg_lw_accum          per level : the band sums in band order
//   rrtmg_lw_heating        per column: heating rates from the net fluxes
// Composed in order for one column they reproduce rtrnmc bit for bit.
#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include "phys/ra_rrtmg_common/strided.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_tables.hpp"
#include <algorithm>
#include <cmath>

namespace cppwrf::phys {

constexpr int RRTMG_LW_SWEEP_MAX = 128;  // layers of one column (stack arrays of rrtmg_lw_sweep)

CPPWRF_HD inline void rrtmg_lw_secdiff(rb pwvcm, SD secdiff) {
  const rb a0[16] = {rb(1.66), rb(1.55), rb(1.58), rb(1.66), rb(1.54), rb(1.454), rb(1.89), rb(1.33),
                         rb(1.668), rb(1.66), rb(1.66), rb(1.66), rb(1.66), rb(1.66), rb(1.66), rb(1.66)};
  const rb a1[16] = {rb(0.00), rb(0.25), rb(0.22), rb(0.00), rb(0.13), rb(0.446), -rb(0.10), rb(0.40),
                         -rb(0.006), rb(0.00), rb(0.00), rb(0.00), rb(0.00), rb(0.00), rb(0.00), rb(0.00)};
  const rb a2[16] = {rb(0.00), -rb(12.0), -rb(11.7), rb(0.00), -rb(0.72), -rb(0.243), rb(0.19), -rb(0.062),
                         rb(0.414), rb(0.00), rb(0.00), rb(0.00), rb(0.00), rb(0.00), rb(0.00), rb(0.00)};
  for (int ibnd = 0; ibnd < 16; ++ibnd) {
    if (ibnd == 0 || ibnd == 3 || ibnd >= 9) {
      secdiff[ibnd] = rb(1.66);
    } else {
      rb s = a0[ibnd] + a1[ibnd] * libm::exp(a2[ibnd] * pwvcm);
      if (s > rb(1.80)) s = rb(1.80);
      if (s < rb(1.50)) s = rb(1.50);
      secdiff[ibnd] = s;
    }
  }
}

// icldlyr for layers [lay0, lay1)
CPPWRF_FORCEINLINE void rrtmg_lw_icldlyr(CSD2 cldfmc, SI icldlyr, int lay0, int lay1) {
  for (int lay = lay0; lay < lay1; ++lay) {
    int c = 0;
    for (int ig = 0; ig < 140; ++ig)
      if (cldfmc[ig][lay] == rb(1.0)) c = 1;
    icldlyr[lay] = c;
  }
}

// One g-point igc (of band iband) of rtrnmc: its down and up sweeps. The radiances the sweeps add to the band sums
// go to `out` per level, in the order of the sweeps:
//   out.down(l, radld, radclrd, iclddn)  the bottom of layer l (rtrnmc's lev - 1), iclddn after layer l
//   out.up(lev, radlu, radclru, iclddn)  level lev = 0..nlayers, iclddn at the end of the down sweep
// CLEAR_COL: a column without cloudy layers (rtrnmc's iclddn stays 0; radclrd = radld, radclru = radlu).
// A2 (CSD2 or SD2): the g-point's arrays; each element of fracs, taut is read once, at its layer of the down sweep,
// cldfmc's at its layer of each sweep, so `out` may overwrite them after (the GPU batch's in-place store).
template <int MAX_LAY, bool CLEAR_COL, class A2, class Out>
CPPWRF_FORCEINLINE void rrtmg_lw_gpoint(int nlayers, int iband, int igc, CSD secdiff, rb reflect, rb pbnd,
                                        A2 cldfmc, A2 taucmc, CSD2 planklay, CSD2 planklev, A2 fracs, A2 taut,
                                        CSI icldlyr, const RrtmgLwTables& CPPWRF_RESTRICT tables, Out& out) {
  constexpr rb rec_6 = rb(0.166667);
  constexpr rb bpade = rb(1.0) / rb(0.278);
  constexpr rb tblint = rb(10000.0);
  const rb sd = secdiff[iband];

  if constexpr (CLEAR_COL) {
    rb atrans[MAX_LAY], bbugas[MAX_LAY];
    {
      rb radld = rb(0.0);
      rb plfrac0 = rb(0.0);
      for (int lev = nlayers; lev >= 1; --lev) {
        int lay = lev - 1;
        rb plfrac = fracs[lay][igc];
        rb blay = planklay[lay][iband];
        rb dplankup = planklev[lev][iband] - blay;
        rb dplankdn = planklev[lev - 1][iband] - blay;
        rb odepth = sd * taut[lay][igc];
        if (odepth < rb(0.0)) odepth = rb(0.0);
        rb bbd = rb(0.0);
        if (odepth <= rb(0.06)) {
          atrans[lay] = odepth - rb(0.5) * odepth * odepth;
          rb odepth_rec = rec_6 * odepth;
          bbd = plfrac * (blay + dplankdn * odepth_rec);
          bbugas[lay] = plfrac * (blay + dplankup * odepth_rec);
        } else {
          rb tblind = odepth / (bpade + odepth);
          int itr = static_cast<int>(tblint * tblind + rb(0.5));
          rb transc = tables.exp_tbl[itr];
          atrans[lay] = rb(1.0) - transc;
          rb tausfac = tables.tfn_tbl[itr];
          bbd = plfrac * (blay + tausfac * dplankdn);
          bbugas[lay] = plfrac * (blay + tausfac * dplankup);
        }
        radld = radld + (bbd - radld) * atrans[lay];
        if (lay == 0) plfrac0 = plfrac;
        out.down(lev - 1, radld, radld, 0);
      }

      rb rad0 = plfrac0 * pbnd;  // fracs[0][igc] * pbnd
      rb radlu = rad0 + reflect * radld;
      out.up(0, radlu, radlu, 0);

      for (int lev = 1; lev <= nlayers; ++lev) {
        int lay = lev - 1;
        radlu = radlu + (bbugas[lay] - radlu) * atrans[lay];
        out.up(lev, radlu, radlu, 0);
      }
    }
  } else {
    rb atrans[MAX_LAY], bbugas[MAX_LAY], bbutot_atot[MAX_LAY], efcl[MAX_LAY];
    {
      rb radld = rb(0.0);
      rb radclrd = rb(0.0);
      rb plfrac0 = rb(0.0);
      int iclddn = 0;

      for (int lev = nlayers; lev >= 1; --lev) {
        int lay = lev - 1;
        rb plfrac = fracs[lay][igc];
        rb blay = planklay[lay][iband];
        rb dplankup = planklev[lev][iband] - blay;
        rb dplankdn = planklev[lev - 1][iband] - blay;
        rb odepth = sd * taut[lay][igc];
        if (odepth < rb(0.0)) odepth = rb(0.0);

        rb bbd = rb(0.0);

        if (icldlyr[lay] == 1) {
          iclddn = 1;
          rb odcld = rb(0.0);
          rb efcl_lay = rb(0.0);
          const rb cf = cldfmc[igc][lay];
          if (cf == rb(1.0)) {
            odcld = secdiff[tables.ngb[igc] - 1] * taucmc[igc][lay];
            rb transcld = libm::exp(-odcld);
            rb abscld = rb(1.0) - transcld;
            efcl_lay = abscld * cf;
          }
          efcl[lay] = efcl_lay;
          rb odtot = odepth + odcld;
          rb atot_lay, bbutot_lay;
          if (odtot < rb(0.06)) {
            atrans[lay] = odepth - rb(0.5) * odepth * odepth;
            rb odepth_rec = rec_6 * odepth;
            rb gassrc = plfrac * (blay + dplankdn * odepth_rec) * atrans[lay];

            atot_lay = odtot - rb(0.5) * odtot * odtot;
            rb odtot_rec = rec_6 * odtot;
            rb bbdtot = plfrac * (blay + dplankdn * odtot_rec);
            bbd = plfrac * (blay + dplankdn * odepth_rec);

            radld = radld - radld * (atrans[lay] + efcl_lay * (rb(1.0) - atrans[lay])) +
                    gassrc + cf * (bbdtot * atot_lay - gassrc);

            bbugas[lay] = plfrac * (blay + dplankup * odepth_rec);
            bbutot_lay = plfrac * (blay + dplankup * odtot_rec);
          } else if (odepth <= rb(0.06)) {
            atrans[lay] = odepth - rb(0.5) * odepth * odepth;
            rb odepth_rec = rec_6 * odepth;
            rb gassrc = plfrac * (blay + dplankdn * odepth_rec) * atrans[lay];

            rb tblind = odtot / (bpade + odtot);
            int ittot = static_cast<int>(tblint * tblind + rb(0.5));
            rb tfactot = tables.tfn_tbl[ittot];
            rb bbdtot = plfrac * (blay + tfactot * dplankdn);
            bbd = plfrac * (blay + dplankdn * odepth_rec);
            atot_lay = rb(1.0) - tables.exp_tbl[ittot];

            radld = radld - radld * (atrans[lay] + efcl_lay * (rb(1.0) - atrans[lay])) +
                    gassrc + cf * (bbdtot * atot_lay - gassrc);

            bbugas[lay] = plfrac * (blay + dplankup * odepth_rec);
            bbutot_lay = plfrac * (blay + tfactot * dplankup);
          } else {
            rb tblind = odepth / (bpade + odepth);
            int itgas = static_cast<int>(tblint * tblind + rb(0.5));
            odepth = tables.tau_tbl[itgas];
            atrans[lay] = rb(1.0) - tables.exp_tbl[itgas];
            rb tfacgas = tables.tfn_tbl[itgas];
            rb gassrc = atrans[lay] * plfrac * (blay + tfacgas * dplankdn);

            odtot = odepth + odcld;
            tblind = odtot / (bpade + odtot);
            int ittot = static_cast<int>(tblint * tblind + rb(0.5));
            rb tfactot = tables.tfn_tbl[ittot];
            rb bbdtot = plfrac * (blay + tfactot * dplankdn);
            bbd = plfrac * (blay + tfacgas * dplankdn);
            atot_lay = rb(1.0) - tables.exp_tbl[ittot];

            radld = radld - radld * (atrans[lay] + efcl_lay * (rb(1.0) - atrans[lay])) +
                    gassrc + cf * (bbdtot * atot_lay - gassrc);

            bbugas[lay] = plfrac * (blay + tfacgas * dplankup);
            bbutot_lay = plfrac * (blay + tfactot * dplankup);
          }
          bbutot_atot[lay] = bbutot_lay * atot_lay;
        } else {
          if (odepth <= rb(0.06)) {
            atrans[lay] = odepth - rb(0.5) * odepth * odepth;
            rb odepth_rec = rec_6 * odepth;
            bbd = plfrac * (blay + dplankdn * odepth_rec);
            bbugas[lay] = plfrac * (blay + dplankup * odepth_rec);
          } else {
            rb tblind = odepth / (bpade + odepth);
            int itr = static_cast<int>(tblint * tblind + rb(0.5));
            rb transc = tables.exp_tbl[itr];
            atrans[lay] = rb(1.0) - transc;
            rb tausfac = tables.tfn_tbl[itr];
            bbd = plfrac * (blay + tausfac * dplankdn);
            bbugas[lay] = plfrac * (blay + tausfac * dplankup);
          }
          radld = radld + (bbd - radld) * atrans[lay];
        }

        if (iclddn == 1) radclrd = radclrd + (bbd - radclrd) * atrans[lay];
        else radclrd = radld;
        if (lay == 0) plfrac0 = plfrac;
        out.down(lev - 1, radld, radclrd, iclddn);
      }

      rb rad0 = plfrac0 * pbnd;  // fracs[0][igc] * pbnd
      rb radlu = rad0 + reflect * radld;
      rb radclru = rad0 + reflect * radclrd;
      out.up(0, radlu, radclru, iclddn);

      for (int lev = 1; lev <= nlayers; ++lev) {
        int lay = lev - 1;
        if (icldlyr[lay] == 1) {
          rb gassrc = bbugas[lay] * atrans[lay];
          radlu = radlu - radlu * (atrans[lay] + efcl[lay] * (rb(1.0) - atrans[lay])) +
                  gassrc + cldfmc[igc][lay] * (bbutot_atot[lay] - gassrc);
        } else {
          radlu = radlu + (bbugas[lay] - radlu) * atrans[lay];
        }

        if (iclddn == 1) radclru = radclru + (bbugas[lay] - radclru) * atrans[lay];
        else radclru = radlu;
        out.up(lev, radlu, radclru, iclddn);
      }
    }
  }
}

// rtrnmc's band sums of one column (urad, drad, clrurad, clrdrad of one band): what each g-point's sweeps add,
// and below the first cloud (iclddn = 0) clrdrad = drad, clrurad = urad (level 0: clrurad adds radclru)
struct LwBandAcc {
  SD urad, drad, clrurad, clrdrad;
  CPPWRF_FORCEINLINE void down(int l, rb radld, rb radclrd, int iclddn) const {
    drad[l] += radld;
    if (iclddn == 1) clrdrad[l] += radclrd;
    else clrdrad[l] = drad[l];
  }
  CPPWRF_FORCEINLINE void up(int lev, rb radlu, rb radclru, int iclddn) const {
    urad[lev] += radlu;
    if (lev == 0 || iclddn == 1) clrurad[lev] += radclru;
    else clrurad[lev] = urad[lev];
  }
};
// the clear column's sums: the clear-sky sums are copied at the end (rtrnmc's clrdrad = drad, clrurad = urad)
struct LwBandAccClear {
  SD urad, drad;
  CPPWRF_FORCEINLINE void down(int l, rb radld, rb, int) const { drad[l] += radld; }
  CPPWRF_FORCEINLINE void up(int lev, rb radlu, rb, int) const { urad[lev] += radlu; }
};

template <int MAX_LAY, bool CLEAR_COL>
CPPWRF_FORCEINLINE void rrtmg_lw_band_impl(int nlayers, int iband, CSD secdiff, CSD semiss, CSD2 cldfmc, CSD2 taucmc,
                                           CSD2 planklay, CSD2 planklev, CSD plankbnd, CSD2 fracs, CSD2 taut,
                                           CSI icldlyr, SD2 bu, SD2 bd, SD2 bcu, SD2 bcd,
                                           const RrtmgLwTables& CPPWRF_RESTRICT tables) {
  const SD urad = bu[iband], drad = bd[iband], clrurad = bcu[iband], clrdrad = bcd[iband];
  for (int lev = 0; lev <= nlayers; ++lev) {
    urad[lev] = rb(0.0); drad[lev] = rb(0.0);
    if constexpr (!CLEAR_COL) {
      clrurad[lev] = rb(0.0); clrdrad[lev] = rb(0.0);
    }
  }
  const rb reflect = rb(1.0) - semiss[iband];
  const rb pbnd = plankbnd[iband];
  const int igc0 = iband == 0 ? 0 : tables.ngs[iband - 1];
  const int igc1 = tables.ngs[iband];

  if constexpr (CLEAR_COL) {
    LwBandAccClear acc{urad, drad};
    for (int igc = igc0; igc < igc1; ++igc)
      rrtmg_lw_gpoint<MAX_LAY, true, CSD2>(nlayers, iband, igc, secdiff, reflect, pbnd, cldfmc, taucmc, planklay, planklev,
                                     fracs, taut, icldlyr, tables, acc);
    for (int lev = 0; lev <= nlayers; ++lev) {
      clrdrad[lev] = drad[lev];
      clrurad[lev] = urad[lev];
    }
  } else {
    LwBandAcc acc{urad, drad, clrurad, clrdrad};
    for (int igc = igc0; igc < igc1; ++igc)
      rrtmg_lw_gpoint<MAX_LAY, false, CSD2>(nlayers, iband, igc, secdiff, reflect, pbnd, cldfmc, taucmc, planklay,
                                      planklev, fracs, taut, icldlyr, tables, acc);
  }
}

// One band of rtrnmc's band loop (iband 0..15): its g-points in order, down and up sweeps, the
// band sums urad/drad/clrurad/clrdrad of every level (bu/bd/bcu/bcd [16][nlayers + 1]) with WRF's
// clear-sky assignments (clrdrad = drad, clrurad = urad below the first cloud).
template <int MAX_LAY = RRTMG_LW_SWEEP_MAX>
CPPWRF_FORCEINLINE void rrtmg_lw_band(int nlayers, int iband, CSD secdiff, CSD semiss, CSD2 cldfmc, CSD2 taucmc,
                                      CSD2 planklay, CSD2 planklev, CSD plankbnd, CSD2 fracs, CSD2 taut, CSI icldlyr,
                                      SD2 bu, SD2 bd, SD2 bcu, SD2 bcd, const RrtmgLwTables& CPPWRF_RESTRICT tables,
                                      bool clear_col = false) {
  const bool fast_clear = CPPWRF_WARP_ALL(clear_col);
  if (fast_clear)
    rrtmg_lw_band_impl<MAX_LAY, true>(nlayers, iband, secdiff, semiss, cldfmc, taucmc, planklay, planklev, plankbnd,
                                      fracs, taut, icldlyr, bu, bd, bcu, bcd, tables);
  else
    rrtmg_lw_band_impl<MAX_LAY, false>(nlayers, iband, secdiff, semiss, cldfmc, taucmc, planklay, planklev, plankbnd,
                                       fracs, taut, icldlyr, bu, bd, bcu, bcd, tables);
}

// Level lev (0..nlayers): rtrnmc's band sums in band order, scaled by fluxfac; fnet / fnetc.
// bu / bd / bcu / bcd: written by rrtmg_lw_band (the GPU batch adds the same sums in rrtmg_lw_bandsum_level)
CPPWRF_FORCEINLINE void rrtmg_lw_accum(int lev, RSD2 bu, RSD2 bd, RSD2 bcu, RSD2 bcd,
                                       const RrtmgLwTables& CPPWRF_RESTRICT tables, rb& totuflux, rb& totdflux,
                                       rb& totuclfl, rb& totdclfl, rb& fnet, rb& fnetc) {
  constexpr rb pi = rb(3.14159265358979323846);
  constexpr rb fluxfac = pi * rb(2.0e4);
  constexpr rb wtdiff = rb(0.5);
  rb tu = rb(0.0), td = rb(0.0), tuc = rb(0.0), tdc = rb(0.0);
  for (int iband = 0; iband < 16; ++iband) {
    const rb dw = tables.delwave[iband];
    rb uflux = bu[iband][lev] * wtdiff;
    rb dflux = bd[iband][lev] * wtdiff;
    tu += uflux * dw;
    td += dflux * dw;
    rb uclfl = bcu[iband][lev] * wtdiff;
    rb dclfl = bcd[iband][lev] * wtdiff;
    tuc += uclfl * dw;
    tdc += dclfl * dw;
  }
  tu *= fluxfac;
  td *= fluxfac;
  fnet = tu - td;
  tuc *= fluxfac;
  tdc *= fluxfac;
  fnetc = tuc - tdc;
  totuflux = tu; totdflux = td; totuclfl = tuc; totdclfl = tdc;
}

CPPWRF_FORCEINLINE void rrtmg_lw_heating(int nlayers, CSD pz, CSD fnet, CSD fnetc, SD htr, SD htrc, rb heatfac) {
  for (int lev = 1; lev <= nlayers; ++lev) {
    int l = lev - 1;
    htr[l] = heatfac * (fnet[l] - fnet[lev]) / (pz[l] - pz[lev]);
    htrc[l] = heatfac * (fnetc[l] - fnetc[lev]) / (pz[l] - pz[lev]);
  }
  htr[nlayers] = rb(0.0);
  htrc[nlayers] = rb(0.0);
}

}  // namespace cppwrf::phys
