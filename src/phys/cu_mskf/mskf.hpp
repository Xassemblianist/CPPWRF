// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Multi-scale Kain-Fritsch cumulus (cu_physics = 11): WRF v4.8.0 phys/module_cu_mskf.F, MSKF_CPS,
// MSKF_eta_PARA and MSKF_CMT, with aercu_opt = 0 (no aerosol-aware microphysics, mskf_mphy not called) and
// kf_edrates = 0. MSKF_eta_PARA is KF_eta_PARA (module_cu_kfeta.F, phys/cu_kf) with the scale-aware changes:
// the grid-length factor Scale_Fac = 1 + ln(25 km / DX) below 24.999 km (conversion rate, Tokioka entrainment,
// convective time scale, the closure's minimum CAPE removal), the trigger perturbation without the 25 km grid
// scaling below 25 km, the updraft radius from the LCL and the source layer, the dynamic convective time scale
// from the sub-cloud velocity scale (ZOL, HFX, UST, PBLH), no convection where the cloud layer is saturated, cloud
// fractions over the updraft-free area, and the cumulus momentum transport MSKF_CMT (RUCUTEN, RVCUTEN). The lookup
// tables, TPMIX2, TPMIX2DD, DTFRZNEW, CONDLOAD, PROF5 and ENVIRTHT are KF-eta's (identical code).
#include "core/backend.hpp"
#include "core/column_work.hpp"
#include "core/field.hpp"
#include "phys/cu_kf/kf.hpp"
#include <cmath>

namespace cppwrf::phys::cu_mskf {

using cu_kf::kMaxLevels;
using cu_kf::KFTables;
using cu_kf::envirtht;
using cu_kf::tpmix2;
using cu_kf::tpmix2dd;
using cu_kf::dtfrznew;
using cu_kf::condload;
using cu_kf::prof5;

// the input column
#define CPPWRF_MSKF_COLUMN_IN(X) \
    X(T, u0, MaxLevels) X(T, v0, MaxLevels) X(T, t0, MaxLevels) X(T, qv0, MaxLevels) \
    X(T, p0, MaxLevels) X(T, dzq, MaxLevels)
// MSKF_eta_PARA's column arrays
#define CPPWRF_MSKF_COLUMN(X) \
    X(T, q0, MaxLevels) X(T, z0, MaxLevels) X(T, tv0, MaxLevels) X(T, tu, MaxLevels) \
    X(T, tvu, MaxLevels) X(T, qu, MaxLevels) X(T, tz, MaxLevels) X(T, tvd, MaxLevels) \
    X(T, qd, MaxLevels) X(T, qes, MaxLevels) X(T, tg, MaxLevels) X(T, tvg, MaxLevels) \
    X(T, qg, MaxLevels) X(T, wu, MaxLevels) X(T, wd, MaxLevels) X(T, ems, MaxLevels) \
    X(T, emsd, MaxLevels) X(T, umf, MaxLevels) X(T, uer, MaxLevels) X(T, udr, MaxLevels) \
    X(T, dmf, MaxLevels) X(T, der, MaxLevels) X(T, ddr, MaxLevels) X(T, umf2, MaxLevels) \
    X(T, uer2, MaxLevels) X(T, udr2, MaxLevels) X(T, dmf2, MaxLevels) X(T, der2, MaxLevels) \
    X(T, ddr2, MaxLevels) X(T, dza, MaxLevels) X(T, thta0, MaxLevels) X(T, thetee, MaxLevels) \
    X(T, thtau, MaxLevels) X(T, theteu, MaxLevels) X(T, thtad, MaxLevels) X(T, theted, MaxLevels) \
    X(T, qliq, MaxLevels) X(T, qice, MaxLevels) X(T, qlqout, MaxLevels) X(T, qicout, MaxLevels) \
    X(T, pptliq, MaxLevels) X(T, pptice, MaxLevels) X(T, detlq, MaxLevels) X(T, detic, MaxLevels) \
    X(T, detlq2, MaxLevels) X(T, detic2, MaxLevels) X(T, ratio2, MaxLevels) X(T, domgdp, MaxLevels) \
    X(T, tvqu, MaxLevels) X(T, dp, MaxLevels) X(T, rh, MaxLevels) X(T, eqfrc, MaxLevels) \
    X(T, wspd, MaxLevels) X(T, qdt, MaxLevels) X(T, fxm, MaxLevels) X(T, thtag, MaxLevels) \
    X(T, thpa, MaxLevels) X(T, thfxout, MaxLevels) X(T, thfxin, MaxLevels) X(T, qpa, MaxLevels) \
    X(T, qfxout, MaxLevels) X(T, qfxin, MaxLevels) X(T, ql0, MaxLevels) X(T, qlg, MaxLevels) \
    X(T, qi0, MaxLevels) X(T, qig, MaxLevels) X(T, qr0, MaxLevels) X(T, qrg, MaxLevels) \
    X(T, qs0, MaxLevels) X(T, qsg, MaxLevels) X(T, rainfb, MaxLevels) X(T, snowfb, MaxLevels) \
    X(T, cldhgt, MaxLevels) X(T, qsd, MaxLevels) X(T, dilfrc, MaxLevels) X(T, ddilfrc, MaxLevels) \
    X(T, tgu, MaxLevels) X(T, qgu, MaxLevels) X(T, thteeg, MaxLevels) X(T, omg, MaxLevels + 1) \
    X(int, kcheck, MaxLevels)
// the momentum transport: MSKF_eta_PARA's inputs of MSKF_CMT (index JK = KTE-KQ+1, 1-based, as the CAM column) and
// MSKF_CMT's work arrays (1-based J; ZF and the J+1 neighbours up to ILEV+1)
#define CPPWRF_MSKF_CMT(X) \
    X(T, zfw, MaxLevels + 2) X(T, cdp, MaxLevels + 2) X(T, ced, MaxLevels + 2) X(T, ceu, MaxLevels + 2) \
    X(T, cmc, MaxLevels + 2) X(T, cmd, MaxLevels + 2) X(T, cmu, MaxLevels + 2) X(T, cpru, MaxLevels + 2) \
    X(T, cqd, MaxLevels + 2) X(T, cqu, MaxLevels + 2) X(T, cqhat, MaxLevels + 2) X(T, csd, MaxLevels + 2) \
    X(T, csu, MaxLevels + 2) X(T, cshat, MaxLevels + 2) X(T, ct, MaxLevels + 2) X(T, cu, MaxLevels + 2) \
    X(T, cv, MaxLevels + 2) X(T, cz, MaxLevels + 2) X(T, czf, MaxLevels + 2) X(T, cdudt, MaxLevels + 2) \
    X(T, cdvdt, MaxLevels + 2) X(T, bu, MaxLevels + 2) X(T, bd, MaxLevels + 2) X(T, crho, MaxLevels + 2) \
    X(T, rhohat, MaxLevels + 2) X(T, dzf, MaxLevels + 2) X(T, uhat, MaxLevels + 2) X(T, vhat, MaxLevels + 2) \
    X(T, uc, MaxLevels + 2) X(T, vc, MaxLevels + 2) X(T, cdz, MaxLevels + 2) X(T, au, MaxLevels + 2) \
    X(T, ad, MaxLevels + 2) X(T, ac, MaxLevels + 2) X(T, cw0, MaxLevels + 2) X(T, acfl, MaxLevels + 2) \
    X(T, aufl, MaxLevels + 2) X(T, adfl, MaxLevels + 2) X(T, w0fl, MaxLevels + 2) X(T, d0, MaxLevels + 2) \
    X(T, ce, MaxLevels + 2) X(T, d0hat, MaxLevels + 2) X(T, ehat, MaxLevels + 2) X(T, w1, MaxLevels + 2) \
    X(T, b1, MaxLevels + 2) X(T, w1fl, MaxLevels + 2) X(T, delpx, MaxLevels + 2) \
    X(T, delpy, MaxLevels + 2) X(T, caa, MaxLevels + 2) X(T, cbb, MaxLevels + 2) X(T, ccc, MaxLevels + 2) \
    X(T, cdx, MaxLevels + 2) X(T, cdy, MaxLevels + 2) X(T, cfx, MaxLevels + 2) X(T, cfy, MaxLevels + 2)

// the bytes of one column's arrays (all in the column work buffer, core/column_work.hpp)
template <typename T, int MaxLevels>
inline constexpr std::size_t kColumnBytes = 0 CPPWRF_MSKF_COLUMN_IN(CPPWRF_COLUMN_BYTES)
    CPPWRF_MSKF_COLUMN(CPPWRF_COLUMN_BYTES) CPPWRF_MSKF_CMT(CPPWRF_COLUMN_BYTES);

template <typename T>
struct MSKFKernel {
    // Grid sizes
    int nx;
    int ny;
    int nz;

    // Scalars
    T dt;
    T dx;
    T cudt;
    int stepcu;
    bool adapt_step_flag;  // adaptive time step: W0AVG averages over 2*MAX(CUDT*60,DT) seconds
    int ktau;
    int trigger;
    bool warm_rain;
    bool flag_qi;
    bool flag_qs;
    bool flag_qr;

    // Physical constants
    T xlv0;
    T xlv1;
    T xls0;
    T xls1;
    T cp;
    T r;
    T g;
    T ep1;
    T ep2;
    T svp1;
    T svp2;
    T svp3;
    T svpt0;

    // Lookup tables
    const KFTables<T>* tables;

    // 3D Inputs (nx, ny, nz)
    View3<const T> u;
    View3<const T> v;
    View3<const T> w; // (nx, ny, nz+1)
    View3<const T> th;
    View3<const T> t;
    View3<const T> qv;
    View3<const T> dz8w;
    View3<const T> pcps;
    View3<const T> rho;
    View3<const T> pi;

    // 3D Inout
    View3<T> w0avg;

    // 2D Inout
    View2<T> raincv;
    View2<T> pratec;
    View2<T> nca;

    // 2D Out
    View2<T> cutop;
    View2<T> cubot;

    // 3D Tendency Outputs
    View3<T> rthcuten;
    View3<T> rqvcuten;
    View3<T> rqccuten;
    View3<T> rqrcuten;
    View3<T> rqicuten;
    View3<T> rqscuten;

    // momentum tendencies (MSKF_CMT; cmt_opt_flag is .TRUE. in MSKF_CPS)
    View3<T> rucuten;
    View3<T> rvcuten;

    // Diagnostics
    View3<T> cldfra_dp_kf;
    View3<T> cldfra_sh_kf;
    View3<T> qc_kf;
    View3<T> qi_kf;

    // the surface layer's and the PBL's fields of this step (the dynamic convective time scale)
    View2<const T> zol;
    View2<const T> hfx;
    View2<const T> ust;
    View2<const T> pblh;

    // the column arrays (column_for_work through MSKFRunner)
    ColumnWork work{};

    
    template <int MaxLevels = kMaxLevels>
    CPPWRF_HD void compute(int i, int j) const {
        using std::sqrt;
        using std::exp;
        using std::log;
        using std::pow;
        using std::abs;

        const int kx = nz;
        const int kl = nz;
        const T dxsq = dx * dx;

        // 1. Running mean vertical velocity W0AVG
        const T ntst = static_cast<T>(stepcu);
        const T tst = ntst * static_cast<T>(2.0);
        T w0avgfctr, w0fctr, w0den;
        if (adapt_step_flag) {  // module_cu_mskf.F MSKF_CPS (W0AVG over 2*MAX(CUDT*60,DT) seconds)
            const T m = cudt * static_cast<T>(60) > dt ? cudt * static_cast<T>(60) : dt;
            w0avgfctr = static_cast<T>(2) * m - dt;
            w0fctr = dt;
            w0den = static_cast<T>(2) * m;
        } else {
            w0avgfctr = tst - static_cast<T>(1.0);
            w0fctr = static_cast<T>(1.0);
            w0den = tst;
        }

        for (int k = 0; k < kx; ++k) {
            T w0 = static_cast<T>(0.5) * (w(i, j, k) + w(i, j, k + 1));
            w0avg(i, j, k) = (w0avg(i, j, k) * w0avgfctr + w0 * w0fctr) / w0den;
        }

        // 2. Check NCA relaxation condition
        if (nca(i, j) >= static_cast<T>(0.5) * dt) {
            // Convection active from prior steps, skip new initiation
            return;
        }

        // Initialize tendencies and outputs (MSKF_CPS: DUDT = DVDT = 0, then RUCUTEN = DUDT after MSKF_eta_PARA)
        for (int k = 0; k < kx; ++k) {
            rucuten(i, j, k) = static_cast<T>(0.0);
            rvcuten(i, j, k) = static_cast<T>(0.0);
            rthcuten(i, j, k) = static_cast<T>(0.0);
            rqvcuten(i, j, k) = static_cast<T>(0.0);
            rqccuten(i, j, k) = static_cast<T>(0.0);
            rqrcuten(i, j, k) = static_cast<T>(0.0);
            if (flag_qi) rqicuten(i, j, k) = static_cast<T>(0.0);
            if (flag_qs) rqscuten(i, j, k) = static_cast<T>(0.0);
            cldfra_dp_kf(i, j, k) = static_cast<T>(0.0);
            cldfra_sh_kf(i, j, k) = static_cast<T>(0.0);
            qc_kf(i, j, k) = static_cast<T>(0.0);
            qi_kf(i, j, k) = static_cast<T>(0.0);
        }
        raincv(i, j) = static_cast<T>(0.0);
        pratec(i, j) = static_cast<T>(0.0);
        cutop(i, j) = static_cast<T>(1.0);          // KTS = 1
        cubot(i, j) = static_cast<T>(kx + 1);       // KTE + 1

        // Local column scratch arrays
        CPPWRF_COLUMN_SLOTS(work, i, j)
        CPPWRF_MSKF_COLUMN_IN(CPPWRF_COLUMN_ARRAY)

        for (int k = 0; k < kx; ++k) {
            u0[k] = u(i, j, k);
            v0[k] = v(i, j, k);
            t0[k] = t(i, j, k);
            qv0[k] = qv(i, j, k);
            p0[k] = pcps(i, j, k);
            dzq[k] = dz8w(i, j, k);
        }

        // --- MSKF_eta_PARA ---
        const T p00 = static_cast<T>(1.0e5);
        const T t00 = static_cast<T>(273.16);
        const T rlf = static_cast<T>(3.339e5);
        const T ttfrz = static_cast<T>(268.16);
        const T tbfrz = static_cast<T>(248.16);
        const T rate = static_cast<T>(0.03);

        const T gdry = -g / cp;
        const T aliq = svp1 * static_cast<T>(1000.0);
        const T bliq = svp2;
        const T cliq = svp2 * svpt0;
        const T dliq = svp3;

        T fbfrc = static_cast<T>(0.0);
        int ishall = 0;
        // the grid-length factor (scalar log; DX >= 24.999 km: none)
        T scale_fac, capedx;
        if (dx >= static_cast<T>(24.999e3)) {
            scale_fac = static_cast<T>(1.0);
            capedx = static_cast<T>(0.1);
        } else {
            scale_fac = static_cast<T>(1.0) + (libm::log(static_cast<T>(25.e3) / dx));
            capedx = static_cast<T>(0.1) * sqrt(scale_fac);
        }
        const T dpmin = static_cast<T>(5.0e3);
        const T p300 = p0[0] - static_cast<T>(30000.0);

        CPPWRF_MSKF_COLUMN(CPPWRF_COLUMN_ARRAY)

        int l5 = 0;
        int llfc = 0;
        for (int k = 0; k < kx; ++k) {
            T es = aliq * vm::exp((bliq * t0[k] - cliq) / (t0[k] - dliq), k, kx);  // vectorised in WRF (libmvec)
            qes[k] = static_cast<T>(0.622) * es / (p0[k] - es);
            q0[k] = (qes[k] < qv0[k]) ? qes[k] : qv0[k];
            if (q0[k] < static_cast<T>(0.000001)) q0[k] = static_cast<T>(0.000001);
            ql0[k] = static_cast<T>(0.0);
            qi0[k] = static_cast<T>(0.0);
            qr0[k] = static_cast<T>(0.0);
            qs0[k] = static_cast<T>(0.0);
            rh[k] = q0[k] / qes[k];
            dilfrc[k] = static_cast<T>(1.0);
            tv0[k] = t0[k] * (static_cast<T>(1.0) + static_cast<T>(0.608) * q0[k]);
            dp[k] = rho(i, j, k) * g * dzq[k];
            cldhgt[k] = static_cast<T>(0.0);
            if (p0[k] >= static_cast<T>(0.5) * p0[0]) l5 = k;
            if (p0[k] >= p300) llfc = k;
        }

        z0[0] = static_cast<T>(0.5) * dzq[0];
        for (int k = 1; k < kl; ++k) {
            z0[k] = z0[k - 1] + static_cast<T>(0.5) * (dzq[k] + dzq[k - 1]);
            dza[k - 1] = z0[k] - z0[k - 1];
        }
        dza[kl - 1] = static_cast<T>(0.0);

        int ncheck = 1;
        kcheck[0] = 0;
        T pm15 = p0[0] - static_cast<T>(15.0e2);
        for (int k = 1; k <= llfc; ++k) {
            if (p0[k] < pm15) {
                kcheck[ncheck] = k;
                ncheck++;
                pm15 -= static_cast<T>(15.0e2);
            }
        }

        int nu = -1;
        int nuchm = -1;
        int lc = 0;
        int kpbl = 0;
        int klcl = 0;
        int ltop = 0;
        int let = 0;
        int lcl = 0;
        T zlcl = static_cast<T>(0.0);
        T tlcl = static_cast<T>(0.0);
        T tvlcl = static_cast<T>(0.0);
        T plcl = static_cast<T>(0.0);
        T wlcl = static_cast<T>(0.0);
        T wkl = static_cast<T>(0.0);
        T dpthmx = static_cast<T>(0.0);
        T vmflcl = static_cast<T>(0.0);
        T upold = static_cast<T>(0.0);
        T upnew = static_cast<T>(0.0);
        T trppt = static_cast<T>(0.0);
        T au0 = static_cast<T>(0.0);
        T tmix = static_cast<T>(0.0);
        T qmix = static_cast<T>(0.0);
        T zmix = static_cast<T>(0.0);
        T pmix = static_cast<T>(0.0);
        T abe = static_cast<T>(0.0);

        T chmin = static_cast<T>(0.0);  // MSKF: also the dynamic time scale's minimum cloud depth
        // USL search loop
        while (true) {
            nu++;
            if (nu >= ncheck) {
                if (ishall == 1) {
                    T chmax = static_cast<T>(0.0);
                    for (int nk = 0; nk < ncheck; ++nk) {
                        int nnn = kcheck[nk];
                        if (cldhgt[nnn] > chmax) {
                            nuchm = nk;
                            chmax = cldhgt[nnn];
                        }
                    }
                    nu = nuchm - 1;
                    fbfrc = static_cast<T>(1.0);
                    continue;
                } else {
                    return; // No convection occurs
                }
            }

            int kmix = kcheck[nu];
            lc = kmix;

            int nlayrs = 0;
            dpthmx = static_cast<T>(0.0);
            for (int nk = lc; nk < kx; ++nk) {
                dpthmx += dp[nk];
                nlayrs++;
                if (dpthmx > dpmin) break;
            }
            if (dpthmx < dpmin) return;
            kpbl = lc + nlayrs - 1;

            tmix = static_cast<T>(0.0);
            qmix = static_cast<T>(0.0);
            zmix = static_cast<T>(0.0);
            pmix = static_cast<T>(0.0);
            for (int nk = lc; nk <= kpbl; ++nk) {
                tmix += dp[nk] * t0[nk];
                qmix += dp[nk] * q0[nk];
                zmix += dp[nk] * z0[nk];
                pmix += dp[nk] * p0[nk];
            }
            tmix /= dpthmx;
            qmix /= dpthmx;
            zmix /= dpthmx;
            pmix /= dpthmx;
            T emix = qmix * pmix / (static_cast<T>(0.622) + qmix);

            const T astrt = static_cast<T>(1.0e-3);
            const T ainc_lut = static_cast<T>(0.075);
            T a1 = emix / aliq;
            T tp = (a1 - astrt) / ainc_lut;
            int indlu = static_cast<int>(tp) + 1;
            if (indlu < 1) indlu = 1;
            if (indlu > 199) indlu = 199;
            T value = static_cast<T>(indlu - 1) * ainc_lut + astrt;
            T aintrp = (a1 - value) / ainc_lut;
            T tlog = aintrp * tables->alu[indlu] + (static_cast<T>(1.0) - aintrp) * tables->alu[indlu - 1];
            T tdpt = (cliq - dliq * tlog) / (bliq - tlog);
            tlcl = tdpt - (static_cast<T>(0.212) + static_cast<T>(1.571e-3) * (tdpt - t00) -
                           static_cast<T>(4.36e-4) * (tmix - t00)) * (tmix - tdpt);
            if (tlcl > tmix) tlcl = tmix;
            tvlcl = tlcl * (static_cast<T>(1.0) + static_cast<T>(0.608) * qmix);
            zlcl = zmix + (tlcl - tmix) / gdry;

            klcl = lc;
            for (int nk = lc; nk < kl; ++nk) {
                klcl = nk;
                if (zlcl <= z0[nk]) break;
            }
            if (zlcl > z0[kl - 1]) return;

            int k_lcl = klcl - 1;
            T dlp = (zlcl - z0[k_lcl]) / (z0[klcl] - z0[k_lcl]);
            T tenv = t0[k_lcl] + (t0[klcl] - t0[k_lcl]) * dlp;
            T qenv = q0[k_lcl] + (q0[klcl] - q0[k_lcl]) * dlp;
            T tven = tenv * (static_cast<T>(1.0) + static_cast<T>(0.608) * qenv);

            T wklcl = (zlcl < static_cast<T>(2.0e3)) ? static_cast<T>(0.02) * zlcl / static_cast<T>(2.0e3)
                                                     : static_cast<T>(0.02);
            const T w0_klclm1 = w0avg(i, j, k_lcl);
            const T w0_klcl = w0avg(i, j, klcl);
            // MSKF: DTRH = 0; the Bechtold DTLCL from W0AVG(KLCL) is overwritten by the KF trigger below (dead)
            T dtrh = static_cast<T>(0.0);
            // KF's trigger (Kain 2004 Eq. 1); the 25 km grid scaling of W only for DX >= 25 km
            if (dx >= static_cast<T>(25.e3)) {
                wkl = (w0_klclm1 + (w0_klcl - w0_klclm1) * dlp) * dx / static_cast<T>(25.0e3) - wklcl;
            } else {
                wkl = (w0_klclm1 + (w0_klcl - w0_klclm1) * dlp) - wklcl;
            }
            T dtlcl = (wkl < static_cast<T>(0.0001)) ? static_cast<T>(0.0) : static_cast<T>(4.64) * libm::pow(wkl, static_cast<T>(0.33));

            // Check trigger condition (MSKF: TLCL+DTLCL.LT.TENV)
            if (tlcl + dtlcl < tenv) {
                continue; // Not buoyant
            }

            // Parcel buoyant
            envirtht(*tables, pmix, tmix, qmix, theteu[k_lcl], aliq, bliq, cliq, dliq);
            T dttot = dtlcl + dtrh;
            if (dttot > static_cast<T>(1.0e-4)) {
                T gdt = static_cast<T>(2.0) * g * dttot * static_cast<T>(500.0) / tven;
                wlcl = static_cast<T>(1.0) + static_cast<T>(0.5) * sqrt(gdt);
                if (wlcl > static_cast<T>(3.0)) wlcl = static_cast<T>(3.0);
            } else {
                wlcl = static_cast<T>(1.0);
            }
            plcl = p0[k_lcl] + (p0[klcl] - p0[k_lcl]) * dlp;
            T wtw = wlcl * wlcl;
            tvlcl = tlcl * (static_cast<T>(1.0) + static_cast<T>(0.608) * qmix);
            T rholcl = plcl / (r * tvlcl);

            lcl = klcl;
            let = lcl;
            // MSKF: the updraft radius from the LCL height and the top of the source layer, 500 m to 4 km
            T rad = zlcl;
            const T sourceht = z0[kpbl];
            rad = c_max(sourceht, rad);
            rad = c_min(static_cast<T>(4000.), rad);
            rad = c_max(static_cast<T>(500.), rad);

            wu[k_lcl] = wlcl;
            au0 = static_cast<T>(0.01) * dxsq;
            umf[k_lcl] = rholcl * au0;
            vmflcl = umf[k_lcl];
            upold = vmflcl;
            upnew = upold;

            ratio2[k_lcl] = static_cast<T>(0.0);
            uer[k_lcl] = static_cast<T>(0.0);
            abe = static_cast<T>(0.0);
            trppt = static_cast<T>(0.0);
            tu[k_lcl] = tlcl;
            tvu[k_lcl] = tvlcl;
            qu[k_lcl] = qmix;
            eqfrc[k_lcl] = static_cast<T>(1.0);
            qliq[k_lcl] = static_cast<T>(0.0);
            qice[k_lcl] = static_cast<T>(0.0);
            qlqout[k_lcl] = static_cast<T>(0.0);
            qicout[k_lcl] = static_cast<T>(0.0);
            detlq[k_lcl] = static_cast<T>(0.0);
            detic[k_lcl] = static_cast<T>(0.0);
            pptliq[k_lcl] = static_cast<T>(0.0);
            pptice[k_lcl] = static_cast<T>(0.0);

            T ttemp = ttfrz;
            T ee1 = static_cast<T>(1.0);
            T ud1 = static_cast<T>(0.0);
            T rei = static_cast<T>(0.0);
            T dilbe = static_cast<T>(0.0);

            // Updraft loop
            ltop = kl - 1;
            for (int nk = k_lcl; nk < kl - 1; ++nk) {
                int nk1 = nk + 1;
                ratio2[nk1] = ratio2[nk];
                T frc1 = static_cast<T>(0.0);
                tu[nk1] = t0[nk1];
                theteu[nk1] = theteu[nk];
                qu[nk1] = qu[nk];
                qliq[nk1] = qliq[nk];
                qice[nk1] = qice[nk];
                T qnewlq, qnewic;
                tpmix2(*tables, p0[nk1], theteu[nk1], tu[nk1], qu[nk1], qliq[nk1],
                       qice[nk1], qnewlq, qnewic, xlv1, xlv0);

                if (tu[nk1] <= ttfrz) {
                    if (tu[nk1] > tbfrz) {
                        if (ttemp > ttfrz) ttemp = ttfrz;
                        frc1 = (ttemp - tu[nk1]) / (ttemp - tbfrz);
                    } else {
                        frc1 = static_cast<T>(1.0);
                    }
                    ttemp = tu[nk1];
                    T qfrz = (qliq[nk1] + qnewlq) * frc1;
                    qnewic = qnewic + qnewlq * frc1;
                    qnewlq = qnewlq - qnewlq * frc1;
                    qice[nk1] = qice[nk1] + qliq[nk1] * frc1;
                    qliq[nk1] = qliq[nk1] - qliq[nk1] * frc1;
                    dtfrznew(tu[nk1], p0[nk1], theteu[nk1], qu[nk1], qfrz, qice[nk1],
                             aliq, bliq, cliq, dliq);
                }
                tvu[nk1] = tu[nk1] * (static_cast<T>(1.0) + static_cast<T>(0.608) * qu[nk1]);

                T be, boterm, dzz;
                if (nk == k_lcl) {
                    be = (tvlcl + tvu[nk1]) / (tven + tv0[nk1]) - static_cast<T>(1.0);
                    boterm = static_cast<T>(2.0) * (z0[nk1] - zlcl) * g * be / static_cast<T>(1.5);
                    dzz = z0[nk1] - zlcl;
                } else {
                    be = (tvu[nk] + tvu[nk1]) / (tv0[nk] + tv0[nk1]) - static_cast<T>(1.0);
                    boterm = static_cast<T>(2.0) * dza[nk] * g * be / static_cast<T>(1.5);
                    dzz = dza[nk];
                }
                T enterm = static_cast<T>(2.0) * rei * wtw / upold;
                // MSKF: the conversion rate over the grid-length factor below 24.999 km
                const T rate_kay = dx >= static_cast<T>(24.999e3) ? rate : rate / scale_fac;

                condload(qliq[nk1], qice[nk1], wtw, dzz, boterm, enterm, rate_kay,
                         qnewlq, qnewic, qlqout[nk1], qicout[nk1], g);

                if (wtw < static_cast<T>(1.0e-3)) {
                    ltop = nk;
                    break;
                } else {
                    wu[nk1] = sqrt(wtw);
                }

                envirtht(*tables, p0[nk1], t0[nk1], q0[nk1], thetee[nk1], aliq, bliq, cliq, dliq);
                // MSKF: the Tokioka parameter 0.03 times the grid-length factor
                const T tokioka = static_cast<T>(0.03) * scale_fac;
                rei = vmflcl * dp[nk1] * tokioka / rad;
                tvqu[nk1] = tu[nk1] * (static_cast<T>(1.0) + static_cast<T>(0.608) * qu[nk1] - qliq[nk1] - qice[nk1]);
                if (nk == k_lcl) {
                    dilbe = ((tvlcl + tvqu[nk1]) / (tven + tv0[nk1]) - static_cast<T>(1.0)) * dzz;
                } else {
                    dilbe = ((tvqu[nk] + tvqu[nk1]) / (tv0[nk] + tv0[nk1]) - static_cast<T>(1.0)) * dzz;
                }
                if (dilbe > static_cast<T>(0.0)) abe += dilbe * g;

                T ee2, ud2;
                if (tvqu[nk1] <= tv0[nk1]) {
                    ee2 = static_cast<T>(0.5);
                    ud2 = static_cast<T>(1.0);
                    eqfrc[nk1] = static_cast<T>(0.0);
                } else {
                    let = nk1;
                    T ttmp = tvqu[nk1];
                    T f1 = static_cast<T>(0.95);
                    T f2 = static_cast<T>(1.0) - f1;
                    T thttmp = f1 * thetee[nk1] + f2 * theteu[nk1];
                    T qtmp = f1 * q0[nk1] + f2 * qu[nk1];
                    T tmpliq = f2 * qliq[nk1];
                    T tmpice = f2 * qice[nk1];
                    T dummy1, dummy2;
                    tpmix2(*tables, p0[nk1], thttmp, ttmp, qtmp, tmpliq, tmpice, dummy1, dummy2, xlv1, xlv0);
                    T tu95 = ttmp * (static_cast<T>(1.0) + static_cast<T>(0.608) * qtmp - tmpliq - tmpice);
                    if (tu95 > tv0[nk1]) {
                        ee2 = static_cast<T>(1.0);
                        ud2 = static_cast<T>(0.0);
                        eqfrc[nk1] = static_cast<T>(1.0);
                    } else {
                        f1 = static_cast<T>(0.10);
                        f2 = static_cast<T>(1.0) - f1;
                        thttmp = f1 * thetee[nk1] + f2 * theteu[nk1];
                        qtmp = f1 * q0[nk1] + f2 * qu[nk1];
                        tmpliq = f2 * qliq[nk1];
                        tmpice = f2 * qice[nk1];
                        tpmix2(*tables, p0[nk1], thttmp, ttmp, qtmp, tmpliq, tmpice, dummy1, dummy2, xlv1, xlv0);
                        T tu10 = ttmp * (static_cast<T>(1.0) + static_cast<T>(0.608) * qtmp - tmpliq - tmpice);
                        T tvdiff = abs(tu10 - tvqu[nk1]);
                        if (tvdiff < static_cast<T>(1.0e-3)) {
                            ee2 = static_cast<T>(1.0);
                            ud2 = static_cast<T>(0.0);
                            eqfrc[nk1] = static_cast<T>(1.0);
                        } else {
                            eqfrc[nk1] = (tv0[nk1] - tvqu[nk1]) * f1 / (tu10 - tvqu[nk1]);
                            if (eqfrc[nk1] < static_cast<T>(0.0)) eqfrc[nk1] = static_cast<T>(0.0);
                            if (eqfrc[nk1] > static_cast<T>(1.0)) eqfrc[nk1] = static_cast<T>(1.0);
                            if (eqfrc[nk1] == static_cast<T>(1.0)) {
                                ee2 = static_cast<T>(1.0);
                                ud2 = static_cast<T>(0.0);
                            } else if (eqfrc[nk1] == static_cast<T>(0.0)) {
                                ee2 = static_cast<T>(0.0);
                                ud2 = static_cast<T>(1.0);
                            } else {
                                prof5(eqfrc[nk1], ee2, ud2);
                            }
                        }
                    }
                }

                if (ee2 < static_cast<T>(0.5)) ee2 = static_cast<T>(0.5);
                ud2 = static_cast<T>(1.5) * ud2;
                uer[nk1] = static_cast<T>(0.5) * rei * (ee1 + ee2);
                udr[nk1] = static_cast<T>(0.5) * rei * (ud1 + ud2);

                if (umf[nk] - udr[nk1] < static_cast<T>(10.0)) {
                    if (dilbe > static_cast<T>(0.0)) {
                        abe -= dilbe * g;
                    }
                    let = nk;
                    ltop = nk;
                    break;
                } else {
                    ee1 = ee2;
                    ud1 = ud2;
                    upold = umf[nk] - udr[nk1];
                    upnew = upold + uer[nk1];
                    umf[nk1] = upnew;
                    dilfrc[nk1] = upnew / upold;
                    detlq[nk1] = qliq[nk1] * udr[nk1];
                    detic[nk1] = qice[nk1] * udr[nk1];
                    qdt[nk1] = qu[nk1];
                    qu[nk1] = (upold * qu[nk1] + uer[nk1] * q0[nk1]) / upnew;
                    theteu[nk1] = (theteu[nk1] * upold + thetee[nk1] * uer[nk1]) / upnew;
                    qliq[nk1] = qliq[nk1] * upold / upnew;
                    qice[nk1] = qice[nk1] * upold / upnew;
                    pptliq[nk1] = qlqout[nk1] * umf[nk];
                    pptice[nk1] = qicout[nk1] * umf[nk];
                    trppt = trppt + pptliq[nk1] + pptice[nk1];  // WRF order (TRPPT+PPTLIQ)+PPTICE
                    if (nk1 <= kpbl) uer[nk1] += vmflcl * dp[nk1] / dpthmx;
                }
            } // updraft loop

            cldhgt[lc] = z0[ltop] - zlcl;
            if (tlcl > static_cast<T>(293.0)) {
                chmin = static_cast<T>(4.0e3);
            } else if (tlcl >= static_cast<T>(273.0)) {
                chmin = static_cast<T>(2.0e3) + static_cast<T>(100.0) * (tlcl - static_cast<T>(273.0));
            } else {
                chmin = static_cast<T>(2.0e3);
            }

            for (int nk = k_lcl; nk <= ltop; ++nk) {
                qc_kf(i, j, nk) = qliq[nk];
                qi_kf(i, j, nk) = qice[nk];
            }
            // MSKF: no convection when the mean relative humidity (over ice below 273.16 K) of the levels K-1 ..
            // LTOP+1 reaches 1.01; a level above K below 0.99 ends the average (envRHavg = 0); scalar exp (the loop
            // exits early, not vectorised). K-1 is level 0 of WRF's arrays (outside kts:kte) when KLCL = 2, and
            // LTOP+1 is KTE+1 when the updraft reaches the top: WRF reads T0, P0, Q0 there outside the arrays
            // (gfortran puts the automatic arrays on the heap: T0(0), P0(0), Q0(0) are malloc's chunk size, a
            // positive subnormal, so ENVEsat = Inf and envRH = NaN). Such a level counts with T0 = P0 = Q0 = 0,
            // which gives the same NaN (the mean is then NaN: no suppression), on the CPU and the GPU alike.
            T envrhavg = static_cast<T>(0.0);
            {
                bool dry = false;
                for (int nk = k_lcl - 1; nk <= ltop + 1; ++nk) {
                    const bool in = nk >= 0 && nk < kx;
                    const T t0k = in ? t0[nk] : static_cast<T>(0.0), p0k = in ? p0[nk] : static_cast<T>(0.0),
                            q0k = in ? q0[nk] : static_cast<T>(0.0);
                    T envesat;
                    if (t0k <= static_cast<T>(273.16)) {
                        envesat = static_cast<T>(6.112) * libm::exp(static_cast<T>(21.87) * (t0k - static_cast<T>(273.16)) /
                                                                     (t0k - static_cast<T>(7.66)));
                    } else {
                        envesat = static_cast<T>(6.112) * libm::exp(static_cast<T>(17.67) * (t0k - static_cast<T>(273.16)) /
                                                                     (static_cast<T>(243.5) + t0k - static_cast<T>(273.16)));
                    }
                    envesat = envesat * static_cast<T>(100.0);
                    const T envqsat = static_cast<T>(0.622) * envesat / (p0k - envesat);
                    const T envrh = q0k / envqsat;
                    if (nk > k_lcl && envrh < static_cast<T>(0.99)) {
                        envrhavg = static_cast<T>(0.0);
                        dry = true;
                        break;
                    }
                    envrhavg = envrhavg + envrh;
                }
                if (!dry) envrhavg = envrhavg / static_cast<T>(ltop - k_lcl + 3);  // float(LTOP-K+1+2)
            }

            if (ltop <= klcl || ltop <= kpbl || let + 1 <= kpbl || envrhavg >= static_cast<T>(1.01)) {
                cldhgt[lc] = static_cast<T>(0.0);
                for (int nk = k_lcl; nk <= ltop; ++nk) {
                    umf[nk] = static_cast<T>(0.0);
                    udr[nk] = static_cast<T>(0.0);
                    uer[nk] = static_cast<T>(0.0);
                    detlq[nk] = static_cast<T>(0.0);
                    detic[nk] = static_cast<T>(0.0);
                    pptliq[nk] = static_cast<T>(0.0);
                    pptice[nk] = static_cast<T>(0.0);
                    cldfra_dp_kf(i, j, nk) = static_cast<T>(0.0);
                    cldfra_sh_kf(i, j, nk) = static_cast<T>(0.0);
                    qc_kf(i, j, nk) = static_cast<T>(0.0);
                    qi_kf(i, j, nk) = static_cast<T>(0.0);
                }
            } else if (cldhgt[lc] > chmin && abe > static_cast<T>(1.0)) {
                ishall = 0;
                for (int nk = k_lcl; nk <= ltop; ++nk) {
                    cldfra_sh_kf(i, j, nk) = static_cast<T>(0.0);
                }
                break; // Deep convection found
            } else {
                ishall = 1;
                for (int nk = k_lcl; nk <= ltop; ++nk) {
                    cldfra_dp_kf(i, j, nk) = static_cast<T>(0.0);
                }
                if (nu == nuchm) {
                    break; // Shallow convection found
                } else {
                    for (int nk = k_lcl; nk <= ltop; ++nk) {
                        umf[nk] = static_cast<T>(0.0);
                        udr[nk] = static_cast<T>(0.0);
                        uer[nk] = static_cast<T>(0.0);
                        detlq[nk] = static_cast<T>(0.0);
                        detic[nk] = static_cast<T>(0.0);
                        pptliq[nk] = static_cast<T>(0.0);
                        pptice[nk] = static_cast<T>(0.0);
                        cldfra_dp_kf(i, j, nk) = static_cast<T>(0.0);
                        cldfra_sh_kf(i, j, nk) = static_cast<T>(0.0);
                        qc_kf(i, j, nk) = static_cast<T>(0.0);
                        qi_kf(i, j, nk) = static_cast<T>(0.0);
                    }
                }
            }
        } // end usl while loop

        if (ishall == 1) {
            int kstart = (kpbl > klcl) ? kpbl : klcl;
            let = kstart;
        }

        if (let == ltop) {
            udr[ltop] = umf[ltop] + udr[ltop] - uer[ltop];
            detlq[ltop] = qliq[ltop] * udr[ltop] * upnew / upold;
            detic[ltop] = qice[ltop] * udr[ltop] * upnew / upold;
            uer[ltop] = static_cast<T>(0.0);
            umf[ltop] = static_cast<T>(0.0);
        } else {
            T dptt = static_cast<T>(0.0);
            for (int nj = let + 1; nj <= ltop; ++nj) {
                dptt += dp[nj];
            }
            T dumfdp = umf[let] / dptt;
            for (int nk = let + 1; nk <= ltop; ++nk) {
                if (nk == ltop) {
                    udr[nk] = umf[nk - 1];
                    uer[nk] = static_cast<T>(0.0);
                    detlq[nk] = udr[nk] * qliq[nk] * dilfrc[nk];
                    detic[nk] = udr[nk] * qice[nk] * dilfrc[nk];
                } else {
                    umf[nk] = umf[nk - 1] - dp[nk] * dumfdp;
                    uer[nk] = umf[nk] * (static_cast<T>(1.0) - static_cast<T>(1.0) / dilfrc[nk]);
                    udr[nk] = umf[nk - 1] - umf[nk] + uer[nk];
                    detlq[nk] = udr[nk] * qliq[nk] * dilfrc[nk];
                    detic[nk] = udr[nk] * qice[nk] * dilfrc[nk];
                }
                if (nk >= let + 2) {
                    trppt = trppt - pptliq[nk] - pptice[nk];
                    pptliq[nk] = umf[nk - 1] * qlqout[nk];
                    pptice[nk] = umf[nk - 1] * qicout[nk];
                    trppt = trppt + pptliq[nk] + pptice[nk];
                }
            }
        }

        int ml = -1;  // WRF ML = 0: no level up to LTOP warmer than T00
        for (int nk = 0; nk <= ltop; ++nk) {
            if (t0[nk] > t00) ml = nk;
        }

        int k_lcl = klcl - 1;
        for (int nk = 0; nk <= k_lcl; ++nk) {
            if (nk >= lc) {
                if (nk == lc) {
                    umf[nk] = vmflcl * dp[nk] / dpthmx;
                    uer[nk] = vmflcl * dp[nk] / dpthmx;
                } else if (nk <= kpbl) {
                    uer[nk] = vmflcl * dp[nk] / dpthmx;
                    umf[nk] = umf[nk - 1] + uer[nk];
                } else {
                    umf[nk] = vmflcl;
                    uer[nk] = static_cast<T>(0.0);
                }
                tu[nk] = tmix + (z0[nk] - zmix) * gdry;
                qu[nk] = qmix;
                wu[nk] = wlcl;
            } else {
                tu[nk] = static_cast<T>(0.0);
                qu[nk] = static_cast<T>(0.0);
                umf[nk] = static_cast<T>(0.0);
                wu[nk] = static_cast<T>(0.0);
                uer[nk] = static_cast<T>(0.0);
                cldfra_dp_kf(i, j, nk) = static_cast<T>(0.0);
                cldfra_sh_kf(i, j, nk) = static_cast<T>(0.0);
                qc_kf(i, j, nk) = static_cast<T>(0.0);
                qi_kf(i, j, nk) = static_cast<T>(0.0);
            }
            udr[nk] = static_cast<T>(0.0);
            qdt[nk] = static_cast<T>(0.0);
            qliq[nk] = static_cast<T>(0.0);
            qice[nk] = static_cast<T>(0.0);
            qlqout[nk] = static_cast<T>(0.0);
            qicout[nk] = static_cast<T>(0.0);
            pptliq[nk] = static_cast<T>(0.0);
            pptice[nk] = static_cast<T>(0.0);
            detlq[nk] = static_cast<T>(0.0);
            detic[nk] = static_cast<T>(0.0);
            ratio2[nk] = static_cast<T>(0.0);
            envirtht(*tables, p0[nk], t0[nk], q0[nk], thetee[nk], aliq, bliq, cliq, dliq);
            eqfrc[nk] = static_cast<T>(1.0);
        }

        int ltop1 = ltop + 1;
        int ltopm1 = ltop - 1;
        for (int nk = ltop1; nk < kx; ++nk) {
            umf[nk] = static_cast<T>(0.0);
            udr[nk] = static_cast<T>(0.0);
            uer[nk] = static_cast<T>(0.0);
            qdt[nk] = static_cast<T>(0.0);
            qliq[nk] = static_cast<T>(0.0);
            qice[nk] = static_cast<T>(0.0);
            qlqout[nk] = static_cast<T>(0.0);
            qicout[nk] = static_cast<T>(0.0);
            detlq[nk] = static_cast<T>(0.0);
            detic[nk] = static_cast<T>(0.0);
            pptliq[nk] = static_cast<T>(0.0);
            pptice[nk] = static_cast<T>(0.0);
            if (nk > ltop1) {
                tu[nk] = static_cast<T>(0.0);
                qu[nk] = static_cast<T>(0.0);
                wu[nk] = static_cast<T>(0.0);
                cldfra_dp_kf(i, j, nk) = static_cast<T>(0.0);
                cldfra_sh_kf(i, j, nk) = static_cast<T>(0.0);
                qc_kf(i, j, nk) = static_cast<T>(0.0);
                qi_kf(i, j, nk) = static_cast<T>(0.0);
            }
            thta0[nk] = static_cast<T>(0.0);
            thtau[nk] = static_cast<T>(0.0);
            ems[nk] = static_cast<T>(0.0);
            emsd[nk] = static_cast<T>(0.0);
            tg[nk] = t0[nk];
            qg[nk] = q0[nk];
            qlg[nk] = static_cast<T>(0.0);
            qig[nk] = static_cast<T>(0.0);
            qrg[nk] = static_cast<T>(0.0);
            qsg[nk] = static_cast<T>(0.0);
            omg[nk] = static_cast<T>(0.0);
        }
        omg[kx] = static_cast<T>(0.0);

        for (int nk = 0; nk <= ltop; ++nk) {
            ems[nk] = dp[nk] * dxsq / g;
            emsd[nk] = static_cast<T>(1.0) / ems[nk];
            T exn_nk = vm::pow(p00 / p0[nk], static_cast<T>(0.2854) * (static_cast<T>(1.0) - static_cast<T>(0.28) * qdt[nk]), nk, ltop + 1);
            thtau[nk] = tu[nk] * exn_nk;
            exn_nk = vm::pow(p00 / p0[nk], static_cast<T>(0.2854) * (static_cast<T>(1.0) - static_cast<T>(0.28) * q0[nk]), nk, ltop + 1);
            thta0[nk] = t0[nk] * exn_nk;
            ddilfrc[nk] = static_cast<T>(1.0) / dilfrc[nk];
            omg[nk] = static_cast<T>(0.0);
        }

        // Convective timescale
        wspd[klcl] = sqrt(u0[klcl] * u0[klcl] + v0[klcl] * v0[klcl]);
        wspd[l5] = sqrt(u0[l5] * u0[l5] + v0[l5] * v0[l5]);
        wspd[ltop] = sqrt(u0[ltop] * u0[ltop] + v0[ltop] * v0[ltop]);
        T vconv = static_cast<T>(0.5) * (wspd[klcl] + wspd[l5]);
        T timec = dx / vconv;
        T tadvec = timec;
        // MSKF: the dynamic time scale (deep and shallow) from the cloud depth and the sub-cloud velocity scale
        timec = c_max(chmin, cldhgt[lc]);
        timec = timec * scale_fac;
        {
            const T densplume = p0[0] / r / t0[0];
            T wst = hfx(i, j) / densplume / cp;
            wst = c_max(static_cast<T>(0.), wst);
            wst = g * pblh(i, j) * wst;
            T thetav = libm::pow(static_cast<T>(1.e5) / p0[0], r / cp);
            thetav = t0[0] * thetav;
            const T eps1u = static_cast<T>(0.622);
            thetav = thetav * (static_cast<T>(1.) + q0[0] * eps1u);
            wst = wst / thetav;
            wst = libm::pow(wst, static_cast<T>(0.3333));
            T sclvel = wst * wst * wst;  // WST**3
            T zlcl_kay = c_max(zlcl, z0[kpbl]);
            sclvel = sclvel / pblh(i, j);
            sclvel = sclvel * zlcl_kay;
            sclvel = libm::pow(sclvel, static_cast<T>(0.333));  // Wsb of the convective PBL
            T frc2;
            if (zol(i, j) <= static_cast<T>(0.0)) {
                frc2 = static_cast<T>(3.8) * ust(i, j) * ust(i, j);
                frc2 = frc2 + static_cast<T>(0.22) * sclvel * sclvel;
                const T zz_kay = static_cast<T>(-1.0) * zol(i, j);
                zlcl_kay = libm::pow(zz_kay, static_cast<T>(2.) / static_cast<T>(3.));
                zlcl_kay = zlcl_kay * (static_cast<T>(1.9) * ust(i, j) * ust(i, j));
                frc2 = frc2 + zlcl_kay;
            } else {
                frc2 = static_cast<T>(3.8) * ust(i, j) * ust(i, j);
            }
            frc2 = sqrt(frc2);
            sclvel = frc2;
            if (sclvel < static_cast<T>(0.1)) sclvel = static_cast<T>(0.1);
            if (abe <= static_cast<T>(0.0)) abe = static_cast<T>(1.0);
            timec = timec / libm::pow(static_cast<T>(0.03) * sclvel * abe, static_cast<T>(0.3333));
        }
        timec = c_max(tadvec, timec);
        int nic = static_cast<int>(lround_x86(timec / dt));
        timec = static_cast<T>(nic) * dt;
        timec = c_min(timec, static_cast<T>(86400.));  // cap at 24 h (WRF 4.8.0)

        T shsign = (wspd[ltop] > wspd[klcl]) ? static_cast<T>(1.0) : static_cast<T>(-1.0);
        T vws = (u0[ltop] - u0[klcl]) * (u0[ltop] - u0[klcl]) + (v0[ltop] - v0[klcl]) * (v0[ltop] - v0[klcl]);
        vws = static_cast<T>(1.0e3) * shsign * sqrt(vws) / (z0[ltop] - z0[lcl]);
        T pef = static_cast<T>(1.591) + vws * (static_cast<T>(-0.639) + vws * (static_cast<T>(9.53e-2) - vws * static_cast<T>(4.96e-3)));
        if (pef < static_cast<T>(0.2)) pef = static_cast<T>(0.2);
        if (pef > static_cast<T>(0.9)) pef = static_cast<T>(0.9);

        T cbh = (zlcl - z0[0]) * static_cast<T>(3.281e-3);
        T rcbh;
        if (cbh < static_cast<T>(3.0)) {
            rcbh = static_cast<T>(0.02);
        } else {
            rcbh = static_cast<T>(0.96729352) + cbh * (static_cast<T>(-0.70034167) + cbh * (static_cast<T>(0.162179896) +
                   cbh * (static_cast<T>(-1.2569798e-2) + cbh * (static_cast<T>(4.2772e-4) - cbh * static_cast<T>(5.44e-6)))));
        }
        if (cbh > static_cast<T>(25.0)) rcbh = static_cast<T>(2.4);
        T pefcbh = static_cast<T>(1.0) / (static_cast<T>(1.0) + rcbh);
        if (pefcbh > static_cast<T>(0.9)) pefcbh = static_cast<T>(0.9);
        T peff = static_cast<T>(0.5) * (pef + pefcbh);

        // Downdraft properties
        T tder = static_cast<T>(0.0);
        int lfs = 0;
        int ldt = 0;
        int ldb = 0;
        int kstart = kpbl + 1;
        T dmffrc = static_cast<T>(0.0);

        if (ishall == 1) {
            lfs = 0;
        } else {
            int klfs = let - 1;
            for (int nk = kstart + 1; nk < kl; ++nk) {
                T dppp = p0[kstart] - p0[nk];
                if (dppp > static_cast<T>(150.0e2)) {
                    klfs = nk;
                    break;
                }
            }
            if (klfs > let - 1) klfs = let - 1;
            lfs = klfs;

            if (p0[kstart] - p0[lfs] > static_cast<T>(50.0e2)) {
                theted[lfs] = thetee[lfs];
                qd[lfs] = q0[lfs];
                T qss;
                tpmix2dd(*tables, p0[lfs], theted[lfs], tz[lfs], qss);
                thtad[lfs] = tz[lfs] * libm::pow(p00 / p0[lfs], static_cast<T>(0.2854) * (static_cast<T>(1.0) - static_cast<T>(0.28) * qss));
                tvd[lfs] = tz[lfs] * (static_cast<T>(1.0) + static_cast<T>(0.608) * qss);
                T rdd = p0[lfs] / (r * tvd[lfs]);
                T a1 = (static_cast<T>(1.0) - peff) * au0;
                dmf[lfs] = -a1 * rdd;
                der[lfs] = dmf[lfs];
                ddr[lfs] = static_cast<T>(0.0);
                T rhbar = rh[lfs] * dp[lfs];
                T dptt = dp[lfs];
                for (int nd = lfs - 1; nd >= kstart; --nd) {
                    int nd1 = nd + 1;
                    der[nd] = der[lfs] * ems[nd] / ems[lfs];
                    ddr[nd] = static_cast<T>(0.0);
                    dmf[nd] = dmf[nd1] + der[nd];
                    theted[nd] = (theted[nd1] * dmf[nd1] + thetee[nd] * der[nd]) / dmf[nd];
                    qd[nd] = (qd[nd1] * dmf[nd1] + q0[nd] * der[nd]) / dmf[nd];
                    dptt += dp[nd];
                    rhbar += rh[nd] * dp[nd];
                }
                rhbar /= dptt;
                dmffrc = static_cast<T>(2.0) * (static_cast<T>(1.0) - rhbar);
                T dpdd = static_cast<T>(0.0);
                T pptmlt = static_cast<T>(0.0);
                for (int nk = klcl; nk <= ltop; ++nk) {
                    pptmlt += pptice[nk];
                }
                T dtmelt = static_cast<T>(0.0);
                if (lc < ml) {
                    dtmelt = rlf * pptmlt / (cp * umf[klcl]);
                }
                ldt = (lfs - 1 < kstart - 1) ? (lfs - 1) : (kstart - 1);
                tpmix2dd(*tables, p0[kstart], theted[kstart], tz[kstart], qss);
                tz[kstart] -= dtmelt;
                T es = aliq * libm::exp((bliq * tz[kstart] - cliq) / (tz[kstart] - dliq));
                qss = static_cast<T>(0.622) * es / (p0[kstart] - es);
                theted[kstart] = tz[kstart] * libm::pow(static_cast<T>(1.0e5) / p0[kstart], static_cast<T>(0.2854) * (static_cast<T>(1.0) - static_cast<T>(0.28) * qss)) *
                                libm::exp((static_cast<T>(3374.6525) / tz[kstart] - static_cast<T>(2.5403)) * qss * (static_cast<T>(1.0) + static_cast<T>(0.81) * qss));

                ldt = (lfs - 1 < kstart - 1) ? (lfs - 1) : (kstart - 1);
                ldb = 0;
                for (int nd = ldt; nd >= 0; --nd) {
                    dpdd += dp[nd];
                    theted[nd] = theted[kstart];
                    qd[nd] = qd[kstart];
                    tpmix2dd(*tables, p0[nd], theted[nd], tz[nd], qss);
                    qsd[nd] = qss;
                    T rhh = static_cast<T>(1.0) - static_cast<T>(0.2) / static_cast<T>(1000.0) * (z0[kstart] - z0[nd]);
                    if (rhh < static_cast<T>(1.0)) {
                        T dssdt = (cliq - bliq * dliq) / ((tz[nd] - dliq) * (tz[nd] - dliq));
                        T rl = xlv0 - xlv1 * tz[nd];
                        T dtmp = rl * qss * (static_cast<T>(1.0) - rhh) / (cp + rl * rhh * qss * dssdt);
                        T t1rh = tz[nd] + dtmp;
                        es = rhh * aliq * libm::exp((bliq * t1rh - cliq) / (t1rh - dliq));
                        T qsrh = static_cast<T>(0.622) * es / (p0[nd] - es);
                        if (qsrh < qd[nd]) {
                            qsrh = qd[nd];
                            t1rh = tz[nd] + (qss - qsrh) * rl / cp;
                        }
                        tz[nd] = t1rh;
                        qss = qsrh;
                        qsd[nd] = qss;
                    }
                    tvd[nd] = tz[nd] * (static_cast<T>(1.0) + static_cast<T>(0.608) * qsd[nd]);
                    if (tvd[nd] > tv0[nd] || nd == 0) {
                        ldb = nd;
                        break;
                    }
                }
                if (p0[ldb] - p0[lfs] > static_cast<T>(50.0e2)) {
                    for (int nd = ldt; nd >= ldb; --nd) {
                        int nd1 = nd + 1;
                        ddr[nd] = -dmf[kstart] * dp[nd] / dpdd;
                        der[nd] = static_cast<T>(0.0);
                        dmf[nd] = dmf[nd1] + ddr[nd];
                        tder += (qsd[nd] - qd[nd]) * ddr[nd];
                        qd[nd] = qsd[nd];
                        thtad[nd] = tz[nd] * libm::pow(p00 / p0[nd], static_cast<T>(0.2854) * (static_cast<T>(1.0) - static_cast<T>(0.28) * qd[nd]));
                    }
                }
            }
        }

        T cpr = trppt;
        T pptflx = trppt;
        if (tder < static_cast<T>(1.0)) {
            pptflx = trppt;
            cpr = trppt;
            tder = static_cast<T>(0.0);
            ldb = lfs;
            for (int ndk = 0; ndk <= ltop; ++ndk) {
                dmf[ndk] = static_cast<T>(0.0);
                der[ndk] = static_cast<T>(0.0);
                ddr[ndk] = static_cast<T>(0.0);
                thtad[ndk] = static_cast<T>(0.0);
                wd[ndk] = static_cast<T>(0.0);
                tz[ndk] = static_cast<T>(0.0);
                qd[ndk] = static_cast<T>(0.0);
            }
            // above LTOP WRF leaves DMF and DER undefined here; only MSKF_CMT's MC and ED of the cloud-free levels
            // above the cloud read them, which MSKF_CMT does not use
            for (int ndk = ltop + 1; ndk < kx; ++ndk) {
                dmf[ndk] = static_cast<T>(0.0);
                der[ndk] = static_cast<T>(0.0);
            }
        } else {
            T ddinc = -dmffrc * umf[klcl] / dmf[kstart];
            if (tder * ddinc > trppt) {
                ddinc = trppt / tder;
            }
            tder *= ddinc;
            for (int nk = ldb; nk <= lfs; ++nk) {
                dmf[nk] *= ddinc;
                der[nk] *= ddinc;
                ddr[nk] *= ddinc;
            }
            cpr = trppt;
            pptflx = trppt - tder;  // WRF: no clip (TDER*DDINC <= TRPPT; can round to a tiny negative)
            peff = pptflx / trppt;

            if (ldb > 0) {
                for (int nk = 0; nk < ldb; ++nk) {
                    dmf[nk] = static_cast<T>(0.0);
                    der[nk] = static_cast<T>(0.0);
                    ddr[nk] = static_cast<T>(0.0);
                    wd[nk] = static_cast<T>(0.0);
                    tz[nk] = static_cast<T>(0.0);
                    qd[nk] = static_cast<T>(0.0);
                    thtad[nk] = static_cast<T>(0.0);
                }
            }
            for (int nk = lfs + 1; nk < kx; ++nk) {
                dmf[nk] = static_cast<T>(0.0);
                der[nk] = static_cast<T>(0.0);
                ddr[nk] = static_cast<T>(0.0);
                wd[nk] = static_cast<T>(0.0);
                tz[nk] = static_cast<T>(0.0);
                qd[nk] = static_cast<T>(0.0);
                thtad[nk] = static_cast<T>(0.0);
            }
            for (int nk = ldt + 1; nk < lfs; ++nk) {
                tz[nk] = static_cast<T>(0.0);
                qd[nk] = static_cast<T>(0.0);
                thtad[nk] = static_cast<T>(0.0);
            }
        }

        T aincmx = static_cast<T>(1000.0);
        int lmax = (klcl > lfs) ? klcl : lfs;
        for (int nk = lc; nk <= lmax; ++nk) {
            if ((uer[nk] - der[nk]) > static_cast<T>(1.0e-3)) {
                T aincm1 = ems[nk] / ((uer[nk] - der[nk]) * timec);
                if (aincm1 < aincmx) aincmx = aincm1;
            }
        }
        T ainc = static_cast<T>(1.0);
        if (aincmx < ainc) ainc = aincmx;

        T tder2 = tder;
        T pptfl2 = pptflx;
        for (int nk = 0; nk <= ltop; ++nk) {
            detlq2[nk] = detlq[nk];
            detic2[nk] = detic[nk];
            udr2[nk] = udr[nk];
            uer2[nk] = uer[nk];
            ddr2[nk] = ddr[nk];
            der2[nk] = der[nk];
            umf2[nk] = umf[nk];
            dmf2[nk] = dmf[nk];
        }
        T fabe = static_cast<T>(1.0);
        const T stab = static_cast<T>(0.95);
        int noitr = 0;

        if (ishall == 1) {
            T tkemax = static_cast<T>(5.0);
            T evac = static_cast<T>(0.5) * tkemax * static_cast<T>(0.1);
            ainc = evac * dpthmx * dxsq / (vmflcl * g * timec);
            tder = tder2 * ainc;
            pptflx = pptfl2 * ainc;
            for (int nk = 0; nk <= ltop; ++nk) {
                umf[nk] = umf2[nk] * ainc;
                dmf[nk] = dmf2[nk] * ainc;
                detlq[nk] = detlq2[nk] * ainc;
                detic[nk] = detic2[nk] * ainc;
                udr[nk] = udr2[nk] * ainc;
                uer[nk] = uer2[nk] * ainc;
                der[nk] = der2[nk] * ainc;
                ddr[nk] = ddr2[nk] * ainc;
            }
        }

        // Iterative closure loop
        T aincold = static_cast<T>(0.0);
        T fabeold = static_cast<T>(0.0);
        int nstep = 0;
        T dtime = static_cast<T>(0.0);

        for (int ncount = 1; ncount <= 10; ++ncount) {
            T dtt = timec;
            for (int nk = 0; nk <= ltop; ++nk) {
                domgdp[nk] = -(uer[nk] - der[nk] - udr[nk] - ddr[nk]) * emsd[nk];
                if (nk > 0) {
                    omg[nk] = omg[nk - 1] - dp[nk - 1] * domgdp[nk - 1];
                    T absomg = abs(omg[nk]);
                    T absomgtc = absomg * timec;
                    T frdp = static_cast<T>(0.75) * dp[nk - 1];
                    if (absomgtc > frdp) {
                        T dtt1 = frdp / absomg;
                        if (dtt1 < dtt) dtt = dtt1;
                    }
                }
            }
            nstep = static_cast<int>(std::round(timec / dtt + static_cast<T>(1.0)));
            dtime = timec / static_cast<T>(nstep);
            for (int nk = 0; nk <= ltop; ++nk) {
                thpa[nk] = thta0[nk];
                qpa[nk] = q0[nk];
                fxm[nk] = omg[nk] * dxsq / g;
            }

            for (int ntc = 1; ntc <= nstep; ++ntc) {
                for (int nk = 0; nk <= ltop; ++nk) {
                    thfxin[nk] = static_cast<T>(0.0);
                    thfxout[nk] = static_cast<T>(0.0);
                    qfxin[nk] = static_cast<T>(0.0);
                    qfxout[nk] = static_cast<T>(0.0);
                }
                for (int nk = 1; nk <= ltop; ++nk) {
                    if (omg[nk] <= static_cast<T>(0.0)) {
                        thfxin[nk] = -fxm[nk] * thpa[nk - 1];
                        qfxin[nk] = -fxm[nk] * qpa[nk - 1];
                        thfxout[nk - 1] += thfxin[nk];
                        qfxout[nk - 1] += qfxin[nk];
                    } else {
                        thfxout[nk] = fxm[nk] * thpa[nk];
                        qfxout[nk] = fxm[nk] * qpa[nk];
                        thfxin[nk - 1] += thfxout[nk];
                        qfxin[nk - 1] += qfxout[nk];
                    }
                }
                for (int nk = 0; nk <= ltop; ++nk) {
                    thpa[nk] += (thfxin[nk] + udr[nk] * thtau[nk] + ddr[nk] * thtad[nk] -
                                 thfxout[nk] - (uer[nk] - der[nk]) * thta0[nk]) * dtime * emsd[nk];
                    qpa[nk] += (qfxin[nk] + udr[nk] * qdt[nk] + ddr[nk] * qd[nk] -
                                qfxout[nk] - (uer[nk] - der[nk]) * q0[nk]) * dtime * emsd[nk];
                }
            }
            for (int nk = 0; nk <= ltop; ++nk) {
                thtag[nk] = thpa[nk];
                qg[nk] = qpa[nk];
            }

            // Check non-negative moisture
            for (int nk = 0; nk <= ltop; ++nk) {
                if (qg[nk] < static_cast<T>(0.0)) {
                    if (nk == 0) {  // WRF: wrf_error_fatal('QG, QG(NK).LT.0')
                        raise_fatal(kFatalKfQgSurface);
                        return;
                    }
                    int nk1 = (nk == ltop) ? klcl : (nk + 1);
                    T tma = qg[nk1] * ems[nk1];
                    T tmb = qg[nk - 1] * ems[nk - 1];
                    T tmm = (qg[nk] - static_cast<T>(1.0e-9)) * ems[nk];
                    T bcoeff = -tmm / ((tma * tma) / tmb + tmb);
                    T acoeff = bcoeff * tma / tmb;
                    tmb *= (static_cast<T>(1.0) - bcoeff);
                    tma *= (static_cast<T>(1.0) - acoeff);
                    qg[nk] = static_cast<T>(1.0e-9);
                    qg[nk1] = tma * emsd[nk1];
                    qg[nk - 1] = tmb * emsd[nk - 1];
                }
            }

            T topomg = (udr[ltop] - uer[ltop]) * dp[ltop] * emsd[ltop];
            if (abs(topomg - omg[ltop]) > static_cast<T>(1.0e-3)) {
                // WRF: ISTOP = 1, EXIT iter, then wrf_error_fatal('KAIN-FRITSCH, istop=1, diags')
                raise_fatal(kFatalKfIstop);
                return;
            }

            for (int nk = 0; nk <= ltop; ++nk) {
                T exn_nk = vm::pow(p00 / p0[nk], static_cast<T>(0.2854) * (static_cast<T>(1.0) - static_cast<T>(0.28) * qg[nk]), nk, ltop + 1);
                tg[nk] = thtag[nk] / exn_nk;
                tvg[nk] = tg[nk] * (static_cast<T>(1.0) + static_cast<T>(0.608) * qg[nk]);
            }
            if (ishall == 1) {
                break;
            }

            // Adjusted ABE
            tmix = static_cast<T>(0.0);
            qmix = static_cast<T>(0.0);
            for (int nk = lc; nk <= kpbl; ++nk) {
                tmix += dp[nk] * tg[nk];
                qmix += dp[nk] * qg[nk];
            }
            tmix /= dpthmx;
            qmix /= dpthmx;
            T es = aliq * libm::exp((tmix * bliq - cliq) / (tmix - dliq));
            T qss = static_cast<T>(0.622) * es / (pmix - es);
            if (qmix > qss) {
                T rl = xlv0 - xlv1 * tmix;
                T cpm = cp * (static_cast<T>(1.0) + static_cast<T>(0.887) * qmix);
                T dssdt = qss * (cliq - bliq * dliq) / ((tmix - dliq) * (tmix - dliq));
                T dq = (qmix - qss) / (static_cast<T>(1.0) + rl * dssdt / cpm);
                tmix += rl / cp * dq;
                qmix -= dq;
                tlcl = tmix;
            } else {
                if (qmix < static_cast<T>(0.0)) qmix = static_cast<T>(0.0);
                T emix = qmix * pmix / (static_cast<T>(0.622) + qmix);
                const T astrt = static_cast<T>(1.0e-3);
                const T binc = static_cast<T>(0.075);
                T a1 = emix / aliq;
                T tp = (a1 - astrt) / binc;
                int indlu = static_cast<int>(tp) + 1;
                if (indlu < 1) indlu = 1;
                if (indlu > 199) indlu = 199;
                T value = static_cast<T>(indlu - 1) * binc + astrt;
                T aintrp = (a1 - value) / binc;
                T tlog = aintrp * tables->alu[indlu] + (static_cast<T>(1.0) - aintrp) * tables->alu[indlu - 1];
                T tdpt = (cliq - dliq * tlog) / (bliq - tlog);
                tlcl = tdpt - (static_cast<T>(0.212) + static_cast<T>(1.571e-3) * (tdpt - t00) -
                               static_cast<T>(4.36e-4) * (tmix - t00)) * (tmix - tdpt);
                if (tlcl > tmix) tlcl = tmix;
            }
            tvlcl = tlcl * (static_cast<T>(1.0) + static_cast<T>(0.608) * qmix);
            zlcl = zmix + (tlcl - tmix) / gdry;
            for (int nk = lc; nk < kl; ++nk) {
                klcl = nk;
                if (zlcl <= z0[nk]) break;
            }
            int k_lcl_adj = klcl - 1;
            T dlp = (zlcl - z0[k_lcl_adj]) / (z0[klcl] - z0[k_lcl_adj]);
            T tenv = tg[k_lcl_adj] + (tg[klcl] - tg[k_lcl_adj]) * dlp;
            T qenv = qg[k_lcl_adj] + (qg[klcl] - qg[k_lcl_adj]) * dlp;
            T tven = tenv * (static_cast<T>(1.0) + static_cast<T>(0.608) * qenv);
            theteu[k_lcl_adj] = tmix * libm::pow(static_cast<T>(1.0e5) / pmix, static_cast<T>(0.2854) * (static_cast<T>(1.0) - static_cast<T>(0.28) * qmix)) *
                               libm::exp((static_cast<T>(3374.6525) / tlcl - static_cast<T>(2.5403)) * qmix * (static_cast<T>(1.0) + static_cast<T>(0.81) * qmix));

            T abeg = static_cast<T>(0.0);
            for (int nk = k_lcl_adj; nk <= ltopm1; ++nk) {
                int nk1 = nk + 1;
                theteu[nk1] = theteu[nk];
                tpmix2dd(*tables, p0[nk1], theteu[nk1], tgu[nk1], qgu[nk1]);
                tvqu[nk1] = tgu[nk1] * (static_cast<T>(1.0) + static_cast<T>(0.608) * qgu[nk1] - qliq[nk1] - qice[nk1]);
                T dzz = (nk == k_lcl_adj) ? (z0[klcl] - zlcl) : dza[nk];
                T dilbe = ((nk == k_lcl_adj) ? ((tvlcl + tvqu[nk1]) / (tven + tvg[nk1]) - static_cast<T>(1.0))
                                             : ((tvqu[nk] + tvqu[nk1]) / (tvg[nk] + tvg[nk1]) - static_cast<T>(1.0))) * dzz;
                if (dilbe > static_cast<T>(0.0)) abeg += dilbe * g;
                envirtht(*tables, p0[nk1], tg[nk1], qg[nk1], thteeg[nk1], aliq, bliq, cliq, dliq);
                theteu[nk1] = theteu[nk1] * ddilfrc[nk1] + thteeg[nk1] * (static_cast<T>(1.0) - ddilfrc[nk1]);
            }

            if (noitr == 1) {
                break;
            }
            T dabe = (abe - abeg > capedx * abe) ? (abe - abeg) : (capedx * abe);  // MSKF: capeDX (0.1 at 25 km)
            fabe = abeg / abe;
            if (fabe > static_cast<T>(1.0) && ishall == 0) return;
            if (ncount != 1) {
                if (abs(ainc - aincold) < static_cast<T>(0.0001)) {
                    noitr = 1;
                    ainc = aincold;
                    continue;
                }
                T dfda = (fabe - fabeold) / (ainc - aincold);
                if (dfda > static_cast<T>(0.0)) {
                    noitr = 1;
                    ainc = aincold;
                    continue;
                }
            }
            aincold = ainc;
            fabeold = fabe;
            if (ainc / aincmx > static_cast<T>(0.999) && fabe > static_cast<T>(1.05) - stab) {
                break;
            }
            if ((fabe <= static_cast<T>(1.05) - stab && fabe >= static_cast<T>(0.95) - stab) || ncount == 10) {
                break;
            } else {
                if (fabe == static_cast<T>(0.0)) {
                    ainc *= static_cast<T>(0.5);
                } else {
                    if (dabe < static_cast<T>(1.0e-4)) {
                        noitr = 1;
                        ainc = aincold;
                        continue;
                    } else {
                        ainc = ainc * stab * abe / dabe;
                    }
                }
                if (ainc > aincmx) ainc = aincmx;
                if (ainc < static_cast<T>(0.05)) return;
                tder = tder2 * ainc;
                pptflx = pptfl2 * ainc;
                for (int nk = 0; nk <= ltop; ++nk) {
                    umf[nk] = umf2[nk] * ainc;
                    dmf[nk] = dmf2[nk] * ainc;
                    detlq[nk] = detlq2[nk] * ainc;
                    detic[nk] = detic2[nk] * ainc;
                    udr[nk] = udr2[nk] * ainc;
                    uer[nk] = uer2[nk] * ainc;
                    der[nk] = der2[nk] * ainc;
                    ddr[nk] = ddr2[nk] * ainc;
                }
            }
        } // iteration loop

        // Cloud fractions (MSKF): the updraft mass flux over the area without the updraft, (100-AINC)/100*DX**2, at
        // KLCL..LTOP (vectorised in WRF: libmvec log over the loop's trip count)
        {
            T updil = (static_cast<T>(100.) - ainc);
            updil = updil / static_cast<T>(100.);
            updil = updil * dxsq;
            const int n_lanes = ltop - klcl + 1;
            if (ishall == 1) {
                for (int nk = klcl; nk <= ltop; ++nk) {
                    T umf_new = umf[nk] / updil;
                    T xcldfra = static_cast<T>(0.07) * vm::log(static_cast<T>(1.0) + (static_cast<T>(500.0) * umf_new), nk - klcl, n_lanes);
                    if (xcldfra < static_cast<T>(0.01)) xcldfra = static_cast<T>(0.01);
                    if (xcldfra > static_cast<T>(0.2)) xcldfra = static_cast<T>(0.2);
                    cldfra_sh_kf(i, j, nk) = xcldfra;
                }
            } else {
                for (int nk = klcl; nk <= ltop; ++nk) {
                    T umf_new = umf[nk] / updil;
                    T xcldfra = static_cast<T>(0.14) * vm::log(static_cast<T>(1.0) + (static_cast<T>(500.0) * umf_new), nk - klcl, n_lanes);
                    if (xcldfra < static_cast<T>(0.01)) xcldfra = static_cast<T>(0.01);
                    if (xcldfra > static_cast<T>(0.6)) xcldfra = static_cast<T>(0.6);
                    cldfra_dp_kf(i, j, nk) = xcldfra;
                }
            }
        }

        // Hydrometeor tendencies (alias dead closure scratch arrays)
        auto qlpa = umf2;
        auto qipa = dmf2;
        auto qrpa = detlq2;
        auto qspa = detic2;
        auto qlfxin = thfxin;
        auto qlfxout = thfxout;
        auto qifxin = qfxin;
        auto qifxout = qfxout;
        auto qrfxin = udr2;
        auto qrfxout = uer2;
        auto qsfxin = der2;
        auto qsfxout = ddr2;

        T frc2 = (cpr > static_cast<T>(0.0)) ? (pptflx / (cpr * ainc)) : static_cast<T>(0.0);
        for (int nk = 0; nk <= ltop; ++nk) {
            qlpa[nk] = ql0[nk];
            qipa[nk] = qi0[nk];
            qrpa[nk] = qr0[nk];
            qspa[nk] = qs0[nk];
            rainfb[nk] = pptliq[nk] * ainc * fbfrc * frc2;
            snowfb[nk] = pptice[nk] * ainc * fbfrc * frc2;
        }

        for (int ntc = 1; ntc <= nstep; ++ntc) {
            for (int nk = 0; nk <= ltop; ++nk) {
                qlfxin[nk] = static_cast<T>(0.0);
                qlfxout[nk] = static_cast<T>(0.0);
                qifxin[nk] = static_cast<T>(0.0);
                qifxout[nk] = static_cast<T>(0.0);
                qrfxin[nk] = static_cast<T>(0.0);
                qrfxout[nk] = static_cast<T>(0.0);
                qsfxin[nk] = static_cast<T>(0.0);
                qsfxout[nk] = static_cast<T>(0.0);
            }
            for (int nk = 1; nk <= ltop; ++nk) {
                if (omg[nk] <= static_cast<T>(0.0)) {
                    qlfxin[nk] = -fxm[nk] * qlpa[nk - 1];
                    qifxin[nk] = -fxm[nk] * qipa[nk - 1];
                    qrfxin[nk] = -fxm[nk] * qrpa[nk - 1];
                    qsfxin[nk] = -fxm[nk] * qspa[nk - 1];
                    qlfxout[nk - 1] += qlfxin[nk];
                    qifxout[nk - 1] += qifxin[nk];
                    qrfxout[nk - 1] += qrfxin[nk];
                    qsfxout[nk - 1] += qsfxin[nk];
                } else {
                    qlfxout[nk] = fxm[nk] * qlpa[nk];
                    qifxout[nk] = fxm[nk] * qipa[nk];
                    qrfxout[nk] = fxm[nk] * qrpa[nk];
                    qsfxout[nk] = fxm[nk] * qspa[nk];
                    qlfxin[nk - 1] += qlfxout[nk];
                    qifxin[nk - 1] += qifxout[nk];
                    qrfxin[nk - 1] += qrfxout[nk];
                    qsfxin[nk - 1] += qsfxout[nk];
                }
            }
            for (int nk = 0; nk <= ltop; ++nk) {
                qlpa[nk] += (qlfxin[nk] + detlq[nk] - qlfxout[nk]) * dtime * emsd[nk];
                qipa[nk] += (qifxin[nk] + detic[nk] - qifxout[nk]) * dtime * emsd[nk];
                qrpa[nk] += (qrfxin[nk] - qrfxout[nk] + rainfb[nk]) * dtime * emsd[nk];
                qspa[nk] += (qsfxin[nk] - qsfxout[nk] + snowfb[nk]) * dtime * emsd[nk];
            }
        }
        for (int nk = 0; nk <= ltop; ++nk) {
            qlg[nk] = qlpa[nk];
            qig[nk] = qipa[nk];
            qrg[nk] = qrpa[nk];
            qsg[nk] = qspa[nk];
        }

        // Convective timescale & NCA
        if (tadvec < timec) {
            nic = static_cast<int>(lround_x86(tadvec / dt));
        }
        nca(i, j) = static_cast<T>(nic) * dt;
        if (ishall == 1) {  // MSKF: TIMEC stays the dynamic one (KF: 2400 s)
            nca(i, j) = cudt * static_cast<T>(60.0);
        }

        // WRF: PRATEC = PPTFLX*(1.-FBFRC)/DXSQ, RAINCV = DT*PRATEC, unclipped (can be a tiny negative)
        pratec(i, j) = pptflx * (static_cast<T>(1.0) - fbfrc) / dxsq;
        raincv(i, j) = dt * pratec(i, j);
        cutop(i, j) = static_cast<T>(ltop + 1); // 1-based level
        cubot(i, j) = static_cast<T>(lcl + 1);  // 1-based level

        for (int k_idx = 0; k_idx < kx; ++k_idx) {
            T dqcdt_k, dqidt_k, dqrdt_k, dqsdt_k;
            if (warm_rain) {
                T cpm = cp * (static_cast<T>(1.0) + static_cast<T>(0.887) * qg[k_idx]);
                tg[k_idx] -= (qig[k_idx] + qsg[k_idx]) * rlf / cpm;
                dqcdt_k = (qlg[k_idx] + qig[k_idx] - ql0[k_idx] - qi0[k_idx]) / timec;
                dqidt_k = static_cast<T>(0.0);
                dqrdt_k = (qrg[k_idx] + qsg[k_idx] - qr0[k_idx] - qs0[k_idx]) / timec;
                dqsdt_k = static_cast<T>(0.0);
            } else if (!flag_qs) {
                T cpm = cp * (static_cast<T>(1.0) + static_cast<T>(0.887) * qg[k_idx]);
                if (k_idx <= ml) {
                    tg[k_idx] -= (qig[k_idx] + qsg[k_idx]) * rlf / cpm;
                } else {
                    tg[k_idx] += (qlg[k_idx] + qrg[k_idx]) * rlf / cpm;
                }
                dqcdt_k = (qlg[k_idx] + qig[k_idx] - ql0[k_idx] - qi0[k_idx]) / timec;
                dqidt_k = static_cast<T>(0.0);
                dqrdt_k = (qrg[k_idx] + qsg[k_idx] - qr0[k_idx] - qs0[k_idx]) / timec;
                dqsdt_k = static_cast<T>(0.0);
            } else {
                dqcdt_k = (qlg[k_idx] - ql0[k_idx]) / timec;
                dqsdt_k = (qsg[k_idx] - qs0[k_idx]) / timec;
                dqrdt_k = (qrg[k_idx] - qr0[k_idx]) / timec;
                if (flag_qi) {
                    dqidt_k = (qig[k_idx] - qi0[k_idx]) / timec;
                } else {
                    dqsdt_k += (qig[k_idx] - qi0[k_idx]) / timec;
                }
            }
            T dtdt_k = (tg[k_idx] - t0[k_idx]) / timec;
            T dqdt_k = (qg[k_idx] - q0[k_idx]) / timec;

            rthcuten(i, j, k_idx) = dtdt_k / pi(i, j, k_idx);
            rqvcuten(i, j, k_idx) = dqdt_k;
            if (flag_qr) {
                rqrcuten(i, j, k_idx) = dqrdt_k;
                rqccuten(i, j, k_idx) = dqcdt_k;
            } else {
                rqrcuten(i, j, k_idx) = static_cast<T>(0.0);
                rqccuten(i, j, k_idx) = dqrdt_k + dqcdt_k;
            }
            if (flag_qi) rqicuten(i, j, k_idx) = dqidt_k;
            if (flag_qs) rqscuten(i, j, k_idx) = dqsdt_k;
        }

        // ---- the momentum transport (cmt_opt_flag = .TRUE.): MSKF_eta_PARA's inputs, then MSKF_CMT ----
        // CAM's column (J = KTE-KQ+1 from the top; MSG = 0, one column); GRAV = 9.8 there, G (9.81) in the inputs
        CPPWRF_MSKF_CMT(CPPWRF_COLUMN_ARRAY)
        {
            const int kte = kx;
            const int ilev = kte;
            const T grav = static_cast<T>(9.8);
            const T cpin = cp;
            const T rdry = r;
            czf[kte + 1] = static_cast<T>(0.0);
            const T dsubcld = zlcl;
            int jbb = kte - (klcl + 1) + 1;  // updraft base (Fortran KLCL = klcl + 1)
            int jdd = kte - (lfs + 1) + 1;
            int jtt = kte - (ltop + 1) + 1;
            if (jdd < jtt || jdd > jbb) jdd = jbb - 1;  // no downdraft
            if (jtt > jbb) jtt = jbb;
            zfw[0] = static_cast<T>(0.0);
            for (int kq = 1; kq <= kte; ++kq) zfw[kq] = zfw[kq - 1] + dzq[kq - 1];
            for (int kq = 1; kq <= kte; ++kq) {
                const int jk = kte - kq + 1, k = kq - 1;
                cdudt[jk] = static_cast<T>(0.0);
                cdvdt[jk] = static_cast<T>(0.0);
                czf[jk] = zfw[kq];
                const T se = cp * (t0[k] * (static_cast<T>(1.0) + static_cast<T>(0.622) * q0[k])) + g * zfw[kq];
                if (kq == kte) {
                    cqhat[jk] = q0[k];
                    cshat[jk] = se;
                } else {
                    cqhat[jk] = static_cast<T>(0.5) * (q0[k] / (static_cast<T>(1.) + q0[k]) +
                                                       q0[k + 1] / (static_cast<T>(1.) + q0[k + 1]));
                    const T se1 = cp * (t0[k + 1] * (static_cast<T>(1.0) + static_cast<T>(0.622) * q0[k + 1])) +
                                  g * zfw[kq + 1];
                    cshat[jk] = static_cast<T>(0.5) * (se + se1);
                }
                if (k < klcl || k > ltop) {  // subcloud or above-cloud layer
                    cqu[jk] = q0[k] / (static_cast<T>(1.) + q0[k]);
                    cqd[jk] = q0[k] / (static_cast<T>(1.) + q0[k]);
                    csu[jk] = cp * (t0[k] * (static_cast<T>(1.0) + static_cast<T>(0.622) * q0[k])) + g * zfw[kq];
                    csd[jk] = cp * (t0[k] * (static_cast<T>(1.0) + static_cast<T>(0.622) * q0[k])) + g * zfw[kq];
                } else {
                    cqu[jk] = qu[k] / (static_cast<T>(1.) + qu[k]);
                    cqd[jk] = qd[k] / (static_cast<T>(1.) + qd[k]);
                    const T tdn = tz[k] != static_cast<T>(0.0) ? tz[k] : t0[k];
                    const T tup = tu[k] != static_cast<T>(0.0) ? tu[k] : t0[k];
                    csu[jk] = cp * (tup * (static_cast<T>(1.0) + static_cast<T>(0.622) * cqu[jk])) + g * zfw[kq];
                    csd[jk] = cp * (tdn * (static_cast<T>(1.0) + static_cast<T>(0.622) * cqd[jk])) + g * zfw[kq];
                }
                ced[jk] = (((der[k] / dxsq) * g) / static_cast<T>(100.)) / dzq[k];
                ceu[jk] = (((uer[k] / dxsq) * g) / static_cast<T>(100.)) / dzq[k];
                cmd[jk] = ((dmf[k] / dxsq) * g) / static_cast<T>(100.);
                cmu[jk] = ((umf[k] / dxsq) * g) / static_cast<T>(100.);
                cmc[jk] = cmd[jk] + cmu[jk];
                cdp[jk] = dp[k] / static_cast<T>(100.);
                cpru[jk] = p0[k] / static_cast<T>(100.0);
                cu[jk] = u0[k];
                cv[jk] = v0[k];
                cz[jk] = z0[k];
                ct[jk] = t0[k];
            }

            // MSKF_CMT(DUDT, DVDT, ..., DP, ED, EU, MC, MD, MU, ..., P, QD, QU, QHAT, SD, SU, SHAT, T, U, V, Z, ZF,
            // DSUBCLD, JB, JD, JT, ...): ALPHA = 0.5, RNU = 0, NBF = 10 Bessel modes
            const T alfa1[10] = {static_cast<T>(4.33675e-05), static_cast<T>(7.88429e-05), static_cast<T>(1.13065e-04),
                                 static_cast<T>(1.45836e-04), static_cast<T>(1.76823e-04), static_cast<T>(2.05660e-04),
                                 static_cast<T>(2.32101e-04), static_cast<T>(2.55829e-04), static_cast<T>(2.76672e-04),
                                 static_cast<T>(2.94347e-04)};
            const T alfa2[10] = {static_cast<T>(-8.64731e-05), static_cast<T>(-1.56092e-04), static_cast<T>(-2.21315e-04),
                                 static_cast<T>(-2.80995e-04), static_cast<T>(-3.33799e-04), static_cast<T>(-3.78455e-04),
                                 static_cast<T>(-4.14020e-04), static_cast<T>(-4.39600e-04), static_cast<T>(-4.54652e-04),
                                 static_cast<T>(-4.58761e-04)};
            const T bsj0[10] = {static_cast<T>(-4.02759e-01), static_cast<T>(3.00116e-01), static_cast<T>(-2.49705e-01),
                                static_cast<T>(2.18359e-01), static_cast<T>(-1.96465e-01), static_cast<T>(1.80062e-01),
                                static_cast<T>(-1.67183e-01), static_cast<T>(1.56722e-01), static_cast<T>(-1.48011e-01),
                                static_cast<T>(1.40605e-01)};
            const T bsj1[10] = {static_cast<T>(1.14193e-01), static_cast<T>(2.05841e-01), static_cast<T>(2.91209e-01),
                                static_cast<T>(3.68619e-01), static_cast<T>(4.36201e-01), static_cast<T>(4.92233e-01),
                                static_cast<T>(5.35486e-01), static_cast<T>(5.64886e-01), static_cast<T>(5.79879e-01),
                                static_cast<T>(5.80195e-01)};
            const T tau[10] = {static_cast<T>(3.83170e+00), static_cast<T>(7.01560e+00), static_cast<T>(1.01735e+01),
                               static_cast<T>(1.33237e+01), static_cast<T>(1.64705e+01), static_cast<T>(1.96120e+01),
                               static_cast<T>(2.27560e+01), static_cast<T>(2.58980e+01), static_cast<T>(2.90480e+01),
                               static_cast<T>(3.21926e+01)};
            const T bsj0chi = static_cast<T>(-0.402759);
            const T factor = static_cast<T>(0.179503);
            const T rc = static_cast<T>(3000.);
            const T rmax = static_cast<T>(50000.);
            const T chi = static_cast<T>(3.8317) / rc;
            const T shape = static_cast<T>(0.8);
            const T alpha = static_cast<T>(0.5);
            const T one = static_cast<T>(1.);
            const T c100 = static_cast<T>(100.);
            const int jb = jbb, jd = jdd, jt = jtt;
            // index 0 (J - 1 of J = JT = 1): outside WRF's arrays, read only into results that are not used
            cdudt[0] = static_cast<T>(0.);
            cdvdt[0] = static_cast<T>(0.);
            crho[0] = static_cast<T>(1.);
            for (int jj = 1; jj <= ilev; ++jj) {  // DO 5
                bu[jj] = static_cast<T>(0.);
                bd[jj] = static_cast<T>(0.);
                delpx[jj] = static_cast<T>(0.);
                delpy[jj] = static_cast<T>(0.);
                if (ct[jj] > static_cast<T>(0.)) crho[jj] = c100 * cpru[jj] / (rdry * ct[jj]);
            }
            for (int jj = 2; jj <= ilev; ++jj) {  // DO 15
                const T gz = grav / cpin * czf[jj];
                bu[jj] = (csu[jj] - cshat[jj] + static_cast<T>(0.608) * (cqu[jj] * (csu[jj] - gz) - cqhat[jj] * (cshat[jj] - gz))) /
                         ((cshat[jj] - gz) * (one + static_cast<T>(.608) * cqhat[jj])) * grav;
                bd[jj] = (csd[jj] - cshat[jj] + static_cast<T>(0.608) * (cqd[jj] * (csd[jj] - gz) - cqhat[jj] * (cshat[jj] - gz))) /
                         ((cshat[jj] - gz) * (one + static_cast<T>(.608) * cqhat[jj])) * grav;
            }
            T umn = static_cast<T>(0.), vmn = static_cast<T>(0.), cosa = static_cast<T>(0.), sina = static_cast<T>(0.);
            for (int jj = 1; jj <= ilev; ++jj) {  // DO 25
                if (jj >= jt && jj < jb) {
                    umn = umn + cu[jj] * cpru[jj] / ct[jj] * (czf[jj] - czf[jj + 1]);
                    vmn = vmn + cv[jj] * cpru[jj] / ct[jj] * (czf[jj] - czf[jj + 1]);
                }
            }
            {  // DO 30
                const T windmag = sqrt(umn * umn + vmn * vmn);
                if (windmag != static_cast<T>(0.)) {
                    cosa = umn / windmag;
                    sina = vmn / windmag;
                }
            }
            for (int jj = 1; jj <= ilev; ++jj) {  // DO 35
                if (jj > 1) {
                    rhohat[jj] = alpha * crho[jj - 1] + (one - alpha) * crho[jj];
                    dzf[jj] = cz[jj - 1] - cz[jj];
                    uhat[jj] = alpha * cu[jj - 1] + (one - alpha) * cu[jj];
                    vhat[jj] = alpha * cv[jj - 1] + (one - alpha) * cv[jj];
                } else {
                    rhohat[jj] = crho[jj];
                    dzf[jj] = czf[jj] - cz[jj];
                    uhat[jj] = cu[jj];
                    vhat[jj] = cv[jj];
                }
                uc[jj] = uhat[jj];
                vc[jj] = vhat[jj];
            }
            for (int jj = 1; jj <= ilev; ++jj)  // DO 40
                cdz[jj] = jj < ilev ? czf[jj] - czf[jj + 1] : cdp[ilev] * c100 / (crho[ilev] * grav);
            for (int jj = 1; jj <= ilev; ++jj) {  // DO 75: fractional cloud areas times GRAV
                au[jj] = jj >= jt ? cmu[jb] * c100 / grav / rhohat[jb] : static_cast<T>(0.);
                ad[jj] = jj >= jd ? -(cmd[jd] * c100 / grav / rhohat[jd]) : static_cast<T>(0.);
                ac[jj] = au[jj] + ad[jj];
                cw0[jj] = ac[jj] > static_cast<T>(0.) ? cmc[jj] * c100 / grav / (rhohat[jj] * ac[jj]) : static_cast<T>(0.);
            }
            for (int jj = 1; jj <= ilev; ++jj) {  // DO 80
                if (jj < ilev) {
                    acfl[jj] = alpha * ac[jj] + (one - alpha) * ac[jj + 1];
                    aufl[jj] = alpha * au[jj] + (one - alpha) * au[jj + 1];
                    adfl[jj] = alpha * ad[jj] + (one - alpha) * ad[jj + 1];
                    w0fl[jj] = alpha * cw0[jj] + (one - alpha) * cw0[jj + 1];
                } else {
                    acfl[jj] = alpha * ac[jj];
                    aufl[jj] = alpha * au[jj];
                    adfl[jj] = alpha * ad[jj];
                    w0fl[jj] = alpha * cw0[jj];
                }
            }
            for (int jj = 1; jj <= ilev; ++jj) {  // DO 250
                if (jj < jb && jj >= jt && acfl[jj] > static_cast<T>(0.)) {
                    d0[jj] = (cmc[jj + 1] - cmc[jj]) * c100 / grav / (crho[jj] * cdz[jj] * acfl[jj]);
                } else if (jj == jb && acfl[jj] > static_cast<T>(0.)) {
                    d0[jj] = -(cmc[jj] * c100 / grav / (crho[jj] * cdz[jj] * acfl[jj]));
                } else {
                    d0[jj] = static_cast<T>(0.);
                }
                if (jj < jb && jj >= jt && acfl[jj] > static_cast<T>(0.) && adfl[jj] > static_cast<T>(0.) &&
                    aufl[jj] > static_cast<T>(0.)) {
                    ce[jj] = one / (chi * chi) * adfl[jj] / (acfl[jj] * crho[jj]) *
                             (one / (aufl[jj] * cdz[jj]) * (cmu[jj] - cmu[jj + 1]) * c100 / grav -
                              one / (adfl[jj] * cdz[jj]) * (cmd[jj] - cmd[jj + 1]) * c100 / grav) *
                             shape / factor;
                } else {
                    ce[jj] = static_cast<T>(0.);
                }
            }
            for (int jj = 1; jj <= ilev; ++jj) {  // DO 275
                if (jj > 1) {
                    d0hat[jj] = alpha * d0[jj - 1] + (one - alpha) * d0[jj];
                    ehat[jj] = alpha * ce[jj - 1] + (one - alpha) * ce[jj];
                } else {
                    d0hat[jj] = (one - alpha) * d0[jj];
                    ehat[jj] = (one - alpha) * ce[jj];
                }
            }
            for (int jj = 1; jj <= ilev; ++jj) {  // DO 325: first harmonics in the thermodynamics
                if (jj >= jd && jj <= jb && au[jj] > static_cast<T>(0.) && ad[jj] > static_cast<T>(0.) &&
                    ac[jj] > static_cast<T>(0.)) {
                    w1[jj] = shape / factor * (cmu[jj] / au[jj] - cmd[jj] / ad[jj]) * c100 / grav * ad[jj] /
                             (ac[jj] * rhohat[jj]);
                    b1[jj] = shape / factor * (bu[jj] - bd[jj]) * ad[jj] / ac[jj];
                } else {
                    w1[jj] = static_cast<T>(0.);
                    b1[jj] = static_cast<T>(0.);
                }
            }
            for (int jj = 1; jj <= ilev; ++jj)  // DO 350 (B1FL: not used)
                w1fl[jj] = jj < ilev ? alpha * w1[jj] + (one - alpha) * w1[jj + 1] : alpha * w1[jj];
            // DO 500 - DO 700 for one Bessel mode N at a time (the modes are independent; DELPX / DELPY sum them in N
            // order at each level as WRF's DO 700 N, J)
            for (int n = 0; n < 10; ++n) {
                for (int jj = 1; jj <= ilev; ++jj) {  // DO 10, DO 500
                    cfx[jj] = static_cast<T>(0.);
                    cfy[jj] = static_cast<T>(0.);
                    if (jj < jb && jj >= jt && cmc[jj] > static_cast<T>(0.) && cmc[jj + 1] > static_cast<T>(0.)) {
                        const T lrm = libm::log(crho[jj - 1]), lr = libm::log(crho[jj]), lrp = libm::log(crho[jj + 1]);
                        const T zt = lrm / (cdz[jj] * dzf[jj]) - (one / dzf[jj] + one / dzf[jj + 1]) / cdz[jj] * lr +
                                     lrp / (cdz[jj] * dzf[jj + 1]);
                        const T y = (cw0[jj] - cw0[jj + 1]) * (w1[jj] - w1[jj + 1]) / (cdz[jj] * cdz[jj]) -
                                    w0fl[jj] * w1fl[jj] * zt;
                        cfx[jj] = static_cast<T>(2.) * crho[jj] / (bsj0[n] * bsj0[n]) *
                                  (d0[jj] * ce[jj] * (chi * chi) * cosa * alfa1[n] +
                                   static_cast<T>(2.) * w0fl[jj] * rc / (rmax * rmax) * bsj1[n] *
                                       ((ehat[jj] - ehat[jj + 1]) / cdz[jj] * chi * cosa * bsj0chi +
                                        (uhat[jj] - uhat[jj + 1]) / cdz[jj]) -
                                   (d0hat[jj] - d0hat[jj + 1]) / cdz[jj] * w1fl[jj] * cosa * (alfa2[n] - alfa1[n]) -
                                   static_cast<T>(2.) * y * cosa * alfa1[n] +
                                   (rhohat[jj] * b1[jj] - rhohat[jj + 1] * b1[jj + 1]) / (cdz[jj] * crho[jj]) * cosa * alfa1[n]);
                        cfy[jj] = static_cast<T>(2.) * crho[jj] / (bsj0[n] * bsj0[n]) *
                                  (d0[jj] * ce[jj] * (chi * chi) * sina * alfa1[n] +
                                   static_cast<T>(2.) * w0fl[jj] * rc / (rmax * rmax) * bsj1[n] *
                                       ((ehat[jj] - ehat[jj + 1]) / cdz[jj] * chi * sina * bsj0chi +
                                        (vhat[jj] - vhat[jj + 1]) / cdz[jj]) -
                                   (d0hat[jj] - d0hat[jj + 1]) / cdz[jj] * w1fl[jj] * sina * (alfa2[n] - alfa1[n]) -
                                   static_cast<T>(2.) * y * sina * alfa1[n] +
                                   (rhohat[jj] * b1[jj] - rhohat[jj + 1] * b1[jj + 1]) / (cdz[jj] * crho[jj]) * sina * alfa1[n]);
                    }
                }
                // DO 525
                caa[1] = static_cast<T>(0.);
                cbb[1] = one / dzf[2];
                ccc[1] = -(one / dzf[2]);
                caa[ilev] = one / dzf[ilev];
                cbb[ilev] = -(one / dzf[ilev]);
                ccc[ilev] = static_cast<T>(0.);
                cdx[1] = b1[2] * alfa1[n] * static_cast<T>(2.) * cosa / (bsj0[n] * bsj0[n]);
                cdx[ilev] = b1[ilev] * alfa1[n] * static_cast<T>(2.) * cosa / (bsj0[n] * bsj0[n]);
                cdy[1] = b1[2] * alfa1[n] * static_cast<T>(2.) * sina / (bsj0[n] * bsj0[n]);
                cdy[ilev] = b1[ilev] * alfa1[n] * static_cast<T>(2.) * sina / (bsj0[n] * bsj0[n]);
                for (int jj = 2; jj <= ilev - 1; ++jj) {  // DO 550
                    caa[jj] = one / (cdz[jj] * dzf[jj]);
                    cbb[jj] = -(one / cdz[jj] * (one / dzf[jj] + one / dzf[jj + 1]) + (tau[n] / rmax) * (tau[n] / rmax));
                    ccc[jj] = one / (cdz[jj] * dzf[jj + 1]);
                    cdx[jj] = cfx[jj];
                    cdy[jj] = cfy[jj];
                }
                ccc[1] = ccc[1] / cbb[1];  // DO 575
                cdx[1] = cdx[1] / cbb[1];
                cdy[1] = cdy[1] / cbb[1];
                for (int jj = 2; jj <= ilev; ++jj) {  // DO 600
                    ccc[jj] = ccc[jj] / (cbb[jj] - caa[jj] * ccc[jj - 1]);
                    cdx[jj] = (cdx[jj] - caa[jj] * cdx[jj - 1]) / (cbb[jj] - caa[jj] * ccc[jj - 1]);
                    cdy[jj] = (cdy[jj] - caa[jj] * cdy[jj - 1]) / (cbb[jj] - caa[jj] * ccc[jj - 1]);
                }
                for (int jj = ilev - 1; jj >= 1; --jj) {  // DO 650
                    cdx[jj] = cdx[jj] - ccc[jj] * cdx[jj + 1];
                    cdy[jj] = cdy[jj] - ccc[jj] * cdy[jj + 1];
                }
                for (int jj = 1; jj <= ilev; ++jj) {  // DO 700
                    delpx[jj] = delpx[jj] + cdx[jj] * bsj1[n];
                    delpy[jj] = delpy[jj] + cdy[jj] * bsj1[n];
                }
            }
            for (int jj = 1; jj <= ilev; ++jj) {  // DO 850
                delpx[jj] = delpx[jj] / rc;
                delpy[jj] = delpy[jj] / rc;
                delpx[jj] = acfl[jj] * delpx[jj] / crho[jj];
                delpy[jj] = acfl[jj] * delpy[jj] / crho[jj];
            }
            for (int jj = ilev - 1; jj >= 1; --jj) {  // DO 875: the cloud mean wind
                if (cmc[jj] > static_cast<T>(0.) && cmc[jj + 1] > static_cast<T>(0.) && jj > jt && jj < jb) {
                    const T ec = ceu[jj] + ced[jj];
                    uc[jj] = uc[jj + 1] + crho[jj] * cdz[jj] / ((cmc[jj] + cmc[jj + 1]) * static_cast<T>(0.5) * c100 / grav) *
                                              (ec * (cu[jj] - uc[jj + 1]) - delpx[jj]);
                    vc[jj] = vc[jj + 1] + crho[jj] * cdz[jj] / ((cmc[jj] + cmc[jj + 1]) * static_cast<T>(0.5) * c100 / grav) *
                                              (ec * (cv[jj] - vc[jj + 1]) - delpy[jj]);
                }
            }
            const T rnu = static_cast<T>(0.);
            for (int jj = 1; jj <= ilev; ++jj) {  // DO 950
                if (jj >= jt && jj <= jb) {
                    // RNU = 0 (the term only adds a signed zero); DUDT(J-1) of J = JT = 1 (LTOP = KTE) is outside
                    // WRF's array (cdudt[0], zero here); UHAT(1) is not read after this
                    uhat[jj] = uhat[jj] + rnu * alpha * dt * cdudt[jj - 1];
                    vhat[jj] = vhat[jj] + rnu * alpha * dt * cdvdt[jj - 1];
                    if (jj == jt) {
                        const T den = one / (one + rnu * alpha * cmc[jj + 1] * dt / cdp[jj]);
                        cdudt[jj] = den * cmc[jj + 1] * (uc[jj + 1] - uhat[jj + 1]) / cdp[jj];
                        cdvdt[jj] = den * cmc[jj + 1] * (vc[jj + 1] - vhat[jj + 1]) / cdp[jj];
                    } else if (jj < jb) {
                        const T den = one / (one + rnu * alpha * cmc[jj + 1] * dt / cdp[jj]);
                        cdudt[jj] = den * (cmc[jj + 1] * (uc[jj + 1] - uhat[jj + 1]) - cmc[jj] * (uc[jj] - uhat[jj])) / cdp[jj];
                        cdvdt[jj] = den * (cmc[jj + 1] * (vc[jj + 1] - vhat[jj + 1]) - cmc[jj] * (vc[jj] - vhat[jj])) / cdp[jj];
                    } else {
                        cdudt[jj] = -(one / dsubcld * cmc[jj] * (uc[jj] - uhat[jj]));
                        cdvdt[jj] = -(one / dsubcld * cmc[jj] * (vc[jj] - vhat[jj]));
                    }
                }
            }
            for (int kq = 1; kq <= kte; ++kq) {
                rucuten(i, j, kq - 1) = cdudt[kte - kq + 1];
                rvcuten(i, j, kq - 1) = cdvdt[kte - kq + 1];
            }
        }
    }
};

template <typename T, int MaxLevels>
struct MSKFRunner {
    MSKFKernel<T> k;
    static constexpr std::size_t kWorkBytes = kColumnBytes<T, MaxLevels>;
    CPPWRF_HD void operator()(int i, int j) const { k.template compute<MaxLevels>(i, j); }
    void set_work(const ColumnWork& w) { k.work = w; }
};

// the column work bytes of run_mskf (the suite reserves them outside the time loop)
template <typename T>
inline std::size_t mskf_work_bytes(int nz) {
    return nz <= 48 ? kColumnBytes<T, 48> : kColumnBytes<T, kMaxLevels>;
}

template <typename T>
void run_mskf(MSKFKernel<T> kernel, const Range2D& range);
} // namespace cppwrf::phys::cu_mskf
