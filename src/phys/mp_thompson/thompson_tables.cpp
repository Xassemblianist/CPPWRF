// SPDX-License-Identifier: GPL-3.0-or-later
#include "phys/mp_thompson/thompson_tables.hpp"
#include <initializer_list>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <type_traits>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#if defined(_OPENMP)
#include <omp.h>
#endif

namespace cppwrf::phys::mp_thompson {

namespace {

template <typename F>
void par_for_host(int n_items, F&& fn) {
#if defined(_OPENMP)
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < n_items; ++i) fn(i);
#else
    const int nthreads = 4;
    std::atomic<int> next{0};
    std::vector<std::thread> threads;
    threads.reserve(nthreads);
    for (int t = 0; t < nthreads; ++t) {
        threads.emplace_back([&]() {
            for (;;) {
                int i = next.fetch_add(1, std::memory_order_relaxed);
                if (i >= n_items) break;
                fn(i);
            }
        });
    }
    for (auto& th : threads) th.join();
#endif
}

// Exact Numerical Recipes / Lancaster approximations matching module_mp_thompson.F

// thompson_init's bin edges xDx(n) = DEXP((n-1)/nb*DLOG(xDx(nb+1)/xDx(1)) + DLOG(xDx(1))), n = 2..nb: gfortran
// folds the logs and the whole terms n = 2, nb - 1 and nb at compile time with MPFR (correctly rounded; WRF's
// module_mp_thompson.o stores them as constants: the pairs xDx(1:2) and xDx(nb-1:nb) of every species) and runs
// n = 3..nb-2 through libmvec's 2-lane exp. cr_*: correctly rounded via long double. n: the 0-based C++ index.
inline double cr_log(double x) { return double(logl((long double)x)); }
inline double cr_exp(double x) { return double(expl((long double)x)); }
inline double bin_exp(double a, int n, int nb) { return (n == 1 || n >= nb - 2) ? cr_exp(a) : vm::exp(a, 0, 2); }

// GAMMLN, WGAMMA, GSER, GCF, GAMMP: REAL functions (R: float in WRF real*4, double in the FP64 builds); GAMMLN's
// series in DOUBLE PRECISION, its result REAL; WGAMMA = EXP(GAMMLN(y)) in REAL
template <typename R>
inline R gammln(R xx) {
    constexpr double stp = 2.5066282746310005;
    constexpr double cof[6] = {
        76.18009172947146, -86.50532032941677,
        24.01409824083091, -1.231739572450155,
        0.1208650973866179e-2, -0.5395239384953e-5
    };
    double x = xx;
    double y = x;
    double tmp = x + 5.5;
    tmp = (x + 0.5) * libm::log(tmp) - tmp;
    double ser = 1.000000000190015;
    for (int j = 0; j < 6; ++j) {
        y += 1.0;
        ser += cof[j] / y;
    }
    return R(tmp + libm::log(stp * ser / x));
}

template <typename R>
inline R wgamma(R y) {
    return libm::exp(gammln(y));
}

template <typename R>
inline void gser(R& gamser, R a, R x, R& gln) {
    constexpr int itmax = 100;
    constexpr R geps = R(3.0e-7);
    gln = gammln(a);
    if (x <= R(0.0)) {
        gamser = R(0.0);
        return;
    }
    R ap = a;
    R sum = R(1.0) / a;
    R del = sum;
    for (int n = 1; n <= itmax; ++n) {
        ap += R(1.0);
        del = del * x / ap;
        sum += del;
        if (std::abs(del) < std::abs(sum) * geps) break;
    }
    gamser = sum * libm::exp(-x + a * libm::log(x) - gln);
}

template <typename R>
inline void gcf(R& gammcf, R a, R x, R& gln) {
    constexpr int itmax = 100;
    constexpr R geps = R(3.0e-7);
    constexpr R fpmin = R(1.0e-30);
    gln = gammln(a);
    R b = x + R(1.0) - a;
    R c = R(1.0) / fpmin;
    R d = R(1.0) / b;
    R h = d;
    for (int i = 1; i <= itmax; ++i) {
        R an = -R(i) * (R(i) - a);
        b += R(2.0);
        d = an * d + b;
        if (std::abs(d) < fpmin) d = fpmin;
        c = b + an / c;
        if (std::abs(c) < fpmin) c = fpmin;
        d = R(1.0) / d;
        R del = d * c;
        h = h * del;
        if (std::abs(del - R(1.0)) < geps) break;
    }
    gammcf = libm::exp(-x + a * libm::log(x) - gln) * h;
}

template <typename R>
inline R gammp(R a, R x) {
    if (x < R(0.0) || a <= R(0.0)) return R(0.0);
    R gamser = 0, gammcf = 0, gln = 0;
    if (x < a + R(1.0)) {
        gser(gamser, a, x, gln);
        return gamser;
    } else {
        gcf(gammcf, a, x, gln);
        return R(1.0) - gammcf;
    }
}

// WRF's table files (module_mp_thompson.F qr_acr_qg / qr_acr_qs / freezeH2O write them on the first
// run and read them afterwards): Fortran unformatted sequential records of doubles, either byte
// order (detected from the first record marker). Returns false if a file is missing or malformed.
template <typename Real>
bool read_wrf_dat(const std::string& path, std::initializer_list<std::vector<Real>*> arrays) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    bool swap = false, first = true;
    for (std::vector<Real>* a : arrays) {
        uint32_t m;
        if (!f.read(reinterpret_cast<char*>(&m), 4)) return false;
        const uint64_t want = uint64_t(a->size()) * 8;
        if (first) {
            if (m != want) {
                if (__builtin_bswap32(m) != want) return false;
                swap = true;
            }
            first = false;
        }
        if ((swap ? __builtin_bswap32(m) : m) != want) return false;
        std::vector<uint64_t> buf(a->size());
        if (!f.read(reinterpret_cast<char*>(buf.data()), std::streamsize(want))) return false;
        for (size_t i = 0; i < buf.size(); ++i) {
            uint64_t u = swap ? __builtin_bswap64(buf[i]) : buf[i];
            double x;
            std::memcpy(&x, &u, 8);
            (*a)[i] = Real(x);
        }
        f.read(reinterpret_cast<char*>(&m), 4);
    }
    return bool(f);
}

// table_ccnAct: CCN_ACTIVATE.BIN is one unformatted sequential record of REAL(KIND=R4SIZE) tnccn_act (either
// byte order); WRF stops when the file cannot be opened or read
template <typename Real>
void read_ccn_act(const std::string& path, std::vector<Real>& t) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("module_mp_thompson: error opening CCN_ACTIVATE.BIN (" + path + ")");
    const uint32_t want = uint32_t(t.size() * 4);
    uint32_t m = 0;
    std::vector<uint32_t> buf(t.size());
    bool ok = bool(f.read(reinterpret_cast<char*>(&m), 4));
    const bool swap = ok && m != want && __builtin_bswap32(m) == want;
    ok = ok && (swap || m == want) && f.read(reinterpret_cast<char*>(buf.data()), std::streamsize(want));
    if (!ok) throw std::runtime_error("module_mp_thompson: error reading CCN_ACTIVATE.BIN (" + path + ")");
    for (size_t i = 0; i < buf.size(); ++i) {
        const uint32_t u = swap ? __builtin_bswap32(buf[i]) : buf[i];
        float x;
        std::memcpy(&x, &u, 4);
        t[i] = Real(x);
    }
}

} // namespace

template <typename Real>
void init_thompson_tables(ThompsonHostTables<Real>& tables, const std::string& wrf_dat_dir, bool force_read,
                          bool aerosol_aware) {
    // thompson_init: table_ccnAct (is_aerosol_aware) comes before the collection and freezing tables
    if (aerosol_aware)
        read_ccn_act<Real>((wrf_dat_dir.empty() ? std::string(".") : wrf_dat_dir) + "/CCN_ACTIVATE.BIN",
                           tables.tnccn_act);
    // WRF's own table files when present (bit-identical to the WRF run), else computed below
    const std::string dd = wrf_dat_dir.empty() ? std::string(".") : wrf_dat_dir;
    const bool have_qg = !wrf_dat_dir.empty() && read_wrf_dat<double>(dd + "/qr_acr_qg_V4.dat", {&tables.tcg_racg, &tables.tmr_racg, &tables.tcr_gacr, &tables.tnr_racg, &tables.tnr_gacr});
    const bool have_qs = !wrf_dat_dir.empty() && read_wrf_dat<double>(dd + "/qr_acr_qsV2.dat", {&tables.tcs_racs1, &tables.tmr_racs1, &tables.tcs_racs2, &tables.tmr_racs2, &tables.tcr_sacr1, &tables.tms_sacr1, &tables.tcr_sacr2, &tables.tms_sacr2, &tables.tnr_racs1, &tables.tnr_racs2, &tables.tnr_sacr1, &tables.tnr_sacr2});
    const bool have_fz = !wrf_dat_dir.empty() && read_wrf_dat<double>(dd + "/freezeH2O.dat", {&tables.tpi_qrfz, &tables.tni_qrfz, &tables.tpg_qrfz, &tables.tnr_qrfz, &tables.tpi_qcfz, &tables.tni_qcfz});
    // force_read_thompson: qr_acr_qg, qr_acr_qs and freezeH2O (in this order) stop instead of computing
    auto require = [&](bool have, const char* name) {
        if (!force_read || have) return;
        const std::string why = std::ifstream(dd + "/" + name) ? "Error reading " : "Non-existent ";
        throw std::runtime_error(why + name + ". Aborting because force_read_thompson is .true.");
    };
    require(have_qg, "qr_acr_qg_V4.dat");
    require(have_qs, "qr_acr_qsV2.dat");
    require(have_fz, "freezeH2O.dat");
    // Physical constants matching module_mp_thompson.F. R is WRF's REAL (float in WRF real*4, double in the FP64
    // builds): REAL parameters and REAL expressions in R (gfortran folds the products of REAL constants in REAL,
    // left to right), DOUBLE PRECISION in double, a REAL operand widened where it meets a double one. With
    // R = double every expression below is the FP64 code as before.
    using R = Real;
    constexpr bool r4 = std::is_same_v<R, float>;
    // a power of REAL constants folded by gfortran (correctly rounded); the FP64 builds keep glibc's pow
    auto fold_pow = [](R a, R b) -> R {
        if constexpr (r4) return R(powl((long double)a, (long double)b));
        else return libm::pow(a, b);
    };
    constexpr R PI = R(3.1415926536);
    constexpr R rho_w = R(1000.0);
    constexpr R rho_i = R(890.0);
    constexpr R rho_g_val = R(400.0); // idx_bg1 = 5, rho_g(5) = 400.0

    constexpr R am_r = PI * rho_w / R(6.0);
    constexpr R bm_r = R(3.0);
    constexpr R am_s = R(0.069);
    constexpr R bm_s = R(2.0);
    constexpr R am_g = PI * rho_g_val / R(6.0);
    constexpr R bm_g = R(3.0);
    constexpr R am_i = PI * rho_i / R(6.0);
    constexpr R bm_i = R(3.0);

    constexpr R av_r = R(4854.0);
    constexpr R bv_r = R(1.0);
    constexpr R av_s = R(40.0);
    constexpr R bv_s = R(0.55);
    constexpr R fv_s = R(100.0);
    constexpr R av_g = R(442.0); // av_g_old
    constexpr R bv_g = R(0.89);  // bv_g_old
    constexpr R bv_i = R(1.0);
    constexpr R bv_c = R(2.0);

    constexpr R C_cube = R(0.5);
    constexpr R C_sqrd = R(0.15);
    constexpr R Ef_rs = R(0.95);
    constexpr R Ef_rg = R(0.75);

    constexpr R Sc = R(0.632);
    constexpr R lsub = R(2.834e6);
    constexpr R lvap0 = R(2.5e6);
    constexpr R lfus = lsub - lvap0;
    constexpr R olfus = R(1.0) / lfus;

    constexpr R xm0i = R(1.e-12);
    constexpr R D0c = R(1.e-6);
    constexpr R D0r = R(50.e-6);
    constexpr R D0s = R(300.e-6);
    constexpr R D0g = R(350.e-6);

    constexpr R mu_r = R(0.0);
    constexpr R mu_g = R(0.0);
    constexpr R mu_i = R(0.0);
    constexpr R mu_s = R(0.6357);
    constexpr R Kap0 = R(490.6);
    constexpr R Kap1 = R(17.46);
    constexpr R Lam0 = R(20.78);
    constexpr R Lam1 = R(3.29);

    constexpr R sa[10] = {R(5.065339), R(-0.062659), R(-3.032362), R(0.029469), R(-0.000285),
                          R(0.31255), R(0.000204), R(0.003199), R(0.0), R(-0.015952)};
    constexpr R sb[10] = {R(0.476221), R(-0.015896), R(0.165977), R(0.007468), R(-0.000141),
                          R(0.060366), R(0.000079), R(0.000594), R(0.0), R(-0.003577)};
    // a REAL literal or REAL value inside a DOUBLE PRECISION expression: widened
    auto d = [](R x) { return double(x); };

    // Table coordinates initialization
    const double r_c_init[ntb_c] = {
        1.e-6,2.e-6,3.e-6,4.e-6,5.e-6,6.e-6,7.e-6,8.e-6,9.e-6,
        1.e-5,2.e-5,3.e-5,4.e-5,5.e-5,6.e-5,7.e-5,8.e-5,9.e-5,
        1.e-4,2.e-4,3.e-4,4.e-4,5.e-4,6.e-4,7.e-4,8.e-4,9.e-4,
        1.e-3,2.e-3,3.e-3,4.e-3,5.e-3,6.e-3,7.e-3,8.e-3,9.e-3,
        1.e-2
    };
    for (int i = 0; i < ntb_c; ++i) tables.r_c[i] = Real(r_c_init[i]);

    const double r_i_init[ntb_i] = {
        1.e-10,2.e-10,3.e-10,4.e-10,
        5.e-10,6.e-10,7.e-10,8.e-10,9.e-10,
        1.e-9,2.e-9,3.e-9,4.e-9,5.e-9,6.e-9,7.e-9,8.e-9,9.e-9,
        1.e-8,2.e-8,3.e-8,4.e-8,5.e-8,6.e-8,7.e-8,8.e-8,9.e-8,
        1.e-7,2.e-7,3.e-7,4.e-7,5.e-7,6.e-7,7.e-7,8.e-7,9.e-7,
        1.e-6,2.e-6,3.e-6,4.e-6,5.e-6,6.e-6,7.e-6,8.e-6,9.e-6,
        1.e-5,2.e-5,3.e-5,4.e-5,5.e-5,6.e-5,7.e-5,8.e-5,9.e-5,
        1.e-4,2.e-4,3.e-4,4.e-4,5.e-4,6.e-4,7.e-4,8.e-4,9.e-4,
        1.e-3
    };
    for (int i = 0; i < ntb_i; ++i) tables.r_i[i] = Real(r_i_init[i]);

    for (int i = 0; i < ntb_r; ++i) tables.r_r[i] = Real(r_c_init[i]);
    for (int i = 0; i < ntb_g; ++i) tables.r_g[i] = Real(r_c_init[i]);
    for (int i = 0; i < ntb_s; ++i) tables.r_s[i] = Real(r_c_init[i]);

    const double N0r_exp_init[ntb_r1] = {
        1.e6,2.e6,3.e6,4.e6,5.e6,6.e6,7.e6,8.e6,9.e6,
        1.e7,2.e7,3.e7,4.e7,5.e7,6.e7,7.e7,8.e7,9.e7,
        1.e8,2.e8,3.e8,4.e8,5.e8,6.e8,7.e8,8.e8,9.e8,
        1.e9,2.e9,3.e9,4.e9,5.e9,6.e9,7.e9,8.e9,9.e9,
        1.e10
    };
    for (int i = 0; i < ntb_r1; ++i) tables.N0r_exp[i] = Real(N0r_exp_init[i]);

    const double N0g_exp_init[ntb_g1] = {
        1.e2,2.e2,3.e2,4.e2,5.e2,6.e2,7.e2,8.e2,9.e2,
        1.e3,2.e3,3.e3,4.e3,5.e3,6.e3,7.e3,8.e3,9.e3,
        1.e4,2.e4,3.e4,4.e4,5.e4,6.e4,7.e4,8.e4,9.e4,
        1.e5,2.e5,3.e5,4.e5,5.e5,6.e5,7.e5,8.e5,9.e5,
        1.e6
    };
    for (int i = 0; i < ntb_g1; ++i) tables.N0g_exp[i] = Real(N0g_exp_init[i]);

    const double Nt_i_init[ntb_i1] = {
        1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,
        1.e1,2.e1,3.e1,4.e1,5.e1,6.e1,7.e1,8.e1,9.e1,
        1.e2,2.e2,3.e2,4.e2,5.e2,6.e2,7.e2,8.e2,9.e2,
        1.e3,2.e3,3.e3,4.e3,5.e3,6.e3,7.e3,8.e3,9.e3,
        1.e4,2.e4,3.e4,4.e4,5.e4,6.e4,7.e4,8.e4,9.e4,
        1.e5,2.e5,3.e5,4.e5,5.e5,6.e5,7.e5,8.e5,9.e5,
        1.e6
    };
    for (int i = 0; i < ntb_i1; ++i) tables.Nt_i[i] = Real(Nt_i_init[i]);
    for (int i = 0; i < ntb_IN; ++i) tables.Nt_IN[i] = Real(Nt_i_init[i]);

    const double Tc_init[ntb_t] = {-0.01, -5., -10., -15., -20., -25., -30., -35., -40.};
    for (int i = 0; i < ntb_t; ++i) tables.Tc[i] = Real(Tc_init[i]);

    // Exponents and Gamma calculations (all REAL)
    R Sc3 = fold_pow(Sc, R(1.0) / R(3.0));
    R D0i = fold_pow(xm0i / am_i, R(1.0) / bm_i);
    R xm0s = am_s * fold_pow(D0s, bm_s);
    R xm0g = (PI * R(800.0) / R(6.0)) * fold_pow(D0g, bm_g); // am_g(NRHG) with rho_g=800

    R cce[5][15], ccg[5][15], ocg1[15], ocg2[15];
    for (int n = 0; n < 15; ++n) {
        int n_1 = n + 1;
        cce[0][n] = R(n_1) + R(1.0);
        cce[1][n] = bm_r + R(n_1) + R(1.0);
        cce[2][n] = bm_r + R(n_1) + R(4.0);
        cce[3][n] = R(n_1) + bv_c + R(1.0);
        cce[4][n] = bm_r + R(n_1) + bv_c + R(1.0);
        for (int k = 0; k < 5; ++k) ccg[k][n] = wgamma(cce[k][n]);
        ocg1[n] = R(1.0) / ccg[0][n];
        ocg2[n] = R(1.0) / ccg[1][n];
    }

    R cie[7], cig[7];
    cie[0] = mu_i + R(1.0);
    cie[1] = bm_i + mu_i + R(1.0);
    cie[2] = bm_i + mu_i + bv_i + R(1.0);
    cie[3] = mu_i + bv_i + R(1.0);
    cie[4] = mu_i + R(2.0);
    cie[5] = bm_i * R(0.5) + mu_i + bv_i + R(1.0);
    cie[6] = bm_i * R(0.5) + mu_i + R(1.0);
    for (int i = 0; i < 7; ++i) cig[i] = wgamma(cie[i]);
    R oig1 = R(1.0) / cig[0];
    R oig2 = R(1.0) / cig[1];
    R obmi = R(1.0) / bm_i;

    R cre[13], crg[13];
    cre[0] = bm_r + R(1.0);
    cre[1] = mu_r + R(1.0);
    cre[2] = bm_r + mu_r + R(1.0);
    cre[3] = bm_r * R(2.0) + mu_r + R(1.0);
    cre[4] = mu_r + bv_r + R(1.0);
    cre[5] = bm_r + mu_r + bv_r + R(1.0);
    cre[6] = bm_r * R(0.5) + mu_r + bv_r + R(1.0);
    cre[7] = bm_r + mu_r + bv_r + R(3.0);
    cre[8] = mu_r + bv_r + R(3.0);
    cre[9] = mu_r + R(2.0);
    cre[10] = R(0.5) * (bv_r + R(5.0) + R(2.0) * mu_r);
    cre[11] = bm_r * R(0.5) + mu_r + R(1.0);
    cre[12] = bm_r * R(2.0) + mu_r + bv_r + R(1.0);
    for (int n = 0; n < 13; ++n) crg[n] = wgamma(cre[n]);
    R obmr = R(1.0) / bm_r;
    R ore1 = R(1.0) / cre[0];
    R org1 = R(1.0) / crg[0];
    R org2 = R(1.0) / crg[1];
    R org3 = R(1.0) / crg[2];

    R cse[17], csg[17];
    cse[0] = bm_s + R(1.0);
    cse[1] = bm_s + R(2.0);
    cse[2] = bm_s * R(2.0);
    cse[3] = bm_s + bv_s + R(1.0);
    cse[4] = bm_s * R(2.0) + bv_s + R(1.0);
    cse[5] = bm_s * R(2.0) + R(1.0);
    cse[6] = bm_s + mu_s + R(1.0);
    cse[7] = bm_s + mu_s + R(2.0);
    cse[8] = bm_s + mu_s + R(3.0);
    cse[9] = bm_s + mu_s + bv_s + R(1.0);
    cse[10] = bm_s * R(2.0) + mu_s + bv_s + R(1.0);
    cse[11] = bm_s * R(2.0) + mu_s + R(1.0);
    cse[12] = bv_s + R(2.0);
    cse[13] = bm_s + bv_s;
    cse[14] = mu_s + R(1.0);
    cse[15] = R(1.0) + (R(1.0) + bv_s) / R(2.0);
    cse[16] = bm_s + bv_s + R(2.0);
    for (int n = 0; n < 17; ++n) csg[n] = wgamma(cse[n]);
    R oams = R(1.0) / am_s;
    R obms = R(1.0) / bm_s;
    R ocms = libm::pow(oams, obms);

    R cge[12], cgg[12];
    cge[0] = bm_g + R(1.0);
    cge[1] = mu_g + R(1.0);
    cge[2] = bm_g + mu_g + R(1.0);
    cge[3] = bm_g * R(2.0) + mu_g + R(1.0);
    cge[4] = bm_g * R(2.0) + mu_g + bv_g + R(1.0);
    cge[5] = bm_g + mu_g + bv_g + R(1.0);
    cge[6] = bm_g * R(0.5) + mu_g + bv_g + R(1.0);
    cge[7] = mu_g + bv_g + R(1.0);
    cge[8] = mu_g + bv_g + R(3.0);
    cge[9] = mu_g + R(2.0);
    cge[10] = R(0.5) * (bv_g + R(5.0) + R(2.0) * mu_g);
    cge[11] = bm_g * R(0.5) + mu_g + R(1.0);
    for (int n = 0; n < 12; ++n) cgg[n] = wgamma(cge[n]);
    R oamg = R(1.0) / am_g;
    R obmg = R(1.0) / bm_g;
    R ocmg = vm::pow(oamg, obmg, 4, 9);  // the m = 1..9 loop: libmvec (4 lanes in real*4, 2 in real*8), m = 5
    R oge1 = R(1.0) / cge[0];
    R ogg1 = R(1.0) / cgg[0];
    R ogg2 = R(1.0) / cgg[1];
    R ogg3 = R(1.0) / cgg[2];

    // Precomputed constants (REAL; products of REAL constants folded left to right)
    tables.t1_qr_qc = Real(PI * R(0.25) * av_r * crg[8]);
    tables.t1_qr_qi = Real(PI * R(0.25) * av_r * crg[8]);
    tables.t2_qr_qi = Real(PI * R(0.25) * am_r * av_r * crg[7]);
    tables.t1_qs_qc = Real(PI * R(0.25) * av_s);
    tables.t1_qs_qi = Real(PI * R(0.25) * av_s);
    tables.t1_qr_ev = Real(R(0.78) * crg[9]);
    tables.t2_qr_ev = Real(R(0.308) * Sc3 * std::sqrt(av_r) * crg[10]);
    tables.t1_qs_sd = Real(0.86);
    tables.t2_qs_sd = Real(R(0.28) * Sc3 * std::sqrt(av_s));
    tables.t1_qs_me = Real(PI * R(4.0) * C_sqrd * olfus * R(0.86));
    tables.t2_qs_me = Real(PI * R(4.0) * C_sqrd * olfus * R(0.28) * Sc3 * std::sqrt(av_s));
    tables.t1_qg_sd = Real(R(0.86) * cgg[9]);
    tables.t2_qg_sd = Real(R(0.28) * Sc3 * std::sqrt(av_g) * cgg[10]);
    tables.t1_qg_me = Real(PI * R(4.0) * C_cube * olfus * R(0.86) * cgg[9]);
    tables.t2_qg_me = Real(PI * R(4.0) * C_cube * olfus * R(0.28) * Sc3 * std::sqrt(av_g) * cgg[10]);

    // Index rounding constants
    tables.nic2 = static_cast<int>(std::round(libm::log10(tables.r_c[0])));
    tables.nii2 = static_cast<int>(std::round(libm::log10(tables.r_i[0])));
    tables.nii3 = static_cast<int>(std::round(libm::log10(tables.Nt_i[0])));
    tables.nir2 = static_cast<int>(std::round(libm::log10(tables.r_r[0])));
    tables.nir3 = static_cast<int>(std::round(libm::log10(tables.N0r_exp[0])));
    tables.nis2 = static_cast<int>(std::round(libm::log10(tables.r_s[0])));
    tables.nig2 = static_cast<int>(std::round(libm::log10(tables.r_g[0])));
    tables.nig3 = static_cast<int>(std::round(libm::log10(tables.N0g_exp[0])));
    tables.niIN2 = static_cast<int>(std::round(libm::log10(tables.Nt_IN[0])));

    // Bins (DOUBLE PRECISION; the seeds D0c*1.0d0 etc. are the REAL parameters widened)
    double Dc[nbc], dtc[nbc];
    Dc[0] = d(D0c);
    dtc[0] = d(D0c);
    for (int n = 1; n < nbc; ++n) {
        Dc[n] = Dc[n - 1] + 1.0e-6;
        dtc[n] = Dc[n] - Dc[n - 1];
    }

    double xDx[nbins + 1];
    xDx[0] = d(D0i);
    xDx[nbi] = 2.0 * d(D0s);
    for (int n = 1; n < nbi; ++n) {
        xDx[n] = bin_exp((double(n) / double(nbi)) * cr_log(xDx[nbi] / xDx[0]) + cr_log(xDx[0]), n, nbi);
    }
    double Di[nbi], dti[nbi];
    for (int n = 0; n < nbi; ++n) {
        Di[n] = std::sqrt(xDx[n] * xDx[n + 1]);
        dti[n] = xDx[n + 1] - xDx[n];
    }

    xDx[0] = D0r;
    xDx[nbr] = 0.005;
    for (int n = 1; n < nbr; ++n) {
        xDx[n] = bin_exp((double(n) / double(nbr)) * cr_log(xDx[nbr] / xDx[0]) + cr_log(xDx[0]), n, nbr);
    }
    double Dr[nbr], dtr[nbr];
    for (int n = 0; n < nbr; ++n) {
        Dr[n] = std::sqrt(xDx[n] * xDx[n + 1]);
        dtr[n] = xDx[n + 1] - xDx[n];
    }

    xDx[0] = D0s;
    xDx[nbs] = 0.02;
    for (int n = 1; n < nbs; ++n) {
        xDx[n] = bin_exp((double(n) / double(nbs)) * cr_log(xDx[nbs] / xDx[0]) + cr_log(xDx[0]), n, nbs);
    }
    double Ds[nbs], dts[nbs];
    for (int n = 0; n < nbs; ++n) {
        Ds[n] = std::sqrt(xDx[n] * xDx[n + 1]);
        dts[n] = xDx[n + 1] - xDx[n];
    }

    xDx[0] = D0g;
    xDx[nbg] = 0.05;
    for (int n = 1; n < nbg; ++n) {
        xDx[n] = bin_exp((double(n) / double(nbg)) * cr_log(xDx[nbg] / xDx[0]) + cr_log(xDx[0]), n, nbg);
    }
    double Dg[nbg], dtg[nbg];
    for (int n = 0; n < nbg; ++n) {
        Dg[n] = std::sqrt(xDx[n] * xDx[n + 1]);
        dtg[n] = xDx[n + 1] - xDx[n];
    }

    xDx[0] = 1.0;
    xDx[nbc] = 3000.0;
    for (int n = 1; n < nbc; ++n) {
        xDx[n] = bin_exp((double(n) / double(nbc)) * cr_log(xDx[nbc] / xDx[0]) + cr_log(xDx[0]), n, nbc);
    }
    double t_Nc[nbc];
    for (int n = 0; n < nbc; ++n) {
        t_Nc[n] = std::sqrt(xDx[n] * xDx[n + 1]) * 1.0e6;
    }
    tables.nic1 = static_cast<int>(libm::log(t_Nc[nbc - 1] / t_Nc[0]));

    for (int k = 0; k < 5; ++k) {
        for (int n = 0; n < 15; ++n) {
            tables.cce[k][n] = Real(cce[k][n]);
            tables.ccg[k][n] = Real(ccg[k][n]);
        }
    }
    for (int n = 0; n < 15; ++n) {
        tables.ocg1[n] = Real(ocg1[n]);
        tables.ocg2[n] = Real(ocg2[n]);
    }
    for (int i = 0; i < 7; ++i) {
        tables.cie[i] = Real(cie[i]);
        tables.cig[i] = Real(cig[i]);
    }
    tables.oig1 = Real(oig1); tables.oig2 = Real(oig2); tables.obmi = Real(obmi);

    for (int i = 0; i < 13; ++i) {
        tables.cre[i] = Real(cre[i]);
        tables.crg[i] = Real(crg[i]);
    }
    tables.ore1 = Real(ore1); tables.org1 = Real(org1); tables.org2 = Real(org2); tables.org3 = Real(org3); tables.obmr = Real(obmr);

    for (int i = 0; i < 17; ++i) {
        tables.cse[i] = Real(cse[i]);
        tables.csg[i] = Real(csg[i]);
    }
    tables.oams = Real(oams); tables.obms = Real(obms); tables.ocms = Real(ocms);

    for (int i = 0; i < 12; ++i) {
        tables.cge[i] = Real(cge[i]);
        tables.cgg[i] = Real(cgg[i]);
    }
    tables.oamg = Real(oamg); tables.obmg = Real(obmg); tables.ocmg = Real(ocmg);
    tables.oge1 = Real(oge1); tables.ogg1 = Real(ogg1); tables.ogg2 = Real(ogg2); tables.ogg3 = Real(ogg3);

    for (int i = 0; i < 10; ++i) {
        tables.sa[i] = Real(sa[i]);
        tables.sb[i] = Real(sb[i]);
    }

    tables.Dr_0 = double(Dr[0]);
    tables.log_Dr = double(libm::log(Dr[nbr - 1] / Dr[0]));
    tables.inv_log_Dr = double(1.0 / libm::log(Dr[nbr - 1] / Dr[0]));
    tables.Ds_0 = double(Ds[0]);
    tables.log_Ds = double(libm::log(Ds[nbs - 1] / Ds[0]));
    tables.inv_log_Ds = double(1.0 / libm::log(Ds[nbs - 1] / Ds[0]));
    tables.t_Nc_0 = double(t_Nc[0]);
    tables.inv_nic1 = Real(1.0 / tables.nic1);

    tables.t1_qg_qc = Real(PI * R(0.25) * av_g * cgg[8]);
    tables.D0i = Real(D0i);
    tables.xm0s = Real(xm0s);
    tables.xm0g = Real(xm0g);

    auto view = tables.view();

    // 1. table_Efrw
    for (int j = 0; j < nbc; ++j) {
        for (int i = 0; i < nbr; ++i) {
            // DOUBLE PRECISION with REAL literals (widened) except the D ones
            double Ef_rw = 0.0;
            double p = Dc[j] / Dr[i];
            if (Dr[i] < d(R(50.e-6)) || Dc[j] < d(R(3.e-6))) {
                Ef_rw = 0.0;
            } else if (p > 0.25) {
                double X = Dc[j] * 1.0e6;
                if (Dr[i] < d(R(75.e-6))) {
                    Ef_rw = d(R(0.026794)) * X - d(R(0.20604));
                } else if (Dr[i] < d(R(125.e-6))) {
                    Ef_rw = -d(R(0.00066842)) * X * X + d(R(0.061542)) * X - d(R(0.37089));
                } else if (Dr[i] < d(R(175.e-6))) {
                    Ef_rw = d(R(4.091e-06)) * X * X * X * X - d(R(0.00030908)) * X * X * X
                          + d(R(0.0066237)) * X * X - d(R(0.0013687)) * X - d(R(0.073022));
                } else if (Dr[i] < d(R(250.e-6))) {
                    Ef_rw = d(R(9.6719e-5)) * X * X * X - d(R(0.0068901)) * X * X + d(R(0.17305)) * X - d(R(0.65988));
                } else if (Dr[i] < d(R(350.e-6))) {
                    Ef_rw = d(R(9.0488e-5)) * X * X * X - d(R(0.006585)) * X * X + d(R(0.16606)) * X - d(R(0.56125));
                } else {
                    Ef_rw = d(R(0.00010721)) * X * X * X - d(R(0.0072962)) * X * X + d(R(0.1704)) * X - d(R(0.46929));
                }
            } else {
                double vtr = d(R(-0.1021)) + d(R(4.932e3)) * Dr[i] - d(R(0.9551e6)) * Dr[i] * Dr[i]
                           + d(R(0.07934e9)) * Dr[i] * Dr[i] * Dr[i]
                           - d(R(0.002362e12)) * Dr[i] * Dr[i] * Dr[i] * Dr[i];
                double stokes = Dc[j] * Dc[j] * vtr * d(rho_w) / (d(R(9.0) * R(1.718e-5)) * Dr[i]);
                double reynolds = 9.0 * stokes / (p * p * d(rho_w));
                double F = libm::log(reynolds);
                double G = -0.1007 - 0.358 * F + 0.0261 * F * F;
                double K0 = libm::exp(G);
                double z = libm::log(stokes / (K0 + 1.0e-15));
                double H = 0.1465 + 1.302 * z - 0.607 * z * z + 0.293 * z * z * z;
                double yc0 = 2.0 / d(PI) * libm::atan(H);
                Ef_rw = (yc0 + p) * (yc0 + p) / ((1.0 + p) * (1.0 + p));
            }
            // MAX(0.0, MIN(SNGL(Ef_rw), 0.95)): in REAL
            tables.t_Efrw[view.idx_efrw(i, j)] = double(std::max(R(0.0), std::min(R(Ef_rw), R(0.95))));
        }
    }

    // 2. table_Efsw
    for (int j = 0; j < nbc; ++j) {
        double vtc = 1.19e4 * (1.0e4 * Dc[j] * Dc[j] * 0.25);
        for (int i = 0; i < nbs; ++i) {
            double vts = d(av_s) * libm::pow(Ds[i], d(bv_s)) * libm::exp(-d(fv_s) * Ds[i]) - vtc;
            double Ds_m = libm::pow(d(am_s) * libm::pow(Ds[i], d(bm_s)) / d(am_r), d(obmr));
            double p = Dc[j] / Ds_m;
            double Ef_sw = 0.0;
            if (p > 0.25 || Ds[i] < d(D0s) || Dc[j] < d(R(6.e-6)) || vts < d(R(1.e-3))) {
                Ef_sw = 0.0;
            } else {
                double stokes = Dc[j] * Dc[j] * vts * d(rho_w) / (d(R(9.0) * R(1.718e-5)) * Ds_m);
                double reynolds = 9.0 * stokes / (p * p * d(rho_w));
                double F = libm::log(reynolds);
                double G = -0.1007 - 0.358 * F + 0.0261 * F * F;
                double K0 = libm::exp(G);
                double z = libm::log(stokes / (K0 + 1.0e-15));
                double H = 0.1465 + 1.302 * z - 0.607 * z * z + 0.293 * z * z * z;
                double yc0 = 2.0 / d(PI) * libm::atan(H);
                Ef_sw = (yc0 + p) * (yc0 + p) / ((1.0 + p) * (1.0 + p));
            }
            tables.t_Efsw[view.idx_efsw(i, j)] = double(std::max(R(0.0), std::min(R(Ef_sw), R(0.95))));
        }
    }

    // 3. table_dropEvap
    double massc[nbc];
    for (int n = 0; n < nbc; ++n) massc[n] = d(am_r) * vm::pow(Dc[n], d(bm_r), n, nbc);  // vectorised (libmvec)

    for (int k = 0; k < nbc; ++k) {
        int nu_c = std::min(15, static_cast<int>(std::round(1000.e6 / t_Nc[k])) + 2);
        int nu_c_idx = nu_c - 1; // 0-based for cce/ccg
        for (int j = 0; j < ntb_c; ++j) {
            double lamc = libm::pow(t_Nc[k] * d(am_r) * d(ccg[1][nu_c_idx]) * d(ocg1[nu_c_idx]) / d(tables.r_c[j]), d(obmr));
            double N0_c = t_Nc[k] * d(ocg1[nu_c_idx]) * libm::pow(lamc, d(cce[0][nu_c_idx]));
            for (int i = 0; i < nbc; ++i) {
                double summ = 0.0;
                double summ2 = 0.0;
                for (int n = 0; n <= i; ++n) {
                    double N_c_n = N0_c * powi(Dc[n], nu_c) * libm::exp(-lamc * Dc[n]) * dtc[n];  // Dc**nu_c: __powidf2
                    summ += massc[n] * N_c_n;
                    summ2 += N_c_n;
                }
                tables.tpc_wev[view.idx_wev(i, j, k)] = double(summ);
                tables.tnc_wev[view.idx_wev(i, j, k)] = double(summ2);
            }
        }
    }

    // 4. qr_acr_qg
    double vr[nbr];
    for (int n2 = 0; n2 < nbr; ++n2) {
        vr[n2] = d(R(-0.1021)) + d(R(4.932e3)) * Dr[n2] - d(R(0.9551e6)) * Dr[n2] * Dr[n2]
               + d(R(0.07934e9)) * Dr[n2] * Dr[n2] * Dr[n2]
               - d(R(0.002362e12)) * Dr[n2] * Dr[n2] * Dr[n2] * Dr[n2];
    }
    double vg[nbg];
    for (int n = 0; n < nbg; ++n) {
        vg[n] = d(av_g) * vm::pow(Dg[n], d(bv_g), n, nbg);
    }

    std::vector<double> N_g_pre(ntb_g * ntb_g1 * nbg);
    auto idx_ng = [](int j, int i, int n) {
        return static_cast<size_t>(n) + nbg * (static_cast<size_t>(i) + ntb_g1 * static_cast<size_t>(j));
    };
    for (int j = 0; j < ntb_g; ++j) {
        for (int i = 0; i < ntb_g1; ++i) {
            // lam_exp = (N0g_exp*am_g*cgg(1,1)/r_g)**oge1: REAL (powf in real*4), stored in DOUBLE PRECISION
            double lam_exp_g = d(libm::pow(tables.N0g_exp[i] * am_g * cgg[0] / tables.r_g[j], oge1));
            double lamg = lam_exp_g * d(libm::pow(cgg[2] * ogg2 * ogg1, obmg));
            double N0_g = d(tables.N0g_exp[i]) / (d(cgg[1]) * lam_exp_g) * libm::pow(lamg, d(cge[1]));
            for (int n = 0; n < nbg; ++n) {
                N_g_pre[idx_ng(j, i, n)] = N0_g * vm::pow(Dg[n], d(mu_g), n, nbg) * vm::exp(-lamg * Dg[n], n, nbg) * dtg[n];
            }
        }
    }

    if (!have_qg) par_for_host(ntb_r * ntb_r1, [&](int km) {
        int m = km / ntb_r1;
        int k = km % ntb_r1;

        double lam_exp_r = d(libm::pow(tables.N0r_exp[k] * am_r * crg[0] / tables.r_r[m], ore1));
        double lamr = lam_exp_r * d(libm::pow(crg[2] * org2 * org1, obmr));
        double N0_r = d(tables.N0r_exp[k]) / (d(crg[1]) * lam_exp_r) * libm::pow(lamr, d(cre[1]));
        double N_r[nbr];
        for (int n2 = 0; n2 < nbr; ++n2) {
            N_r[n2] = N0_r * vm::pow(Dr[n2], d(mu_r), n2, nbr) * vm::exp(-lamr * Dr[n2], n2, nbr) * dtr[n2];
        }

        constexpr int n3 = 0; // dimNRHG1 = 1
        for (int j = 0; j < ntb_g; ++j) {
            for (int i = 0; i < ntb_g1; ++i) {
                const double* N_g = &N_g_pre[idx_ng(j, i, 0)];

                double t1 = 0.0, t2 = 0.0, z1 = 0.0, y1 = 0.0, y2 = 0.0;
                for (int n2 = 0; n2 < nbr; ++n2) {
                    double massr = d(am_r) * libm::pow(Dr[n2], d(bm_r));
                    for (int n = 0; n < nbg; ++n) {
                        double massg = d(am_g) * vm::pow(Dg[n], d(bm_g), n, nbg);
                        double dvg = 0.5 * ((vr[n2] - vg[n]) + std::abs(vr[n2] - vg[n]));
                        double dvr = 0.5 * ((vg[n] - vr[n2]) + std::abs(vg[n] - vr[n2]));
                        double geom = d(PI * R(0.25) * Ef_rg) * (Dg[n] + Dr[n2]) * (Dg[n] + Dr[n2]);

                        t1 += geom * dvg * massg * N_g[n] * N_r[n2];
                        z1 += geom * dvg * massr * N_g[n] * N_r[n2];
                        y1 += geom * dvg * N_g[n] * N_r[n2];

                        t2 += geom * dvr * massr * N_g[n] * N_r[n2];
                        y2 += geom * dvr * N_g[n] * N_r[n2];
                    }
                }
                size_t idx = view.idx_racg(i, j, n3, k, m);
                tables.tcg_racg[idx] = double(t1);
                tables.tmr_racg[idx] = double(std::min(z1, d(tables.r_r[m])));
                tables.tcr_gacr[idx] = double(t2);
                tables.tnr_racg[idx] = double(y1);
                tables.tnr_gacr[idx] = double(y2);
            }
        }
    });

    // 5. qr_acr_qs
    double D1[nbr];
    for (int n2 = 0; n2 < nbr; ++n2) {
        D1[n2] = libm::pow(vr[n2] / d(av_s), d(R(1.0) / bv_s));
    }
    double vs[nbs];
    for (int n = 0; n < nbs; ++n) {
        vs[n] = d(R(1.5) * av_s) * vm::pow(Ds[n], d(bv_s), n, nbs) * vm::exp(-d(fv_s) * Ds[n], n, nbs);
    }

    std::vector<double> N_s_pre(ntb_t * ntb_s * nbs);
    auto idx_ns = [](int j, int i, int n) {
        return static_cast<size_t>(n) + nbs * (static_cast<size_t>(i) + ntb_s * static_cast<size_t>(j));
    };
    for (int j = 0; j < ntb_t; ++j) {
        for (int i = 0; i < ntb_s; ++i) {
            double M2 = d(tables.r_s[i] * oams);
            const R Tc = tables.Tc[j];
            double second = 0.0;
            if (bm_s > R(2.0) - R(1.e-3) && bm_s < R(2.0) + R(1.e-3)) {
                // loga_ and b_: REAL expressions (sa, sb, Tc, bm_s REAL) stored in DOUBLE PRECISION
                double loga_ = d(sa[0] + sa[1] * Tc + sa[2] * bm_s
                             + sa[3] * Tc * bm_s + sa[4] * Tc * Tc
                             + sa[5] * bm_s * bm_s + sa[6] * Tc * Tc * bm_s
                             + sa[7] * Tc * bm_s * bm_s + sa[8] * Tc * Tc * Tc
                             + sa[9] * bm_s * bm_s * bm_s);
                double a_ = libm::pow(d(R(10.0)), loga_);
                double b_ = d(sb[0] + sb[1] * Tc + sb[2] * bm_s
                          + sb[3] * Tc * bm_s + sb[4] * Tc * Tc
                          + sb[5] * bm_s * bm_s + sb[6] * Tc * Tc * bm_s
                          + sb[7] * Tc * bm_s * bm_s + sb[8] * Tc * Tc * Tc
                          + sb[9] * bm_s * bm_s * bm_s);
                second = libm::pow(M2 / a_, 1.0 / b_);
            } else {
                second = M2;
            }

            double loga_ = d(sa[0] + sa[1] * Tc + sa[2] * cse[0]
                         + sa[3] * Tc * cse[0] + sa[4] * Tc * Tc
                         + sa[5] * cse[0] * cse[0] + sa[6] * Tc * Tc * cse[0]
                         + sa[7] * Tc * cse[0] * cse[0] + sa[8] * Tc * Tc * Tc
                         + sa[9] * cse[0] * cse[0] * cse[0]);
            double a_ = libm::pow(d(R(10.0)), loga_);
            double b_ = d(sb[0] + sb[1] * Tc + sb[2] * cse[0]
                      + sb[3] * Tc * cse[0] + sb[4] * Tc * Tc
                      + sb[5] * cse[0] * cse[0] + sb[6] * Tc * Tc * cse[0]
                      + sb[7] * Tc * cse[0] * cse[0] + sb[8] * Tc * Tc * Tc
                      + sb[9] * cse[0] * cse[0] * cse[0]);
            double M3 = a_ * libm::pow(second, b_);
            double oM3 = 1.0 / M3;
            double Mrat = M2 * (M2 * oM3) * (M2 * oM3) * (M2 * oM3);
            double M0 = libm::pow(M2 * oM3, d(mu_s));
            double slam1 = M2 * oM3 * d(Lam0);
            double slam2 = M2 * oM3 * d(Lam1);

            for (int n = 0; n < nbs; ++n) {
                N_s_pre[idx_ns(j, i, n)] = Mrat * (d(Kap0) * vm::exp(-slam1 * Ds[n], n, nbs)
                       + d(Kap1) * M0 * vm::pow(Ds[n], d(mu_s), n, nbs) * vm::exp(-slam2 * Ds[n], n, nbs)) * dts[n];
            }
        }
    }

    if (!have_qs) par_for_host(ntb_r * ntb_r1, [&](int km) {
        int m = km / ntb_r1;
        int k = km % ntb_r1;

        double lam_exp_r = d(libm::pow(tables.N0r_exp[k] * am_r * crg[0] / tables.r_r[m], ore1));
        double lamr = lam_exp_r * d(libm::pow(crg[2] * org2 * org1, obmr));
        double N0_r = d(tables.N0r_exp[k]) / (d(crg[1]) * lam_exp_r) * libm::pow(lamr, d(cre[1]));
        double N_r[nbr];
        for (int n2 = 0; n2 < nbr; ++n2) {
            N_r[n2] = N0_r * vm::pow(Dr[n2], d(mu_r), n2, nbr) * vm::exp(-lamr * Dr[n2], n2, nbr) * dtr[n2];
        }

        for (int j = 0; j < ntb_t; ++j) {
            for (int i = 0; i < ntb_s; ++i) {
                const double* N_s = &N_s_pre[idx_ns(j, i, 0)];

                double t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0;
                double z1 = 0.0, z2 = 0.0, z3 = 0.0, z4 = 0.0;
                double y1 = 0.0, y2 = 0.0, y3 = 0.0, y4 = 0.0;

                for (int n2 = 0; n2 < nbr; ++n2) {
                    double massr = d(am_r) * libm::pow(Dr[n2], d(bm_r));
                    for (int n = 0; n < nbs; ++n) {
                        double masss = d(am_s) * libm::pow(Ds[n], d(bm_s));
                        double dvs = 0.5 * ((vr[n2] - vs[n]) + std::abs(vr[n2] - vs[n]));
                        double dvr = 0.5 * ((vs[n] - vr[n2]) + std::abs(vs[n] - vr[n2]));
                        double geom = d(PI * R(0.25) * Ef_rs) * (Ds[n] + Dr[n2]) * (Ds[n] + Dr[n2]);

                        if (massr > 1.5 * masss) {
                            t1 += geom * dvs * masss * N_s[n] * N_r[n2];
                            z1 += geom * dvs * massr * N_s[n] * N_r[n2];
                            y1 += geom * dvs * N_s[n] * N_r[n2];
                        } else {
                            t3 += geom * dvs * masss * N_s[n] * N_r[n2];
                            z3 += geom * dvs * massr * N_s[n] * N_r[n2];
                            y3 += geom * dvs * N_s[n] * N_r[n2];
                        }

                        if (massr > 1.5 * masss) {
                            t2 += geom * dvr * massr * N_s[n] * N_r[n2];
                            y2 += geom * dvr * N_s[n] * N_r[n2];
                            z2 += geom * dvr * masss * N_s[n] * N_r[n2];
                        } else {
                            t4 += geom * dvr * massr * N_s[n] * N_r[n2];
                            y4 += geom * dvr * N_s[n] * N_r[n2];
                            z4 += geom * dvr * masss * N_s[n] * N_r[n2];
                        }
                    }
                }
                size_t idx = view.idx_racs(i, j, k, m);
                tables.tcs_racs1[idx] = double(t1);
                tables.tmr_racs1[idx] = double(std::min(z1, d(tables.r_r[m])));
                tables.tcs_racs2[idx] = double(t3);
                tables.tmr_racs2[idx] = double(z3);
                tables.tcr_sacr1[idx] = double(t2);
                tables.tms_sacr1[idx] = double(z2);
                tables.tcr_sacr2[idx] = double(t4);
                tables.tms_sacr2[idx] = double(z4);
                tables.tnr_racs1[idx] = double(y1);
                tables.tnr_racs2[idx] = double(y3);
                tables.tnr_sacr1[idx] = double(y2);
                tables.tnr_sacr2[idx] = double(y4);
            }
        }
    });

    // 6. freezeH2O
    double orho_w = d(R(1.0) / rho_w);  // orho_w = 1./rho_w: a REAL quotient
    double massr_all[nbr];
    for (int n2 = 0; n2 < nbr; ++n2) massr_all[n2] = d(am_r) * vm::pow(Dr[n2], d(bm_r), n2, nbr);
    double massc_all[nbc];
    for (int n = 0; n < nbc; ++n) massc_all[n] = d(am_r) * vm::pow(Dc[n], d(bm_r), n, nbc);

    if (!have_fz) par_for_host(ntb_IN * ntb_freeze_temp, [&](int km) {
        int m = km / ntb_freeze_temp;
        int k = km % ntb_freeze_temp; // 0..44 -> 1..45 in Fortran (temp = -1 to -45 C)
        int k_f = k + 1;

        const R T_adjust = std::max(R(-3.0), std::min(R(3.0) - libm::log10(tables.Nt_IN[m]), R(3.0)));  // REAL
        double Texp = libm::exp(double(k_f) - d(T_adjust) * 1.0) - 1.0;

        for (int j = 0; j < ntb_r1; ++j) {
            for (int i = 0; i < ntb_r; ++i) {
                double lam_exp = d(libm::pow(tables.N0r_exp[j] * am_r * crg[0] / tables.r_r[i], ore1));
                double lamr = lam_exp * d(libm::pow(crg[2] * org2 * org1, obmr));
                double N0_r = d(tables.N0r_exp[j]) / (d(crg[1]) * lam_exp) * libm::pow(lamr, d(cre[1]));
                double sum1 = 0.0, sum2 = 0.0, sumn1 = 0.0, sumn2 = 0.0;
                for (int n2 = nbr - 1; n2 >= 0; --n2) {
                    double N_r = N0_r * libm::pow(Dr[n2], d(mu_r)) * libm::exp(-lamr * Dr[n2]) * dtr[n2];
                    double vol = massr_all[n2] * orho_w;
                    double prob = std::max(0.0, 1.0 - libm::exp(-120.0 * vol * 5.2e-4 * Texp));
                    if (massr_all[n2] < d(xm0g)) {
                        sumn1 += prob * N_r;
                        sum1 += prob * N_r * massr_all[n2];
                    } else {
                        sumn2 += prob * N_r;
                        sum2 += prob * N_r * massr_all[n2];
                    }
                    if ((sum1 + sum2) >= d(tables.r_r[i])) break;
                }
                size_t idx = view.idx_qrfz(i, j, k, m);
                tables.tpi_qrfz[idx] = double(sum1);
                tables.tni_qrfz[idx] = double(sumn1);
                tables.tpg_qrfz[idx] = double(sum2);
                tables.tnr_qrfz[idx] = double(sumn2);
            }
        }

        for (int j = 0; j < nbc; ++j) {
            int nu_c = std::min(15, static_cast<int>(std::round(1000.e6 / t_Nc[j])) + 2);
            int nu_c_idx = nu_c - 1;
            for (int i = 0; i < ntb_c; ++i) {
                double lamc = libm::pow(t_Nc[j] * d(am_r) * d(ccg[1][nu_c_idx]) * d(ocg1[nu_c_idx]) / d(tables.r_c[i]), d(obmr));
                double N0_c = t_Nc[j] * d(ocg1[nu_c_idx]) * libm::pow(lamc, d(cce[0][nu_c_idx]));
                double sum1 = 0.0, sumn2 = 0.0;
                for (int n = nbc - 1; n >= 0; --n) {
                    double vol = massc_all[n] * orho_w;
                    double prob = std::max(0.0, 1.0 - libm::exp(-120.0 * vol * 5.2e-4 * Texp));
                    double N_c = N0_c * powi(Dc[n], nu_c) * libm::exp(-lamc * Dc[n]) * dtc[n];
                    sumn2 = std::min(t_Nc[j], sumn2 + prob * N_c);
                    sum1 += prob * N_c * massc_all[n];
                    if (sum1 >= d(tables.r_c[i])) break;
                }
                size_t idx = view.idx_qcfz(i, j, k, m);
                tables.tpi_qcfz[idx] = double(sum1);
                tables.tni_qcfz[idx] = double(sumn2);
            }
        }
    });

    // 7. qi_aut_qs
    for (int j = 0; j < ntb_i1; ++j) {
        for (int i = 0; i < ntb_i; ++i) {
            // lami: a REAL expression and power stored in DOUBLE PRECISION; SNGL(Di_mean) compared in REAL
            double lami = d(libm::pow(am_i * cig[1] * oig1 * tables.Nt_i[j] / tables.r_i[i], obmi));
            double Di_mean = d(bm_i + mu_i + R(1.0)) / lami;
            double N0_i = d(tables.Nt_i[j] * oig1) * libm::pow(lami, d(cie[0]));
            double t1 = 0.0, t2 = 0.0;
            size_t idx = view.idx_iaus(i, j);
            if (R(Di_mean) > R(5.0) * D0s) {
                t1 = tables.r_i[i];
                t2 = tables.Nt_i[j];
                tables.tpi_ide[idx] = double(0);
            } else if (R(Di_mean) < D0i) {
                t1 = 0.0;
                t2 = 0.0;
                tables.tpi_ide[idx] = double(1);
            } else {
                const R xlimit_intg = R(lami * d(D0s));  // REAL
                tables.tpi_ide[idx] = double(gammp(mu_i + R(2.0), xlimit_intg));
                for (int n2 = 0; n2 < nbi; ++n2) {
                    double N_i_n2 = N0_i * libm::pow(Di[n2], d(mu_i)) * libm::exp(-lami * Di[n2]) * dti[n2];
                    if (Di[n2] >= d(D0s)) {
                        t1 += N_i_n2 * d(am_i) * libm::pow(Di[n2], d(bm_i));
                        t2 += N_i_n2;
                    }
                }
            }
            tables.tps_iaus[idx] = double(t1);
            tables.tni_iaus[idx] = double(t2);
        }
    }
}

template void init_thompson_tables<double>(ThompsonHostTables<double>& tables, const std::string&, bool, bool);
template void init_thompson_tables<float>(ThompsonHostTables<float>& tables, const std::string&, bool, bool);

} // namespace cppwrf::phys::mp_thompson
