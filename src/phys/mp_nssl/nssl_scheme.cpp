// SPDX-License-Identifier: GPL-3.0-or-later
// NSSL 2-moment microphysics: the scheme's host interface (nssl_args.hpp). A CUDA build compiles this file as CUDA:
// one thread per row, the module values (NsslParams), the tables and the rows' work spaces in device memory.
#include <algorithm>
#include <stdexcept>
#include <vector>

#include "core/backend.hpp"
#include "phys/mp_nssl/nssl_init.hpp"
#include "phys/mp_nssl/nssl_row.hpp"

namespace cppwrf::phys::mp_nssl {

namespace {

// one thread per row: column i0 of the launch range runs the row j over the tile's i-range
template <class Real>
struct NsslRowKernel {
  const NsslParams<Real>* P;
  NsslRowArgs<Real> A;
  char* work;
  std::size_t wbytes;
  int i0, j0;
  CPPWRF_HD void operator()(int i, int j) const {
    if (i != i0) return;
    char* w = work + std::size_t(j - j0) * wbytes;
    Work W{w, w + wbytes};
    nssl_row(*P, W, A, j);
  }
};

template <class T>
T* upload(const std::vector<T>& v) {
  T* d = static_cast<T*>(dev_alloc(v.size() * sizeof(T)));
  copy_to_dev(d, v.data(), v.size() * sizeof(T));
  return d;
}

}  // namespace

template <typename Real>
struct NsslScheme<Real>::Impl {
  NsslParams<Real>* dparams = nullptr;
  std::vector<void*> dmem;
  char* work = nullptr;
  std::size_t wbytes = 0;
  int rows = 0;  // rows of the work space (a launch takes at most this many)
  int ccna = 0;
  Impl() = default;
  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;
  ~Impl() {
    for (void* p : dmem) dev_free(p);
  }
};

// the work space of one row (bytes): the driver's arrays an, ancuten, t0 ... and the callees' gather arrays (64
// points). The driver's 64 nx nz values stay live through sediment1d's 31 nx nz (about 97 nx nz doubles at many
// levels, tests/unit/test_nssl_work: 1 to 200 points, 35 to 600 levels); the bound takes 88 nx nz and a quarter
// more (counted in doubles also for FP32: some of the arrays are DOUBLE PRECISION). The kernel stops
// (Work::overflow) rather than write past its end.
template <typename Real>
std::size_t NsslScheme<Real>::work_bytes(int nx_row, int nz) {
  const std::size_t nx = std::size_t(nx_row), k = std::size_t(nz);
  const std::size_t v = nx * k * 88 + k * 40 + nx * 900 + 70000;
  return (v * sizeof(double) * 5 / 4 + 255) / 256 * 256;
}

template <typename Real>
NsslScheme<Real>::NsslScheme(const NsslConfig& c, int nx_row, int nz, int max_rows) : impl_(std::make_shared<Impl>()) {
  auto hp = std::make_unique<NsslParams<Real>>();
  auto ht = std::make_unique<NsslTables<Real>>();
  impl_->ccna = nssl_init(*hp, *ht, c);
  auto dp = std::make_unique<NsslParams<Real>>(*hp);  // the device copy: table pointers into device memory
  auto up = [&](const auto& v) {
    auto* d = upload(v);
    impl_->dmem.push_back(d);
    return d;
  };
  dp->tabqvs_p = up(ht->tabqvs);
  dp->tabqis_p = up(ht->tabqis);
  dp->dtabqvs_p = up(ht->dtabqvs);
  dp->dtabqis_p = up(ht->dtabqis);
  dp->gamxinflu_p = up(ht->gamxinflu);
  dp->ciacrratio_p = up(ht->ciacrratio);
  dp->qiacrratio_p = up(ht->qiacrratio);
  dp->ziacrratio_p = up(ht->ziacrratio);
  impl_->dparams = static_cast<NsslParams<Real>*>(dev_alloc(sizeof(NsslParams<Real>)));
  impl_->dmem.push_back(impl_->dparams);
  copy_to_dev(impl_->dparams, dp.get(), sizeof(NsslParams<Real>));
  impl_->wbytes = work_bytes(nx_row, nz);
  // rows per launch: up to a 1 GiB work space (1 / K of it for K processes or tiles on one device), no more rows
  // than the domain has
  const std::size_t space = (std::size_t(1) << 30) / std::size_t(device_work_share());
  impl_->rows = int(std::min<std::size_t>(std::size_t(std::max(1, std::min(4096, max_rows))),
                                          std::max<std::size_t>(1, space / impl_->wbytes)));
  impl_->work = static_cast<char*>(dev_alloc(impl_->wbytes * std::size_t(impl_->rows)));
  impl_->dmem.push_back(impl_->work);
}

template <typename Real>
int NsslScheme<Real>::ccn_is_ccna() const {
  return impl_->ccna;
}

template <typename Real>
void NsslScheme<Real>::run(const NsslRowArgs<Real>& A, int j0, int j1) const {
  if (work_bytes(A.ite - A.its + 1, A.nz) > impl_->wbytes)
    throw std::runtime_error("NSSL microphysics: row longer than its work space");
  for (int jb = j0; jb < j1; jb += impl_->rows) {
    const int je = std::min(j1, jb + impl_->rows);
    column_for_whole(Range2D{A.its, A.its + 1, jb, je},
                     NsslRowKernel<Real>{impl_->dparams, A, impl_->work, impl_->wbytes, A.its, jb});
  }
}

template class NsslScheme<double>;
template class NsslScheme<float>;

}  // namespace cppwrf::phys::mp_nssl
