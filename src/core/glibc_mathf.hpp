// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// glibc's scalar float libm (Debian 13, glibc 2.41-12, x86-64) on host and device, bit for bit: expf, logf, powf,
// sinf, cosf (the FMA variants glibc's ifuncs select on CPUs with FMA and AVX2: __expf_fma, __logf_fma, __powf_fma,
// __sinf_fma, __cosf_fma) and atanf, tanhf, log10f (CORE-MATH's correctly rounded functions, no FMA).
// The CUDA FP32 identity build (CPPWRF_FP32_IDENTITY, CMake option CPPWRF_CUDA_FP32_IDENTITY) computes WRF real*4's
// float functions with them on the device, so that the GPU computes what the CPU FP32 build computes.
// Each FMA variant is transcribed from libm's machine code (objdump -d at the address its ifunc resolves to): every
// fused multiply-add there is an fma() here (gcc contracted a*b+c, also a*b+c and a*b-d of one product, in them),
// every other operation a separate IEEE operation (--fmad=false on the device, -ffp-contract=off on the host). The
// tables of the FMA variants are libm's objects bit for bit (__exp2f_data, __logf_data, __powf_log2_data,
// __sincosf_table: Debian's binary orders the sincosf polynomial differently from the 2.41 tarball); the CORE-MATH
// functions are correctly rounded and transcribed from the 2.41 sources. errno and the exception flags are not set;
// NaN results may differ from glibc's in sign and payload.
// Proof: tools/glibc_mathf_proof.cpp (CPU: every float input against the system libm; powf on samples) and
// tools/glibc_mathf_proof_cuda.cu (device against host, the same inputs).
#include <cmath>
#include <cstdint>
#include <cstring>

#include "core/glibc_math.hpp"

#if defined(__CUDACC__)
// libm's deliberate division by zero (math_errf.c)
#pragma nv_diagnostic push
#pragma nv_diag_suppress 39
#define CPPWRF_GMF __host__ __device__ __forceinline__
#else
#define CPPWRF_GMF inline
#endif

namespace cppwrf::glibcf_tables {
// libm.so.6 build ID befc0308f899f79b901d4e738992787368cf70f7 (64-bit words at the objects' addresses)
inline constexpr std::uint64_t kExp2fData[41] = {
    0x3ff0000000000000ull, 0x3fefd9b0d3158574ull, 0x3fefb5586cf9890full, 0x3fef9301d0125b51ull,
    0x3fef72b83c7d517bull, 0x3fef54873168b9aaull, 0x3fef387a6e756238ull, 0x3fef1e9df51fdee1ull,
    0x3fef06fe0a31b715ull, 0x3feef1a7373aa9cbull, 0x3feedea64c123422ull, 0x3feece086061892dull,
    0x3feebfdad5362a27ull, 0x3feeb42b569d4f82ull, 0x3feeab07dd485429ull, 0x3feea47eb03a5585ull,
    0x3feea09e667f3bcdull, 0x3fee9f75e8ec5f74ull, 0x3feea11473eb0187ull, 0x3feea589994cce13ull,
    0x3feeace5422aa0dbull, 0x3feeb737b0cdc5e5ull, 0x3feec49182a3f090ull, 0x3feed503b23e255dull,
    0x3feee89f995ad3adull, 0x3feeff76f2fb5e47ull, 0x3fef199bdd85529cull, 0x3fef3720dcef9069ull,
    0x3fef5818dcfba487ull, 0x3fef7c97337b9b5full, 0x3fefa4afa2a490daull, 0x3fefd0765b6e4540ull,
    0x42e8000000000000ull, 0x3fac6af84b912394ull, 0x3fcebfce50fac4f3ull, 0x3fe62e42ff0c52d6ull,
    0x40471547652b82feull, 0x3ebc6af84b912394ull, 0x3f2ebfce50fac4f3ull, 0x3f962e42ff0c52d6ull,
    0x4338000000000000ull,
};
inline constexpr std::uint64_t kLogfData[36] = {
    0x3ff661ec79f8f3beull, 0xbfd57bf7808caadeull, 0x3ff571ed4aaf883dull, 0xbfd2bef0a7c06ddbull,
    0x3ff49539f0f010b0ull, 0xbfd01eae7f513a67ull, 0x3ff3c995b0b80385ull, 0xbfcb31d8a68224e9ull,
    0x3ff30d190c8864a5ull, 0xbfc6574f0ac07758ull, 0x3ff25e227b0b8ea0ull, 0xbfc1aa2bc79c8100ull,
    0x3ff1bb4a4a1a343full, 0xbfba4e76ce8c0e5eull, 0x3ff12358f08ae5baull, 0xbfb1973c5a611cccull,
    0x3ff0953f419900a7ull, 0xbfa252f438e10c1eull, 0x3ff0000000000000ull, 0x0000000000000000ull,
    0x3fee608cfd9a47acull, 0x3faaa5aa5df25984ull, 0x3feca4b31f026aa0ull, 0x3fbc5e53aa362eb4ull,
    0x3feb2036576afce6ull, 0x3fc526e57720db08ull, 0x3fe9c2d163a1aa2dull, 0x3fcbc2860d224770ull,
    0x3fe886e6037841edull, 0x3fd1058bc8a07ee1ull, 0x3fe767dcf5534862ull, 0x3fd4043057b6ee09ull,
    0x3fe62e42fefa39efull, 0xbfd00ea348b88334ull, 0x3fd5575b0be00b6aull, 0xbfdffffef20a4123ull,
};
inline constexpr std::uint64_t kPowfLog2Data[37] = {
    0x3ff661ec79f8f3beull, 0xbfdefec65b963019ull, 0x3ff571ed4aaf883dull, 0xbfdb0b6832d4fca4ull,
    0x3ff49539f0f010b0ull, 0xbfd7418b0a1fb77bull, 0x3ff3c995b0b80385ull, 0xbfd39de91a6dcf7bull,
    0x3ff30d190c8864a5ull, 0xbfd01d9bf3f2b631ull, 0x3ff25e227b0b8ea0ull, 0xbfc97c1d1b3b7af0ull,
    0x3ff1bb4a4a1a343full, 0xbfc2f9e393af3c9full, 0x3ff12358f08ae5baull, 0xbfb960cbbf788d5cull,
    0x3ff0953f419900a7ull, 0xbfaa6f9db6475fceull, 0x3ff0000000000000ull, 0x0000000000000000ull,
    0x3fee608cfd9a47acull, 0x3fb338ca9f24f53dull, 0x3feca4b31f026aa0ull, 0x3fc476a9543891baull,
    0x3feb2036576afce6ull, 0x3fce840b4ac4e4d2ull, 0x3fe9c2d163a1aa2dull, 0x3fd40645f0c6651cull,
    0x3fe886e6037841edull, 0x3fd88e9c2c1b9ff8ull, 0x3fe767dcf5534862ull, 0x3fdce0a44eb17bccull,
    0x3fd27616c9496e0bull, 0xbfd71969a075c67aull, 0x3fdec70a6ca7baddull, 0xbfe7154748bef6c8ull,
    0x3ff71547652ab82bull,
};

inline constexpr std::uint64_t kSincosfTable[28] = {
    0x3ff0000000000000ull, 0xbff0000000000000ull, 0xbff0000000000000ull, 0x3ff0000000000000ull,
    0x41645f306dc9c883ull, 0x3ff921fb54442d18ull, 0x3ff0000000000000ull, 0xbfdffffffd0c621cull,
    0xbfc555545995a603ull, 0x3fa55553e1068f19ull, 0x3f81107605230bc4ull, 0xbf56c087e89a359dull,
    0xbf2994eb3774cf24ull, 0x3ef99343027bf8c3ull, 0x3ff0000000000000ull, 0xbff0000000000000ull,
    0xbff0000000000000ull, 0x3ff0000000000000ull, 0x41645f306dc9c883ull, 0x3ff921fb54442d18ull,
    0xbff0000000000000ull, 0x3fdffffffd0c621cull, 0xbfc555545995a603ull, 0xbfa55553e1068f19ull,
    0x3f81107605230bc4ull, 0x3f56c087e89a359dull, 0xbf2994eb3774cf24ull, 0xbef99343027bf8c3ull,
};
inline constexpr double kLog10Tr[65] = {
    0x1p+0, 0x1.f81f82p-1, 0x1.f07c1fp-1, 0x1.e9131acp-1,
    0x1.e1e1e1ep-1, 0x1.dae6077p-1, 0x1.d41d41dp-1, 0x1.cd85689p-1,
    0x1.c71c71cp-1, 0x1.c0e0704p-1, 0x1.bacf915p-1, 0x1.b4e81b5p-1,
    0x1.af286bdp-1, 0x1.a98ef6p-1, 0x1.a41a41ap-1, 0x1.9ec8e95p-1,
    0x1.999999ap-1, 0x1.948b0fdp-1, 0x1.8f9c19p-1, 0x1.8acb90fp-1,
    0x1.8618618p-1, 0x1.8181818p-1, 0x1.7d05f41p-1, 0x1.78a4c81p-1,
    0x1.745d174p-1, 0x1.702e05cp-1, 0x1.6c16c17p-1, 0x1.6816817p-1,
    0x1.642c859p-1, 0x1.605816p-1, 0x1.5c9882cp-1, 0x1.58ed231p-1,
    0x1.5555555p-1, 0x1.51d07ebp-1, 0x1.4e5e0a7p-1, 0x1.4afd6ap-1,
    0x1.47ae148p-1, 0x1.446f865p-1, 0x1.4141414p-1, 0x1.3e22cbdp-1,
    0x1.3b13b14p-1, 0x1.3813814p-1, 0x1.3521cfbp-1, 0x1.323e34ap-1,
    0x1.2f684bep-1, 0x1.2c9fb4ep-1, 0x1.29e412ap-1, 0x1.27350b9p-1,
    0x1.2492492p-1, 0x1.21fb781p-1, 0x1.1f7047ep-1, 0x1.1cf06aep-1,
    0x1.1a7b961p-1, 0x1.1811812p-1, 0x1.15b1e5fp-1, 0x1.135c811p-1,
    0x1.1111111p-1, 0x1.0ecf56cp-1, 0x1.0c9715p-1, 0x1.0a6810ap-1,
    0x1.0842108p-1, 0x1.0624dd3p-1, 0x1.041041p-1, 0x1.0204081p-1,
    0.5,
};
inline constexpr double kLog10Tl[65] = {
    -0x1.d45fd6237ebe3p-47, 0x1.b947689311b6ep-8, 0x1.b5e909c96d7d5p-7, 0x1.45f4f59ed2165p-6,
    0x1.af5f92cbd8f1ep-6, 0x1.0ba01a606de8cp-5, 0x1.3ed119b9a2b7bp-5, 0x1.714834298eec2p-5,
    0x1.a30a9d98357fbp-5, 0x1.d41d512670813p-5, 0x1.02428c0f65519p-4, 0x1.1a23444eecc3ep-4,
    0x1.31b30543f4cb4p-4, 0x1.48f3ed39bfd04p-4, 0x1.5fe8049a0e423p-4, 0x1.769140a6aa008p-4,
    0x1.8cf1836c98cb3p-4, 0x1.a30a9d55541a1p-4, 0x1.b8de4d1ee823ep-4, 0x1.ce6e4202ca2e6p-4,
    0x1.e3bc1accace07p-4, 0x1.f8c9683b5abd4p-4, 0x1.06cbd68ca9a6ep-3, 0x1.11142f19df73p-3,
    0x1.1b3e71fa7a97fp-3, 0x1.254b4d37a46e3p-3, 0x1.2f3b6912cbf07p-3, 0x1.390f683115886p-3,
    0x1.42c7e7fffc5a8p-3, 0x1.4c65808c78d3cp-3, 0x1.55e8c50751c55p-3, 0x1.5f52445dec3d8p-3,
    0x1.68a288c3f12p-3, 0x1.71da17bdf0d19p-3, 0x1.7af973608afd9p-3, 0x1.84011952a2579p-3,
    0x1.8cf1837a7ea6p-3, 0x1.95cb2891e43d6p-3, 0x1.9e8e7b0f869ep-3, 0x1.a73beaa5db18dp-3,
    0x1.afd3e394558d3p-3, 0x1.b856cf060d9f1p-3, 0x1.c0c5134de1ffcp-3, 0x1.c91f1371bc99fp-3,
    0x1.d1652ffcd3f53p-3, 0x1.d997c6f635e75p-3, 0x1.e1b733ab90f3bp-3, 0x1.e9c3ceadac856p-3,
    0x1.f1bdeec43a305p-3, 0x1.f9a5e7a5fa3fep-3, 0x1.00be05ac02f2bp-2, 0x1.04a054d81a2d4p-2,
    0x1.087a0835957fbp-2, 0x1.0c4b457099517p-2, 0x1.101431aa1fe51p-2, 0x1.13d4f08b98dd8p-2,
    0x1.178da53edb892p-2, 0x1.1b3e71e9f9d58p-2, 0x1.1ee777defdeedp-2, 0x1.2288d7b48e23bp-2,
    0x1.2622b0f52e49fp-2, 0x1.29b522a4c6314p-2, 0x1.2d404b0e30f8p-2, 0x1.30c4478f3fbe5p-2,
    0x1.34413509f7915p-2,
};
inline constexpr std::uint32_t kInvPio4[24] = {
    0xa2,       0xa2f9,     0xa2f983,   0xa2f9836e, 0xf9836e4e, 0x836e4e44, 0x6e4e4415, 0x4e441529,
    0x441529fc, 0x1529fc27, 0x29fc2757, 0xfc2757d1, 0x2757d1f5, 0x57d1f534, 0xd1f534dd, 0xf534ddc0,
    0x34ddc0db, 0xddc0db62, 0xc0db6295, 0xdb629599, 0x6295993c, 0x95993c43, 0x993c4390, 0x3c439041,
};
}  // namespace cppwrf::glibcf_tables
#if defined(__CUDACC__)
namespace cppwrf::glibcf_tables::dev {  // device copies (constant-initialised from the host tables)
template <class T, int N>
struct Arr {
  T v[N];
};
template <class T, int N>
constexpr Arr<T, N> arr(const T (&a)[N]) {
  Arr<T, N> w{};
  for (int i = 0; i < N; ++i) w.v[i] = a[i];
  return w;
}
static __device__ const Arr<std::uint64_t, 41> kExp2fData = arr(::cppwrf::glibcf_tables::kExp2fData);
static __device__ const Arr<std::uint64_t, 36> kLogfData = arr(::cppwrf::glibcf_tables::kLogfData);
static __device__ const Arr<std::uint64_t, 37> kPowfLog2Data = arr(::cppwrf::glibcf_tables::kPowfLog2Data);
static __device__ const Arr<std::uint64_t, 28> kSincosfTable = arr(::cppwrf::glibcf_tables::kSincosfTable);
static __device__ const Arr<std::uint32_t, 24> kInvPio4 = arr(::cppwrf::glibcf_tables::kInvPio4);
static __device__ const Arr<double, 65> kLog10Tr = arr(::cppwrf::glibcf_tables::kLog10Tr);
static __device__ const Arr<double, 65> kLog10Tl = arr(::cppwrf::glibcf_tables::kLog10Tl);
}  // namespace cppwrf::glibcf_tables::dev
#endif

namespace cppwrf::glibcf {
namespace detail {
#if defined(__CUDA_ARCH__)
#define CPPWRF_GFT(name) ::cppwrf::glibcf_tables::dev::name.v
#else
#define CPPWRF_GFT(name) ::cppwrf::glibcf_tables::name
#endif
// word w of a table as a double
#define CPPWRF_GFD(name, w) ::cppwrf::glibc::detail::asd(CPPWRF_GFT(name)[w])
using ::cppwrf::glibc::detail::asd;
using ::cppwrf::glibc::detail::asu;
using ::cppwrf::glibc::detail::fma;
CPPWRF_GMF float asf(std::uint32_t b) {
#if defined(__CUDA_ARCH__)
  return __uint_as_float(b);
#else
  float v;
  std::memcpy(&v, &b, sizeof v);
  return v;
#endif
}
CPPWRF_GMF std::uint32_t asuf(float v) {
#if defined(__CUDA_ARCH__)
  return __float_as_uint(v);
#else
  std::uint32_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
#endif
}
CPPWRF_GMF float fmaf(float a, float b, float c) {
#if defined(__CUDA_ARCH__)
  return __fmaf_rn(a, b, c);
#else
  return std::fma(a, b, c);
#endif
}
CPPWRF_GMF int clz32(std::uint32_t x) {
#if defined(__CUDA_ARCH__)
  return __clz(static_cast<int>(x));
#else
  return __builtin_clz(x);
#endif
}
// math_errf.c (no errno): the results of the overflow / underflow / invalid / pole paths
CPPWRF_GMF float xflowf(std::uint32_t sign, float y) {
  const float x = sign ? -y : y;
  return x * y;
}
CPPWRF_GMF float oflowf(std::uint32_t sign) { return xflowf(sign, 0x1p97f); }
CPPWRF_GMF float uflowf(std::uint32_t sign) { return xflowf(sign, 0x1p-95f); }
CPPWRF_GMF float may_uflowf(std::uint32_t sign) { return xflowf(sign, 0x1.4p-75f); }
CPPWRF_GMF float invalidf(float x) {
  const float d = x - x;
  return d / d;
}
CPPWRF_GMF float divzerof(std::uint32_t sign) {
  const float y = sign ? -1.0f : 1.0f;
  const float z = 0.0f;
  return y / z;
}
CPPWRF_GMF std::uint32_t top12(float x) { return asuf(x) >> 20; }
}  // namespace detail

// ---- expf: __expf_fma (libm 0x7d2a0); __exp2f_data: tab words 0-31, shift_scaled 32, poly 33-35,
// invln2_scaled 36, poly_scaled 37-39, shift 40 ----
CPPWRF_GMF float expf(float x) {
  using namespace detail;
  const double xd = double(x);
  const std::uint32_t abstop = top12(x) & 0x7ff;
  if (abstop >= top12(88.0f)) {
    if (asuf(x) == asuf(-INFINITY)) return 0.0f;
    if (abstop >= top12(INFINITY)) return x + x;
    if (x > 0x1.62e42ep6f) return oflowf(0);
    if (x < -0x1.9fe368p6f) return uflowf(0);
    if (x < -0x1.9d1d9ep6f) return may_uflowf(0);
  }
  const double invln2n = CPPWRF_GFD(kExp2fData, 36), shift = CPPWRF_GFD(kExp2fData, 40);
  double kd = fma(invln2n, xd, shift);  // z + SHIFT with z = InvLn2N * xd contracted
  const std::uint64_t ki = asu(kd);
  kd -= shift;
  const double r = fma(invln2n, xd, -kd);  // z - kd
  std::uint64_t t = CPPWRF_GFT(kExp2fData)[ki % 32];
  t += ki << 47;
  const double s = asd(t);
  const double z = fma(CPPWRF_GFD(kExp2fData, 37), r, CPPWRF_GFD(kExp2fData, 38));
  const double r2 = r * r;
  double y = fma(CPPWRF_GFD(kExp2fData, 39), r, 1.0);
  y = fma(z, r2, y);
  y = y * s;
  return static_cast<float>(y);
}

// ---- logf: __logf_fma (libm 0x7dbf0); __logf_data: tab (invc, logc) words 0-31, ln2 32, poly 33-35 ----
CPPWRF_GMF float logf(float x) {
  using namespace detail;
  std::uint32_t ix = asuf(x);
  if (ix == 0x3f800000) return 0.0f;
  if (ix - 0x00800000 >= 0x7f800000 - 0x00800000) {
    if (ix * 2 == 0) return divzerof(1);
    if (ix == 0x7f800000) return x;
    if ((ix & 0x80000000) || ix * 2 >= 0xff000000) return invalidf(x);
    ix = asuf(x * 0x1p23f);
    ix -= 23 << 23;
  }
  const std::uint32_t tmp = ix - 0x3f330000;
  const int i = (tmp >> 19) % 16;
  const int k = static_cast<std::int32_t>(tmp) >> 23;
  const std::uint32_t iz = ix - (tmp & 0xff800000u);
  const double invc = CPPWRF_GFD(kLogfData, 2 * i), logc = CPPWRF_GFD(kLogfData, 2 * i + 1);
  const double z = double(asf(iz));
  const double r = fma(z, invc, -1.0);
  const double y0 = fma(double(k), CPPWRF_GFD(kLogfData, 32), logc);
  const double r2 = r * r;
  double y = fma(CPPWRF_GFD(kLogfData, 34), r, CPPWRF_GFD(kLogfData, 35));
  y = fma(CPPWRF_GFD(kLogfData, 33), r2, y);
  y = fma(y, r2, y0 + r);
  return static_cast<float>(y);
}

// ---- powf: __powf_fma (libm 0x7e360); __powf_log2_data: tab (invc, logc) words 0-31, poly 32-36 ----
namespace detail {
CPPWRF_GMF double powf_log2(std::uint32_t ix) {
  const std::uint32_t tmp = ix - 0x3f330000;
  const int i = (tmp >> 19) % 16;
  const std::uint32_t top = tmp & 0xff800000u;
  const std::uint32_t iz = ix - top;
  const int k = static_cast<std::int32_t>(top) >> 23;
  const double invc = CPPWRF_GFD(kPowfLog2Data, 2 * i), logc = CPPWRF_GFD(kPowfLog2Data, 2 * i + 1);
  const double z = double(asf(iz));
  const double r = fma(z, invc, -1.0);
  const double y0 = double(k) + logc;
  const double r2 = r * r;
  double y = fma(r, CPPWRF_GFD(kPowfLog2Data, 32), CPPWRF_GFD(kPowfLog2Data, 33));
  const double p = fma(r, CPPWRF_GFD(kPowfLog2Data, 34), CPPWRF_GFD(kPowfLog2Data, 35));
  const double r4 = r2 * r2;
  double q = fma(r, CPPWRF_GFD(kPowfLog2Data, 36), y0);
  q = fma(r2, p, q);
  y = fma(y, r4, q);
  return y;
}
CPPWRF_GMF double powf_exp2(double xd, std::uint32_t sign_bias) {
  const double shift = CPPWRF_GFD(kExp2fData, 32);
  double kd = xd + shift;
  const std::uint64_t ki = asu(kd);
  kd -= shift;
  const double r = xd - kd;
  std::uint64_t t = CPPWRF_GFT(kExp2fData)[ki % 32];
  const std::uint64_t ski = ::cppwrf::glibc::detail::addk(ki, sign_bias);
  t += ski << 47;
  const double s = asd(t);
  const double z = fma(r, CPPWRF_GFD(kExp2fData, 33), CPPWRF_GFD(kExp2fData, 34));
  const double r2 = r * r;
  double y = fma(r, CPPWRF_GFD(kExp2fData, 35), 1.0);
  y = fma(z, r2, y);
  y = y * s;
  return y;
}
// 0 not an integer, 1 odd, 2 even
CPPWRF_GMF int checkint(std::uint32_t iy) {
  const int e = iy >> 23 & 0xff;
  if (e < 0x7f) return 0;
  if (e > 0x7f + 23) return 2;
  if (iy & ((1u << (0x7f + 23 - e)) - 1)) return 0;
  if (iy & (1u << (0x7f + 23 - e))) return 1;
  return 2;
}
CPPWRF_GMF bool zeroinfnan(std::uint32_t ix) { return 2 * ix - 1 >= 2u * 0x7f800000 - 1; }
CPPWRF_GMF bool issignalingf(float x) {
  const std::uint32_t ix = asuf(x);
  return (ix & 0x7fc00000) == 0x7f800000 && (ix & 0x003fffff) != 0;
}
}  // namespace detail
CPPWRF_GMF float powf(float x, float y) {
  using namespace detail;
  std::uint32_t sign_bias = 0;
  std::uint32_t ix = asuf(x);
  const std::uint32_t iy = asuf(y);
  if (ix - 0x00800000 >= 0x7f800000 - 0x00800000 || zeroinfnan(iy)) {
    if (zeroinfnan(iy)) {
      if (2 * iy == 0) return issignalingf(x) ? x + y : 1.0f;
      if (ix == 0x3f800000) return issignalingf(y) ? x + y : 1.0f;
      if (2 * ix > 2u * 0x7f800000 || 2 * iy > 2u * 0x7f800000) return x + y;
      if (2 * ix == 2 * 0x3f800000) return 1.0f;
      if ((2 * ix < 2 * 0x3f800000) == !(iy & 0x80000000)) return 0.0f;
      return y * y;
    }
    if (zeroinfnan(ix)) {
      float x2 = x * x;
      if ((ix & 0x80000000) && checkint(iy) == 1) {
        x2 = -x2;
        sign_bias = 1;
      }
      if (2 * ix == 0 && (iy & 0x80000000)) return divzerof(sign_bias);
      return (iy & 0x80000000) ? 1 / x2 : x2;
    }
    if (ix & 0x80000000) {
      const int yint = checkint(iy);
      if (yint == 0) return invalidf(x);
      if (yint == 1) sign_bias = 1u << 16;  // SIGN_BIAS: 1 << (EXP2F_TABLE_BITS + 11)
      ix &= 0x7fffffff;
    }
    if (ix < 0x00800000) {
      ix = asuf(x * 0x1p23f);
      ix &= 0x7fffffff;
      ix -= 23 << 23;
    }
  }
  const double logx = powf_log2(ix);
  const double ylogx = double(y) * logx;
  if ((asu(ylogx) >> 47 & 0xffff) >= asu(126.0) >> 47) {
    if (ylogx > 0x1.fffffffd1d571p+6) return oflowf(sign_bias);
    if (ylogx <= -150.0) return uflowf(sign_bias);
    if (ylogx < -149.0) return may_uflowf(sign_bias);
  }
  return static_cast<float>(powf_exp2(ylogx, sign_bias));
}

// ---- sinf, cosf: __sinf_fma (libm 0x83e40), __cosf_fma (libm 0x7f230); __sincosf_table[2] words 0-13 and 14-27:
// sign[4] 0-3, hpi_inv 4, hpi 5, then c0 6, c1 7, s1 8, c2 9, s2 10, c3 11, s3 12, c4 13 (Debian's binary) ----
namespace detail {
CPPWRF_GMF std::uint32_t abstop12(float x) { return (asuf(x) >> 20) & 0x7ff; }
// sin polynomial (quadrant n even) or cos polynomial (n odd) of x, x2 with table p (0 or 14)
CPPWRF_GMF float sincosf_poly(double x, double x2, int p, int n) {
  if ((n & 1) == 0) {
    const double s1 = fma(x2, CPPWRF_GFD(kSincosfTable, p + 12), CPPWRF_GFD(kSincosfTable, p + 10));
    const double x3 = x2 * x;
    const double x7 = x2 * x3;
    const double s = fma(x3, CPPWRF_GFD(kSincosfTable, p + 8), x);
    return static_cast<float>(fma(s1, x7, s));
  }
  const double x4 = x2 * x2;
  const double c1 = fma(x2, CPPWRF_GFD(kSincosfTable, p + 7), CPPWRF_GFD(kSincosfTable, p + 6));
  const double c2 = fma(x2, CPPWRF_GFD(kSincosfTable, p + 13), CPPWRF_GFD(kSincosfTable, p + 11));
  const double x6 = x2 * x4;
  const double c = fma(x4, CPPWRF_GFD(kSincosfTable, p + 9), c1);
  return static_cast<float>(fma(c2, x6, c));
}
CPPWRF_GMF double reduce_fast(double x, int* np) {
  const double r = x * CPPWRF_GFD(kSincosfTable, 4);
  const int n = (static_cast<std::int32_t>(r) + 0x800000) >> 24;
  *np = n;
  return fma(-double(n), CPPWRF_GFD(kSincosfTable, 5), x);
}
CPPWRF_GMF double reduce_large(std::uint32_t xi, int* np) {
  const int a = (xi >> 26) & 15;
  const int shift = (xi >> 23) & 7;
  xi = (xi & 0xffffff) | 0x800000;
  xi <<= shift;
  std::uint64_t res0 = std::uint32_t(xi * CPPWRF_GFT(kInvPio4)[a]);
  const std::uint64_t res1 = std::uint64_t(xi) * CPPWRF_GFT(kInvPio4)[a + 4];
  const std::uint64_t res2 = std::uint64_t(xi) * CPPWRF_GFT(kInvPio4)[a + 8];
  res0 = (res2 >> 32) | (res0 << 32);
  res0 += res1;
  const std::uint64_t n = (res0 + (1ULL << 61)) >> 62;
  res0 -= n << 62;
  const double x = double(static_cast<std::int64_t>(res0));
  *np = int(n);
  return x * 0x1.921FB54442D18p-62;
}
}  // namespace detail
CPPWRF_GMF float sinf(float y) {
  using namespace detail;
  double x = y;
  int n;
  if (abstop12(y) < abstop12(0x1.921FB6p-1f)) {
    const double s = x * x;
    if (abstop12(y) < abstop12(0x1p-12f)) return y;
    return sincosf_poly(x, s, 0, 0);
  } else if (abstop12(y) < abstop12(120.0f)) {
    x = reduce_fast(x, &n);
    const double s = CPPWRF_GFD(kSincosfTable, n & 3);
    const int p = (n & 2) ? 14 : 0;
    return sincosf_poly(x * s, x * x, p, n);
  } else if (abstop12(y) < abstop12(INFINITY)) {
    const std::uint32_t xi = asuf(y);
    const int sign = xi >> 31;
    x = reduce_large(xi, &n);
    const double s = CPPWRF_GFD(kSincosfTable, (n + sign) & 3);
    const int p = ((n + sign) & 2) ? 14 : 0;
    return sincosf_poly(x * s, x * x, p, n);
  }
  return invalidf(y);
}
CPPWRF_GMF float cosf(float y) {
  using namespace detail;
  double x = y;
  int n;
  if (abstop12(y) < abstop12(0x1.921FB6p-1f)) {
    const double x2 = x * x;
    if (abstop12(y) < abstop12(0x1p-12f)) return 1.0f;
    return sincosf_poly(x, x2, 0, 1);
  } else if (abstop12(y) < abstop12(120.0f)) {
    x = reduce_fast(x, &n);
    const double s = CPPWRF_GFD(kSincosfTable, n & 3);
    const int p = (n & 2) ? 14 : 0;
    return sincosf_poly(x * s, x * x, p, n ^ 1);
  } else if (abstop12(y) < abstop12(INFINITY)) {
    const std::uint32_t xi = asuf(y);
    const int sign = xi >> 31;
    x = reduce_large(xi, &n);
    const double s = CPPWRF_GFD(kSincosfTable, (n + sign) & 3);
    const int p = ((n + sign) & 2) ? 14 : 0;
    return sincosf_poly(x * s, x * x, p, n ^ 1);
  }
  return invalidf(y);
}

// ---- atanf (CORE-MATH, s_atanf.c) ----
CPPWRF_GMF float atanf(float x) {
  using namespace detail;
  const double pi2 = 0x1.921fb54442d18p+0;
  const std::uint32_t t = asuf(x);
  const int e = (t >> 23) & 0xff;
  const bool gt = e >= 127;
  const std::uint32_t ta = t & 0x7fffffff;
  if (ta >= 0x4c700518u) {
    if (ta > 0x7f800000u) return x + x;
    return static_cast<float>((t >> 31) ? -pi2 : pi2);
  }
  if (e < 127 - 13) {
    if (e < 127 - 25) {
      if (!(t << 1)) return x;
      return fmaf(-x, std::fabs(x), x);
    }
    return fmaf(-0x1.555556p-2f * x, x * x, x);
  }
  double z = x;
  if (gt) z = 1 / z;
  const double z2 = z * z, z4 = z2 * z2, z8 = z4 * z4;
  double cn0 = 0x1.51eccde075d67p-2 + z2 * 0x1.a76bb5637f2f2p-1;
  const double cn2 = 0x1.81e0eed20de88p-1 + z2 * 0x1.376c8ca67d11dp-2;
  double cn4 = 0x1.aec7b69202ac6p-5 + z2 * 0x1.9561899acc73ep-9;
  const double cn6 = 0x1.bf9fa5b67e6p-16;
  cn0 += z4 * cn2;
  cn4 += z4 * cn6;
  cn0 += z8 * cn4;
  cn0 *= z;
  double cd0 = 0x1.51eccde075d66p-2 + z2 * 0x1.dfbdd7b392d28p-1;
  const double cd2 = 0x1p+0 + z2 * 0x1.fd22bf0e89b54p-2;
  double cd4 = 0x1.d91ff8b576282p-4 + z2 * 0x1.653ea99fc9bbp-7;
  const double cd6 = 0x1.1e7fcc202340ap-12;
  cd0 += z4 * cd2;
  cd4 += z4 * cd6;
  cd0 += z8 * cd4;
  double r = cn0 / cd0;
  if (!gt) return static_cast<float>(r);
  const bool neg = asu(z) >> 63;
  r = (neg ? -0x1.0fdaa22168c23p-7 : 0x1.0fdaa22168c23p-7) - r + (neg ? -0x1.9p0 : 0x1.9p0);
  return static_cast<float>(r);
}

// ---- tanhf (CORE-MATH, s_tanhf.c) ----
CPPWRF_GMF float tanhf(float x) {
  using namespace detail;
  const double z = x;
  const std::uint32_t ux = asuf(x);
  const int e = (ux >> 23) & 0xff;
  if (e == 0xff) {
    if (ux << 9) return x + x;
    return (ux >> 31) ? -1.0f : 1.0f;
  }
  if (e < 115) {
    if (e < 102) {
      if ((ux << 1) == 0) return x;
      return fmaf(-x, std::fabs(x), x);
    }
    const float x2 = x * x;
    return fmaf(x, -0x1.555556p-2f * x2, x);
  }
  if ((ux << 1) > (0x41102cb3u << 1)) return (ux >> 31) ? -1.0f + 0x1p-25f : 1.0f - 0x1p-25f;
  const double z2 = z * z, z4 = z2 * z2, z8 = z4 * z4;
  double n0 = 0x1p+0 + z2 * 0x1.30877b8b72d33p-3;
  const double n2 = 0x1.694aa09ae9e5ep-8 + z2 * 0x1.4101377abb729p-14;
  double n4 = 0x1.e0392b1db0018p-22 + z2 * 0x1.2533756e546f7p-30;
  const double n6 = 0x1.d62e5abe6ae8ap-41 + z2 * 0x1.b06be534182dep-54;
  n0 += z4 * n2;
  n4 += z4 * n6;
  n0 += z8 * n4;
  double d0 = 0x1p+0 + z2 * 0x1.ed99131b0ebeap-2;
  const double d2 = 0x1.0d27ed6c95a69p-5 + z2 * 0x1.7cbdaca0e9fccp-11;
  double d4 = 0x1.b4e60b892578ep-18 + z2 * 0x1.a6f707c5c71abp-26;
  const double d6 = 0x1.35a8b6e2cd94cp-35 + z2 * 0x1.ca8230677aa01p-47;
  d0 += z4 * d2;
  d4 += z4 * d6;
  d0 += z8 * d4;
  const double r = z * n0 / d0;
  return static_cast<float>(r);
}

// ---- log10f (CORE-MATH, e_log10f.c; the w_log10f wrapper: -Inf for +-0, NaN below 0) ----
CPPWRF_GMF float log10f(float x) {
  using namespace detail;
  std::uint32_t ux = asuf(x);
  if (ux < (1u << 23) || ux >= 0x7f800000u) {
    if (ux == 0 || ux >= 0x7f800000u) {
      if (ux == 0x7f800000u) return x;
      const std::uint32_t ax = ux << 1;
      if (ax == 0u) return divzerof(1);
      if (ax > 0xff000000u) return x + x;
      return invalidf(x);
    }
    const int n = clz32(ux) - 8;
    ux <<= n;
    ux -= std::uint32_t(n) << 23;
  }
  const unsigned m = ux & ((1u << 23) - 1), j = (m + (1u << (23 - 7))) >> (23 - 6);
  const double ix = CPPWRF_GFT(kLog10Tr)[j], l = CPPWRF_GFT(kLog10Tl)[j];
  const int e = (static_cast<int>(ux) >> 23) - 127;
  unsigned je = e + 1;
  je = (je * 0x4d104d4u) >> 28;
  // st[je]: 10^je as a float for je <= 10, 0 above
  constexpr std::uint32_t st[16] = {0x3f800000u, 0x41200000u, 0x42c80000u, 0x447a0000u, 0x461c4000u,
                                    0x47c35000u, 0x49742400u, 0x4b189680u, 0x4cbebc20u, 0x4e6e6b28u,
                                    0x501502f9u, 0, 0, 0, 0, 0};
  if (ux == st[je]) return float(je);
  double tz = asd((std::uint64_t(m) | (std::uint64_t(1023) << 23)) << (52 - 23));
  const double z = tz * ix - 1, z2 = z * z;
  double r = ((e * 0x1.34413509f79ffp-2 + l) + z * 0x1.bcb7b15c5a2f8p-2) +
             z2 * (-0x1.bcbb1dbb88ebap-3 + z * 0x1.2871c39d521c6p-3);
  float ub = static_cast<float>(r), lb = static_cast<float>(r + 0x1.b008p-34);
  if (ub != lb) {
    double f = z * ((0x1.bcb7b1526e50ep-2 + z * -0x1.bcb7b1526e53dp-3) +
                    z2 * ((0x1.287a7636f3fa2p-3 + z * -0x1.bcb7b146a14b3p-4) +
                          z2 * (0x1.63c627d5219cbp-4 + z * -0x1.2880736c8762dp-4 + z2 * 0x1.fc1ecf913961ap-5)));
    f -= 0x1.0cee0ed4ca7e9p-54 * e;
    f += l - CPPWRF_GFT(kLog10Tl)[0];
    const double el = e * 0x1.34413509f7ap-2;
    r = el + f;
    ub = static_cast<float>(r);
    tz = r;
    if (!((asu(tz) & ((1 << 28) - 1)))) {
      const double dr = (el - r) + f;
      r += dr * 32;
      ub = static_cast<float>(r);
    }
  }
  return ub;
}
}  // namespace cppwrf::glibcf
#if defined(__CUDACC__)
#pragma nv_diagnostic pop
#endif
