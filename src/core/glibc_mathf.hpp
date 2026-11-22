// SPDX-License-Identifier: GPL-3.0-or-later AND LGPL-2.1-or-later AND MIT
#pragma once
// glibc's scalar float libm (Debian 13, glibc 2.41-12, x86-64) on host and device, bit for bit: expf, logf, powf,
// sinf, cosf (the FMA variants glibc's ifuncs select on CPUs with FMA and AVX2: __expf_fma, __logf_fma, __powf_fma,
// __sinf_fma, __cosf_fma) and atanf, atan2f, tanf, tanhf, log10f, asinf, acosf, erff, tgammaf, sinhf (CORE-MATH's
// correctly rounded functions, generic C without FMA variants). The FP32 builds' executables define the C functions
// with them on the host (core/glibc_mathf_host.cpp: the results do not depend on the host's libm), and the CUDA FP32
// identity build (CPPWRF_FP32_IDENTITY, CMake option CPPWRF_CUDA_FP32_IDENTITY) computes WRF real*4's float
// functions with them on the device, so that the GPU computes what the CPU FP32 build computes.
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

#if defined(__CUDACC__) && !defined(__HIP__)
// libm's deliberate division by zero (math_errf.c)
#pragma nv_diagnostic push
#pragma nv_diag_suppress 39
#endif
#if defined(CPPWRF_GPUCC)
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
// erff (s_erff.c): C[i - 7][0 .. 7], degree-7 polynomials of erf(i/16 + 1/32 + z), |z| <= 1/32, 7 <= i < 63
inline constexpr double kErffC[448] = {
    0x1.f86faa9428f9cp-2, 0x1.cfc41e36c7dfap-1, -0x1.b2c7dc53508b9p-2, -0x1.5a9de93fa556ep-3,
    0x1.731793dbb01b5p-3, 0x1.133e06426cf18p-6, -0x1.a12a6289cafd8p-5, 0x1.717d6f1d6f557p-9,
    0x1.1855a5fd3dd50p-1, 0x1.b3aafcc27502fp-1, -0x1.cee5ac8e92bb2p-2, -0x1.fa02983ca2d79p-4,
    0x1.77cd746cb1922p-3, -0x1.fa6f277886487p-10, -0x1.8de75458db416p-5, 0x1.00899c98551c9p-7,
    0x1.32a54cb8db67ap-1, 0x1.96164fafd8de5p-1, -0x1.e23a7ea0c9ad3p-2, -0x1.3f5ee15671cf4p-4,
    0x1.70e468a3d72d9p-3, -0x1.3da68037cfc99p-6, -0x1.69ed9ba1f9839p-5, 0x1.8cab9244a4ff4p-7,
    0x1.4b13713ad3513p-1, 0x1.7791b886e7405p-1, -0x1.ecef423109bf5p-2, -0x1.15c3c5cec6847p-5,
    0x1.5f688fc931ba6p-3, -0x1.1da63ed190037p-5, -0x1.38427ca63cca4p-5, 0x1.fa00e52525e17p-7,
    0x1.61955607dd15dp-1, 0x1.58a445da7c74ep-1, -0x1.ef6c246a0f66cp-2, 0x1.e83e0d9d61330p-8,
    0x1.44cc65535bc9fp-3, -0x1.87d3c4860435dp-5, -0x1.f90b10501169bp-6, 0x1.22295856d427ap-6,
    0x1.762870f720c6fp-1, 0x1.39ccc1b136d5cp-1, -0x1.ea4feea4e4744p-2, 0x1.715e5952ebfbap-5,
    0x1.22cdbd83c75c4p-3, -0x1.da50aa1d925b6p-5, -0x1.754dc0a29b4ddp-6, 0x1.350b6bef9392cp-6,
    0x1.88d1cd474a2e0p-1, 0x1.1b7e98fe26219p-1, -0x1.de65a22ce1419p-2, 0x1.40686a3f16400p-4,
    0x1.f6b0cbb216b2bp-4, -0x1.09c7c903edd57p-4, -0x1.da7529fde641p-7, 0x1.362a7a0588eabp-6,
    0x1.999d4192a5717p-1, 0x1.fc3ee5d1524b3p-2, -0x1.cc990045b55c8p-2, 0x1.b37338e68b37dp-4,
    0x1.a0d120c872ea7p-4, -0x1.19bb2b07ecff6p-4, -0x1.a110f5f593aafp-8, 0x1.272c15a57720ep-6,
    0x1.a89c850b7d54dp-1, 0x1.c40b0729ed54ap-2, -0x1.b5eaaef0a2346p-2, 0x1.0847c7dacbae1p-3,
    0x1.47de0ba6d18fbp-4, -0x1.1d9de77a4b648p-4, 0x1.30ffbe56f0726p-10, 0x1.0a9cb99feea01p-6,
    0x1.b5e62fce16096p-1, 0x1.8eed36b886d95p-2, -0x1.9b64a06e50705p-2, 0x1.2bb6e2c744df5p-3,
    0x1.dee3261ca61bcp-5, -0x1.16996004f7da5p-4, 0x1.fdff37bae983ep-8, 0x1.c750083e65f9ap-7,
    0x1.c194b1d49a184p-1, 0x1.5d4fd33729015p-2, -0x1.7e0f4f045addbp-2, 0x1.444bc66c31a1bp-3,
    0x1.356dbf16ec8f1p-5, -0x1.0643de0906cd8p-4, 0x1.b281af7bd3a2cp-7, 0x1.6b97eaa2c6abdp-7,
    0x1.cbc54b476248ep-1, 0x1.2f7cc3fe6f423p-2, -0x1.5ee8429e36de8p-2, 0x1.52a8395f96177p-3,
    0x1.313761ba257dcp-6, -0x1.dcf844d5fed8fp-5, 0x1.1e1420f475fa9p-6, 0x1.091c7dc1e18b2p-7,
    0x1.d4970f9ce00d9p-1, 0x1.059f59af7a905p-2, -0x1.3eda354de36c3p-2, 0x1.57b85ad439779p-3,
    0x1.8e913b9778136p-10, -0x1.a2893bd3435f4p-5, 0x1.4d3a90e37164ap-6, 0x1.4ce7f6e19a902p-8,
    0x1.dc29fb60715b0p-1, 0x1.bf8e1b1ca2277p-3, -0x1.1eb7095e5d6d2p-2, 0x1.549ea6f7a64f4p-3,
    -0x1.b10f12f3877a3p-7, -0x1.61420c8f7156ap-5, 0x1.674f1f92a8812p-6, 0x1.25543ffd74d52p-9,
    0x1.e29e22a89d767p-1, 0x1.7bd5c7df3fe99p-3, -0x1.fe674494077bfp-3, 0x1.4a9feacf86578p-3,
    -0x1.a008269076644p-6, -0x1.1cf0e8fb4f1cbp-5, 0x1.6e0d2ef105fb3p-6, -0x1.367205fbd7876p-12,
    0x1.e812fc64db36ap-1, 0x1.3fda6bc016991p-3, -0x1.c1cb278627920p-3, 0x1.3b10512314f1ep-3,
    -0x1.1e6457bb1b9a9p-5, -0x1.b1f6474e2388cp-6, 0x1.640a5345f7ec7p-6, -0x1.3dae5a997fdbp-9,
    0x1.eca6ccd709544p-1, 0x1.0b3f52ce8c380p-3, -0x1.8885019f63c6dp-3, 0x1.274275fc91a05p-3,
    -0x1.57f73699a8372p-5, -0x1.3076a305fc7cep-6, 0x1.4c6ae04843a41p-6, -0x1.0be5fcf5ecc91p-8,
    0x1.f0762fde45ee7p-1, 0x1.bb1c972f23e4ap-4, -0x1.5341e3c01b58dp-3, 0x1.107929f6f0b60p-3,
    -0x1.7e1b34f976c02p-5, -0x1.73b62589c234ap-7, 0x1.2a97ee1876486p-6, -0x1.595f40a3150fep-8,
    0x1.f39bc242e43e6p-1, 0x1.6c7e64e7281c5p-4, -0x1.2274b86835fd3p-3, 0x1.efb890e5c770dp-4,
    -0x1.92c7db16847e0p-5, -0x1.45477db5e2dd4p-8, 0x1.01fc6165fc866p-6, -0x1.8845509030c2cp-8,
    0x1.f62fe80272419p-1, 0x1.297db960e4f5dp-4, -0x1.ecb83b087c04fp-4, 0x1.bce18363ca3d1p-4,
    -0x1.985aaf776482cp-5, 0x1.cd953efdae886p-12, 0x1.ab9a0b89b54ffp-7, -0x1.9b5e576ccc31cp-8,
    0x1.f848acb544e95p-1, 0x1.e1d4cf1e24501p-5, -0x1.9e12e1fde5552p-4, 0x1.8a27806df3d1bp-4,
    -0x1.91674e5eb3319p-5, 0x1.3bc75595b2db8p-8, 0x1.51bc537ac61afp-7, -0x1.96b23b19ea04dp-8,
    0x1.f9f9ba8d3c733p-1, 0x1.83298d7172108p-5, -0x1.58d101f905a75p-4, 0x1.58f1456f8639bp-4,
    -0x1.808d1850b8231p-5, 0x1.0c1bd99c348a7p-7, 0x1.f61e9d7bc48cap-8, -0x1.7f07c13441774p-8,
    0x1.fb54641aebbc9p-1, 0x1.34ac36ad8dafap-5, -0x1.1c8ec267f9405p-4, 0x1.2a52c5d841848p-4,
    -0x1.68541c02b3b6bp-5, 0x1.5afe400196379p-7, 0x1.565b2d6eda3d6p-8, -0x1.596aaff29e739p-8,
    0x1.fc67bcf2d7b8fp-1, 0x1.e85c449e377efp-6, -0x1.d177f166c07c6p-5, 0x1.fe23b7584b504p-5,
    -0x1.4b12109613313p-5, 0x1.8d9905c0acf7dp-7, 0x1.9265032a669dap-9, -0x1.2ac4a6dbcbf3ep-8,
    0x1.fd40bd6d7a785p-1, 0x1.7f5188610ddc7p-6, -0x1.7954423f7c998p-5, 0x1.af5baae33887fp-5,
    -0x1.2ad77c7cbc474p-5, 0x1.a7b8c47ec2a51p-7, 0x1.46646ee094bccp-10, -0x1.ef19d8db8673p-9,
    0x1.fdea6e062d0c9p-1, 0x1.2a875b5ffab58p-6, -0x1.2f3178cd6dcd5p-5, 0x1.68d1c45b94182p-5,
    -0x1.09648ed3aeaefp-5, 0x1.ad8b150d38164p-7, -0x1.e9a6023d9429fp-13, -0x1.8722d19ee2e8ep-9,
    0x1.fe6e1742f7cf5p-1, 0x1.cd5ec93c1243ap-7, -0x1.e2ff3aaacb386p-6, 0x1.2aa4e5823cc89p-5,
    -0x1.d049842dbe399p-6, 0x1.a34edb21ab302p-7, -0x1.676e5996c7f9bp-10, -0x1.23b01a35140bfp-9,
    0x1.fed37386190fbp-1, 0x1.61beae53b72c2p-7, -0x1.7d6193f22c3c1p-6, 0x1.e947279e3bb7dp-6,
    -0x1.906031b97ca97p-6, 0x1.8d14d62561755p-7, -0x1.1f245e7178882p-9, -0x1.9257d4eb47685p-10,
    0x1.ff20e0a7ba8c2p-1, 0x1.0d1d69569b839p-7, -0x1.2a8ca0dc02752p-6, 0x1.8cc071b709751p-6,
    -0x1.54a149f1b070cp-6, 0x1.6e9137b13412cp-7, -0x1.6577ed3d8e83bp-9, -0x1.e9c1a5178a289p-11,
    0x1.ff5b8fb26f5f6p-1, 0x1.9646f35a7663cp-8, -0x1.cf68ed9311b0bp-7, 0x1.3e8735b5a694fp-6,
    -0x1.1e1612d026fdfp-6, 0x1.4afd8e6ca636dp-7, -0x1.8c375170ccb22p-9, -0x1.c799443c4fd3bp-12,
    0x1.ff87b1913e853p-1, 0x1.30499b5039596p-8, -0x1.64964201ec8bap-7, 0x1.fa73d7eafba98p-7,
    -0x1.daa3022141fbbp-7, 0x1.2509444c063b7p-7, -0x1.99482a2f8a0a1p-9, -0x1.403d1f76c9454p-15,
    0x1.ffa89fe5b3625p-1, 0x1.c4412bf4b8f35p-9, -0x1.100f347126cf0p-7, 0x1.8ebda07671d40p-7,
    -0x1.850c6a31c98c1p-7, 0x1.fdac860c67d21p-8, -0x1.927d03d2ba12cp-9, 0x1.0ff620b4190fep-12,
    0x1.ffc10194fcb64p-1, 0x1.4d78bba8ca621p-9, -0x1.9ba107a443e02p-8, 0x1.36f273fbc04ccp-7,
    -0x1.3b38716ac7e6fp-7, 0x1.b3fe0181914acp-8, -0x1.7d3fe7de98c5cp-9, 0x1.ea31f8e5317f7p-12,
    0x1.ffd2eae369a07p-1, 0x1.e7f232d9e266cp-10, -0x1.34c7442dd48d9p-8, 0x1.e066bed070a0bp-8,
    -0x1.f914f3c42fc0dp-8, 0x1.6f4664ed2260fp-8, -0x1.5e59910761d24p-9, 0x1.39cbb6e84c126p-11,
    0x1.ffdff92db56e5p-1, 0x1.6235fbd7a4373p-10, -0x1.cb5e029b9e56ap-9, 0x1.6fa4c7ef274dap-8,
    -0x1.903a089a835f3p-8, 0x1.30f12e0ca1901p-8, -0x1.39d21b6957f99p-9, 0x1.5d3f8495a703cp-11,
    0x1.ffe96a78a04a9p-1, 0x1.fe41cd9bb4f2cp-11, -0x1.52d7b28966c0cp-9, 0x1.16c192d86a1a7p-8,
    -0x1.39bfce951100cp-8, 0x1.f376a7869f9e3p-9, -0x1.12e6cef999c4fp-9, 0x1.66acd4d667b5p-11,
    0x1.fff0312b010b5p-1, 0x1.6caa0d3583018p-11, -0x1.efb729f4cf75bp-10, 0x1.a2da7cebe12acp-9,
    -0x1.e6c27a24bc759p-9, 0x1.93b1f4d8ea65p-9, -0x1.d82050aa94a08p-10, 0x1.5cd7dc75d6cbap-11,
    0x1.fff50456dab8cp-1, 0x1.0295ef6591865p-11, -0x1.679880e95a4dap-10, 0x1.37d38e3a5c8ebp-9,
    -0x1.75b3708aebb8fp-9, 0x1.4231c4b4b0296p-9, -0x1.8e26476489318p-10, 0x1.45c3b570dd924p-11,
    0x1.fff86cfd3e657p-1, 0x1.6be02102b353dp-12, -0x1.02b157780d6aep-10, 0x1.cc1d886861133p-10,
    -0x1.1bff6f12ec9abp-9, 0x1.fc0f77bd9c736p-10, -0x1.4a3320bd0959dp-10, 0x1.267f8b4f95d2p-11,
    0x1.fffad0b901755p-1, 0x1.fc0d55470cf5ep-13, -0x1.7121aff5e820ep-11, 0x1.506d6992f7de5p-10,
    -0x1.ab595d3ecd0d6p-10, 0x1.8bdd79daaf754p-10, -0x1.0d9b090f997c1p-10, 0x1.031ab9fd1c7dap-11,
    0x1.fffc7a37857d2p-1, 0x1.5feada379d8a5p-13, -0x1.05304df58f3aap-11, 0x1.e79c081b8600fp-11,
    -0x1.3e5dbe33232e0p-10, 0x1.30eb208200729p-10, -0x1.b1d493b147945p-11, 0x1.bd587bbc071bep-12,
    0x1.fffd9fdeabccep-1, 0x1.e3bcf436a1a49p-14, -0x1.6e953111ef0a1p-12, 0x1.5e3edf6768654p-11,
    -0x1.d5be67c0547a4p-11, 0x1.d07d9ffa1d435p-11, -0x1.58328f5f358cap-11, 0x1.76d42d95c42c4p-12,
    0x1.fffe68f4fa777p-1, 0x1.49e17724f4cddp-14, -0x1.fe48c44e229c1p-13, 0x1.f2bd95d76f188p-12,
    -0x1.57388cb12d011p-11, 0x1.5decc25c5c079p-11, -0x1.0d7499d1b0d2dp-11, 0x1.359332c94ecdcp-12,
    0x1.fffef1960d85dp-1, 0x1.be6abbb10a4cdp-15, -0x1.6040381a8c313p-13, 0x1.5fff1dde9ee9dp-12,
    -0x1.f0c933efa9971p-12, 0x1.04cbf4a5cd760p-11, -0x1.a07f150af6dadp-12, 0x1.f68dd183426bap-13,
    0x1.ffff4db27f146p-1, 0x1.2bb5cc22e5cd8p-15, -0x1.e25894899f526p-14, 0x1.ec8a8e5a72757p-13,
    -0x1.64256ae0a3cf9p-12, 0x1.80a836c18c46cp-12, -0x1.3dea401af6775p-12, 0x1.915ddff3fe0d1p-13,
    0x1.ffff8b500e77cp-1, 0x1.8f4ccca7fc769p-16, -0x1.478cffe305946p-14, 0x1.559f04adde504p-13,
    -0x1.f9e1577d6961dp-13, 0x1.18bda53c14716p-12, -0x1.df8634c35541cp-13, 0x1.3bb5c6b616337p-13,
    0x1.ffffb43555b5fp-1, 0x1.07ebd2a2d26c8p-16, -0x1.b93e442a37f2bp-15, 0x1.d5cf15159ce28p-14,
    -0x1.63f5e1469c006p-13, 0x1.95a03acebac18p-13, -0x1.656e5e2a1f8e2p-13, 0x1.e98c437189bdep-14,
    0x1.ffffcf23ff5fcp-1, 0x1.5a2adfa0b492cp-17, -0x1.26c88270759f0p-15, 0x1.40473572b99a8p-14,
    -0x1.f057cbde578a5p-14, 0x1.22178d1c3c948p-13, -0x1.0765b61a0d859p-13, 0x1.765b3ea03ddbep-14,
    0x1.ffffe0bd3e852p-1, 0x1.c282cd3957a72p-18, -0x1.86ad6dfa44faap-16, 0x1.b0f313f03a029p-15,
    -0x1.56e44abecd255p-14, 0x1.9ad1ecfe34a89p-14, -0x1.7fe4033478618p-14, 0x1.1a8184e049fbfp-14,
    0x1.ffffec2641a9ep-1, 0x1.22df29821407ep-18, -0x1.00c902a6cfd98p-16, 0x1.22234eb88671fp-15,
    -0x1.d57a181c9e6e1p-15, 0x1.200c283b54a90p-14, -0x1.14b4c3295a7d0p-14, 0x1.a4f966f713bdep-15,
    0x1.fffff37d63a36p-1, 0x1.74adc8f405eecp-19, -0x1.4ed4228e44858p-17, 0x1.81918baea92bap-16,
    -0x1.3e81b17a0009cp-15, 0x1.9004a36116436p-15, -0x1.8aa1ba400e076p-15, 0x1.35cd4e2340a9ep-15,
    0x1.fffff82cdcf1bp-1, 0x1.d9c73698fa87dp-20, -0x1.b11017ec67115p-18, 0x1.fc0dfadf653f8p-17,
    -0x1.ac4e03cd2dfc2p-16, 0x1.131806b5abbc5p-15, -0x1.1672ef66fcaafp-15, 0x1.c2882c7debed7p-16,
    0x1.fffffb248c39dp-1, 0x1.2acee2f5ec66ap-20, -0x1.15cc570408a36p-18, 0x1.4be757bbb75a3p-17,
    -0x1.1d6aa5f8d2940p-16, 0x1.76c5937d5105ep-16, -0x1.84dffc3ca9302p-16, 0x1.43c8315f2c30ap-16,
    0x1.fffffd01f36afp-1, 0x1.75fa8dbc840bap-21, -0x1.6186da0133f5ap-19, 0x1.ae023231e1af5p-18,
    -0x1.790812f7ca394p-17, 0x1.f9c25656d0ef2p-17, -0x1.0cc66682e304cp-16, 0x1.cc170a75d6f9cp-17,
    0x1.fffffe2ba0ea5p-1, 0x1.d06ad6ecde88ep-22, -0x1.be46aa8edc9a1p-20, 0x1.143860c7840b8p-18,
    -0x1.edaba78fb1260p-18, 0x1.52138a96ecee2p-17, -0x1.6fca538c4e2eep-17, 0x1.434040640bcefp-17,
    0x1.fffffee3cc32cp-1, 0x1.1e1e857adb8ddp-22, -0x1.1769ce5f2a6e8p-20, 0x1.5fe5d479b0543p-19,
    -0x1.405d865c94c2ap-18, 0x1.bfc94feb96afcp-18, -0x1.f245d5f3e8358p-18, 0x1.c142456acf443p-18,
};
// sinhf (e_sinhf.c): 2^(j/32) / 2, j = 0 .. 31
inline constexpr std::uint64_t kSinhfTb[32] = {
    0x3fe0000000000000ull, 0x3fe059b0d3158574ull, 0x3fe0b5586cf9890full, 0x3fe11301d0125b51ull,
    0x3fe172b83c7d517bull, 0x3fe1d4873168b9aaull, 0x3fe2387a6e756238ull, 0x3fe29e9df51fdee1ull,
    0x3fe306fe0a31b715ull, 0x3fe371a7373aa9cbull, 0x3fe3dea64c123422ull, 0x3fe44e086061892dull,
    0x3fe4bfdad5362a27ull, 0x3fe5342b569d4f82ull, 0x3fe5ab07dd485429ull, 0x3fe6247eb03a5585ull,
    0x3fe6a09e667f3bcdull, 0x3fe71f75e8ec5f74ull, 0x3fe7a11473eb0187ull, 0x3fe82589994cce13ull,
    0x3fe8ace5422aa0dbull, 0x3fe93737b0cdc5e5ull, 0x3fe9c49182a3f090ull, 0x3fea5503b23e255dull,
    0x3feae89f995ad3adull, 0x3feb7f76f2fb5e47ull, 0x3fec199bdd85529cull, 0x3fecb720dcef9069ull,
    0x3fed5818dcfba487ull, 0x3fedfc97337b9b5full, 0x3feea4afa2a490daull, 0x3fef50765b6e4540ull,
};
}  // namespace cppwrf::glibcf_tables
#if defined(CPPWRF_GPUCC)
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
static __device__ const Arr<double, 448> kErffC = arr(::cppwrf::glibcf_tables::kErffC);
static __device__ const Arr<std::uint64_t, 32> kSinhfTb = arr(::cppwrf::glibcf_tables::kSinhfTb);
}  // namespace cppwrf::glibcf_tables::dev
#endif

namespace cppwrf::glibcf {
namespace detail {
#if defined(CPPWRF_DEVICE_CODE)
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
#if defined(CPPWRF_DEVICE_CODE)
  return __uint_as_float(b);
#else
  float v;
  std::memcpy(&v, &b, sizeof v);
  return v;
#endif
}
CPPWRF_GMF std::uint32_t asuf(float v) {
#if defined(CPPWRF_DEVICE_CODE)
  return __float_as_uint(v);
#else
  std::uint32_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
#endif
}
CPPWRF_GMF float fmaf(float a, float b, float c) {
#if defined(CPPWRF_DEVICE_CODE)
  return CPPWRF_FMAF_RN(a, b, c);
#else
  return std::fma(a, b, c);
#endif
}
CPPWRF_GMF int clz32(std::uint32_t x) {
#if defined(CPPWRF_DEVICE_CODE)
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

// ---- atan2f (CORE-MATH, e_atan2f.c: correctly rounded) ----
namespace detail {
CPPWRF_GMF double atan2f_muldd(double xh, double xl, double ch, double cl, double* l) {
  const double ahlh = ch * xl, alhh = cl * xh, ahhh = ch * xh;
  double ahhl = fma(ch, xh, -ahhh);
  ahhl += alhh + ahlh;
  ch = ahhh + ahhl;
  *l = (ahhh - ch) + ahhl;
  return ch;
}
CPPWRF_GMF float atan2f_tiny(float y, float x) {
  const double dy = y, dx = x;
  const double z = dy / dx;
  double e = fma(-z, double(x), double(y));
  const double c = -0x1.5555555555555p-2;
  const double zz = z * z, cz = c * z;
  e = e / x + cz * zz;
  std::uint64_t t = asu(z);
  if ((t & 0xfffffffull) == 0) {
    if (z * e > 0) t += 1;
    else t -= 1;
  }
  return static_cast<float>(asd(t));
}
}  // namespace detail
CPPWRF_GMF float atan2f(float y, float x) {
  using namespace detail;
  constexpr double cn[7] = {0x1p+0, 0x1.40e0698f94c35p+1, 0x1.248c5da347f0dp+1, 0x1.d873386572976p-1,
                            0x1.46fa40b20f1dp-3, 0x1.33f5e041eed0fp-7, 0x1.546bbf28667c5p-14};
  constexpr double cd[7] = {0x1p+0, 0x1.6b8b143a3f6dap+1, 0x1.8421201d18ed5p+1, 0x1.8221d086914ebp+0,
                            0x1.670657e3a07bap-2, 0x1.0f4951fd1e72dp-5, 0x1.b3874b8798286p-11};
  constexpr double m[2] = {0, 1};
  constexpr double pi = 0x1.921fb54442d18p+1, pi2 = 0x1.921fb54442d18p+0, pi2l = 0x1.1a62633145c07p-54;
  constexpr double off[8] = {0.0, pi2, pi, pi2, -0.0, -pi2, -pi, -pi2};
  constexpr double offl[8] = {0.0, pi2l, 2 * pi2l, pi2l, -0.0, -pi2l, -2 * pi2l, -pi2l};
  constexpr double sgn[2] = {1, -1};
  const std::uint32_t ux = asuf(x), uy = asuf(y);
  const std::uint32_t ax = ux & (~0u >> 1), ay = uy & (~0u >> 1);
  if (ay >= (0xffu << 23) || ax >= (0xffu << 23)) {
    if (ay > (0xffu << 23)) return x + y;
    if (ax > (0xffu << 23)) return x + y;
    const bool yinf = ay == (0xffu << 23), xinf = ax == (0xffu << 23);
    if (yinf && xinf) {
      if (ux >> 31) return static_cast<float>(0x1.2d97c7f3321d2p+1 * sgn[uy >> 31]);
      return static_cast<float>(0x1.921fb54442d18p-1 * sgn[uy >> 31]);
    }
    if (xinf) {
      if (ux >> 31) return static_cast<float>(pi * sgn[uy >> 31]);
      return static_cast<float>(0.0 * sgn[uy >> 31]);
    }
    if (yinf) return static_cast<float>(pi2 * sgn[uy >> 31]);
  }
  if (ay == 0) {
    if (!ax) {
      const std::uint32_t i = (uy >> 31) * 4 + (ux >> 31) * 2;
      if (ux >> 31) return static_cast<float>(off[i] + offl[i]);
      return static_cast<float>(off[i]);
    }
    if (!(ux >> 31)) return static_cast<float>(0.0 * sgn[uy >> 31]);
  }
  const std::uint32_t gt = ay > ax;
  const std::uint32_t i = (uy >> 31) * 4 + (ux >> 31) * 2 + gt;
  const double zx = x, zy = y;
  double z = (m[gt] * zx + m[1 - gt] * zy) / (m[gt] * zy + m[1 - gt] * zx);
  double r;
  const int d = int(ax) - int(ay);
  if (d < (27 << 23) && d > (-(27 << 23))) {
    const double z2 = z * z, z4 = z2 * z2, z8 = z4 * z4;
    double cn0 = cn[0] + z2 * cn[1];
    const double cn2 = cn[2] + z2 * cn[3];
    double cn4 = cn[4] + z2 * cn[5];
    const double cn6 = cn[6];
    cn0 += z4 * cn2;
    cn4 += z4 * cn6;
    cn0 += z8 * cn4;
    double cd0 = cd[0] + z2 * cd[1];
    const double cd2 = cd[2] + z2 * cd[3];
    double cd4 = cd[4] + z2 * cd[5];
    const double cd6 = cd[6];
    cd0 += z4 * cd2;
    cd4 += z4 * cd6;
    cd0 += z8 * cd4;
    r = cn0 / cd0;
  } else {
    r = 1;
  }
  z *= sgn[gt];
  r = z * r + off[i];
  if (((asu(r) + 8) & 0xfffffffull) <= 16) {
    if (ay < ax && ((ax - ay) >> 23 >= 25)) return atan2f_tiny(y, x);
    double zh, zl;
    if (gt == 0) {
      zh = zy / zx;
      zl = fma(zh, -zx, zy) / zx;
    } else {
      zh = zx / zy;
      zl = fma(zh, -zy, zx) / zy;
    }
    double z2l;
    const double z2h = atan2f_muldd(zh, zl, zh, zl, &z2l);
    constexpr double c[32][2] = {
        {0x1p+0, -0x1.8c1dac5492248p-87},
        {-0x1.5555555555555p-2, -0x1.55553bf3a2abep-56},
        {0x1.999999999999ap-3, -0x1.99deed1ec9071p-57},
        {-0x1.2492492492492p-3, -0x1.fd99c8d18269ap-58},
        {0x1.c71c71c71c717p-4, -0x1.651eee4c4d9dp-61},
        {-0x1.745d1745d1649p-4, -0x1.632683d6c44a6p-58},
        {0x1.3b13b13b11c63p-4, 0x1.bf69c1f8af41dp-58},
        {-0x1.11111110e6338p-4, 0x1.3c3e431e8bb68p-61},
        {0x1.e1e1e1dc45c4ap-5, -0x1.be2db05c77bbfp-59},
        {-0x1.af286b8164b4fp-5, 0x1.a4673491f0942p-61},
        {0x1.86185e9ad4846p-5, 0x1.e12e32d79fceep-59},
        {-0x1.642c6d5161faep-5, 0x1.3ce76c1ca03fp-59},
        {0x1.47ad6f277e5bfp-5, -0x1.abd8d85bdb714p-60},
        {-0x1.2f64a2ee8896dp-5, 0x1.ef87d4b615323p-61},
        {0x1.1a6a2b31741b5p-5, 0x1.a5d9d973547eep-62},
        {-0x1.07fbdad65e0a6p-5, -0x1.65ac07f5d35f4p-61},
        {0x1.ee9932a9a5f8bp-6, 0x1.f8b9623f6f55ap-61},
        {-0x1.ce8b5b9584dc6p-6, 0x1.fe5af96e8ea2dp-61},
        {0x1.ac9cb288087b7p-6, -0x1.450cdfceaf5cap-60},
        {-0x1.84b025351f3e6p-6, 0x1.579561b0d73dap-61},
        {0x1.52f5b8ecdd52bp-6, 0x1.036bd2c6fba47p-60},
        {-0x1.163a8c44909dcp-6, 0x1.18f735ffb9f16p-60},
        {0x1.a400dce3eea6fp-7, -0x1.c90569c0c1b5cp-61},
        {-0x1.1caa78ae6db3ap-7, -0x1.4c60f8161ea09p-61},
        {0x1.52672453c0731p-8, 0x1.834efb598c338p-62},
        {-0x1.5850c5be137cfp-9, -0x1.445fc150ca7f5p-63},
        {0x1.23eb98d22e1cap-10, -0x1.388fbaf1d783p-64},
        {-0x1.8f4e974a40741p-12, 0x1.271198a97da34p-66},
        {0x1.a5cf2e9cf76e5p-14, -0x1.887eb4a63b665p-68},
        {-0x1.420c270719e32p-16, 0x1.efd595b27888bp-71},
        {0x1.3ba2d69b51677p-19, -0x1.4fb06829cdfc7p-73},
        {-0x1.29b7e6f676385p-23, -0x1.a783b6de718fbp-77}};
    // polydd(z2h, z2l, 32, c, &pl)
    int k = 31;
    double ch = c[k][0], cl = c[k][1];
    while (--k >= 0) {
      ch = atan2f_muldd(z2h, z2l, ch, cl, &cl);
      const double th = ch + c[k][0];
      const double tl = (c[k][0] - th) + ch;
      ch = th;
      cl += tl + c[k][1];
    }
    double pl = cl;
    double ph = ch;
    zh *= sgn[gt];
    zl *= sgn[gt];
    ph = atan2f_muldd(zh, zl, ph, pl, &pl);
    const double sh = ph + off[i];
    const double sl = ((off[i] - sh) + ph) + pl + offl[i];
    const float rf = static_cast<float>(sh);
    const double th = rf;
    const double dh = sh - th;
    double tm = dh + sl;
    std::uint64_t tth = asu(th);
    if (th + th * 0x1p-60 == th - th * 0x1p-60) {
      tth &= 0x7ffull << 52;
      tth -= 24ull << 52;
      if (std::fabs(tm) > asd(tth)) tm *= 1.25;
      else tm *= 0.75;
    }
    r = th + tm;
  }
  return static_cast<float>(r);
}

// ---- tanf (CORE-MATH, s_tanf.c: correctly rounded) ----
namespace detail {
CPPWRF_GMF double tanf_rltl(float zf, int* q) {
  const double x = zf;
  const double idl = -0x1.b1bbead603d8bp-32 * x;
  const double idh = 0x1.45f306ep-1 * x;
#if defined(CPPWRF_DEVICE_CODE)
  const double id = ::rint(idh);
#else
  const double id = std::nearbyint(idh);
#endif
  *q = int(static_cast<std::int64_t>(id));
  return (idh - id) + idl;
}
CPPWRF_GMF void tanf_mul64(std::uint64_t a, std::uint64_t b, std::uint64_t* hi, std::uint64_t* lo) {
#if defined(CPPWRF_DEVICE_CODE)
  *hi = __umul64hi(a, b);
  *lo = a * b;
#else
  const unsigned __int128 p = static_cast<unsigned __int128>(a) * b;
  *hi = std::uint64_t(p >> 64);
  *lo = std::uint64_t(p);
#endif
}
CPPWRF_GMF double tanf_rbig(std::uint32_t u, int* q) {
  constexpr std::uint64_t ipi[4] = {0xfe5163abdebbc562ull, 0xdb6295993c439041ull, 0xfc2757d1f534ddc0ull,
                                    0xa2f9836e4e441529ull};
  const int e = (u >> 23) & 0xff;
  const std::uint64_t m = (u & (~0u >> 9)) | 1u << 23;
  // 128-bit products as (hi, lo) pairs; p_{n} += p_{n-1} >> 64
  std::uint64_t h0, l0, h1, l1, h2, l2, h3, l3;
  tanf_mul64(m, ipi[0], &h0, &l0);
  tanf_mul64(m, ipi[1], &h1, &l1);
  l1 += h0;
  h1 += (l1 < h0);
  tanf_mul64(m, ipi[2], &h2, &l2);
  l2 += h1;
  h2 += (l2 < h1);
  tanf_mul64(m, ipi[3], &h3, &l3);
  l3 += h2;
  h3 += (l3 < h2);
  const std::uint64_t p3h = h3, p3l = l3, p2l = l2, p1l = l1;
  std::int64_t a;
  int i;
  const int k = e - 127, s = k - 23;
  if (s < 64) {
    i = int(p3h << s | p3l >> (64 - s));
    a = std::int64_t(p3l << s | p2l >> (64 - s));
  } else if (s == 64) {
    i = int(p3l);
    a = std::int64_t(p2l);
  } else {
    i = int(p3l << (s - 64) | p2l >> (128 - s));
    a = std::int64_t(p2l << (s - 64) | p1l >> (128 - s));
  }
  int sg = int(u);
  sg >>= 31;
  const std::int64_t sm = a >> 63;
  i -= int(sm);
  const double z = double(a ^ sg) * 0x1p-64;
  i = (i ^ sg) - sg;
  *q = i;
  return z;
}
}  // namespace detail
CPPWRF_GMF float tanf(float x) {
  using namespace detail;
  const std::uint32_t t = asuf(x);
  const int e = (t >> 23) & 0xff;
  int i;
  double z;
  if (e < 127 + 28) {
    if (e < 115) {
      if (e < 102) return fmaf(x, std::fabs(x), x);
      const float x2 = x * x;
      return fmaf(x, 0x1.555556p-2f * x2, x);
    }
    z = tanf_rltl(x, &i);
  } else if (e < 0xff) {
    z = tanf_rbig(t, &i);
  } else {
    if (t << 9) return x + x;
    return invalidf(x);
  }
  const double z2 = z * z, z4 = z2 * z2;
  constexpr double cn[4] = {0x1.921fb54442d18p+0, -0x1.fd226e573289fp-2, 0x1.b7a60c8dac9f6p-6,
                            -0x1.725beb40f33e5p-13};
  constexpr double cd[4] = {0x1p+0, -0x1.2395347fb829dp+0, 0x1.2313660f29c36p-3, -0x1.9a707ab98d1c1p-9};
  constexpr double sv[2] = {0, 1};
  double n = cn[0] + z2 * cn[1];
  const double n2 = cn[2] + z2 * cn[3];
  n += z4 * n2;
  double d = cd[0] + z2 * cd[1];
  const double d2 = cd[2] + z2 * cd[3];
  d += z4 * d2;
  n *= z;
  const double s0 = sv[i & 1], s1 = sv[1 - (i & 1)];
  const double r1 = (n * s1 - d * s0) / (n * s0 + d * s1);
  // (the 2.41 source's table of hard cases compares asfloat(arg) with ax and never matches a finite argument
  // above 2^-26: r1 is the result; tools/glibc_mathf_proof checks every input)
  return static_cast<float>(r1);
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

// ---- asinf, acosf, erff, tgammaf, sinhf (CORE-MATH, correctly rounded; generic C in libm, no FMA variant) ----
namespace detail {
CPPWRF_GMF double sqrtd(double x) {
#if defined(CPPWRF_DEVICE_CODE)
  return CPPWRF_DSQRT_RN(x);
#else
  return std::sqrt(x);
#endif
}
CPPWRF_GMF double floord(double x) {
#if defined(CPPWRF_DEVICE_CODE)
  return ::floor(x);
#else
  return std::floor(x);
#endif
}
CPPWRF_GMF float floorff(float x) {
#if defined(CPPWRF_DEVICE_CODE)
  return ::floorf(x);
#else
  return std::floor(x);
#endif
}
// roundeven (finite arguments)
CPPWRF_GMF double roundevend(double x) {
#if defined(CPPWRF_DEVICE_CODE)
  return ::rint(x);
#else
  return std::nearbyint(x);
#endif
}
CPPWRF_GMF double fabsd(double x) { return asd(asu(x) & 0x7fffffffffffffffull); }
CPPWRF_GMF float fabsff(float x) { return asf(asuf(x) & 0x7fffffffu); }
CPPWRF_GMF double copysignd(double y, double x) {
  return asd((asu(y) & 0x7fffffffffffffffull) | (asu(x) & 0x8000000000000000ull));
}
CPPWRF_GMF float copysignff(float y, float x) { return asf((asuf(y) & 0x7fffffffu) | (asuf(x) & 0x80000000u)); }
// poly12 of e_asinf.c / e_acosf.c
CPPWRF_GMF double asinf_poly12(double z, const double* c) {
  const double z2 = z * z, z4 = z2 * z2;
  double c0 = c[0] + z * c[1];
  const double c2 = c[2] + z * c[3];
  double c4 = c[4] + z * c[5];
  const double c6 = c[6] + z * c[7];
  double c8 = c[8] + z * c[9];
  const double c10 = c[10] + z * c[11];
  c0 += c2 * z2;
  c4 += c6 * z2;
  c8 += z2 * c10;
  c0 += z4 * (c4 + z4 * c8);
  return c0;
}
}  // namespace detail

// ---- asinf (CORE-MATH, e_asinf.c) ----
CPPWRF_GMF float asinf(float x) {
  using namespace detail;
  const double pi2 = 0x1.921fb54442d18p+0;
  const double xs = x;
  double r;
  const std::uint32_t ax = asuf(x) << 1;
  if (ax > (0x7fu << 24)) {
    if (ax > (0xffu << 24)) return x + x;
    return invalidf(0.0f);
  }
  if (ax < 0x7ec29000u) {
    if (ax < (115u << 24)) return fmaf(x, 0x1p-25f, x);
    constexpr double b[16] = {0x1.0000000000005p+0,  0x1.55557aeca105dp-3,  0x1.3314ec3db7d12p-4,
                              0x1.775738a5a6f92p-5,  0x1.5d5f7ce1c8538p-8,  0x1.605c6d58740fp-2,
                              -0x1.5728b732d73c6p+1, 0x1.f152170f151ebp+3,  -0x1.f962ea3ca992ep+5,
                              0x1.71971e17375ap+7,   -0x1.860512b4ba23p+8,  0x1.26a3b8d4bdb14p+9,
                              -0x1.36f2ea5698b51p+9, 0x1.b3d722aebfa2ep+8,  -0x1.6cf89703b1289p+7,
                              0x1.1518af6a65e2dp+5};
    const double z = xs, z2 = z * z, z4 = z2 * z2, z8 = z4 * z4, z16 = z8 * z8;
    r = z * ((((b[0] + z2 * b[1]) + z4 * (b[2] + z2 * b[3])) + z8 * ((b[4] + z2 * b[5]) + z4 * (b[6] + z2 * b[7]))) +
             z16 * (((b[8] + z2 * b[9]) + z4 * (b[10] + z2 * b[11])) +
                    z8 * ((b[12] + z2 * b[13]) + z4 * (b[14] + z2 * b[15]))));
    const float ub = static_cast<float>(r);
    const float lb = static_cast<float>(r - z * 0x1.efa8ebp-31);
    if (ub == lb) return ub;
  }
  if (ax < (0x7eu << 24)) {
    constexpr double c[12] = {0x1.555555555529cp-3, 0x1.333333337e0ddp-4, 0x1.6db6db3b4465ep-5,
                              0x1.f1c72e13ac306p-6, 0x1.6e89cebe06bc4p-6, 0x1.1c6dcf5289094p-6,
                              0x1.c6dbbcc7c6315p-7, 0x1.8f8dc2615e996p-7, 0x1.a5833b7bf15e8p-8,
                              0x1.43f44ace1665cp-6, -0x1.0fb17df881c73p-6, 0x1.07520c026b2d6p-5};
    const double z = xs, z2 = z * z;
    const double c0 = asinf_poly12(z2, c);
    r = z + (z * z2) * c0;
  } else {
    if (ax == 0x7e55688au) return copysignff(0x1.75b8a2p-1f, x) + copysignff(0x1p-26f, x);
    if (ax == 0x7e107434u) return copysignff(0x1.1f4b64p-1f, x) + copysignff(0x1p-26f, x);
    const double bx = fabsd(xs), z = 1.0 - bx, s = sqrtd(z);
    constexpr double c[12] = {0x1.6a09e667f3bcbp+0,  0x1.e2b7dddff2db9p-4,  0x1.b27247ab42dbcp-6,
                              0x1.02995cc4e0744p-7,  0x1.5ffb0276ec8eap-9,  0x1.033885a928decp-10,
                              0x1.911f2be23f8c7p-12, 0x1.4c3c55d2437fdp-13, 0x1.af477e1d7b461p-15,
                              0x1.abd6bdff67dcbp-15, -0x1.1717e86d0fa28p-16, 0x1.6ff526de46023p-16};
    r = pi2 - s * asinf_poly12(z, c);
    r = copysignd(r, xs);
  }
  return static_cast<float>(r);
}

// ---- acosf (CORE-MATH, e_acosf.c) ----
CPPWRF_GMF float acosf(float x) {
  using namespace detail;
  const double pi2 = 0x1.921fb54442d18p+0;
  constexpr double o[2] = {0, 0x1.921fb54442d18p+1};
  const double xs = x;
  double r;
  const std::uint32_t t = asuf(x);
  const std::uint32_t ax = t << 1;
  if (ax >= (0x7fu << 24)) {  // as_special
    if (t == (0x7fu << 23)) return 0.0f;
    if (t == (0x17fu << 23)) return 0x1.921fb6p+1f + -0x1p-24f;
    if (ax > (0xffu << 24)) return x + x;
    return invalidf(0.0f);
  }
  if (ax < 0x7ec2a1dcu) {
    constexpr double b[16] = {0x1.fffffffd9ccb8p-1,  0x1.5555c94838007p-3,  0x1.32ded4b7c20fap-4,
                              0x1.8566df703309ep-5,  -0x1.980c959bec9a3p-6, 0x1.56fbb04998344p-1,
                              -0x1.403d8e4c49f52p+2, 0x1.b06c3e9f311eap+4,  -0x1.9ea97c4e2c21fp+6,
                              0x1.200b8261cc61bp+8,  -0x1.2274c2799a5c7p+9, 0x1.a558a59cc19d3p+9,
                              -0x1.aca4b6a529ffp+9,  0x1.228744703f813p+9,  -0x1.d7dbb0b322228p+7,
                              0x1.5c2018c0c0105p+5};
    if (ax <= 0x40000000u) return static_cast<float>(pi2);
    const double z = xs, z2 = z * z, z4 = z2 * z2, z8 = z4 * z4, z16 = z8 * z8;
    r = z * ((((b[0] + z2 * b[1]) + z4 * (b[2] + z2 * b[3])) + z8 * ((b[4] + z2 * b[5]) + z4 * (b[6] + z2 * b[7]))) +
             z16 * (((b[8] + z2 * b[9]) + z4 * (b[10] + z2 * b[11])) +
                    z8 * ((b[12] + z2 * b[13]) + z4 * (b[14] + z2 * b[15]))));
    const float ub = static_cast<float>(0x1.921fb54574191p+0 - r);
    const float lb = static_cast<float>(0x1.921fb543118ap+0 - r);
    if (ub == lb) return ub;
  }
  if (ax < (0x7eu << 24)) {
    constexpr double c[12] = {0x1.555555555529cp-3, 0x1.333333337e0ddp-4, 0x1.6db6db3b4465ep-5,
                              0x1.f1c72e13ac306p-6, 0x1.6e89cebe06bc4p-6, 0x1.1c6dcf5289094p-6,
                              0x1.c6dbbcc7c6315p-7, 0x1.8f8dc2615e996p-7, 0x1.a5833b7bf15e8p-8,
                              0x1.43f44ace1665cp-6, -0x1.0fb17df881c73p-6, 0x1.07520c026b2d6p-5};
    if (t == 0x328885a3u) return static_cast<float>(double(0x1.921fb6p+0f) + 0x1p-25);
    if (t == 0x39826222u) return static_cast<float>(double(0x1.920f6ap+0f) + 0x1p-25);
    const double x2 = xs * xs;
    r = (pi2 - xs) - (xs * x2) * asinf_poly12(x2, c);
  } else {
    constexpr double c[12] = {0x1.6a09e667f3bcbp+0,  0x1.e2b7dddff2db9p-4,  0x1.b27247ab42dbcp-6,
                              0x1.02995cc4e0744p-7,  0x1.5ffb0276ec8eap-9,  0x1.033885a928decp-10,
                              0x1.911f2be23f8c7p-12, 0x1.4c3c55d2437fdp-13, 0x1.af477e1d7b461p-15,
                              0x1.abd6bdff67dcbp-15, -0x1.1717e86d0fa28p-16, 0x1.6ff526de46023p-16};
    const double bx = fabsd(xs), z = 1.0 - bx;
    const double s = copysignd(sqrtd(z), xs);
    r = o[t >> 31] + s * asinf_poly12(z, c);
  }
  return static_cast<float>(r);
}

// ---- erff (CORE-MATH, s_erff.c) ----
CPPWRF_GMF float erff(float x) {
  using namespace detail;
  const float ax = fabsff(x);
  const std::uint32_t ux = asuf(ax);
  const double s = x;
  double z = ax;
  if (ux > 0x407ad444u) {
    const float os = copysignff(1.0f, x);
    if (ux > (0xffu << 23)) return x + x;
    if (ux == (0xffu << 23)) return os;
    return os - 0x1p-25f * os;
  }
  const double v = floord(16.0 * z);
  const std::uint32_t i = static_cast<std::uint32_t>(16.0f * ax);
  if (ux < 0x3ee00000u) {
    constexpr double c[8] = {0x1.20dd750429b6dp+0,  -0x1.812746b0375fbp-2, 0x1.ce2f219fd6f45p-4,
                             -0x1.b82ce2cbf0838p-6, 0x1.565bb655adb85p-8,  -0x1.c025bfc879c94p-11,
                             0x1.f81718f61309cp-14, -0x1.cc67bd88f5867p-17};
    const double z2 = s * s, z4 = z2 * z2, z8 = z4 * z4;
    double c0 = c[0] + z2 * c[1];
    const double c2 = c[2] + z2 * c[3];
    double c4 = c[4] + z2 * c[5];
    const double c6 = c[6] + z2 * c[7];
    c0 += z4 * c2;
    c4 += z4 * c6;
    c0 += z8 * c4;
    return static_cast<float>(s * c0);
  }
  z = (z - 0.03125) - 0.0625 * v;
  const double* c = CPPWRF_GFT(kErffC) + 8 * (i - 7);
  const double z2 = z * z, z4 = z2 * z2;
  double c0 = c[0] + z * c[1];
  const double c2 = c[2] + z * c[3];
  double c4 = c[4] + z * c[5];
  const double c6 = c[6] + z * c[7];
  c0 += z2 * c2;
  c4 += z2 * c6;
  c0 += z4 * c4;
  return static_cast<float>(copysignd(c0, s));
}

// ---- tgammaf (CORE-MATH, e_gammaf_r.c, with the w_tgammaf_compat wrapper: errno only) ----
CPPWRF_GMF float tgammaf(float x) {
  using namespace detail;
  // exceptional cases: x, a binary32 approximation of gamma(x), a correction term
  constexpr std::uint32_t tbu[10] = {0x27de86a9u, 0x27e05475u, 0xb63befb3u, 0x3c7bb570u, 0x41e886d1u,
                                     0xc067d177u, 0xbd99da31u, 0xbf54c45au, 0x41ee77feu, 0x3f843a64u};
  constexpr float tbf[10] = {0x1.268266p+47f, 0x1.242422p+47f, -0x1.5cb6e4p+18f, 0x1.021d9p+6f,
                             0x1.33136ap+98f, 0x1.f6850cp-3f,  -0x1.befe66p+3f,  -0x1.a6b4ecp+2f,
                             0x1.d3631cp+101f, 0x1.f6c638p-1f};
  constexpr float tbdf[10] = {0x1p22f, 0x1p22f, 0x1p-7f, 0x1p-19f, 0x1p73f, 0x1p-28f, -0x1p-22f, 0x1p-23f,
                              -0x1p-76f, 0x1p-26f};
  const std::uint32_t t = asuf(x);
  const std::uint32_t ax = t << 1;
  if (ax >= (0xffu << 24)) {
    if (ax == (0xffu << 24)) {
      if (t >> 31) return invalidf(x);
      return x;
    }
    return x + x;
  }
  double z = x;
  if (ax < 0x6d000000u) {
    const double d = (0x1.fa658c23b1578p-1 - 0x1.d0a118f324b63p-1 * z) * z - 0x1.2788cfc6fb619p-1;
    const double f = 1.0 / z + d;
    const float r = static_cast<float>(f);
    const std::uint64_t rt = asu(f);
    if (((rt + 2) & 0xfffffff) < 4) {
      for (int i = 0; i < 10; i++)
        if (t == tbu[i]) return tbf[i] + tbdf[i];
    }
    return r;
  }
  const float fx = floorff(x);
  if (x >= 0x1.18522p+5f) return x * 0x1p127f;
  const int k = static_cast<int>(fx);
  if (fx == x) {
    if (x == 0.0f) return 1.0f / x;
    if (x < 0.0f) return invalidf(0.0f);
    double t0 = 1, x0 = 1;
    for (int i = 1; i < k; i++, x0 += 1.0) t0 *= x0;
    return static_cast<float>(t0);
  }
  if (x < -42.0f) {
    constexpr float sgn[2] = {0x1p-127f, -0x1p-127f};
    return 0x1p-127f * sgn[k & 1];
  }
  constexpr double c[16] = {0x1.c9a76be577123p+0,  0x1.8f2754ddcf90dp+0,  0x1.0d1191949419bp+0,
                            0x1.e1f42cf0ae4a1p-2,  0x1.82b358a3ab638p-3,  0x1.e1f2b30cd907bp-5,
                            0x1.240f6d4071bd8p-6,  0x1.1522c9f3cd012p-8,  0x1.1fd0051a0525bp-10,
                            0x1.9808a8b96c37ep-13, 0x1.b3f78e01152b5p-15, 0x1.49c85a7e1fd04p-18,
                            0x1.471ca49184475p-19, -0x1.368f0b7ed9e36p-23, 0x1.882222f9049efp-23,
                            -0x1.a69ed2042842cp-25};
  const double m = z - 0x1.7p+1;
  const double i = roundevend(m);
  const double step = copysignd(1.0, i);
  const double d = m - i, d2 = d * d, d4 = d2 * d2, d8 = d4 * d4;
  double f = (c[0] + d * c[1]) + d2 * (c[2] + d * c[3]) +
             d4 * ((c[4] + d * c[5]) + d2 * (c[6] + d * c[7])) +
             d8 * ((c[8] + d * c[9]) + d2 * (c[10] + d * c[11]) +
                   d4 * ((c[12] + d * c[13]) + d2 * (c[14] + d * c[15])));
  const int jm = static_cast<int>(fabsd(i));
  double w = 1;
  if (jm) {
    z -= 0.5 + step * 0.5;
    w = z;
    for (int j = jm - 1; j; j--) {
      z -= step;
      w *= z;
    }
  }
  if (i <= -0.5) w = 1 / w;
  f *= w;
  const std::uint64_t rt = asu(f);
  const float r = static_cast<float>(f);
  if (((rt + 2) & 0xfffffff) < 8) {
    for (int j = 0; j < 10; j++)
      if (t == tbu[j]) return tbf[j] + tbdf[j];
  }
  return r;
}

// ---- sinhf (CORE-MATH, e_sinhf.c) ----
CPPWRF_GMF float sinhf(float x) {
  using namespace detail;
  constexpr double c[4] = {1, 0x1.62e42fef4c4e7p-6, 0x1.ebfd1b232f475p-13, 0x1.c6b19384ecd93p-20};
  constexpr double ch[7] = {1, 0x1.62e42fefa39efp-6, 0x1.ebfbdff82c58fp-13, 0x1.c6b08d702e0edp-20,
                            0x1.3b2ab6fb92e5ep-27, 0x1.5d886e6d54203p-35, 0x1.430976b8ce6efp-43};
  const double iln2 = 0x1.71547652b82fep+5;
  const double z = x;
  const std::uint32_t ux = asuf(x) << 1;
  if (ux > 0x8565a9f8u) {
    const float sgn = copysignff(2.0f, x);
    if (ux >= 0xff000000u) {
      if (ux << 8) return x + x;
      return copysignff(INFINITY, x);
    }
    return sgn * 0x1.fffffep127f;
  }
  if (ux < 0x7c000000u) {
    if (ux <= 0x74250bfeu) {
      if (ux < 0x66000000u) return fmaf(x, fabsff(x), x);
      // |x| = 0x1.250bfep-11 (the 2.41 tarball compares st.uarg with asuint (ux), the bits of float(ux), which never
      // matches; Debian's 2.41 compares it with ux, as CORE-MATH, and the proof shows that libm takes this case)
      if (ux == 0x74250bfeu) {
        const float sgn = copysignff(1.0f, x);
        return sgn * 0x1.250bfep-11f + sgn * 0x1p-36f;
      }
      return (x * 0x1.555556p-3f) * (x * x) + x;
    }
    constexpr double cp[4] = {0x1.5555555555555p-3, 0x1.11111111146e1p-7, 0x1.a01a00930dda6p-13,
                              0x1.71f92198aa6e9p-19};
    const double z2 = z * z, z4 = z2 * z2;
    return static_cast<float>(z + (z2 * z) * ((cp[0] + z2 * cp[1]) + z4 * (cp[2] + z2 * (cp[3]))));
  }
  const double a = iln2 * z;
  const double ia = roundevend(a);
  double h = a - ia;
  double h2 = h * h;
  const std::int64_t jp = static_cast<std::int64_t>(asu(ia + 0x1.8p52));
  const std::int64_t jm = -jp;
  const double sp = asd(CPPWRF_GFT(kSinhfTb)[jp & 31] + (static_cast<std::uint64_t>(jp >> 5) << 52));
  const double sm = asd(CPPWRF_GFT(kSinhfTb)[jm & 31] + (static_cast<std::uint64_t>(jm >> 5) << 52));
  double te = c[0] + h2 * c[2];
  double to = (c[1] + h2 * c[3]);
  const double rp = sp * (te + h * to);
  const double rm = sm * (te - h * to);
  double r = rp - rm;
  float ub = static_cast<float>(r);
  const double lb = r - 1.52e-10 * r;
  if (double(ub) != lb) {
    const double iln2h = 0x1.7154765p+5;
    const double iln2l = 0x1.5c17f0bbbe88p-26;
    h = (iln2h * z - ia) + iln2l * z;
    h2 = h * h;
    te = ch[0] + h2 * ch[2] + (h2 * h2) * (ch[4] + h2 * ch[6]);
    to = ch[1] + h2 * (ch[3] + h2 * ch[5]);
    r = sp * (te + h * to) - sm * (te - h * to);
    ub = static_cast<float>(r);
  }
  return ub;
}
}  // namespace cppwrf::glibcf
#if defined(__CUDACC__) && !defined(__HIP__)
#pragma nv_diagnostic pop
#endif
