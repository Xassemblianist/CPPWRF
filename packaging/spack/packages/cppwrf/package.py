# SPDX-License-Identifier: GPL-3.0-or-later
# Spack recipe for CPPWRF (untested: no Spack installation was at hand when it was written; docs/INSTALL.md).
#   spack repo add packaging/spack   (the repository of this recipe: repo.yaml, namespace cppwrf), then
#   spack install cppwrf precision=double              # CPU FP64
#   spack install cppwrf +cuda cuda_arch=89 precision=single
from spack.package import *


class Cppwrf(CMakePackage, CudaPackage):
    """C++20 / CUDA port of the WRF-ARW model v4.8.0; CPU FP64 and CUDA FP64 output bit-identical to WRF real*8."""

    homepage = "https://github.com/Xassemblianist/CPPWRF"
    git = "https://github.com/Xassemblianist/CPPWRF.git"
    url = "https://github.com/Xassemblianist/CPPWRF/archive/refs/tags/v1.0.0.tar.gz"

    license("GPL-3.0-or-later")

    version("main", branch="main")
    version("1.0.0", tag="v1.0.0")

    variant("precision", default="double", values=("double", "single"), multi=False,
            description="Working precision: double (bit-identical to WRF real*8) or single (the fast GPU mode)")
    variant("native", default=False, description="Tune for the build machine (-march=native)")

    depends_on("cxx", type="build")
    depends_on("cmake@3.24:", type="build")
    depends_on("netcdf-c")
    depends_on("python@3.9:", type="run")
    depends_on("py-numpy", type="run")
    depends_on("py-netcdf4", type="run")
    depends_on("cuda@12:", when="+cuda")
    # the arithmetic of the bit identity: gcc (glibc's libmvec); the proofs use gcc 14
    requires("%gcc@12:", msg="CPPWRF's bit identity with WRF assumes GNU compilers")
    conflicts("cuda_arch=none", when="+cuda", msg="name the GPU architecture: cuda_arch=<N>")

    def cmake_args(self):
        args = [
            self.define("CPPWRF_ENABLE_CUDA", self.spec.satisfies("+cuda")),
            self.define("CPPWRF_DOUBLE", self.spec.satisfies("precision=double")),
            self.define_from_variant("CPPWRF_NATIVE", "native"),
        ]
        if self.spec.satisfies("+cuda"):
            args.append(self.define("CMAKE_CUDA_ARCHITECTURES", ";".join(self.spec.variants["cuda_arch"].value)))
        return args

    def setup_run_environment(self, env):
        env.prepend_path("PYTHONPATH", self.prefix.share.cppwrf.python)
