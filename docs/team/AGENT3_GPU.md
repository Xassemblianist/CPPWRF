# Agent 3 task T3.4: GPU validation, benchmarks and profiles

Start only after T3.3 (docs/team/AGENT3_WRF_REF.md) is finished and its report is written.
You are Agent 3 in CPPWRF (/home/ostriquetrum/CPPWRF). Read docs/team/PLAN.md first.
The NVIDIA driver now works: `nvidia-smi` shows an RTX 2060 (6 GB, sm_75, driver 550).
Goal: prove that the CUDA build gives the same atmosphere as the CPU build, and measure how fast
each one is, with numbers someone can trust.

## Hard rules
1. Create or edit only: tools/gpu/ (new, yours), scripts/ (yours), docs/team/AGENT3_REPORT_T34.md,
   docs/PERF_MEASURED.md (new, yours). Write run output under out/gpu/ only.
2. Do NOT edit any C++ source, CMakeLists.txt, configs/ or tests/. If the CUDA build fails or
   gives wrong results, STOP and report the exact error. Do not fix the model.
3. Never run git write commands. Never use sudo.
4. Every number in the report must come from a command whose output you paste or save
   under out/gpu/logs/.
5. Do not run other heavy jobs during a timing run (no parallel builds/runs while timing).

## Builds (directory names are fixed)
- build-cuda     : `cmake -S . -B build-cuda -G Ninja -DCPPWRF_ENABLE_CUDA=ON` (FP32)
- build-cuda-dp  : same plus -DCPPWRF_DOUBLE=ON (FP64)
- build-cpu, build-cpu-dp already exist; rebuild them (`cmake --build`) so all four use the same commit.
If nvcc rejects the host compiler, retry with `-DCMAKE_CUDA_HOST_COMPILER=g++-13` (or the
newest g++ that nvcc accepts) and record it.
Run `ctest` in every build dir; paste the summaries.

## 1. Correctness: GPU vs CPU (tools/gpu/validate.sh)
For each case in configs/ (rest_state, straka, warm_bubble, igw) and each precision:
run CPU and CUDA into out/gpu/<case>_<cpu|cuda>_<fp32|fp64>/ via `--set output.dir=...`;
run `python3 tests/verify/verify.py check <case> <dir>` on each, and
`python3 tests/verify/verify.py compare <cpu_dir> <cuda_dir> --rtol <tol>` with
tol = 1e-9 for FP64 and 1e-3 for FP32. Record every line that is not PASS.
Note: GPU floating-point results are not expected to be bitwise equal to CPU (FMA, ordering);
small differences are fine, growing or structured ones are not. If compare fails, save
plot_compare.py figures for th_pert, w and u into out/gpu/plots/.
Also run the same CUDA case twice and check bitwise identity (cmp of the last snapshot).

## 2. Speed (tools/gpu/bench.sh)
Disable output for timing: `--set output.out_interval=1000000 --set output.diag_interval=1000000`
(check main.cpp/configs for the exact keys; record what you used).
Use the "ns/cell/step" line printed by cppwrf (and wall time). For each measurement run 3 times,
report the median.
a) Straka (2D) at nx x nz = 512x64, 1024x128, 2048x256 (dx = 100, 50, 25 m; dt = 1, 0.5, 0.25),
   run_time = 60 s.
b) 3D: warm_bubble config made 3D through --set overrides: ny = nx, bc_y = symmetric, dy = dx,
   at nx = ny = 64, 128, 192 with nz = 64 (stop if GPU memory runs out; record it),
   run_time = 20 steps' worth.
c) Precisions FP32 and FP64, backends CPU (OMP_NUM_THREADS=12 and =1) and CUDA.
Write docs/PERF_MEASURED.md: a table (case | grid | cells | backend | precision | threads |
ns/cell/step | speedup vs CPU-12 | speedup vs CPU-1). Include `nvidia-smi -q -d CLOCK,POWER`
output taken during a CUDA run (the laptop may be power-limited: note whether on AC power).

## 3. Profile (tools/gpu/profile.sh)
On the 3D 128x128x64 FP32 CUDA run (20 steps):
- `nsys profile --stats=true -o out/gpu/prof/nsys_3d ./build-cuda/cppwrf ...` and save the
  kernel summary table (cuda_gpu_kern_sum): top 15 kernels by total time, instances, avg time.
- Count how many kernel launches happen per time step and how much time goes to fill_halo
  launches (their names contain "HaloLine" or the halo kernel struct name).
- `ncu --set basic -k regex:"KW|KMuT|KUV|KTendU|KTendT" -c 10 ...` for achieved DRAM
  throughput (GB/s) and occupancy of those kernels. If ncu needs permissions it does not have
  (ERR_NVGPUCTRPERM), write that down and skip ncu; do not use sudo.
Report the RTX 2060's peak DRAM bandwidth (from `nvidia-smi -q` memory clock and bus width,
show the arithmetic) next to the measured GB/s.

## 4. Report: docs/team/AGENT3_REPORT_T34.md (max 60 lines)
Build results, the correctness table (case x precision: check result, compare rel L2 per field),
determinism result, the speed table (or a pointer to PERF_MEASURED.md), the top-15 kernel table,
launches per step, halo-fill share of GPU time, ncu numbers, and anything that looked wrong.
Then stop.
