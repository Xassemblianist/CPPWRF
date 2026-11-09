# SPDX-License-Identifier: GPL-3.0-or-later
"""docs/PERF.md's measurements on the Colab G4 runtime (AMD EPYC 9B45 with 48 threads, NVIDIA RTX PRO 6000 Blackwell),
the reference of cppwrf-bench and the physics scaling of cppwrf-fit. ctest python_package checks that these numbers
are docs/PERF.md's (test_tools.py)."""

# ns per cell and step, history off (docs/PERF.md, "Time per step and per cell"): R1 dynamics only; P4 full physics
# with radt 1 min; P4R15 the same with radt 15 min; L the 9x larger domain with radt 15 min
NS_PER_CELL_STEP = {
    "R1": {"gpu-fp32": 4.2, "gpu-fp64": 18.2, "cpu-fp64": 134.7},
    "P4": {"gpu-fp32": 59.3, "gpu-fp64": 99.4, "cpu-fp64": 389.0},
    "P4R15": {"gpu-fp32": 19.7, "gpu-fp64": 60.2, "cpu-fp64": 266.0},
    "L": {"gpu-fp32": 11.8, "gpu-fp64": 37.6, "cpu-fp64": 200.0},
}
CELLS = {"R1": 119 * 89 * 44, "P4": 119 * 89 * 44, "P4R15": 119 * 89 * 44, "L": 359 * 269 * 44}

# GPU memory peaks in MiB (docs/PERF.md, "GPU memory"): (peak, stack reserve) per case and precision
GPU_MEMORY_MIB = {
    "R1": {"gpu-fp32": (803, 282), "gpu-fp64": (987, 282)},
    "P4": {"gpu-fp32": (8297, 3018), "gpu-fp64": (13749, 7989)},
    "L": {"gpu-fp32": (20791, 3018), "gpu-fp64": (29957, 7989)},
}
REFERENCE_MACHINE = "Colab G4: AMD EPYC 9B45 (48 threads) / NVIDIA RTX PRO 6000 Blackwell (docs/PERF.md)"


def physics_time_factor(mode: str, case: str) -> float:
    """How much slower than the dynamics-only R1 a step of `case` is in `mode` (cpu-fp32: as cpu-fp64)."""
    m = "cpu-fp64" if mode == "cpu-fp32" else mode
    return NS_PER_CELL_STEP[case][m] / NS_PER_CELL_STEP["R1"][m]


def physics_memory_factor(mode: str) -> float:
    """Bytes per cell of the full physics (the large domain L) over the dynamics-only R1's, stack reserve aside (CPU
    modes: the GPU FP64 factor; not measured on the CPU)."""
    m = mode if mode.startswith("gpu") else "gpu-fp64"
    (pl, rl), (pr, rr) = GPU_MEMORY_MIB["L"][m], GPU_MEMORY_MIB["R1"][m]
    return ((pl - rl) / CELLS["L"]) / ((pr - rr) / CELLS["R1"])


def stack_reserve_factor(mode: str) -> float:
    """The full physics' stack reserve over the dynamics-only one (GPU)."""
    return GPU_MEMORY_MIB["P4"][mode][1] / GPU_MEMORY_MIB["R1"][mode][1]
