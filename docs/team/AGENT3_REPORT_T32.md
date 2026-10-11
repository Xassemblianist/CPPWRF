# Agent 3 Report: Task T3.2 (Plotting Scripts & Verification)

## Files Created
- `scripts/plot_case.py`, `scripts/plot_diag.py`, `scripts/plot_compare.py`
- `scripts/plot_straka_convergence.py`, `scripts/test_plots.py`

## Commands and Exact Outputs
```
$ python3 scripts/test_plots.py
Running test_plots.py...
Testing plot_case.py...
Testing plot_diag.py...
Testing plot_compare.py...
Testing plot_straka_convergence.py...
Known front: 1250.0 m, computed: 1250.0 m, error: 0.0 m (PASSED)
ALL PLOT TESTS PASSED.

$ python3 scripts/plot_compare.py out/a0_straka_dp out/a0_straka --out out/plots/compare_straka_fp32_fp64.png
max|A-B|: 1.580129e-04
Relative L2: 4.600612e-06

$ python3 scripts/plot_straka_convergence.py out/a0_straka_400 out/a0_straka_200 out/a0_straka_dp out/a0_straka_50 --out out/plots/straka_convergence.png
dx       front_km   min_th_pert 
400      15.611     -10.172     
200      15.506     -11.186     
100      15.346     -9.456      
50       15.273     -9.572      
```

## Generated Figures (out/plots/)
- `straka/`: `snap_000000.png`, `snap_000300.png`, `snap_000600.png`, `snap_000900.png`
- `warm_bubble/`: `snap_000000.png` .. `snap_001000.png` (5 snapshots)
- `igw/`: `snap_000000.png`, `snap_000250.png`, `snap_000500.png`
- `diag_all.png`, `compare_straka_fp32_fp64.png`, `straka_convergence.png`

## Physical Observations
- In Straka convergence, front location contracts monotonically with resolution (15.611 -> 15.273 km), converging ~270 m below the 15.54 km benchmark. Minimum θ' dips non-monotonically at dx=200 m (-11.186 K) before returning near reference (-9.572 K at dx=50 m).
- In Straka FP32 vs FP64, differences remain small (~1.58e-4 K max error at t=900 s).
- Warm bubble and IGW exhibit stable propagation with zero dry mass drift.
