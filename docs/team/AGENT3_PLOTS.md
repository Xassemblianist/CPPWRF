# Agent 3 task T3.2: plotting scripts for model output

You are Agent 3 in the CPPWRF project (/home/ostriquetrum/CPPWRF). Read docs/team/PLAN.md
first. This task is self-contained: Python scripts that turn model output into PNG figures
so humans can check the simulations by eye.

## Hard rules
1. Create or edit files ONLY in: scripts/ and docs/team/AGENT3_REPORT_T32.md.
   Write PNGs ONLY under out/plots/. Do not modify anything else under out/.
2. **Never run any git command that writes (add, commit, amend, checkout, reset, stash, init).**
   Only Agent 1 uses git. In T3.1 you ran `git commit --amend`; do not do that again.
3. Do not run or rebuild the model (./build-*/cppwrf), and do not edit C++ code.
4. Never claim something works without pasting the exact command and its real output.
5. Python 3 + numpy + matplotlib only (already installed). Use the Agg backend
   (`matplotlib.use("Agg")`) because there is no display.
6. Read the data with your own scripts/cppwrf_io.py (read_run, read_diag, list_snapshots,
   read_snapshot). Do not import anything from tests/.

## Input data (already on disk, produced by the model)
Run directories: out/a0_straka, out/a0_straka_dp, out/a0_warm_bubble, out/a0_igw,
out/a0_rest, out/a0_straka_400, out/a0_straka_200, out/a0_straka_50.
Each has run.json, diag.csv and snap_<step>.bin/.json. Snapshot arrays have shape
[nz, ny, nx] with these staggerings:
- th_pert, theta, p, z_m, ...: mass points, shape [nz, ny, nx]
- u: [nz, ny, nx+1] (cell faces in x). Average neighbours to mass points:
  0.5 * (u[..., :-1] + u[..., 1:])
- w, ph: [nz+1, ny, nx] (cell faces in z). Average to mass levels: 0.5 * (w[:-1] + w[1:])
- mu: 2D, shape [1, ny, nx]
x of mass point i = (i + 0.5) * dx (dx from run.json; find it by key, wherever it sits in
the JSON). Use z_m (metres) as the vertical coordinate: height of mass points, per column.
For 3D runs (ny > 1) plot the slice j = ny // 2. Plot axes in km.

## 1. scripts/plot_case.py
`python3 scripts/plot_case.py <run_dir> [--out out/plots/<name>] [--steps all|last]`
For every selected snapshot, write one PNG with 3 panels stacked vertically: th_pert, w, u
(x-z slices, filled contours with a colorbar, title "<case> t=<time> s <var>").
Case-specific contour levels for th_pert, so plots are comparable with the literature:
- straka: levels -16.5 to -0.5 every 1 K. Also draw the -1 K contour as a black line.
  Plot only x >= domain centre and z <= 4.6 km (the usual Straka figure). The centre is the
  middle of the domain.
- igw: levels from -0.0015 to 0.003 every 0.0005 K. Skip the zero level.
- warm_bubble: levels 0.25 to 2.25 every 0.25 K.
- anything else: automatic levels, symmetric about 0 when the field has both signs.
Find the case name in run.json (key "case").

## 2. scripts/plot_diag.py
`python3 scripts/plot_diag.py <run_dir> [<run_dir> ...] [--out file.png]`
Time series from diag.csv: max_abs_w, min_th_pert, max_th_pert, and the relative drifts
(x - x0)/x0 of dry_mass and theta_mass. One subplot per quantity, one line per run directory,
with the directory names in the legend. Use a log y-axis for |drift| (plot abs, guard zeros).

## 3. scripts/plot_compare.py
`python3 scripts/plot_compare.py <run_a> <run_b> [--var th_pert] [--out file.png]`
Last common snapshot: 3 panels showing A, B and A-B for one variable. The difference panel
uses a symmetric colour scale. Print max|A-B| and the relative L2 difference to stdout.
Must refuse (clear error, exit code 2) if the shapes differ.

## 4. scripts/plot_straka_convergence.py
`python3 scripts/plot_straka_convergence.py <dir1> <dir2> ... [--out file.png]`
For each Straka run: th_pert at the last snapshot, side by side (same levels as above),
titled with dx. Then a small panel: front position vs dx. Front = the outermost x (from the
centre, right side) where th_pert <= -1 K at the lowest model level, linearly interpolated.
Print a table "dx  front_km  min_th_pert" to stdout.

## 5. Tests: scripts/test_plots.py
Write synthetic run directories in a temporary directory (reuse the writer logic from
scripts/test_cppwrf_io.py), one per case type, including a 3D one (ny = 4). Run every script
on them through subprocess. Check that each exits 0 and writes a non-empty PNG. Also check
that plot_compare exits 2 on mismatched shapes, and that the front computed by
plot_straka_convergence on a synthetic field with a known -1 K crossing comes out within
half a grid cell.

## 6. Run on the real data
Produce these figures and list them in the report:
- out/plots/straka/ (from out/a0_straka, all snapshots)
- out/plots/warm_bubble/, out/plots/igw/ (all snapshots)
- out/plots/diag_all.png from out/a0_straka out/a0_warm_bubble out/a0_igw out/a0_rest
- out/plots/compare_straka_fp32_fp64.png from out/a0_straka_dp vs out/a0_straka
- out/plots/straka_convergence.png from out/a0_straka_400 out/a0_straka_200
  out/a0_straka_dp out/a0_straka_50

## 7. Report: docs/team/AGENT3_REPORT_T32.md (max 40 lines)
Files created; exact commands with their real output (test results, the convergence
table, the compare numbers); the list of PNGs written; anything that looked physically
odd in the plots (describe it, do not try to fix the model). Then stop.
