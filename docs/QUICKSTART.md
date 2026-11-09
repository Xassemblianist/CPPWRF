# CPPWRF quick start

This page goes from the GFS analysis to a map of a 3-hour forecast in four steps: WPS, `cppwrf-realexe` (real.exe),
`cppwrf-wrf` (wrf.exe) and a plot. If you already have a WRF run directory, only step 3 is new. Run `cppwrf-wrf`
where you would run `wrf.exe`. The Turkish version is [tr/HIZLI_BASLANGIC.md](tr/HIZLI_BASLANGIC.md).

## What you need

- **CPPWRF, built and installed** ([INSTALL.md](INSTALL.md)), with `bin/` on `PATH`. A build tree works too: the
  tools are in the source tree's `tools/`.
- **A built WPS 4.6:** `geogrid.exe`, `ungrib.exe`, `metgrid.exe` and their tables.
- **WPS's low-resolution static data.** Download `geog_low_res_mandatory.tar.gz` from the WPS download page and
  unpack it.
- **WRF v4.8.0's source tree**, for the physics tables in its `run/` directory. WRF does not need to be built.
- **Python 3** with `netCDF4` and `numpy`, and `matplotlib` for the map.
- **Internet access** for two GFS files of about 500 MB each.

## The whole way in one command

```bash
export WPS_DIR=/path/to/WPS WRF_DIR=/path/to/WRF-4.8.0 GEOG_DIR=/path/to/WPS_GEOG_LOW_RES
MODE=gpu-fp32 examples/quickstart/run.sh ~/qs        # or MODE=cpu-fp64, gpu-fp64
```

The case is WRF's own default:

- the domains of WPS 4.6's default `namelist.wps`: 150 x 130 points at 15 km over the south-eastern United States,
  with a two-way nest of 220 x 214 points at 5 km;
- WRF v4.8.0's `test/em_real/namelist.input`, with `physics_suite = 'CONUS'`;
- the date of the GFS run and 3 forecast hours.

The steps below are what `run.sh` does, so you can do them by hand on your own case.

## 1. WPS: from GFS to met_em files

```bash
mkdir -p ~/qs/wps && cd ~/qs/wps
sed "s|GEOG_DIR|$GEOG_DIR|" $CPPWRF/examples/quickstart/namelist.wps > namelist.wps
ln -sfn $WPS_DIR/geogrid geogrid; ln -sfn $WPS_DIR/metgrid metgrid
ln -sf $WPS_DIR/ungrib/Variable_Tables/Vtable.GFS Vtable
for h in 000 003; do
  curl -sfo gfs.f$h https://noaa-gfs-bdp-pds.s3.amazonaws.com/gfs.20261012/00/atmos/gfs.t00z.pgrb2.0p25.f$h
done
$WPS_DIR/link_grib.csh gfs.f
$WPS_DIR/geogrid.exe && $WPS_DIR/ungrib.exe && $WPS_DIR/metgrid.exe
```

This step is WPS as usual. CPPWRF does not replace WPS. For another date, change the dates in both namelists.

## 2. cppwrf-realexe: wrfinput and wrfbdy

```bash
mkdir -p ~/qs/run && cd ~/qs/run
cp $CPPWRF/examples/quickstart/namelist.input .   # set num_metgrid_levels / num_land_cat as in the met_em files
ln -sf ~/qs/wps/met_em.d0* .
cppwrf-realexe
```

`cppwrf-realexe` writes `wrfinput_d01`, `wrfinput_d02` and `wrfbdy_d01`, the files of WRF's real.exe. They are
identical to real.exe's in CPPWRF's tests (docs/REAL.md). The logs `rsl.out.0000` and `rsl.error.0000` end with
`real_em: SUCCESS COMPLETE REAL_EM INIT`. WRF's own `real.exe` works here too.

`num_metgrid_levels` and `num_land_cat` must be the met_em files' values, as for real.exe. `run.sh` reads them from the
first met_em file.

## 3. cppwrf-wrf: the forecast

```bash
for f in $WRF_DIR/run/*; do case ${f##*/} in namelist.input|*.exe|README*) ;; *) ln -sfn $f . ;; esac; done
cppwrf-check namelist.input          # optional: what CPPWRF ports of this namelist
cppwrf-wrf --mode gpu-fp32           # or cpu-fp64 --threads $(nproc), gpu-fp64
```

`cppwrf-wrf` reads `namelist.input` and writes WRF's files into the run directory:

- `wrfout_d01_2026-10-12_00:00:00` ... `_03:00:00` and the same for `d02`, one file per hour as
  `frames_per_outfile = 1` asks;
- `rsl.out.0000` and `rsl.error.0000`, with WRF's `Timing for main` lines.

The last line of the logs, and of the terminal, is `d01 2026-10-12_03:00:00 wrf: SUCCESS COMPLETE WRF`. Scripts
that check wrf.exe's runs work unchanged:

```bash
grep -q "SUCCESS COMPLETE WRF" rsl.out.0000 && echo done
```

On an RTX PRO 6000 Blackwell, this 3-hour run of both domains took 15 s of stepping in `gpu-fp32` and 42 s in
`gpu-fp64`. `cpu-fp64` took 222 to 273 s on 48 CPU threads. `cppwrf-bench` measures your own machine, and
`cppwrf-fit namelist.input` estimates the memory and time of a run before you start it (docs/TOOLS.md).

An option that CPPWRF does not port stops `cppwrf-wrf` at start-up. The message names the option, its value and the
values CPPWRF supports, and the exit status is nonzero. CPPWRF never ignores such an option silently.

## 4. A map

```bash
python3 $CPPWRF/examples/quickstart/plot.py ~/qs/run      # slp_t2_d01.png
```

The script uses the `cppwrf` Python package (docs/PYTHON.md):

```python
import cppwrf
from cppwrf import diagnostics as dg

ds = cppwrf.open_wrfout("~/qs/run")          # domain 1, every frame
slp = dg.slp(ds)                             # hPa, as wrf-python's slp
t2 = dg.t2_celsius(ds)
rain_1h = dg.precip_accum(ds, hours=1)       # hourly precipitation
```

## Next

- [MIGRATING_FROM_WRF.md](MIGRATING_FROM_WRF.md): what is identical, every ported option, and how to compare your WRF
  run with CPPWRF (`cppwrf-compare`).
- [FAQ.md](FAQ.md): bit identity, FP32, GPUs, domain sizes, bug reports.
- [USER_GUIDE.md](USER_GUIDE.md): everything else (configuration keys, nests, restarts, output streams).
