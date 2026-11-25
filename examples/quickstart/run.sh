#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# A real-data forecast: GFS -> WPS -> cppwrf-realexe -> cppwrf-wrf -> wrfout and a map.
# WPS_DIR: WPS 4.6 binaries and tables. WRF_DIR: WRF v4.8.0 tree with run/ tables.
# GEOG_DIR: unpacked WPS_GEOG_LOW_RES. CPPWRF_BUILD: optional CPPWRF build tree.
# MODE: CPU/GPU FP32/FP64 mode. GFS_DATE and GFS_CYCLE: NOAA GFS date and UTC cycle.
# FORECAST_HOURS: whole hours from 1 to 24 (default 3). GFS_CACHE: optional shared download cache.
# WORK: gets wps/ and run/. If REQUIRE_MAP=1, a plotting error fails the run.
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
EX=$REPO/examples/quickstart
WORK=${1:?usage: run.sh WORK}
: "${WPS_DIR:?set WPS_DIR}" "${WRF_DIR:?set WRF_DIR}" "${GEOG_DIR:?set GEOG_DIR}"
WPS_DIR=$(realpath "$WPS_DIR")
WRF_DIR=$(realpath "$WRF_DIR")
GEOG_DIR=$(realpath "$GEOG_DIR")
GFS_DATE=${GFS_DATE:-20261012} GFS_CYCLE=${GFS_CYCLE:-00} FORECAST_HOURS=${FORECAST_HOURS:-3}
if [[ ! $GFS_DATE =~ ^[0-9]{8}$ || ! $GFS_CYCLE =~ ^(00|06|12|18)$ ]]; then
  printf 'Expected GFS_DATE=YYYYMMDD and GFS_CYCLE=00, 06, 12 or 18 UTC\n' >&2
  exit 2
fi
if [[ ! $FORECAST_HOURS =~ ^([1-9]|1[0-9]|2[0-4])$ ]]; then
  printf 'Expected FORECAST_HOURS to be a whole number from 1 to 24\n' >&2
  exit 2
fi
start=$(date -u -d "${GFS_DATE:0:4}-${GFS_DATE:4:2}-${GFS_DATE:6:2} $GFS_CYCLE:00" +%s)
# Supply the next 3-hour GFS boundary when the requested end time falls between inputs.
forcing_hours=$(( (FORECAST_HOURS + 2) / 3 * 3 ))
wrf_date() { date -u -d "@$((start + $1 * 3600))" '+%Y-%m-%d_%H:%M:%S'; }
start_time=$(wrf_date 0)
end_time=$(wrf_date "$FORECAST_HOURS")
forcing_end=$(wrf_date "$forcing_hours")

mkdir -p "$WORK/wps" "$WORK/run"
WORK=$(realpath "$WORK")
gfs_cache=$WORK/wps
if [[ -n ${GFS_CACHE:-} || $GFS_DATE$GFS_CYCLE != 2026101200 ]]; then
  gfs_cache=${GFS_CACHE:-$WORK/wps/gfs}/$GFS_DATE$GFS_CYCLE
fi
mkdir -p "$gfs_cache"
gfs_cache=$(realpath "$gfs_cache")
if [[ -n ${CPPWRF_BUILD:-} ]]; then
  B=$(cd "$CPPWRF_BUILD" && pwd)
  export CPPWRF_EXE=$B/cppwrf CPPWRF_REAL_EXE=$B/src/init_real/cppwrf_real
fi
tool() { command -v "$1" || echo "$REPO/tools/$1"; }

# 1. Geogrid, ungrib and metgrid on the two quickstart domains.
cd "$WORK/wps"
# Geogrid uses fixed-length buffers for the data paths.
ln -sfn "$GEOG_DIR" geog-data
sed -e "s|GEOG_DIR|geog-data|" \
    -e "s|^ start_date .*| start_date = '$start_time', '$start_time',|" \
    -e "s|^ end_date .*| end_date = '$forcing_end', '$start_time',|" \
    "$EX/namelist.wps" > namelist.wps
ln -sfn "$WPS_DIR/geogrid" geogrid
ln -sfn "$WPS_DIR/metgrid" metgrid
ln -sfn "$WPS_DIR/ungrib/Variable_Tables/Vtable.GFS" Vtable

download_gfs() (
  tmp=$(mktemp "$1.download.XXXXXXXX")
  trap 'rm -f -- "$tmp"' EXIT
  url="https://noaa-gfs-bdp-pds.s3.amazonaws.com/gfs.$GFS_DATE/$GFS_CYCLE/atmos/gfs.t${GFS_CYCLE}z.pgrb2.0p25.f$2"
  printf 'Downloading NOAA GFS %s %sZ f%s\n' "$GFS_DATE" "$GFS_CYCLE" "$2"
  curl -fsSL --retry 5 -o "$tmp" "$url"
  mv -fT "$tmp" "$1"
)
letters=ABCDEFGHIJKLMNOPQRSTUVWXYZ
find . -maxdepth 1 -type l -name 'GRIBFILE.AA?' -delete
for (( hour=0; hour<=forcing_hours; hour+=3 )); do
  printf -v lead '%03d' "$hour"
  file=$gfs_cache/gfs.f$lead
  [[ -s $file ]] || download_gfs "$file" "$lead"
  ln -sfn "$file" "GRIBFILE.AA${letters:$((hour / 3)):1}"
done
for p in geogrid ungrib metgrid; do
  log=$WORK/wps/$p.stdout.log
  if ! "$WPS_DIR/$p.exe" > "$log" 2>&1 || ! grep -q "Successful completion of $p" "$log"; then
    printf '%s.exe failed. See %s\n' "$p" "$log" >&2
    exit 1
  fi
done

# 2. Generate wrfinput_d0N and wrfbdy_d01 through the last GFS forcing time.
cd "$WORK/run"
python3 - "$EX/namelist.input" "$WORK/wps/met_em.d01.$start_time.nc" \
  "$start_time" "$forcing_end" "$end_time" "$forcing_hours" "$FORECAST_HOURS" <<'PY'
import pathlib
import re
import sys
from datetime import datetime

import netCDF4

template, first_met, start, forcing_end, end, forcing_hours, hours = sys.argv[1:]
text = pathlib.Path(template).read_text()
with netCDF4.Dataset(first_met) as ds:
    nlev = len(ds.dimensions["num_metgrid_levels"])
    ncat = ds.getncattr("NUM_LAND_CAT")


def write_namelist(path, finish, length):
    fields = {"run_hours": length, "num_metgrid_levels": nlev, "num_land_cat": ncat}
    for prefix, stamp in (("start", start), ("end", finish)):
        date = datetime.strptime(stamp, "%Y-%m-%d_%H:%M:%S")
        for key in ("year", "month", "day", "hour"):
            value = getattr(date, key)
            fields[f"{prefix}_{key}"] = f"{value}, {value}"
    pattern = r"^(\s*(?:" + "|".join(fields) + r")\s*=)[^\n]+"
    namelist = re.sub(pattern, lambda m: f"{m[1]} {fields[m[1].split('=')[0].strip()]},", text, flags=re.M)
    pathlib.Path(path).write_text(namelist)


write_namelist("namelist.input", forcing_end, forcing_hours)
write_namelist("namelist.forecast", end, hours)
PY
ln -sfn "$WORK"/wps/met_em.d0*.nc .
precision=double
if [[ ${MODE:-} == *-fp32 ]]; then precision=float; fi
"$(tool cppwrf-realexe)" --precision "$precision" || { tail -5 rsl.error.0000; exit 1; }
mv -f namelist.forecast namelist.input

# 3. Link the WRF tables and run the forecast through the requested hour.
for f in "$WRF_DIR"/run/*; do
  case ${f##*/} in namelist.input|*.exe|README*) ;; *) ln -sfn "$f" . ;; esac
done
"$(tool cppwrf-wrf)" ${MODE:+--mode "$MODE"} || { tail -5 rsl.error.0000; exit 1; }
grep -q "SUCCESS COMPLETE WRF" rsl.out.0000
ls wrfout_d0*

# 4. Plot only the requested final frame, even if this work directory has older output.
if ! python3 "$EX/plot.py" "wrfout_d01_$end_time" "$WORK/run/slp_t2_d01.png"; then
  if [[ ${REQUIRE_MAP:-} == 1 ]]; then exit 1; fi
  printf 'No map. Check the plotting error above.\n' >&2
fi
