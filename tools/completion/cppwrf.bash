# SPDX-License-Identifier: GPL-3.0-or-later
# Bash completion of the CPPWRF tools. Installed to share/bash-completion/completions; from a source tree:
#   source tools/completion/cppwrf.bash
_cppwrf_modes="gpu-fp32 gpu-fp64 cpu-fp64 cpu-fp32"

_cppwrf_tool() {
  local cur prev opts
  cur=${COMP_WORDS[COMP_CWORD]}
  prev=${COMP_WORDS[COMP_CWORD-1]}
  case ${COMP_WORDS[0]##*/} in
    cppwrf-wrf) opts="--mode --threads --exe --dir --set --native-restart --echo --dry-run --version --help" ;;
    cppwrf-realexe) opts="--exe --dir --met-dir --precision --echo --version --help" ;;
    cppwrf-check) opts="--verbose --json --version --help" ;;
    cppwrf-compare) opts="--variables --exact --json --no-metadata --all --version --help" ;;
    cppwrf-bench) opts="--modes --size --steps --threads --work --keep --json --save --no-save --version --help" ;;
    cppwrf-fit) opts="--grid --steps --dt --physics --radt --modes --bench --json --version --help" ;;
    cppwrf-post) opts="--config --output-dir --format --jobs --list-fields --print-config --quiet --help" ;;
    cppwrf-verify-obs) opts="--obs-source --obs-file --stations --output-dir --compare --labels --cache-dir --offline
      --pause --boundary --interpolation --no-height-correction --lapse-rate --max-elev-diff --window --thresholds
      --precip-hours --min-wind-dir --no-plots --quiet --help" ;;
    *) opts="--version --help" ;;
  esac
  case $prev in
    --mode) COMPREPLY=($(compgen -W "$_cppwrf_modes" -- "$cur")); return ;;
    --modes) COMPREPLY=($(compgen -W "$_cppwrf_modes" -- "${cur##*,}")); return ;;
    --precision) COMPREPLY=($(compgen -W "float64 float32" -- "$cur")); return ;;
    --obs-source) COMPREPLY=($(compgen -W "isd iem file" -- "$cur")); return ;;
    --interpolation) COMPREPLY=($(compgen -W "bilinear nearest" -- "$cur")); return ;;
    --obs-file) COMPREPLY=($(compgen -f -- "$cur")); return ;;
    --cache-dir) COMPREPLY=($(compgen -d -- "$cur")); return ;;
    --stations|--labels|--pause|--boundary|--lapse-rate|--max-elev-diff|--window|--thresholds|--precip-hours|\
    --min-wind-dir) return ;;
    --format) COMPREPLY=($(compgen -W "netcdf grib2 both" -- "$cur")); return ;;
    --physics) COMPREPLY=($(compgen -W "none full" -- "$cur")); return ;;
    --dir|--met-dir|--work|--output-dir|-o) COMPREPLY=($(compgen -d -- "$cur")); return ;;
    --config|-c) COMPREPLY=($(compgen -f -- "$cur")); return ;;
    --jobs|-j) return ;;
    --exe|--bench|--save) COMPREPLY=($(compgen -f -- "$cur")); return ;;
    --threads|--steps|--size|--grid|--dt|--radt|--set|--variables|-v) return ;;
  esac
  if [[ $cur == -* ]]; then
    COMPREPLY=($(compgen -W "$opts" -- "$cur"))
  else
    COMPREPLY=($(compgen -f -- "$cur"))
  fi
}
complete -o filenames -F _cppwrf_tool cppwrf-wrf cppwrf-realexe cppwrf-check cppwrf-compare cppwrf-bench cppwrf-fit cppwrf-post \
  cppwrf-verify-obs
