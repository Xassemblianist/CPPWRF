#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# One CPPWRF performance session on Kaggle (2 x Tesla T4): tools/kaggle/kaggle_perf.py --help.
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec python3 -B "$here/kaggle_perf.py" "$@"
