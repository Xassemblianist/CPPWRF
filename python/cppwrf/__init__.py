# SPDX-License-Identifier: GPL-3.0-or-later
"""CPPWRF from Python: WRF namelists, runs of the model in WRF run directories and their wrfout files.

    import cppwrf
    cppwrf.namelist.check("namelist.input")             # what CPPWRF ports, per option
    run = cppwrf.Run("case_dir", mode="gpu-fp32").run()  # the drop-in wrf.exe, waited for
    ds = cppwrf.open_wrfout("case_dir")                  # xarray Dataset (or numpy without xarray)
    slp = cppwrf.diagnostics.slp(ds)

docs/PYTHON.md is the API reference."""
from ._version import __version__  # noqa: F401
from . import namelist  # noqa: F401
from . import diagnostics  # noqa: F401
from .run import CppwrfError, NamelistError, Progress, Run, RunFailed  # noqa: F401
from .wrfout import WrfoutData, open_wrfout  # noqa: F401

__all__ = ["__version__", "namelist", "diagnostics", "Run", "Progress", "CppwrfError", "NamelistError", "RunFailed",
           "open_wrfout", "WrfoutData"]
