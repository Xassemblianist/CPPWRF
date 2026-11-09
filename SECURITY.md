# Security policy

## Supported versions

Security fixes go into the latest release and the `main` branch.

| Version | Supported |
|---|---|
| 1.0.x | yes |
| older | no |

## Reporting a vulnerability

Please do not open a public issue. Report it privately, either:

- through GitHub's private vulnerability reporting ("Report a vulnerability" on the repository's Security tab), or
- by e-mail to omerkaan20102003@gmail.com.

Include the version (`cppwrf --version`), what an attacker can do, and the steps or files that show it. You will get
an answer within a week. When the problem is confirmed, a fix is released and the advisory credits you unless you
prefer otherwise.

## Scope

CPPWRF is a numerical model that runs as the user who starts it and reads files the user gives it. Relevant
problems include:

- memory errors (buffer overflows, out-of-bounds reads) triggered by a namelist, configuration file or NetCDF input
  file, including the met_em, wrfinput, wrfbdy and wrfrst files;
- the Python package or the tools (`cppwrf-wrf`, `cppwrf-realexe`, `cppwrf-check`, `cppwrf-compare`, `cppwrf-bench`,
  `cppwrf-fit`) executing code or writing outside the run directory because of their input;
- the release tarballs, containers or recipes in packaging/.

A wrong forecast or a difference from WRF is a bug, not a security problem: use the bug report template.
