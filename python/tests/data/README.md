# Saved observation samples (cppwrf-verify-obs parser tests)

Small excerpts of real public reports, unchanged except for the selection of lines:

- `isd_17130099999_2025_excerpt.csv`: NOAA NCEI Integrated Surface Database, Global Hourly CSV of station
  17130099999 (Ankara Central, Turkey), file `2025/17130099999.csv` of NOAA's open-data bucket
  (noaa-global-hourly-pds, where www.ncei.noaa.gov/data/global-hourly/access/ redirects). The header, the rows of
  2025-01-04 10 UTC to 2025-01-05 05 UTC, and the five rows of 2025 whose temperature has quality code 2 (suspect).
  Downloaded 2026-11-09. Public domain (US Government work).
- `metar_iem_excerpt.csv`: METAR reports from the Iowa Environmental Mesonet ASOS / METAR download service
  (mesonet.agron.iastate.edu/cgi-bin/request/asos.py, data=metar, latlon=yes, elev=yes): LTAC (Ankara Esenboga)
  2026-11-08 18:20 to 23:50 UTC, and KORD / KDSM reports of 2025-04-02 with US remarks (SLP, hourly and 3-hour
  precipitation, precise temperatures). Downloaded 2026-11-09.
