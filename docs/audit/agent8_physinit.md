# Audit A5: Physics Initialization (Agent 8, Commit 6d64875) vs WRF 4.6.1-r8 $t = 0$ State

**Auditor**: Agent 7 (Quality Gate / Independent Auditor)  
**Date**: 2026-10-13  
**Target Code**: `src/phys/driver/physics_init.*`, `src/phys/driver/landuse_tables.*` (Agent 8, commit `6d64875`)  
**Reference Model**: WRF v4.6.1-r8 (`/home/ostriquetrum/wrf_real/WRF-4.6.1-r8`), FP64 execution  
**Verdict**: **VERIFIED WITH DISCREPANCIES / ACTION ITEMS FOR AGENT 8**  
- **17 of 19 fields match WRF $t = 0$ state to exact bitwise zero (`0.0000e+00`)**, including `ALBEDO`, `ALBBCK`, `EMISS`, `EMBCK`, `ZNT`, `Z0`, `MAVAIL`, `THC`, `XLAND`, `LU_INDEX`, `IVGTYP`, `ISLTYP`, `VEGFRA`, `SHDMAX`, `SHDMIN`, `SMOIS`, and `TSLB`.
- **`SNOALB` has large discrepancy (max diff 0.62, rel L2 0.7358)** because Agent 8 omitted the WRF default `RDMAXALB = .true.` branch and unconditionally overwrites spatial snow albedo from `wrfinput` with coarse table values.
- **`SH2O` differs by $3.55\times 10^{-7}$ in the 2 sub-freezing soil points** because Agent 8 passed `SMOIS` directly to `frh2o` instead of the explicit first guess $FK = \min(FK, SMOIS)$ from Koren et al. (1999) Eqn 17.

---

## 1. Exact Commands to Reproduce Verification

All builds and model executions were performed under `tools/heavy.sh`:

### 1.1 WRF-4.6.1-r8 $t = 0$ Reference Run Setup & Execution
```bash
# 1. Create isolated run directory
mkdir -p /home/ostriquetrum/wrf_real/run_A5_r8
cd /home/ostriquetrum/wrf_real/run_A5_r8

# 2. Symlink tables and inputs from WRF-4.6.1-r8 and WPS
for f in /home/ostriquetrum/wrf_real/WRF-4.6.1-r8/run/*; do
    bn=$(basename "$f")
    if [[ ! "$bn" =~ ^(namelist.*|wrf.*|real.*|ndown.*|tc.*)$ ]]; then
        ln -snf "$f" .
    fi
done
ln -snf /home/ostriquetrum/wrf_real/WRF-4.6.1-r8/main/real.exe .
ln -snf /home/ostriquetrum/wrf_real/WRF-4.6.1-r8/main/wrf.exe .
ln -snf /home/ostriquetrum/wrf_real/wps_run/met_em.d01.*.nc .

# 3. Create myoutfields.txt to output every surface and soil variable to history stream 0
cat << 'EOF' > myoutfields.txt
+:h:0:ALBEDO,ALBBCK,EMISS,EMBCK,ZNT,Z0,MAVAIL,THC,XLAND,SNOALB,SH2O,SMOIS,TSLB,LU_INDEX,IVGTYP,ISLTYP,VEGFRA,SHDMAX,SHDMIN
EOF

# 4. Generate wrfinput_d01 and wrfbdy_d01 with Noah LSM (sf_surface_physics = 2, num_soil_layers = 4)
/home/ostriquetrum/CPPWRF/tools/heavy.sh ./real.exe

# 5. Run wrf.exe for 1 step (output frame 0 at t=0 and frame 1 at t=60s)
/home/ostriquetrum/CPPWRF/tools/heavy.sh ./wrf.exe
```

### 1.2 Compile and Run Independent C++ Verification Driver
```bash
cd /home/ostriquetrum/CPPWRF/tests/phys/driver

# Build C++ test driving Agent 8's init code in FP64
/home/ostriquetrum/CPPWRF/tools/heavy.sh g++ -std=c++20 -DCPPWRF_DOUBLE -DCPPWRF_HAVE_NETCDF -O3 -march=native -ffp-contract=off -fopenmp \
  -I /home/ostriquetrum/CPPWRF/src \
  test_audit_a5.cpp \
  /home/ostriquetrum/CPPWRF/src/phys/driver/landuse_tables.cpp \
  /home/ostriquetrum/CPPWRF/src/phys/driver/physics_init.cpp \
  /home/ostriquetrum/CPPWRF/src/phys/lsm_noah/noah.cpp \
  /home/ostriquetrum/CPPWRF/src/phys/lsm_noah/noah_tables.cpp \
  /home/ostriquetrum/CPPWRF/src/phys/sf_sfclayrev/sfclayrev.cpp \
  /home/ostriquetrum/CPPWRF/src/phys/cu_kf/kf_tables.cpp \
  /home/ostriquetrum/CPPWRF/build-cpu-dp/libcppwrf_core.a \
  -lnetcdf \
  -o build/test_audit_a5

# Execute comparison against WRF wrfout frame 0
/home/ostriquetrum/CPPWRF/tools/heavy.sh ./build/test_audit_a5 \
  /home/ostriquetrum/wrf_real/run_A5_r8 \
  /home/ostriquetrum/wrf_real/run_A5_r8/wrfinput_d01 \
  /home/ostriquetrum/wrf_real/run_A5_r8/wrfout_d01_2026-10-12_00:00:00
```

---

## 2. End-to-End Numerical Verification Results

Domain dimensions: $119 \times 89$ horizontal mass points, 4 soil layers, Julian day 285, central latitude $39.0^\circ\text{N}$, dataset `MODIFIED_IGBP_MODIS_NOAH`.

Comparison of Agent 8 initialized state against WRF 4.6.1-r8 history stream 0 at $t = 0$:

| Variable | Stagger / Dim | Max \|Diff\| | Rel $L_2$ Error | Max \|WRF\| | Max \|CPP\| | Differing Points / Total | Status |
|---|---|---|---|---|---|---|---|
| `ALBEDO` | 2D M | **0.0000e+00** | **0.0000e+00** | 2.5000e-01 | 2.5000e-01 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `ALBBCK` | 2D M | **0.0000e+00** | **0.0000e+00** | 2.5000e-01 | 2.5000e-01 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `EMISS` | 2D M | **0.0000e+00** | **0.0000e+00** | 9.8500e-01 | 9.8500e-01 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `EMBCK` | 2D M | **0.0000e+00** | **0.0000e+00** | 9.8500e-01 | 9.8500e-01 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `ZNT` | 2D M | **0.0000e+00** | **0.0000e+00** | 8.0000e-01 | 8.0000e-01 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `Z0` | 2D M | **0.0000e+00** | **0.0000e+00** | 8.0000e-01 | 8.0000e-01 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `MAVAIL` | 2D M | **0.0000e+00** | **0.0000e+00** | 1.0000e+00 | 1.0000e+00 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `THC` | 2D M | **0.0000e+00** | **0.0000e+00** | 6.0000e-02 | 6.0000e-02 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `XLAND` | 2D M | **0.0000e+00** | **0.0000e+00** | 2.0000e+00 | 2.0000e+00 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `LU_INDEX` | 2D M | **0.0000e+00** | **0.0000e+00** | 1.7000e+01 | 1.7000e+01 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `IVGTYP` | 2D M | **0.0000e+00** | **0.0000e+00** | 1.7000e+01 | 1.7000e+01 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `ISLTYP` | 2D M | **0.0000e+00** | **0.0000e+00** | 1.4000e+01 | 1.4000e+01 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `VEGFRA` | 2D M | **0.0000e+00** | **0.0000e+00** | 8.3554e+01 | 8.3554e+01 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `SHDMAX` | 2D M | **0.0000e+00** | **0.0000e+00** | 9.1000e+01 | 9.1000e+01 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `SHDMIN` | 2D M | **0.0000e+00** | **0.0000e+00** | 6.2162e+01 | 6.2162e+01 | 0 / 10,591 | **PASS (Bitwise Exact)** |
| `SMOIS` | 3D Soil | **0.0000e+00** | **0.0000e+00** | 1.0000e+00 | 1.0000e+00 | 0 / 42,364 | **PASS (Bitwise Exact)** |
| `TSLB` | 3D Soil | **0.0000e+00** | **0.0000e+00** | 3.0856e+02 | 3.0856e+02 | 0 / 42,364 | **PASS (Bitwise Exact)** |
| `SNOALB` | 2D M | 6.2000e-01 | 7.3585e-01 | 8.3569e-01 | 7.5000e-01 | 10,576 / 10,591 | **DIFF (Missing `rdmaxalb`)** |
| `SH2O` | 3D Soil | 3.5452e-07 | 2.7615e-09 | 1.0000e+00 | 1.0000e+00 | 2 / 42,364 | **DIFF (Missing `FK` guess in `frh2o`)** |

---

## 3. Discrepancy Analysis with Exact WRF Source Citations

### 3.1 Discrepancy 1: `SNOALB` Overwriting Spatial Input with Category Tables
- **Observation**: 10,576 land points differ, with max difference 0.6200 and relative $L_2$ error 0.7358.
- **WRF Source**: [`phys/module_sf_noahdrv.F:1902-1904`](file:///home/ostriquetrum/wrf_real/WRF-4.6.1-r8/phys/module_sf_noahdrv.F#L1902-L1904):
  ```fortran
  IF(.not.RDMAXALB) THEN
     SNOALB(i,j)=MAXALB(IVGTYP(i,j))*0.01
  ENDIF
  ```
- **WRF Registry**: [`Registry/Registry.EM_COMMON:2564`](file:///home/ostriquetrum/wrf_real/WRF-4.6.1-r8/Registry/Registry.EM_COMMON#L2564):
  ```
  rconfig   logical rdmaxalb  namelist,physics  1  .true.  h  "rdmaxalb"  "false set it to table values"
  ```
- **Root Cause in CPPWRF**:
  In [`src/phys/driver/physics_init.cpp:76-81`](file:///home/ostriquetrum/CPPWRF/src/phys/driver/physics_init.cpp#L76-L81):
  ```cpp
  int iv = static_cast<int>(std::round(ivgtyp(i, j)));
  if (iv >= 1 && iv <= 50) {
    snoalb(i, j) = tables.maxalb[iv - 1] * real(0.01);
  } else {
    snoalb(i, j) = real(0.75);
  }
  ```
  Agent 8 did not port the `rdmaxalb` namelist config option. In WRF, `rdmaxalb` is `.true.` by default, instructing Noah to preserve the continuous, geographically-interpolated maximum snow albedo field from `wrfinput_d01` (derived by WPS `geogrid`/`metgrid` from MODIS/USGS spatial albedo climatology). Agent 8's implementation unconditionally overwrites `snoalb` with the coarse discrete lookup table values from `VEGPARM.TBL`.
- **Impact**: In operational real-data runs, this destroys spatial variations in maximum snow albedo, setting entire vegetation biomes to uniform coarse constants.

### 3.2 Discrepancy 2: `SH2O` Supercooled Water at Frozen Soil Points
- **Observation**: Exactly 2 points out of 42,364 differ by $3.545\times 10^{-7}$ (relative $L_2$ error $2.76\times 10^{-9}$):
  - Point $(i=101, j=76, k=0)$: WRF `0.3296420630`, CPP `0.3296424185`, $\Delta = 3.545\times 10^{-7}$.
  - Point $(i=98, j=77, k=0)$: WRF `0.2140449724`, CPP `0.2140453235`, $\Delta = 3.511\times 10^{-7}$.
  These two points are the only grid cells in the domain where soil temperature is sub-freezing ($T < 273.149\text{ K}$). All remaining 42,362 unfrozen points match to exact bitwise zero ($0.0000\text{e}+00$).
- **WRF Source**: [`phys/module_sf_noahdrv.F:1937-1951`](file:///home/ostriquetrum/wrf_real/WRF-4.6.1-r8/phys/module_sf_noahdrv.F#L1937-L1951):
  ```fortran
  BX = BB(ISLTYP(I,J))
  SMCMAX = MAXSMC(ISLTYP(I,J))
  PSISAT = SATPSI(ISLTYP(I,J))
  IF ( BX >  BLIM ) BX = BLIM
  FK=(( (HLICE/(GRAV*(-PSISAT))) *                              &
     ((TSLB(I,NS,J)-T0)/TSLB(I,NS,J)) )**(-1/BX) )*SMCMAX
  IF (FK < 0.02) FK = 0.02
  SH2O(I,NS,J) = MIN( FK, SMOIS(I,NS,J) )
  CALL FRH2O (FREE,TSLB(I,NS,J),SMOIS(I,NS,J),SH2O(I,NS,J),    &
     SMCMAX,BX,PSISAT)
  SH2O(I,NS,J) = FREE
  ```
- **Root Cause in CPPWRF**:
  In [`src/phys/driver/physics_init.cpp:91-94`](file:///home/ostriquetrum/CPPWRF/src/phys/driver/physics_init.cpp#L91-L94):
  ```cpp
  if (t < real(273.149) && bx > real(0) && smcmax > real(0) && psisat > real(0)) {
    real free_w = real(0);
    noah::NoahKernel::frh2o(free_w, t, sm, sm, smcmax, bx, psisat);
    sh2o(i, j, k) = free_w;
  }
  ```
  1. In WRF, argument 4 to `FRH2O` is the initial guess `SH2O = MIN(FK, SMOIS)`, computed via the explicit approximation formula from Koren et al. (1999) Eqn 17.
  2. In `FRH2O` ([`phys/module_sf_noahlsm.F:1517`](file:///home/ostriquetrum/wrf_real/WRF-4.6.1-r8/phys/module_sf_noahlsm.F#L1517)), initial ice content is set as `SWL = SMC - SH2O`. WRF starts Newton iteration from `SWL = SMOIS - MIN(FK, SMOIS) > 0`.
  3. Agent 8 passed `sm` (`SMOIS`) directly as argument 4, which forces `SWL = sm - sm = 0.0`.
  4. Because `FRH2O` stops when $|SWLK - SWL| \le 0.005$ (`ERROR = 0.005`, [`phys/module_sf_noahlsm.F:1551`](file:///home/ostriquetrum/wrf_real/WRF-4.6.1-r8/phys/module_sf_noahlsm.F#L1551)), starting from zero ice vs starting from `FK` causes the solver to terminate at a slightly different iteration step, producing the $3.5\times 10^{-7}$ residual.
  5. Furthermore, Agent 8 omitted clamping `bx` to `blim = 5.5` in `physics_init.cpp` before evaluating soil moisture equations (though inside `NoahKernel::frh2o`, `bx` is clamped).

---

## 4. Line-by-Line Code Review

Detailed comparison between CPPWRF files and WRF 4.6.1 source:

### 4.1 `src/phys/driver/landuse_tables.cpp` vs WRF 4.6.1 `phys/module_physics_init.F`
| CPPWRF (`landuse_tables.cpp`) | WRF 4.6.1 (`module_physics_init.F`) | Description | Branch Status & Real-Case Impact |
|---|---|---|---|
| Lines 54–79 | Lines 1836–1887 | Open `LANDUSE.TBL`, scan line-by-line matching dataset name `MMINLU` | **Ported**. Handles upper-case/trim matching. |
| Lines 83–93 | Lines 1878, 1888–1889 | Read header line: `LUCATS`, `LUSEAS` | **Ported**. Stores `n_cats`, `n_seasons`. |
| Lines 94–116 | Lines 1898–1916 | Loop over seasons and categories; parse `ALBD`, `SLMO`, `SFEM`, `SFZ0`, `THERIN`, `SCFX`, `SFHC` | **Ported**. |
| *Omitted* | Lines 1917–1919: `IF (NSN.EQ.1 .AND. FOUND_LU) ISN = 1` | Force season 1 if dataset has only 1 season | **Skipped**. If a single-season table was used, winter lookups would access uninitialized memory. *Not executed in our case* (MODIS has 2 seasons). |
| *Omitted* | Line 1909: `IF (LC.NE.LI) CALL wrf_error_fatal` | Verify category index continuity | **Skipped**. Harmless omission. |

### 4.2 `src/phys/driver/physics_init.cpp` (`init_surface_properties`) vs WRF 4.6.1 `phys/module_physics_init.F`
| CPPWRF (`physics_init.cpp`) | WRF 4.6.1 (`module_physics_init.F`) | Description | Branch Status & Real-Case Impact |
|---|---|---|---|
| Lines 149–150 | Lines 1831–1833: `ISN=1; IF(JULDAY.LT.105.OR.JULDAY.GT.288)ISN=2; IF(CEN_LAT.LT.0)ISN=3-ISN` | Summer/Winter season selection by Julian day and hemisphere | **Ported**. Summer=0, Winter=1. Exact match. |
| Lines 36–38 | Lines 1944, 1953–1955: `IS=nint(lu_index); IF(IS.EQ.0) IS=ISWATER` | Extract landuse index, fallback to water | **Ported**. Clamps out-of-range index to `is_water`. |
| Lines 39–48 | Lines 1956–1975: `ALBBCK=ALBD/100; ALBEDO=ALBBCK; THC=THERIN/100; Z0=SFZ0/100; ZNT=Z0; EMBCK=SFEM; EMISS=EMBCK; MAVAIL=SLMO; XLAND=1.0 or 2.0` | Assign surface baseline properties from table | **Ported**. Produces **exact bitwise match (0.0000e+00)**. |
| *Omitted* | Lines 1958–1964: `IF(SNOWC(I,J) .GT. 0.5) ALBEDO = ALBBCK*(1.+SCFX)` | Snow-cover albedo adjustment | **Skipped**. Agent 8 does not take `SNOWC` as input. *Not executed in this case* (`SNOWC = 0`). |
| *Omitted* | Lines 1956, 1959: `IF(usemonalb) ...` | Climatological monthly albedo flag | **Skipped**. *Not executed in this case* (`usemonalb = .false.`). |
| Lines 50–61 | Lines 1978–1995: Sea ice property override when `XICE >= threshold` | Assign sea ice albedo, emissivity, thermal inertia, roughness | **Partially Ported with Inconsistency**: In WRF, if `fractional_seaice == 1` (`threshold = 0.02`), `ALBEDO` and `EMISS` are blended linearly with open water ($0.08$ albedo, $0.98$ emissivity). Agent 8 used threshold 0.02 but binary 100% ice properties. *Not executed in this case* (`SEAICE = 0`). |

### 4.3 `src/phys/driver/physics_init.cpp` (`init_noah_soil`) vs WRF 4.6.1 `phys/module_sf_noahdrv.F` (`lsminit`)
| CPPWRF (`physics_init.cpp`) | WRF 4.6.1 (`module_sf_noahdrv.F`) | Description | Branch Status & Real-Case Impact |
|---|---|---|---|
| Lines 76–81 | Lines 1902–1904: `IF(.not.RDMAXALB) SNOALB = MAXALB*0.01` | Snow albedo initialization | **INCORRECT (Missing `rdmaxalb`)**. WRF keeps `wrfinput` `SNOALB` when `RDMAXALB = .true.` (default). Agent 8 overwrote all 10,576 land points. **Active and caused 0.62 error**. |
| Lines 83–86 | Lines 1925–1928: `BX = BB(ISLTYP); SMCMAX = MAXSMC(ISLTYP); PSISAT = SATPSI(ISLTYP)` | Soil hydraulic property lookup | **Ported**. Exact match. |
| Lines 88–98 | Lines 1929–1965: Loop over soil layers; test `TSLB < 273.149` | Call `frh2o` for supercooled liquid water, otherwise `SH2O = SMOIS` | **Partially Ported with Inaccuracy**: Agent 8 omitted explicit first guess `FK = min(FK, SMOIS)` before `frh2o`. **Active and caused $3.5\times 10^{-7}$ discrepancy**. |
| *Omitted* | Lines 1940: `IF ( BX > BLIM ) BX = BLIM` (`BLIM = 5.5`) | Clamp soil exponent parameter `BX` | **Skipped in caller** (though clamped inside `frh2o`). |
| *Omitted* | Lines 1968–1978: `IF(.NOT.FNDSNOWH) SNOWH = SNOW * 0.005` | Calculate snow depth if not present | **Skipped**. *Not active in this case* (`SNOWH` in wrfinput). |
| *Omitted* | Lines 1980–1989: `CANWAT(I,J) = 0.0` | Zero out canopy water content | **Skipped**. *Not active in this case*. |

### 4.4 `src/phys/lsm_noah/noah_tables.cpp` vs WRF 4.6.1 `phys/module_sf_noahdrv.F` (`SOIL_VEG_GEN_PARM`)
- `read_vegparm` ([`src/phys/lsm_noah/noah_tables.cpp:123-180`](file:///home/ostriquetrum/CPPWRF/src/phys/lsm_noah/noah_tables.cpp#L123-L180)) matches WRF lines 2043–2135.
- `read_soilparm` ([`src/phys/lsm_noah/noah_tables.cpp:82-121`](file:///home/ostriquetrum/CPPWRF/src/phys/lsm_noah/noah_tables.cpp#L82-L121)) matches WRF lines 2140–2225.
- `read_genparm` ([`src/phys/lsm_noah/noah_tables.cpp:39-80`](file:///home/ostriquetrum/CPPWRF/src/phys/lsm_noah/noah_tables.cpp#L39-L80)) matches WRF lines 2230–2335.
All 3 table readers are verified exact (0 differences vs Fortran reference).

---

## 5. Required Fixes for Agent 8

To bring CPPWRF physics initialization into full bitwise compliance with WRF:

1. **Fix `SNOALB` preservation (`physics_init.cpp:76-81`)**:
   Add a boolean parameter `rdmaxalb = true` (or check if `snoalb` in `wrfinput` is already non-zero):
   ```cpp
   if (!rdmaxalb) {
     int iv = static_cast<int>(std::round(ivgtyp(i, j)));
     if (iv >= 1 && iv <= 50) {
       snoalb(i, j) = tables.maxalb[iv - 1] * real(0.01);
     } else {
       snoalb(i, j) = real(0.75);
     }
   }
   ```
2. **Fix `SH2O` initial guess (`physics_init.cpp:88-98`)**:
   Before calling `frh2o`, compute the explicit Koren et al. (1999) estimate:
   ```cpp
   constexpr real hlice = real(3.335e5);
   constexpr real grav = real(9.81);
   constexpr real t0 = real(273.15);
   constexpr real blim = real(5.5);
   real b_clamped = (bx > blim) ? blim : bx;
   real fk = std::pow((hlice / (grav * psisat)) * ((t0 - t) / t), -real(1.0) / b_clamped) * smcmax;
   if (fk < real(0.02)) fk = real(0.02);
   real sh2o_guess = std::min(fk, sm);
   real free_w = real(0);
   noah::NoahKernel::frh2o(free_w, t, sm, sh2o_guess, smcmax, b_clamped, psisat);
   sh2o(i, j, k) = free_w;
   ```
3. **Add Fractional Sea Ice Blending (`physics_init.cpp:50-61`)**:
   When fractional sea ice is enabled (`fractional_seaice == 1`), blend `albedo` and `emiss`:
   ```cpp
   albedo(i, j) = (xice(i, j) * albbck(i, j)) + ((real(1.0) - xice(i, j)) * real(0.08));
   emiss(i, j)  = (xice(i, j) * embck(i, j))  + ((real(1.0) - xice(i, j)) * real(0.98));
   ```
4. **Add Snow-Cover Albedo Modification (`physics_init.cpp:41`)**:
   Take `snowc` as an argument and apply `albedo(i, j) = albbck(i, j) * (real(1.0) + cat.scfx)` when `snowc > 0.5`.

---

## 6. Audit Verdict

- **Bitwise Exact Core Surface Fields**: **PASS**. 17 out of 19 fields match WRF 4.6.1-r8 at $t = 0$ with `0.0000e+00` error.
- **Table Parsers (`LANDUSE.TBL`, `VEGPARM.TBL`, `SOILPARM.TBL`, `GENPARM.TBL`)**: **PASS**.
- **`SNOALB` & `SH2O` Init**: **IDENTIFIED BUGS WITH EXACT SOURCE TRACES**. Detailed above with line citations on both sides for Agent 8 to patch.
