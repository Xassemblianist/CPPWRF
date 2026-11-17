! SPDX-License-Identifier: GPL-3.0-or-later
! The last RK3 stage of the moist species as WRF v4.8.0's solve_em runs it for moist_adv_opt = 1 .. 4 (the WRF build's
! own objects: -r8 for 1 / 2, -r8 -fdefault-double-8 for the WENO options 3 / 4): rk_update_scalar_pd and
! set_physical_bc3d of moist_old (not for 3), then per species rk_scalar_tend's advection (advect_scalar_pd /
! advect_scalar_mono / advect_scalar_weno / advect_scalar_wenopd; set_tend of RQVFTEN for qv) and rk_update_scalar.
!   driver_advection_f90 IN OUT   (IN: generate_inputs.py; OUT: per species q_2 and q_1 of the mass points, then
!   RQVFTEN, float64, x fastest, then y, then k)
PROGRAM driver_advection
  USE module_configure, ONLY: grid_config_rec_type
  USE module_em, ONLY: rk_update_scalar, rk_update_scalar_pd
  USE module_advect_em, ONLY: advect_scalar_pd, advect_scalar_mono, advect_scalar_weno, advect_scalar_wenopd
  USE module_big_step_utilities_em, ONLY: set_tend
  USE module_bc, ONLY: set_physical_bc3d
  IMPLICIT NONE
  INTEGER, PARAMETER :: H = 5
  INTEGER :: nx, ny, nz, bc, nspec, p_qv, adv_opt, n, iu, k
  REAL :: dx, dy, dt, rdx, rdy
  INTEGER :: ids, ide, jds, jde, kds, kde, ims, ime, jms, jme, kms, kme, its, ite, jts, jte, kts, kte
  REAL, ALLOCATABLE :: c1h(:), c2h(:), rdnw(:), fnm(:), fnp(:)
  REAL, ALLOCATABLE :: msftx(:,:), msfty(:,:), mub(:,:), mu_1(:,:), mu_2(:,:), muts(:,:)
  REAL, ALLOCATABLE :: ru(:,:,:), rv(:,:,:), ww(:,:,:), wwi(:,:,:), adv(:,:,:), ht(:,:,:), zt(:,:,:), rqvften(:,:,:)
  REAL, ALLOCATABLE :: q1(:,:,:,:), q2(:,:,:,:), qt(:,:,:,:)
  CHARACTER(LEN=512) :: inp, outp
  TYPE(grid_config_rec_type) :: cf

  CALL GET_COMMAND_ARGUMENT(1, inp)
  CALL GET_COMMAND_ARGUMENT(2, outp)
  OPEN(NEWUNIT=iu, FILE=TRIM(inp), ACCESS='STREAM', FORM='UNFORMATTED', STATUS='OLD')
  READ(iu) nx, ny, nz, bc, nspec, p_qv, adv_opt, dx, dy, dt
  rdx = 1. / dx
  rdy = 1. / dy
  ids = 1; ide = nx + 1; jds = 1; jde = ny + 1; kds = 1; kde = nz + 1
  ims = 1 - H; ime = nx + 1 + H; jms = 1 - H; jme = ny + 1 + H; kms = 1; kme = nz + 1
  its = ids; ite = ide; jts = jds; jte = jde; kts = kds; kte = kde
  ALLOCATE(c1h(kms:kme), c2h(kms:kme), rdnw(kms:kme), fnm(kms:kme), fnp(kms:kme))
  c1h = 0.; c2h = 0.; rdnw = 0.
  READ(iu) c1h(1:nz), c2h(1:nz), rdnw(1:nz), fnm(1:nz + 1), fnp(1:nz + 1)
  ALLOCATE(msftx(ims:ime, jms:jme), msfty(ims:ime, jms:jme), mub(ims:ime, jms:jme), mu_1(ims:ime, jms:jme), &
           mu_2(ims:ime, jms:jme), muts(ims:ime, jms:jme))
  CALL rd2(msftx); CALL rd2(msfty); CALL rd2(mub); CALL rd2(mu_1); CALL rd2(mu_2); CALL rd2(muts)
  ALLOCATE(ru(ims:ime, kms:kme, jms:jme), rv(ims:ime, kms:kme, jms:jme), ww(ims:ime, kms:kme, jms:jme), &
           wwi(ims:ime, kms:kme, jms:jme), adv(ims:ime, kms:kme, jms:jme), ht(ims:ime, kms:kme, jms:jme), &
           zt(ims:ime, kms:kme, jms:jme), rqvften(ims:ime, kms:kme, jms:jme))
  ALLOCATE(q1(ims:ime, kms:kme, jms:jme, nspec), q2(ims:ime, kms:kme, jms:jme, nspec), &
           qt(ims:ime, kms:kme, jms:jme, nspec))
  ru = 0.; rv = 0.; ww = 0.; wwi = 0.; rqvften = 0.; q1 = 0.; q2 = 0.; qt = 0.
  CALL rd3(ru, 1, 0, nz); CALL rd3(rv, 0, 1, nz); CALL rd3(ww, 0, 0, nz + 1)
  DO n = 1, nspec
    CALL rd3(q1(:,:,:,n), 0, 0, nz); CALL rd3(q2(:,:,:,n), 0, 0, nz); CALL rd3(qt(:,:,:,n), 0, 0, nz)
  END DO
  CLOSE(iu)

  cf%periodic_x = bc == 1; cf%periodic_y = bc == 1
  cf%symmetric_xs = .FALSE.; cf%symmetric_xe = .FALSE.; cf%symmetric_ys = .FALSE.; cf%symmetric_ye = .FALSE.
  cf%open_xs = bc == 2; cf%open_xe = bc == 2; cf%open_ys = bc == 2; cf%open_ye = bc == 2
  cf%specified = bc == 0; cf%nested = .FALSE.; cf%polar = .FALSE.
  cf%h_sca_adv_order = 5; cf%v_sca_adv_order = 3; cf%rk_ord = 3
  cf%moist_adv_opt = adv_opt; cf%zadvect_implicit = 0; cf%spec_zone = 1

  ! solve_em: rk_update_scalar_pd of every species, then the halos of moist_old (set_physical_bc3d 'p'), but for
  ! ORIGINAL and WENO_SCALAR
  IF (adv_opt /= 3) THEN
  DO n = 1, nspec
    CALL rk_update_scalar_pd(n, n, q1(ims, kms, jms, n), qt(ims, kms, jms, n), c1h, c2h, mu_1, mu_1, mub, &
                             3, dt, 1, cf, ids, ide, jds, jde, kds, kde, ims, ime, jms, jme, kms, kme, &
                             its, ite, jts, jte, kts, kte)
  END DO
  DO n = 1, nspec
    CALL set_physical_bc3d(q1(:,:,:,n), 'p', cf, ids, ide, jds, jde, kds, kde, ims, ime, jms, jme, kms, kme, &
                           ids, ide, jds, jde, kds, kde, its, ite, jts, jte, kts, kte)
  END DO
  END IF
  ! moist_scalar_advance: rk_scalar_tend (advection, rk_step 3) and rk_update_scalar of each species
  DO n = 1, nspec
    adv = 0.; ht = 0.; zt = 0.
    IF (adv_opt == 1) THEN
      CALL advect_scalar_pd(q2(:,:,:,n), q1(:,:,:,n), adv, ht, zt, ru, rv, ww, c1h, c2h, muts, mub, mu_1, &
                            1, cf, .FALSE., msftx, msftx, msftx, msftx, msftx, msfty, fnm, fnp, rdx, rdy, rdnw, dt, &
                            ids, ide, jds, jde, kds, kde, ims, ime, jms, jme, kms, kme, its, ite, jts, jte, kts, kte)
    ELSE IF (adv_opt == 3) THEN
      CALL advect_scalar_weno(q2(:,:,:,n), q2(:,:,:,n), adv, ru, rv, ww, c1h, c2h, muts, 1, cf, &
                              msftx, msftx, msftx, msftx, msftx, msfty, fnm, fnp, rdx, rdy, rdnw, &
                              ids, ide, jds, jde, kds, kde, ims, ime, jms, jme, kms, kme, its, ite, jts, jte, kts, kte)
    ELSE IF (adv_opt == 4) THEN
      CALL advect_scalar_wenopd(q2(:,:,:,n), q1(:,:,:,n), adv, ru, rv, ww, c1h, c2h, muts, mub, mu_1, 1, cf, &
                                msftx, msftx, msftx, msftx, msftx, msfty, fnm, fnp, rdx, rdy, rdnw, dt, &
                                ids, ide, jds, jde, kds, kde, ims, ime, jms, jme, kms, kme, its, ite, jts, jte, kts, kte)
    ELSE
      CALL advect_scalar_mono(q2(:,:,:,n), q1(:,:,:,n), adv, ht, zt, ru, rv, ww, wwi, c1h, c2h, muts, mub, mu_1, &
                              cf, .FALSE., msftx, msftx, msftx, msftx, msftx, msfty, fnm, fnp, rdx, rdy, rdnw, dt, &
                              ids, ide, jds, jde, kds, kde, ims, ime, jms, jme, kms, kme, its, ite, jts, jte, kts, kte)
    END IF
    IF (n == p_qv + 1) CALL set_tend(rqvften, adv, msfty, ids, ide, jds, jde, kds, kde, ims, ime, jms, jme, &
                                     kms, kme, its, ite, jts, jte, kts, kte)
    CALL rk_update_scalar(scs=n, sce=n, scalar_1=q1(ims, kms, jms, n), scalar_2=q2(ims, kms, jms, n), &
                          sc_tend=qt(ims, kms, jms, n), advect_tend=adv, h_tendency=ht, z_tendency=zt, &
                          msftx=msftx, msfty=msfty, c1=c1h, c2=c2h, mu_old=mu_1, mu_new=mu_2, mu_base=mub, &
                          rk_step=3, dt=dt, spec_zone=1, config_flags=cf, tenddec=.FALSE., &
                          ids=ids, ide=ide, jds=jds, jde=jde, kds=kds, kde=kde, ims=ims, ime=ime, jms=jms, &
                          jme=jme, kms=kms, kme=kme, its=its, ite=ite, jts=jts, jte=jte, kts=kts, kte=kte)
  END DO

  OPEN(NEWUNIT=iu, FILE=TRIM(outp), ACCESS='STREAM', FORM='UNFORMATTED', STATUS='REPLACE')
  DO n = 1, nspec
    DO k = 1, nz
      WRITE(iu) q2(1:nx, k, 1:ny, n)
    END DO
    DO k = 1, nz
      WRITE(iu) q1(1:nx, k, 1:ny, n)
    END DO
  END DO
  DO k = 1, nz
    WRITE(iu) rqvften(1:nx, k, 1:ny)
  END DO
  CLOSE(iu)

CONTAINS

  SUBROUTINE rd2(a)  ! 2-D mass-point field over the padded range
    REAL, INTENT(INOUT) :: a(ims:, jms:)
    REAL :: t(1 - H:nx + H, 1 - H:ny + H)
    a = 0.
    READ(iu) t
    a(1 - H:nx + H, 1 - H:ny + H) = t
  END SUBROUTINE rd2

  SUBROUTINE rd3(a, sx, sy, nzs)  ! 3-D field of staggering (sx, sy), nzs levels, over the padded range
    REAL, INTENT(INOUT) :: a(ims:, kms:, jms:)
    INTEGER, INTENT(IN) :: sx, sy, nzs
    REAL :: t(1 - H:nx + sx + H, 1 - H:ny + sy + H, nzs)
    INTEGER :: kk
    READ(iu) t
    DO kk = 1, nzs
      a(1 - H:nx + sx + H, kk, 1 - H:ny + sy + H) = t(:, :, kk)
    END DO
  END SUBROUTINE rd3

END PROGRAM driver_advection
