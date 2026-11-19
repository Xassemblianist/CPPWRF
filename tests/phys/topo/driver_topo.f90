! SPDX-License-Identifier: GPL-3.0-or-later
! Terrain-option column test, WRF side: WRF v4.8.0's radconst, toposhad_init and toposhad (module_radiation_driver.o,
! one patch: niter = 1, nested = .false., as pre_radiation_driver of a serial run) and TOPO_RAD_ADJ_DRVR
! (module_surface_driver.o) of the -r8 build on the inputs of generate_inputs.py.
!   driver_topo_fortran INPUT OUTPUT
program driver_topo
  use module_radiation_driver, only: toposhad_init, toposhad, radconst
  use module_surface_driver, only: topo_rad_adj_drvr
  use module_model_constants, only: degrad, dpd
  use module_bc, only: set_physical_bc2d
  use module_configure, only: grid_config_rec_type
  implicit none
  character(len=256) :: fin, fout
  real(8) :: h(10)
  integer :: nx, ny, nt, n, it, ims, ime, jms, jme, i, j
  real :: dx, dy, shadlen, gmt, radfrq, julian, declin, solcon, xtime
  real, allocatable :: ht(:,:), ht_loc(:,:), ht_shad(:,:), xlat(:,:), xlong(:,:), sina(:,:), cosa(:,:), times(:)
  integer, allocatable :: mask(:,:)
  real, allocatable :: r_xlat(:,:), cz(:,:), df(:,:), sw(:,:), gsw(:,:), hr(:,:), sl(:,:), az(:,:), shd(:,:), &
                       swn(:,:), gss(:,:)
  integer, allocatable :: imask(:,:)
  real :: rdecl
  type(grid_config_rec_type) :: cf
  call get_command_argument(1, fin)
  call get_command_argument(2, fout)
  open(10, file=fin, access='stream', form='unformatted', status='old')
  read(10) h
  nx = nint(h(1)); ny = nint(h(2)); dx = h(3); dy = h(4); shadlen = h(5); gmt = h(6); radfrq = h(7); julian = h(8)
  nt = nint(h(9)); n = nint(h(10))
  ! the memory of a serial WRF run: the patch (ids..ide = 1..nx+1) and bdx = 4 points
  ims = 1 - 4; ime = nx + 1 + 4; jms = 1 - 4; jme = ny + 1 + 4
  allocate(ht(ims:ime,jms:jme), ht_loc(ims:ime,jms:jme), ht_shad(ims:ime,jms:jme), xlat(ims:ime,jms:jme), &
           xlong(ims:ime,jms:jme), sina(ims:ime,jms:jme), cosa(ims:ime,jms:jme), mask(ims:ime,jms:jme), times(nt))
  ht = 0.; xlat = 0.; xlong = 0.; sina = 0.; cosa = 0.; ht_shad = 0.; mask = 0
  read(10) times
  read(10) ((ht(i,j), i = 1, nx), j = 1, ny)
  read(10) ((xlat(i,j), i = 1, nx), j = 1, ny)
  read(10) ((xlong(i,j), i = 1, nx), j = 1, ny)
  read(10) ((sina(i,j), i = 1, nx), j = 1, ny)
  read(10) ((cosa(i,j), i = 1, nx), j = 1, ny)
  ! start_em: set_physical_bc2d(grid%ht, 'r') of a real-data domain (specified boundaries; tile = patch = domain)
  cf%specified = .true.; cf%nested = .false.; cf%polar = .false.
  cf%periodic_x = .false.; cf%periodic_y = .false.
  cf%symmetric_xs = .false.; cf%symmetric_xe = .false.; cf%symmetric_ys = .false.; cf%symmetric_ye = .false.
  cf%open_xs = .false.; cf%open_xe = .false.; cf%open_ys = .false.; cf%open_ye = .false.
  call set_physical_bc2d(ht, 'r', cf, 1, nx + 1, 1, ny + 1, ims, ime, jms, jme, 1, nx + 1, 1, ny + 1, &
                         1, nx + 1, 1, ny + 1)
  allocate(r_xlat(n,1), cz(n,1), df(n,1), sw(n,1), gsw(n,1), hr(n,1), sl(n,1), az(n,1), shd(n,1), swn(n,1), &
           gss(n,1), imask(n,1))
  read(10) rdecl
  read(10) r_xlat, cz, df, sw, gsw, hr, sl, az, shd
  close(10)
  open(11, file=fout, access='stream', form='unformatted', status='replace')
  do it = 1, nt
    xtime = times(it)
    call radconst(xtime, declin, solcon, julian, degrad, dpd)
    ht_loc = ht  ! pre_radiation_driver: ht_loc(ims:ime, jms:jme) = ht
    call toposhad_init(ht_shad, ht_loc, mask, .false., 1, 1, nx + 1, 1, ny + 1, 1, 2, ims, ime, jms, jme, 1, 2, &
                       1, nx, 1, ny, 1, 2, 1, nx, 1, ny, 1, 1)
    call toposhad(xlat, xlong, sina, cosa, xtime, gmt, radfrq, declin, dx, dy, ht_shad, ht_loc, 1, mask, shadlen, &
                  1, nx + 1, 1, ny + 1, 1, 2, ims, ime, jms, jme, 1, 2, 1, nx, 1, ny, 1, 2, 1, nx, 1, ny, 1, 1)
    write(11) real(declin, 8), ((real(mask(i,j), 8), i = 1, nx), j = 1, ny), ((real(ht_shad(i,j), 8), i = 1, nx), j = 1, ny)
  end do
  imask = nint(shd)
  swn = 0.; gss = 0.
  call topo_rad_adj_drvr(r_xlat, r_xlat, cz, imask, df, rdecl, sw, gsw, swn, gss, 1370., hr, sl, az, &
                         1, n + 1, 1, 2, 1, 2, 1, n, 1, 1, 1, 2, 1, n, 1, 1, 1, 1)
  write(11) real(sw, 8), real(gsw, 8), real(swn, 8), real(gss, 8)
  close(11)
end program driver_topo
