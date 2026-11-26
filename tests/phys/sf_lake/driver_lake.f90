! SPDX-License-Identifier: GPL-3.0-or-later
! Lake model column test, WRF side: lakeini and Lake of WRF v4.8.0's module_sf_lake.o (-r8 build) on the points and
! forcing of lake_input.bin (generate_inputs.py), every state and output array written after lakeini and each step.
program driver_lake
  use module_sf_lake
  implicit none
  integer :: n, nsteps, ims, ime, jms, jme, kms, kme, i, k, s, lakeflag, lake_depth_flag, use_lakedepth, iswater, u
  real :: hdr(10)
  real :: dt, lakedepth_default, lake_min_elev, xice_threshold
  real, allocatable :: ht(:,:), snow(:,:), tsk(:,:), xice(:,:), xland(:,:), lakemask(:,:), lake_depth(:,:), xlat(:,:)
  real, allocatable :: isl_r(:,:), ivg_r(:,:)
  integer, allocatable :: ivgtyp(:,:), isltyp(:,:)
  logical, allocatable :: lakepts(:,:)
  real, allocatable :: lakedepth2d(:,:), savedtke12d(:,:), snowdp2d(:,:), h2osno2d(:,:), snl2d(:,:), t_grnd2d(:,:)
  real, allocatable :: t_lake3d(:,:,:), lake_icefrac3d(:,:,:), z_lake3d(:,:,:), dz_lake3d(:,:,:)
  real, allocatable :: t_soisno3d(:,:,:), h2osoi_ice3d(:,:,:), h2osoi_liq3d(:,:,:), h2osoi_vol3d(:,:,:), z3d(:,:,:), dz3d(:,:,:)
  real, allocatable :: zi3d(:,:,:), watsat3d(:,:,:), csol3d(:,:,:), tkmg3d(:,:,:), tkdry3d(:,:,:), tksatu3d(:,:,:)
  real, allocatable :: t_phy(:,:,:), p8w(:,:,:), dz8w(:,:,:), qv(:,:,:), u_phy(:,:,:), v_phy(:,:,:)
  real, allocatable :: glw(:,:), emiss(:,:), rainbl(:,:), swdown(:,:), albedo(:,:)
  real, allocatable :: hfx(:,:), lh(:,:), grdflx(:,:), qfx(:,:), t2(:,:), th2(:,:), q2(:,:)
  real, allocatable :: buf(:)
  character(len=256) :: fin, fout

  call get_command_argument(1, fin)
  call get_command_argument(2, fout)
  open(newunit=u, file=trim(fin), access='stream', form='unformatted', status='old')
  read(u) hdr
  n = nint(hdr(1)); nsteps = nint(hdr(2)); dt = hdr(3); lakedepth_default = hdr(4); use_lakedepth = nint(hdr(5))
  lake_depth_flag = nint(hdr(6)); lakeflag = nint(hdr(7)); lake_min_elev = hdr(8); xice_threshold = hdr(9)
  iswater = nint(hdr(10))
  ims = 1; ime = n; jms = 1; jme = 1; kms = 1; kme = 2
  allocate(ht(n,1), snow(n,1), tsk(n,1), xice(n,1), xland(n,1), lakemask(n,1), lake_depth(n,1), xlat(n,1))
  allocate(isl_r(n,1), ivg_r(n,1), ivgtyp(n,1), isltyp(n,1), lakepts(n,1))
  read(u) ht, snow, tsk, xice, xland, lakemask, lake_depth, xlat, isl_r, ivg_r
  isltyp = nint(isl_r); ivgtyp = nint(ivg_r)
  allocate(lakedepth2d(n,1), savedtke12d(n,1), snowdp2d(n,1), h2osno2d(n,1), snl2d(n,1), t_grnd2d(n,1))
  allocate(t_lake3d(n,10,1), lake_icefrac3d(n,10,1), z_lake3d(n,10,1), dz_lake3d(n,10,1))
  allocate(t_soisno3d(n,-4:10,1), h2osoi_ice3d(n,-4:10,1), h2osoi_liq3d(n,-4:10,1), h2osoi_vol3d(n,-4:10,1))
  allocate(z3d(n,-4:10,1), dz3d(n,-4:10,1), zi3d(n,-5:10,1))
  allocate(watsat3d(n,10,1), csol3d(n,10,1), tkmg3d(n,10,1), tkdry3d(n,10,1), tksatu3d(n,10,1))
  lakedepth2d = 0; savedtke12d = 0; snowdp2d = 0; h2osno2d = 0; snl2d = 0; t_grnd2d = 0
  t_lake3d = 0; lake_icefrac3d = 0; z_lake3d = 0; dz_lake3d = 0; t_soisno3d = 0; h2osoi_ice3d = 0; h2osoi_liq3d = 0
  h2osoi_vol3d = 0; z3d = 0; dz3d = 0; zi3d = 0; watsat3d = 0; csol3d = 0; tkmg3d = 0; tkdry3d = 0; tksatu3d = 0
  call lakeini(ivgtyp, isltyp, ht, snow, lake_min_elev, .false., lakedepth_default, lake_depth, &
               lakedepth2d, savedtke12d, snowdp2d, h2osno2d, snl2d, t_grnd2d, t_lake3d, lake_icefrac3d, &
               z_lake3d, dz_lake3d, t_soisno3d, h2osoi_ice3d, h2osoi_liq3d, h2osoi_vol3d, z3d, dz3d, &
               zi3d, watsat3d, csol3d, tkmg3d, iswater, xice, xice_threshold, xland, tsk, &
               lakemask, lakeflag, lake_depth_flag, use_lakedepth, &
               tkdry3d, tksatu3d, lakepts, 1, n, 1, 1, ims, ime, jms, jme)
  open(newunit=k, file=trim(fout), access='stream', form='unformatted', status='replace')
  write(k) real(ivgtyp), xland, xice, lakemask
  write(k) watsat3d, csol3d, tkmg3d, tkdry3d, tksatu3d, z_lake3d, dz_lake3d, lakedepth2d
  call put_state()
  allocate(t_phy(n,2,1), p8w(n,2,1), dz8w(n,2,1), qv(n,2,1), u_phy(n,2,1), v_phy(n,2,1))
  allocate(glw(n,1), emiss(n,1), rainbl(n,1), swdown(n,1), albedo(n,1))
  allocate(hfx(n,1), lh(n,1), grdflx(n,1), qfx(n,1), t2(n,1), th2(n,1), q2(n,1))
  t_phy = 0; p8w = 0; dz8w = 0; qv = 0; u_phy = 0; v_phy = 0
  hfx = -1; lh = -1; grdflx = -1; qfx = -1; t2 = -1; th2 = -1; q2 = -1
  do s = 1, nsteps
    read(u) t_phy(:,1,1), p8w(:,1,1), p8w(:,2,1), dz8w(:,1,1), qv(:,1,1), u_phy(:,1,1), v_phy(:,1,1), glw, emiss, &
            rainbl, swdown, albedo
    call Lake(t_phy, p8w, dz8w, qv, u_phy, v_phy, glw, emiss, rainbl, dt, swdown, albedo, &
              xlat, z_lake3d, dz_lake3d, lakedepth2d, watsat3d, csol3d, tkmg3d, tkdry3d, tksatu3d, &
              ivgtyp, ht, xland, iswater, xice, xice_threshold, lake_min_elev, &
              1, n + 1, 1, 2, 1, 2, ims, ime, jms, jme, kms, kme, 1, n, 1, 1, 1, 1, &
              h2osno2d, snowdp2d, snl2d, z3d, dz3d, zi3d, h2osoi_vol3d, h2osoi_liq3d, h2osoi_ice3d, t_grnd2d, &
              t_soisno3d, t_lake3d, savedtke12d, lake_icefrac3d, lakemask, &
              hfx, lh, grdflx, tsk, qfx, t2, th2, q2)
    write(k) hfx, lh, grdflx, tsk, qfx, t2, th2, q2, albedo
    call put_state()
  end do
  close(k)
  close(u)
contains
  subroutine put_state()
    write(k) savedtke12d, snowdp2d, h2osno2d, snl2d, t_grnd2d, t_lake3d, lake_icefrac3d, t_soisno3d, &
             h2osoi_ice3d, h2osoi_liq3d, h2osoi_vol3d, z3d, dz3d, zi3d
  end subroutine
end program driver_lake
