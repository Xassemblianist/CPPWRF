! SPDX-License-Identifier: GPL-3.0-or-later
! WRF v4.8.0's QNSESFCINIT, QNSESFC and QNSEPBL (module_sf_qnsesfc.o, module_bl_qnsepbl.o of the -r8 build) for the
! columns of generate_inputs.py, as the drivers call them: the surface layer (SCM_FORCE_FLUX = 0), then the PBL with
! ELFLX = the surface layer's latent heat flux and WTHV_MF = LM_BL89 = 0 (mfshconv = 0: the driver's zeroed
! u_phytmp / v_phytmp). Writes the tables and every output in driver_qnse.cpp's order.
program driver_qnse_fortran
  use module_sf_qnsesfc, only: qnsesfcinit, qnsesfc, psim1, psih1, psim2, psih2, ztmin1, ztmax1, dzeta1, &
       ztmin2, ztmax2, dzeta2
  use module_bl_qnsepbl, only: qnsepbl
  implicit none
  character(len=256) :: infile, outfile
  integer :: n, nz, itimestep, stepbl, i, k, iunit
  real(8) :: dt
  real(8), allocatable, dimension(:,:,:) :: dz, pmid, th, t, exner, qv, qc, u, v, tke, pint, exch_h, exch_m, el, &
       rublten, rvblten, rthblten, rqvblten, rqcblten, zero3
  real(8), allocatable, dimension(:,:) :: ht, tsk, xland, z0base, mavail, znt, ust, qsfc, thz0, qz0, uz0, vz0, &
       akhs, akms, hfx, qfx, corf, chklowq, sice, snow, ct, pblh, rmol, rib, chs, chs2, cqs2, flx_lh, flhc, flqc, &
       qgh, cpm, u10, v10, t02, th02, tshltr, th10, q02, qshltr, q10, pshltr, u10e, v10e, xice
  integer, allocatable :: lowlyr(:,:), ivgtyp(:,:), kpbl(:,:)
  call get_command_argument(1, infile)
  call get_command_argument(2, outfile)
  open(newunit=iunit, file=trim(infile), status='old', access='stream', form='unformatted', convert='little_endian')
  read(iunit) n, nz, itimestep, stepbl
  read(iunit) dt
  allocate(dz(n,nz+1,1), pmid(n,nz+1,1), th(n,nz+1,1), t(n,nz+1,1), exner(n,nz+1,1), qv(n,nz+1,1), qc(n,nz+1,1), &
           u(n,nz+1,1), v(n,nz+1,1), tke(n,nz+1,1), pint(n,nz+1,1), exch_h(n,nz+1,1), exch_m(n,nz+1,1), &
           el(n,nz+1,1), rublten(n,nz+1,1), rvblten(n,nz+1,1), rthblten(n,nz+1,1), rqvblten(n,nz+1,1), &
           rqcblten(n,nz+1,1), zero3(n,nz+1,1))
  dz = 0; pmid = 0; th = 0; t = 0; exner = 1; qv = 0; qc = 0; u = 0; v = 0; tke = 0; pint = 0; exch_h = 0
  exch_m = 0; el = 0; rublten = 0; rvblten = 0; rthblten = 0; rqvblten = 0; rqcblten = 0; zero3 = 0
  read(iunit) ((dz(i,k,1), i=1,n), k=1,nz)
  read(iunit) ((pmid(i,k,1), i=1,n), k=1,nz)
  read(iunit) ((th(i,k,1), i=1,n), k=1,nz)
  read(iunit) ((t(i,k,1), i=1,n), k=1,nz)
  read(iunit) ((exner(i,k,1), i=1,n), k=1,nz)
  read(iunit) ((qv(i,k,1), i=1,n), k=1,nz)
  read(iunit) ((qc(i,k,1), i=1,n), k=1,nz)
  read(iunit) ((u(i,k,1), i=1,n), k=1,nz)
  read(iunit) ((v(i,k,1), i=1,n), k=1,nz)
  read(iunit) ((tke(i,k,1), i=1,n), k=1,nz)
  read(iunit) ((pint(i,k,1), i=1,n), k=1,nz+1)
  allocate(ht(n,1), tsk(n,1), xland(n,1), z0base(n,1), mavail(n,1), znt(n,1), ust(n,1), qsfc(n,1), thz0(n,1), &
           qz0(n,1), uz0(n,1), vz0(n,1), akhs(n,1), akms(n,1), hfx(n,1), qfx(n,1), corf(n,1), chklowq(n,1), &
           sice(n,1), snow(n,1), ct(n,1), pblh(n,1), rmol(n,1), rib(n,1), chs(n,1), chs2(n,1), cqs2(n,1), &
           flx_lh(n,1), flhc(n,1), flqc(n,1), qgh(n,1), cpm(n,1), u10(n,1), v10(n,1), t02(n,1), th02(n,1), &
           tshltr(n,1), th10(n,1), q02(n,1), qshltr(n,1), q10(n,1), pshltr(n,1), u10e(n,1), v10e(n,1), xice(n,1), &
           lowlyr(n,1), ivgtyp(n,1), kpbl(n,1))
  read(iunit) ht, tsk, xland, z0base, mavail, znt, ust, qsfc, thz0, qz0, uz0, vz0, akhs, akms, hfx, qfx, corf, &
       chklowq, sice, snow, ct
  close(iunit)
  pblh = 0; rmol = 0; rib = 0; chs = 0; chs2 = 0; cqs2 = 0; flx_lh = 0; flhc = 0; flqc = 0; qgh = 0; cpm = 0
  u10 = 0; v10 = 0; t02 = 0; th02 = 0; tshltr = 0; th10 = 0; q02 = 0; qshltr = 0; q10 = 0; pshltr = 0; u10e = 0
  v10e = 0; xice = 0; ivgtyp = 1; kpbl = 0

  ! qnsesfcinit (restart: keeps the inputs' USTAR; LOWLYR = 1, the tables)
  call qnsesfcinit(lowlyr, ust, znt, xland, xice, ivgtyp, .true., .true., 1, n+1, 1, 2, 1, nz+1, &
       1, n, 1, 1, 1, nz+1, 1, n, 1, 1, 1, nz)
  call qnsesfc(itimestep, ht, dz, pmid, pint, th, t, qv, qc, u, v, tke, tsk, qsfc, thz0, qz0, uz0, vz0, lowlyr, &
       xland, ust, znt, z0base, pblh, mavail, rmol, akhs, akms, rib, chs, chs2, cqs2, hfx, qfx, flx_lh, flhc, flqc, &
       qgh, cpm, ct, u10, v10, t02, th02, tshltr, th10, q02, qshltr, q10, pshltr, u10e, v10e, &
       1, n+1, 1, 2, 1, nz+1, 1, n, 1, 1, 1, nz+1, 1, n, 1, 1, 1, nz, 0)
  open(newunit=iunit, file=trim(outfile), status='replace', access='stream', form='unformatted', &
       convert='little_endian')
  write(iunit) psim1, psih1, psim2, psih2, ztmin1, ztmax1, dzeta1, ztmin2, ztmax2, dzeta2
  write(iunit) qsfc, thz0, qz0, uz0, vz0, ust, znt, pblh, rmol, akhs, akms, rib, chs, chs2, cqs2, hfx, qfx, &
       flx_lh, flhc, flqc, qgh, cpm, ct, u10, v10, t02, th02, tshltr, th10, q02, qshltr, q10, pshltr, u10e, v10e
  call qnsepbl(dt=dt, stepbl=stepbl, ht=ht, dz=dz, pmid=pmid, pint=pint, th=th, t=t, exner=exner, qv=qv, cwm=qc, &
       u=u, v=v, rho=zero3, tsk=tsk, qsfc=qsfc, chklowq=chklowq, thz0=thz0, qz0=qz0, uz0=uz0, vz0=vz0, corf=corf, &
       lowlyr=lowlyr, xland=xland, sice=sice, snow=snow, tke=tke, exch_h=exch_h, exch_m=exch_m, ustar=ust, &
       znt=znt, el_myj=el, pblh=pblh, kpbl=kpbl, ct=ct, akhs=akhs, akms=akms, elflx=flx_lh, hfx=hfx, &
       wthv_mf=zero3, lm_bl89=zero3, rublten=rublten, rvblten=rvblten, rthblten=rthblten, rqvblten=rqvblten, &
       rqcblten=rqcblten, ids=1, ide=n+1, jds=1, jde=2, kds=1, kde=nz+1, ims=1, ime=n, jms=1, jme=1, kms=1, &
       kme=nz+1, its=1, ite=n, jts=1, jte=1, kts=1, kte=nz)
  write(iunit) ((tke(i,k,1), i=1,n), k=1,nz)
  write(iunit) ((exch_h(i,k,1), i=1,n), k=1,nz)
  write(iunit) ((exch_m(i,k,1), i=1,n), k=1,nz)
  write(iunit) ((el(i,k,1), i=1,n), k=1,nz)
  write(iunit) ((rublten(i,k,1), i=1,n), k=1,nz)
  write(iunit) ((rvblten(i,k,1), i=1,n), k=1,nz)
  write(iunit) ((rthblten(i,k,1), i=1,n), k=1,nz)
  write(iunit) ((rqvblten(i,k,1), i=1,n), k=1,nz)
  write(iunit) ((rqcblten(i,k,1), i=1,n), k=1,nz)
  write(iunit) pblh, (real(kpbl(i,1), 8), i=1,n), ct, qsfc, thz0, qz0
  close(iunit)
end program driver_qnse_fortran
