! SPDX-License-Identifier: GPL-3.0-or-later
! WRF v4.8.0's NSSL 2-moment microphysics (module_mp_nssl_2mom.o of the -r8 -fdefault-double-8 build):
! nssl_2mom_init as mp_init calls it for mp_physics = 18 with its defaults (2-moment, hail, CCN, graupel and hail
! volume) and nssl_2mom_driver as module_microphysics_driver.F calls it, on an (i, k, j) tile for nsteps calls.
! Writes the state and diagnostics after every call.
program driver_nssl
  use module_mp_nssl_2mom, only: nssl_2mom_init, nssl_2mom_driver
  implicit none
  character(len=256) :: infile, outfile
  integer :: nx, ny, nz, nsteps, s, cu_used, ssat_out, ccna
  integer :: itimestep, diag, ke_diag
  real*8 :: dt
  real :: nssl_params(20)
  real*8, allocatable, dimension(:,:,:) :: th, qv, qc, qr, qi, qs, qh, qhl, ccw, crw, cci, csw, chw, chl, vhw, vhl, &
       cn, pii, p, w, dz, dn, qrcuten, qscuten, qicuten, qccuten, dbz, ssat3d, ssati, re_cloud, re_ice, re_snow, &
       zrw, zhw, zhl
  real*8, allocatable, dimension(:,:) :: rainnc, rainncv, snownc, snowncv, hailnc, hailncv, grplnc, grplncv, sr, &
       hail_maxk1, hail_max2d

  call get_command_argument(1, infile)
  call get_command_argument(2, outfile)
  open(10, file=trim(infile), access='stream', form='unformatted', status='old', convert='little_endian')
  read(10) nx, ny, nz, nsteps, cu_used, ssat_out, dt
  allocate(th(nx,nz,ny), qv(nx,nz,ny), qc(nx,nz,ny), qr(nx,nz,ny), qi(nx,nz,ny), qs(nx,nz,ny), qh(nx,nz,ny), &
       qhl(nx,nz,ny), ccw(nx,nz,ny), crw(nx,nz,ny), cci(nx,nz,ny), csw(nx,nz,ny), chw(nx,nz,ny), chl(nx,nz,ny), &
       vhw(nx,nz,ny), vhl(nx,nz,ny), cn(nx,nz,ny), pii(nx,nz,ny), p(nx,nz,ny), w(nx,nz,ny), dz(nx,nz,ny), &
       dn(nx,nz,ny), qrcuten(nx,nz,ny), qscuten(nx,nz,ny), qicuten(nx,nz,ny), qccuten(nx,nz,ny), dbz(nx,nz,ny), &
       ssat3d(nx,nz,ny), ssati(nx,nz,ny), re_cloud(nx,nz,ny), re_ice(nx,nz,ny), re_snow(nx,nz,ny), zrw(nx,nz,ny), &
       zhw(nx,nz,ny), zhl(nx,nz,ny))
  allocate(rainnc(nx,ny), rainncv(nx,ny), snownc(nx,ny), snowncv(nx,ny), hailnc(nx,ny), hailncv(nx,ny), &
       grplnc(nx,ny), grplncv(nx,ny), sr(nx,ny), hail_maxk1(nx,ny), hail_max2d(nx,ny))
  read(10) th, qv, qc, qr, qi, qs, qh, qhl, ccw, crw, cci, csw, chw, chl, vhw, vhl, cn, pii, p, w, dz, dn, &
       qrcuten, qscuten, qicuten, qccuten
  dbz = 0.; ssat3d = 0.; ssati = 0.; re_cloud = 0.; re_ice = 0.; re_snow = 0.; zrw = 0.; zhw = 0.; zhl = 0.
  read(10) rainnc, snownc, hailnc, grplnc
  rainncv = 0.; snowncv = 0.; hailncv = 0.; grplncv = 0.; sr = 0.; hail_maxk1 = 0.; hail_max2d = 0.

  ! mp_init: an empty namelist.input (no &nssl_mp_params) and namelist.output in the run directory
  open(12, file='namelist.input', status='replace'); write(12,'(A)') '&time_control'; write(12,'(A)') '/'; close(12)
  open(12, file='namelist.output', status='replace'); close(12)
  nssl_params(:) = 0
  nssl_params(1) = 0.5e9; nssl_params(2) = 0.; nssl_params(3) = 1.; nssl_params(4) = 4.e5; nssl_params(5) = 4.e4
  nssl_params(6) = 8.e5; nssl_params(7) = 3.e6; nssl_params(8) = 500.; nssl_params(9) = 900.
  nssl_params(10) = 100.; nssl_params(11) = 0.; nssl_params(12) = 4.
  ccna = 0
  call nssl_2mom_init(nssl_params=nssl_params, ipctmp=5, mixphase=0, nssl_density_on=.true., nssl_hail_on=.true., &
       nssl_ccn_on=.true., nssl_icdx=6, nssl_icdxhl=6, ccn_is_ccna=ccna)

  open(11, file=trim(outfile), access='stream', form='unformatted', status='replace', convert='little_endian')
  do s = 1, nsteps
    read(10) itimestep, diag, ke_diag
    call nssl_2mom_driver(ITIMESTEP=itimestep, TH=th, QV=qv, QC=qc, QR=qr, QI=qi, QS=qs, QH=qh, QHL=qhl, &
         CCW=ccw, CRW=crw, CCI=cci, CSW=csw, CHW=chw, CHL=chl, VHW=vhw, f_vhw=.true., VHL=vhl, f_vhl=.true., &
         ZRW=zrw, f_zrw=.false., ZHW=zhw, f_zhw=.false., ZHL=zhl, f_zhl=.false., cn=cn, f_cn=.true., &
         PII=pii, P=p, W=w, DZ=dz, DTP=dt, DN=dn, RAINNC=rainnc, RAINNCV=rainncv, SNOWNC=snownc, &
         SNOWNCV=snowncv, HAILNC=hailnc, HAILNCV=hailncv, GRPLNC=grplnc, GRPLNCV=grplncv, SR=sr, dbz=dbz, &
         ssat3d=ssat3d, ssati=ssati, nssl_ssat_output=ssat_out, nssl_progn=.false., diagflag=(diag /= 0), &
         ke_diag=ke_diag, cu_used=cu_used, qrcuten=qrcuten, qscuten=qscuten, qicuten=qicuten, qccuten=qccuten, &
         re_cloud=re_cloud, re_ice=re_ice, re_snow=re_snow, has_reqc=1, has_reqi=1, has_reqs=1, &
         hail_maxk1=hail_maxk1, hail_max2d=hail_max2d, nwp_diagnostics=0, &
         IDS=1, IDE=nx+1, JDS=1, JDE=ny+1, KDS=1, KDE=nz+1, IMS=1, IME=nx, JMS=1, JME=ny, KMS=1, KME=nz, &
         ITS=1, ITE=nx, JTS=1, JTE=ny, KTS=1, KTE=nz)
    write(11) th, qv, qc, qr, qi, qs, qh, qhl, ccw, crw, cci, csw, chw, chl, vhw, vhl, cn, dbz, ssat3d, ssati, &
         re_cloud, re_ice, re_snow
    write(11) rainnc, rainncv, snownc, snowncv, hailnc, hailncv, grplnc, grplncv, sr
  enddo
  close(11)
  close(10)
end program driver_nssl
