! SPDX-License-Identifier: GPL-3.0-or-later
! Single-layer urban canopy model column driver: WRF v4.8.0's urban_param_init and SUBROUTINE urban
! (module_sf_urban.o of the -r8 build) on the points of urban_input.bin, NSTEP steps each (the state carried).
!   driver_urban_fortran INPUT OUTPUT      (URBPARM.TBL / URBPARM_LCZ.TBL in the working directory)
program driver_urban_fortran
  use module_sf_urban, only: urban, urban_param_init
  implicit none
  character(len=256) :: infile, outfile
  integer :: npts, nstep, lcz, ip, istep, k, utype, jmonth
  integer, parameter :: nl = 4
  real(8) :: dzr(nl), dzb(nl), dzg(nl)
  real(8) :: f(18), st(29), o(18)
  real(8) :: ta, qa, ua, u1, v1, ssg, ssgd, ssgq, llg, rain, rhoo, za, declin, cosz, omg, xlat, delt, znt
  real(8) :: chs, chs2, tr, tb, tg, tc, qc, uc, xxxr, xxxb, xxxg, xxxc
  real(8) :: trl(nl), tbl(nl), tgl(nl)
  real(8) :: ts, qs, sh, lh, lhk, sw, alb, lw, g, rn, psim, psih, gz1oz0, cmr, chr, cmc, chc
  real(8) :: u10, v10, th2, q2, ust, mh, stdh, lf(4), lp, hgt, frc, lb, chk
  real(8) :: cmcr, tgr, tgrl(nl), smr(nl), cmgr, chgr, drelr, drelb, drelg, fhr, fhb, fhg
  real(8) :: tvg, tt, xxxvg, tvgl(nl), smg(nl), cmcg, fhvg, fht, lfs, z0u, vegf
  integer :: u_in, u_out

  call get_command_argument(1, infile)
  call get_command_argument(2, outfile)
  open(newunit=u_in, file=trim(infile), access='stream', form='unformatted', status='old')
  read(u_in) npts, nstep, lcz
  call urban_param_init(dzr, dzb, dzg, nl, 1, lcz, .false.)
  open(newunit=u_out, file=trim(outfile), access='stream', form='unformatted', status='replace')
  write(u_out) dzr, dzb, dzg
  do ip = 1, npts
    read(u_in) utype
    read(u_in) f, st
    ta = f(1); qa = f(2); ua = f(3); u1 = f(4); v1 = f(5); ssg = f(6); llg = f(7); rain = f(8); rhoo = f(9)
    za = f(10); declin = f(11); cosz = f(12); omg = f(13); xlat = f(14); delt = f(15); znt = f(16)
    tr = st(1); tb = st(2); tg = st(3); tc = st(4); qc = st(5); uc = st(6)
    xxxr = st(7); xxxb = st(8); xxxg = st(9); xxxc = st(10)
    trl = st(11:14); tbl = st(15:18); tgl = st(19:22)
    fhr = st(23); fhb = st(24); fhg = st(25); cmr = st(26); chr = st(27); cmc = st(28); chc = st(29)
    do istep = 1, nstep
      chs = f(17); chs2 = f(18)
      ssgd = 0.8 * ssg
      ssgq = ssg - ssgd
      mh = 0.; stdh = 0.; lf = 0.; lp = 0.; hgt = 0.; frc = 0.9; lb = 0.; chk = 0.
      cmcr = 0.; tgr = tr; tgrl = trl; smr = 0.2; cmgr = 0.; chgr = 0.; drelr = 0.; drelb = 0.; drelg = 0.
      tvg = tg; tt = tg; xxxvg = 0.; tvgl = tgl; smg = 0.2; cmcg = 0.; fhvg = 0.; fht = 0.
      lfs = 0.; z0u = 0.; vegf = 0.; jmonth = 7
      call urban(.false., nl, nl, nl, dzr, dzb, dzg, utype, ta, qa, ua, u1, v1, ssg, ssgd, ssgq, llg, rain, rhoo, &
                 za, declin, cosz, omg, xlat, delt, znt, chs, chs2, tr, tb, tg, tc, qc, uc, trl, tbl, tgl, &
                 xxxr, xxxb, xxxg, xxxc, ts, qs, sh, lh, lhk, sw, alb, lw, g, rn, psim, psih, gz1oz0, &
                 cmr, chr, cmc, chc, u10, v10, th2, q2, ust, mh, stdh, lf, lp, hgt, frc, lb, chk, &
                 cmcr, tgr, tgrl, smr, cmgr, chgr, jmonth, drelr, drelb, drelg, fhr, fhb, fhg, &
                 tvg, tt, xxxvg, tvgl, smg, cmcg, fhvg, fht, lfs, z0u, vegf)
      o = (/ ts, qs, sh, lh, lhk, sw, alb, lw, g, rn, psim, psih, gz1oz0, u10, v10, th2, q2, ust /)
      st(1:6) = (/ tr, tb, tg, tc, qc, uc /)
      st(7:10) = (/ xxxr, xxxb, xxxg, xxxc /)
      st(11:14) = trl; st(15:18) = tbl; st(19:22) = tgl
      st(23:29) = (/ fhr, fhb, fhg, cmr, chr, cmc, chc /)
      write(u_out) o, st
    end do
  end do
  close(u_in)
  close(u_out)
end program driver_urban_fortran
