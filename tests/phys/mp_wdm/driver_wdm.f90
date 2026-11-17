! SPDX-License-Identifier: GPL-3.0-or-later
! The WRF side of the WDM column test: wdm6init + wdm6 (scheme 6) or wdm5init + wdm5 (scheme 5) of the -r8 build's
! module_mp_wdm6.o / module_mp_wdm5.o on an ni x nj slab
! (its..ite = 1..ni: the lanes of the vectorised i loop), nsteps calls in a row (itimestep it0, it0 + 1, ...: 1 sets
! NN = ccn0 first), with the microphysics_driver's arguments (module_model_constants), the namelist's ccn_conc and
! hail_opt, and the effective radii (has_reqc = has_reqi = has_reqs = 1, as with RRTMG). Output: the init's module
! variables, then per step the state, RE_CLOUD / RE_ICE / RE_SNOW and the precipitation (layout of compare.py).
program driver_wdm
  use module_mp_wdm6, only: wdm6, wdm6init, qc0, qc1, qck1, pidnc, g1pbr, g2pbr, g3pbr, g4pbr, g5pbr, g6pbr, g7pbr, &
      g5pbro2, g7pbro2, pi, pvtr, pvtrn, pacrr, pidn0r, pidnr, precr1, precr2, xmmax, roqimax, g1pbs, g3pbs, g4pbs, &
      g5pbso2, pvts, pacrs, precs1, precs2, pidn0s, xlv1, pacrc, g1pbg, g3pbg, g4pbg, g5pbgo2, pvtg, pacrg, precg1, &
      precg2, pidn0g, n0g, avtg, bvtg, deng, lamdagmax, rslopecmax, rslopec2max, rslopec3max, rslopermax, rslopesmax, &
      rslopegmax, rsloperbmax, rslopesbmax, rslopegbmax, rsloper2max, rslopes2max, rslopeg2max, rsloper3max, &
      rslopes3max, rslopeg3max
  use module_mp_wdm5, only: wdm5, wdm5init, w5_pidnc => pidnc, w5_pi => pi, w5_pvtr => pvtr, w5_pvtrn => pvtrn, &
      w5_pidnr => pidnr, w5_precr1 => precr1, w5_precr2 => precr2, w5_roqimax => roqimax, w5_pvts => pvts, &
      w5_precs1 => precs1, w5_precs2 => precs2, w5_pidn0s => pidn0s, w5_xlv1 => xlv1, w5_pacrc => pacrc, &
      w5_rslopecmax => rslopecmax, w5_rslopec2max => rslopec2max, w5_rslopec3max => rslopec3max, &
      w5_rslopermax => rslopermax, w5_rslopesmax => rslopesmax, w5_rsloperbmax => rsloperbmax, &
      w5_rslopesbmax => rslopesbmax, w5_rsloper2max => rsloper2max, w5_rslopes2max => rslopes2max, &
      w5_rsloper3max => rsloper3max, w5_rslopes3max => rslopes3max
  use module_model_constants, only: g, cp, cpv, r_d, r_v, svpt0, ep_1, ep_2, epsilon, xls, xlv, xlf, rhoair0, &
      rhowater, rhosnow, cliq, cice, psat
  implicit none
  integer, parameter :: nconst = 63
  character(len=256) :: infile, outfile
  integer :: hdr(7), ni, nj, nz, nsteps, scheme, hail, it0, s
  real*8 :: dt, ccn
  real*8, allocatable, dimension(:,:,:) :: th, q, qc, qr, qi, qs, qg, qh, nn, nc, nr, den, pii, p, delz, re_c, re_i, &
      re_s, refl
  real*8, allocatable, dimension(:,:) :: xland, rain, rainncv, snow, snowncv, graupel, graupelncv, hail2, hailncv, sr
  real*8 :: c(nconst)
  call get_command_argument(1, infile)
  call get_command_argument(2, outfile)
  open(10, file=trim(infile), status='old', access='stream', form='unformatted')
  read(10) hdr, dt, ccn
  ni = hdr(1); nj = hdr(2); nz = hdr(3); nsteps = hdr(4); scheme = hdr(5); hail = hdr(6); it0 = hdr(7)
  allocate(th(ni,nz,nj), q(ni,nz,nj), qc(ni,nz,nj), qr(ni,nz,nj), qi(ni,nz,nj), qs(ni,nz,nj), qg(ni,nz,nj), &
           qh(ni,nz,nj), nn(ni,nz,nj), nc(ni,nz,nj), nr(ni,nz,nj), den(ni,nz,nj), pii(ni,nz,nj), p(ni,nz,nj), &
           delz(ni,nz,nj), re_c(ni,nz,nj), re_i(ni,nz,nj), re_s(ni,nz,nj), refl(ni,nz,nj), xland(ni,nj), &
           rain(ni,nj), rainncv(ni,nj), snow(ni,nj), snowncv(ni,nj), graupel(ni,nj), graupelncv(ni,nj), &
           hail2(ni,nj), hailncv(ni,nj), sr(ni,nj))
  read(10) th, q, qc, qr, qi, qs, qg, qh, nn, nc, nr, den, pii, p, delz, xland, rain, snow, graupel, hail2
  close(10)
  rainncv = 0.d0; snowncv = 0.d0; graupelncv = 0.d0; hailncv = 0.d0; sr = 0.d0
  re_c = 0.d0; re_i = 0.d0; re_s = 0.d0; refl = 0.d0
  c = 0.d0
  if (scheme == 6) then
    call wdm6init(rhoair0, rhowater, rhosnow, cliq, cpv, ccn, hail, .true.)
    c = (/ qc0, qc1, qck1, pidnc, g1pbr, g2pbr, g3pbr, g4pbr, g5pbr, g6pbr, g7pbr, g5pbro2, g7pbro2, pi, pvtr, &
           pvtrn, pacrr, pidn0r, pidnr, precr1, precr2, xmmax, roqimax, g1pbs, g3pbs, g4pbs, g5pbso2, pvts, pacrs, &
           precs1, precs2, pidn0s, xlv1, pacrc, g1pbg, g3pbg, g4pbg, g5pbgo2, pvtg, pacrg, precg1, precg2, pidn0g, &
           n0g, avtg, bvtg, deng, lamdagmax, rslopecmax, rslopec2max, rslopec3max, rslopermax, rslopesmax, &
           rslopegmax, rsloperbmax, rslopesbmax, rslopegbmax, rsloper2max, rslopes2max, rslopeg2max, rsloper3max, &
           rslopes3max, rslopeg3max /)
  else if (scheme == 5) then  ! the wdm5init variables wdm52D reads
    call wdm5init(rhoair0, rhowater, rhosnow, cliq, cpv, ccn, .true.)
    c(1:25) = (/ w5_pidnc, w5_pi, w5_pvtr, w5_pvtrn, w5_pidnr, w5_precr1, w5_precr2, w5_roqimax, w5_pvts, &
                 w5_precs1, w5_precs2, w5_pidn0s, w5_xlv1, w5_pacrc, w5_rslopecmax, w5_rslopec2max, w5_rslopec3max, &
                 w5_rslopermax, w5_rslopesmax, w5_rsloperbmax, w5_rslopesbmax, w5_rsloper2max, w5_rslopes2max, &
                 w5_rsloper3max, w5_rslopes3max /)
  end if
  open(20, file=trim(outfile), status='replace', access='stream', form='unformatted')
  write(20) c
  do s = 1, nsteps
    if (scheme == 6) then
      call wdm6(TH=th, Q=q, QC=qc, QR=qr, QI=qi, QS=qs, QG=qg, NN=nn, NC=nc, NR=nr, DEN=den, PII=pii, P=p, &
                DELZ=delz, DELT=dt, G=g, CPD=cp, CPV=cpv, CCN0=ccn, RD=r_d, RV=r_v, T0C=svpt0, EP1=ep_1, EP2=ep_2, &
                QMIN=epsilon, XLS=xls, XLV0=xlv, XLF0=xlf, DEN0=rhoair0, DENR=rhowater, CLIQ=cliq, CICE=cice, &
                PSAT=psat, XLAND=xland, RAIN=rain, RAINNCV=rainncv, SNOW=snow, SNOWNCV=snowncv, SR=sr, &
                REFL_10CM=refl, diagflag=.false., do_radar_ref=0, GRAUPEL=graupel, GRAUPELNCV=graupelncv, &
                ITIMESTEP=it0 + s - 1, has_reqc=1, has_reqi=1, has_reqs=1, re_cloud=re_c, re_ice=re_i, &
                re_snow=re_s, &
                IDS=1, IDE=ni+1, JDS=1, JDE=nj+1, KDS=1, KDE=nz+1, IMS=1, IME=ni, JMS=1, JME=nj, KMS=1, KME=nz, &
                ITS=1, ITE=ni, JTS=1, JTE=nj, KTS=1, KTE=nz)
    else if (scheme == 5) then
      call wdm5(TH=th, Q=q, QC=qc, QR=qr, QI=qi, QS=qs, NN=nn, NC=nc, NR=nr, DEN=den, PII=pii, P=p, DELZ=delz, &
                DELT=dt, G=g, CPD=cp, CPV=cpv, CCN0=ccn, RD=r_d, RV=r_v, T0C=svpt0, EP1=ep_1, EP2=ep_2, &
                QMIN=epsilon, XLS=xls, XLV0=xlv, XLF0=xlf, DEN0=rhoair0, DENR=rhowater, CLIQ=cliq, CICE=cice, &
                PSAT=psat, RAIN=rain, RAINNCV=rainncv, SNOW=snow, SNOWNCV=snowncv, SR=sr, REFL_10CM=refl, &
                diagflag=.false., do_radar_ref=0, has_reqc=1, has_reqi=1, has_reqs=1, re_cloud=re_c, re_ice=re_i, &
                re_snow=re_s, ITIMESTEP=it0 + s - 1, &
                IDS=1, IDE=ni+1, JDS=1, JDE=nj+1, KDS=1, KDE=nz+1, IMS=1, IME=ni, JMS=1, JME=nj, KMS=1, KME=nz, &
                ITS=1, ITE=ni, JTS=1, JTE=nj, KTS=1, KTE=nz)
    end if
    write(20) th, q, qc, qr, qi, qs, qg, qh, nn, nc, nr, re_c, re_i, re_s, rain, rainncv, snow, snowncv, graupel, &
              graupelncv, hail2, hailncv, sr
  end do
  close(20)
end program driver_wdm
