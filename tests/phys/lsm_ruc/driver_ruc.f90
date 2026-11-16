! SPDX-License-Identifier: GPL-3.0-or-later
! WRF v4.8.0's RUCLSMINIT, LSMRUC and SFCDIAGS_RUCLSM (module_sf_ruclsm.o, module_sf_sfcdiags_ruclsm.o of the -r8
! build) on the points of generate_inputs.py, as module_physics_init / module_surface_driver call them (one row,
! kms = kme = 1: the lowest level of the 3-D fields; nlcat = 28 (SOILVEGIN loops over the categories); spp_lsm = 0, lakemodel = 0, mosaic_lu = mosaic_soil = 0).
program driver_ruc_fortran
  use module_sf_ruclsm, only: ruclsminit, lsmruc
  use module_sf_sfcdiags_ruclsm, only: sfcdiags_ruclsm
  implicit none
  character(len=256) :: infile, outfile
  character(len=32) :: mminlu
  integer :: n, nsl, nsteps, i, k, iu, ou, step, iswater, isice, imm
  real(8) :: hdr(11), dt, xice_threshold
  logical :: myj, frpcpn, rdlai2d
  real(8), parameter :: cp = 7.*287./2., r_d = 287., rcp = r_d/cp, g = 9.81, xlv = 2.5e6, stbolt = 5.67051e-8
  real(8), allocatable :: zs(:)
  integer, allocatable :: ivgtyp(:,:), isltyp(:,:)
  real(8), allocatable, dimension(:,:) :: xland, xice, tbot, shdmin, shdmax, albbck, vegfra, snow, snowh, snowc, &
       canwat, snoalb, alb, emiss, lai, mavail, tsk, soilt1, tsnav, qvg, qcg, qsg, hfx, qfx, lh, znt, z0, sfcexc, &
       sfcevp, sfcrunoff, udrunoff, acrunoff, grdflx, acsnow, acsnom, dew, qsfc, chklowq, smstav, smstot, rhosnf, &
       precipfr, snowfallac, rainbl, rainncv, snowncv, graupelncv, sr, glw, gsw, flhc, flqc, chs, cpm, cqs2, chs2, &
       psfc, cqs, t2, th2, q2, lakemask
  real(8), allocatable, dimension(:,:,:) :: smois, tslb, sh2o, smfr3d, keepfr, t3d, qv3d, qc3d, p8w, rho3d, dz8w, &
       landusef, soilctop, spp
  call get_command_argument(1, infile)
  call get_command_argument(2, outfile)
  open(newunit=iu, file=trim(infile), status='old', access='stream', form='unformatted', convert='little_endian')
  read(iu) hdr
  n = nint(hdr(1)); nsl = nint(hdr(2)); nsteps = nint(hdr(3)); myj = hdr(4) > 0.5; frpcpn = hdr(5) > 0.5
  rdlai2d = hdr(6) > 0.5; imm = nint(hdr(7)); dt = hdr(8); xice_threshold = hdr(9); iswater = nint(hdr(10))
  isice = nint(hdr(11))
  mminlu = 'USGS'
  if (imm == 2) mminlu = 'MODIFIED_IGBP_MODIS_NOAH'
  allocate(zs(nsl), ivgtyp(n,1), isltyp(n,1))
  allocate(xland(n,1), xice(n,1), tbot(n,1), shdmin(n,1), shdmax(n,1), albbck(n,1), vegfra(n,1), snow(n,1), &
       snowh(n,1), snowc(n,1), canwat(n,1), snoalb(n,1), alb(n,1), emiss(n,1), lai(n,1), mavail(n,1), tsk(n,1), &
       soilt1(n,1), tsnav(n,1), qvg(n,1), qcg(n,1), qsg(n,1), hfx(n,1), qfx(n,1), lh(n,1), znt(n,1), z0(n,1), &
       sfcexc(n,1), sfcevp(n,1), sfcrunoff(n,1), udrunoff(n,1), acrunoff(n,1), grdflx(n,1), acsnow(n,1), &
       acsnom(n,1), dew(n,1), qsfc(n,1), chklowq(n,1), smstav(n,1), smstot(n,1), rhosnf(n,1), precipfr(n,1), &
       snowfallac(n,1), rainbl(n,1), rainncv(n,1), snowncv(n,1), graupelncv(n,1), sr(n,1), glw(n,1), gsw(n,1), &
       flhc(n,1), flqc(n,1), chs(n,1), cpm(n,1), cqs2(n,1), chs2(n,1), psfc(n,1), cqs(n,1), t2(n,1), th2(n,1), &
       q2(n,1), lakemask(n,1))
  allocate(smois(n,nsl,1), tslb(n,nsl,1), sh2o(n,nsl,1), smfr3d(n,nsl,1), keepfr(n,nsl,1), t3d(n,1,1), &
       qv3d(n,1,1), qc3d(n,1,1), p8w(n,1,1), rho3d(n,1,1), dz8w(n,1,1), landusef(n,28,1), soilctop(n,1,1), &
       spp(n,nsl,1))
  read(iu) zs
  call rd2i(ivgtyp); call rd2i(isltyp)
  call rd2(xland); call rd2(xice); call rd2(tbot); call rd2(shdmin); call rd2(shdmax); call rd2(albbck)
  call rd2(vegfra)
  call rd2(snow); call rd2(snowh); call rd2(snowc); call rd2(canwat); call rd2(snoalb); call rd2(alb)
  call rd2(emiss); call rd2(lai); call rd2(mavail); call rd2(tsk); call rd2(soilt1); call rd2(tsnav); call rd2(qvg)
  call rd2(qcg); call rd2(qsg); call rd2(hfx); call rd2(qfx); call rd2(lh); call rd2(znt)
  read(iu) ((smois(i,k,1), i=1,n), k=1,nsl)
  read(iu) ((tslb(i,k,1), i=1,n), k=1,nsl)
  z0 = znt; sfcexc = 0; sfcevp = 0; sfcrunoff = 0; udrunoff = 0; acrunoff = 0; grdflx = 0; acsnow = 0; acsnom = 0
  dew = 0; qsfc = 0; chklowq = 0; smstav = 0; smstot = 0; rhosnf = 0; precipfr = 0; snowfallac = 0
  sh2o = 0; smfr3d = 0; keepfr = 0; landusef = 0; soilctop = 0; spp = 0; lakemask = 0
  open(newunit=ou, file=trim(outfile), status='replace', access='stream', form='unformatted', convert='little_endian')
  call ruclsminit(sh2o, smfr3d, tslb, smois, isltyp, ivgtyp, mminlu, xice, mavail, nsl, iswater, isice, znt, &
       .false., .true., 1, n+1, 1, 2, 1, 2, 1, n, 1, 1, 1, 1, 1, n, 1, 1, 1, 1)
  write(ou) znt, mavail, sh2o, smfr3d
  do step = 1, nsteps
    call rd2(rainbl); call rd2(rainncv); call rd2(snowncv); call rd2(graupelncv); call rd2(sr); call rd2(glw)
    call rd2(gsw); call rd2(flhc); call rd2(flqc); call rd2(chs)
    read(iu) t3d; read(iu) qv3d; read(iu) qc3d; read(iu) p8w; read(iu) rho3d; read(iu) dz8w
    call rd2(cpm); call rd2(cqs2); call rd2(chs2); call rd2(psfc)
    call lsmruc(0, spp, spp, dt, step, nsl, 0, lakemask, graupelncv, snowncv, rainncv, &
         zs, rainbl, snow, snowh, snowc, sr, frpcpn, rhosnf, precipfr, &
         dz8w, p8w, t3d, qv3d, qc3d, rho3d, glw, gsw, emiss, chklowq, chs, flqc, flhc, mavail, canwat, vegfra, alb, &
         znt, z0, snoalb, albbck, lai, mminlu, landusef, 28, 0, 0, soilctop, 1, &
         qsfc, qsg, qvg, qcg, dew, soilt1, tsnav, tbot, ivgtyp, isltyp, xland, iswater, isice, xice, xice_threshold, &
         cp, rcp, g, xlv, stbolt, smois, sh2o, smstav, smstot, tslb, tsk, hfx, qfx, lh, &
         sfcrunoff, udrunoff, acrunoff, sfcexc, sfcevp, grdflx, snowfallac, acsnow, acsnom, smfr3d, keepfr, &
         myj, shdmin, shdmax, rdlai2d, 1, n+1, 1, 2, 1, 2, 1, n, 1, 1, 1, 1, 1, n, 1, 1, 1, 1)
    do i = 1, n
      cqs(i,1) = flqc(i,1)/(mavail(i,1)*rho3d(i,1,1))
      chs(i,1) = flhc(i,1)/(cpm(i,1)*rho3d(i,1,1))
    end do
    call sfcdiags_ruclsm(hfx, qfx, tsk, qsfc, cqs, cqs2, chs, chs2, t2, th2, q2, t3d, qv3d, rho3d, p8w, psfc, snow, &
         cp, r_d, rcp, 1, n+1, 1, 2, 1, 2, 1, n, 1, 1, 1, 1, 1, n, 1, 1, 1, 1)
    write(ou) snow, snowh, snowc, canwat, snoalb, alb, emiss, lai, mavail, tsk, soilt1, tsnav, qvg, qcg, qsg, hfx, &
         qfx, lh, znt, vegfra, z0, sfcexc, sfcevp, sfcrunoff, udrunoff, acrunoff, grdflx, acsnom, dew, qsfc, chklowq, &
         smstav, smstot, rhosnf, precipfr, snowfallac, cqs, chs, t2, th2, q2
    write(ou) smois, sh2o, tslb, smfr3d, keepfr
  end do
  close(iu); close(ou)
contains
  subroutine rd2(a)
    real(8), intent(out) :: a(:,:)
    read(iu) a
  end subroutine rd2
  subroutine rd2i(a)
    integer, intent(out) :: a(:,:)
    real(8) :: b(size(a,1),1)
    read(iu) b
    a = nint(b)
  end subroutine rd2i
end program driver_ruc_fortran
