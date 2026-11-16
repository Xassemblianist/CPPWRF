! SPDX-License-Identifier: GPL-3.0-or-later
! Grell-3D (cu_physics = 5) patch test, the WRF side: G3DRV and CONV_GRELL_SPREAD3D of WRF v4.8.0's own
! module_cu_g3.o (the -r8 build, WRF_DIR) on an nx x ny patch (one tile, as a serial wrf.exe), the arguments the
! cumulus driver passes (RTHFTEN already (RTHFTEN + RTHRATEN + RTHBLTEN) * pi, as module_cumulus_driver.F:867).
!   driver_g3_fortran IN OUT   (IN: generate_inputs.py or the model's CPPWRF_G3_DUMP; OUT: see the writes below)
program driver_g3
  use module_cu_g3, only : g3drv, conv_grell_spread3d
  use module_model_constants, only : cp, xlv, g, r_v
  implicit none
  integer, parameter :: ensdim = 144
  integer :: nx, ny, nz, avedx, ishallow, ichoice, ifqc, ifqi, u, i, j, k, n
  real :: dt, dx
  real, allocatable, dimension(:,:,:) :: uu, vv, t, q, p, pi, rho, rthften, rqvften, rthblten, rqvblten, w, p8w, &
       rthcuten, rqvcuten, rqccuten, rqicuten, gdc, gdc2, cugd_tten, cugd_qvten, cugd_ttens, cugd_qvtens, cugd_qcten, &
       xf_ens, pr_ens
  real, allocatable, dimension(:,:) :: ht, xland, gsw, raincv, pratec, mass_flux, apr_gr, apr_w, apr_mc, apr_st, &
       apr_as, apr_capma, apr_capme, apr_capmi, edt_out, htop, hbot, xmb_shallow, buf2
  integer, allocatable, dimension(:,:) :: kpbl, k22_shallow, kbcon_shallow, ktop_shallow, ktop_deep
  logical, allocatable :: cu_act_flag(:,:)
  logical :: fqc, fqi
  character(len=512) :: fin, fout
  integer :: ids, ide, jds, jde, kds, kde, ims, ime, jms, jme, kms, kme, its, ite, jts, jte, kts, kte

  call get_command_argument(1, fin)
  call get_command_argument(2, fout)
  open(newunit=u, file=trim(fin), access='stream', form='unformatted', status='old', convert='little_endian')
  read(u) nx, ny, nz, avedx, ishallow, ichoice, ifqc, ifqi
  read(u) dt, dx
  fqc = ifqc /= 0
  fqi = ifqi /= 0
  ids = 1; ide = nx + 1; jds = 1; jde = ny + 1; kds = 1; kde = nz + 1
  ims = ids - 3; ime = ide + 3; jms = jds - 3; jme = jde + 3; kms = 1; kme = nz + 1
  its = 1; ite = nx; jts = 1; jte = ny; kts = 1; kte = nz
  allocate(t(ims:ime,kms:kme,jms:jme))
  allocate(uu, vv, q, p, pi, rho, rthften, rqvften, rthblten, rqvblten, w, p8w, rthcuten, rqvcuten, rqccuten, &
           rqicuten, gdc, gdc2, cugd_tten, cugd_qvten, cugd_ttens, cugd_qvtens, cugd_qcten, mold=t)
  allocate(xf_ens(ims:ime,jms:jme,ensdim), pr_ens(ims:ime,jms:jme,ensdim))
  allocate(ht(ims:ime,jms:jme))
  allocate(xland, gsw, raincv, pratec, mass_flux, apr_gr, apr_w, apr_mc, apr_st, apr_as, apr_capma, apr_capme, &
           apr_capmi, edt_out, htop, hbot, xmb_shallow, mold=ht)
  allocate(kpbl(ims:ime,jms:jme), k22_shallow(ims:ime,jms:jme), kbcon_shallow(ims:ime,jms:jme), &
           ktop_shallow(ims:ime,jms:jme), ktop_deep(ims:ime,jms:jme), cu_act_flag(ims:ime,jms:jme))
  allocate(buf2(nx, ny))
  t = 0; uu = 0; vv = 0; q = 0; p = 0; pi = 1; rho = 0; rthften = 0; rqvften = 0; rthblten = 0; rqvblten = 0
  w = 0; p8w = 0; rthcuten = 0; rqvcuten = 0; rqccuten = 0; rqicuten = 0; gdc = 0; gdc2 = 0
  cugd_tten = 0; cugd_qvten = 0; cugd_ttens = 0; cugd_qvtens = 0; cugd_qcten = 0; xf_ens = 0; pr_ens = 0
  raincv = 0; pratec = 0; mass_flux = 0; edt_out = 0; htop = 0; hbot = 0; xmb_shallow = 0
  k22_shallow = 0; kbcon_shallow = 0; ktop_shallow = 0; ktop_deep = 0
  call rd3(uu, nz); call rd3(vv, nz); call rd3(t, nz); call rd3(q, nz); call rd3(p, nz); call rd3(pi, nz)
  call rd3(rho, nz); call rd3(rthften, nz); call rd3(rqvften, nz); call rd3(rthblten, nz); call rd3(rqvblten, nz)
  call rd3(w, nz + 1); call rd3(p8w, nz + 1)
  call rd2(ht); call rd2(xland); call rd2(gsw)
  call rd2(buf2(:,:)); kpbl = 1; kpbl(1:nx,1:ny) = nint(buf2)
  call rd2(apr_gr); call rd2(apr_w); call rd2(apr_mc); call rd2(apr_st); call rd2(apr_as); call rd2(apr_capma)
  call rd2(apr_capme); call rd2(apr_capmi)
  close(u)
  ! the halo: copies of the outermost row (only the 4-row boundary columns ever read it)
  call halo3(rthften); call halo3(rqvften); call halo3(w)

  call g3drv(dt=dt, itimestep=2, dx=dx, rho=rho, raincv=raincv, pratec=pratec, u=uu, v=vv, t=t, w=w, q=q, p=p, &
       pi=pi, dz8w=rho, p8w=p8w, xlv=xlv, cp=cp, g=g, r_v=r_v, htop=htop, hbot=hbot, cu_act_flag=cu_act_flag, &
       warm_rain=.false., apr_gr=apr_gr, apr_w=apr_w, apr_mc=apr_mc, apr_st=apr_st, apr_as=apr_as, &
       apr_capma=apr_capma, apr_capme=apr_capme, apr_capmi=apr_capmi, mass_flux=mass_flux, xf_ens=xf_ens, &
       pr_ens=pr_ens, ht=ht, xland=xland, gsw=gsw, edt_out=edt_out, gdc=gdc, gdc2=gdc2, kpbl=kpbl, &
       k22_shallow=k22_shallow, kbcon_shallow=kbcon_shallow, ktop_shallow=ktop_shallow, xmb_shallow=xmb_shallow, &
       ktop_deep=ktop_deep, cugd_tten=cugd_tten, cugd_qvten=cugd_qvten, cugd_qcten=cugd_qcten, &
       cugd_ttens=cugd_ttens, cugd_qvtens=cugd_qvtens, cugd_avedx=avedx, imomentum=0, ensdim=ensdim, maxiens=1, &
       maxens=3, maxens2=3, maxens3=16, ichoice=ichoice, ishallow_g3=ishallow, &
       ids=ids, ide=ide, jds=jds, jde=jde, kds=kds, kde=kde, ims=ims, ime=ime, jms=jms, jme=jme, kms=kms, kme=kme, &
       ips=its, ipe=ide, jps=jts, jpe=jde, kps=kts, kpe=kde, its=its, ite=ite, jts=jts, jte=jte, kts=kts, kte=kte, &
       periodic_x=.false., periodic_y=.false., rqvcuten=rqvcuten, rqccuten=rqccuten, rqicuten=rqicuten, &
       rqvften=rqvften, rthften=rthften, rthcuten=rthcuten, rqvblten=rqvblten, rthblten=rthblten, &
       f_qv=.true., f_qc=fqc, f_qr=.false., f_qi=fqi, f_qs=.false.)

  open(newunit=u, file=trim(fout), access='stream', form='unformatted', status='replace', convert='little_endian')
  call wr3(cugd_tten); call wr3(cugd_qvten); call wr3(cugd_ttens); call wr3(cugd_qvtens); call wr3(cugd_qcten)
  call wr3(rqccuten); call wr3(rqicuten); call wr3(gdc); call wr3(gdc2)
  call wr2(raincv); call wr2(pratec); call wr2(htop); call wr2(hbot); call wr2(edt_out); call wr2(xmb_shallow)
  call wr2(real(k22_shallow)); call wr2(real(kbcon_shallow)); call wr2(real(ktop_shallow)); call wr2(real(ktop_deep))
  call wr2(apr_gr); call wr2(apr_w); call wr2(apr_mc); call wr2(apr_st); call wr2(apr_as); call wr2(apr_capma)
  call wr2(apr_capme); call wr2(apr_capmi)
  do n = 1, ensdim
    call wr2(xf_ens(:,:,n))
  enddo
  do n = 1, ensdim
    call wr2(pr_ens(:,:,n))
  enddo

  call conv_grell_spread3d(rthcuten=rthcuten, rqvcuten=rqvcuten, rqccuten=rqccuten, raincv=raincv, &
       cugd_avedx=avedx, cugd_tten=cugd_tten, cugd_qvten=cugd_qvten, rqicuten=rqicuten, cugd_ttens=cugd_ttens, &
       cugd_qvtens=cugd_qvtens, cugd_qcten=cugd_qcten, pi_phy=pi, moist_qv=q, pratec=pratec, dt=dt, num_tiles=1, &
       imomentum=0, f_qv=.true., f_qc=fqc, f_qr=.false., f_qi=fqi, f_qs=.false., &
       ids=ids, ide=ide, jds=jds, jde=jde, kds=kds, kde=kde, ips=its, ipe=ide, jps=jts, jpe=jde, kps=kts, kpe=kde, &
       ims=ims, ime=ime, jms=jms, jme=jme, kms=kms, kme=kme, its=its, ite=ite, jts=jts, jte=jte, kts=kts, kte=kte)
  call wr3(rthcuten); call wr3(rqvcuten); call wr3(rqccuten); call wr3(rqicuten)
  call wr2(raincv); call wr2(pratec)
  close(u)

contains
  subroutine rd3(a, nk)
    real, intent(inout) :: a(ims:ime,kms:kme,jms:jme)
    integer, intent(in) :: nk
    do j = 1, ny
      do k = 1, nk
        read(u) (a(i,k,j), i = 1, nx)
      enddo
    enddo
  end subroutine rd3
  subroutine rd2(a)
    real, intent(inout) :: a(:,:)
    real :: b(nx, ny)
    read(u) b
    if (size(a, 1) == nx) then
      a = b
    else
      a(1-ims+1:nx-ims+1, 1-jms+1:ny-jms+1) = b
    endif
  end subroutine rd2
  subroutine halo3(a)
    real, intent(inout) :: a(ims:ime,kms:kme,jms:jme)
    do i = ims, 0
      a(i,:,1:ny) = a(1,:,1:ny)
    enddo
    do i = nx + 1, ime
      a(i,:,1:ny) = a(nx,:,1:ny)
    enddo
    do j = jms, 0
      a(:,:,j) = a(:,:,1)
    enddo
    do j = ny + 1, jme
      a(:,:,j) = a(:,:,ny)
    enddo
  end subroutine halo3
  subroutine wr3(a)
    real, intent(in) :: a(ims:ime,kms:kme,jms:jme)
    do j = 1, ny
      do k = 1, nz
        write(u) (a(i,k,j), i = 1, nx)
      enddo
    enddo
  end subroutine wr3
  subroutine wr2(a)
    real, intent(in) :: a(ims:ime,jms:jme)
    write(u) a(1:nx,1:ny)
  end subroutine wr2
end program driver_g3
