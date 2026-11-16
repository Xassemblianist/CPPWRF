! SPDX-License-Identifier: GPL-3.0-or-later
! WRF v4.8.0's MFSHCONVPBL (module_bl_mfshconvpbl.o of the -r8 build) on the tile of generate_inputs.py, as
! module_pbl_driver.F calls it (KRR = 2, the EDKF arrays present; the tendencies start at 7 to show what it assigns).
! Writes RTHBLTEN, RQVBLTEN, RQCBLTEN, RUBLTEN, RVBLTEN, RC_MF, MASSFLUX_EDKF, ENTR_EDKF, THL_UP, RT_UP, FRAC_UP
! (levels 1..nz; i fastest, then j, then k), in driver_mf.cpp's order.
program driver_mf_fortran
  use module_bl_mfshconvpbl, only: mfshconvpbl
  implicit none
  character(len=256) :: infile, outfile
  integer :: nx, ny, nz, stepbl, i, j, k, m, iunit
  real(8) :: dt
  real(8), allocatable, dimension(:,:,:) :: dz, rho, pmid, th, exner, qv, qc, u, v, tke, rc_mf, rthblten, &
       rqvblten, rqcblten, rublten, rvblten, emf, entr, detr, thl_up, thv_up, rt_up, rv_up, rc_up, u_up, v_up, &
       frac_up, wthv, plm
  real(8), allocatable, dimension(:,:) :: hfx, qfx, ht
  call get_command_argument(1, infile)
  call get_command_argument(2, outfile)
  open(newunit=iunit, file=trim(infile), status='old', access='stream', form='unformatted', convert='little_endian')
  read(iunit) nx, ny, nz, stepbl
  read(iunit) dt
  allocate(dz(nx,nz+1,ny), rho(nx,nz+1,ny), pmid(nx,nz+1,ny), th(nx,nz+1,ny), exner(nx,nz+1,ny), &
           qv(nx,nz+1,ny), qc(nx,nz+1,ny), u(nx,nz+1,ny), v(nx,nz+1,ny), tke(nx,nz+1,ny), rc_mf(nx,nz+1,ny), &
           rthblten(nx,nz+1,ny), rqvblten(nx,nz+1,ny), rqcblten(nx,nz+1,ny), rublten(nx,nz+1,ny), &
           rvblten(nx,nz+1,ny), emf(nx,nz+1,ny), entr(nx,nz+1,ny), detr(nx,nz+1,ny), thl_up(nx,nz+1,ny), &
           thv_up(nx,nz+1,ny), rt_up(nx,nz+1,ny), rv_up(nx,nz+1,ny), rc_up(nx,nz+1,ny), u_up(nx,nz+1,ny), &
           v_up(nx,nz+1,ny), frac_up(nx,nz+1,ny), wthv(nx,nz+1,ny), plm(nx,nz+1,ny), hfx(nx,ny), qfx(nx,ny), &
           ht(nx,ny))
  dz = 1; rho = 1; pmid = 1; th = 1; exner = 1; qv = 0; qc = 0; u = 0; v = 0; tke = 1; rc_mf = 0
  rthblten = 7; rqvblten = 7; rqcblten = 7; rublten = 7; rvblten = 7
  call rd3(dz); call rd3(rho); call rd3(pmid); call rd3(th); call rd3(exner); call rd3(qv); call rd3(qc)
  call rd3(u); call rd3(v); call rd3(tke); call rd3(rc_mf)
  read(iunit) hfx, qfx, ht
  close(iunit)
  call mfshconvpbl(DT=dt, STEPBL=stepbl, HT=ht, DZ=dz, RHO=rho, PMID=pmid, PINT=pmid, TH=th, EXNER=exner, &
       QV=qv, QC=qc, U=u, V=v, HFX=hfx, QFX=qfx, TKE=tke, RUBLTEN=rublten, RVBLTEN=rvblten, RTHBLTEN=rthblten, &
       RQVBLTEN=rqvblten, RQCBLTEN=rqcblten, IDS=1, IDE=nx+1, JDS=1, JDE=ny+1, KDS=1, KDE=nz+1, &
       IMS=1, IME=nx, JMS=1, JME=ny, KMS=1, KME=nz+1, ITS=1, ITE=nx, JTS=1, JTE=ny, KTS=1, KTE=nz, KRR=2, &
       MASSFLUX_EDKF=emf, ENTR_EDKF=entr, DETR_EDKF=detr, THL_UP=thl_up, THV_UP=thv_up, RT_UP=rt_up, &
       RV_UP=rv_up, RC_UP=rc_up, U_UP=u_up, V_UP=v_up, FRAC_UP=frac_up, RC_MF=rc_mf, WTHV=wthv, PLM_BL89=plm)
  open(newunit=iunit, file=trim(outfile), status='replace', access='stream', form='unformatted', &
       convert='little_endian')
  call wr3(rthblten); call wr3(rqvblten); call wr3(rqcblten); call wr3(rublten); call wr3(rvblten)
  call wr3(rc_mf); call wr3(emf); call wr3(entr); call wr3(thl_up); call wr3(rt_up); call wr3(frac_up)
  close(iunit)
contains
  subroutine rd3(x)
    real(8), intent(inout) :: x(:,:,:)
    read(iunit) (((x(i,k,j), i=1,nx), j=1,ny), k=1,nz)
  end subroutine rd3
  subroutine wr3(x)
    real(8), intent(in) :: x(:,:,:)
    write(iunit) (((x(i,k,j), i=1,nx), j=1,ny), k=1,nz)
  end subroutine wr3
end program driver_mf_fortran
