! SPDX-License-Identifier: GPL-3.0-or-later
! Vortex-following nest test, WRF side: compute_500mb_height of WRF v4.8.0's -r8 build (module_compute_geop.o in
! libwrflib.a) and the vortex search and move of time_for_move2 (share/mediation_nest_move.F:471-603, 653-697),
! which a wrf.exe without -DVORTEX_CENTER does not compile: wrf_search and wrf_decide below are those lines with
! grid%xi, grid%xj, grid%vc_i, grid%vc_j as arguments, the global copies as arrays and the messages left out.
! The sequence per input (generate_inputs.py): search on field set A (first time: vc = -1), move, move again
! (alarm not ringing), search on field set B (the box around vc), move.
!   driver_vortex_fortran INPUT OUTPUT
program driver_vortex
  use module_compute_geop, only: compute_500mb_height
  implicit none
  character(len=256) :: fin, fout
  real(8) :: h(8)
  integer :: nx, ny, nz, track_level, vortex_interval, max_vortex_speed, pgr, ids, ide, jds, jde, kds, kde, set, i, j
  real :: dx, xi, xj, vc_i, vc_j
  real, allocatable :: ph(:,:,:), phb(:,:,:), p(:,:,:), pb(:,:,:), psfc(:,:), ht(:,:), height(:,:), hg(:,:), &
                       pg(:,:), tg(:,:)
  call get_command_argument(1, fin)
  call get_command_argument(2, fout)
  open(10, file=fin, access='stream', form='unformatted', status='old')
  open(11, file=fout, access='stream', form='unformatted', status='replace')
  read(10) h
  nx = nint(h(1)); ny = nint(h(2)); nz = nint(h(3)); track_level = nint(h(4)); vortex_interval = nint(h(5))
  max_vortex_speed = nint(h(6)); pgr = nint(h(7)); dx = h(8)
  ids = 1; ide = nx + 1; jds = 1; jde = ny + 1; kds = 1; kde = nz + 1
  ! memory = the domain (ims:ime = ids:ide ...)
  allocate(ph(ids:ide,kds:kde,jds:jde), phb(ids:ide,kds:kde,jds:jde), p(ids:ide,kds:kde,jds:jde), &
           pb(ids:ide,kds:kde,jds:jde), psfc(ids:ide,jds:jde), ht(ids:ide,jds:jde), height(ids:ide,jds:jde), &
           hg(ids:ide,jds:jde), pg(ids:ide,jds:jde), tg(ids:ide,jds:jde))
  xi = -1.0; xj = -1.0; vc_i = -1.0; vc_j = -1.0   ! module_domain.F:798-801
  do set = 1, 2
    ph = 0.; phb = 0.; p = 0.; pb = 0.; psfc = 0.; ht = 0.; height = 0.
    read(10) ph(1:nx,1:nz+1,1:ny), phb(1:nx,1:nz+1,1:ny), p(1:nx,1:nz,1:ny), pb(1:nx,1:nz,1:ny), &
             psfc(1:nx,1:ny), ht(1:nx,1:ny)
    call compute_500mb_height(ph, phb, p, pb, height, track_level, &
                              ids, ide, jds, jde, kds, kde, ids, ide, jds, jde, kds, kde, ids, ide, jds, jde, kds, kde)
    write(11) ((real(height(i,j), 8), i = 1, nx), j = 1, ny)
    ! wrf_patch_to_global_real: the global copies
    hg = height; pg = psfc; tg = ht
    call wrf_search(hg, pg, tg, ids, ide, jds, jde, dx, vortex_interval, max_vortex_speed, xi, xj, vc_i, vc_j)
    call wrf_decide(ids, ide, jds, jde, pgr, vc_i, vc_j)
    if (set == 1) call wrf_decide(ids, ide, jds, jde, pgr, vc_i, vc_j)
  end do
  close(11)
contains

  subroutine put(vals)
    real(8), intent(in) :: vals(:)
    write(11) vals
  end subroutine put

  subroutine wrf_search(height, psfc, terrain, ids, ide, jds, jde, dx, vortex_interval, max_vortex_speed, &
                        xi, xj, vc_i, vc_j)
    integer, intent(in) :: ids, ide, jds, jde, vortex_interval, max_vortex_speed
    real, intent(inout) :: height(ids:ide,jds:jde), psfc(ids:ide,jds:jde), terrain(ids:ide,jds:jde)
    real, intent(in) :: dx
    real, intent(inout) :: xi, xj, vc_i, vc_j
    INTEGER :: is, ie, js, je, i, j, imploc, jmploc, fje, fjs, fie, fis, fimloc, fjmloc, imloc, jmloc, maxi, maxj
    REAL    :: ipbar, pbar, jpbar, fact, dijsmooth, vmax, vmin, a, b, pmin
    REAL    :: rsmooth = 100000.  ! in meters

       IF ( vc_i < 0. .AND. vc_j < 0. ) THEN
          ! first time through
          is = ids
          ie = ide-1
          js = jds
          je = jde-1
       ELSE
          is = max( vc_i - 60 * vortex_interval * max_vortex_speed / dx , 1.0 * ids )
          js = max( vc_j - 60 * vortex_interval * max_vortex_speed / dx , 1.0 * jds )
          ie = min( vc_i + 60 * vortex_interval * max_vortex_speed / dx , 1.0 * (ide-1) )
          je = min( vc_j + 60 * vortex_interval * max_vortex_speed / dx , 1.0 * (jde-1) )
       ENDIF

       imploc = -1
       jmploc = -1

       ! find minimum psfc
       pmin = 99999999.0     ! make this very large to be sure we find a minumum
       DO j = js, je
       DO i = is, ie
         ! adjust approximately to sea level pressure (same as below: ATCF)
         psfc(i,j)=psfc(i,j)+11.38*terrain(i,j)
         IF ( psfc(i,j) .LT. pmin ) THEN
           pmin = psfc(i,j)
           imploc = i
           jmploc = j
         ENDIF
       ENDDO
       ENDDO

       IF ( imploc .EQ. -1 .OR. jmploc .EQ. -1 ) STOP 'time_for_move2: Method failure searching for minimum psfc.'

       imloc = -1
       jmloc = -1
       maxi = -1
       maxj = -1

       ! find local min, max
       vmin =  99999999.0
       vmax = -99999999.0
       DO j = js, je
       DO i = is, ie
         IF ( height(i,j) .LT. vmin ) THEN
           vmin = height(i,j)
           imloc = i
           jmloc = j
         ENDIF
         IF ( height(i,j) .GT. vmax ) THEN
           vmax = height(i,j)
           maxi = i
           maxj = j
         ENDIF
       ENDDO
       ENDDO

       IF ( imloc .EQ. -1 .OR. jmloc .EQ. -1 .OR. maxi .EQ. -1 .OR. maxj .EQ. -1 ) &
         STOP 'time_for_move2: Method failure searching max/min of height.'

       fimloc = imloc
       fjmloc = jmloc

       if ( xi .EQ. -1. ) xi = fimloc
       if ( xj .EQ. -1. ) xj = fjmloc

       dijsmooth = rsmooth / dx

       fjs = max(fjmloc-dijsmooth,1.0)
       fje = min(fjmloc+dijsmooth,jde-2.0)
       fis = max(fimloc-dijsmooth,1.0)
       fie = min(fimloc+dijsmooth,ide-2.0)
       js = fjs
       je = fje
       is = fis
       ie = fie

       vmin =  1000000.0
       vmax = -1000000.0
       DO j = js, je
       DO i = is, ie
         IF ( height(i,j) .GT. vmax ) THEN
           vmax = height(i,j)
         ENDIF
       ENDDO
       ENDDO

       pbar  = 0.0
       ipbar = 0.0
       jpbar = 0.0

       do j=js,je
          do i=is,ie
             fact = vmax - height(i,j)
             pbar  = pbar + fact
             ipbar = ipbar + fact*(i-is)
             jpbar = jpbar + fact*(j-js)
          enddo
       enddo

      IF ( pbar .NE. 0. ) THEN
         a = 0.0
         b = 1.0
         xi =  (a * xi + b * (is + ipbar / pbar))  / ( a + b )
         xj =  (a * xj + b * (js + jpbar / pbar))  / ( a + b )

         vc_i = xi + .5
         vc_j = xj + .5
      ENDIF
      call put([real(xi, 8), real(xj, 8), real(vc_i, 8), real(vc_j, 8), real(pmin, 8), real(imploc, 8), &
                real(jmploc, 8), real(imloc, 8), real(jmloc, 8), real(is, 8), real(ie, 8), real(js, 8), real(je, 8)])
  end subroutine wrf_search

  subroutine wrf_decide(ids, ide, jds, jde, pgr, vc_i, vc_j)
    integer, intent(in) :: ids, ide, jds, jde, pgr
    real, intent(inout) :: vc_i, vc_j
    INTEGER :: move_cd_x, move_cd_y, irad
    REAL :: disp_x, disp_y, radius, dc_i, dc_j

   dc_i = (ide-ids+1)/2.    ! domain center
   dc_j = (jde-jds+1)/2.

   disp_x = vc_i - dc_i * 1.0
   disp_y = vc_j - dc_j * 1.0

   irad = 6

   radius = irad

   if ( disp_x .GT. 0 ) disp_x = min( disp_x , radius )
   if ( disp_y .GT. 0 ) disp_y = min( disp_y , radius )

   if ( disp_x .LT. 0 ) disp_x = max( disp_x , -radius )
   if ( disp_y .LT. 0 ) disp_y = max( disp_y , -radius )

   move_cd_x = int ( disp_x  / pgr )
   move_cd_y = int ( disp_y  / pgr )

   IF ( move_cd_x .GT. 0 ) move_cd_x = min ( move_cd_x , 1 )
   IF ( move_cd_y .GT. 0 ) move_cd_y = min ( move_cd_y , 1 )
   IF ( move_cd_x .LT. 0 ) move_cd_x = max ( move_cd_x , -1 )
   IF ( move_cd_y .LT. 0 ) move_cd_y = max ( move_cd_y , -1 )

   vc_i = vc_i - move_cd_x * pgr
   vc_j = vc_j - move_cd_y * pgr
   call put([real(dc_i, 8), real(dc_j, 8), real(disp_x, 8), real(disp_y, 8), real(move_cd_x, 8), &
             real(move_cd_y, 8), real(vc_i, 8), real(vc_j, 8)])
  end subroutine wrf_decide
end program driver_vortex
