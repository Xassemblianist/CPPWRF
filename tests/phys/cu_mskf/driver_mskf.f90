! SPDX-License-Identifier: GPL-3.0-or-later
! WRF v4.8.0's MSKF_CPS (module_cu_mskf.o of the -r8 build) on the columns of generate_inputs.py, called as
! module_cumulus_driver.F does (aercu_opt = 0, kf_edrates = 0, ADAPT_STEP_FLAG = .false.); one row of ncols columns.
!   driver_mskf IN OUT
PROGRAM driver_mskf
  USE module_cu_mskf, ONLY: mskf_cps, mskf_init
  IMPLICIT NONE
  INTEGER :: u_in, u_out, ncols, nl, stepcu, ktau, trigger, iwarm, iqi, iqs, iqr
  REAL*8 :: dt, dx, cudt
  LOGICAL :: warm_rain, f_qi, f_qs, f_qr, adapt
  REAL*8, ALLOCATABLE, DIMENSION(:,:,:) :: u, v, th, t, qv, dz8w, pcps, rho, pi, w, w0avg, &
       rthcuten, rqvcuten, rqccuten, rqrcuten, rqicuten, rqscuten, rqvften, rucuten, rvcuten, &
       cldfra_dp, cldfra_sh, qc_kf, qi_kf, qr_kf, qs_kf, nc_kf, ni_kf, nr_kf, ns_kf, ccn_kf, efcs, efis, efss, &
       udr_kf, ddr_kf, uer_kf, der_kf
  REAL*8, ALLOCATABLE, DIMENSION(:,:) :: raincv, pratec, nca, cutop, cubot, hfx, ust, pblh, zol, xland, &
       ainc_frac, timec_kf
  LOGICAL, ALLOCATABLE :: cu_act_flag(:,:)
  CHARACTER(LEN=512) :: fin, fout

  CALL get_command_argument(1, fin)
  CALL get_command_argument(2, fout)
  OPEN(NEWUNIT=u_in, FILE=TRIM(fin), FORM="unformatted", ACCESS="stream", STATUS="old")
  READ(u_in) ncols, nl, dt, dx, cudt, stepcu, ktau, trigger, iwarm, iqi, iqs, iqr
  warm_rain = iwarm /= 0
  f_qi = iqi /= 0
  f_qs = iqs /= 0
  f_qr = iqr /= 0
  adapt = .FALSE.

  ALLOCATE(u(ncols,nl+1,1), v(ncols,nl+1,1), th(ncols,nl+1,1), t(ncols,nl+1,1), qv(ncols,nl+1,1), &
           dz8w(ncols,nl+1,1), pcps(ncols,nl+1,1), rho(ncols,nl+1,1), pi(ncols,nl+1,1), w(ncols,nl+1,1), &
           w0avg(ncols,nl+1,1), rthcuten(ncols,nl+1,1), rqvcuten(ncols,nl+1,1), rqccuten(ncols,nl+1,1), &
           rqrcuten(ncols,nl+1,1), rqicuten(ncols,nl+1,1), rqscuten(ncols,nl+1,1), rqvften(ncols,nl+1,1), &
           rucuten(ncols,nl+1,1), rvcuten(ncols,nl+1,1), cldfra_dp(ncols,nl+1,1), cldfra_sh(ncols,nl+1,1), &
           qc_kf(ncols,nl+1,1), qi_kf(ncols,nl+1,1), qr_kf(ncols,nl+1,1), qs_kf(ncols,nl+1,1), &
           nc_kf(ncols,nl+1,1), ni_kf(ncols,nl+1,1), nr_kf(ncols,nl+1,1), ns_kf(ncols,nl+1,1), &
           ccn_kf(ncols,nl+1,1), efcs(ncols,nl+1,1), efis(ncols,nl+1,1), efss(ncols,nl+1,1), &
           udr_kf(ncols,nl+1,1), ddr_kf(ncols,nl+1,1), uer_kf(ncols,nl+1,1), der_kf(ncols,nl+1,1))
  ALLOCATE(raincv(ncols,1), pratec(ncols,1), nca(ncols,1), cutop(ncols,1), cubot(ncols,1), hfx(ncols,1), &
           ust(ncols,1), pblh(ncols,1), zol(ncols,1), xland(ncols,1), ainc_frac(ncols,1), timec_kf(ncols,1), &
           cu_act_flag(ncols,1))
  u = 0.; v = 0.; th = 0.; t = 0.; qv = 0.; dz8w = 0.; pcps = 0.; rho = 0.; pi = 0.; w = 0.; w0avg = 0.
  READ(u_in) u(:,1:nl,1)
  READ(u_in) v(:,1:nl,1)
  READ(u_in) th(:,1:nl,1)
  READ(u_in) t(:,1:nl,1)
  READ(u_in) qv(:,1:nl,1)
  READ(u_in) dz8w(:,1:nl,1)
  READ(u_in) pcps(:,1:nl,1)
  READ(u_in) rho(:,1:nl,1)
  READ(u_in) pi(:,1:nl,1)
  READ(u_in) w0avg(:,1:nl,1)
  READ(u_in) w(:,1:nl+1,1)
  READ(u_in) raincv(:,1)
  READ(u_in) pratec(:,1)
  READ(u_in) nca(:,1)
  READ(u_in) cutop(:,1)
  READ(u_in) cubot(:,1)
  READ(u_in) hfx(:,1)
  READ(u_in) ust(:,1)
  READ(u_in) pblh(:,1)
  READ(u_in) zol(:,1)
  READ(u_in) xland(:,1)
  CLOSE(u_in)

  ! mskf_init (restart = .false.): the tendencies, W0AVG of the input kept below; it calls mskf_lutab
  rthcuten = 0.; rqvcuten = 0.; rqccuten = 0.; rqrcuten = 0.; rqicuten = 0.; rqscuten = 0.; rqvften = 0.
  rucuten = 0.; rvcuten = 0.
  cldfra_dp = 0.; cldfra_sh = 0.; qc_kf = 0.; qi_kf = 0.; qr_kf = 0.; qs_kf = 0.; nc_kf = 0.; ni_kf = 0.
  nr_kf = 0.; ns_kf = 0.; ccn_kf = 0.; efcs = 0.; efis = 0.; efss = 0.; udr_kf = 0.; ddr_kf = 0.; uer_kf = 0.
  der_kf = 0.; ainc_frac = 0.; timec_kf = 0.
  CALL mskf_init_tables()

  CALL MSKF_CPS(ids=1, ide=ncols, jds=1, jde=1, kds=1, kde=nl,                 &
       ims=1, ime=ncols, jms=1, jme=1, kms=1, kme=nl+1,                          &
       its=1, ite=ncols, jts=1, jte=1, kts=1, kte=nl,                            &
       trigger=trigger, DT=dt, KTAU=ktau, DX=dx, CUDT=cudt, ADAPT_STEP_FLAG=adapt, &
       rho=rho, RAINCV=raincv, PRATEC=pratec, NCA=nca,                           &
       U=u, V=v, TH=th, T=t, W=w, dz8w=dz8w, Pcps=pcps, pi=pi, W0AVG=w0avg,       &
       XLV0=3.15E6, XLV1=2370.E0, XLS0=2.905E6, XLS1=259.532E0, CP=1004.5E0, R=287.E0, G=9.81E0, &
       EP1=461.6E0/287.E0-1.E0, EP2=287.E0/461.6E0, SVP1=0.6112E0, SVP2=17.67E0, SVP3=29.65E0, &
       SVPT0=273.15E0, STEPCU=stepcu, CU_ACT_FLAG=cu_act_flag, warm_rain=warm_rain, &
       CUTOP=cutop, CUBOT=cubot, QV=qv,                                          &
       F_QV=.TRUE., F_QC=.TRUE., F_QR=f_qr, F_QI=f_qi, F_QS=f_qs,                &
       RTHCUTEN=rthcuten, RQVCUTEN=rqvcuten, RQCCUTEN=rqccuten, RQRCUTEN=rqrcuten, &
       RQICUTEN=rqicuten, RQSCUTEN=rqscuten, RQVFTEN=rqvften,                    &
       cldfra_dp_KF=cldfra_dp, cldfra_sh_KF=cldfra_sh,                           &
       qc_KF=qc_kf, qi_KF=qi_kf, qr_KF=qr_kf, qs_KF=qs_kf,                       &
       nc_KF=nc_kf, ni_KF=ni_kf, nr_KF=nr_kf, ns_KF=ns_kf, ccn_KF=ccn_kf, ainc_frac=ainc_frac, &
       UDR_KF=udr_kf, DDR_KF=ddr_kf, UER_KF=uer_kf, DER_KF=der_kf, TIMEC_KF=timec_kf, KF_EDRATES=0, &
       ZOL=zol, HFX=hfx, UST=ust, PBLH=pblh,                                     &
       no_src_types_cu=1, aercu_fct=1.E0, aercu_opt=0,                           &
       EFCS=efcs, EFIS=efis, EFSS=efss, RUCUTEN=rucuten, RVCUTEN=rvcuten, XLAND=xland)

  OPEN(NEWUNIT=u_out, FILE=TRIM(fout), FORM="unformatted", ACCESS="stream", STATUS="replace")
  WRITE(u_out) raincv(:,1)
  WRITE(u_out) pratec(:,1)
  WRITE(u_out) nca(:,1)
  WRITE(u_out) cutop(:,1)
  WRITE(u_out) cubot(:,1)
  WRITE(u_out) w0avg(:,1:nl,1)
  WRITE(u_out) rthcuten(:,1:nl,1)
  WRITE(u_out) rqvcuten(:,1:nl,1)
  WRITE(u_out) rqccuten(:,1:nl,1)
  WRITE(u_out) rqrcuten(:,1:nl,1)
  WRITE(u_out) rqicuten(:,1:nl,1)
  WRITE(u_out) rqscuten(:,1:nl,1)
  WRITE(u_out) rucuten(:,1:nl,1)
  WRITE(u_out) rvcuten(:,1:nl,1)
  WRITE(u_out) cldfra_dp(:,1:nl,1)
  WRITE(u_out) cldfra_sh(:,1:nl,1)
  WRITE(u_out) qc_kf(:,1:nl,1)
  WRITE(u_out) qi_kf(:,1:nl,1)
  CLOSE(u_out)
CONTAINS
  SUBROUTINE mskf_init_tables()
    ! mskf_init with restart = .true. only builds the lookup tables (MSKF_LUTAB), leaving the arrays alone
    REAL*8 :: d3(ncols, nl+1, 1), d2(ncols, 1)
    CALL mskf_init(d3, d3, d3, d3, d3, d3, d2, d3, 0, 0, 0.6112E0, 17.67E0, 29.65E0, 273.15E0, &
         1, .TRUE., .TRUE., 1, ncols, 1, 1, 1, nl, 1, ncols, 1, 1, 1, nl+1, 1, ncols, 1, 1, 1, nl, d3, d3)
  END SUBROUTINE mskf_init_tables
END PROGRAM driver_mskf
