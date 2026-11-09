# CPPWRF hızlı başlangıç

Bu sayfa, GFS analizinden 3 saatlik bir tahminin haritasına dört adımda ulaşır: WPS, `cppwrf-realexe` (real.exe
yerine), `cppwrf-wrf` (wrf.exe yerine) ve bir çizim. Elinizde hazır bir WRF koşu dizini varsa yalnızca 3. adım
yenidir: `wrf.exe`'yi nerede çalıştırıyorsanız `cppwrf-wrf`'yi orada çalıştırın. İngilizcesi
[../QUICKSTART.md](../QUICKSTART.md)'dir.

## Gerekenler

- **Kurulu bir CPPWRF** ([KURULUM.md](KURULUM.md)), `bin/` dizini `PATH`'te olacak şekilde.
- **Derlenmiş bir WPS 4.6:** `geogrid.exe`, `ungrib.exe`, `metgrid.exe`.
- **WPS'in düşük çözünürlüklü coğrafi verisi.** WPS indirme sayfasındaki `geog_low_res_mandatory.tar.gz` dosyasını
  indirip açın.
- **WRF v4.8.0 kaynak ağacı:** yalnızca `run/` dizinindeki fizik tabloları için; WRF'yi derlemeniz gerekmez.
- **Python 3** (`netCDF4`, `numpy`; harita için `matplotlib`).
- **İnternet bağlantısı:** yaklaşık 500'er MB'lık iki GFS dosyası için.

## Tek komutla

```bash
export WPS_DIR=/yol/WPS WRF_DIR=/yol/WRF-4.8.0 GEOG_DIR=/yol/WPS_GEOG_LOW_RES
MODE=gpu-fp32 examples/quickstart/run.sh ~/hb        # ya da MODE=cpu-fp64, gpu-fp64
```

Vaka, WRF'nin kendi varsayılan vakasıdır:

- WPS'in varsayılan alanı: ABD'nin güneydoğusunda 15 km'lik bir alan ve içinde iki yönlü, 5 km'lik bir iç alan;
- WRF'nin `test/em_real/namelist.input` dosyası (`physics_suite = 'CONUS'`);
- 3 saatlik bir tahmin.

Aşağıdaki adımlar betiğin yaptıklarıdır; kendi vakanızda elle de uygulayabilirsiniz.

## 1. WPS

```bash
mkdir -p ~/hb/wps && cd ~/hb/wps
sed "s|GEOG_DIR|$GEOG_DIR|" $CPPWRF/examples/quickstart/namelist.wps > namelist.wps
ln -sfn $WPS_DIR/geogrid geogrid; ln -sfn $WPS_DIR/metgrid metgrid
ln -sf $WPS_DIR/ungrib/Variable_Tables/Vtable.GFS Vtable
for h in 000 003; do
  curl -sfo gfs.f$h https://noaa-gfs-bdp-pds.s3.amazonaws.com/gfs.20261012/00/atmos/gfs.t00z.pgrb2.0p25.f$h
done
$WPS_DIR/link_grib.csh gfs.f
$WPS_DIR/geogrid.exe && $WPS_DIR/ungrib.exe && $WPS_DIR/metgrid.exe
```

Bu adımda değişen bir şey yok; CPPWRF, WPS'in yerini almaz. Başka bir tarih için her iki namelist'teki tarihleri
değiştirin.

## 2. cppwrf-realexe: wrfinput ve wrfbdy

```bash
mkdir -p ~/hb/run && cd ~/hb/run
cp $CPPWRF/examples/quickstart/namelist.input .
ln -sf ~/hb/wps/met_em.d0* .
cppwrf-realexe
```

`cppwrf-realexe`, real.exe'nin yazdığı `wrfinput_d01`, `wrfinput_d02` ve `wrfbdy_d01` dosyalarını yazar.
`rsl.out.0000` günlüğü `real_em: SUCCESS COMPLETE REAL_EM INIT` satırıyla biter. İsterseniz WRF'nin kendi real.exe'sini
de kullanabilirsiniz.

real.exe'de olduğu gibi `num_metgrid_levels` ve `num_land_cat`, met_em dosyalarındaki değerlerle aynı olmalıdır.
`run.sh` bunları ilk met_em dosyasından okur.

## 3. cppwrf-wrf: tahmin

```bash
for f in $WRF_DIR/run/*; do case ${f##*/} in namelist.input|*.exe|README*) ;; *) ln -sfn $f . ;; esac; done
cppwrf-check namelist.input          # isteğe bağlı: bu namelist'in hangi seçenekleri destekleniyor?
cppwrf-wrf --mode gpu-fp32           # ya da cpu-fp64 --threads $(nproc), gpu-fp64
```

`cppwrf-wrf`, `namelist.input` dosyasını okur ve WRF'nin dosyalarını koşu dizinine, WRF'nin adlarıyla yazar:

- her iki alan için saatte bir `wrfout_d0N_<tarih>` dosyası;
- `Timing for main` satırlarını içeren `rsl.out.0000` ve `rsl.error.0000` günlükleri.

Son satır `d01 2026-10-12_03:00:00 wrf: SUCCESS COMPLETE WRF` olur. WRF koşularını denetleyen betikleriniz değişmeden
çalışır:

```bash
grep -q "SUCCESS COMPLETE WRF" rsl.out.0000 && echo bitti
```

CPPWRF'in desteklemediği bir seçenek, koşuyu en başta durdurur. İleti seçeneğin adını, değerini ve desteklenen
değerleri söyler; çıkış durumu sıfırdan farklıdır. Böyle bir seçenek hiçbir zaman sessizce yok sayılmaz.

Bu 3 saatlik koşu, RTX PRO 6000 Blackwell üzerinde `gpu-fp32` ile 15 s, `gpu-fp64` ile 42 s sürdü. `cpu-fp64` 48 iş
parçacığıyla 222-273 s sürdü. Kendi makinenizin hızını `cppwrf-bench` ölçer. `cppwrf-fit namelist.input` ise bir
koşunun bellek ve süre ihtiyacını koşmadan önce tahmin eder.

## 4. Harita

```bash
python3 $CPPWRF/examples/quickstart/plot.py ~/hb/run      # slp_t2_d01.png
```

Çizim, `cppwrf` Python paketini kullanır (../PYTHON.md):

```python
import cppwrf
from cppwrf import diagnostics as dg

ds = cppwrf.open_wrfout("~/hb/run")
slp = dg.slp(ds)                      # deniz seviyesi basıncı (hPa), wrf-python'daki yöntemle
t2 = dg.t2_celsius(ds)                # 2 m sıcaklığı (°C)
yagis = dg.precip_accum(ds, hours=1)  # saatlik yağış (mm)
```

## Sonra

- [../MIGRATING_FROM_WRF.md](../MIGRATING_FROM_WRF.md): neyin aynı kaldığı, desteklenen tüm seçenekler, kendi WRF
  koşunuzu CPPWRF ile karşılaştırma (`cppwrf-compare`).
- [../FAQ.md](../FAQ.md): sık sorulan sorular.
