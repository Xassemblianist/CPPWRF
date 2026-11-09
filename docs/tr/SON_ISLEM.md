# cppwrf-post: tahminci alanları, NetCDF ve GRIB2 olarak

`cppwrf-post`, wrfout dosyalarından bir tahmincinin baktığı alanları üretir: 2 m sıcaklık ve nem, 10 m rüzgâr,
deniz seviyesi basıncı, yağış, bulutluluk, kararsızlık indisleri, basınç seviyelerindeki ve yerden belirli
yüksekliklerdeki alanlar. Her çıktı zamanı için bir dosya yazar: CF-1.8 NetCDF, GRIB2 ya da ikisi birden.
İngilizcesi ve tüm ayrıntılar: [../POST.md](../POST.md).

```bash
cppwrf-post kosu/wrfout_d01_*                                # bulunduğunuz dizine NetCDF
cppwrf-post kosu/wrfout_d01_* -o son --format both -j 4      # NetCDF ve GRIB2, dört süreçle
cppwrf-post kosu/wrfout_d01_* -c post.yaml                   # kendi alan, seviye, birim ve paketleme ayarlarınız
cppwrf-post --list-fields                                    # bütün alanlar, birimleriyle
cppwrf-post --print-config > post.ini                        # varsayılan ayarlar, düzenlenecek bir dosya olarak
```

`cppwrf` Python paketinin parçasıdır ([../PYTHON.md](../PYTHON.md)); `numpy` ve `netCDF4` yeterlidir. `.yaml`
ayar dosyası için PyYAML, JPEG 2000 / CCSDS paketlemeli GRIB2 için ecCodes'un Python bağlantıları (`pip install
eccodes`) gerekir. Basit paketlemeli GRIB2'yi CPPWRF kendi kodlayıcısıyla yazar, başka bir şeye ihtiyaç duymaz.

## Yöntem: wrf-python'ınki

Her türetilmiş alan wrf-python 1.3.4.1'in hesapladığı gibi hesaplanır: Fortran yordamları numpy'a aktarılmıştır
(`python/cppwrf/post/wrfpy.py`; her fonksiyon aktardığı yordamın adını verir) ve wrf-python'ın `getvar`'ının
verdiği argümanlarla çağrılır. Yeni bir formül yoktur; tek istisna wrf-python'da bulunmayan yağış türüdür (aşağıda).

wrf-python'ın kendisiyle karşılaştırma (`tools/post/compare_wrfpython.py`), WEB2 Türkiye vakasının dört wrfout
dosyasında (100 x 100 x 45 nokta, 9 km): GPU FP32 koşusunun üç tek duyarlıklı dosyasında **bütün alanlar bit bit
aynıdır**; GPU FP64 koşusunun çift duyarlıklı dosyasında en büyük göreli fark 1e-11'dir (numpy'ın işlem sırasından
gelen yuvarlama). Daha büyük tek fark, `COSALPHA` / `SINALPHA` ile döndürülen 10 m rüzgârdadır (varsayılan):
wrf-python `uvmet10` için projeksiyonun koni formülünü kullanır, ikisi en çok 7,7e-7 m/s ayrışır.
`options.wind_rotation: uvmet` wrf-python'ın değerlerini aynen verir.

## Alanlar

Yüzey ve kolon: T2 (K, °C), TD2, RH2, yere göre (doğu/kuzey) döndürülmüş 10 m rüzgâr (U10, V10, hız, yön),
çıktı aralığındaki en büyük 10 m rüzgâr (GUST10, `WSPD10MAX` varsa), MSLP, PSFC, başlangıçtan beri ve son saatlerdeki
yağış (toplam, konvektif, ızgara ölçekli; kova sayaçları dahil), kar, graupel, yağış türü, kar derinliği, düşük /
orta / yüksek / toplam bulutluluk, yağışa geçebilir su, CAPE / CIN (yüzey, karışık tabaka, en kararsız parsel), LCL,
LFC, 0-1 ve 0-3 km fırtınaya göreli helisite, 2-5 km yükselim helisitesi, donma seviyesi, en büyük yansıtıcılık,
bulut tepesi sıcaklığı.

Basınç seviyeleri (varsayılan 1000, 925, 850, 700, 500, 300, 250, 200 hPa): T, TD, RH, Q, GHT, U, V, W, omega,
mutlak vortisite, yansıtıcılık. Yerden yükseklikler (varsayılan 80, 100, 120 m, rüzgâr türbini göbek yükseklikleri):
U, V, hız, yön (istenirse T, RH, P).

Her alanın yöntemi, birimi ve GRIB2 kimliği [../POST.md](../POST.md)'deki tablolardadır.

**Yağış türü** (CPPWRF'nin kuralı; wrf-python'da yok): aralıktaki toplam en az 0,1 mm ise, donmuş oran f = (kar +
graupel + dolu) / ızgara ölçekli yağış. f >= 0,9: kar (5) ya da graupel ve dolu kardan fazlaysa graupel (9);
0,1 < f < 0,9: karla karışık yağmur (7); aksi halde T2 < 0 °C ise donan yağmur (3), değilse yağmur (1).

**Görüş mesafesi** üretilmez: wrf-python'da bir görüş yordamı yoktur ve kullanılan formüller mikrofizik şemasına
bağlıdır.

## Çıktı dosyaları

Her wrfout zamanı için bir dosya, örneğin `wrfpost_d01_2026-11-09_12:00:00.nc` ve `.grib2`.

- **NetCDF (CF-1.8):** iki boyutlu alanlar kendi adlarıyla, basınç seviyesindekiler `_P`, yükseklikteki alanlar
  `_Z` ekiyle (`T_P`, `U_Z`). `standard_name`, `units`, 2 boyutlu `lat` / `lon`, metre cinsinden projeksiyon
  koordinatları `x` / `y`, `grid_mapping` (Lambert, polar stereografik, Mercator ya da enlem-boylam; WRF'nin 6370
  km'lik küresi), CF zamanı (`hours since` başlangıç, `forecast_reference_time`). IOOS CF denetleyicisi
  (`compliance-checker --test cf:1.8`) hatasız ve uyarısız geçer.
- **GRIB2:** her alan ve seviye için bir mesaj; WMO Kod Tablosu 4.2 parametreleri ve Tablo 4.5 seviye türleri,
  birikimler ve en büyük rüzgâr için Şablon 4.8, ızgara Şablonları 3.30 / 3.20 / 3.10 / 3.0, alan başına ayarlanabilir
  duyarlıklı basit paketleme (Şablon 5.0). Yirmi beş zamanlık bir WEB2 koşusunun 3540 mesajının tamamı ecCodes ile
  çözülmüş; parametre, seviye, zaman aralığı ve ızgara doğru, değerler NetCDF'tekilerle paketleme duyarlılığı
  içinde aynı, ecCodes'un hesapladığı enlem / boylamlar XLAT / XLONG ile 2e-5 derece içinde çıkmıştır. xarray +
  cfgrib dosyaları okur (`filter_by_keys` ile seviye türü seçerek).

## Ayar dosyası

`post.yaml` ya da `post.ini` varsayılanları değiştirir; var olmayan bir anahtar hata verir (yazım hatası sessizce
geçmesin diye).

```yaml
output:
  format: both            # netcdf | grib2 | both
  dir: son
fields:
  surface: [T2C, TD2, RH2, U10, V10, WSPD10, GUST10, MSLP, PRECIP_ACC, PRECIP_3H, PTYPE, CLOUD_TOTAL, CAPE_MU]
  pressure: [T, RH, GHT, U, V]
  height: [U, V, WSPD]
levels:
  pressure: [1000, 850, 700, 500, 250]
  height: [80, 100, 120, 150]
units:
  T2: degC
options:
  precip_hours: 3         # PRECIP_3H vb.
grib2:
  centre: 255             # kendi merkezinizin kodu (Kod Tablosu C-11)
  precision:              # ondalık ölçek çarpanı D: değerler 10**-D duyarlılıkla saklanır
    T2: 1
```

`.ini` biçimi aynı bölümleri kullanır (`[output]`, `[fields]`, `[levels]`, `[units]`, `[options]`, `[netcdf]`,
`[grib2]`, `[grib2.precision]`); listeler virgülle ayrılır.

## Hız

Dosyalar zaman zaman işlenir, bellek tek bir zamanınki kadar kalır. Intel i7-10750H'de 24 saatlik bir WEB2 koşusu
(100 x 100 x 45 nokta, 25 saatlik zaman, bütün varsayılan alanlar, NetCDF ve GRIB2): tek süreçle 31-35 s (230 MB
bellek), `-j 4` ile 16 s.

## Python'dan

```python
from cppwrf import post
yazilanlar = post.process("kosu/wrfout_d01_*", config="post.yaml", output_dir="son", fmt="both", jobs=4)
```
