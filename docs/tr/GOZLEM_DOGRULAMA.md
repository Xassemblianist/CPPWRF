# cppwrf-verify-obs: bir koşunun yer gözlemleriyle doğrulanması

`cppwrf-verify-obs`, bir koşunun wrfout dosyalarını alanındaki ve dönemindeki yer gözlemleriyle karşılaştırır: 2 m
sıcaklık, çiy noktası ve nem, 10 m rüzgâr, basınç ve yağış; skorlar değişken, tahmin süresi ve istasyon başına
hesaplanır. Gözlemleri kendisi indirir (NOAA NCEI'nin ISD arşivi ya da Iowa Environmental Mesonet'in METAR arşivi)
veya sizin CSV dosyanızı okur. İngilizcesi ve tüm ayrıntılar: [../VERIFY_OBS.md](../VERIFY_OBS.md).

```bash
cppwrf-verify-obs kosu/wrfout_d01_*                                  # alandaki bütün havalimanlarının METAR'ları
cppwrf-verify-obs kosu/wrfout_d01_* -o dogrulama --stations LTAC,LTBA,LTFM
cppwrf-verify-obs kosu/wrfout_d01_* --obs-source file --obs-file mgm.csv
cppwrf-verify-obs gpu/wrfout_d01_* --compare cpu/wrfout_d01_* --labels gpu-fp32,cpu-fp64
```

Yalnızca `numpy` ve `netCDF4` gerekir; grafikler düz SVG olarak yazılır.

## Gözlemler

- **`iem`** (varsayılan): Iowa Environmental Mesonet'in ASOS indirme servisindeki METAR / SPECI raporları, dünyanın
  her yerindeki havalimanları. Sıcaklık, çiy noktası, rüzgâr, hamle, QNH. ABD dışındaki METAR'larda yağış miktarı
  yoktur.
- **`isd`**: NOAA NCEI Integrated Surface Database, istasyon ve yıl başına bir Global Hourly CSV dosyası; SYNOP
  yağış miktarlarını (1-24 saat) da içerir. **NCEI bu dosyaları Ağustos 2025'te güncellemeyi bıraktı;** sonraki
  koşular için `iem` ya da kendi dosyanızı kullanın.
- **`file`**: kendi CSV dosyanız (`--obs-file`), örneğin MGM istasyon verisi.

İndirmeler nazik yapılır: tek seferde bir istek, aralarında bekleme (`--pause`, 1 s), HTTP 429 / 5xx'te üstel
geri çekilme, yalnızca alandaki istasyonlar ve koşunun dönemi. Her şey önbelleğe (`--cache-dir`, varsayılan
`~/.cache/cppwrf/obs`) yazılır; aynı dönemi yeniden doğrulamak hiçbir şey indirmez, `--offline` yalnızca önbelleği
kullanır.

**Kendi dosyanız:** virgül, noktalı virgül ya da sekmeyle ayrılmış CSV (noktalı virgüllüde ondalık virgül olabilir),
istasyon ve zaman (UTC) başına bir satır:

```text
istasyon;tarih;enlem;boylam;yukseklik;sicaklik;nem;ruzgar_hizi;ruzgar_yonu;yagis;precip_hours
17130;2026-11-09 06:00;39,95;32,88;891;-1,5;78;2,0;90;0,4;6
```

Zorunlu sütunlar istasyon, zaman, enlem ve boylamdır. Birimler: °C, %, m/s, derece, hPa, mm. Yağış `yagis` +
`precip_hours` ya da `precip_6h` gibi sütunlarla verilir. Eksik değer: boş, `NA`, `M` ya da `-9999`. İngilizce
sütun adları (`station`, `time`, `lat`, `lon`, `elev`, `t2`, `td2`, `rh2`, `wspd`, `wdir`, `mslp`, `qnh`, `precip`)
de geçerlidir.

## Kalite kontrol

ISD'de kalite kodu 2, 3, 6 ya da 7 (şüpheli, hatalı) olan değerler atılır; çözülemeyen METAR raporları (istasyon,
zaman ya da hiç rüzgâr / sıcaklık / basınç grubu yoksa) reddedilir ve sayılır; bütün kaynaklarda kaba sınırlar
uygulanır ve sıcaklıktan 0,5 K'den fazla yüksek çiy noktası atılır. Her ret nedenine göre `summary.md` ve
`scores.json`'da sayılır.

## Model değerleri

- İstasyonun ızgaradaki kesirli konumu XLAT / XLONG'un çift doğrusal enterpolasyonunun tersinden bulunur (her
  projeksiyonda çalışır). Sınırdan `spec_bdy_width` nokta içindeki (gevşeme bölgesindeki) istasyonlar dışarıda
  kalır; değer `namelist.input`'tan okunur, yoksa 5'tir (`--boundary` ile değişir).
- Çift doğrusal enterpolasyon (`--interpolation nearest`: en yakın nokta).
- T2, model arazisinin yüksekliğinden istasyonun yüksekliğine standart atmosfer gradyanıyla (6,5 K/km) taşınır
  (`--no-height-correction` kapatır). Çiy noktası ve nem wrf-python'ın td2 / rh2'sidir; rüzgâr COSALPHA / SINALPHA
  ile yere göre döndürülür; QNH, PSFC ve arazi yüksekliğinden ICAO altimetre formülüyle hesaplanır.
- Her çıktı zamanı için, o değişkeni içeren ve `--window` dakika (30) içindeki en yakın gözlem alınır (11:50'deki
  METAR 12:00'ı doğrular).

## Skorlar

Sürekli değişkenler için sayı, yanlılık (tahmin - gözlem), MAE, RMSE ve korelasyon; rüzgâr için vektör RMSE ve yön
hatası (gözlenen hız en az 1 m/s iken); yağış için birikim süresi (1, 3, 6, 12, 24 saat) ve eşik (0,1, 1, 5, 10, 25
mm) başına olasılık tablosu ile POD, FAR, CSI, frekans yanlılığı ve ETS. `--compare` ikinci bir koşuyu tam olarak aynı
çiftler üzerinde skorlar.

Çıktılar (`-o DIZIN`): `summary.md`, `scores_*.csv`, `contingency.csv`, `pairs.csv`, `stations.csv`, `scores.json`
ve `plots/` altında tahmin süresine göre skor grafikleri ile istasyon başına yanlılık haritaları.

## Örnek: WEB2 Türkiye vakası, GPU FP32 ile CPU FP64

GFS 2026-11-08 18 UTC, 24 saat, 39,5 K 33 D merkezli 9 km'lik 100 x 100 nokta; aynı vakanın GPU FP32 ve CPU FP64
koşuları, her üç saatte bir (9 zaman), IEM METAR'larıyla. Alanın içinde 38 havalimanı var; 26'sı bu dönemde rapor
vermiş (890 rapor, hiçbiri reddedilmemiş), 3156 çift.

| Değişken | n | yanlılık | MAE | RMSE | korelasyon |
|---|---|---|---|---|---|
| 2 m sıcaklık (°C) | 211 | +1,19 | 2,03 | 2,54 | 0,94 |
| 2 m çiy noktası (°C) | 211 | -2,26 | 2,87 | 3,58 | 0,79 |
| 10 m rüzgâr hızı (m/s) | 211 | +0,49 | 1,22 | 1,61 | 0,55 |
| 10 m rüzgâr vektörü (m/s) | 180 | | | 2,58 | |
| QNH (hPa) | 211 | -0,29 | 0,72 | 0,89 | 0,91 |

İki koşunun skorları ikinci ondalığa kadar aynıdır: tek bir istasyon ve zamanda tahminleri en çok 0,46 K (T2), 0,14
m/s (rüzgâr) ve 0,01 hPa (QNH) ayrışır; bu, tahmin hatalarının çok altındadır.

## Python'dan

```python
from cppwrf import verify_obs
sonuc = verify_obs.verify("kosu/wrfout_d01_*", obs_source="iem", output_dir="dogrulama")
sonuc["overall"]["kosu"]["t2"]["rmse"]
```
