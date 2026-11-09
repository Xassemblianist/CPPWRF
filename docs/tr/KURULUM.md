# CPPWRF kurulumu

Bu belge, temiz bir Debian ya da Ubuntu sisteminde CPPWRF'i derleyip kurmayı ve kurulumun doğru çalıştığını
denetlemeyi anlatır. Ayrıntılı İngilizce sürümü [../INSTALL.md](../INSTALL.md)'dir. Gerçek veriyle ilk tahmin için
[HIZLI_BASLANGIC.md](HIZLI_BASLANGIC.md)'ye geçin.

## Hangi derleme?

Aynı kaynak ağacından dört ayrı derleme ("flavour") çıkar. İstediğiniz kadarını aynı dizine kurabilirsiniz;
`cppwrf-wrf --mode` çalışırken hangisinin kullanılacağını seçer.

| Derleme | CMake bayrakları | Ne için |
|---|---|---|
| `cpu-fp64` | `-DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=ON` | WRF v4.8.0 real*8 ile bit bit aynı sonuç. GPU gerekmez. |
| `gpu-fp64` | `-DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=ON` | Yine WRF real*8 ile bit bit aynı, ama tek bir NVIDIA GPU'da ve çok daha hızlı. |
| `gpu-fp32` | `-DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=OFF` | En hızlısı. WRF real*8'den, WRF'nin kendi real*4 derlemesi kadar ayrılır. |
| `cpu-fp32` | `-DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=OFF` | Doğrulama paketinin dışında. |

## Gerekli paketler

Bit bit aynılık ispatları Debian 13 üzerinde yapıldı (gcc 14.2, glibc 2.41):

```bash
sudo apt install build-essential g++-14 cmake libnetcdf-dev python3 python3-netcdf4 python3-numpy git
```

Ubuntu 24.04'te de aynı paketler var (`g++-14` "universe" deposunda). Gerekenler şunlar:

- **C++20 derleyicisi.** İspatlarda g++ 14.2 kullanıldı; g++ 11-14 sorunsuz çalışıyor. CPPWRF'in aritmetiği
  gfortran'ınkini birebir izlediği için GNU derleyicileri kullanın; clang denenmedi.
- **CMake 3.24 ya da üstü.** OpenMP g++ ile birlikte gelir.
- **NetCDF-C ve başlık dosyaları** (`libnetcdf-dev`).
- **Python 3, `numpy` ve `netCDF4`:** araçlar için.
- **GPU derlemeleri için CUDA Toolkit 12 ya da üstü ve bir NVIDIA GPU.**
  - Derleme CUDA 12.4 ve 13.0 ile denendi.
  - sm_120 (Blackwell) için CUDA 12.8 ya da üstü gerekir.
  - nvcc'nin kabul etmediği yeni bir g++ varsa `-DCMAKE_CUDA_HOST_COMPILER=g++-13` verin.

## Derleme

```bash
git clone https://github.com/Xassemblianist/CPPWRF.git && cd CPPWRF

cmake -S . -B build-cpu -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=ON
cmake --build build-cpu -j "$(nproc)"

# GPU derlemeleri (bu makinedeki GPU için; başka GPU'lar için CMAKE_CUDA_ARCHITECTURES verin)
cmake -S . -B build-gpu-dp -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=ON
cmake --build build-gpu-dp -j "$(nproc)"
cmake -S . -B build-gpu -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=OFF
cmake --build build-gpu -j "$(nproc)"

ctest --test-dir build-cpu -L unit --output-on-failure
```

CUDA derlemesi uzun sürer ve iş başına birkaç GB bellek ister. Bellek azsa `-j 4` ya da daha az kullanın.

### Önemli CMake seçenekleri

| Seçenek | Varsayılan | Anlamı |
|---|---|---|
| `CMAKE_CUDA_ARCHITECTURES` | `native` | Hangi GPU'lar için derleneceği. Başka makinelerde çalışacaksa listeleyin, ör. `80;86;89;90;120`. |
| `CPPWRF_NATIVE` | `ON` | `-march=native`. Başka işlemcilerde çalışacak ikili dosyalar için `OFF` verin; sonuçlar değişmez. |
| `CPPWRF_PORTABLE_LIBMVEC` | `ON` | glibc'nin vektör `log` / `log10` işlevlerini her işlemcide Intel'deki gibi hesaplar. |
| `CPPWRF_CUDA_GLIBC_MATH` | `ON` | FP64 GPU derlemesinde glibc'nin matematik işlevlerini GPU'da çalıştırır. |

Son iki seçenek neden var? WRF'nin sonucu, matematik işlevlerinin son bitine kadar nasıl yuvarlandığına bağlıdır:

- **`CPPWRF_PORTABLE_LIBMVEC`:** gfortran, vektörleştirdiği döngülerde glibc'nin libmvec kitaplığını çağırır.
  libmvec'in `log` işlevi Intel'de ve AMD'de son bitte farklı sonuç verir. Başvuru WRF koşuları Intel'de yapıldı,
  bu yüzden `ON`, AMD'de de Intel'deki sonucu üretir.
- **`CPPWRF_CUDA_GLIBC_MATH`:** CUDA'nın kendi matematik kitaplığı glibc ile aynı yuvarlamayı yapmaz. `ON`, glibc'nin
  algoritmalarını GPU'ya taşır; `gpu-fp64`'ü WRF ile bit bit aynı yapan budur ve FP64 süresine en çok %7 ekler.

`-ffast-math`, `-Ofast`, `--use_fast_math` gibi sonucu değiştiren bayraklar yapılandırma hatası verir.

## Kurulum

```bash
cmake --install build-cpu --prefix "$HOME/cppwrf"     # build-gpu-dp ve build-gpu de aynı dizine kurulabilir
export PATH="$HOME/cppwrf/bin:$PATH"
```

Kurulan dosyalar:

- `bin/`: her derleme için `cppwrf-<derleme>` (`cppwrf` sonuncusuna bağlantıdır), `cppwrf_real` (real.exe) ve
  `cppwrf-wrf`, `cppwrf-realexe`, `cppwrf-check`, `cppwrf-compare`, `cppwrf-bench`, `cppwrf-fit` araçları;
- `share/cppwrf/`: veriler (RRTMG tabloları, WRF G/Ç şeması), çevirici, Python paketi ve örnekler;
- `share/doc/cppwrf/`: belgeler ve lisanslar.

Kurulan programlar verilerini kendi konumlarından bulur, yani kaynak ağacı silinebilir. Başka bir yer göstermek
için `CPPWRF_DATA_DIR` ortam değişkenini kullanın. WRF'nin fizik tabloları (`LANDUSE.TBL`, `RRTM_DATA` vb.) CPPWRF'e
dahil değildir; WRF v4.8.0 kaynağının `run/` dizininden gelir, ama WRF'yi derlemeniz gerekmez.

## Kurulumu denetleme

```bash
$HOME/cppwrf/share/cppwrf/examples/selftest/run.sh cpu-fp64        # GPU için: gpu-fp64, gpu-fp32
```

Bu betik, `cppwrf-wrf` ile 10 x 10 x 10 noktalık, 5 adımlık küçük bir em_quarter_ss koşusu yapar. Son çıktının
değerlerinden bir SHA-256 özeti hesaplar. FP64 derlemeleri tam olarak şu özeti vermelidir:

```
selftest cpu-fp64: 1c771835584e049d54934c255fb2a6db152c615e3601bb6d52d25682fce9b993 (...)
PASS: identical to the reference values (expected_fp64.txt)
```

`gpu-fp64` de aynı özeti verir, çünkü `cpu-fp64` ile bit bit aynıdır. `gpu-fp32` için başvuru özeti yoktur; koşunun
başarıyla ve sonlu değerlerle bittiği denetlenir. Özet dosyanın baytlarını değil değerleri kapsadığı için NetCDF
sürümünden etkilenmez.

## Hazır paketler ve kapsayıcılar

- **Sürüm paketleri.** Her derleme için bir `.tar.gz` vardır, ör. `cppwrf-1.0.0-cpu-fp64-linux-x86_64.tar.gz`.
  Açıp `bin/` dizinini kullanmanız yeterli. Debian 13'te derlendikleri için glibc 2.38+, GCC 13.2+ libstdc++ ve
  Debian 13'ün NetCDF kitaplığı (`libnetcdf22`) gerekir. GPU paketleri ayrıca uygun bir NVIDIA sürücüsü (CUDA 12.8
  için 570+) ve CUDA çalışma kitaplığı (`libcudart`) ister.
- **Kapsayıcılar.** `packaging/docker/Dockerfile.cpu`, `Dockerfile.cuda` ve `packaging/apptainer/*.def`:

  ```bash
  docker build -f packaging/docker/Dockerfile.cuda -t cppwrf:cuda .
  docker run --rm --gpus all -v "$PWD/vaka:/work" cppwrf:cuda cppwrf-wrf --mode gpu-fp32
  ```

## Sık karşılaşılan sorunlar

| Belirti | Çözüm |
|---|---|
| `netcdf.h: No such file` | `sudo apt install libnetcdf-dev` |
| `'-ffast-math' changes the floating-point results` | Bayrağı kaldırın; ortamdaki `CXXFLAGS`'a da bakın. |
| `Unsupported gpu architecture 'compute_120'` | CUDA 12.8'den eski: mimari listesinden 120'yi çıkarın. |
| `unsupported GNU version` | `-DCMAKE_CUDA_HOST_COMPILER=g++-13` verin ya da daha yeni bir CUDA kurun. |
| Derleme sırasında nvcc ölüyor | Daha az paralel iş: `-j 2`. |
| `libnetcdf.so.22` bulunamıyor | Hazır paket başka bir dağıtımda: kaynaktan derleyin ya da kapsayıcı kullanın. |
| `cannot open .../wrf_schema/wrf-4.8.0.tsv` | `CPPWRF_DATA_DIR`'i `share/cppwrf/data` dizinine ayarlayın. |
| `a cppwrf binary for mode gpu-fp64 was not found` | O derleme kurulu değil: kurun ya da `--exe` ile gösterin. |
| Öz-denetimin özeti farklı | Sonucu değiştiren bir bayrak ya da GNU dışı bir derleyici kullanılmıştır. Varsayılanlarla yeniden derleyin; sorun sürerse bildirin (../FAQ.md). |
