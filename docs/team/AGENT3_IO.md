# Agent 3 task T3.1 — I/O library, config parser, Python reader

You are Agent 3 in the CPPWRF project (/home/ostriquetrum/CPPWRF), a C++20/CUDA weather
model. First read docs/team/PLAN.md. Your task is small, self-contained and fully
specified below. Two other agents are editing other parts of this directory at the same time.

## Hard rules
1. Create or edit files ONLY in: src/io/, tests/unit/io/, scripts/, docs/team/AGENT3_REPORT.md.
   Do not touch any other file, including the top-level CMakeLists.txt.
2. Never run git write commands (add, commit, checkout, stash, reset, init).
3. Never claim something works without pasting the exact command and its real output.
4. C++20, standard library only. No CUDA, no Boost, no nlohmann/json, no external libraries.
5. Do not add features beyond this spec. If something in the spec is ambiguous, pick the
   simplest option and write it down in your report.

## 1. src/io/config.hpp / config.cpp
INI file format:
```
# comment            ; comment too
[domain]
nx = 512
dx = 100.0           # inline comment allowed after whitespace
eta_levels = 1.0, 0.98, 0.95
[run]
case = straka
use_cuda = true
```
- Keys are addressed as "section.key" (lower-case; section and key names match [a-z0-9_]+).
  Keys before any section header -> error.
- Values: trimmed strings. Duplicate key -> error. Unknown syntax -> error with file:line.
- Overrides: strings "section.key=value", applied after loading. They may create new keys.

```cpp
namespace cppwrf::io {
struct ConfigError : std::runtime_error { using std::runtime_error::runtime_error; };
class Config {
 public:
  static Config load(const std::string& path, const std::vector<std::string>& overrides = {});
  static Config from_string(const std::string& text, const std::vector<std::string>& overrides = {});
  bool has(const std::string& key) const;
  std::string get_string(const std::string& key) const;                  // throws if missing
  std::string get_string(const std::string& key, const std::string& def) const;
  long long   get_int(const std::string& key) const;     long long get_int(const std::string& key, long long def) const;
  double      get_double(const std::string& key) const;  double    get_double(const std::string& key, double def) const;
  bool        get_bool(const std::string& key) const;    bool      get_bool(const std::string& key, bool def) const; // true/false/1/0/yes/no
  std::vector<double> get_double_list(const std::string& key) const;     // comma separated
  std::vector<std::string> keys() const;                                 // sorted
  std::string to_json() const;  // {"domain":{"nx":"512",...},...}; values as JSON strings
};
}
```
Every conversion must check the whole string (e.g. "12abc" as int -> ConfigError naming the key).

## 2. src/io/output.hpp / output.cpp
```cpp
namespace cppwrf::io {
enum class DType { F32, F64 };
struct VarView {
  std::string name;
  char stagger;            // 'M', 'U', 'V', 'W'
  int nx, ny, nz;          // logical extents of THIS variable (already staggered), no halo
  const void* data;        // host pointer to element (0,0,0)
  DType dtype;
  std::ptrdiff_t sj, sk;   // element strides for j and k; the i stride is always 1
};                         // element (i,j,k) = data[i + j*sj + k*sk]; 2D fields: nz = 1

class JsonWriter {          // minimal builder: objects, arrays, string/number/bool, correct escaping
 public:                    // numbers printed with max_digits10 precision; NaN/Inf -> error
  ...design the API yourself, small and obvious...
  std::string str() const;
};

class SnapshotWriter {
 public:
  explicit SnapshotWriter(std::string out_dir);   // creates the directory (std::filesystem)
  // Writes snap_<step:06d>.bin (raw little-endian, variables back to back, each one packed
  // contiguous [nz][ny][nx], no halo, no padding) and snap_<step:06d>.json. Each file is
  // written to a temporary name and then renamed, so readers never see partial files.
  void write(long step, double time, const std::vector<VarView>& vars) const;
};
// JSON: {"step":..,"time":..,"variables":[{"name":"u","stagger":"U","shape":[nz,ny,nx],
//        "dtype":"float32","offset":0,"nbytes":...}, ...]}

class DiagWriter {
 public:
  DiagWriter(const std::string& path, std::vector<std::string> columns); // writes the header line
  void row(const std::vector<double>& values);  // size must match; %.9g formatting; flush each row
};
void write_text_file_atomic(const std::string& path, const std::string& content);
}
```

## 3. src/io/CMakeLists.txt
Defines `add_library(cppwrf_io STATIC config.cpp output.cpp)` with public include dir
`${CMAKE_SOURCE_DIR}/src`, and the test executables from tests/unit/io/ registered with
`add_test`. Agent 1 will add `add_subdirectory(src/io)` to the top-level file. To test
before that exists, create your own throwaway top-level project in /tmp/cppwrf_io_build
that adds this subdirectory, and build it there.

## 4. tests/unit/io/
Plain executables using a tiny assert macro (no framework), covering at least:
config — comments, inline comments, whitespace, duplicate key error, bad line error with
line number, overrides creating/replacing keys, every getter incl. bad conversions,
lists, bools; output — round trip: write a strided VarView (sj/sk larger than nx/nx*ny,
mimicking halos) in F32 and F64, read the .bin back and compare bit-exactly; JSON escaping
(quotes, backslash, control chars); DiagWriter header + rows; NaN in JSON -> error.

## 5. scripts/cppwrf_io.py (python3 + numpy only)
```python
read_run(out_dir) -> dict
read_diag(out_dir) -> dict[str, np.ndarray]           # column name -> array
list_snapshots(out_dir) -> list[int]                  # steps, sorted
read_snapshot(out_dir, step) -> dict[str, np.ndarray] # arrays shaped [nz, ny, nx], dtype as stored
```
Plus scripts/test_cppwrf_io.py: generate a snapshot with numpy in the same format (write
the .bin/.json from Python) and check that read_snapshot returns identical arrays. Also
add a small C++ test program that writes a snapshot, then run the Python reader on its
output (cross-language round trip).

## 6. Report: docs/team/AGENT3_REPORT.md (max 40 lines)
Files created; exact build + test commands with their real output (PASS/FAIL counts);
the decisions you took on ambiguities; anything you could not finish. Then stop.
