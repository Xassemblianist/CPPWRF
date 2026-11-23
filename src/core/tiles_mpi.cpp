// SPDX-License-Identifier: GPL-3.0-or-later
// MPI ranks as the workers of a decomposed run (core/tiles.hpp, docs/MPI.md): the transport of core/tiles.cpp's
// collectives over MPI. Built only with CPPWRF_MPI.
#if defined(CPPWRF_MPI)
#include <mpi.h>
#include <sys/resource.h>

#include <cctype>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/tiles_internal.hpp"

#if defined(CPPWRF_USE_GPU)
#define CPPWRF_GPU_RUNTIME_HOST 1
#include "core/gpu_runtime.hpp"
// Open MPI's extensions: MPIX_CUDA_AWARE_SUPPORT / MPIX_Query_cuda_support (and the ROCm ones)
#if defined(OPEN_MPI) && OPEN_MPI && __has_include(<mpi-ext.h>)
#include <mpi-ext.h>
#endif
#endif

namespace cppwrf::tile {

namespace detail {
void install_hooks();
}

namespace {

void check(int rc, const char* what) {
  if (rc != MPI_SUCCESS) throw std::runtime_error(std::string("MPI: ") + what + " failed");
}

// Messages in order per (sender, receiver) pair (one tag per kind of message: MPI does not overtake), on a
// communicator of their own (a duplicate of MPI_COMM_WORLD). A send is MPI_Isend of a buffer this object keeps until
// flush(), so two ranks that send to each other before they receive cannot block. A halo exchange's messages (tag
// kBulk): every receive posted first, then every send, then one MPI_Waitall; from and into the caller's buffers
// (device memory with a GPU-aware MPI).
class MpiComm : public detail::Comm {
 public:
  // the communicator is left to MPI_Finalize: freed while a stopping rank's messages are unmatched, it would end the
  // run with MPI's error in place of the rank's own
  explicit MpiComm(bool device) : device_(device) { check(MPI_Comm_dup(MPI_COMM_WORLD, &comm_), "MPI_Comm_dup"); }
  MpiComm(const MpiComm&) = delete;
  MpiComm& operator=(const MpiComm&) = delete;
  void send(int to, std::vector<char>&& m) override {
    if (m.size() > std::size_t(INT_MAX)) throw std::runtime_error("MPI: a message of more than 2 GiB");
    pending_.push_back(std::move(m));
    reqs_.push_back(MPI_REQUEST_NULL);
    check(MPI_Isend(pending_.back().data(), int(pending_.back().size()), MPI_BYTE, to, kMsg, comm_, &reqs_.back()),
          "MPI_Isend");
  }
  std::vector<char> recv(int from) override {
    MPI_Status st;
    check(MPI_Probe(from, kMsg, comm_, &st), "MPI_Probe");
    int n = 0;
    check(MPI_Get_count(&st, MPI_BYTE, &n), "MPI_Get_count");
    std::vector<char> m(std::size_t(n > 0 ? n : 0));
    check(MPI_Recv(m.data(), n, MPI_BYTE, from, kMsg, comm_, MPI_STATUS_IGNORE), "MPI_Recv");
    return m;
  }
  void flush() override {
    if (reqs_.empty()) return;
    check(MPI_Waitall(int(reqs_.size()), reqs_.data(), MPI_STATUSES_IGNORE), "MPI_Waitall");
    reqs_.clear();
    pending_.clear();
  }
  void exchange(const std::vector<detail::Post>& out, const std::vector<detail::Post>& in, bool) override {
    breqs_.assign(in.size() + out.size(), MPI_REQUEST_NULL);
    std::size_t r = 0;
    for (const detail::Post& m : in) {
      if (m.bytes > std::size_t(INT_MAX)) throw std::runtime_error("MPI: a halo message of more than 2 GiB");
      check(MPI_Irecv(m.p, int(m.bytes), MPI_BYTE, m.peer, kBulk, comm_, &breqs_[r++]), "MPI_Irecv");
    }
    for (const detail::Post& m : out) {
      if (m.bytes > std::size_t(INT_MAX)) throw std::runtime_error("MPI: a halo message of more than 2 GiB");
      check(MPI_Isend(m.p, int(m.bytes), MPI_BYTE, m.peer, kBulk, comm_, &breqs_[r++]), "MPI_Isend");
    }
    bst_.resize(breqs_.size());
    check(MPI_Waitall(int(breqs_.size()), breqs_.data(), bst_.data()), "MPI_Waitall");
    for (std::size_t q = 0; q < in.size(); ++q) {  // a message of another size: the ranks' calls differ
      int n = 0;
      check(MPI_Get_count(&bst_[q], MPI_BYTE, &n), "MPI_Get_count");
      if (std::size_t(n) != in[q].bytes)
        throw std::logic_error("decomposition: a halo message of " + std::to_string(n) + " bytes from rank " +
                               std::to_string(in[q].peer) + ", " + std::to_string(in[q].bytes) +
                               " expected (the ranks' calls differ)");
    }
  }
  bool device_buffers() const override { return device_; }
  bool allgather(const void* mine, std::size_t bytes, void* all) override {
    if (bytes > std::size_t(INT_MAX)) return false;
    check(MPI_Allgather(mine, int(bytes), MPI_BYTE, all, int(bytes), MPI_BYTE, comm_), "MPI_Allgather");
    return true;
  }

 private:
  static constexpr int kMsg = 0, kBulk = 1;
  MPI_Comm comm_ = MPI_COMM_NULL;
  bool device_ = false;
  std::deque<std::vector<char>> pending_;
  std::vector<MPI_Request> reqs_, breqs_;
  std::vector<MPI_Status> bst_;
};

#if defined(CPPWRF_USE_GPU)
// whether the MPI library takes device memory, as it reports at run time: 1 yes, 0 no, -1 it cannot say
int library_takes_device_memory() {
#if defined(CPPWRF_USE_HIP)
#if defined(MPIX_ROCM_AWARE_SUPPORT) && MPIX_ROCM_AWARE_SUPPORT
  return MPIX_Query_rocm_support() ? 1 : 0;
#elif defined(MPICH_NUMVERSION) && defined(MPIX_GPU_SUPPORT_HIP)
  int yes = 0;
  return MPIX_GPU_query_support(MPIX_GPU_SUPPORT_HIP, &yes) == MPI_SUCCESS ? (yes ? 1 : 0) : -1;
#else
  return -1;
#endif
#else
#if defined(MPIX_CUDA_AWARE_SUPPORT) && MPIX_CUDA_AWARE_SUPPORT
  return MPIX_Query_cuda_support() ? 1 : 0;
#elif defined(OPEN_MPI) && OPEN_MPI && defined(MPIX_CUDA_AWARE_SUPPORT)
  return 0;  // Open MPI built without CUDA
#elif defined(MPICH_NUMVERSION) && defined(MPIX_GPU_SUPPORT_CUDA)
  int yes = 0;
  return MPIX_GPU_query_support(MPIX_GPU_SUPPORT_CUDA, &yes) == MPI_SUCCESS ? (yes ? 1 : 0) : -1;
#else
  return -1;
#endif
#endif
}
#endif

// run.gpu_aware_mpi / CPPWRF_GPU_AWARE_MPI (auto, on, off): whether the halo exchanges hand device buffers to MPI.
// auto: when the library reports that it takes device memory; on: always (the library must take it); off: never
// (pinned host staging). The same choice on every rank (rank 0's). CPU build: never.
bool device_transport(const std::string& key, int rank) {
  std::string v = key.empty() ? "auto" : key;
  if (const char* e = std::getenv("CPPWRF_GPU_AWARE_MPI"); e && *e) v = e;
  for (char& c : v) c = char(std::tolower(static_cast<unsigned char>(c)));
  if (v != "auto" && v != "on" && v != "off" && v != "1" && v != "0")
    throw std::runtime_error("run.gpu_aware_mpi (CPPWRF_GPU_AWARE_MPI): auto, on or off, not '" + v + "'");
  int use = 0;
#if defined(CPPWRF_USE_GPU)
  const int lib = library_takes_device_memory();
  use = v == "on" || v == "1" ? 1 : v == "auto" ? (lib == 1 ? 1 : 0) : 0;
  check(MPI_Bcast(&use, 1, MPI_INT, 0, MPI_COMM_WORLD), "MPI_Bcast");
  if (rank == 0)
    std::printf("cppwrf: halo exchanges %s (run.gpu_aware_mpi = %s; the MPI library %s device memory)\n",
                use ? "from and into device buffers (GPU-aware MPI)" : "through pinned host buffers", v.c_str(),
                lib == 1 ? "takes" : lib == 0 ? "does not take" : "does not say whether it takes");
#else
  (void)rank;
#endif
  return use != 0;
}

bool started = false;

}  // namespace

bool mpi_start(int* argc, char*** argv) {
  int provided = 0;
  check(MPI_Init_thread(argc, argv, MPI_THREAD_FUNNELED, &provided), "MPI_Init_thread");
  started = true;
  int n = 1;
  MPI_Comm_size(MPI_COMM_WORLD, &n);
  // a launcher that does not match the MPI library starts every process as a world of its own: each would run the
  // whole case as one process and write the same files
  if (n == 1) {
    const char* e = std::getenv("CPPWRF_RANKS");
    int p = 1, q = 1;
    if (e && std::sscanf(e, "%dx%d", &p, &q) >= 1 && p * q > 1)
      throw std::runtime_error(std::string("CPPWRF_RANKS=") + e + " but this process is the only MPI rank (does mpirun "
                               "belong to the MPI library the build links?)");
  }
  return n > 1;
}

void mpi_finish() {
  if (started) MPI_Finalize();
  started = false;
}

void mpi_abort(int code) {
  if (started) MPI_Abort(MPI_COMM_WORLD, code);
}

int mpi_run(const std::string& run_ranks, const std::string& gpu_aware, const std::function<int(int)>& body) {
  int n = 1, rank = 0;
  MPI_Comm_size(MPI_COMM_WORLD, &n);
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  int rc = 2;
  try {
    Options o;
    std::string t = run_ranks;
    if (const char* e = std::getenv("CPPWRF_RANKS"); e && *e) t = e;
    if (t.empty()) {  // the most square P x Q = n with P <= Q (WRF's MPASPECT)
      o.px = 1;
      for (int p = 1; p * p <= n; ++p)
        if (n % p == 0) o.px = p;
      o.py = n / o.px;
    } else {
      char c = 0;
      if (std::sscanf(t.c_str(), "%d%c%d", &o.px, &c, &o.py) != 3 || (c != 'x' && c != 'X') || o.px < 1 || o.py < 1)
        throw std::runtime_error("run.ranks: PxQ (patches along x and y), not '" + t + "'");
      if (o.px * o.py != n)
        throw std::runtime_error("run.ranks = " + t + " needs " + std::to_string(o.px * o.py) + " MPI ranks, this run has " +
                                 std::to_string(n));
    }
    if (rank == 0) std::printf("cppwrf: %d MPI ranks as %d x %d patches\n", n, o.px, o.py);
    // the log is the writer's (rank 0); errors of every rank go to stderr
    else if (!std::freopen("/dev/null", "w", stdout)) throw std::runtime_error("MPI: cannot silence a rank's stdout");
#if defined(CPPWRF_USE_GPU)
    // the rank's device, by its rank among the ranks of its node (core/tiles.hpp rank_device; docs/MPI.md)
    {
      MPI_Comm node = MPI_COMM_NULL;
      check(MPI_Comm_split_type(MPI_COMM_WORLD, MPI_COMM_TYPE_SHARED, rank, MPI_INFO_NULL, &node),
            "MPI_Comm_split_type");
      int local = 0, nlocal = 1;
      MPI_Comm_rank(node, &local);
      MPI_Comm_size(node, &nlocal);
      int count = 0;
      if (gpuGetDeviceCount(&count) != gpuSuccess) count = 0;
      std::string err;
      int dev = -1;
      try {
        dev = rank_device(std::getenv("CPPWRF_RANK_DEVICES"), local, count);
      } catch (const std::exception& e) {
        err = e.what();
      }
      if (err.empty() && dev >= 0) {
        const gpuError_t ce = gpuSetDevice(dev);
        if (ce != gpuSuccess)
          err = std::string("MPI: gpuSetDevice(") + std::to_string(dev) + "): " + gpuGetErrorString(ce);
      }
      // the node's ranks on this rank's device (by its UUID: the ranks may see the devices under other numbers)
      int cur = 0;
#if defined(CPPWRF_USE_HIP)
      // hipDeviceGetUuid (ROCm 5 too: hipDeviceProp_t has its uuid member from ROCm 6.0 on)
      hipUUID uuid{};
      if (err.empty() && (gpuGetDevice(&cur) != gpuSuccess || hipDeviceGetUuid(&uuid, cur) != hipSuccess))
        err = "MPI: cannot query the rank's " CPPWRF_GPU_API_NAME " device";
#else
      gpuDeviceProp prop{};
      if (err.empty() && (gpuGetDevice(&cur) != gpuSuccess || gpuGetDeviceProperties(&prop, cur) != gpuSuccess))
        err = "MPI: cannot query the rank's " CPPWRF_GPU_API_NAME " device";
      const auto& uuid = prop.uuid;
#endif
      std::vector<char> ids(std::size_t(nlocal) * sizeof(uuid));
      MPI_Allgather(&uuid, int(sizeof(uuid)), MPI_BYTE, ids.data(), int(sizeof(uuid)), MPI_BYTE, node);
      MPI_Comm_free(&node);
      if (!err.empty()) throw std::runtime_error(err);
      int share = 0;
      for (int r = 0; r < nlocal; ++r)
        share += std::memcmp(&ids[std::size_t(r) * sizeof(uuid)], &uuid, sizeof(uuid)) == 0;
      set_device_work_share(share);
      // rank 0 names the mapping unless every rank is on device 0
      std::vector<int> devs(std::size_t(n), 0);
      MPI_Gather(&cur, 1, MPI_INT, devs.data(), 1, MPI_INT, 0, MPI_COMM_WORLD);
      if (rank == 0) {
        bool all0 = true;
        for (int d : devs) all0 = all0 && d == 0;
        if (!all0) {
          std::string s;
          for (int r = 0; r < n; ++r)
            s += (r ? ", " : "") + std::to_string(r) + ": " + std::to_string(devs[std::size_t(r)]);
          std::printf("cppwrf: " CPPWRF_GPU_API_NAME " device of each rank (rank: device) %s\n", s.c_str());
        }
      }
    }
#endif
    const bool device = device_transport(gpu_aware, rank);
    detail::install_hooks();
    MpiComm comm(device);
    rc = detail::run_worker(comm, rank, o, body);
    comm.flush();
    // the ranks' peak resident memory: a rank holds its patch, not the domain; the writer also the fields it
    // gathers for the output
    struct rusage ru {};
    getrusage(RUSAGE_SELF, &ru);
    long mine = ru.ru_maxrss / 1024;
    std::vector<long> all(static_cast<std::size_t>(n), 0);
    MPI_Gather(&mine, 1, MPI_LONG, all.data(), 1, MPI_LONG, 0, MPI_COMM_WORLD);
    if (rank == 0) {
      long other = 0, sum = 0;
      for (int r = 0; r < n; ++r) {
        sum += all[static_cast<std::size_t>(r)];
        if (r > 0 && all[static_cast<std::size_t>(r)] > other) other = all[static_cast<std::size_t>(r)];
      }
      std::printf("cppwrf: peak host memory: %ld MiB the writer (rank 0), %ld MiB the largest other rank, %ld MiB in all\n",
                  all[0], other, sum);
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "cppwrf: error (rank %d): %s\n", rank, e.what());
    std::fflush(stderr);
    MPI_Abort(MPI_COMM_WORLD, 2);
  }
  return rc;
}

}  // namespace cppwrf::tile
#endif
