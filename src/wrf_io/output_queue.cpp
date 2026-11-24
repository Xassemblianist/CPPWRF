// SPDX-License-Identifier: GPL-3.0-or-later
#include "wrf_io/output_queue.hpp"

#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <exception>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace cppwrf::wrf_io {

namespace {

// constant-initialised (no dynamic initialisation in the destructors that use it); the writer thread is joined before
// the process's exit (output_queue::finish)
constinit std::mutex g_nc;
std::mutex& nc_mutex() { return g_nc; }
constinit thread_local bool held = false;  // nc_hold
constinit thread_local int depth = 0;      // NcCall scopes of the thread (a nested one takes no lock)
// the thread has the mutex locked; always owns == (held || depth > 0)
constinit thread_local bool owns = false;

void take() {
  if (!owns) {
    nc_mutex().lock();
    owns = true;
  }
}
void give_back() {
  if (owns && !held && depth == 0) {
    owns = false;
    nc_mutex().unlock();
  }
}

}  // namespace

NcCall::NcCall() {
  take();
  ++depth;
}
NcCall::~NcCall() {
  --depth;
  give_back();
}

void nc_hold() {
  take();
  held = true;
}
void nc_unhold() {
  held = false;
  give_back();
}
bool nc_held() { return held; }

NcReleased::NcReleased() : was_held_(held) {
  // (a wait for the writer inside an NcCall scope would keep the lock the writer needs: never done)
  if (depth > 0) {
    std::fprintf(stderr, "cppwrf: error: NcReleased inside an NcCall scope\n");
    std::abort();
  }
  if (was_held_) nc_unhold();
}
NcReleased::~NcReleased() {
  // (a mutex's lock throws only on a system error; caught here, no exception leaves the destructor: GCC 14 would
  // otherwise call __cxa_call_terminate, which the libstdc++ of a CUDA build's older host compiler does not have)
  try {
    if (was_held_) nc_hold();
  } catch (...) {
    std::abort();
  }
}

namespace output_queue {

namespace {

// the run a job belongs to (OwnerScope: an ensemble member; 0: the process's single run)
constinit thread_local int t_owner = 0;
constinit thread_local const char* t_owner_label = nullptr;

struct Job {
  std::size_t bytes = 0;
  int owner = 0;
  std::string where;  // the file and the run, for an error's message
  std::function<void()> fn;
};

// an owner's jobs not finished and its first error (an ensemble's members: each its own; the writer goes on with
// the jobs of the others)
struct Owner {
  long inflight = 0;
  std::exception_ptr err;
  bool reported = false;
};

// " (writing FILE[, ensemble member I])"
std::string where_of(const std::string& file) {
  std::string w = " (writing " + file;
  if (t_owner_label) w += std::string(", ") + t_owner_label;
  return w + ")";
}
// the error of a job with the file it was writing (and its member); a non-standard exception as it is
std::exception_ptr with_where(std::exception_ptr e, const std::string& where) {
  try {
    std::rethrow_exception(e);
  } catch (const std::exception& x) {
    return std::make_exception_ptr(std::runtime_error(x.what() + where));
  } catch (...) {
    return e;
  }
}

struct Queue {
  std::mutex m;
  std::condition_variable work, space;
  std::deque<Job> jobs;
  std::size_t cap = 0;
  bool forbidden = false;
  bool configured = false;  // the cap the run asked for first (configure)
  std::size_t asked = 0;
  std::size_t bytes = 0;  // of the jobs not finished (queued or running)
  long inflight = 0;      // jobs not finished
  bool stop = false;
  std::map<int, Owner> owners;
  std::thread th;

  void run() {
    for (;;) {
      Job j;
      {
        std::unique_lock<std::mutex> lk(m);
        work.wait(lk, [&] { return stop || !jobs.empty(); });
        if (jobs.empty()) return;
        j = std::move(jobs.front());
        jobs.pop_front();
      }
      bool failed;
      {
        std::lock_guard<std::mutex> lk(m);
        failed = bool(owners[j.owner].err);
      }
      // after an error the synchronous writer of that run would have stopped: its later records are not written
      if (!failed) {
        try {
          j.fn();
        } catch (...) {
          std::exception_ptr e = with_where(std::current_exception(), j.where);
          std::lock_guard<std::mutex> lk(m);
          Owner& o = owners[j.owner];
          if (!o.err) o.err = e;
        }
      }
      j.fn = nullptr;  // the job's state goes here (the last reference to a file closes it, under NcCall)
      std::lock_guard<std::mutex> lk(m);
      bytes -= j.bytes;
      --inflight;
      --owners[j.owner].inflight;
      space.notify_all();
    }
  }
};

Queue& queue() {
  static Queue* q = new Queue;  // never destroyed (finish() joins the thread)
  return *q;
}

}  // namespace

void configure(std::size_t cap_bytes) {
  Queue& q = queue();
  std::lock_guard<std::mutex> lk(q.m);
  if (q.forbidden) return;
  if (q.configured) {  // one value for the process: a change while records are queued would reorder their writes
    if (cap_bytes != q.asked)
      throw std::runtime_error("output.write_queue_mb: " + std::to_string(cap_bytes >> 20) + " MiB here, " +
                               std::to_string(q.asked >> 20) +
                               " MiB set first (one value for the whole run: every domain and ensemble member)");
    return;
  }
  q.configured = true;
  q.asked = cap_bytes;
  q.cap = cap_bytes;
}

void forbid() {
  Queue& q = queue();
  std::lock_guard<std::mutex> lk(q.m);
  q.forbidden = true;
  q.cap = 0;
}

bool on() {
  Queue& q = queue();
  std::lock_guard<std::mutex> lk(q.m);
  return q.cap > 0;
}

OwnerScope::OwnerScope(int owner, const char* label) : owner_(t_owner), label_(t_owner_label) {
  t_owner = owner;
  t_owner_label = label;
}
OwnerScope::~OwnerScope() {
  t_owner = owner_;
  t_owner_label = label_;
}

void check() {
  Queue& q = queue();
  std::exception_ptr e;
  {
    std::lock_guard<std::mutex> lk(q.m);
    auto it = q.owners.find(t_owner);
    if (it != q.owners.end() && it->second.err && !it->second.reported) {
      it->second.reported = true;
      e = it->second.err;
    }
  }
  if (e) std::rethrow_exception(e);
}

namespace {
// waits until no job is queued or running, the NetCDF lock let go meanwhile (a holding thread keeps holding it after)
void wait_idle() {
  Queue& q = queue();
  {
    std::lock_guard<std::mutex> lk(q.m);
    if (q.inflight == 0) return;
  }
  const NcReleased r;
  std::unique_lock<std::mutex> lk(q.m);
  q.space.wait(lk, [&] { return q.inflight == 0; });
}
}  // namespace

void before_create(bool path_open) {
  check();
  if (path_open) {
    wait_idle();
    check();  // an error of those jobs: no new file (it would clobber the one they wrote)
  }
}

void submit(std::size_t bytes, const std::string& file, std::function<void()> job) {
  if (!on()) {  // synchronous: after any job still queued (the record order of every file)
    wait_idle();
    check();
    try {
      job();
    } catch (...) {
      std::rethrow_exception(with_where(std::current_exception(), where_of(file)));
    }
    return;
  }
  check();
  nc_hold();  // from now on this thread's NetCDF calls exclude the writer's (NcCall)
  Queue& q = queue();
  std::unique_lock<std::mutex> lk(q.m);
  Owner& own = q.owners[t_owner];  // (a std::map: the reference stays valid)
  auto full = [&] { return !own.err && q.inflight > 0 && (bytes > q.cap || q.bytes > q.cap - bytes); };
  // back-pressure: wait for the writer without the NetCDF lock. Again after the wait: the threads of an ensemble's
  // members woken together take their turns at the locks, and an earlier one's record can fill the queue again
  while (full()) {
    lk.unlock();
    {
      const NcReleased r;
      std::unique_lock<std::mutex> l2(q.m);
      q.space.wait(l2, [&] { return !full(); });
    }
    lk.lock();
  }
  if (own.err) {
    lk.unlock();
    check();
    return;  // reported before: drop the record, as the stopped synchronous writer
  }
  if (!q.th.joinable()) {
    q.stop = false;
    q.th = std::thread([&q] { q.run(); });
  }
  q.jobs.push_back({bytes, t_owner, where_of(file), std::move(job)});
  q.bytes += bytes;
  ++q.inflight;
  ++own.inflight;
  lk.unlock();
  q.work.notify_one();
}

void drain() {
  Queue& q = queue();
  {
    const NcReleased r;
    std::unique_lock<std::mutex> lk(q.m);
    // this run's jobs (the queue is in order: the jobs of other runs queued before them are done too, the ones
    // queued after them are not waited for)
    q.space.wait(lk, [&] {
      auto it = q.owners.find(t_owner);
      return it == q.owners.end() || it->second.inflight == 0;
    });
  }
  nc_unhold();
  check();
}

bool finish() noexcept {
  bool clean = true;
  try {
    Queue& q = queue();
    {  // every run's jobs
      const NcReleased r;
      std::unique_lock<std::mutex> lk(q.m);
      q.space.wait(lk, [&] { return q.inflight == 0; });
    }
    nc_unhold();
    // the errors not reported yet (the process's run, or an ensemble member's), in the runs' order
    std::vector<std::exception_ptr> errs;
    {
      std::lock_guard<std::mutex> lk(q.m);
      for (auto& [id, o] : q.owners)
        if (o.err && !o.reported) {
          o.reported = true;
          errs.push_back(o.err);
        }
    }
    for (const std::exception_ptr& e : errs) {
      try {
        std::rethrow_exception(e);
      } catch (const std::exception& x) {
        std::fprintf(stderr, "cppwrf: error: %s\n", x.what());
      } catch (...) {
        std::fprintf(stderr, "cppwrf: error: the output writer failed\n");
      }
      clean = false;
    }
    {
      std::lock_guard<std::mutex> lk(q.m);
      q.stop = true;
    }
    q.work.notify_all();
    if (q.th.joinable()) q.th.join();
  } catch (...) {
    std::fprintf(stderr, "cppwrf: error: the output writer failed\n");
    clean = false;
  }
  return clean;
}

}  // namespace output_queue
}  // namespace cppwrf::wrf_io
