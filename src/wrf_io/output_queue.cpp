// SPDX-License-Identifier: GPL-3.0-or-later
#include "wrf_io/output_queue.hpp"

#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

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

struct Job {
  std::size_t bytes = 0;
  std::function<void()> fn;
};

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
  std::exception_ptr err;
  bool reported = false;
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
        failed = bool(err);
      }
      if (!failed) {  // after an error the synchronous writer would have stopped: the later records are not written
        try {
          j.fn();
        } catch (...) {
          std::lock_guard<std::mutex> lk(m);
          if (!err) err = std::current_exception();
        }
      }
      j.fn = nullptr;  // the job's state goes here (the last reference to a file closes it, under NcCall)
      std::lock_guard<std::mutex> lk(m);
      bytes -= j.bytes;
      --inflight;
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

void check() {
  Queue& q = queue();
  std::exception_ptr e;
  {
    std::lock_guard<std::mutex> lk(q.m);
    if (q.err && !q.reported) {
      q.reported = true;
      e = q.err;
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
  if (path_open) wait_idle();
}

void submit(std::size_t bytes, std::function<void()> job) {
  if (!on()) {  // synchronous: after any job still queued (the record order of every file)
    wait_idle();
    check();
    job();
    return;
  }
  check();
  nc_hold();  // from now on this thread's NetCDF calls exclude the writer's (NcCall)
  Queue& q = queue();
  std::unique_lock<std::mutex> lk(q.m);
  auto full = [&] { return !q.err && q.inflight > 0 && (bytes > q.cap || q.bytes > q.cap - bytes); };
  if (full()) {  // back-pressure: wait for the writer without the NetCDF lock
    lk.unlock();
    {
      const NcReleased r;
      std::unique_lock<std::mutex> l2(q.m);
      q.space.wait(l2, [&] { return !full(); });
    }
    lk.lock();
  }
  if (q.err) {
    lk.unlock();
    check();
    return;  // reported before: drop the record, as the stopped synchronous writer
  }
  if (!q.th.joinable()) {
    q.stop = false;
    q.th = std::thread([&q] { q.run(); });
  }
  q.jobs.push_back({bytes, std::move(job)});
  q.bytes += bytes;
  ++q.inflight;
  lk.unlock();
  q.work.notify_one();
}

void drain() {
  Queue& q = queue();
  {
    const NcReleased r;
    std::unique_lock<std::mutex> lk(q.m);
    q.space.wait(lk, [&] { return q.inflight == 0; });
  }
  nc_unhold();
  check();
}

bool finish() noexcept {
  bool clean = true;
  try {
    try {
      drain();
    } catch (const std::exception& e) {
      std::fprintf(stderr, "cppwrf: error: %s\n", e.what());
      clean = false;
    } catch (...) {
      std::fprintf(stderr, "cppwrf: error: the output writer failed\n");
      clean = false;
    }
    Queue& q = queue();
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
