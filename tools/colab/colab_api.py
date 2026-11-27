#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""CPPWRF's client for Google's official Colab API (the MCP server https://colaboratory.googleapis.com/mcp: JSON-RPC
`tools/call` over HTTPS POST), with the subcommands of the old `colab` CLI that the tools under tools/colab and
tools/bench use, so that they keep their command lines:

  colab_api.py new -s NAME --gpu ACC [--shape standard|highmem]   create a GPU runtime (ONE create_runtime request;
        DEADLINE_EXCEEDED only means "still being created": the identical call with the same requestId is repeated
        to wait for it, which never creates a second runtime). Exit 0 ready, 3 no capacity / quota
        (FAILED_PRECONDITION), 1 anything else. No CPU runtimes (Colab is for GPU work only). Only
        tools/colab/colab_guard.sh may call this (tests/tools/test_colab_guard.py checks the repository).
  colab_api.py exec -s NAME [--timeout S] [--env K=V]...   Python from stdin in the runtime's kernel; its stdout and
        stderr are printed, an exception exits 1, a timeout exits 124 and closes the kernel session (which aborts
        the code, like the old CLI). Calls are cut by the server after 600 s and reattached with the same
        executionId until --timeout.
  colab_api.py upload -s NAME SRC DST      local SRC -> runtime DST (absolute), 32 MiB pieces, parents created; prints
        "Uploaded DST" only when the runtime's file has SRC's size
  colab_api.py download -s NAME SRC DEST   runtime SRC -> local DEST (resumed with Range, at most 3 tries)
  colab_api.py stop -s NAME                delete the runtime (and its session); a runtime that does not exist is fine.
        On an attached runtime (below) stop only detaches: it never deletes a runtime this project did not create.
  colab_api.py attach -s NAME [--runtime R]   use a GPU runtime the owner opened in the Colab web UI (creating G4 /
        A100 / L4 from the API fails, a Google bug; a web-opened runtime is listed and can be driven): R, else the
        environment CPPWRF_COLAB_RUNTIME, else the one GPU runtime that is neither ours (cppwrf-*) nor CPPMPAS's
        (cppmpas-*) and has no session another tool used in the last CPPWRF_COLAB_ATTACH_IDLE_S (600) s (several such
        runtimes, or none: refused, exit 4). Opens our own kernel session cppwrf-<NAME> on it and records the runtime
        in $CPPWRF_COLAB_GUARD_DIR/attached/, so exec / upload / download / stop -s NAME use it. Prints the
        accelerator ("Accelerator: G4") and our directory on it ("Root: /content/cppwrf_<NAME>"); uploads to an
        attached runtime must go below that directory.
  colab_api.py detach -s NAME              end our use of an attached runtime: kill our processes (command lines with
        our directory), remove /content/cppwrf_<NAME>, close our session. The runtime stays up.
  colab_api.py sessions | runtimes         the account's runtimes (ours marked); own: only ours, count first
  colab_api.py specs | usage               runtime specs; subscription and runtimes

NAME is the tools' session name; the runtime is runtimes/<NAME in lower case, '_' -> '-', prefix "cppwrf-">, so every
command finds it without local state, and stop / exec / upload / download refuse a runtime without that prefix:
nothing here touches a runtime this project did not create (the owner's notebooks, CPPMPAS's runtimes), except one
that `attach` picked: there only our session and our directory are used, and nothing deletes the runtime.

Environment: CPPWRF_COLAB_API_URL (default the MCP endpoint; the offline tests point it at a mock), and
CPPWRF_COLAB_API_HEADERS (default ~/.local/bin/colab_mcp_headers.sh: prints the JSON auth headers, a fresh gcloud ADC
token with the colaboratory scope; called for every request)."""
import argparse
import base64
import json
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

URL = os.environ.get("CPPWRF_COLAB_API_URL", "https://colaboratory.googleapis.com/mcp")
HEADERS_CMD = os.environ.get("CPPWRF_COLAB_API_HEADERS", os.path.expanduser("~/.local/bin/colab_mcp_headers.sh"))
PREFIX = "cppwrf-"
SESSION_ID = "main"
GUARD_DIR = os.environ.get("CPPWRF_COLAB_GUARD_DIR", os.path.expanduser("~/.cache/cppwrf/colab"))
NEVER = ("cppmpas-",)      # runtimes of other tools: never attached, never touched
ATTACH_IDLE_S = float(os.environ.get("CPPWRF_COLAB_ATTACH_IDLE_S", 600))
PIECE = int(os.environ.get("CPPWRF_COLAB_API_PIECE", 32 << 20))  # bytes per upload request (the runtime takes < 50 MB)
CREATE_WAIT_S = 1800       # create_runtime: at most this long waiting for one runtime to come up
STATUSES = ("FAILED_PRECONDITION", "DEADLINE_EXCEEDED", "RESOURCE_EXHAUSTED", "UNAVAILABLE", "NOT_FOUND", "ABORTED",
            "ALREADY_EXISTS", "PERMISSION_DENIED", "UNAUTHENTICATED", "INVALID_ARGUMENT", "INTERNAL", "CANCELLED")


def say(*a):
    print("[colab_api]", *a, file=sys.stderr, flush=True)


class ApiError(Exception):
    def __init__(self, status, text):
        super().__init__(f"{status}: {text}")
        self.status, self.text = status, text


def runtime_id(name):
    r = re.sub(r"[^a-z0-9-]", "-", name.lower()).strip("-")
    if not r.startswith(PREFIX):
        r = PREFIX + r
    return r[:63].rstrip("-")


def ours(rt):  # "runtimes/ID" -> created by this project?
    return rt.split("/")[-1].startswith(PREFIX)


def headers():
    out = subprocess.run([HEADERS_CMD], capture_output=True, text=True, timeout=120)
    if out.returncode != 0:
        raise ApiError("UNAUTHENTICATED", f"{HEADERS_CMD} failed: {out.stderr.strip()[-300:]}")
    h = json.loads(out.stdout)
    h.update({"Content-Type": "application/json", "Accept": "application/json, text/event-stream",
              "User-Agent": "cppwrf-colab-api/1"})
    return h


# Google's canonical messages for the status codes: the Colab API's tool errors carry these texts, not the code names
# (a create without capacity answered "Precondition check failed." on 2026-11-26)
MESSAGES = (("precondition check failed", "FAILED_PRECONDITION"), ("deadline exceeded", "DEADLINE_EXCEEDED"),
            ("resource has been exhausted", "RESOURCE_EXHAUSTED"), ("quota exceeded", "RESOURCE_EXHAUSTED"),
            ("service is currently unavailable", "UNAVAILABLE"), ("requested entity was not found", "NOT_FOUND"),
            ("already exists", "ALREADY_EXISTS"), ("does not have permission", "PERMISSION_DENIED"),
            ("invalid authentication credentials", "UNAUTHENTICATED"), ("invalid argument", "INVALID_ARGUMENT"),
            ("operation was aborted", "ABORTED"))


def status_of(text):
    for s in STATUSES:
        if s in text:
            return s
    low = text.lower()
    for m, s in MESSAGES:
        if m in low:
            return s
    return "ERROR"


def call(tool, args, timeout=700):
    """One JSON-RPC tools/call. Returns structuredContent (or the parsed text); raises ApiError (never retries)."""
    body = json.dumps({"jsonrpc": "2.0", "id": str(uuid.uuid4()), "method": "tools/call",
                       "params": {"name": tool, "arguments": args}}).encode()
    req = urllib.request.Request(URL, data=body, headers=headers(), method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as f:
            raw = f.read().decode()
    except urllib.error.HTTPError as e:
        text = e.read().decode(errors="replace")[:2000]
        st = status_of(text)
        if st == "ERROR":
            st = {429: "RESOURCE_EXHAUSTED", 503: "UNAVAILABLE", 504: "DEADLINE_EXCEEDED", 404: "NOT_FOUND",
                  401: "UNAUTHENTICATED", 403: "PERMISSION_DENIED"}.get(e.code, "ERROR")
        raise ApiError(st, f"HTTP {e.code}: {text}")
    except (TimeoutError, OSError) as e:  # urllib's socket timeout / connection reset
        raise ApiError("DEADLINE_EXCEEDED" if "timed out" in str(e) else "UNAVAILABLE", f"{type(e).__name__}: {e}")
    if raw.lstrip().startswith(("event:", "data:")):  # a server-sent event stream: take the last data line
        raw = [ln[5:] for ln in raw.splitlines() if ln.startswith("data:")][-1]
    msg = json.loads(raw)
    if "error" in msg:
        e = msg["error"]
        text = f"{e.get('message', '')} {json.dumps(e.get('data', ''))}"
        raise ApiError(status_of(text), text)
    res = msg.get("result", {})
    texts = "\n".join(c.get("text", "") for c in res.get("content", []) if c.get("type") == "text")
    if res.get("isError"):
        raise ApiError(status_of(texts), texts)
    if "structuredContent" in res:
        return res["structuredContent"]
    try:
        return json.loads(texts)
    except ValueError:
        return {"text": texts}


def call_retry(tool, args, tries=3, timeout=700):
    """For calls on a runtime that is up (never create_runtime): a few tries on UNAVAILABLE / rate limit."""
    for i in range(tries):
        try:
            return call(tool, args, timeout)
        except ApiError as e:
            if e.status not in ("UNAVAILABLE", "RESOURCE_EXHAUSTED") or i == tries - 1:
                raise
            say(f"{tool}: {e.status}, try {i + 2} of {tries} in {10 * 2 ** i} s")
            time.sleep(10 * 2 ** i)


# ---- runtimes

def list_runtimes():
    return call_retry("list_runtimes", {}).get("runtimes", [])


def spec_for(acc, shape):
    want = "SHAPE_HIGHMEM" if shape == "highmem" else "SHAPE_STANDARD"
    specs = call_retry("list_runtime_specs", {}).get("runtimeSpecs", [])
    for s in specs:
        k = s.get("key", {})
        if k.get("variant") == "VARIANT_GPU" and k.get("accelerator", "").upper() == acc.upper() and k.get("shape") == want:
            if not s.get("eligible", False):
                raise ApiError("FAILED_PRECONDITION", f"the account is not eligible for {acc} {want}")
            return k
    raise ApiError("INVALID_ARGUMENT", f"no GPU spec {acc} {want}; specs: "
                   + ", ".join(f"{s['key'].get('accelerator')}/{s['key'].get('shape')}" for s in specs))


def cmd_new(a):
    if not a.gpu or a.gpu.upper() in ("NONE", "CPU"):
        say("REFUSED: Colab is for GPU work only; no CPU runtimes (run CPU work on the gate or on GitHub CI)")
        return 2
    rid = runtime_id(a.session)
    try:
        key = spec_for(a.gpu, a.shape)
    except ApiError as e:
        say(f"no runtime: {e}")
        return 3 if e.status == "FAILED_PRECONDITION" else 1
    args = {"runtimeId": rid, "requestId": str(uuid.uuid4()), "runtime": {"runtimeSpec": key}}
    say(f"create_runtime runtimes/{rid} {key['accelerator']} {key['shape']} (one request; requestId {args['requestId']})")
    t0 = time.time()
    while True:
        try:
            rt = call("create_runtime", args, timeout=660)
            break
        except ApiError as e:
            # DEADLINE_EXCEEDED: still being created; the identical call (same requestId) waits for the same runtime
            if e.status == "DEADLINE_EXCEEDED" and time.time() - t0 < CREATE_WAIT_S:
                say(f"still being created after {time.time() - t0:.0f} s; waiting on the same request")
                continue
            say(f"create_runtime failed: {e}")
            return 3 if e.status == "FAILED_PRECONDITION" else 1
    name = rt.get("name") or rt.get("runtime", {}).get("name") or f"runtimes/{rid}"
    say(f"{name} READY ({key['accelerator']}) after {time.time() - t0:.0f} s")
    print(f"Session READY: {name}")
    return 0


def own_runtime(session):
    rid = runtime_id(session)
    if not rid.startswith(PREFIX):
        raise ApiError("PERMISSION_DENIED", f"{rid} was not created by CPPWRF")
    return f"runtimes/{rid}"


# ---- attached runtimes (opened by the owner in the web UI; never deleted here)

def attach_file(session):
    return os.path.join(GUARD_DIR, "attached", runtime_id(session))


def attached(session):
    """The runtime that `attach -s session` picked, or None."""
    try:
        with open(attach_file(session)) as f:
            return f.read().split()[0]
    except (OSError, IndexError):
        return None


def job_root(session):
    """Our directory on an attached runtime: /content/cppwrf_<name>."""
    return "/content/" + runtime_id(session).replace("-", "_")


def target(session):
    """(runtime, kernel session, attached?) of a tool's session name."""
    rt = attached(session)
    if rt:
        return rt, f"{rt}/sessions/{runtime_id(session)}", True
    rt = own_runtime(session)
    return rt, f"{rt}/sessions/{SESSION_ID}", False


def short(rt):
    return rt.split("/")[-1]


def accel(r):
    return (r.get("runtimeSpec", {}).get("accelerator") or "NONE").upper()


def _time(ts):
    from datetime import datetime, timezone
    try:
        return datetime.strptime(ts.rstrip("Z")[:26], "%Y-%m-%dT%H:%M:%S.%f").replace(tzinfo=timezone.utc).timestamp()
    except ValueError:
        try:
            return datetime.strptime(ts.rstrip("Z")[:19], "%Y-%m-%dT%H:%M:%S").replace(tzinfo=timezone.utc).timestamp()
        except ValueError:
            return None


def busy_sessions(rt, own_sid):
    """Sessions on rt that another tool opened and used in the last ATTACH_IDLE_S s (or that run code now). The
    notebook's own session (fileId=...) is the owner's browser and does not count. When the runtime's activity
    cannot be read, every other session counts as busy."""
    names = [s.get("name", "").split("/")[-1] for s in call_retry("list_sessions", {"parent": rt}).get("sessions", [])]
    others = [n for n in names if n and n != own_sid and not n.startswith("fileId=")]
    if not others:
        return []
    try:
        ci = call_retry("get_runtime", {"name": rt}).get("connectionInfo", {})
        req = urllib.request.Request(ci["url"].rstrip("/") + "/api/sessions",
                                     headers={"X-Colab-Runtime-Proxy-Token": ci["token"],
                                              "User-Agent": "cppwrf-colab-api/1"})
        with urllib.request.urlopen(req, timeout=60) as f:
            live = {x.get("path") or x.get("name"): x.get("kernel") or {} for x in json.loads(f.read())}
    except (ApiError, KeyError, ValueError, OSError) as e:
        say(f"{rt}: cannot read its sessions' activity ({e}); counting {len(others)} other sessions as busy")
        return others
    busy = []
    for n in others:
        k = live.get(n)
        if k is None:
            continue  # no kernel any more
        t = _time(k.get("last_activity") or "")
        if k.get("execution_state") == "busy" or t is None or time.time() - t < ATTACH_IDLE_S:
            busy.append(n)
    return busy


def ensure_session(rt, sid):
    try:
        call_retry("create_session", {"parent": rt, "sessionId": sid}, timeout=300)
    except ApiError as e:
        if e.status != "ALREADY_EXISTS":
            raise


def cmd_attach(a):
    sid = runtime_id(a.session)
    want = a.runtime or os.environ.get("CPPWRF_COLAB_RUNTIME", "")
    if want and not want.startswith("runtimes/"):
        want = "runtimes/" + want
    rts = list_runtimes()
    if want:
        hit = [r for r in rts if r.get("name", "").rstrip(".") == want.rstrip(".")]
        if not hit:
            say(f"REFUSED: no runtime {want}; the account has: " + (", ".join(r.get("name", "?") for r in rts) or "none"))
            return 4
        r = hit[0]
        if short(r["name"]).startswith(NEVER + (PREFIX,)):
            say(f"REFUSED: {r['name']} belongs to a tool (cppmpas-* is CPPMPAS's; cppwrf-* runtimes are created and "
                "deleted by colab_guard.sh), not one the owner opened")
            return 4
        if accel(r) == "NONE":
            say(f"REFUSED: {r['name']} has no GPU (Colab is for GPU work only)")
            return 4
        busy = busy_sessions(r["name"], sid)
        if busy:
            say(f"REFUSED: {r['name']} has sessions another tool used in the last {ATTACH_IDLE_S:.0f} s: "
                + ", ".join(busy))
            return 4
    else:
        cands = [r for r in rts if accel(r) != "NONE" and not short(r.get("name", "")).startswith(NEVER + (PREFIX,))]
        free = [r for r in cands if not busy_sessions(r["name"], sid)]
        if len(free) != 1:
            if not cands:
                say("REFUSED: no GPU runtime to attach to. Open one in the browser: colab.research.google.com, a "
                    "notebook, Runtime > Change runtime type > GPU (G4), Connect; then run again (it is found by "
                    "list_runtimes)")
            else:
                say(f"REFUSED: {len(free)} free GPU runtimes of {len(cands)} the owner opened ("
                    + ", ".join(f"{r['name']} {accel(r)}{'' if r in free else ' busy'}" for r in cands)
                    + "): name one with --runtime NAME or CPPWRF_COLAB_RUNTIME")
            return 4
        r = free[0]
    rt, acc = r["name"], accel(r)
    ensure_session(rt, sid)
    os.makedirs(os.path.dirname(attach_file(a.session)), exist_ok=True)
    with open(attach_file(a.session), "w") as f:
        f.write(f"{rt} {acc}\n")
    say(f"attached to {rt} ({acc}): our session {sid}, our directory {job_root(a.session)}; the runtime is never "
        "deleted by CPPWRF")
    print(f"Session READY: {rt}/sessions/{sid}")
    print(f"Accelerator: {acc}")
    print(f"Root: {job_root(a.session)}")
    return 0


CLEAN = """import os, signal, subprocess, time, shutil
root = {root!r}
assert root.startswith("/content/cppwrf_") and "/" not in root[len("/content/"):]
me = os.getpid()
for sig in (signal.SIGTERM, signal.SIGKILL):
    for p in os.listdir("/proc"):
        if not p.isdigit() or int(p) == me:
            continue
        try:
            cmd = open(f"/proc/{{p}}/cmdline", "rb").read().replace(b"\\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if root in cmd:
            try:
                os.kill(int(p), sig)
            except OSError:
                pass
    time.sleep(2 if sig == signal.SIGTERM else 0)


def mounts():  # mount points below root (the chroot jobs bind /proc, /sys, /dev into root/deb13), deepest first
    ms = [ln.split()[1].replace("\\\\040", " ") for ln in open("/proc/mounts")]
    return sorted((m for m in ms if m.startswith(root + "/")), key=len, reverse=True)


for m in mounts():
    subprocess.run(["umount", "-l", m], capture_output=True)
left = mounts()
if left:  # never delete through a bind mount of the runtime's /dev or /proc
    print("MOUNTED", root, " ".join(left))
else:
    shutil.rmtree(root, ignore_errors=True)
    print("REMOVED", root, not os.path.exists(root))
"""


def detach(session):
    rt, sess, _ = target(session)
    root, rc = job_root(session), 0
    try:
        r = exec_code(rt, sess, CLEAN.format(root=root), 300, attached_rt=True)
        if "REMOVED" not in (r.get("stdout") or "") or r.get("executionError"):
            say(f"detach: removing {root} on {rt} failed: {r.get('stderr', '')} {r.get('executionError', '')}")
            rc = 1
    except ApiError as e:
        say(f"detach: could not clean {root} on {rt}: {e}")
        rc = 0 if e.status == "NOT_FOUND" else 1  # the runtime is gone: nothing of ours is left
    try:
        call("delete_session", {"name": sess}, timeout=120)
    except ApiError:
        pass
    try:
        os.remove(attach_file(session))
    except OSError:
        pass
    say(f"detached from {rt}: our session closed, {root} removed; the runtime is left up")
    return rc


def cmd_detach(a):
    if not attached(a.session):
        say(f"{a.session}: not attached")
        return 0
    return detach(a.session)


def cmd_stop(a):
    if attached(a.session):
        return detach(a.session)  # never delete a runtime the owner opened
    rt = own_runtime(a.session)
    try:
        call("delete_session", {"name": f"{rt}/sessions/{SESSION_ID}"}, timeout=120)
    except ApiError:
        pass  # no session, or the runtime is gone: deleting the runtime closes its sessions anyway
    try:
        call_retry("delete_runtime", {"name": rt, "allowMissing": True}, timeout=300)
    except ApiError as e:
        say(f"delete_runtime {rt} failed: {e}")
        return 1
    say(f"deleted {rt}")
    return 0


def cmd_runtimes(a):
    rts = list_runtimes()
    own = [r for r in rts if ours(r.get("name", ""))]
    if a.cmd == "own":
        print(len(own))
        for r in own:
            print(r["name"], r.get("runtimeSpec", {}).get("accelerator", "?"))
        return 0
    for r in rts:
        print(r.get("name"), r.get("runtimeSpec", {}).get("accelerator", "?"),
              "(cppwrf)" if ours(r.get("name", "")) else "(not ours: never touched)")
    if not rts:
        print("no runtimes")
    return 0


def cmd_specs(a):
    for s in call_retry("list_runtime_specs", {}).get("runtimeSpecs", []):
        k = s.get("key", {})
        print(k.get("variant"), k.get("accelerator"), k.get("shape"), "eligible" if s.get("eligible") else "-")
    return 0


def cmd_usage(a):
    try:
        print("subscription:", json.dumps(call_retry("get_subscription", {})))
    except ApiError as e:
        print("subscription: ?", e)
    return cmd_runtimes(argparse.Namespace(cmd="runtimes"))


# ---- code

WRAP = """import os as _cpw_os
_cpw_env = {env}
_cpw_old = {{k: _cpw_os.environ.get(k) for k in _cpw_env}}
_cpw_os.environ.update(_cpw_env)
try:
    exec(compile({code}, "<cppwrf>", "exec"), {{"__name__": "__main__"}})
finally:
    for _k, _v in _cpw_old.items():
        if _v is None:
            _cpw_os.environ.pop(_k, None)
        else:
            _cpw_os.environ[_k] = _v
"""


class Timeout(Exception):
    pass


def exec_code(rt, sess, code, timeout, attached_rt=False):
    """execute_code until it returns (reattached past the server's cutoff); Timeout closes the kernel session. On an
    attached runtime our session is opened again when it is gone (closed by a timeout, or the kernel restarted)."""
    args = {"session": sess, "code": code, "executionId": str(uuid.uuid4())}
    t0, reopened = time.time(), False
    while True:
        left = timeout - (time.time() - t0)
        if left <= 0:
            try:
                call("delete_session", {"name": sess}, timeout=120)
            except ApiError:
                pass
            raise Timeout()
        try:
            return call("execute_code", args, timeout=min(660, left + 30))
        except ApiError as e:
            if e.status in ("DEADLINE_EXCEEDED", "UNAVAILABLE"):
                continue  # the identical call reattaches to the same execution
            if e.status == "RESOURCE_EXHAUSTED":
                time.sleep(20)
                continue
            if e.status == "NOT_FOUND" and attached_rt and not reopened:
                reopened = True
                ensure_session(rt, sess.split("/")[-1])
                continue
            raise


def cmd_exec(a):
    rt, sess, att = target(a.session)
    env = {}
    for kv in a.env:
        k, _, v = kv.partition("=")
        env[k] = v
    code = WRAP.format(env=repr(env), code=repr(sys.stdin.read()))
    try:
        r = exec_code(rt, sess, code, a.timeout, att)
    except Timeout:
        say(f"timeout after {a.timeout} s: closed the kernel session (aborts the code)")
        return 124
    except ApiError as e:
        say(f"execute_code: {e}")
        return 1
    out, err = r.get("stdout", ""), r.get("stderr", "")
    if out:
        sys.stdout.write(out if out.endswith("\n") else out + "\n")
    if err:
        sys.stderr.write(err if err.endswith("\n") else err + "\n")
    if r.get("outputTruncated"):
        say("output truncated by the server")
    xe = r.get("executionError")
    if xe:
        if isinstance(xe, dict):
            tb = xe.get("traceback")
            head = f"{xe.get('name', '')}: {xe.get('value', '')}"
            xe = "\n".join(tb) + "\n" + head if isinstance(tb, list) and tb else head
        sys.stderr.write(f"{xe}\n")
        return 1
    return 0


# ---- files (HTTP to the runtime, header X-Colab-Runtime-Proxy-Token)

class Conn:
    def __init__(self, session):
        self.rt, _, self.attached = target(session)
        self.root = job_root(session)
        self.t = 0

    def info(self):
        if time.time() - self.t > 1800:  # the proxy token lives about an hour
            ci = call_retry("get_runtime", {"name": self.rt}).get("connectionInfo", {})
            self.url, self.tok, self.t = ci["url"].rstrip("/"), ci["token"], time.time()
        return self.url, self.tok

    def req(self, method, path, body=None, hdr=None, timeout=900):
        url, tok = self.info()
        h = {"X-Colab-Runtime-Proxy-Token": tok, "User-Agent": "cppwrf-colab-api/1"}
        h.update(hdr or {})
        if body is not None:
            h["Content-Type"] = "application/json"
            body = json.dumps(body).encode()
        return urllib.request.urlopen(urllib.request.Request(url + path, data=body, headers=h, method=method),
                                      timeout=timeout)


def rel(p):
    return urllib.parse.quote(p.lstrip("/"))


def remote_size(c, path):
    try:
        with c.req("GET", f"/api/contents/{rel(path)}?content=0", timeout=120) as f:
            m = json.loads(f.read())
        return m.get("size"), m.get("type")
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return None, None
        raise


def cmd_upload(a):
    c = Conn(a.session)
    dst = a.dst if a.dst.startswith("/") else "/content/" + a.dst
    if c.attached and not os.path.normpath(dst).startswith(c.root + "/"):
        say(f"REFUSED: {dst} is outside our directory {c.root} on the attached runtime {c.rt}")
        return 1
    parts = dst.strip("/").split("/")
    for i in range(1, len(parts)):  # the parents, one level per request
        d = "/".join(parts[:i])
        if remote_size(c, d)[1] != "directory":
            with c.req("PUT", f"/api/contents/{rel(d)}", {"type": "directory"}, timeout=120):
                pass
    size = os.path.getsize(a.src)
    name = parts[-1]
    with open(a.src, "rb") as f:
        n, sent = 0, 0
        while True:
            piece = f.read(PIECE)
            n += 1
            last = f.tell() >= size
            body = {"type": "file", "format": "base64", "name": name, "path": "/".join(parts),
                    "content": base64.b64encode(piece).decode()}
            if not (n == 1 and last):
                body["chunk"] = -1 if last else n
            for t in range(3):
                try:
                    with c.req("PUT", f"/api/contents/{rel(dst)}", body):
                        break
                except (urllib.error.URLError, OSError) as e:
                    if t == 2 or body.get("chunk"):  # a chunked file cannot resend a piece safely
                        say(f"upload of {a.src} failed at piece {n}: {e}")
                        return 1
                    time.sleep(10)
            sent += len(piece)
            if last:
                break
    got = remote_size(c, dst)[0]
    if got != size:
        say(f"upload of {a.src}: the runtime has {got} bytes, not {size}")
        return 1
    print(f"Uploaded {dst} ({size} bytes)")
    return 0


def cmd_download(a):
    c = Conn(a.session)
    tmp = a.dest + ".part"
    if os.path.exists(tmp):
        os.remove(tmp)
    total = None
    for t in range(3):
        have = os.path.getsize(tmp) if os.path.exists(tmp) else 0
        try:
            with c.req("GET", f"/files/{rel(a.src)}", hdr={"Range": f"bytes={have}-"} if have else None,
                       timeout=1800) as f, open(tmp, "ab" if have else "wb") as o:
                if have and f.status != 206:  # the server ignored the range: start again
                    o.truncate(0)
                cr = f.headers.get("Content-Range")
                total = int(cr.split("/")[-1]) if cr and "/" in cr else int(f.headers.get("Content-Length") or -1)
                while True:
                    b = f.read(1 << 20)
                    if not b:
                        break
                    o.write(b)
            if total in (None, -1) or os.path.getsize(tmp) == total:
                os.replace(tmp, a.dest)
                print(f"Downloaded {a.src} -> {a.dest} ({os.path.getsize(a.dest)} bytes)")
                return 0
        except urllib.error.HTTPError as e:
            if e.code == 416 and total is not None and have == total:
                os.replace(tmp, a.dest)
                return 0
            say(f"download of {a.src}: HTTP {e.code}")
            if e.code == 404:
                break
        except (urllib.error.URLError, OSError) as e:
            say(f"download of {a.src}: {e} (try {t + 1} of 3)")
        time.sleep(10)
    if os.path.exists(tmp):
        os.remove(tmp)
    return 1


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("new")
    s.add_argument("-s", "--session", required=True)
    s.add_argument("--gpu")
    s.add_argument("--shape", choices=("standard", "highmem"), default="standard")
    s = sub.add_parser("exec")
    s.add_argument("-s", "--session", required=True)
    s.add_argument("--timeout", type=float, default=600)
    s.add_argument("--env", action="append", default=[])
    for name in ("upload", "download"):
        s = sub.add_parser(name)
        s.add_argument("-s", "--session", required=True)
        s.add_argument("src")
        s.add_argument("dst" if name == "upload" else "dest")
    for name in ("stop", "detach"):
        s = sub.add_parser(name)
        s.add_argument("-s", "--session", required=True)
    s = sub.add_parser("attach")
    s.add_argument("-s", "--session", required=True)
    s.add_argument("--runtime", help="runtimes/ID or ID (default: CPPWRF_COLAB_RUNTIME, else the only free one)")
    for name in ("sessions", "runtimes", "own", "specs", "usage"):
        sub.add_parser(name)
    a = p.parse_args(argv)
    fn = {"new": cmd_new, "exec": cmd_exec, "upload": cmd_upload, "download": cmd_download, "stop": cmd_stop, "attach": cmd_attach, "detach": cmd_detach,
          "sessions": cmd_runtimes, "runtimes": cmd_runtimes, "own": cmd_runtimes, "specs": cmd_specs,
          "usage": cmd_usage}[a.cmd]
    try:
        return fn(a)
    except ApiError as e:
        say(f"{a.cmd}: {e}")
        return 1
    except urllib.error.HTTPError as e:
        say(f"{a.cmd}: HTTP {e.code} {e.read().decode(errors='replace')[:500]}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
