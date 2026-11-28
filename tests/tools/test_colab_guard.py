#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""tools/colab/colab_guard.sh (the one way the Colab tools ask for a runtime) and tools/colab/colab_api.py (the client
of Google's Colab API), offline: a mock of the API's MCP endpoint (JSON-RPC tools/call) and of a runtime's Jupyter
file server runs on 127.0.0.1 and logs every call. Checks: one create request per spec, one fallback only on "no
capacity", no retry otherwise; at most 2 of this project's runtimes (the account's other runtimes neither counted nor
touched); the block lock; no CPU runtimes; the runtime deleted on exit after success, failure, an error later and
SIGTERM, and --keep; DEADLINE_EXCEEDED waits on the same request; upload / download / exec; colab_lib.sh's cj_start;
attaching to a runtime the owner opened (by name, the only free one, refusals, never deleted, our session, processes
and files removed on exit, error and SIGTERM); and that nothing else in the repository asks for a runtime. Never talks to the real endpoint (every script checks the
URL)."""
import base64
import json
import os
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
GUARD = os.path.join(REPO, "tools", "colab", "colab_guard.sh")
LIB = os.path.join(REPO, "tools", "colab", "colab_lib.sh")
API = os.path.join(REPO, "tools", "colab", "colab_api.py")

SPECS = [{"eligible": True, "key": {"variant": v, "accelerator": a, "shape": s}} for v, a, s in (
    ("VARIANT_CPU", "NONE", "SHAPE_STANDARD"), ("VARIANT_GPU", "G4", "SHAPE_STANDARD"),
    ("VARIANT_GPU", "A100", "SHAPE_STANDARD"), ("VARIANT_GPU", "A100", "SHAPE_HIGHMEM"),
    ("VARIANT_GPU", "L4", "SHAPE_STANDARD"))]


class Err(Exception):
    def __init__(self, status, text):
        super().__init__(text)
        self.status, self.text = status, text


class Mock:
    """The API's state: runtimes, what create_runtime answers per accelerator, the call log, the runtime's files."""

    def __init__(self, root):
        self.root = root
        self.runtimes = {}                  # name -> spec key
        self.create = {}                    # accelerator -> ok | precondition | abusive | unavailable | deadline_once
        self.deadline_seen = set()
        self.calls = []
        self.lock = threading.Lock()
        self.url = None
        self.sessions = {}                  # runtime -> {session id: (execution_state, last_activity or None)}
        self.exec_timeout = 1.5

    def log(self, tool, args):
        with self.lock:
            self.calls.append((tool, args))

    def tool(self, name, a):
        self.log(name, a)
        if name == "list_runtime_specs":
            return {"runtimeSpecs": SPECS}
        if name == "list_runtimes":
            return {"runtimes": [{"name": n, "runtimeSpec": k} for n, k in self.runtimes.items()]}
        if name == "get_subscription":
            return {"tier": "PRO_PLUS"}
        if name == "create_runtime":
            key = a["runtime"]["runtimeSpec"]
            mode = self.create.get(key["accelerator"], "ok")
            if mode == "precondition":
                raise Err("", "Precondition check failed.")   # the real API's text for FAILED_PRECONDITION
            if mode == "unavailable":
                raise Err("UNAVAILABLE", "503 Service Unavailable")
            if mode == "abusive":
                raise Err("PERMISSION_DENIED", "this account's access to Colab runtimes is blocked on suspicion of "
                          "abusive activity")
            if mode == "deadline_once" and a["requestId"] not in self.deadline_seen:
                self.deadline_seen.add(a["requestId"])
                raise Err("DEADLINE_EXCEEDED", "the runtime is still being created")
            name = "runtimes/" + a["runtimeId"]
            self.runtimes[name] = key
            return {"name": name, "runtimeSpec": key}
        if name == "get_runtime":
            if a["name"] not in self.runtimes:
                raise Err("NOT_FOUND", a["name"])
            return {"name": a["name"], "connectionInfo": {"url": self.url, "token": "tok-" + a["name"],
                                                          "expireTime": "2099-01-01T00:00:00Z"}}
        if name == "delete_runtime":
            if a["name"] not in self.runtimes and not a.get("allowMissing"):
                raise Err("NOT_FOUND", a["name"])
            self.runtimes.pop(a["name"], None)
            return {}
        if name == "delete_session":
            rt, _, sid = a["name"].rpartition("/sessions/")
            self.sessions.get(rt, {}).pop(sid, None)
            return {}
        if name == "create_session":
            if a["parent"] not in self.runtimes:
                raise Err("NOT_FOUND", a["parent"])
            sid = a.get("sessionId") or "s-generated"
            if sid in self.sessions.setdefault(a["parent"], {}):
                raise Err("ALREADY_EXISTS", sid)
            self.sessions[a["parent"]][sid] = ("idle", time.strftime("%Y-%m-%dT%H:%M:%S.000000Z", time.gmtime()))
            return {"name": f"{a['parent']}/sessions/{sid}"}
        if name == "list_sessions":
            return {"sessions": [{"name": f"{a['parent']}/sessions/{sid}"} for sid in self.sessions.get(a["parent"], {})]}
        if name == "execute_code":
            rt, _, sid = a["session"].rpartition("/sessions/")
            if rt not in self.runtimes:
                raise Err("NOT_FOUND", rt)
            if sid != "main" and sid not in self.sessions.get(rt, {}):   # an attached runtime: our session first
                raise Err("NOT_FOUND", a["session"])
            code = a["code"].replace("/content", self.root + "/content")
            try:
                r = subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True,
                                   timeout=self.exec_timeout, cwd=self.root)
            except subprocess.TimeoutExpired:
                raise Err("DEADLINE_EXCEEDED", "execution cut off")
            out = {"stdout": r.stdout, "stderr": r.stderr if r.returncode == 0 else ""}
            if r.returncode:
                out["executionError"] = {"name": "Error", "value": r.stderr.strip().splitlines()[-1], "traceback": []}
            return out
        raise Err("INVALID_ARGUMENT", "unknown tool " + name)


def handler(mock):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def reply(self, code, body=b"", ctype="application/json", extra=None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def file_path(self, prefix):
            p = unquote(urlparse(self.path).path)[len(prefix):]
            return os.path.join(mock.root, p.lstrip("/"))

        def token_ok(self):
            tok = self.headers.get("X-Colab-Runtime-Proxy-Token", "")
            return tok.startswith("tok-runtimes/") and tok[4:] in mock.runtimes

        def do_POST(self):
            if self.path != "/mcp":
                return self.reply(404)
            req = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            if self.headers.get("Authorization") != "Bearer offline-test":
                return self.reply(401)
            p = req["params"]
            try:
                res = mock.tool(p["name"], p.get("arguments", {}))
                body = {"jsonrpc": "2.0", "id": req["id"], "result": {
                    "content": [{"type": "text", "text": json.dumps(res)}], "structuredContent": res}}
            except Err as e:
                body = {"jsonrpc": "2.0", "id": req["id"], "result": {
                    "isError": True, "content": [{"type": "text", "text": f"{e.status}: {e.text}" if e.status else e.text}]}}
            self.reply(200, json.dumps(body).encode())

        def do_GET(self):
            if not self.token_ok():
                return self.reply(404)
            u = urlparse(self.path)
            if u.path == "/api/sessions":   # the runtime's Jupyter sessions with their kernels' activity
                rt = self.headers["X-Colab-Runtime-Proxy-Token"][4:]
                mock.log("http_sessions", rt)
                return self.reply(200, json.dumps([
                    {"path": sid, "kernel": {"execution_state": st, "last_activity": t}}
                    for sid, (st, t) in mock.sessions.get(rt, {}).items() if t is not None]).encode())
            if u.path.startswith("/files/"):
                mock.log("http_get", u.path)
                p = self.file_path("/files/")
                if not os.path.isfile(p):
                    return self.reply(404)
                with open(p, "rb") as fh:
                    data = fh.read()
                rng = self.headers.get("Range")
                if rng:
                    start = int(rng.split("=")[1].split("-")[0])
                    return self.reply(206, data[start:], "application/octet-stream",
                                      {"Content-Range": f"bytes {start}-{len(data) - 1}/{len(data)}"})
                return self.reply(200, data, "application/octet-stream")
            if u.path.startswith("/api/contents/"):
                p = self.file_path("/api/contents/")
                if os.path.isdir(p):
                    return self.reply(200, json.dumps({"type": "directory"}).encode())
                if os.path.isfile(p):
                    return self.reply(200, json.dumps({"type": "file", "size": os.path.getsize(p)}).encode())
                return self.reply(404)
            self.reply(404)

        def do_PUT(self):
            if not self.token_ok():
                return self.reply(404)
            p = self.file_path("/api/contents/")
            m = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            mock.log("http_put", {k: v for k, v in m.items() if k != "content"})
            if m["type"] == "directory":
                if not os.path.isdir(p):
                    os.mkdir(p)
                return self.reply(201, b"{}")
            if not os.path.isdir(os.path.dirname(p)):
                return self.reply(404)
            chunk = m.get("chunk")
            with open(p, "ab" if chunk not in (None, 1) else "wb") as f:
                f.write(base64.b64decode(m["content"]))
            self.reply(201, b"{}")
    return H


PRE = """set -euo pipefail
case $CPPWRF_COLAB_API_URL in http://127.0.0.1:*) ;; *) echo "NOT THE MOCK: $CPPWRF_COLAB_API_URL"; exit 99 ;; esac
. "$GUARD"
"""


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="colab_guard_test.")
        self.d = self.tmp.name
        self.root = os.path.join(self.d, "runtime")
        os.makedirs(os.path.join(self.root, "content"))
        self.mock = Mock(self.root)
        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), handler(self.mock))
        self.mock.url = f"http://127.0.0.1:{self.srv.server_address[1]}"
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.hdr = os.path.join(self.d, "headers.sh")
        with open(self.hdr, "w") as f:
            f.write('#!/bin/sh\necho \'{"Authorization": "Bearer offline-test"}\'\n')
        os.chmod(self.hdr, 0o755)
        self.state = os.path.join(self.d, "state")
        # the account's other runtimes: never counted, never touched
        self.mock.runtimes["runtimes/owner-notebook"] = SPECS[4]["key"]
        self.mock.runtimes["runtimes/cppmpas-opm14-gpu"] = SPECS[4]["key"]

    def tearDown(self):
        self.srv.shutdown()
        self.srv.server_close()
        for tool, a in self.mock.calls:
            name = a.get("name") or a.get("session") or a.get("parent") if isinstance(a, dict) else ""
            if tool == "delete_runtime":
                self.assertTrue(name.startswith("runtimes/cppwrf-"), f"{tool} on {name}")
            if tool in ("delete_session", "execute_code"):   # our runtime, or our session on an attached one
                self.assertTrue(name.startswith("runtimes/cppwrf-") or "/sessions/cppwrf-" in name, f"{tool} on {name}")
            if tool == "create_session":
                self.assertTrue(a.get("sessionId", "").startswith("cppwrf-"), f"{tool} {a}")
            if name:
                self.assertNotIn("cppmpas-", name, f"{tool} on {name}")
        self.assertIn("runtimes/owner-notebook", self.mock.runtimes)
        self.assertIn("runtimes/cppmpas-opm14-gpu", self.mock.runtimes)
        self.tmp.cleanup()

    def env(self, **kw):
        e = {k: v for k, v in os.environ.items() if not k.startswith(("CPPWRF_COLAB", "KEEP"))}
        e.update(CPPWRF_COLAB_API_URL=self.mock.url + "/mcp", CPPWRF_COLAB_API_HEADERS=self.hdr,
                 CPPWRF_COLAB_GUARD_DIR=self.state, GUARD=GUARD, LIB=LIB, HOME=self.d)
        e.update({k: v for k, v in kw.items() if v is not None})
        return e

    def bash(self, script, **kw):
        r = subprocess.run(["bash", "-c", PRE + script], env=self.env(**kw), capture_output=True, text=True,
                           timeout=120)
        self.assertNotEqual(r.returncode, 99, r.stdout + r.stderr)
        return r

    def api(self, *args, stdin=None, **kw):
        return subprocess.run([sys.executable, API, *args], env=self.env(**kw), input=stdin, capture_output=True,
                              text=True, timeout=120)

    def creates(self):
        return [a for t, a in self.mock.calls if t == "create_runtime"]

    def deletes(self):
        return [a["name"] for t, a in self.mock.calls if t == "delete_runtime"]

    def ours(self):
        return sorted(n for n in self.mock.runtimes if n.startswith("runtimes/cppwrf-"))


class Guard(Base):
    NEW = ('trap "echo CALLER_CLEANUP" EXIT\ncolab_guard_new cppwrf-g4-20261126_101010 --gpu G4 {fb}|| exit 1\n'
           'echo "UP $COLAB_GUARD_GPU"\n')
    RT = "runtimes/cppwrf-g4-20261126-101010"

    def new(self, fb="", extra="", **kw):
        return self.bash(self.NEW.format(fb=f"--fallback {fb} " if fb else "") + extra, **kw)

    # ---- once
    def test_success_one_request_deleted_on_exit(self):
        r = self.new()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("UP G4", r.stdout)
        self.assertIn("CALLER_CLEANUP", r.stdout)
        c = self.creates()
        self.assertEqual(len(c), 1)
        self.assertEqual(c[0]["runtimeId"], self.RT.split("/")[1])
        self.assertEqual(c[0]["runtime"]["runtimeSpec"]["accelerator"], "G4")
        self.assertRegex(c[0]["requestId"], r"^[0-9a-f-]{36}$")
        self.assertEqual(self.deletes(), [self.RT])
        self.assertEqual(self.ours(), [])

    # ---- a failure: one request, no retry, no fallback unless "no capacity"
    def test_failure_is_one_request_and_stops(self):
        self.mock.create["G4"] = "unavailable"
        r = self.new(fb="A100")
        self.assertNotEqual(r.returncode, 0)
        self.assertNotIn("UP ", r.stdout)
        self.assertIn("FAILED", r.stderr)
        self.assertIn("CALLER_CLEANUP", r.stdout)
        self.assertEqual(len(self.creates()), 1)
        self.assertEqual(self.deletes(), [self.RT])

    def test_no_capacity_without_fallback_is_one_request(self):
        self.mock.create["G4"] = "precondition"
        r = self.new()
        self.assertNotEqual(r.returncode, 0)
        self.assertEqual(len(self.creates()), 1)

    def test_no_capacity_takes_the_fallback_once(self):
        self.mock.create["G4"] = "precondition"
        r = self.new(fb="A100")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("UP A100", r.stdout)
        self.assertEqual([c["runtime"]["runtimeSpec"]["accelerator"] for c in self.creates()], ["G4", "A100"])
        self.assertEqual(self.ours(), [])

    def test_no_capacity_anywhere_is_two_requests(self):
        self.mock.create["G4"] = self.mock.create["A100"] = "precondition"
        r = self.new(fb="A100")
        self.assertNotEqual(r.returncode, 0)
        self.assertEqual(len(self.creates()), 2)
        self.assertIn("FAILED: no runtime", r.stderr)

    def test_still_being_created_waits_on_the_same_request(self):
        self.mock.create["G4"] = "deadline_once"
        r = self.new()
        self.assertEqual(r.returncode, 0, r.stderr)
        c = self.creates()
        self.assertEqual(len(c), 2)
        self.assertEqual(c[0], c[1])   # the identical call: same requestId and runtimeId

    # ---- the 2-runtime limit
    def test_two_of_ours_refuse(self):
        self.mock.runtimes["runtimes/cppwrf-a"] = SPECS[1]["key"]
        self.mock.runtimes["runtimes/cppwrf-b"] = SPECS[1]["key"]
        r = self.new()
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("REFUSED: 2 of this project's runtimes", r.stderr)
        self.assertEqual(self.creates(), [])
        self.assertEqual(self.ours(), ["runtimes/cppwrf-a", "runtimes/cppwrf-b"])  # not deleted: not this tool's

    def test_one_of_ours_and_others_allow(self):
        self.mock.runtimes["runtimes/cppwrf-a"] = SPECS[1]["key"]
        r = self.new()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(len(self.creates()), 1)
        self.mock.runtimes.pop("runtimes/cppwrf-a")

    def test_third_runtime_in_one_shell_is_refused(self):
        r = self.new(extra="colab_guard_new cppwrf-x --gpu L4\ncolab_guard_new cppwrf-y --gpu L4 || echo THIRD_REFUSED\n")
        self.assertIn("THIRD_REFUSED", r.stdout)
        self.assertEqual(len(self.creates()), 2)
        self.assertEqual(self.ours(), [])   # both deleted on exit

    # ---- the block lock
    def test_abusive_writes_the_lock_and_it_blocks_everything(self):
        self.mock.create["G4"] = "abusive"
        r = self.new(fb="A100")
        self.assertNotEqual(r.returncode, 0)
        lock = os.path.join(self.state, "BLOCKED")
        self.assertTrue(os.path.exists(lock))
        with open(lock) as f:
            self.assertIn("abusive", f.read())
        self.assertEqual(len(self.creates()), 1)   # no fallback after a block
        self.mock.create["G4"] = "ok"
        r = self.new()
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("BLOCKED exists", r.stderr)
        self.assertEqual(len(self.creates()), 1)

    def test_lock_file_refuses(self):
        os.makedirs(self.state)
        with open(os.path.join(self.state, "BLOCKED"), "w") as f:
            f.write("by hand\n")
        r = self.new()
        self.assertNotEqual(r.returncode, 0)
        self.assertEqual(self.mock.calls, [])

    # ---- GPU only
    def test_no_cpu_runtimes(self):
        for args in ("", "--gpu CPU", "--gpu none"):
            r = self.bash(f"colab_guard_new cppwrf-c {args} || echo REFUSED_CPU\n")
            self.assertIn("REFUSED_CPU", r.stdout)
        r = self.api("new", "-s", "cppwrf-c")
        self.assertEqual(r.returncode, 2)
        self.assertEqual(self.mock.calls, [])

    # ---- the delete trap
    def test_error_after_success_deletes(self):
        r = self.new(extra="false\necho NOT_HERE\n")
        self.assertNotEqual(r.returncode, 0)
        self.assertNotIn("NOT_HERE", r.stdout)
        self.assertEqual(self.deletes(), [self.RT])
        self.assertEqual(self.ours(), [])

    def test_sigterm_deletes(self):
        p = subprocess.Popen(["bash", "-c", PRE + self.NEW.format(fb="") + "sleep 60 > /dev/null 2>&1 & wait $!\n"], env=self.env(),
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        for _ in range(300):
            if self.ours():
                break
            time.sleep(0.1)
        time.sleep(1)
        p.send_signal(signal.SIGTERM)
        p.communicate(timeout=60)
        self.assertEqual(self.deletes(), [self.RT])
        self.assertEqual(self.ours(), [])

    def test_keep_leaves_a_runtime_that_came_up(self):
        r = self.new(KEEP="1")
        self.assertEqual(r.returncode, 0)
        self.assertEqual(self.deletes(), [])
        self.assertEqual(self.ours(), [self.RT])
        self.mock.runtimes.pop(self.RT)

    def test_keep_still_deletes_a_failed_request(self):
        self.mock.create["G4"] = "unavailable"
        self.new(KEEP="1")
        self.assertEqual(self.deletes(), [self.RT])

    def test_status(self):
        r = subprocess.run(["bash", GUARD, "status"], env=self.env(), capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("no lock", r.stderr)
        self.assertEqual(self.creates(), [])

    # ---- colab_lib.sh
    def test_cj_start_success_failure_cpu(self):
        lib = 'OUT=$HOME TAG=t S= KEEP=0 BK= ANY_CPU=1; REPO=x\n. "$LIB"\n'
        bindir = os.path.join(self.d, "bin")
        os.makedirs(bindir)
        with open(os.path.join(bindir, "rclone"), "w") as f:
            f.write('#!/bin/sh\n[ "$1" = listremotes ] && echo cppwrf-drive:\n')
        os.chmod(os.path.join(bindir, "rclone"), 0o755)
        path = bindir + os.pathsep + os.environ["PATH"]
        r = self.bash(lib + 'cj_start G4\necho "NPROC $NPROC"\n', PATH=path)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertRegex(r.stdout, r"NPROC \d+")
        self.assertTrue(os.path.isdir(os.path.join(self.root, "content", "cpu", "in")))
        self.assertEqual(self.ours(), [])
        self.mock.create["G4"] = "unavailable"
        n = len(self.creates())
        r = self.bash(lib + 'if cj_start G4; then echo STARTED; else echo "CJ_RC $?"; fi\n', PATH=path)
        self.assertIn("CJ_RC 1", r.stdout)
        self.assertEqual(len(self.creates()), n + 1)
        r = self.bash(lib + 'cj_start CPU || echo CPU_REFUSED\n', PATH=path)
        self.assertIn("CPU_REFUSED", r.stdout)
        self.assertEqual(len(self.creates()), n + 1)


class Client(Base):
    S = "cppwrf-l4-x"

    def setUp(self):
        super().setUp()
        self.mock.runtimes["runtimes/" + self.S] = SPECS[4]["key"]

    def tearDown(self):
        self.mock.runtimes.pop("runtimes/" + self.S, None)
        super().tearDown()

    def test_upload_download_round_trip(self):
        for size, piece in ((0, None), (1000, None), (250_000, "100000")):
            src = os.path.join(self.d, f"src{size}")
            with open(src, "wb") as f:
                f.write(os.urandom(size))
            r = self.api("upload", "-s", self.S, src, f"/content/a/b/f{size}", CPPWRF_COLAB_API_PIECE=piece)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn("Uploaded", r.stdout)
            dst = os.path.join(self.d, f"back{size}")
            r = self.api("download", "-s", self.S, f"/content/a/b/f{size}", dst)
            self.assertEqual(r.returncode, 0, r.stderr)
            with open(dst, "rb") as a, open(src, "rb") as b:
                self.assertEqual(a.read(), b.read())
        chunks = [a.get("chunk") for t, a in self.mock.calls if t == "http_put" and a["type"] == "file"]
        self.assertEqual(chunks, [None, None, 1, 2, -1])

    def test_download_missing_fails(self):
        r = self.api("download", "-s", self.S, "/content/nope", os.path.join(self.d, "x"))
        self.assertNotEqual(r.returncode, 0)
        self.assertFalse(os.path.exists(os.path.join(self.d, "x")))
        self.assertFalse(os.path.exists(os.path.join(self.d, "x.part")))

    def test_exec_env_output_and_error(self):
        r = self.api("exec", "-s", self.S, "--env", "A=1 2", stdin="import os\nprint('A', os.environ['A'])\n")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stdout.strip(), "A 1 2")
        r = self.api("exec", "-s", self.S, stdin="raise ValueError('boom')\n")
        self.assertEqual(r.returncode, 1)
        self.assertIn("boom", r.stderr)

    def test_exec_reattaches_then_times_out_and_closes_the_session(self):
        r = self.api("exec", "-s", self.S, "--timeout", "4", stdin="import time\ntime.sleep(30)\n")
        self.assertEqual(r.returncode, 124)
        ex = [a for t, a in self.mock.calls if t == "execute_code"]
        self.assertGreaterEqual(len(ex), 2)
        self.assertEqual(len({a["executionId"] for a in ex}), 1)   # the identical call, reattached
        self.assertIn("delete_session", [t for t, a in self.mock.calls])

    def test_runtime_names_always_ours(self):
        r = self.api("stop", "-s", "owner-notebook")      # becomes cppwrf-owner-notebook: never the owner's
        self.assertEqual(r.returncode, 0)
        self.assertEqual(self.deletes(), ["runtimes/cppwrf-owner-notebook"])
        r = self.api("own")
        self.assertEqual(r.stdout.splitlines()[0], "1")


class Attach(Base):
    """`colab_api.py attach` / colab_guard_attach: runtimes the owner opened in the web UI are used, never deleted."""
    S = "cppwrf-attach-20261127_101010"
    SID = "cppwrf-attach-20261127-101010"
    WEB = "runtimes/j_Web-G4_abc."           # the shape of a web-opened runtime's name
    ROOT = "/content/cppwrf_attach_20261127_101010"

    def setUp(self):
        super().setUp()
        self.mock.exec_timeout = 15
        # the base's runtimes/owner-notebook (L4) is the owner's other GPU runtime; its notebook session
        self.mock.sessions["runtimes/owner-notebook"] = {"fileId=1abc": ("busy", self.ago(0))}
        self.mock.runtimes[self.WEB] = SPECS[1]["key"]
        self.mock.sessions[self.WEB] = {"fileId=2def": ("idle", self.ago(5))}
        self.procs = []

    def tearDown(self):
        for p in self.procs:
            if p.poll() is None:
                p.kill()
        super().tearDown()

    @staticmethod
    def ago(s):
        return time.strftime("%Y-%m-%dT%H:%M:%S.000000Z", time.gmtime(time.time() - s))

    def local(self, remote):
        return os.path.join(self.root, remote.lstrip("/"))

    def attach(self, *args, **kw):
        return self.api("attach", "-s", self.S, *args, **kw)

    def attaches(self):
        return [a for t, a in self.mock.calls if t == "create_session"]

    def no_runtime_touched(self):
        self.assertEqual(self.deletes(), [])
        self.assertEqual(self.creates(), [])
        self.assertIn(self.WEB, self.mock.runtimes)

    def test_attach_by_name_exec_upload_and_stop_only_detaches(self):
        r = self.attach("--runtime", self.WEB.split("/")[1])
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("Accelerator: G4", r.stdout)
        self.assertIn(f"Root: {self.ROOT}", r.stdout)
        self.assertEqual(self.attaches(), [{"parent": self.WEB, "sessionId": self.SID}])
        self.assertTrue(os.path.exists(os.path.join(self.state, "attached", self.SID)))
        # exec on our session; files only below our directory
        r = self.api("exec", "-s", self.S, stdin=f"import os\nos.makedirs('{self.ROOT}/job', exist_ok=True)\n"
                     f"open('{self.ROOT}/job/x', 'w').write('1')\nprint('RAN')\n")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("RAN", r.stdout)
        ex = [a["session"] for t, a in self.mock.calls if t == "execute_code"]
        self.assertEqual(ex, [f"{self.WEB}/sessions/{self.SID}"])
        src = os.path.join(self.d, "f")
        with open(src, "wb") as f:
            f.write(b"abc")
        r = self.api("upload", "-s", self.S, src, f"{self.ROOT}/f")
        self.assertEqual(r.returncode, 0, r.stderr)
        r = self.api("upload", "-s", self.S, src, "/content/owners_file")
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("outside our directory", r.stderr)
        self.assertFalse(os.path.exists(self.local("/content/owners_file")))
        # a process of ours (its command line names our directory) and the owner's file next to ours
        p = subprocess.Popen(["bash", "-c", "sleep 300; true", self.local(self.ROOT) + "/remote_job.py"])
        self.procs.append(p)
        with open(self.local("/content/owner.ipynb"), "w") as f:
            f.write("{}")
        r = self.api("stop", "-s", self.S)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.no_runtime_touched()
        self.assertFalse(os.path.exists(self.local(self.ROOT)))
        self.assertTrue(os.path.exists(self.local("/content/owner.ipynb")))
        self.assertIsNotNone(p.wait(timeout=10))
        self.assertNotIn(self.SID, self.mock.sessions[self.WEB])
        self.assertIn("fileId=2def", self.mock.sessions[self.WEB])
        self.assertFalse(os.path.exists(os.path.join(self.state, "attached", self.SID)))
        # after the detach the name is not attached any more: stop is the old path (our own runtime ID only)
        self.assertNotIn(self.WEB, [a.get("name") for t, a in self.mock.calls if t == "delete_runtime"])

    def test_attach_by_environment(self):
        r = self.attach(CPPWRF_COLAB_RUNTIME="owner-notebook")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("Accelerator: L4", r.stdout)   # the owner's notebook session (busy) does not count
        self.assertEqual(self.attaches()[0]["parent"], "runtimes/owner-notebook")
        self.assertEqual(self.api("detach", "-s", self.S).returncode, 0)
        self.no_runtime_touched()

    def test_attach_the_only_one(self):
        self.mock.runtimes.pop("runtimes/owner-notebook")
        self.mock.runtimes["runtimes/cpu-notebook"] = SPECS[0]["key"]          # no GPU: not a candidate
        self.mock.runtimes["runtimes/cppwrf-l4-y"] = SPECS[4]["key"]           # ours, created: not a candidate
        r = self.attach()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(self.attaches(), [{"parent": self.WEB, "sessionId": self.SID}])
        self.assertEqual(self.api("detach", "-s", self.S).returncode, 0)
        self.no_runtime_touched()
        self.mock.runtimes.pop("runtimes/cppwrf-l4-y")
        self.mock.runtimes["runtimes/owner-notebook"] = SPECS[4]["key"]

    def test_two_candidates_refused(self):
        r = self.attach()
        self.assertEqual(r.returncode, 4)
        self.assertIn("2 free GPU runtimes", r.stderr)
        self.assertEqual(self.attaches(), [])
        self.assertFalse(os.path.exists(os.path.join(self.state, "attached", self.SID)))
        self.no_runtime_touched()

    def test_none_refused_with_how_to(self):
        self.mock.runtimes.pop("runtimes/owner-notebook")
        self.mock.runtimes.pop(self.WEB)
        r = self.attach()
        self.assertEqual(r.returncode, 4)
        self.assertIn("Open one in the browser", r.stderr)
        self.mock.runtimes["runtimes/owner-notebook"] = SPECS[4]["key"]
        self.mock.runtimes[self.WEB] = SPECS[1]["key"]

    def test_another_tools_busy_session(self):
        # another tool used owner-notebook a minute ago: the WEB runtime is then the only free one
        self.mock.sessions["runtimes/owner-notebook"]["s-1234"] = ("idle", self.ago(60))
        # a stale API session (an hour old) and one whose kernel is gone do not make WEB busy
        self.mock.sessions[self.WEB]["s-old"] = ("idle", self.ago(3600))
        self.mock.sessions[self.WEB]["s-gone"] = ("idle", None)
        r = self.attach()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(self.attaches()[0]["parent"], self.WEB)
        self.assertEqual(self.api("detach", "-s", self.S).returncode, 0)
        # named, but busy: refused
        r = self.attach("--runtime", "owner-notebook")
        self.assertEqual(r.returncode, 4)
        self.assertIn("s-1234", r.stderr)
        # running code now (execution_state busy), even if its last activity is old
        self.mock.sessions[self.WEB]["cppmpas-x"] = ("busy", self.ago(3600))
        r = self.attach("--runtime", self.WEB)
        self.assertEqual(r.returncode, 4)
        self.assertEqual(len(self.attaches()), 1)
        self.no_runtime_touched()

    def test_never_other_tools_runtimes_or_cpu(self):
        self.mock.runtimes["runtimes/cppwrf-l4-y"] = SPECS[4]["key"]
        self.mock.runtimes["runtimes/cpu-notebook"] = SPECS[0]["key"]
        for name in ("cppmpas-opm14-gpu", "cppwrf-l4-y", "cpu-notebook", "nope"):
            r = self.attach("--runtime", name)
            self.assertEqual(r.returncode, 4, name)
        self.assertEqual(self.attaches(), [])
        self.mock.runtimes.pop("runtimes/cppwrf-l4-y")

    # ---- through the guard: the trap detaches, never deletes
    GUARDED = ('trap "echo CALLER_CLEANUP" EXIT\ncolab_guard_attach cppwrf-attach-20261127_101010 {rt}|| exit 1\n'
               'echo "UP $COLAB_GUARD_GPU $COLAB_GUARD_ROOT"\n'
               'echo "import os; os.makedirs(\'$COLAB_GUARD_ROOT\', exist_ok=True); print(\'MADE\')" | '
               '"$COLAB_API" exec -s cppwrf-attach-20261127_101010\n')

    def guarded(self, extra="", rt="j_Web-G4_abc.", **kw):
        return self.bash(self.GUARDED.format(rt=f"{rt} " if rt else "") + extra, **kw)

    def assert_detached(self):
        self.no_runtime_touched()
        self.assertFalse(os.path.exists(self.local(self.ROOT)))
        self.assertNotIn(self.SID, self.mock.sessions[self.WEB])
        self.assertIn(("delete_session", {"name": f"{self.WEB}/sessions/{self.SID}"}), self.mock.calls)

    def test_guard_success_detaches_on_exit(self):
        r = self.guarded()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn(f"UP G4 {self.ROOT}", r.stdout)
        self.assertIn("MADE", r.stdout)
        self.assertIn("CALLER_CLEANUP", r.stdout)
        self.assertIn("the runtime stays up", r.stderr)
        self.assert_detached()

    def test_guard_error_detaches(self):
        r = self.guarded(extra="false\necho NOT_HERE\n")
        self.assertNotEqual(r.returncode, 0)
        self.assertNotIn("NOT_HERE", r.stdout)
        self.assert_detached()

    def test_guard_sigterm_detaches(self):
        p = subprocess.Popen(["bash", "-c", PRE + self.GUARDED.format(rt="j_Web-G4_abc. ")
                              + "echo READY\nsleep 60 > /dev/null 2>&1 & wait $!\n"],
                             env=self.env(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        for _ in range(300):
            if os.path.isdir(self.local(self.ROOT)):
                break
            time.sleep(0.1)
        time.sleep(1)
        p.send_signal(signal.SIGTERM)
        p.communicate(timeout=120)
        self.assert_detached()

    def test_guard_refusal_touches_nothing(self):
        r = self.guarded(rt="")    # two candidates
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("Open one in the browser", r.stderr)
        self.assertEqual(self.attaches(), [])
        self.assertEqual([t for t, a in self.mock.calls if t in ("delete_session", "execute_code")], [])
        self.no_runtime_touched()

    def test_guard_keep_leaves_our_session(self):
        r = self.guarded(KEEP="1")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn(self.SID, self.mock.sessions[self.WEB])
        self.assertTrue(os.path.isdir(self.local(self.ROOT)))
        self.assertEqual(self.api("stop", "-s", self.S).returncode, 0)   # stop then only detaches
        self.assert_detached()

    def test_cj_start_attach(self):
        bindir = os.path.join(self.d, "bin")
        os.makedirs(bindir)
        with open(os.path.join(bindir, "rclone"), "w") as f:
            f.write('#!/bin/sh\n[ "$1" = listremotes ] && echo cppwrf-drive:\n')
        os.chmod(os.path.join(bindir, "rclone"), 0o755)
        lib = (f'OUT=$HOME TAG=t S={self.S} KEEP=0 BK= ANY_CPU=1 ATTACH=1 ATTACH_RT={self.WEB}; REPO=x\n. "$LIB"\n')
        r = self.bash(lib + 'cj_start G4\necho "NPROC $NPROC ROOT $CJ_ROOT"\n'
                      'echo "import os; print(os.path.isdir(\'/content/cpu/in\'))" | rx 60\n',
                      PATH=bindir + os.pathsep + os.environ["PATH"])
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn(f"ROOT {self.ROOT}", r.stdout)
        self.assertIn("True", r.stdout)                     # rx: /content/cpu is ours, below ROOT
        self.assertFalse(os.path.exists(self.local("/content/cpu")))
        self.assert_detached()
        # a refused attach (two candidates): the tool's own cleanup sends nothing either
        self.mock.calls.clear()
        r = self.bash(lib.replace(f" ATTACH_RT={self.WEB}", "") + 'cj_start G4 || echo CJ_REFUSED\n',
                      PATH=bindir + os.pathsep + os.environ["PATH"])
        self.assertIn("CJ_REFUSED", r.stdout)
        self.assertEqual([t for t, a in self.mock.calls if t not in ("list_runtimes", "list_sessions", "get_runtime",
                                                                     "http_sessions")], [])

    def test_colab_run_needs_attach_or_t4(self):
        run = os.path.join(REPO, "tools", "colab", "colab_run.sh")
        for args in ([], ["--gpu", "G4"], ["--attach", "--gpu", "T4"]):
            r = subprocess.run(["bash", run, *args], env=self.env(CPPWRF_WRF_RUNS=self.d, CPPWRF_SCRATCH=self.d),
                               capture_output=True, text=True, timeout=60)
            self.assertEqual(r.returncode, 2, args)
            if not args:
                self.assertIn("Open one in the browser", r.stderr)
        self.assertEqual(self.mock.calls, [])


class Repository(unittest.TestCase):
    def test_no_runtime_request_outside_the_guard(self):
        """The old CLI (`colab new|run|exec|upload|download|stop`) nowhere; `colab_api.py new` only in the guard."""
        bad = []
        for root, dirs, files in os.walk(REPO):
            dirs[:] = [d for d in dirs if d not in (".git", "build") and not d.startswith("build")]
            for f in files:
                if not f.endswith((".sh", ".py", ".yml", ".yaml")):
                    continue
                p = os.path.join(root, f)
                if p in (GUARD, API, os.path.abspath(__file__)):
                    continue
                try:
                    with open(p, encoding="utf-8") as fh:
                        text = fh.read()
                except (UnicodeDecodeError, OSError):
                    continue
                for n, line in enumerate(text.splitlines(), 1):
                    code = line.split("#", 1)[0] if f.endswith(".sh") else line
                    if (re.search(r"(^|[\s;&|(`\"'])colab\s+(new|run|exec|upload|download|stop)\b", code)
                            or re.search(r"[\"']colab[\"']\s*,\s*[\"'](new|run)[\"']", code)
                            or re.search(r"(COLAB_API\}?\"?|colab_api\.py)\s+new\b", code)
                            or "create_runtime" in code):
                        bad.append(f"{os.path.relpath(p, REPO)}:{n}: {line.strip()}")
        self.assertEqual(bad, [])
        for rel in ("tools/colab/colab_lib.sh", "tools/colab/colab_run.sh", "tools/colab/drive_sync.sh",
                    "tools/bench/colab_paper.sh"):
            with open(os.path.join(REPO, rel)) as f:
                self.assertIn("colab_guard_new", f.read(), rel)


if __name__ == "__main__":
    unittest.main()
