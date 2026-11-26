#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""tools/colab/colab_guard.sh, the one way the Colab tools ask for a runtime, offline: a mock `colab` (and `rclone`)
first on PATH answers success, 503 (with and without an exit status) or Colab's "abusive activity" block and logs
every call. Checks: one attempt and no retry on failure (also none inside the CLI), the 10-minute and 24-hour limits,
the block lock, CPPWRF_COLAB_FORCE=1 for one call, the session stopped on exit after success, failure, an error later
in the script and SIGTERM, --keep, colab_lib.sh's cj_start, and that no other file of the repository calls
`colab new` / `colab run`. Never runs the real CLI (each script checks that `colab` is the mock)."""
import os
import re
import signal
import subprocess
import tempfile
import time
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
GUARD = os.path.join(REPO, "tools", "colab", "colab_guard.sh")
LIB = os.path.join(REPO, "tools", "colab", "colab_lib.sh")

MOCK_COLAB = r"""#!/usr/bin/env bash
echo "$* RETRIES=${COLAB_CLI_ASSIGN_RETRIES-unset}" >> "$MOCK_COLAB_LOG"
case $1 in
  new)
    case ${MOCK_COLAB_MODE:-ok} in
      ok) echo "[colab] Creating session '$3'..."; echo "[colab] Session READY." ;;
      503) echo "[colab] Creating session '$3'..."; echo "colab_cli.client.ColabRequestError: 503 Service Unavailable" >&2
           exit 1 ;;
      503rc0) echo "[colab] Creating session '$3'..."; echo "503 Service Unavailable" ;;
      rc1) exit 1 ;;
      abusive) echo "[colab] Creating session '$3'..."
               echo "Error: this account's access to Colab runtimes is blocked on suspicion of abusive activity" >&2
               exit 1 ;;
    esac ;;
  stop) echo "[colab] Session stopped." ;;
  exec) cat > /dev/null; printf '%s\n' "${MOCK_EXEC_OUT:-}" ;;
  *) ;;
esac
"""
MOCK_RCLONE = """#!/usr/bin/env bash
case $1 in listremotes) echo "cppwrf-drive:" ;; esac
"""

PRE = """set -euo pipefail
[ "$(command -v colab)" = "$MOCK_DIR/colab" ] || { echo "NOT THE MOCK: $(command -v colab)"; exit 99; }
. "$GUARD"
"""


class Guard(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="colab_guard_test.")
        self.d = self.tmp.name
        self.mock = os.path.join(self.d, "bin")
        os.makedirs(self.mock)
        for name, body in (("colab", MOCK_COLAB), ("rclone", MOCK_RCLONE)):
            p = os.path.join(self.mock, name)
            with open(p, "w") as f:
                f.write(body)
            os.chmod(p, 0o755)
        self.state = os.path.join(self.d, "state")
        self.log = os.path.join(self.d, "colab.log")

    def tearDown(self):
        self.tmp.cleanup()

    def env(self, **kw):
        e = {k: v for k, v in os.environ.items() if not k.startswith(("CPPWRF_COLAB", "MOCK_", "COLAB_CLI"))}
        e.update(PATH=self.mock + os.pathsep + os.environ["PATH"], MOCK_DIR=self.mock, MOCK_COLAB_LOG=self.log,
                 CPPWRF_COLAB_GUARD_DIR=self.state, GUARD=GUARD, LIB=LIB, HOME=self.d)
        e.pop("KEEP", None)
        e.update(kw)
        return e

    def bash(self, script, **kw):
        r = subprocess.run(["bash", "-c", PRE + script], env=self.env(**kw), capture_output=True, text=True,
                           timeout=60)
        self.assertNotEqual(r.returncode, 99, r.stdout + r.stderr)
        return r

    def calls(self, verb=None):
        if not os.path.exists(self.log):
            return []
        with open(self.log) as f:
            ls = [l.split() for l in f.read().splitlines() if l.strip()]
        return [l for l in ls if verb is None or l[0] == verb]

    def attempts(self, *ages, forced=False):
        """Writes attempts AGES seconds ago into the state file."""
        os.makedirs(self.state, exist_ok=True)
        now = int(time.time())
        with open(os.path.join(self.state, "attempts"), "a") as f:
            for a in ages:
                f.write(f"{now - a} old-{a} {'forced' if forced else 'normal'}\n")

    NEW = 'trap "echo CALLER_CLEANUP" EXIT\ncolab_guard_new sess1 --gpu G4 || exit 1\necho SESSION_IS_UP\n'

    # ---- one attempt, no retry
    def test_failure_is_one_attempt_and_stops(self):
        r = self.bash(self.NEW, MOCK_COLAB_MODE="503")
        self.assertNotEqual(r.returncode, 0)
        self.assertNotIn("SESSION_IS_UP", r.stdout)
        self.assertEqual(len(self.calls("new")), 1, self.calls())
        self.assertIn("FAILED", r.stderr)
        self.assertIn("CALLER_CLEANUP", r.stdout)
        self.assertEqual([c[:3] for c in self.calls("stop")], [["stop", "-s", "sess1"]])

    def test_cli_retries_switched_off(self):
        self.bash(self.NEW, MOCK_COLAB_MODE="503", COLAB_CLI_ASSIGN_RETRIES="3")
        self.assertEqual(self.calls("new")[0][-1], "RETRIES=0")

    def test_503_with_exit_status_zero_is_a_failure(self):
        r = self.bash(self.NEW, MOCK_COLAB_MODE="503rc0")
        self.assertNotEqual(r.returncode, 0)
        self.assertNotIn("SESSION_IS_UP", r.stdout)
        self.assertEqual(len(self.calls("new")), 1)

    def test_silent_non_zero_exit_is_a_failure(self):
        r = self.bash(self.NEW, MOCK_COLAB_MODE="rc1")
        self.assertNotEqual(r.returncode, 0)
        self.assertNotIn("SESSION_IS_UP", r.stdout)
        self.assertIn("FAILED", r.stderr)

    # ---- the rate limit
    def test_ten_minute_limit(self):
        self.assertEqual(self.bash(self.NEW).returncode, 0)
        r = self.bash(self.NEW)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("REFUSED", r.stderr)
        self.assertEqual(len(self.calls("new")), 1)

    def test_failed_attempt_counts(self):
        self.bash(self.NEW, MOCK_COLAB_MODE="503")
        r = self.bash(self.NEW)
        self.assertIn("REFUSED", r.stderr)
        self.assertEqual(len(self.calls("new")), 1)

    def test_attempt_older_than_ten_minutes_allows(self):
        self.attempts(700)
        self.assertEqual(self.bash(self.NEW).returncode, 0)
        self.assertEqual(len(self.calls("new")), 1)

    def test_day_limit(self):
        self.attempts(3600, 7200, 10800, 14400, 18000, 21600)  # 6 in the last 24 h, none in the last 10 min
        r = self.bash(self.NEW)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("REFUSED: 6 runtime requests", r.stderr)
        self.assertEqual(self.calls("new"), [])

    def test_day_limit_counts_only_the_last_day(self):
        self.attempts(3600, 7200, 10800, 14400, 18000, 90000, 100000)  # 5 in the last 24 h
        self.assertEqual(self.bash(self.NEW).returncode, 0)
        self.assertEqual(len(self.calls("new")), 1)

    def test_status(self):
        self.attempts(3600)
        r = subprocess.run(["bash", GUARD, "status"], env=self.env(), capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("old-3600", r.stderr)
        self.assertEqual(self.calls(), [])

    # ---- the block lock
    def test_abusive_writes_the_lock_and_it_blocks_everything(self):
        r = self.bash(self.NEW, MOCK_COLAB_MODE="abusive")
        self.assertNotEqual(r.returncode, 0)
        lock = os.path.join(self.state, "BLOCKED")
        self.assertTrue(os.path.exists(lock))
        self.assertIn(lock, r.stderr)
        with open(lock) as f:
            self.assertIn("abusive", f.read())
        os.remove(os.path.join(self.state, "attempts"))   # no rate limit left: only the lock refuses
        r = self.bash(self.NEW, CPPWRF_COLAB_FORCE="1")
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("BLOCKED exists", r.stderr)
        self.assertEqual(len(self.calls("new")), 1)

    def test_lock_file_refuses(self):
        os.makedirs(self.state)
        with open(os.path.join(self.state, "BLOCKED"), "w") as f:
            f.write("by hand\n")
        r = self.bash(self.NEW)
        self.assertNotEqual(r.returncode, 0)
        self.assertEqual(self.calls("new"), [])

    # ---- CPPWRF_COLAB_FORCE=1
    def test_force_once(self):
        self.attempts(60)
        r = self.bash(self.NEW + 'colab_guard_new sess2 || echo SECOND_REFUSED\n', CPPWRF_COLAB_FORCE="1")
        self.assertIn("SESSION_IS_UP", r.stdout)
        self.assertIn("SECOND_REFUSED", r.stdout)
        self.assertEqual([c[2] for c in self.calls("new")], ["sess1"])
        with open(os.path.join(self.state, "attempts")) as f:
            self.assertIn("forced", f.read())

    def test_force_has_its_own_day_limit(self):
        self.attempts(60)
        self.attempts(1000, 2000, 3000, forced=True)
        r = self.bash(self.NEW, CPPWRF_COLAB_FORCE="1")
        self.assertNotEqual(r.returncode, 0)
        self.assertEqual(self.calls("new"), [])

    # ---- the session is stopped on exit
    def test_success_stops_on_exit_and_keeps_the_callers_trap(self):
        r = self.bash(self.NEW)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("SESSION_IS_UP", r.stdout)
        self.assertIn("CALLER_CLEANUP", r.stdout)
        self.assertEqual([c[:3] for c in self.calls("stop")], [["stop", "-s", "sess1"]])

    def test_error_after_success_stops(self):
        r = self.bash(self.NEW + "false\necho NOT_HERE\n")
        self.assertNotEqual(r.returncode, 0)
        self.assertNotIn("NOT_HERE", r.stdout)
        self.assertEqual(len(self.calls("stop")), 1)

    def test_sigterm_stops(self):
        p = subprocess.Popen(["bash", "-c", PRE + self.NEW + "sleep 30 & wait $!\n"], env=self.env(),
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        for _ in range(100):
            if self.calls("new"):
                break
            time.sleep(0.1)
        time.sleep(0.5)
        p.send_signal(signal.SIGTERM)
        p.communicate(timeout=30)
        p.wait()
        self.assertEqual(len(self.calls("stop")), 1, self.calls())

    def test_keep_leaves_a_session_that_came_up(self):
        r = self.bash(self.NEW, KEEP="1")
        self.assertEqual(r.returncode, 0)
        self.assertEqual(self.calls("stop"), [])

    def test_keep_still_stops_a_failed_request(self):
        self.bash(self.NEW, KEEP="1", MOCK_COLAB_MODE="503")
        self.assertEqual(len(self.calls("stop")), 1)

    # ---- colab_lib.sh
    def test_cj_start_failure_returns_at_once_and_stops(self):
        r = self.bash('OUT=$HOME TAG=t S= KEEP=0 BK=; REPO=x\n. "$LIB"\n'
                      'if cj_start G4; then echo STARTED; else echo "CJ_RC $?"; fi\n', MOCK_COLAB_MODE="503",
                      MOCK_EXEC_OUT="Intel Xeon | 8")
        self.assertIn("CJ_RC 1", r.stdout)
        self.assertNotIn("CPU:", r.stdout)
        self.assertEqual(len(self.calls("new")), 1)
        self.assertGreaterEqual(len(self.calls("stop")), 1)

    def test_cj_start_success(self):
        r = self.bash('OUT=$HOME TAG=t S= KEEP=0 BK=; REPO=x\n. "$LIB"\ncj_start G4\necho "NPROC $NPROC"\n',
                      MOCK_EXEC_OUT="Intel Xeon | 8\nOK")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("NPROC 8", r.stdout)
        self.assertEqual(len(self.calls("new")), 1)
        self.assertEqual(len(self.calls("stop")), 2)  # cj_cleanup's stop and the guard's


class Repository(unittest.TestCase):
    def test_no_runtime_request_outside_the_guard(self):
        """`colab new` / `colab run` (both assign a runtime) only in colab_guard.sh; the four request sites use it."""
        bad = []
        for root, dirs, files in os.walk(REPO):
            dirs[:] = [d for d in dirs if d not in (".git", "build") and not d.startswith("build")]
            for f in files:
                if not f.endswith((".sh", ".py", ".yml", ".yaml")):
                    continue
                p = os.path.join(root, f)
                if p in (GUARD, os.path.abspath(__file__)):
                    continue
                try:
                    with open(p, encoding="utf-8") as fh:
                        text = fh.read()
                except (UnicodeDecodeError, OSError):
                    continue
                for n, line in enumerate(text.splitlines(), 1):
                    code = line.split("#", 1)[0] if f.endswith(".sh") else line
                    if re.search(r"(^|[\s;&|(`\"'])colab\s+(new|run)\b", code) or re.search(r"[\"']colab[\"']\s*,\s*[\"'](new|run)[\"']", code):
                        bad.append(f"{os.path.relpath(p, REPO)}:{n}: {line.strip()}")
        self.assertEqual(bad, [])
        for rel in ("tools/colab/colab_lib.sh", "tools/colab/colab_run.sh", "tools/colab/drive_sync.sh",
                    "tools/bench/colab_paper.sh"):
            with open(os.path.join(REPO, rel)) as f:
                self.assertIn("colab_guard_new", f.read(), rel)


if __name__ == "__main__":
    unittest.main()
