"""Test helpers: run the lightweight backend as a subprocess and tally checks.

A separate process is required: the carla client holds the GIL during blocking
RPCs, so an in-process server would deadlock.
"""

import contextlib
import socket
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
failures = []


def check(name, condition, detail=""):
    print(("PASS " if condition else "FAIL ") + name + (f"  [{detail}]" if detail else ""))
    if not condition:
        failures.append(name)


def wait_for_port(host, port, timeout=60.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            socket.create_connection((host, port), timeout=1.0).close()
            return
        except OSError:
            time.sleep(0.2)
    raise RuntimeError(f"backend did not start on {host}:{port}")


@contextlib.contextmanager
def backend_process(host, port, extra_args=()):
    proc = subprocess.Popen(
        [sys.executable, str(HERE / "server.py"), "--host", host, "--port", str(port), *extra_args])
    try:
        wait_for_port(host, port)
        yield proc
    finally:
        proc.terminate()
        proc.wait(timeout=10)


def finish():
    print("\nRESULT:", "FAILED " + ", ".join(failures) if failures else "all checks passed")
    sys.exit(1 if failures else 0)


def add_server_args(parser):
    parser.add_argument("--external", action="store_true",
                        help="use an already running server (for example Unreal) instead of "
                             "starting the lightweight backend")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
