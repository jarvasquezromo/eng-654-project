#!/usr/bin/env python3
"""Local helper: lets the GoFa lab pages (GitHub Pages or a local copy) use the supplied
crb15000-ik solver, which cannot run inside a browser.

    python3 tools/ik-helper/ik_helper.py          (Linux x86_64, glibc >= 2.38)
    py tools\\ik-helper\\ik_helper.py             (Windows x86_64)

The course solver for both platforms is in tools/ik-helper/solver/ and is picked automatically.
To use another build (e.g. one compiled from the Rust source on macOS), pass --solver PATH or set
CRB15000_IK. Then open the site and switch "IK" (top right) to "Supplied solver".
Only the standard library is used. The server listens on 127.0.0.1 only.

Endpoints
  GET  /health      -> {"ok": true, "solver": ..., "threads": n}
  POST /solve       {"frame": "tool0", "poses": [[16 row-major numbers], ...]}
                    -> {"results": [{"solutions": [...]} | {"error": "..."}], "seconds": t}
  POST /solve-pose  {"pose": [px, py, pz, qw, qx, qy, qz], "frame": "tool0"} -> solver output
The solver's answers are passed through unchanged; the pages verify every configuration
with their own forward kinematics before using it.
"""
import argparse
import glob
import json
import os
import platform
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ALLOWED_ORIGINS = ("https://jarvasquezromo.github.io",)
MAX_POSES = 100_000
HERE = os.path.dirname(os.path.abspath(__file__))


def bundled_solver():
    """The course binary shipped in this repo for the current platform, or None."""
    system, machine = platform.system(), platform.machine().lower()
    if machine not in ("x86_64", "amd64"):
        return None
    if system == "Linux":
        return os.path.join(HERE, "solver", "linux-x86_64", "crb15000-ik")
    if system == "Windows":
        return os.path.join(HERE, "solver", "windows-x86_64", "crb15000-ik.exe")
    return None


def find_solver(explicit):
    # 1) --solver PATH, 2) CRB15000_IK, 3) the binary bundled in tools/ik-helper/solver/
    for name, c in (("--solver", explicit), ("CRB15000_IK", os.environ.get("CRB15000_IK"))):
        if c:
            if not os.path.isfile(c):
                sys.exit(f"{name} {c}: file not found")
            return os.path.abspath(c)
    c = bundled_solver()
    return os.path.abspath(c) if c and os.path.isfile(c) else None


def origin_allowed(origin):
    if not origin:
        return False
    if origin in ALLOWED_ORIGINS:
        return True
    return origin.startswith(("http://localhost", "http://127.0.0.1", "http://[::1]"))


class Handler(BaseHTTPRequestHandler):
    solver = None
    threads = 2

    def _cors(self):
        origin = self.headers.get("Origin")
        if origin_allowed(origin):
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Access-Control-Allow-Private-Network", "true")  # Chrome: public page -> localhost
            self.send_header("Access-Control-Max-Age", "600")

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self._cors()
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_GET(self):
        if self.path.split("?")[0] == "/health":
            return self._json(200, {"ok": True, "solver": os.path.basename(self.solver), "threads": self.threads})
        self._json(404, {"error": "unknown path"})

    def _read(self):
        n = int(self.headers.get("Content-Length", "0"))
        if n <= 0 or n > 200_000_000:
            raise ValueError("missing or too large body")
        return json.loads(self.rfile.read(n))

    def _run(self, args, payload):
        r = subprocess.run([self.solver, *args], input=json.dumps(payload), capture_output=True, text=True)
        try:
            return json.loads(r.stdout)
        except json.JSONDecodeError:
            raise RuntimeError((r.stderr or r.stdout or "solver failed").strip()[:500])

    def do_POST(self):
        try:
            body = self._read()
            path = self.path.split("?")[0]
            t0 = time.time()
            if path == "/solve":
                poses = body.get("poses")
                if not isinstance(poses, list) or len(poses) > MAX_POSES or any(not isinstance(p, list) or len(p) != 16 for p in poses):
                    return self._json(400, {"error": f"poses must be a list of at most {MAX_POSES} lists of 16 numbers"})
                out = self._run(["solve", "--threads", str(self.threads)], {"frame": body.get("frame", "tool0"), "poses": poses})
                results = [{"solutions": r["Ok"]["solutions"], "diagnostics": r["Ok"].get("diagnostics")} if "Ok" in r else {"error": r.get("Err", "error")} for r in out]
                self.log_message("solved %d poses in %.2f s", len(poses), time.time() - t0)
                return self._json(200, {"results": results, "seconds": time.time() - t0})
            if path == "/solve-pose":
                return self._json(200, self._run(["solve-pose"], {"pose": body["pose"], "frame": body.get("frame", "tool0")}))
            self._json(404, {"error": "unknown path"})
        except Exception as e:  # report, keep serving
            self._json(500, {"error": str(e)})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--solver", help="use another crb15000-ik build instead of the bundled one")
    ap.add_argument("--port", type=int, default=8654)
    ap.add_argument("--threads", type=int, default=max(1, (os.cpu_count() or 2)))
    a = ap.parse_args()
    solver = find_solver(a.solver)
    if not solver:
        sys.exit(f"No bundled crb15000-ik for {platform.system()} {platform.machine()} (bundled: Linux and Windows x86_64).\n"
                 "Build it from the course's Rust source (cargo build --release --locked) and pass --solver PATH.")
    if platform.system() != "Windows" and not os.access(solver, os.X_OK):
        os.chmod(solver, os.stat(solver).st_mode | 0o111)
    Handler.solver, Handler.threads = solver, a.threads
    try:
        probe = subprocess.run([solver, "solve-pose"], input='{"pose":[0.571,0,0.899,0.7071067811865476,0,0.7071067811865476,0]}', capture_output=True, text=True)
        ok, msg = probe.returncode == 0, probe.stderr.strip()[:300]
    except OSError as e:
        ok, msg = False, str(e)
    if not ok:
        hint = " (the Linux build needs glibc 2.38 or newer; otherwise build from source and pass --solver)" if platform.system() == "Linux" else ""
        sys.exit(f"solver did not run: {msg}{hint}")
    print(f"crb15000-ik helper: {solver}\nlistening on http://127.0.0.1:{a.port}  ({a.threads} solver threads)  Ctrl+C to stop")
    ThreadingHTTPServer(("127.0.0.1", a.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
