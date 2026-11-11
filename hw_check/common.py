"""
Shared bits so every check prints and exits the same way.

Every check gives back one of three results:
    PASS = green, it's there and working
    WARN = yellow, kinda works but something's off (slow, low value, not written yet)
    FAIL = red, not there / not talking

Exit codes (run_all.sh uses these to know when to stop):
    0 = all passed, 1 = something failed, 2 = only warnings
"""

import subprocess
import sys

# ---- terminal colours ----
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
BOLD = "\033[1m"
DIM = "\033[2m"
RESET = "\033[0m"

PASS, WARN, FAIL = "PASS", "WARN", "FAIL"
_COLOUR = {PASS: GREEN, WARN: YELLOW, FAIL: RED}
_results = []


def header(title):
    print(f"\n{BOLD}== {title} =={RESET}")


def report(status, name, detail=""):
    """Print one result line, e.g. report(PASS, "ping pi", "0.4 ms")."""
    _results.append(status)
    line = f"  {_COLOUR[status]}{status}{RESET}  {name}"
    if detail:
        line += f"  {DIM}{detail}{RESET}"
    print(line)


def run(cmd, timeout=5):
    """
    Run a shell command and give back (ok, output). Never throws, so a missing
    tool or a hang just turns into ok=False instead of crashing the check.

        ok, out = run(["ping", "-c", "1", "192.168.2.2"])
    """
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode == 0, (p.stdout + p.stderr).strip()
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        return False, str(e)


def finish():
    """Call this at the very end of every check script."""
    if FAIL in _results:
        sys.exit(1)
    if WARN in _results:
        sys.exit(2)
    sys.exit(0)
