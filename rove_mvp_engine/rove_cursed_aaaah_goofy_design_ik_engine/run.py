#!/usr/bin/env python3
"""Entry point that bootstraps a virtualenv on first run, installs the
requirements, then launches the IK engine.

Re-running `python3 run.py` after the first time is a no-op for setup: it
just imports `engine.server` and starts it. We re-exec into the venv's
python so the user never has to think about activation.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import urllib.request
from pathlib import Path


HERE = Path(__file__).resolve().parent
VENV_DIR = HERE / ".venv"
REQUIREMENTS = HERE / "requirements.txt"
# Sentinel env var so the bootstrap step doesn't run a second time after
# we re-exec into the venv interpreter.
_REENTRY_FLAG = "ROVE_IK_ENV_READY"


def _venv_python() -> Path:
    sub = "Scripts" if os.name == "nt" else "bin"
    return VENV_DIR / sub / "python"


def _in_target_env() -> bool:
    if os.environ.get(_REENTRY_FLAG) == "1":
        return True
    if sys.prefix == sys.base_prefix:
        return False  # not in any venv, bootstrap one
    try:
        import numpy  # noqa: F401
        return True
    except ImportError:
        return False


def _venv_already_satisfied(py: Path) -> bool:
    try:
        subprocess.check_call(
            [str(py), "-c", "import numpy"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False


def _ensure_pip_in_venv(py: Path) -> None:
    try:
        subprocess.check_call(
            [str(py), "-m", "pip", "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass

    print("[run.py] venv has no pip — bootstrapping with get-pip.py", flush=True)
    tmp = HERE / ".get-pip.py"
    try:
        urllib.request.urlretrieve(
            "https://bootstrap.pypa.io/get-pip.py", tmp
        )
        subprocess.check_call([str(py), str(tmp)])
    finally:
        if tmp.exists():
            tmp.unlink()


def _create_venv() -> Path:
    if _venv_python().exists():
        return _venv_python()

    print(f"[run.py] creating venv at {VENV_DIR}", flush=True)
    try:
        subprocess.check_call([sys.executable, "-m", "venv", str(VENV_DIR)])
    except subprocess.CalledProcessError:
        if VENV_DIR.exists():
            shutil.rmtree(VENV_DIR)
        subprocess.check_call(
            [sys.executable, "-m", "venv", "--without-pip", str(VENV_DIR)]
        )
    py = _venv_python()
    _ensure_pip_in_venv(py)
    return py


def _install_requirements(py: Path) -> None:
    if not REQUIREMENTS.exists():
        return
    print(f"[run.py] installing {REQUIREMENTS.name} into {VENV_DIR}", flush=True)
    subprocess.check_call(
        [str(py), "-m", "pip", "install", "--upgrade", "pip", "--quiet"]
    )
    subprocess.check_call(
        [str(py), "-m", "pip", "install", "-r", str(REQUIREMENTS), "--quiet"]
    )


def _bootstrap_and_reexec() -> None:
    py = _create_venv()
    if not _venv_already_satisfied(py):
        _install_requirements(py)
    env = os.environ.copy()
    env[_REENTRY_FLAG] = "1"
    os.execve(str(py), [str(py), str(Path(__file__).resolve()), *sys.argv[1:]], env)


def main() -> None:
    if not _in_target_env():
        _bootstrap_and_reexec()
        return  # unreachable — execve replaces the process
    from engine.server import main as _server_main
    _server_main()


if __name__ == "__main__":
    main()
