"""Clone-and-run: the project sets up its own Python environment.

The scripts always run with the Python inside this project's `.venv` folder.
If it does not exist yet (a fresh clone), the first run creates it and installs
requirements.txt into it. That happens once, takes a few minutes and needs
internet. Later runs start immediately. If requirements.txt changes, the next
run updates `.venv` by itself.

Start a script with any Python 3.11+ (`python3 code/emg_arm_control.py`, the VS
Code Run button, ...). It restarts itself inside `.venv` with the same
arguments. Set EMG_ARM_NO_BOOTSTRAP=1 to skip all of this and use the
current Python as it is.
"""

import hashlib
import importlib.util
import os
import shutil
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
VENV_DIR = PROJECT_ROOT / ".venv"
REQUIREMENTS = PROJECT_ROOT / "requirements.txt"
STAMP = VENV_DIR / ".requirements.sha256"
MODULES = ("numpy", "PyQt5", "joblib", "serial", "sklearn", "xarm", "hid")
MIN_PYTHON = (3, 11)
_RELAUNCH_FLAG = "EMG_ARM_BOOTSTRAPPED"


def _venv_python():
    for candidate in (VENV_DIR / "bin" / "python", VENV_DIR / "Scripts" / "python.exe"):
        if candidate.exists():
            return candidate
    return None


def _in_venv():
    return Path(sys.prefix).resolve() == VENV_DIR.resolve()


def _requirements_hash():
    return hashlib.sha256(REQUIREMENTS.read_bytes()).hexdigest()


def _up_to_date():
    try:
        return STAMP.read_text().strip() == _requirements_hash()
    except OSError:
        return False


def _missing():
    return [m for m in MODULES if importlib.util.find_spec(m) is None]


def _fail(message):
    sys.exit(f"\n[setup] {message}\n")


def _create_venv():
    if sys.version_info < MIN_PYTHON:
        _fail(f"Python {sys.version.split()[0]} is too old. Install Python 3.11, 3.12 or 3.13 "
              "from https://www.python.org/downloads/ and run this script with it.")
    print(f"[setup] First run: creating the project environment in {VENV_DIR} …", flush=True)
    if VENV_DIR.exists():
        shutil.rmtree(VENV_DIR)  # broken / half-created environment
    try:
        subprocess.check_call([sys.executable, "-m", "venv", str(VENV_DIR)])
    except (subprocess.CalledProcessError, OSError) as exc:
        _fail(f"Could not create the environment ({exc}).\n"
              "On Ubuntu/Debian install the venv module first: sudo apt install python3-venv")


def _install(python):
    print("[setup] Installing the required libraries (one time, needs internet, a few minutes) …",
          flush=True)
    try:
        STAMP.unlink()
    except OSError:
        pass
    cmd = [str(python), "-m", "pip", "install", "--disable-pip-version-check", "-r", str(REQUIREMENTS)]
    if subprocess.call(cmd) != 0:
        _fail("Installing the libraries failed (see the pip messages above).\n"
              "Check the internet connection and run the script again. If it keeps failing, delete "
              f"{VENV_DIR} and try with Python 3.12.")
    STAMP.write_text(_requirements_hash())
    print("[setup] Done. Starting …\n", flush=True)


def _relaunch(python):
    os.environ[_RELAUNCH_FLAG] = "1"
    args = [str(python), os.path.abspath(sys.argv[0]), *sys.argv[1:]]
    sys.stdout.flush()
    sys.stderr.flush()
    if os.name == "nt":  # execv on Windows does not replace the console process cleanly
        sys.exit(subprocess.call(args))
    os.execv(str(python), args)


def ensure():
    """Make sure we run inside the project's .venv with every library installed."""
    if os.environ.get("EMG_ARM_NO_BOOTSTRAP"):
        return
    if _in_venv():
        if _up_to_date() and not _missing():
            return
        if os.environ.get(_RELAUNCH_FLAG) and not _missing():
            return  # just installed; the stamp could not be written (read-only?) — still fine
        _install(sys.executable)
        importlib.invalidate_caches()
        if _missing():
            _fail(f"Still missing after install: {', '.join(_missing())}.")
        return
    if os.environ.get(_RELAUNCH_FLAG):
        _fail(f"Could not start inside {VENV_DIR}. Delete that folder and run again.")
    python = _venv_python()
    if python is None:
        _create_venv()
        python = _venv_python()
        if python is None:
            _fail(f"The environment in {VENV_DIR} has no Python. Delete that folder and run again.")
    if not _up_to_date():
        _install(python)
    _relaunch(python)
