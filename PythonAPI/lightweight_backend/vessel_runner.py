"""ctypes wrapper around the engine-free C++ VesselRunner (no dynamics in Python)."""

import ctypes
import os
import subprocess
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_VESSEL_DIR = (_HERE.parent.parent / "Unreal" / "CarlaUnreal" / "Plugins" / "Carla"
               / "Source" / "Carla" / "Vessel")
_SHIM = _HERE / "vessel_runner_c.cpp"


def _library_path():
    cache = Path(os.environ.get("CARLA_MARINE_CACHE", Path.home() / ".cache" / "carla-marine"))
    cache.mkdir(parents=True, exist_ok=True)
    return cache / "libvessel_runner_c.so"


def _build(output):
    cmd = ["g++", "-std=c++17", "-O2", "-shared", "-fPIC", f"-I{_VESSEL_DIR}",
           str(_SHIM), "-o", str(output)]
    print("building vessel runner:", " ".join(cmd), file=sys.stderr)
    subprocess.run(cmd, check=True)


def _load():
    lib_path = _library_path()
    sources = [_SHIM, _VESSEL_DIR / "VesselRunner.h", _VESSEL_DIR / "VesselDynamicsModel.h"]
    newest = max(p.stat().st_mtime for p in sources)
    if not lib_path.exists() or lib_path.stat().st_mtime < newest:
        _build(lib_path)
    lib = ctypes.CDLL(str(lib_path))
    d = ctypes.c_double
    lib.vr_create.restype = ctypes.c_void_p
    lib.vr_destroy.argtypes = [ctypes.c_void_p]
    lib.vr_reset.argtypes = [ctypes.c_void_p, d, d, d]
    lib.vr_set_pose.argtypes = [ctypes.c_void_p, d, d, d]
    lib.vr_set_control.argtypes = [ctypes.c_void_p, d, d]
    lib.vr_advance.argtypes = [ctypes.c_void_p, d]
    lib.vr_advance.restype = ctypes.c_int
    lib.vr_get.argtypes = [ctypes.c_void_p, ctypes.POINTER(d)]
    return lib


_LIB = None


class VesselRunner:
    """Same stepping semantics as carla::vessel::VesselRunner."""

    def __init__(self, x=0.0, y=0.0, yaw_deg=0.0):
        global _LIB
        if _LIB is None:
            _LIB = _load()
        self._handle = _LIB.vr_create()
        _LIB.vr_reset(self._handle, x, y, yaw_deg)
        self._out = (ctypes.c_double * 7)()

    def __del__(self):
        if _LIB is not None and getattr(self, "_handle", None):
            _LIB.vr_destroy(self._handle)
            self._handle = None

    def set_pose(self, x, y, yaw_deg):
        _LIB.vr_set_pose(self._handle, x, y, yaw_deg)

    def set_control(self, rudder_rad, propeller_rad_s):
        _LIB.vr_set_control(self._handle, rudder_rad, propeller_rad_s)

    def advance(self, dt):
        return _LIB.vr_advance(self._handle, dt)

    def get(self):
        """Returns (north, east, yaw_deg, v_north, v_east, yaw_rate_deg_s, sim_time)."""
        _LIB.vr_get(self._handle, self._out)
        return tuple(self._out)
