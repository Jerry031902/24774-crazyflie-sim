"""Compile/load the actual controller sources for CPU boundary and dynamics tests.

Host tests use the repository's 47-17 defaults and UNIT_TEST_MODE. This is not
an STM32 build or a PhysX run. CC may be "gcc", "clang", or a path to zig.exe.
"""
import argparse
import ctypes
import os
from pathlib import Path
import shutil
import subprocess
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/"build/lqr/native"

def build():
    OUT.mkdir(parents=True, exist_ok=True)
    config = OUT/"autoconf.h"
    config.write_text("#define CONFIG_PLATFORM_CF2 1\n"
                      "#define CONFIG_CRAZYFLIE_21_PLUS 1\n"
                      "#define CONFIG_ENABLE_THRUST_BAT_COMPENSATED 1\n", encoding="utf-8")
    cc = os.environ.get("LQR_CC") or shutil.which("gcc") or shutil.which("clang")
    if not cc:
        raise RuntimeError("Install GCC/Clang or set LQR_CC to zig.exe")
    command = [cc] + (["cc"] if Path(cc).stem == "zig" else [])
    library = OUT/("lqr_native.dll" if os.name == "nt" else "liblqr_native.so")
    includes = ["src/modules/interface", "src/modules/interface/controller",
                "src/utils/interface", "src/utils/interface/lighthouse",
                "src/hal/interface", "src/drivers/interface", "src/platform/interface",
                "src/config", str(OUT)]
    sources = ["sim/lqr/native_bridge.c", "src/modules/src/controller/horizontal_lqr.c",
               "src/modules/src/controller/controller_pid.c",
               "src/modules/src/controller/position_controller_pid.c",
               "src/modules/src/controller/attitude_pid_controller.c",
               "src/modules/src/power_distribution_quadrotor.c",
               "src/utils/src/pid.c", "src/utils/src/filter.c", "src/utils/src/num.c"]
    command += ["-shared", "-O2", "-std=gnu11", "-DUNIT_TEST_MODE",
                "-Wno-address-of-packed-member", "-fno-strict-aliasing"]
    if os.name != "nt": command += ["-fPIC"]
    command += ["-I"+str(ROOT/i) for i in includes] + sources + ["-o", str(library), "-lm"]
    subprocess.run(command, cwd=ROOT, check=True)
    return library

class NativeController:
    def __init__(self, lqr=False, library=None):
        # Process-global C state: never interleave two controller instances.
        library = library or OUT/("lqr_native.dll" if os.name == "nt" else "liblqr_native.so")
        self.lib = ctypes.CDLL(str(library))
        ptr = ctypes.POINTER(ctypes.c_float)
        self.lib.bridgeReset.argtypes = [ctypes.c_int]
        self.lib.bridgeReset.restype = None
        self.lib.bridgeEnable.argtypes = [ctypes.c_int]
        self.lib.bridgeEnable.restype = None
        self.lib.bridgeStep.argtypes = [ptr, ptr] + [ctypes.c_int]*4 + [ctypes.c_uint, ptr]
        self.lib.bridgeStep.restype = None
        self.lib.bridgeReset(int(lqr))
    def step(self, state, reference, tick=0, velocity=False, body=False, stop=False, brake=False):
        state = np.asarray(state, dtype=np.float32)
        ref = np.asarray(reference, dtype=np.float32)
        if state.shape != (12,) or ref.shape != (6,):
            raise ValueError("Expected state[12], reference[6]")
        out = np.zeros(18, dtype=np.float32)
        ptr = ctypes.POINTER(ctypes.c_float)
        self.lib.bridgeStep(state.ctypes.data_as(ptr), ref.ctypes.data_as(ptr),
                            int(velocity), int(body), int(stop), int(brake),
                            tick, out.ctypes.data_as(ptr))
        return out

if __name__ == "__main__":
    print(build())

