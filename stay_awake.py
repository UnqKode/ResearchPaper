"""
Prevent Windows from sleeping while a simulation is running.
Exits when 'stay_awake.stop' file appears next to this script.
"""
import ctypes
import time
import os

ES_CONTINUOUS        = 0x80000000
ES_SYSTEM_REQUIRED   = 0x00000001
ES_AWAYMODE_REQUIRED = 0x00000040
_KEEP_AWAKE = ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_AWAYMODE_REQUIRED

_HERE = os.path.dirname(os.path.abspath(__file__))
_STOP = os.path.join(_HERE, "stay_awake.stop")

def _set(flags):
    ctypes.windll.kernel32.SetThreadExecutionState(ctypes.c_uint32(flags))

# Remove any leftover stop file from a previous run
try:
    os.remove(_STOP)
except FileNotFoundError:
    pass

_set(_KEEP_AWAKE)
print(f"[stay_awake] PID={os.getpid()} keeping system awake; "
      f"create stay_awake.stop to exit", flush=True)

while not os.path.exists(_STOP):
    _set(_KEEP_AWAKE)
    time.sleep(10)

_set(ES_CONTINUOUS)
print("[stay_awake] stop signal received, exiting", flush=True)
