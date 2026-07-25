"""Single import point for the TraCI API. Prefers in-process libsumo.

Set env var FORCE_TRACI=1 to use network TraCI (needed for sumo-gui
or multi-client debugging). libsumo constraints: one simulation per
process, no GUI — both satisfied by our one-worker-one-process pool.
The multiprocessing pool uses Windows default 'spawn' start method,
so each worker gets a fresh libsumo instance (no cross-process state).
"""
import os

# Fix U: ensure sumo_data/bin (PyPI wheel DLLs) is on the DLL search path before
# importing libsumo.  Python 3.8+ no longer searches PATH for DLLs loaded by
# extension modules; os.add_dll_directory() is the supported API.  The libsumo
# __init__.py checks for zlib.dll as a sentinel, but sumo_data 1.27.1 ships
# zlib-ng.dll instead, so the sentinel check silently fails and _libsumo.pyd
# cannot find libsumocpp.dll at import time.  We add the directory explicitly.
_dll_handles = []  # keep alive — add_dll_directory handle removed on GC
if hasattr(os, "add_dll_directory"):
    try:
        import sumo_data as _sd
        _sd_bin = os.path.join(_sd.__path__[0], "bin")
        if os.path.isdir(_sd_bin):
            _dll_handles.append(os.add_dll_directory(os.path.abspath(_sd_bin)))
    except ImportError:
        pass

if os.environ.get("FORCE_TRACI") == "1":
    import traci
    USING_LIBSUMO = False
else:
    try:
        import libsumo as traci
        USING_LIBSUMO = True
    except ImportError:
        import traci
        USING_LIBSUMO = False
