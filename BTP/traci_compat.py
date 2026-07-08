"""Single import point for the TraCI API. Prefers in-process libsumo.

Set env var FORCE_TRACI=1 to use network TraCI (needed for sumo-gui
or multi-client debugging). libsumo constraints: one simulation per
process, no GUI — both satisfied by our one-worker-one-process pool.
The multiprocessing pool uses Windows default 'spawn' start method,
so each worker gets a fresh libsumo instance (no cross-process state).
"""
import os
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
