@echo off
:: Smoke test: 1 seed, n=25, grade mode, with debug-cfs.
:: Fixes vs previous failed runs:
::   - stay_awake.py prevents Windows sleep (10-min AC timer was breaking TraCI TCP).
::   - python -u = unbuffered stdout so progress appears immediately in smoke_out.txt.

cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario"
del stay_awake.stop 2>nul
start /min "stay_awake" python stay_awake.py

cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario\BTP"
python -u -m Simulation.compare_routing --paired --baseline ablation --road-condition grade --degraded-edges 153391#0,153391#1 --targeted-od --seeds 1 --n 25 --depart-start 21600 --scale 2.0 --debug-cfs --theta-fuel 0.05 --theta-time 0.10 > ..\smoke_out.txt 2> ..\smoke_err.txt
echo Exit code: %ERRORLEVEL% >> ..\smoke_out.txt

:: Signal the stay-awake helper to exit
cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario"
echo done > stay_awake.stop
