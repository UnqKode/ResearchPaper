@echo off
:: Full K=10 multi-seed run: seeds 1-10, n=25, seed-parallelism=3, no debug-cfs.
:: Fixes vs previous failed runs:
::   - stay_awake.py prevents Windows sleep (10-min AC timer was breaking TraCI TCP).
::   - python -u = unbuffered stdout so progress appears immediately.

cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario"
del stay_awake.stop 2>nul
start /min "stay_awake" python stay_awake.py

cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario\BTP"
python -u -m Simulation.compare_routing --paired --baseline ablation --road-condition grade --degraded-edges 153391#0,153391#1 --targeted-od --seeds 1,2,3,4,5,6,7,8,9,10 --n 25 --depart-start 21600 --scale 2.0 --seed-parallelism 3 > ..\multiseed_v2_out.txt 2> ..\multiseed_v2_err.txt
echo Exit code: %ERRORLEVEL% >> ..\multiseed_v2_out.txt

:: Signal the stay-awake helper to exit
cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario"
echo done > stay_awake.stop
