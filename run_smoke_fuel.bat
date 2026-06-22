@echo off
:: Calibration check for --cost-mode fuel.
:: Adds [DRIVEN_MATCH] to test whether per-segment fuel cost matches realized fuel
:: on the actual driven route (not the injection-time snapshot).
:: Also fixes in_bypass logging bug and adds trip-11 non-arrival diagnosis.

cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario"
del stay_awake.stop 2>nul
del calib_check.txt 2>nul
start /min "stay_awake" python stay_awake.py

cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario\BTP"
python -u -m Simulation.compare_routing --paired --baseline ablation --road-condition grade --degraded-edges 153391#0,153391#1 --targeted-od --seeds 1 --n 25 --depart-start 21600 --scale 2.0 --debug-cfs --cost-mode fuel --teleport 300 > ..\calib_check.txt 2>&1
echo Exit code: %ERRORLEVEL% >> ..\calib_check.txt

cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario"
echo done > stay_awake.stop
