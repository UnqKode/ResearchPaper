@echo off
REM ============================================================
REM run_k10_fuelspec.bat
REM Round-4 three-arm fuel-specificity campaign (k=10 seeds)
REM   arms: ours-fuel, ours-augtime, ablation  (ablation = reference)
REM   seeds 1-10, n=25 OD pairs, scale=2.0, teleport=300
REM   grade mode on corridor selected by per-seed saveState gate (k=20 candidates -> top-2)
REM   Round-4 changes:
REM     --vehicle-sample-mod 2  (was 5; 1/2 sampling fills baseline faster)
REM     --warmup-buffer 1200    (was 600; 20 min pre-departure RSU warmup)
REM     --grade-lead 600        (grade activates 600s before egos; was 300s)
REM   Fix K changes (zero-traffic corridor fix):
REM     OD-coverage filter removed; structural candidates sorted by network degree
REM     CORRIDOR_OCC_MIN=0.005 added to gate; _select_k raised from 6 to 20
REM
REM Launch from the BTP\ directory:
REM   cd /d C:\Users\LNMIIT\Desktop\SumoSimulation\ResearchPaper\BTP
REM   run_k10_fuelspec.bat
REM ============================================================

setlocal enabledelayedexpansion

set BTPDIR=%~dp0
if "%BTPDIR:~-1%"=="\" set BTPDIR=%BTPDIR:~0,-1%

set OUTLOG=%BTPDIR%\k10_fuelspec_run.txt
set ERRLOG=%BTPDIR%\k10_fuelspec_run.err.txt

echo [run_k10_fuelspec] Starting Round-4 three-arm fuel-specificity campaign >> "%OUTLOG%"
echo [run_k10_fuelspec] Seeds: 1-10  n=25  scale=2.0  teleport=300 >> "%OUTLOG%"
echo [run_k10_fuelspec] arms: ours-fuel,ours-augtime,ablation >> "%OUTLOG%"
echo [run_k10_fuelspec] Round-4: sample-mod=2 warmup-buffer=1200 grade-lead=600 >> "%OUTLOG%"

conda run --no-capture-output -n ml ^
    python -u -m Simulation.compare_routing ^
    --paired ^
    --seeds 1,2,3,4,5,6,7,8,9,10 ^
    --n 25 ^
    --road-condition grade ^
    --degraded-edges auto-nonbottleneck ^
    --n-degraded 2 ^
    --targeted-od ^
    --scale 2.0 ^
    --depart-start 21600 ^
    --teleport 300 ^
    --arms ours-fuel,ours-augtime,ablation ^
    --baseline ablation ^
    --fuel-aggregator median ^
    --fuel-hysteresis 0.01 ^
    --junction-weight 1.0 ^
    --cost-mode fuel ^
    --use-hysteresis ^
    --warmup-savestate ^
    --vehicle-sample-mod 2 ^
    --warmup-buffer 1200 ^
    --grade-lead 600 ^
    >> "%OUTLOG%" 2>> "%ERRLOG%"

echo [run_k10_fuelspec] Done. Exit code: %ERRORLEVEL% >> "%OUTLOG%"
echo Done. See %OUTLOG% and %ERRLOG%
endlocal
