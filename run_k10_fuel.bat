@echo off
:: Definitive K=10 fuel-mode run.
:: Same engine confirmed: both arms use our Dijkstra + 30s cadence + same placement.
:: Only difference: ours=fuel costs, ablation=L/avg_speed (pure travel time).
:: seed-parallelism 3: 3 seeds x 2 arms = 6 concurrent SUMO instances, ~20h wall-clock.

cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario"
del stay_awake.stop 2>nul
del k10_fuel.txt 2>nul
start /min "stay_awake" python stay_awake.py

cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario\BTP"
python -u -m Simulation.compare_routing --paired --baseline ablation --cost-mode fuel --road-condition grade --degraded-edges 153391#0,153391#1 --targeted-od --seeds 1,2,3,4,5,6,7,8,9,10 --n 25 --depart-start 21600 --scale 2.0 --debug-cfs --teleport 300 --seed-parallelism 3 > ..\k10_fuel.txt 2>&1
echo Exit code: %ERRORLEVEL% >> ..\k10_fuel.txt

cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario"
echo done > stay_awake.stop
