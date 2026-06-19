@echo off
cd /d "c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario\BTP"
python -m Simulation.compare_routing --paired --baseline ablation --road-condition grade --degraded-edges 153391#0,153391#1 --targeted-od --seeds 1,2,3,4,5,6,7,8,9,10 --n 50 --depart-start 21600 --scale 2.0 --debug-cfs > ..\multiseed_out.txt 2> ..\multiseed_err.txt
echo Exit code: %ERRORLEVEL% >> ..\multiseed_out.txt
