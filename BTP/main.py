"""
main.py — single-trip entry point for the fuel-aware ego-routing experiment.

Intended launch commands (run from the project root BTP/):
    python main.py
    python -m main

Do NOT cd into a subdirectory before running; file paths are resolved relative
to this file's location so any CWD inside BTP/ or its parent will work.
"""
import os
import sys

# 1. Locate the SUMO tools library using the SUMO_HOME environment variable
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Error: Please declare the environment variable 'SUMO_HOME'")

import traci

from Simulation.simulate import Simulation


def main():
    # 2. Configuration / network file paths
    # Resolve relative to the repo root (MoSTScenario/), which is one directory
    # above the BTP/ folder where this file lives:
    #   __file__  ->  BTP/main.py
    #   _here     ->  BTP/
    #   _root     ->  MoSTScenario/   <-- scenario files live here
    _here       = os.path.dirname(os.path.abspath(__file__))   # BTP/
    _root       = os.path.dirname(_here)                       # MoSTScenario/
    config_file = os.path.join(_root, "scenario", "most.sumocfg")
    net_file    = os.path.join(_root, "scenario", "in", "most.net.xml")

    # 3. SUMO launch command.
    #    Use "sumo" instead of "sumo-gui" to run headless (faster, no UI) --
    #    do that for the alpha/beta/gamma tuning runs.
    sumo_cmd = ["sumo", "-c", config_file]

    started = False
    try:
        # 4. Build the integrated simulation (reads the net offline via sumolib;
        #    does not need TraCI yet).
        sim = Simulation(
            net_file=net_file,
            reroute_interval=30,   # recompute weights & reroute every 30 steps
            alpha=1.0, beta=0.8, gamma=1.5,
        )

        # 5. Start TraCI, then hand control to the orchestrator.
        traci.start(sumo_cmd)
        started = True

        steps = sim.run()        # pass max_steps=N to cap the run length
        print(f"\nSimulation finished after {steps} steps.")

    except traci.FatalTraCIError as e:
        print(f"\nTraCI Error: {e}")
        print("Check that SUMO is installed and the path to your .sumocfg file is correct.")

    finally:
        # 6. Always close TraCI cleanly to free the port -- but only if it started.
        if started:
            traci.close()
            sys.stdout.flush()
            print("\nSimulation closed.")


if __name__ == "__main__":
    main()
