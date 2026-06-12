import os
import sys
import sumolib
from collections import defaultdict
import subprocess

def test_imports():
    from rsu import RSUManager
    from edgecost import EdgeCostCalculator
    from network_builder import NetworkBuilder
    print("Imports OK")

test_imports()
