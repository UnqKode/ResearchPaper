import os
import re

file_path = "compare_routing.py"
with open(file_path, "r") as f:
    content = f.read()

# 1. Add args to main()
args_insert_idx = content.find("args = ap.parse_args()")
new_args = '''
    # Phase 2: Degraded Road args
    ap.add_argument("--road-condition", choices=["none", "rough", "accident"], default="none", help="degradation mode")
    ap.add_argument("--degraded-edges", type=str, default="", help="comma-separated list of edges, or 'auto'")
    ap.add_argument("--n-degraded", type=int, default=3, help="k for auto-selection")
    ap.add_argument("--degrade-start", type=float, default=-1, help="time to start degradation (-1 = depart_start - 300)")
    ap.add_argument("--degrade-vlow", type=float, default=5.0, help="v_low for rough mode")
    ap.add_argument("--degrade-vhigh", type=float, default=12.0, help="v_high for rough mode")
    ap.add_argument("--degrade-period", type=float, default=20.0, help="period for rough mode")
    ap.add_argument("--event-duration", type=float, default=600.0, help="duration for accident mode")
    ap.add_argument("--targeted-od", action="store_true", help="force OD generation to target degraded edges")
    ap.add_argument("--verify-degradation", action="store_true", help="Gate A verification: run headless and check fuel per edge")
    
'''
content = content[:args_insert_idx] + new_args + content[args_insert_idx:]

with open(file_path, "w") as f:
    f.write(content)
print("Updated compare_routing.py phase 2 args")
