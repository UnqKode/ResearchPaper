import os
import re

file_path = "compare_routing.py"
with open(file_path, "r") as f:
    content = f.read()

# Add the artifact warning logic right after the avoidance print statement
old_str = "print(f\"  {nameB} : {avoidance_B}/{total_B} ({rate_B:.1f}%)\")"
new_str = '''print(f"  {nameB} : {avoidance_B}/{total_B} ({rate_B:.1f}%)")
        
        # We will check the warning later in the function, after fuel savings are calculated.
        # Just store the rates in run_meta for the JSON
        run_meta[f"avoidance_ours"] = rate_A
        run_meta[f"avoidance_{nameB}"] = rate_B
'''
content = content.replace(old_str, new_str)

# At the end of analyze_paired_results, there's a JSON dump or a print of savings. Let's find "json_out".
old_json = "json_out = {"
new_json = '''
    mean_savings = sum(s) / len(s) if s else 0
    if degraded_edges and mean_savings > 0:
        if rate_A <= rate_B:
            print("\\n***************************************************************")
            print("                     SUSPECTED ARTIFACT")
            print("  Fuel was saved without a higher avoidance rate of degraded")
            print("  edges by ours. This violates the behavioural signature.")
            print("***************************************************************\\n")
            
    json_out = {'''

content = content.replace("json_out = {", new_json)

with open(file_path, "w") as f:
    f.write(content)
print("Updated compare_routing.py phase 7 artifact warning")
