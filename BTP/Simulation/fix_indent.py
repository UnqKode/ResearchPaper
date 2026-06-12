import os
import re

file_path = "compare_routing.py"
with open(file_path, "r") as f:
    content = f.read()

content = content.replace("\nargs = ap.parse_args()", "\n    args = ap.parse_args()")

with open(file_path, "w") as f:
    f.write(content)
