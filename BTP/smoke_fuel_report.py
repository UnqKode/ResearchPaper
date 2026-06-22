import xml.etree.ElementTree as ET, csv, sys, math

DEGRADED = {"153391#0", "153391#1"}

def parse_tripinfo(path):
    out = {}
    for t in ET.parse(path).getroot().iter("tripinfo"):
        vid = t.get("id", "")
        if not vid.startswith("ego_"):
            continue
        em = t.find("emissions")
        fuel_mg = float(em.get("fuel_abs", 0)) * 1000 if em is not None else None
        out[vid] = {
            "duration": float(t.get("duration", 0)),
            "routeLength": float(t.get("routeLength", 0)),
            "fuel_mg": fuel_mg,
        }
    return out

def parse_progress(path):
    rows = {}
    try:
        with open(path) as f:
            for row in csv.DictReader(f):
                rows[row["trip"]] = row
    except Exception as e:
        print(f"WARNING: {path}: {e}", file=sys.stderr)
    return rows

ours_ti = parse_tripinfo("ours_1_2.0_21600.0.tripinfo.xml")
abl_ti  = parse_tripinfo("ablation_1_2.0_21600.0.tripinfo.xml")
ours_pr = parse_progress("progress_ours_1_2.0_21600.0.csv")
abl_pr  = parse_progress("progress_ablation_1_2.0_21600.0.csv")

print(f"ours tripinfo ego trips  : {len(ours_ti)}")
print(f"ablation tripinfo ego    : {len(abl_ti)}")
print(f"ours progress rows       : {len(ours_pr)}")
print(f"ablation progress rows   : {len(abl_pr)}")
print()

all_trips = sorted(set(ours_ti) & set(abl_ti), key=lambda x: int(x.split("_")[1]))

# Per-trip table
print("trip | ours_fuel_mg | abl_fuel_mg | delta_fuel% | ours_dur_s | abl_dur_s | delta_dur% | o_avoid | a_avoid")
print("-" * 110)

fuel_diffs, time_diffs = [], []
ours_avoid_n, abl_avoid_n = 0, 0

for vid in all_trips:
    o = ours_ti[vid]
    a = abl_ti[vid]
    k = vid.split("_")[1]
    o_dr = ours_pr.get(k, {}).get("driven_edges", "").split("|")
    a_dr = abl_pr.get(k, {}).get("driven_edges", "").split("|")
    o_avoid = "Y" if not any(e in DEGRADED for e in o_dr if e) else "N"
    a_avoid = "Y" if not any(e in DEGRADED for e in a_dr if e) else "N"
    if o_avoid == "Y": ours_avoid_n += 1
    if a_avoid == "Y": abl_avoid_n  += 1

    df_str = "n/a"
    if o["fuel_mg"] and a["fuel_mg"] and a["fuel_mg"] > 0:
        df = 100.0 * (o["fuel_mg"] - a["fuel_mg"]) / a["fuel_mg"]
        fuel_diffs.append(df / 100.0)
        df_str = f"{df:+.1f}%"

    dt_str = "n/a"
    if a["duration"] > 0:
        dt = 100.0 * (o["duration"] - a["duration"]) / a["duration"]
        time_diffs.append(dt / 100.0)
        dt_str = f"{dt:+.1f}%"

    print(
        f"{k:4s} | {(o['fuel_mg'] or 0):12.0f} | {(a['fuel_mg'] or 0):11.0f} | "
        f"{df_str:11s} | {o['duration']:10.1f} | {a['duration']:9.1f} | "
        f"{dt_str:10s} | {o_avoid:7s} | {a_avoid}"
    )

print()
print("=== SUMMARY ===")
print(f"Gate C avoidance: ours={ours_avoid_n}/{len(all_trips)} "
      f"({100*ours_avoid_n/len(all_trips):.1f}%)  "
      f"ablation={abl_avoid_n}/{len(all_trips)} "
      f"({100*abl_avoid_n/len(all_trips):.1f}%)")

if fuel_diffs:
    mean_f = sum(fuel_diffs) / len(fuel_diffs)
    print(f"Net fuel vs ablation: mean={100*mean_f:+.1f}%  n={len(fuel_diffs)}")

if time_diffs:
    mean_t = sum(time_diffs) / len(time_diffs)
    print(f"Net time vs ablation: mean={100*mean_t:+.1f}%  n={len(time_diffs)}")

# Check FREEZE_BASELINE count in err log
import os
for errfile in ["ours_1_2.0_21600.0.errors.log", "ablation_1_2.0_21600.0.errors.log"]:
    if os.path.exists(errfile):
        with open(errfile) as f:
            content = f.read()
        fb = content.count("FREEZE_BASELINE")
        print(f"{errfile}: FREEZE_BASELINE mentions={fb}")
