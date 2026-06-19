import xml.etree.ElementTree as ET
import os, math, csv

btp = r"c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario\BTP"

def parse_tripinfo(path):
    """Returns dict: ego_id -> {duration, fuel_abs, routeLength, arrived}."""
    if not os.path.exists(path): return {}
    out = {}
    try:
        tree = ET.parse(path)
        root = tree.getroot()
    except ET.ParseError:
        return {}
    for ti in root.iter("tripinfo"):
        vid = ti.get("id","")
        if not vid.startswith("ego_"): continue
        arrival = ti.get("arrival")
        dur = ti.get("duration")
        rl  = ti.get("routeLength")
        em  = ti.find("emissions")
        fuel = em.get("fuel_abs") if em is not None else None
        out[vid] = {
            "arrived": arrival is not None and arrival != "-1",
            "duration": float(dur) if dur else None,
            "fuel_abs": float(fuel) if fuel else None,
            "routeLength": float(rl) if rl else None,
        }
    return out

seeds = [1,2,3,4,5,6]
seed_stats = {}

print(f"{'Seed':>5}  {'N':>3}  {'Fuel_sav%':>10}  {'Time_sav%':>10}  "
      f"{'ours_mg':>10}  {'abl_mg':>10}")
print("-"*65)

for s in seeds:
    op = os.path.join(btp, f"ours_{s}_2.0_21600.0.tripinfo.xml")
    ap = os.path.join(btp, f"ablation_{s}_2.0_21600.0.tripinfo.xml")
    od = parse_tripinfo(op)
    ad = parse_tripinfo(ap)
    if not od or not ad: continue
    # pair on trips that both arms completed with fuel data
    paired = sorted(
        [k for k in od if k in ad
         and od[k]["arrived"] and ad[k]["arrived"]
         and od[k]["fuel_abs"] and ad[k]["fuel_abs"]
         and od[k]["duration"] and ad[k]["duration"]],
        key=lambda x: int(x.split("_")[1])
    )
    if not paired: continue
    of = [od[k]["fuel_abs"] for k in paired]
    af = [ad[k]["fuel_abs"] for k in paired]
    ot = [od[k]["duration"] for k in paired]
    at = [ad[k]["duration"] for k in paired]
    n  = len(paired)
    fuel_sav = 100*(sum(af)-sum(of))/sum(af)
    time_sav = 100*(sum(at)-sum(ot))/sum(at)
    seed_stats[s] = {"n":n, "fuel_sav":fuel_sav, "time_sav":time_sav,
                     "mean_of":sum(of)/n, "mean_af":sum(af)/n}
    print(f"{s:>5}  {n:>3}  {fuel_sav:>+10.2f}%  {time_sav:>+10.2f}%  "
          f"{sum(of)/n:>10.1f}  {sum(af)/n:>10.1f}")

K = len(seed_stats)
if K < 2:
    print(f"\nOnly {K} seeds; need ≥2 for CI"); exit()

fs = [v["fuel_sav"] for v in seed_stats.values()]
ts = [v["time_sav"] for v in seed_stats.values()]
fm = sum(fs)/K; tm = sum(ts)/K
fvar = sum((x-fm)**2 for x in fs)/(K-1)
tvar = sum((x-tm)**2 for x in ts)/(K-1)
t_crit = {1:12.706,2:4.303,3:3.182,4:2.776,5:2.571,6:2.447,7:2.365,8:2.306,9:2.262}
tc = t_crit.get(K-1, 2.262)
fse = math.sqrt(fvar/K); tse = math.sqrt(tvar/K)
ft  = fm/fse;             tt  = tm/tse
fdz = fm/math.sqrt(fvar); tdz = tm/math.sqrt(tvar)
wins_f = sum(1 for x in fs if x > 0)
wins_t = sum(1 for x in ts if x > 0)

print(f"\n=== PARTIAL K={K} seeds (df={K-1}, t_crit={tc:.3f}) ===")
print(f"Fuel saving: {fm:+.2f}%  95%CI=[{fm-tc*fse:+.2f}%, {fm+tc*fse:+.2f}%]  "
      f"t={ft:+.3f}  d_z={fdz:+.3f}  ours_better={wins_f}/{K}")
print(f"Time saving: {tm:+.2f}%  95%CI=[{tm-tc*tse:+.2f}%, {tm+tc*tse:+.2f}%]  "
      f"t={tt:+.3f}  d_z={tdz:+.3f}  ours_better={wins_t}/{K}")
print(f"(positive = ours saves fuel/time vs ablation)")
