import csv, os, math

btp = r"c:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario\BTP"
stamp = "20260619_105700"

print(f"{'Seed':>5}  {'ours_avoid%':>12}  {'abl_avoid%':>12}  {'diff_pp':>8}  ours>abl?")
print("-"*60)

seed_diffs = []
for s in range(1, 11):
    f = os.path.join(btp, f"crossing_diag_{stamp}_{s}.csv")
    if not os.path.exists(f): continue
    rows = list(csv.DictReader(open(f)))
    o_rows = [r for r in rows if r["arm"]=="ours"]
    a_rows = [r for r in rows if r["arm"]=="ablation"]
    o_avoid = sum(1 for r in o_rows if r["crossed"]=="False")
    a_avoid = sum(1 for r in a_rows if r["crossed"]=="False")
    o_n = len(o_rows); a_n = len(a_rows)
    if o_n == 0 or a_n == 0: continue
    op = 100*o_avoid/o_n; ap = 100*a_avoid/a_n
    diff = op - ap
    seed_diffs.append(diff)
    print(f"{s:>5}  {op:>11.1f}%  {ap:>11.1f}%  {diff:>+7.1f}pp  {'YES' if diff>0 else 'no'}")

K = len(seed_diffs)
if K > 1:
    m  = sum(seed_diffs)/K
    s2 = sum((x-m)**2 for x in seed_diffs)/(K-1)
    tc = {1:12.706,2:4.303,3:3.182,4:2.776,5:2.571,6:2.447,7:2.365,8:2.306,9:2.262}.get(K-1,2.262)
    se = math.sqrt(s2/K)
    t  = m/se
    dz = m/math.sqrt(s2)
    wins = sum(1 for x in seed_diffs if x > 0)
    print(f"\n=== Avoidance gap (ours - ablation), K={K} seeds (df={K-1}) ===")
    print(f"Mean diff: {m:+.2f}pp  95%CI=[{m-tc*se:+.2f}, {m+tc*se:+.2f}]pp")
    print(f"t={t:+.3f}  d_z={dz:+.3f}  seeds_ours>ablation={wins}/{K}")
