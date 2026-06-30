import csv

rows = list(csv.DictReader(open('paired_per_ego_20260624_003235_ours_vs_ablation.csv')))

for seed in [5, 6, 7]:
    ours = {int(r['trip']): r for r in rows if r['arm']=='ours'     and int(r['seed'])==seed}
    abla = {int(r['trip']): r for r in rows if r['arm']=='ablation' and int(r['seed'])==seed}

    paired = [t for t in ours if t in abla
              and ours[t]['arrived']=='True'
              and abla[t]['arrived']=='True'
              and ours[t].get('fuel_abs') not in ('','None',None)
              and abla[t].get('fuel_abs') not in ('','None',None)]

    if not paired:
        print(f'Seed {seed}: ours={len(ours)} trips, ablation={len(abla)} trips  --  NO PAIRED DATA')
        continue

    o_fuel = [float(ours[t]['fuel_abs']) for t in paired]
    b_fuel = [float(abla[t]['fuel_abs']) for t in paired]
    o_dur  = [float(ours[t]['duration'])  for t in paired]
    b_dur  = [float(abla[t]['duration'])  for t in paired]

    fuel_sav = [100*(b-o)/b for o,b in zip(o_fuel, b_fuel)]
    dur_sav  = [100*(b-o)/b for o,b in zip(o_dur,  b_dur)]

    mof = sum(o_fuel)/len(o_fuel)
    mbf = sum(b_fuel)/len(b_fuel)
    mod = sum(o_dur) /len(o_dur)
    mbd = sum(b_dur) /len(b_dur)
    mfs = sum(fuel_sav)/len(fuel_sav)
    mds = sum(dur_sav) /len(dur_sav)

    print(f'--- Seed {seed}  (n_paired={len(paired)}) ---')
    print(f'  Fuel : ours={mof:9,.1f} mg   ablation={mbf:9,.1f} mg   saving={mfs:+.2f}%   diff={mbf-mof:+,.1f} mg/trip')
    print(f'  Time : ours={mod:7.1f} s    ablation={mbd:7.1f} s    saving={mds:+.2f}%   diff={mbd-mod:+.1f} s/trip')
