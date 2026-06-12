import pandas as pd
import numpy as np
import warnings; warnings.filterwarnings('ignore')

try:
    df_route = pd.read_csv('route_cfs_ours_1_2.5_21600.0.csv')
    
    print('--- PARTIAL ROUTE CFS METRICS ---')
    print(f'Total routing queries so far: {len(df_route)}')
    print(f'max_route_multiplier: {df_route["max_multiplier"].max():.4f}')
    mean_multi = df_route["mean_multiplier"].mean()
    print(f'Mean multiplier across all routing queries: {mean_multi:.4f}')
    
    df_cfs = pd.read_csv('cfs_debug_ours_1_2.5_21600.0.csv')
    locked = df_cfs["baseline_locked"].sum()
    total_edges = len(df_cfs)
    print(f'\nbaseline_locked=True fraction (edge level): {locked}/{total_edges} ({locked/total_edges*100:.1f}%)')
    
    # Compute C, F, S contributions for edges where multiplier > 1.05
    active = df_cfs[df_cfs["multiplier"] > 1.05]
    if len(active) > 0:
        c_mean = active["C"].mean() * 0.15 # alpha=0.15
        f_mean = active["F"].mean() * 0.20 # beta=0.20
        s_mean = active["S"].mean() * 0.20 # gamma=0.20
        total = c_mean + f_mean + s_mean
        if total > 0:
            print(f'\nBreakdown on active edges (C, F, S):')
            print(f'  Congestion: {c_mean/total*100:.1f}%')
            print(f'  Fuel:       {f_mean/total*100:.1f}%')
            print(f'  Stops:      {s_mean/total*100:.1f}%')
    else:
        print('\nNo edges with multiplier > 1.05 yet.')
        
except Exception as e:
    print(f'Error reading route_cfs: {e}')

try:
    df_ab = pd.read_csv('progress_ablation_1_2.5_21600.0.csv')
    df_ou = pd.read_csv('progress_ours_1_2.5_21600.0.csv')
    
    common = pd.merge(df_ab, df_ou, on="trip", suffixes=("_ab", "_ou"))
    
    print('\n--- PARTIAL PAIRED FUEL ---')
    print(f'Egos finished in BOTH arms: {len(common)}')
    if len(common) > 0:
        fuel_ab = common["fuel_ml_ab"].sum() if "fuel_ml_ab" in common else common["fuel_mg_ab"].sum() / 832.0
        fuel_ou = common["fuel_ml_ou"].sum() if "fuel_ml_ou" in common else common["fuel_mg_ou"].sum() / 832.0
        savings = (fuel_ab - fuel_ou) / fuel_ab * 100
        print(f'Fuel Savings so far: {savings:+.2f}%')
except Exception as e:
    print(f'Error reading progress: {e}')
