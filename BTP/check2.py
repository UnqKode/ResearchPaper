import pandas as pd
import warnings; warnings.filterwarnings('ignore')

try:
    df_cfs = pd.read_csv('cfs_debug_ours_1_2.5_21600.0.csv')
    
    print('--- PARTIAL ROUTE CFS METRICS ---')
    print(f'Total edge evaluations so far: {len(df_cfs)}')
    print(f'max_multiplier observed on any edge: {df_cfs["multiplier"].max():.4f}')
    
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
    print(f'Error reading cfs_debug: {e}')
