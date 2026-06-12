import pandas as pd
df = pd.read_csv('cfs_debug_ours_1_2.5_21600.0.csv')
print('Mean Multiplier:', df['multiplier'].mean())
