import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def process_and_align(filename):
    """Lê os dados em micro-segundos e alinha o degrau no tempo t=0."""
    if not os.path.exists(filename):
        print(f"Aviso: {filename} não encontrado.")
        return None
    
    df = pd.read_csv(filename)
    # Conversão de micro-segundos para segundos
    df['time_s_raw'] = df['time_ms'] / 1_000_000.0
    
    # Detetar o degrau (quando a referência sobe para 30)
    step_indices = df[df['ref'] >= 29].index
    t_ref = df.loc[step_indices[0], 'time_s_raw'] if len(step_indices) > 0 else df['time_s_raw'].min()
    
    df['time_s'] = df['time_s_raw'] - t_ref
    return df

def get_performance_metrics(df, tolerance=0.05):
    """Calcula Overshoot e Settling Time."""
    if df is None: return 0, 0
    post_step = df[(df['time_s'] >= 0) & (df['time_s'] <= 8)].copy()
    if post_step.empty: return 0, 0
    
    r_final = 30.0
    y_max = post_step['lux'].max()
    overshoot = max(0, (y_max - r_final) / r_final * 100)
    
    # Settling Time (t_s) - Banda de 5%
    band_hi, band_lo = r_final * 1.05, r_final * 0.95
    outside = post_step[(post_step['lux'] < band_lo) | (post_step['lux'] > band_hi)]
    t_s = outside['time_s'].iloc[-1] if not outside.empty else 0
    
    return overshoot, t_s

# 1. Processar os 3 ficheiros
df_low = process_and_align('kp_with_ki_low.csv')
df_med = process_and_align('kp_with_ki_medium.csv')
df_high = process_and_align('kp_with_ki_high.csv')

# 2. Calcular Métricas
metrics = {}
for name, df in zip(['Low', 'Medium', 'High'], [df_low, df_med, df_high]):
    if df is not None:
        metrics[name] = get_performance_metrics(df)

# 3. Plotting
plt.figure(figsize=(12, 7))

# Curva Low
if df_low is not None:
    plt.plot(df_low['time_s'], df_low['lux'], color='tab:blue', 
             label=f'$K_i = 0.01$', linewidth=1.5)

# Curva Medium
if df_med is not None:
    plt.plot(df_med['time_s'], df_med['lux'], color='tab:orange', 
             label=f'$K_i= 0.05$ ($t_s$: {metrics["Medium"][1]:.2f}s)', linewidth=1.5)

# Curva High
if df_high is not None:
    plt.plot(df_high['time_s'], df_high['lux'], color='tab:green', 
             label=f'$K_i = 0.1$ (OS: {metrics["High"][0]:.1f}%, $t_s$: {metrics["High"][1]:.2f}s)', linewidth=1.5)

# Referência
ref_df = df_med if df_med is not None else df_low
plt.step(ref_df['time_s'], ref_df['ref'], 'r--', label='Setpoint ($r$)', alpha=0.7, where='post')

# Configurações do Gráfico
plt.title('PI Sensitivity Analysis: Comparative $K_i$ Response, for optimal $K_p$', fontsize=14)
plt.xlabel('Time from Step [s]', fontsize=12)
plt.ylabel('Illuminance [Lux]', fontsize=12)
plt.legend(loc='lower right', frameon=True, fontsize=10)
plt.grid(True, which='both', linestyle='--', alpha=0.5)
plt.xlim(-0.5, 30)
plt.ylim(-2, 45) # Limite superior aumentado para mostrar possível overshoot do Kp High

plt.tight_layout()
plt.savefig('final_kp_comparison_3curves.png', dpi=300)
plt.show()