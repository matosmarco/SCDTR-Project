import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def process_and_align_b(filename):
    """Processa o ficheiro, converte us para s e alinha no degrau."""
    try:
        df = pd.read_csv(filename)
        # Conversão de micro-segundos para segundos
        df['time_s_raw'] = df['time_ms'] / 1_000_000.0
        
        # Detetar o primeiro degrau (referência vai para 30)
        step_indices = df[df['ref'] >= 29].index
        if len(step_indices) > 0:
            t_ref = df.loc[step_indices[0], 'time_s_raw']
        else:
            t_ref = df['time_s_raw'].min()
        
        df['time_s'] = df['time_s_raw'] - t_ref
        return df
    except Exception as e:
        print(f"Erro ao processar {filename}: {e}")
        return None

def calculate_b_metrics(df, tolerance=0.05):
    if df is None: return 0, 0
    post_step = df[(df['time_s'] >= 0) & (df['time_s'] <= 10)].copy()
    if post_step.empty: return 0, 0
    
    r_final = 30.0
    # Overshoot
    y_max = post_step['lux'].max()
    overshoot = max(0, (y_max - r_final) / r_final * 100)
    
    # Settling Time (t_s) - Banda de 5%
    band_hi, band_lo = r_final * (1 + tolerance), r_final * (1 - tolerance)
    outside = post_step[(post_step['lux'] < band_lo) | (post_step['lux'] > band_hi)]
    t_s = outside['time_s'].iloc[-1] if not outside.empty else 0
    
    return overshoot, t_s

# --- CONFIGURAÇÃO DOS FICHEIROS ---
files = ['b10.csv', 'b07.csv', 'b02.csv', 'b05.csv']
labels = ['$b = 1.0$ (No weighting)', '$b = 0.7$', '$b = 0.2$', '$b = 0.5$']

plt.figure(figsize=(12, 7))

for file, label in zip(files, labels):
    df = process_and_align_b(file)
    if df is not None:
        os_val, ts_val = calculate_b_metrics(df)
        plt.plot(df['time_s'], df['lux'], label=f'{label} (OS: {os_val:.1f}%, $t_s$: {ts_val:.2f}s)', linewidth=1.5)

# Plot da Referência
# (Usamos o último df carregado para desenhar a linha vermelha)
plt.step(df['time_s'], df['ref'], 'r--', label='Setpoint ($r$)', alpha=0.7, where='post')

# Estilização em Inglês para o Relatório
plt.title('PID Performance: Impact of Setpoint Weighting ($b$)', fontsize=14)
plt.xlabel('Time from Step [s]', fontsize=12)
plt.ylabel('Illuminance [Lux]', fontsize=12)
plt.legend(loc='upper right', frameon=True, fontsize=10)
plt.grid(True, which='both', linestyle='--', alpha=0.5)

# Ajuste de escala (mostra os primeiros 10 segundos para ver bem o efeito do b)
plt.xlim(-0.5, 20   )
plt.ylim(-2, 40)

plt.tight_layout()
plt.savefig('setpoint_weighting_comparison.png', dpi=300)
plt.show()