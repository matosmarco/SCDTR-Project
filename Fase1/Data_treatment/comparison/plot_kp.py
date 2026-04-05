import pandas as pd

import matplotlib.pyplot as plt



def load_and_process(filename):

    """Loads the CSV and converts time to seconds."""

    df = pd.read_csv(filename)

    # Ensure time starts at 0 for overlapping comparison

    df['time_s'] = (df['time_ms'] - df['time_ms'].min()) / 1e6

    return df



def plot_kp_comparison(files, labels):

    plt.figure(figsize=(10, 6))

    

    # Plot each test curve

    for file, label in zip(files, labels):

        try:

            df = load_and_process(file)

            plt.plot(df['time_s'], df['lux'], label=label, linewidth=1.5)

        except FileNotFoundError:

            print(f"Warning: {file} not found. Please run the capture script first.")



    # Plot the Reference (using the last file's reference column)

    # We use 'step' to show the discrete nature of the reference

    df_ref = load_and_process(files[0])

    plt.step(df_ref['time_s'], df_ref['ref'], 'r--', label='Reference ($r$)', 

             where='post', alpha=0.7, linewidth=1.2)



    # Graph Styling (English)

    plt.title('PID Sensitivity Analysis: Varying Proportional Gain ($K_p$)', fontsize=14)

    plt.xlabel('Time [s]', fontsize=12)

    plt.ylabel('Illuminance [Lux]', fontsize=12)

    plt.legend(loc='best', frameon=True)

    plt.grid(True, which='both', linestyle='--', alpha=0.5)

    

    # Set axis limits to focus on the step transitions

    plt.xlim(0, df_ref['time_s'].max())

    plt.ylim(-2, 40)

    

    plt.tight_layout()

    plt.savefig('kp_sensitivity_analysis.png', dpi=300)

    plt.show()



# --- MAIN EXECUTION ---

# Ensure these filenames match the ones saved by your capture script

files_to_compare = [

    'kp_low.csv',      # Data with Kp = 0.01

    'kp_medium.csv'  # Data with Kp = 0.05

    #'kp_high.csv'      # Data with Kp = 0.15 or 0.20

]



labels_to_show = [

    '$K_p = 0.01$',

    '$K_p = 0.05$ (Optimal)',

    '$K_p = 0.15$ (Overdamped/Aggressive)'

]



plot_kp_comparison(files_to_compare, labels_to_show)


