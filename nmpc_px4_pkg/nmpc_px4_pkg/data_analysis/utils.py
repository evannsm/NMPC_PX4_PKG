import matplotlib.pyplot as plt
import numpy as np

LOOKAHEAD = 300

def rmse(df):
    """
    Calculate the overall RMSE across x, y, z, and yaw compared to their reference values.
    
    Parameters:
    df (pandas.DataFrame): DataFrame containing the actual values and reference values.
    
    Returns:
    float: The overall RMSE across all dimensions.
    """
    # Extract actual and reference values as numpy arrays
    actual_values = df[['x', 'y', 'z', 'yaw']].to_numpy()
    reference_values = df[['x_ref', 'y_ref', 'z_ref', 'yaw_ref']].to_numpy()
    actual_values = actual_values[LOOKAHEAD:, :]
    reference_values = reference_values[:-LOOKAHEAD, :]
    actual_values[:,3] = actual_values[:,3] * .18
    reference_values[:,3] = reference_values[:,3] * .18

    # Compute the squared differences
    squared_errors = (actual_values - reference_values) ** 2
    rmse = np.sqrt(np.sum(squared_errors, axis=1).mean())
    
    return rmse

def make_plot(df):
    fig, axs = plt.subplots(4, 4, figsize=(20, 12), sharex=False)
    time_max = df['time'].max()
    time_min = df['time'].min()
    x_lim = (time_min-1, time_max+1)
    
# Row 1: Plot x, y, z, psi vs references
    # plot x vs x_ref
    axs[0, 0].plot(df['time'][:-LOOKAHEAD], df['x'][LOOKAHEAD:], label='x', color='red')
    axs[0, 0].plot(df['time'][:-LOOKAHEAD], df['x_ref'][:-LOOKAHEAD], label='x_ref', color='blue', linestyle='--')
    axs[0, 0].set_ylabel('x / x_ref')
    axs[0, 0].set_xlabel('time')
    axs[0, 0].legend()
    axs[0, 0].set_xlim(x_lim)

    # plot y vs y_ref
    axs[0, 1].plot(df['time'][:-LOOKAHEAD], df['y'][LOOKAHEAD:], label='y', color='red')
    axs[0, 1].plot(df['time'][:-LOOKAHEAD], df['y_ref'][:-LOOKAHEAD], label='y_ref', color='blue', linestyle='--')
    axs[0, 1].set_ylabel('y / y_ref')
    axs[0, 1].set_xlabel('time')
    axs[0, 1].legend()
    axs[0, 1].set_xlim(x_lim)

    # plot z vs z_ref
    axs[0, 2].plot(df['time'][:-LOOKAHEAD], -1*df['z'][LOOKAHEAD:], label='z', color='red')
    axs[0, 2].plot(df['time'][:-LOOKAHEAD], -1*df['z_ref'][:-LOOKAHEAD], label='z_ref', color='blue', linestyle='--')
    axs[0, 2].set_ylabel('z / z_ref')
    axs[0, 2].set_xlabel('time')
    axs[0, 2].legend()
    axs[0, 2].set_xlim(x_lim)
    axs[0, 2].set_ylim(0,-1*df['z'].min()+.1)


    # plot psi vs psi_ref
    axs[0, 3].plot(df['time'][:-LOOKAHEAD], df['yaw'][LOOKAHEAD:], label='psi', color='red')
    axs[0, 3].plot(df['time'][:-LOOKAHEAD], df['yaw_ref'][:-LOOKAHEAD], label='psi_ref', color='blue', linestyle='--')
    axs[0, 3].set_ylabel('psi / psi_ref')
    axs[0, 3].set_xlabel('time')
    axs[0, 3].legend()
    axs[0, 3].set_xlim(x_lim)



# Row 2: Plot cross comparisons (x vs y, x vs z, y vs z, time vs solve_time)
    # plot x vs y and x_ref vs y_ref
    axs[1, 0].plot(df['x'], df['y'], label='x vs y', color='red')
    axs[1, 0].plot(df['x_ref'], df['y_ref'], label='x_ref vs y_ref', color='blue', linestyle='--')
    axs[1, 0].set_ylabel('y')
    axs[1, 0].set_xlabel('x')
    axs[1,0].set_ylim(-.5, .5)
    axs[1, 0].set_xlim(-.5, .5)
    axs[1, 0].legend()

    # plot x vs z and x_ref vs z_ref
    axs[1, 1].plot(df['x'], -1*df['z'], label='x vs z', color='red')
    axs[1, 1].plot(df['x_ref'], -1*df['z_ref'], label='x_ref vs z_ref', color='blue', linestyle='--')
    axs[1, 1].set_ylabel('z')
    axs[1, 1].set_xlabel('x')
    axs[1, 1].set_ylim(0,-1*df['z'].min()+.1)
    axs[1, 1].legend()

    # plot y vs z and y_ref vs z_ref
    axs[1, 2].plot(df['y'], -1*df['z'], label='y vs z', color='red')
    axs[1, 2].plot(df['y_ref'], -1*df['z_ref'], label='y_ref vs z_ref', color='blue', linestyle='--')
    axs[1, 2].set_ylabel('z')
    axs[1, 2].set_xlabel('y')
    axs[1, 2].set_ylim(0,-1*df['z'].min()+.1)
    axs[1, 2].legend()

    # plot time vs solve_time
    axs[1, 3].plot(df['time'], df['mpc_time']* 1e-9, label='mpc_time', color='purple')
    axs[1, 3].set_ylabel('mpc_time')
    axs[1, 3].set_xlabel('time')
    axs[1, 3].legend()
    axs[1, 3].set_xlim(x_lim)



# Row 3: Plot throttle, and roll/pitch/yaw-rates vs time

    max_throttle = 1.0
    max_rate = 0.8
    ylim_throttle = (-0.2, 1.2)
    ylim_rates = (-1.0, 1.0)

    # plot throttle vs time
    axs[2, 0].plot(df['time'], -1*df['throttle'], label='throttle', color='blue')
    axs[2, 0].axhline(y=max_throttle, color='red', linestyle='--', label=f'+{max_throttle}')
    axs[2, 0].axhline(y=0, color='red', linestyle='--', label=f'{0}')
    axs[2, 0].set_ylabel('throttle')
    axs[2, 0].set_xlabel('time')
    axs[2, 0].legend()
    axs[2, 0].set_ylim(ylim_throttle)
    axs[2, 0].set_xlim(x_lim)
    axs[2, 0].set_yticks(np.arange(ylim_throttle[0], ylim_throttle[1] + 0.1, 0.2))


    # plot roll_rate vs time
    axs[2, 1].plot(df['time'], df['roll_rate'], label='roll_rate', color='orange')
    axs[2, 1].axhline(y=max_rate, color='red', linestyle='--', label=f'+{max_rate}')
    axs[2, 1].axhline(y=-max_rate, color='red', linestyle='--', label=f'-{max_rate}')
    axs[2, 1].set_ylabel('roll_rate')
    axs[2, 1].set_xlabel('time')
    axs[2, 1].legend()
    axs[2, 1].set_ylim(ylim_rates)
    axs[2, 1].set_xlim(x_lim)
    axs[2, 1].set_yticks(np.arange(ylim_rates[0], ylim_rates[1] + 0.1, 0.2))


    # plot pitch_rate vs time
    axs[2, 2].plot(df['time'], df['pitch_rate'], label='pitch_rate', color='green')
    axs[2, 2].axhline(y=max_rate, color='red', linestyle='--', label=f'+{max_rate}')
    axs[2, 2].axhline(y=-max_rate, color='red', linestyle='--', label=f'-{max_rate}')
    axs[2, 2].set_ylabel('pitch_rate')
    axs[2, 2].set_xlabel('time')
    axs[2, 2].legend()
    axs[2, 2].set_ylim(ylim_rates)
    axs[2, 2].set_xlim(x_lim)
    axs[2, 2].set_yticks(np.arange(ylim_rates[0], ylim_rates[1] + 0.1, 0.2))

    # plot yaw_rate vs time
    axs[2, 3].plot(df['time'], df['yaw_rate'], label='yaw_rate', color='purple')
    axs[2, 3].axhline(y=max_rate, color='red', linestyle='--', label=f'+{max_rate}')
    axs[2, 3].axhline(y=-max_rate, color='red', linestyle='--', label=f'-{max_rate}')
    axs[2, 3].set_ylabel('yaw_rate')
    axs[2, 3].set_xlabel('time')
    axs[2, 3].legend()
    axs[2, 3].set_ylim(ylim_rates)
    axs[2, 3].set_xlim(x_lim)
    axs[2, 3].set_yticks(np.arange(ylim_rates[0], ylim_rates[1] + 0.1, 0.2))

    # Row 4: plot pred_time * mpc_time vs time
    # plot mpc_time vs time
    axs[3, 0].plot(df['time'][1:], df['mpc_time'][1:], label='mpc_time', color='orange')
    axs[3, 0].set_ylabel('mpc_time')
    axs[3, 0].set_xlabel('time')
    axs[3, 0].legend()
    axs[3, 0].set_xlim(x_lim)    

    # # plot pred_time vs time
    # axs[3, 1].plot(df['time'][1:], df['pred_time'][1:], label='pred_time', color='blue')
    # axs[3, 1].set_ylabel('pred_time')
    # axs[3, 1].set_xlabel('time')
    # axs[3, 1].legend()
    # axs[3, 1].set_xlim(x_lim)