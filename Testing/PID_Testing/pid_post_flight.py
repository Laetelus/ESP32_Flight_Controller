import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Define the moving average function
def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size) / window_size, mode='valid')

# Function to pad the beginning of the smoothed data with NaNs to match the original length
def pad_smoothed_data(smoothed_data, original_length):
    padding = [np.nan] * (original_length - len(smoothed_data))
    return np.concatenate([padding, smoothed_data])

# Default path assuming it's in the subfolder
csv_path_default = r'./PID_Testing/flight_data.csv'

# Path if the script is called directly in the current directory
csv_path_direct = r'flight_data.csv'

# Check if the script is being run by pid_csv Python script or directly
if os.path.exists(csv_path_direct):
    df = pd.read_csv(csv_path_direct)
elif os.path.exists(csv_path_default):
    df = pd.read_csv(csv_path_default)
else:
    raise FileNotFoundError(f"Neither '{csv_path_direct}' nor '{csv_path_default}' were found. Ensure the paths are correct.")

# Apply the moving average filter to smooth the data (adjust window size as needed)
window_size = 10  # You can adjust this value for more or less smoothing

# Create smoothed data and pad it to match the original length
smoothed_roll = moving_average(df['angle_roll'], window_size)
df['smoothed_roll'] = pad_smoothed_data(smoothed_roll, len(df))

smoothed_pitch = moving_average(df['angle_pitch'], window_size)
df['smoothed_pitch'] = pad_smoothed_data(smoothed_pitch, len(df))

# Optionally, you can smooth yaw as well if you want to test it in the future
# smoothed_yaw = moving_average(df['gyro_yaw_input'], window_size)
# df['smoothed_yaw'] = pad_smoothed_data(smoothed_yaw, len(df))

def plot_pid_vs_response(df):
    fig, axs = plt.subplots(2, 1, figsize=(10, 10))

    # Plot raw roll setpoint vs actual
    axs[0].plot(df.index, df['angle_roll'], label="Roll Actual", color="blue", alpha=0.5)
    axs[0].plot(df.index, df['pid_roll_setpoint'], label="Roll Setpoint", color="green")
    axs[0].set_title('Roll: Setpoint vs Actual (without smoothing)')
    axs[0].legend()

    # Plot raw pitch setpoint vs actual
    axs[1].plot(df.index, df['angle_pitch'], label="Pitch Actual", color="blue", alpha=0.5)
    axs[1].plot(df.index, df['pid_pitch_setpoint'], label="Pitch Setpoint", color="green")
    axs[1].set_title('Pitch: Setpoint vs Actual (without smoothing)')
    axs[1].legend()

    # Uncomment this section when you're ready to test yaw
    # axs[2].plot(df.index, df['smoothed_yaw'], label="Smoothed Yaw Actual", color="red")
    # axs[2].plot(df.index, df['pid_yaw_setpoint'], label="Yaw Setpoint", color="green")
    # axs[2].plot(df.index, df['gyro_yaw_input'], label="Yaw Actual", color="blue", alpha=0.5)
    # axs[2].set_title('Yaw: Setpoint vs Actual (with smoothing)')
    # axs[2].legend()

    plt.tight_layout()
    plt.show()

# Call the function to display the graphs
plot_pid_vs_response(df)
