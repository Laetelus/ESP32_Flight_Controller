from collections import deque
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import logging

# Set up logging
logging.basicConfig(filename="error_log.txt", level=logging.INFO, format='%(asctime)s: %(message)s')

# Configure serial connection
try:
    ser = serial.Serial('COM8', 11520, timeout=0.1)
except serial.SerialException as e:
    logging.error(f"Serial port initialization error: {e}")
    raise SystemExit(e)

# Initialize deques for data storage
roll_angles = deque(maxlen=100)
pitch_angles = deque(maxlen=100)

# Set up the matplotlib figure and axes
fig, ax = plt.subplots()
ax.set_title("Roll and Pitch Angles")
line1, = ax.plot([], [], label='Roll Angle [°]', color='orange')
line2, = ax.plot([], [], label='Pitch Angle [°]', color='purple')
ax.legend()
ax.set_ylim(-10, 10)
ax.set_xlim(0, 100)

# Initialize the plot lines
def init():
    line1.set_data([], [])
    line2.set_data([], [])
    return line1, line2

# Update function for the animation
def update(frame):
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if not line:
                break  # Break if no data is received
            if ',' not in line:
                logging.info(f"Ignored malformed line: {line}")
                continue  # Skip lines without delimiter
            parts = line.split(',')
            if len(parts) < 2 or any(not part.strip() for part in parts[:2]):
                logging.info(f"Incomplete data received: {line}")
                continue  # Skip incomplete data lines

            roll_angle, pitch_angle = map(float, parts[:2])
            roll_angles.append(roll_angle)
            pitch_angles.append(pitch_angle)
        except ValueError as e:
            logging.error(f"Error parsing data: {line}, with error: {e}")
            continue
        except Exception as e:
            logging.error(f"Unexpected error: {e}")
            continue

    xdata = range(len(roll_angles))
    line1.set_data(xdata, roll_angles)
    line2.set_data(xdata, pitch_angles)
    return line1, line2

# Handle closing of the plot window gracefully
def on_close(event):
    logging.info("Closing the application.")
    ser.close()

fig.canvas.mpl_connect('close_event', on_close)

# Create and start the animation
ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=20, cache_frame_data=False)

plt.show()
