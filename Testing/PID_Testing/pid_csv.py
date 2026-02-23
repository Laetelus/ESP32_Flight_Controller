import serial
import csv
import subprocess  # For running another script

# Set up serial communication (adjust port and baud rate)
ser = serial.Serial('COM8', 115200)  # Replace with your correct port

# Open a CSV file to log the data
with open(r'flight_data.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    
    # Header for the CSV file
    writer.writerow(["pid_roll_setpoint", "pid_pitch_setpoint",
                     "angle_roll", "angle_pitch",
                     "pid_output_roll", "pid_output_pitch"])

    try:
        while True:
            # Read and decode the serial data
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            # Filter out non-numeric lines (e.g., boot messages)
            if any(x.isalpha() for x in line):
                continue

            # Process valid numeric data
            if line:
                try:
                    # Split the data into individual values
                    data = list(map(float, line.split(',')))

                    # Ensure the correct number of data points
                    if len(data) == 7:  # Expecting 7 values (millis, setpoints, angles, and outputs)
                        writer.writerow(data)
                        print(f"Logged data: {data}")
                    else:
                        print(f"Extra data received, trimming: {line[:7]}")
                        
                except ValueError:
                    print(f"Non-numeric data encountered: {line}")

    except KeyboardInterrupt:
        print("Logging stopped.")
    finally:
        ser.close()

# After logging is complete, call the pid_post_flight script
print("Calling pid_post_flight.py for post-flight analysis...")
subprocess.run(['python', 'pid_post_flight.py'], check=True)
