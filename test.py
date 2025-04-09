import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

# Serial port configuration
SERIAL_PORT = 'COM13'  # Replace with your Arduino's serial port
BAUD_RATE = 9600

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Allow time for the connection to establish

# Data storage
time_data = []
distance_data = []
rps_left_data = []
rps_right_data = []
speed_left_data = []
speed_right_data = []
error_left_data = []
error_right_data = []
pid_left_data = []
pid_right_data = []
motor_left_data = []
motor_right_data = []

# Initialize plot
fig, ax = plt.subplots(5, 1, figsize=(12, 10))
fig.suptitle("Line Mode Debug Visualization")

# Subplots for distance, speed, error, and PID
distance_ax = ax[0]
speed_ax = ax[1]
error_ax = ax[2]
pid_ax = ax[3]
motor_ax = ax[4]

distance_ax.set_title("Distance Driven")
distance_ax.set_xlabel("Time (s)")
distance_ax.set_ylabel("Distance (m)")

speed_ax.set_title("Wheel Speeds")
speed_ax.set_xlabel("Time (s)")
speed_ax.set_ylabel("Speed (m/s)")

error_ax.set_title("Speed Errors")
error_ax.set_xlabel("Time (s)")
error_ax.set_ylabel("Error (m/s)")

pid_ax.set_title("PID Outputs")
pid_ax.set_xlabel("Time (s)")
pid_ax.set_ylabel("PID Value")

motor_ax.set_title("Motor Speeds")
motor_ax.set_xlabel("Time (s)")
motor_ax.set_ylabel("Motor Speed (PWM)")

# Update function for animation
start_time = time.time()

def update(frame):
    global time_data, distance_data, rps_left_data, rps_right_data, speed_left_data, speed_right_data
    global error_left_data, error_right_data, pid_left_data, pid_right_data, motor_left_data, motor_right_data

    # Read data from serial
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        try:
            # Parse the data
            data = {key: float(value) for key, value in (item.split(":") for item in line.split(","))}
            current_time = time.time() - start_time

            # Append data to lists
            time_data.append(current_time)
            distance_data.append(data["dis"])
            rps_left_data.append(data["rpsL"])
            rps_right_data.append(data["rpsR"])
            speed_left_data.append(data["speedL"])
            speed_right_data.append(data["speedR"])
            error_left_data.append(data["errorL"])
            error_right_data.append(data["errorR"])
            pid_left_data.append(data["pidL"])
            pid_right_data.append(data["pidR"])
            motor_left_data.append(data["motorL"])
            motor_right_data.append(data["motorR"])

            # Update plots
            distance_ax.clear()
            distance_ax.plot(time_data, distance_data, label="Distance")
            distance_ax.legend()

            speed_ax.clear()
            speed_ax.plot(time_data, speed_left_data, label="Left Speed")
            speed_ax.plot(time_data, speed_right_data, label="Right Speed")
            speed_ax.legend()

            error_ax.clear()
            error_ax.plot(time_data, error_left_data, label="Left Error")
            error_ax.plot(time_data, error_right_data, label="Right Error")
            error_ax.legend()

            pid_ax.clear()
            pid_ax.plot(time_data, pid_left_data, label="Left PID")
            pid_ax.plot(time_data, pid_right_data, label="Right PID")
            pid_ax.legend()

            motor_ax.clear()
            motor_ax.plot(time_data, motor_left_data, label="Left Motor Speed")
            motor_ax.plot(time_data, motor_right_data, label="Right Motor Speed")
            motor_ax.legend()

        except ValueError:
            pass

# Animate the plot
ani = FuncAnimation(fig, update, interval=50)

# Show the plot
plt.tight_layout()
plt.show()

# Close the serial connection when done
ser.close()