import subprocess
import json
from helpers.kalman_filter import KalmanFilter
import time
import math


def read_mpu_data(line):
    """
    Process the line received from the remote mpu6050.py script.
    The input line is a json string containing MPU data.
    Input:
    line: str, a line of data received from the remote script
    Output:
    Ax, Ay, Az, Gx, Gy, Gz: float, accelerometer and gyroscope data
    """
    data = json.loads(line)
    Ax = data.get("Ax", 0.0)
    Ay = data.get("Ay", 0.0)
    Az = data.get("Az", 0.0)
    Gx = data.get("Gx", 0.0)
    Gy = data.get("Gy", 0.0)
    Gz = data.get("Gz", 0.0)
    return Ax, Ay, Az, Gx, Gy, Gz


def main():
    print("Starting SSH command to run mpu6050.py on remote machine...")
    # Run SSH command and process each line
    proc = subprocess.Popen(
        ["ssh", "arcs@10.13.65.136", "python3 Desktop/Hexapod/mpu6050.py"],
        stdout=subprocess.PIPE,
        universal_newlines=True
    )

    try:
        
        # Initialize Kalman filters for X and Y axes
        kalmanX = KalmanFilter()
        kalmanY = KalmanFilter()
        timer = time.time()

        # Read lines from the subprocess output
        for line in proc.stdout:
            line = line.strip()
            
            if line:
                # Time delta
                dt = time.time() - timer
                timer = time.time()
                print("RECEIVED:", line)
            
                Ax, Ay, Az, Gx, Gy, Gz = read_mpu_data(line)

                # Calculate angle from accelerometer
                acc_pitch = math.degrees(math.atan2(-Ax, math.sqrt(Ay*Ay + Az*Az)))
                acc_roll  = math.degrees(math.atan2(Ay, Az))

                # Use Kalman filter to get best angle estimate
                pitch = kalmanX.get_angle(acc_pitch, Gx, dt)
                roll  = kalmanY.get_angle(acc_roll, Gy, dt)

                print(f"Pitch: {pitch:.2f}, Roll: {roll:.2f}")

                time.sleep(0.02)

    except KeyboardInterrupt:
        proc.terminate()