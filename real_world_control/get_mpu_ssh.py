import subprocess

# Run SSH command and process each line
proc = subprocess.Popen(
    ["ssh", "arcs@10.13.65.136", "python3 Desktop/Hexapod/mpu6050.py"],
    stdout=subprocess.PIPE,
    universal_newlines=True
)

try:
    for line in proc.stdout:
        line = line.strip()
        if line: 
            print("RECEIVED:", line)
            # Add your processing here
except KeyboardInterrupt:
    proc.terminate()