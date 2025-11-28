#%% ===== Autonomous Disc Tracking for Pioneer P3DX in CoppeliaSim =====
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

print("System Booting... Stand by.")

# Connect to simulation
client = RemoteAPIClient()
sim = client.require('sim')

sim.setStepping(False)
sim.startSimulation()

# === GET HANDLES ===
wR_Handle = sim.getObject("/rightMotor")
wL_Handle = sim.getObject("/leftMotor")
p3dx_Handle = sim.getObject("/PioneerP3DX")
disc_Handle = sim.getObject("/Disc")

print("Assets Loaded Successfully")
time.sleep(0.5)

# Control Gains
linear_gain = 1.2
angular_gain = 2.0
stop_threshold = 0.12
max_speed = 3.5

# Tracking Logs
trajectory_robot = []
trajectory_disc = []
heading_errors = []
distance_errors = []
timestamps = []

print("Tracking Online: Robot now follows disc dynamically.")

while True:

    # ========== POSITION & ORIENTATION STATE ==========
    robot_pos = sim.getObjectPosition(p3dx_Handle, -1)
    robot_orient = sim.getObjectOrientation(p3dx_Handle, -1)
    disc_pos = sim.getObjectPosition(disc_Handle, -1)

    dx = disc_pos[0] - robot_pos[0]
    dy = disc_pos[1] - robot_pos[1]

    distance_error = math.sqrt(dx**2 + dy**2)
    target_angle = math.atan2(dy, dx)
    robot_yaw = robot_orient[2]

    heading_error = target_angle - robot_yaw
    heading_error = (heading_error + np.pi) % (2*np.pi) - np.pi   # wrap angle

    # ========== CONTROLLER LOGIC ==========
    if distance_error > stop_threshold:
        v = linear_gain * distance_error
        w = angular_gain * heading_error
    else:
        v = 0
        w = 0

    # velocity limiting
    v = np.clip(v, -max_speed, max_speed)
    w = np.clip(w, -max_speed, max_speed)

    # Compute wheel speeds
    wheel_L = v - w
    wheel_R = v + w

    # Push to motors
    sim.setJointTargetVelocity(wL_Handle, wheel_L)
    sim.setJointTargetVelocity(wR_Handle, wheel_R)

    # Logging for plots
    timestamps.append(datetime.now().timestamp())
    trajectory_robot.append(robot_pos[:2])
    trajectory_disc.append(disc_pos[:2])
    heading_errors.append(heading_error)
    distance_errors.append(distance_error)

    time.sleep(0.05)

    # Exit logic (press STOP button inside Coppelia)
    if sim.getSimulationState() == 0:
        break


# ====== VISUAL OUTPUT AFTER SIMULATION STOP ======

trajectory_robot = np.array(trajectory_robot)
trajectory_disc = np.array(trajectory_disc)

plt.figure(figsize=(7, 7))
plt.plot(trajectory_robot[:,0], trajectory_robot[:,1], label="Robot Trajectory")
plt.plot(trajectory_disc[:,0], trajectory_disc[:,1], label="Disc Path", linestyle='--')
plt.title("XY Tracking Trajectory")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.legend()
plt.grid()
plt.show()

plt.figure()
plt.plot(distance_errors)
plt.title("Distance Error Over Time")
plt.ylabel("Distance (m)")
plt.xlabel("Samples")
plt.grid()
plt.show()

plt.figure()
plt.plot(heading_errors)
plt.title("Heading Error Over Time")
plt.ylabel("Radians")
plt.xlabel("Samples")
plt.grid()
plt.show()

print("Simulation Ended. Tracking report generated.")
