# -*- coding: utf-8 -*-
"""
Author: Praveen Jawaharlal Ayyanathan
Algorithm details can be found in https://arc.aiaa.org/doi/10.2514/6.2024-4290
"""

import iqmotion as iq
import time
import numpy as np
import matplotlib.pyplot as plt
import math
import keyboard

plt.close('all')

# === Initialization ===
# Serial communication with the Vertiq motor
com = iq.SerialCommunicator("COM6", 921600)
vertiq = iq.Vertiq2306(com, 0)

# PID gains for angular speed controller (manually tuned)
pid_gains = {"velocity_Kp": 0.03, "velocity_Ki": 0.009, "velocity_Kd": 0.009}
for param, value in pid_gains.items():
    vertiq.set("propeller_motor_control", param, value)

# Parameters
sp = 39.873 * math.pi  # Speed setpoint in rad/sec
asp = 5  # Angular setpoint in degrees
ang_time = asp / (360 * 5 / 60)  # Time to reach the setpoint
ramp_time = 5  # Ramp time (seconds)

# Data storage (pre-allocate for efficiency)
num_samples = 5000  # Estimate number of iterations
speed_arr = np.zeros(num_samples)
angle_arr = np.zeros(num_samples)
index = 0

# ===  Data Logging ===
def log_data(speed, angle):
    global index
    if index < num_samples:
        speed_arr[index] = speed
        angle_arr[index] = angle
        index += 1

# Ramping velocity to the setpoint
start_time = time.time()
while (time.time() - start_time) < ramp_time:
    ramp = (time.time() - start_time) / ramp_time
    vertiq.set("propeller_motor_control", "ctrl_velocity", sp * ramp)

    # Retrieve speed and angle data
    angle = vertiq.get("brushless_drive", "obs_angle") * 180 / math.pi + 180
    speed = vertiq.get("brushless_drive", "obs_velocity") * 60 / (2 * math.pi)
    log_data(speed, angle)

# === Phase Change Maneuver ===
while True:
    vertiq.set("propeller_motor_control", "ctrl_velocity", sp)

    angle = vertiq.get("brushless_drive", "obs_angle") * 180 / math.pi + 180
    speed = vertiq.get("brushless_drive", "obs_velocity") * 60 / (2 * math.pi)
    log_data(speed, angle)

    if keyboard.is_pressed('a'):
        # Phase change
        start_time_ang = time.time()
        while (time.time() - start_time_ang) < ang_time:
            vertiq.set("propeller_motor_control", "ctrl_velocity", sp + 5 * 2 * math.pi / 60)

            angle = vertiq.get("brushless_drive", "obs_angle") * 180 / math.pi + 180
            speed = vertiq.get("brushless_drive", "obs_velocity") * 60 / (2 * math.pi)
            log_data(speed, angle)

        #print(f"Angle adjustment duration: {time.time() - start_time_ang:.3f} s")
        #print(f"Final Angle: {angle:.2f} degrees")

    if keyboard.is_pressed('q'):
        break  # Exit program

# === Save Data ===
np.savetxt('speed_arr.csv', speed_arr[:index], delimiter=",")
np.savetxt('angle_arr.csv', angle_arr[:index], delimiter=",")

# === Close Communication ===
vertiq._com._ser_handle.close()

# === Plot Results ===
plt.figure(1)
plt.plot(speed_arr[:index], label='RPM', linewidth=2)
plt.axhline(y=sp * 60 / (2 * math.pi), color='r', linestyle='--', label='Setpoint')
plt.text(0.5, 1.05, f"Kp: {pid_gains['velocity_Kp']}, Ki: {pid_gains['velocity_Ki']}, Kd: {pid_gains['velocity_Kd']}",
         fontsize=12, transform=plt.gca().transAxes, horizontalalignment='center')
plt.xlabel('Iteration')
plt.ylabel('Speed (RPM)')
plt.legend()
plt.grid()
plt.show()

plt.figure(2)
plt.plot(angle_arr[:index], linewidth=2)
plt.xlabel('Iteration')
plt.ylabel('Angle (deg)')
plt.grid()
plt.show()