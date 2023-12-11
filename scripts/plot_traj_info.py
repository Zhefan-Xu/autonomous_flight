#!/usr/bin/env python

# FILE: plot_traj_info.py
# ------------------------
# Plot traj velocities and acceleration profiles

import numpy as np
import os
import matplotlib.pyplot as plt
from pathlib import Path


file_dir = Path(__file__).parent.parent.absolute()
file_dir = os.path.join(file_dir, "log")
vel_filename = "vel_info.txt"
acc_filename = "acc_info.txt"
vel_adjusted_filename = "vel_adjusted_info.txt"
acc_adjusted_filename = "acc_adjusted_info.txt"
save_path = os.path.join(file_dir, "traj_info.png")


vel_file_path = os.path.join(file_dir, vel_filename)
acc_file_path = os.path.join(file_dir, acc_filename)
vel_adjusted_file_path = os.path.join(file_dir, vel_adjusted_filename)
acc_adjusted_file_path = os.path.join(file_dir, acc_adjusted_filename)

vel_data = np.loadtxt(vel_file_path)
vel_time = vel_data[:, 0]
vel_x = vel_data[:, 1]
vel_y = vel_data[:, 2]
vel_z = vel_data[:, 3]
vel_norm = np.sqrt(vel_x**2 + vel_y**2 + vel_z**2)

acc_data = np.loadtxt(acc_file_path)
acc_time = acc_data[:, 0]
acc_x = acc_data[:, 1]
acc_y = acc_data[:, 2]
acc_z = acc_data[:, 3]
acc_norm = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)

vel_adjusted_data = np.loadtxt(vel_adjusted_file_path)
vel_adjusted_time = vel_adjusted_data[:, 0]
vel_adjusted_x = vel_adjusted_data[:, 1]
vel_adjusted_y = vel_adjusted_data[:, 2]
vel_adjusted_z = vel_adjusted_data[:, 3]
vel_adjusted_norm = np.sqrt(vel_adjusted_x**2 + vel_adjusted_y**2 + vel_adjusted_z**2)

acc_adjusted_data = np.loadtxt(acc_adjusted_file_path)
acc_adjusted_time = acc_adjusted_data[:, 0]
acc_adjusted_x = acc_adjusted_data[:, 1]
acc_adjusted_y = acc_adjusted_data[:, 2]
acc_adjusted_z = acc_adjusted_data[:, 3]
acc_adjusted_norm = np.sqrt(acc_adjusted_x**2 + acc_adjusted_y**2 + acc_adjusted_z**2)


fig, ax = plt.subplots(2, 2, sharex='col', sharey='row')
fig.set_size_inches(18.5, 10.5)
ax[0, 0].plot(vel_time, vel_norm, label="|V|")
ax[0, 0].plot(vel_time, vel_x, label="Vx")
ax[0, 0].plot(vel_time, vel_y, label="Vy")
ax[0, 0].plot(vel_time, vel_z, label="Vz")
ax[0, 0].set(ylabel='Velocity [m/s]', title='Velocity Profile')
ax[0, 0].set_xlim([0, vel_time[-1]])
# ax[0, 0].set_xticks(np.arange(0, vel_time[-1], int(vel_time[-1]/10)))
ax[0, 0].legend()
ax[0, 0].grid()

ax[0, 1].plot(vel_adjusted_time, vel_adjusted_norm, label="|V|")
ax[0, 1].plot(vel_adjusted_time, vel_adjusted_x, label="Vx")
ax[0, 1].plot(vel_adjusted_time, vel_adjusted_y, label="Vy")
ax[0, 1].plot(vel_adjusted_time, vel_adjusted_z, label="Vz")
ax[0, 1].set(ylabel='Velocity [m/s]', title='Adjusted Velocity Profile')
ax[0, 1].set_xlim([0, vel_adjusted_time[-1]])
# ax[0, 0].set_xticks(np.arange(0, vel_time[-1], int(vel_time[-1]/10)))
ax[0, 1].legend()
ax[0, 1].grid()


ax[1, 0].plot(acc_time, acc_norm, label="|A|")
ax[1, 0].plot(acc_time, acc_x, label="Ax")
ax[1, 0].plot(acc_time, acc_y, label="Ay")
ax[1, 0].plot(acc_time, acc_z, label="Az")
ax[1, 0].set(xlabel='Time [s]', ylabel='Acceleration [m/s/s]', title='Acceleration Profile')
ax[1, 0].set_xlim([0, acc_time[-1]])
# ax[1, 0].set_xticks(np.arange(0, acc_time[-1], int(acc_time[-1]/10)))
ax[1, 0].legend()
ax[1, 0].grid()

ax[1, 1].plot(acc_adjusted_time, acc_adjusted_norm, label="|A|")
ax[1, 1].plot(acc_adjusted_time, acc_adjusted_x, label="Ax")
ax[1, 1].plot(acc_adjusted_time, acc_adjusted_y, label="Ay")
ax[1, 1].plot(acc_adjusted_time, acc_adjusted_z, label="Az")
ax[1, 1].set(xlabel='Time [s]', ylabel='Acceleration [m/s/s]', title='Adjusted Acceleration Profile')
ax[1, 1].set_xlim([0, acc_adjusted_time[-1]])
# ax[1, 0].set_xticks(np.arange(0, acc_time[-1], int(acc_time[-1]/10)))
ax[1, 1].legend()
ax[1, 1].grid()


plt.show()
fig.savefig(save_path)
