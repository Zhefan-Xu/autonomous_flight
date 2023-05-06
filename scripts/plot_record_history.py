#!/usr/bin/env python

# FILE: plot_record_history.py
# ------------------------
# Plot historical velocities and acceleration profiles

import numpy as np
import os
import matplotlib.pyplot as plt
from pathlib import Path


file_dir = Path(__file__).parent.parent.absolute()
file_dir = os.path.join(file_dir, "log")
vel_filename = "vel_hist.txt"
acc_filename = "acc_hist.txt"
vel_file_path = os.path.join(file_dir, vel_filename)
acc_file_path = os.path.join(file_dir, acc_filename)

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

fig, ax = plt.subplots(2, sharex='col', sharey='row')

ax[0].plot(vel_time, vel_norm, label="|V|")
ax[0].plot(vel_time, vel_x, label="Vx")
ax[0].plot(vel_time, vel_y, label="Vy")
ax[0].plot(vel_time, vel_z, label="Vz")
ax[0].set(ylabel='Velocity [m/s]', title='Velocity Profile')
ax[0].set_xlim([0, vel_time[-1]])
ax[0].set_xticks(np.arange(0, vel_time[-1], int(vel_time[-1]/20)))
ax[0].legend()
ax[0].grid()


ax[1].plot(acc_time, acc_norm, label="|A|")
ax[1].plot(acc_time, acc_x, label="Ax")
ax[1].plot(acc_time, acc_y, label="Ay")
ax[1].plot(acc_time, acc_z, label="Az")
ax[1].set(xlabel='Time [s]', ylabel='Acceleration [m/s/s]', title='Acceleration Profile')
ax[1].set_xlim([0, acc_time[-1]])
ax[1].set_xticks(np.arange(0, acc_time[-1], int(acc_time[-1]/20)))
ax[1].legend()
ax[1].grid()


# ax[1].legend(loc='upper left')

plt.show()
