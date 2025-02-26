import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

filenames = ['tpid20.txt', 'tpid10.txt', 'tpid5.txt', 'pid20.txt', 'pid10.txt', 'pid5.txt', 'npid20.txt', 'npid10.txt', 'npid5.txt']
#treshold
# 20*360/4096 = 1.7578125
# 10*360/4096 = 0.87890625
# 5*360/4096 = 0.439453125
treshold_deg = [1.7578125, 0.87890625, 0.439453125]
#pembulatan ke 2 angka dibelakang koma
treshold = [round(i, 2) for i in treshold_deg]

fig, axs = plt.subplots(3, 3, figsize=(15, 15))
axs = axs.T.flatten() # Transpose

for i, filename in enumerate(filenames):
    times = []
    positions_8 = []
    positions_9 = []

    with open(filename, 'r') as file:
        for line in file:
            # parsing format: pos: 8: 2048, 9: 2048 || time: 0
            parts = line.strip().split(' || ')
            pos_part = parts[0].split(', ')
            time_part = parts[1].split(': ')[1] 

            pos_8 = int(pos_part[0].split(': ')[1])
            pos_9 = int(pos_part[1].split(': ')[1])
            time = int(time_part)

            # Convert positions to degrees
            pos_8 = (pos_8 - 2048) * 360 / 4096
            pos_9 = (pos_9 - 2048) * 360 / 4096

            positions_8.append(pos_8)
            positions_9.append(pos_9)
            times.append(time)

    # Convert to arrays
    times = np.array(times)
    positions_8 = np.array(positions_8)
    positions_9 = np.array(positions_9)

    # filter duplicate data
    unique_times, unique_indices = np.unique(times, return_index=True)
    positions_8 = positions_8[unique_indices]
    positions_9 = positions_9[unique_indices]

    # interpolation
    times_smooth = np.linspace(unique_times.min(), unique_times.max(), 300)
    positions_8_smooth = make_interp_spline(unique_times, positions_8)(times_smooth)
    positions_9_smooth = make_interp_spline(unique_times, positions_9)(times_smooth)

    axs[i].plot(times_smooth, positions_8_smooth, label='Pan (deg)')
    axs[i].plot(times_smooth, positions_9_smooth, label='Tilt (deg)')
    axs[i].set_xlabel('Time (s)', fontsize=10)
    axs[i].set_ylabel('Position (deg)', fontsize=10)
    axs[i].set_title(f'treshold: ±{treshold[i%3]}°', fontsize=10)
    axs[i].grid(True)

# Add a single legend for the entire figure
handles, labels = axs[0].get_legend_handles_labels()
fig.legend(handles, labels, loc='upper left')

fig.suptitle('Position Responses', y=0.99, fontsize=14)
fig.text(0.17, 0.94, 'Without PID', ha='center', fontsize=12)
fig.text(0.5, 0.94, 'Proportional (P)', ha='center', fontsize=12)
fig.text(0.83, 0.94, 'Proportional Integral (PI)', ha='center', fontsize=12)

# Adjust layout
plt.subplots_adjust(top = 0.90, bottom = 0.073, left = 0.045, right = 0.989, hspace = 0.471, wspace = 0.18)
plt.show()