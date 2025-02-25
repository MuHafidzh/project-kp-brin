import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

# Membaca data dari file
times = []
positions_8 = []
positions_9 = []

with open('tpid20.txt', 'r') as file:
    for line in file:
        parts = line.strip().split(' || ')
        pos_part = parts[0].split(', ')
        time_part = parts[1].split(': ')[1]

        pos_8 = int(pos_part[0].split(': ')[1])
        pos_9 = int(pos_part[1].split(': ')[1])
        time = int(time_part)

        positions_8.append(pos_8)
        positions_9.append(pos_9)
        times.append(time)

# Mengubah data menjadi numpy array
times = np.array(times)
positions_8 = np.array(positions_8)
positions_9 = np.array(positions_9)

# Menghapus duplikat
unique_times, unique_indices = np.unique(times, return_index=True)
positions_8 = positions_8[unique_indices]
positions_9 = positions_9[unique_indices]

# Membuat interpolasi spline
times_smooth = np.linspace(unique_times.min(), unique_times.max(), 300)
positions_8_smooth = make_interp_spline(unique_times, positions_8)(times_smooth)
positions_9_smooth = make_interp_spline(unique_times, positions_9)(times_smooth)

# Membuat plot
plt.figure(figsize=(10, 5))
plt.plot(times_smooth, positions_8_smooth, label='Position 8')
plt.plot(times_smooth, positions_9_smooth, label='Position 9')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.title('Position vs Time')
plt.legend()
plt.grid(True)
plt.show()