import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import atan2, degrees

base_path = "/media/ubuwgb/Samsung_T5_WGB/apollo_rosbag/"
dry = np.loadtxt(base_path + 'dry_ring.txt', delimiter=';')
rain = np.loadtxt(base_path + 'rain_ring.txt', delimiter=';')

# for i in range(dry.shape[0]):
#     dry[i, 5] = degrees(atan2(dry[i, 0], dry[i, 1])) - degrees(atan2(dry[i - 1, 0], dry[i - 1, 1]))
# plt.scatter(np.arange(dry.shape[0] - 1), dry[1:, 5])
# plt.title('dry', fontsize=20)
# plt.show()

# for i in range(rain.shape[0]):
#     rain[i, 5] = degrees(atan2(rain[i, 0], rain[i, 1])) - degrees(atan2(rain[i - 1, 0], rain[i - 1, 1]))
# plt.scatter(np.arange(rain.shape[0] - 1), rain[1:, 5])
# plt.title('rain', fontsize=20)
# plt.show()

fig2 = plt.figure(2)
plt.axis('equal')
plt.scatter(dry[:, 0], dry[:, 2])
# ax2 = Axes3D(fig2)
# ax2.plot3D(dry[:, 0], dry[:, 1], dry[:, 2], 'r', label='first')  #GPS
plt.show()