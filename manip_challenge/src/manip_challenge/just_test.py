from operator import add
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import numpy as np

extend_size = 100
p1 = np.array([ 0.55000025, -0.26749989,  0.57500018])*extend_size
p2 = np.array([ 0.49000025, -0.26750011,  0.575     ])*extend_size
p3 = np.array([ 0.48999975, -0.13250011,  0.575     ])*extend_size
p4 = np.array([ 0.54999975, -0.13249989,  0.57500018])*extend_size
p5 = np.array([ 0.55000011, -0.26749989,  0.62500018])*extend_size
p6 = np.array([ 0.49000011, -0.26750011,  0.625     ])*extend_size
p7 = np.array([ 0.4899996 , -0.13250011,  0.625     ])*extend_size
p8 = np.array([ 0.5499996 , -0.13249989,  0.62500018])*extend_size

observation_space_low = [-0.1, -0.7, -0.1]
observation_space_high = [0.8, 0.7, 1.0]

grid_limits = [observation_space_low, observation_space_high]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim3d(grid_limits[0][0]*extend_size, grid_limits[1][0]*extend_size)
ax.set_ylim3d(grid_limits[0][1]*extend_size, grid_limits[1][1]*extend_size)
ax.set_zlim3d(grid_limits[0][2]*extend_size, grid_limits[1][2]*extend_size)

vertices = np.array([p1, p2, p3, p4, p5, p6, p7, p8])
ax.scatter3D(vertices[:, 0], vertices[:, 1], vertices[:, 2])
faces = [[p1, p2, p3, p4], [p1, p5, p8, p4], [p5, p6, p7, p8],
[p8, p7, p3, p4], [p7, p6, p2, p3], [p5, p6, p2, p1]]

ax.add_collection3d(Poly3DCollection(faces, 
    facecolors='cyan', linewidths=1, edgecolors='cyan', alpha=.25))

plt.show()