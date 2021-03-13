#This python plts the function from motor speed to robot speed / angular speed

import bisect
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import misc
from odometry_data import points


#dem=misc.imread('/home/roman/Downloads/dem.tif')
speed = []
delta = []
for t in points:
    speed.append(t[0])
    delta.append(t[1])

#Remove duplicates and sort
speed = sorted(set(speed))
delta = sorted(set(delta))

#Fill the path speed matrix
path_speed = [[np.nan for x in range(len(speed))] for y in range(len(delta))]
ang_speed = [[np.nan  for x in range(len(speed))] for y in range(len(delta))]
for t in points:
    speed_idx=speed.index(t[0])
    delta_idx=delta.index(t[1])
    path_speed[delta_idx][speed_idx]=t[3]
    ang_speed[delta_idx][speed_idx]=t[2]

z_path=np.array(path_speed)
z_ang=np.array(ang_speed)

xv, yv = np.meshgrid(speed, delta)
fig1 = plt.figure()
ax = fig1.add_subplot(111, projection='3d')
path_speed3d=ax.plot_wireframe(xv,yv,z_path,cmap='afmhot', linewidth=1)
ax.scatter(xv,yv,z_path, cmap='viridis', linewidth=0.5);
ax.set_title('Path speed')
ax.set_zlabel('Path speed (m/s)')
ax.set_xlabel('Speed (m/s)')
ax.set_ylabel('Delta (m/s)')
fig2 = plt.figure()
bx = fig2.add_subplot(111, projection='3d')
path_speed3d=bx.plot_wireframe(xv,yv,z_ang,cmap='afmhot', linewidth=1)
bx.scatter(xv,yv,z_ang, cmap='viridis', linewidth=0.5);
bx.set_title('Angular speed')
bx.set_zlabel('Angular speed (rad/s)')
bx.set_xlabel('Speed (m/s)')
bx.set_ylabel('Delta (m/s)')


plt.show()


