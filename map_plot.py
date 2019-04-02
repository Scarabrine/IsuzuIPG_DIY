import matplotlib.pyplot as plt 
import argparse

parser = argparse.ArgumentParser(description='arguments')
parser.add_argument('--path', action='store_true', help='plot paths')
args = parser.parse_args()

map_vehicle_file = "map_vehicle.dat"
map_gt_file = "gt_map.dat"
map_file = "map.dat"
path_file = "path.dat"
pos_file = "pos.dat"
lidar_file = "lidar.dat"
robot_file = "robot_pos.dat"
dynamic_file = "dynamic_map.dat"
motion_mea_file = "robot_pos_mea.dat"
pos_est_file = "estimated_pos.dat"
sample_file = "pf_sample.dat"

# map
plt.plotfile(map_file, delimiter=' ', cols=(0,1), 
	names=('x','y'), checkrows = 0, marker='o', 
	linestyle=' ')

plt.plotfile(lidar_file, delimiter=' ', cols=(0,1), 
	names=('x','y'), checkrows = 0, marker='o', 
	linestyle=' ', color='r', newfig=False)

plt.plotfile(robot_file, delimiter=' ', cols=(0,1), 
	names=('x','y'), checkrows = 0, marker='*', 
	linestyle=' ', color='g', newfig=True)

plt.plotfile(pos_est_file, delimiter=' ', cols=(0,1), 
	names=('x','y'), checkrows = 0, marker='*', 
	linestyle=' ', color='m', newfig=False)

plt.plotfile(motion_mea_file, delimiter=' ', cols=(0,1), 
	names=('x','y'), checkrows = 0, marker='*', 
	linestyle=' ', color='y', newfig=False)

# plt.plotfile(sample_file, delimiter=' ', cols=(0,1), 
# 	names=('x','y'), checkrows = 0, marker='*', 
# 	linestyle=' ', color='c', newfig=False)

plt.plotfile(map_gt_file, delimiter=' ', cols=(0,1), 
	names=('x','y'), checkrows = 0, marker='o', 
	linestyle=' ', color='b', newfig=True)

plt.plotfile(dynamic_file, delimiter=' ', cols=(0,1), 
	names=('x','y'), checkrows = 0, marker='o', 
	linestyle=' ', color='r', newfig=False)



# if args.path:
#     # path
#     plt.plotfile(path_file, delimiter=' ', cols=(0,1), 
# 	checkrows = 0, linestyle='-', color='r', 
# 	newfig=False, label='reference path')
#     # pos
#     plt.plotfile(pos_file, delimiter=' ', cols=(0,1),
# 	checkrows = 0, linestyle=':', color='b',
# 	newfig=False, label='actual path')
#     plt.legend()

plt.xlabel('x')
plt.ylabel('y')

plt.show()
