import matplotlib.pyplot as plt 
import argparse

parser = argparse.ArgumentParser(description='arguments')
parser.add_argument('--path', action='store_true', help='plot paths')
args = parser.parse_args()

map_file = "map.dat"
path_file = "path.dat"
pos_file = "pos.dat"
lidar_file = "lidar.dat"

# map
plt.plotfile(map_file, delimiter=' ', cols=(0,1), 
	names=('x','y'), checkrows = 0, marker='o', 
	linestyle=' ')

plt.plotfile(lidar_file, delimiter=' ', cols=(0,1), 
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
