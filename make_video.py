import matplotlib.pyplot as plt
import argparse
import csv

# parser = argparse.ArgumentParser(description='arguments')
# parser.add_argument('--path', action='store_true', help='plot paths')
# args = parser.parse_args()

# Here is all the parameters you need to change
fps = 5.0 # shuould be 1/dt where dt is your sampling rate
target = 'radar' # should be the type of figure you want to plot
                 # either: lidar, radar, or dynamic
color = 'y' # lidar: r, radar: y, dynamic: b
file_num = 87 # the total number of samples you have.
              # should be the largest number in your lidar_step#.dat
# End of all the parameters definition

map_gt_file = 'map_vehicle.dat'
lidar_file = "data/lidar.dat"
robot_file = "data/robot_pos.dat"
dynamic_file = "data/dynamic_map.dat"
motion_mea_file = "data/robot_pos_mea.dat"
pos_est_file = "data/estimated_pos.dat"
radar_file = "data/radar_dynamic_map.dat"
path_plan_file = "data/Astart_Path.dat"

## read global file start
with open(map_gt_file, 'r') as data:
    gt_x = []
    gt_y = []
    for line in data:
        p = line.split()
        gt_x.append(float(p[0]))
        gt_y.append(float(p[1]))

with open(robot_file, 'r') as data:
    pos_x = []
    pos_y = []
    for line in data:
        p = line.split()
        pos_x.append(float(p[0]))
        pos_y.append(float(p[1]))

with open(pos_est_file, 'r') as data:
    est_x = []
    est_y = []
    for line in data:
        p = line.split()
        est_x.append(float(p[0]))
        est_y.append(float(p[1]))

with open(motion_mea_file, 'r') as data:
    mea_x = []
    mea_y = []
    for line in data:
        p = line.split()
        mea_x.append(float(p[0]))
        mea_y.append(float(p[1]))

# map the dynamic map for each step

for k in range(file_num):
    # add the current step's dynamic map
    plt.scatter(gt_x, gt_y, c='k')
    dynamic_step_file = 'data/' + target + '_step' + str(k+1) + '.dat'
    with open(dynamic_step_file, 'r') as data:
        dy_x = []
        dy_y = []
        for line in data:
            p = line.split()
            dy_x.append(float(p[0]))
            dy_y.append(float(p[1]))
    if len(dy_x) > 0:
        plt.scatter(dy_x, dy_y, color=color)
    plt.plot(pos_x[:(k+1)],pos_y[:(k+1)],'b')
    plt.plot(est_x[:(k+1)],est_y[:(k+1)],'c', linestyle=':')
    plt.plot(mea_x[:(k+1)],mea_y[:(k+1)],'m', linestyle='--')
    plt.xlabel('x')
    plt.ylabel('y')
    output_name = 'output_img/' + target + str(k+1) +'.png'
    plt.savefig(output_name)
    plt.clf()
    print(k)

# plt.scatter(gt_x,gt_y)
# plt.plot(pos_x, pos_y)
# plt.show()
