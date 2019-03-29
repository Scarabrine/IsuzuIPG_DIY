#include "pf.h"
#include "vehicle.h"
#include "map.h"
#include <vector>
#include <cmath>

using namespace std;

void updateOccupancyMap(vector<double>& mea, vector<vector<Node*>>& map_in, Vehicle& car, double res){
	vector<vector<int>> m_obstacle(mea.size(), vector<int>(2,0));
	vector<double> car_state = car.getState();
	double cur_x = car_state[0], cur_y = car_state[1], cur_yaw = car_state[2]; // current car's state
	double ori_x = car.getOri()[0], ori_y = car.getOri()[1]; // the origin of the world map
	for(auto ob: mea){
		// calculate obstacle's position in world frame
		double o_x = cur_x + mea[0]*cos(mea[1]+cur_yaw), o_y = cur_y + mea[0]*sin(mea[1]+cur_yaw);
		// calcualte obstacle's index infomation in map
		double o_x_i = (o_x-ori_x)/res+1;
		double o_y_i = (o_y-ori_y)/res+1;
		// --> one thing can be improved: use Bresenham’s algorithm to find grid along the laser beam
		// in this case, we can check all the cell on the laser beam. 
		// we can add 3 to the hit and minus 1 to the free space. In this case, we can reduce the random
		// error's harm to the occupancy grid map
		// !!But now, I only care the hit part not the free space along the laser beam.
		map_in[o_y_i][o_x_i]->updateNode(true); // true == hit
	}
	return;
}

