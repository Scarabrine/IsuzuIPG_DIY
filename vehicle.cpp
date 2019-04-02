#include "vehicle.h"

#include <cmath>
#include <unordered_set>
#include <vector>
#include <queue>
#include <string>
#include <iostream>
#include <random>

using namespace std;

default_random_engine gene;

string coorToStr(int x, int y){
	return (to_string(x)+","+to_string(y));
}

void Vehicle::move(double v, double sa, double dt){
	x += v*cos(yaw)*dt;
	y += v*sin(yaw)*dt;
	yaw += sa;
}

vector<double> Vehicle::move_mea(double v, double sa, double dt){
	normal_distribution<double> dist_v(mean_v, stdev_v);
	normal_distribution<double> dist_sa(mean_sa, stdev_sa);
	double v_n = (v + dist_v(gene));
	double sa_n = (sa + dist_sa(gene));
	cout << "v: " << v << " sa: " << sa << " v_n: " << v_n << " sa_n: " << sa_n << endl; 
	x_mea += v_n*cos(yaw_mea)*dt;
	y_mea += v_n*sin(yaw_mea)*dt;
	yaw_mea += sa_n;

	vector<double> noise_motion = {v_n, sa_n, dt};
	return noise_motion;
}

void Vehicle::fixOri(vector<double>& origin){
	ori_x = origin[0];
	ori_y = origin[1];
}

vector<vector<double>> Vehicle::scanMeasure(vector<vector<Node*>>& map_in, double res){
	normal_distribution<double> dist_r(mean_r, stdev_r);
	normal_distribution<double> dist_a(mean_a, stdev_a);

	unordered_set<string> visit; // closed set
	vector<vector<int>> dir = {{-1,0},{1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,-1},{-1,1}};
	vector<vector<double>> block; // {{a,b},......} which means the angle between a and b have been blocked
	vector<vector<double>> result;
	double angle_g; // global angle for laser beam, NOT in vehicle coordinate

	queue<vector<int>> q; // open set
	// cout << "vehicle current position: x->" << x << " y->" << y << endl;
	int idx_cur_x = (x-ori_x)/res;
	int idx_cur_y = (y-ori_y)/res;
	// cout << "idx_cur_y: " << idx_cur_y << endl;
	vector<int> temp = {idx_cur_x, idx_cur_y};
	q.push(temp);

	while(!q.empty()){
		// cout << "search here" << endl;
		// cout << "q size: " << q.size() << endl;
		for(int i = q.size(); i > 0; --i){
			// cout << "check adjecent i: " << i << endl;
			vector<int> cur = q.front();
			q.pop();
			int cur_x = cur[0], cur_y = cur[1];
			// cout << "origin_x: " << ori_x << " origin_y: " << ori_y << endl;
			// cout << cur_x << "/" << map_in[0].size() << " " << cur_y << "/" << map_in.size() << endl;
			for(auto iter: dir){
				// cout << "check 8 diection..." << endl;
				int flag = 0;
				int next_x = cur_x+iter[0], next_y = cur_y+iter[1];
				if(next_x < map_in[0].size() && next_x >= 0 && next_y < map_in.size() && next_y >= 0){
					Node* next_node = map_in[next_y][next_x];
					double x_nd = next_node->getPos()[0], y_nd = next_node->getPos()[1]; // nd means next pos's double coordinate
					angle_g = atan2(y_nd-y, x_nd-x); // this is golbal coordinate
					// cout << "x_nd " << x_nd << " y_nd " << y_nd << endl;
					double r = sqrt((x_nd-x)*(x_nd-x)+(y_nd-y)*(y_nd-y));
					// cout << "I'm fine till now" << endl;
					// whether next position is in closed set
					if(!visit.count(coorToStr(next_x, next_y))){
						// add the next point in the check list
						vector<int> next_pos = {next_x, next_y};
						q.push(next_pos);
						visit.insert(coorToStr(next_x, next_y));
					}
					else
						continue;
					// whether this point is out of range
					if(r > range)
						continue;
					// whether this pos is blocked by the previous obstacle
					for(auto a: block){
						if(angle_g > a[0] && angle_g < a[1]){
							// cout << "check angle_g " << angle_g << " is blocked" << endl;
							flag = 1;
							break;
						}
					}
				
					// whether this is a block, if is a block, add the range it blocks and put it in lidar measurement
					if(flag == 0 && next_node->isOcc()){
						double relative_angle = angle_g-yaw; // turn global angle into robot coordinate
						// add gaussian noise to measurement
						r += dist_r(gene);
						relative_angle += dist_a(gene);
						// end gaussian noise to measurement
						vector<double> measure = {r,relative_angle};
						result.push_back(measure);
						// calculate block angle, only use a approximation
						// cout << "angle world frame: " << angle_g << endl;
						vector<double> block_range = {angle_g-1.5*res/r/2.0, angle_g+1.5*res/r/2.0};
						// cout << "block range: from " << block_range[0] << " to " << block_range[1] << endl;
						block.push_back(block_range);
					}
				}
			}
		}
	}

	return result;
}

