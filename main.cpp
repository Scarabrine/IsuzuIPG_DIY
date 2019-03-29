#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include "map.h"
#include "vehicle.h"

using namespace std;

int main(){
	double reso = 0.5; // resolution for map
	string static_map = "map.dat";
	string gt_map = "map.dat";
	// static map -> doesn't include vechicle
	vector<vector<Node*>> map_s;
	// ground truth map -> include all the vehicle
	vector<vector<Node*>> map_g;
	// dynamic map -> all the measurements are from SLAM
	vector<vector<Node*>> map_d;
	vector<double> limit;
	limit = initMap(map_s, static_map, reso);
	// initMap(map_s, gt_map, reso);
	initMap(map_d, limit, reso, 0, 9);

	// //--- lidar test at from a fixed vehicle position ---//
	// double test_x = 553.427, test_y = 32.1875;
	// Vehicle test_v(test_x, test_y, 0.0, 60.0);
	// vector<double> origin = {limit[0], limit[2]};
	// test_v.fixOri(origin);
	// vector<vector<double>> measure;
	// measure = test_v.scanMeasure(map_s, reso);
	// cout << "x size: " << measure[0].size() << ", y size: " << measure.size() << endl;
	// for(auto mea: measure){
	// 	cout << "range: " << mea[0] << " angle: " << mea[1] << endl; 
	// }
	// ofstream output ("lidar.dat");
	// if(output.is_open())
	// {
	//     for(auto iter: measure){
	//     	double o_x = test_x + iter[0]*cos(iter[1]);
	//     	double o_y = test_y + iter[0]*sin(iter[1]);
	//     	output << o_x << " " << o_y << endl;
	//     }
	//     output.close();
	// }
	// else cout << "Unable to open file";
	// //-------- Lidar Test End --------//

	
	// cout << map_s.size() << " " << map_s[0].size();
	// cout << endl;
	// for(auto i: map_s){
	// 	for(auto j: i){
	// 		cout << "[" << j[0].getPos()[0] << "," << j[0].getPos()[1] << "," << j[0].getOcc() << "] ";
	// 	}
	// 	cout << endl;
	// }

	return 0;
}
