#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>
#include "map.h"
#include "vehicle.h"
#include "pf.h"

using namespace std;

int main(){
	double reso = 0.5; // resolution for map
	string static_map = "map2.dat";
	string gt_map = "map_vehicle.dat";
	// static map -> doesn't include vechicle
	vector<vector<Node*>> map_s;
	// ground truth map -> include all the vehicle
	vector<vector<Node*>> map_g;
	// dynamic map -> all the measurements are from SLAM
	vector<vector<Node*>> map_d;
	vector<double> limit; // x_min, x_max, y_min, y_max for static map
	vector<double> limit_gt; // x_min, x_max, y_min, y_max for ground truth map
	limit = initMap(map_s, static_map, reso);
	limit_gt = initMap(map_g, gt_map, reso);
	initMap(map_d, limit, reso, 0, 12); // initialize the dynamic map
	//---------- See if the static map and ground truth map have the same size ------------//
	cout << "map_s-> " << "x_min: " << limit[0] << " x_max: " << limit[1] << " y_min: " << limit[2] << " y_max: " << limit[3] << endl;
	cout << "map_gt-> " << "x_min: " << limit_gt[0] << " x_max: " << limit_gt[1] << " y_min: " << limit_gt[2] << " y_max: " << limit_gt[3] << endl; 
	//---------------------------------- Check size end -----------------------------------//

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

	// //-------- Test Particle Filter ---------//
	// // measure 1
	// double test_x = 553.427, test_y = 32.1875;
	// Vehicle test_v(test_x, test_y, 0.0, 60.0);
	// vector<double> origin = {limit[0], limit[2]}; // origin is the min_x and min_y
	// test_v.fixOri(origin);
	// vector<vector<double>> measure;
	// measure = test_v.scanMeasure(map_s, reso);
	// cout << "Total obstacles captured: " << measure.size() << endl;
	// updateOccupancyMap(measure, map_d, test_v, reso);
	// cout << "measure1 finish" << endl; 

	// ofstream lidar_out ("lidar.dat");
	// if(lidar_out.is_open())
	// {
	//     for(auto iter: measure){
	//     	double o_x = test_x + iter[0]*cos(iter[1]);
	//     	double o_y = test_y + iter[0]*sin(iter[1]);
	//     	lidar_out << o_x << " " << o_y << endl;
	//     }
	// }
	// else cout << "Unable to open file";
	
	// // measure 2
	// test_x = 570.427; test_y = 32.1875;
	// Vehicle test_v2(test_x, test_y, 0.0, 60.0);
	// origin = {limit[0], limit[2]};
	// test_v2.fixOri(origin);
	// measure.clear();
	// cout << "has been here" << endl;
	// measure = test_v2.scanMeasure(map_s, reso);
	// cout << "has been here 1" << endl;
	// cout << "Total obstacles captured: " << measure.size() << endl;
	// updateOccupancyMap(measure, map_d, test_v2, reso);
	// cout << "measure2 finish" << endl; 

	// for(auto iter: measure){
 //    	double o_x = test_x + iter[0]*cos(iter[1]);
 //    	double o_y = test_y + iter[0]*sin(iter[1]);
 //    	lidar_out << o_x << " " << o_y << endl;
 //    }

	// // measure 3
	// test_x = 540.427; test_y = 32.1875;
	// Vehicle test_v3(test_x, test_y, 0.0, 60.0);
	// origin = {limit[0], limit[2]};
	// test_v3.fixOri(origin);
	// measure.clear();
	// measure = test_v3.scanMeasure(map_s, reso);
	// cout << "Total obstacles captured: " << measure.size() << endl;
	// updateOccupancyMap(measure, map_d, test_v3, reso);
	// cout << "measure3 finish" << endl;

	// for(auto iter: measure){
 //    	double o_x = test_x + iter[0]*cos(iter[1]);
 //    	double o_y = test_y + iter[0]*sin(iter[1]);
 //    	lidar_out << o_x << " " << o_y << endl;
 //    }

	// // measure 4
	// test_x = 530.427; test_y = 32.1875;
	// Vehicle test_v4(test_x, test_y, 0.0, 60.0);
	// origin = {limit[0], limit[2]};
	// test_v4.fixOri(origin);
	// measure.clear();
	// measure = test_v4.scanMeasure(map_s, reso);
	// cout << "Total obstacles captured: " << measure.size() << endl;
	// updateOccupancyMap(measure, map_d, test_v4, reso);

	// for(auto iter: measure){
 //    	double o_x = test_x + iter[0]*cos(iter[1]);
 //    	double o_y = test_y + iter[0]*sin(iter[1]);
 //    	lidar_out << o_x << " " << o_y << endl;
 //    }

	// cout << "finish measuere" << endl;
	// lidar_out.close();
	// // record the dynamic map based on the data from last several measurements
	// ofstream output ("dynamic_map.dat");
	// if(output.is_open())
	// {
	//     for(auto iter: map_d){
	//     	for(auto node: iter){
	//     		if(node->isOcc()){
	//     			double o_x = node->getPos()[0];
	//     			double o_y = node->getPos()[1];
	// 		    	output << o_x << " " << o_y << endl;
	// 		    }
	//     	}
	//     }
	//     output.close();
	// }
	// else cout << "Unable to open file";
	// //------- Particle Filter Test End ------//

	//-------- Measure during motion + particle filter -----------//
	particleFilter pf(200); // initiliaze the sample number as 50
	double test_x = 530.427, test_y = 32.1875, test_yaw = 0.0; // initialize the test vehicle position
	vector<double> stdv_pf = {2.0, 2.0, 0.1}; // start position's standard deviation [x, y, yaw] for particle filter
	Vehicle test_v(test_x, test_y, test_yaw, 20.0);
	pf.init(test_x, test_y, test_yaw, stdv_pf);
	vector<double> origin = {limit[0], limit[2]};
	test_v.fixOri(origin);
	test_v.addNoiseMotion(0.0, 0.5, 0.0, 0.05); // add noise to motion model's velocity and steering angle. In the order of:
										   		 // mean_v, stdev_v, mean_sa, stdev_sa. This step is necessary or the motion will be ground truth
	test_v.addNoiseMea(0.0, 0.0, 0.0, 0.0); // add noise to measurement model's range and angle. In the order of:
										   	  // mean_r, stdev_r, mean_a, stdev_a. This step is necessary or the measurement will be ground truth

	string v_profile = "motion_command.dat"; // 1->v, 2->sa, 3->dt
	ifstream motion_c;
	motion_c.open(v_profile);
	string line;

	vector<double> noise_motion; // the noise motion command: [v_n, sa_n, dt]
	vector<vector<double>> measure;
	int counter = 0;

	ofstream output_lidar ("lidar.dat");
	ofstream output_pose ("robot_pos.dat");
	ofstream output_motion_mea ("robot_pos_mea.dat");
	ofstream output_est_state("estimated_pos.dat");

	while(getline(motion_c, line)){
		++counter;
		istringstream iss(line);
		double v, sa, dt;
		if(!(iss >> v >> sa >> dt)) break;
		test_v.move(v,sa,dt);
		noise_motion = test_v.move_mea(v,sa,dt);
		measure.clear();
		measure = test_v.scanMeasure(map_g, reso);
		// update all the obstacles
		for(auto iter: measure){
	    	double o_x = test_v.getState()[0] + iter[0]*cos(iter[1]+test_v.getState()[2]);
	    	double o_y = test_v.getState()[1] + iter[0]*sin(iter[1]+test_v.getState()[2]);
	    	output_lidar << o_x << " " << o_y << endl;
	    }
	    output_pose << test_v.getState()[0] << " " << test_v.getState()[1] << endl;
	    output_motion_mea << test_v.getMeaState()[0] << " " << test_v.getMeaState()[1] << endl;
	    // obstacles update ends
		cout << "Total obstacles captured: " << measure.size() << endl;
		// Particle filter starts to take estimation
		pf.updateOccupancyMap(measure, map_d, test_v, reso);
		pf.prediction(noise_motion);
		pf.updateWeights(map_g, measure, test_v, reso);
		pf.resample();
		pf.estState(test_v);
		output_est_state << test_v.x_est << " " << test_v.y_est << " " << test_v.yaw_est << endl;
		// Particle filter estimation ends
		cout << "finish the motion step " << counter << endl;
	}
	output_lidar.close();
	output_pose.close();
	output_motion_mea.close();
	output_est_state.close();
	// record the dynamic map based on the data from last several measurements
	ofstream output ("dynamic_map.dat");
	if(output.is_open())
	{
	    for(auto iter: map_d){
	    	for(auto node: iter){
	    		if(node->isOcc()){
	    			double o_x = node->getPos()[0];
	    			double o_y = node->getPos()[1];
			    	output << o_x << " " << o_y << endl;
			    }
	    	}
	    }
	    output.close();
	}
	else cout << "Unable to open file";
	// record the ground truth map based on the map_vehicle.dat
	ofstream output_gt ("gt_map.dat");
	if(output_gt.is_open())
	{
	    for(auto iter: map_g){
	    	for(auto node: iter){
	    		if(node->isOcc()){
	    			double o_x = node->getPos()[0];
	    			double o_y = node->getPos()[1];
			    	output_gt << o_x << " " << o_y << endl;
			    }
	    	}
	    }
	    output_gt.close();
	}

	//-------- Measure during motion + particle filter finished-----------//


	
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
