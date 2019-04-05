#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>

#include "Radar.h"
#include "map.h"
#include "vehicle.h"
#include "pf.h"
#include "Parameters.hpp"

// planning lib
#include "HybridAstar_truck_cplusplus/src/pathfinder_hybrid_astar.hpp"
#include "HybridAstar_truck_cplusplus/src/map.hpp"
#include "HybridAstar_truck_cplusplus/src/def_all.hpp"

using namespace std;

static vector<double> path_x;
static vector<double> path_y;
static vector<double> path_yaw;
static vector<double> path_yaw1;
static vector<bool> path_direction;

int main(){
	double reso = 0.5; // resolution for map
	string static_map = "map2.dat";
	string gt_map = "map_vehicle.dat";
	// static map -> doesn't include vechicle
	vector<vector<Cell*>> map_s;
	// ground truth map -> include all the vehicle
	vector<vector<Cell*>> map_g;
	vector<vector<Cell*>> map_g_radar;
	// dynamic map -> all the measurements are from SLAM
	vector<vector<Cell*>> map_d;
	vector<vector<Cell*>> map_d_radar;

	vector<double> limit; // x_min, x_max, y_min, y_max for static map
	vector<double> limit_gt; // x_min, x_max, y_min, y_max for ground truth map
	vector<double> limit_gt_radar; // x_min, x_max, y_min, y_max for ground truth radar map

	limit = initMap(map_s, static_map, reso);
	limit_gt = initMap(map_g, gt_map, reso);
	limit_gt_radar = initMap(map_g_radar, gt_map, RES);
	initMap(map_d_radar, limit_gt_radar, RES, 0, 2);
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
	//     			double o_x = nodeNode->getPos()[0];
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
	particleFilter pf(8000); // initiliaze the sample number as 50
	double test_x = 430.427, test_y = 32.1875, test_yaw = 0.0; // initialize the test vehicle position
	vector<double> stdv_pf = {4.0, 4.0, 0.1}; // start position's standard deviation [x, y, yaw] for particle filter
	Vehicle test_v(test_x, test_y, test_yaw, 40.0);
	pf.init(test_x, test_y, test_yaw, stdv_pf);
	vector<double> origin = {limit[0], limit[2]};
	test_v.fixOri(origin);
	test_v.addNoiseMotion(0.0, 0.5, 0.0, 0.02); // add noise to motion model's velocity and steering angle. In the order of:
										   		 // mean_v, stdev_v, mean_sa, stdev_sa. This step is necessary or the motion will be ground truth
	test_v.addNoiseMea(0.0, 0.02, 0.0, 0.01); // add noise to measurement model's range and angle. In the order of:
										   	  // mean_r, stdev_r, mean_a, stdev_a. This step is necessary or the measurement will be ground truth

  vector<double> goal = {0.0, 0.0}; // !!! Goal location of the vehicle: will be passed to A* later
	double init_x, init_y, heading;   // radar position, will used in the motion while loop


	string v_profile = "motion_command.dat"; // 1->v, 2->sa, 3->dt
	ifstream motion_c;
	motion_c.open(v_profile);
	string line;

	vector<double> noise_motion; // the noise motion command: [v_n, sa_n, dt]
	vector<vector<double>> measure;
	// radar settings
	Radar Radar_Handle(R_RADAR_X, R_RADAR_Y, R_RADAR_Z, R_RADAR_HEADING,
                       R_RADAR_RANGE, R_RADAR_AOS, R_RADAR_NUMDIR); // x,y,z,yaw, range,AoS, NUM_DIR
	vector<radar_t> measure_radar;

	ofstream output_lidar ("data/lidar.dat");
	ofstream output_pose ("data/robot_pos.dat");
	ofstream output_motion_mea ("data/robot_pos_mea.dat");
	ofstream output_est_state("data/estimated_pos.dat");

	int counter = 0;

	while(getline(motion_c, line)){
		++counter;
		// define the file names will make videos
		string lidar_every_step = "data/lidar_step" + to_string(counter) + ".dat"; // record every step's radar measurement
		string dynamic_every_step = "data/dynamic_step" + to_string(counter) + ".dat"; // record the dynamic map built in every step;
		string radar_every_step = "data/radar_step" + to_string(counter) + ".dat"; // record the every step's radar measurement
		// end define
		// open the files defined in last step
		ofstream lidar_step (lidar_every_step);
		ofstream radar_step (radar_every_step);
		ofstream dynamic_step (dynamic_every_step);
		// end open
		istringstream iss(line);
		double v, sa, dt;
		if(!(iss >> v >> sa >> dt)) break;
		test_v.move(v,sa,dt);
		noise_motion = test_v.move_mea(v,sa,dt);
		measure.clear();
		measure = test_v.scanMeasure(map_g, reso);
		// update all the obstacles by lidar data
		for(auto iter: measure){
	    	double o_x = test_v.getState()[0] + iter[0]*cos(iter[1]+test_v.getState()[2]);
	    	double o_y = test_v.getState()[1] + iter[0]*sin(iter[1]+test_v.getState()[2]);
	    	output_lidar << o_x << " " << o_y << endl;
				lidar_step << o_x << " " << o_y << endl; // record every step's measurement
    }
		lidar_step.close();
    output_pose << test_v.getState()[0] << " " << test_v.getState()[1] << endl;
    output_motion_mea << test_v.getMeaState()[0] << " " << test_v.getMeaState()[1] << endl;
    // obstacles update ends
		cout << "Total obstacles captured: " << measure.size() << endl;
		// Particle filter starts to take estimation
		pf.prediction(noise_motion);
		pf.updateWeights(map_d, measure, test_v, reso);
		pf.resample(test_v);
		pf.estState(test_v);
		pf.updateOccupancyMap(measure, map_d, test_v, reso);
		output_est_state << test_v.x_est << " " << test_v.y_est << " " << test_v.yaw_est << endl;
		// record current step's dynamic map
		if(dynamic_step.is_open())
		{
		    for(auto iter: map_d){
		    	for(auto node: iter){
		    		if(node->isOcc()){
		    			double o_x = node->getPos()[0];
		    			double o_y = node->getPos()[1];
				    	dynamic_step << o_x << " " << o_y << endl;
				    }
		    	}
		    }
		    dynamic_step.close();
		}
		// end record current step's dynamic map
		// radar start ->
		double space_idx = -1;
    if (test_v.x_est>=PARALLEL_SPACE_CORNER[0][0] && test_v.x_est<PARALLEL_SPACE_CORNER[1][0]) space_idx = 0;
    if (test_v.x_est>=PARALLEL_SPACE_CORNER[1][0] && test_v.x_est<PARALLEL_SPACE_CORNER[2][0]) space_idx = 1;
    if (test_v.x_est>=PARALLEL_SPACE_CORNER[2][0] && test_v.x_est<PARALLEL_SPACE_CORNER[3][0]) space_idx = 2;
    if (test_v.x_est>=PARALLEL_SPACE_CORNER[3][0] && test_v.x_est<PARALLEL_SPACE_CORNER[4][0]) space_idx = 3;
    if (test_v.x_est>=PARALLEL_SPACE_CORNER[4][0] && test_v.x_est<PARALLEL_SPACE_CORNER[5][0]) space_idx = 4;
    if (test_v.x_est>=PARALLEL_SPACE_CORNER[5][0]) space_idx = 5;
			// radar position
    init_x = test_v.x_est + Radar_Handle.getPose()[0];
    init_y = test_v.y_est + Radar_Handle.getPose()[1];
    heading = test_v.yaw_est + Radar_Handle.getPose()[3];
			// radar scan
		vector<double> vehicle_pose = {test_v.x_est, test_v.y_est, 0.0};
		measure_radar = Radar_Handle.scanRadar(map_g_radar, RES, vehicle_pose);
		if(space_idx != -1){
    	Radar_Handle.spaceFilter(measure_radar, space_idx, map_d_radar, limit_gt_radar, vehicle_pose, RES, "ONE_CHECK");
			// output the current step's radar measurement
			for (auto data : measure_radar) {
				vector<double> radar_pose_local = Radar_Handle.getPose();
				double init_x = test_v.x_est + radar_pose_local[0];
				double init_y = test_v.y_est + radar_pose_local[1];
				double heading = radar_pose_local[3] + test_v.yaw_est;
        // obstacle location: in global frame
        double ob_x_G = init_x + data.range * cos(data.alpha+heading);
        double ob_y_G = init_y + data.range * sin(data.alpha+heading);
				radar_step << ob_x_G << " " << ob_y_G << endl;
			}
			// output end;
		}
		radar_step.close();

			// check occupancy
		int check_idx = -1;
    if (test_v.x_est < (PARALLEL_SPACE_CORNER[1][0] + TOLERANCE) && test_v.x_est > (PARALLEL_SPACE_CORNER[1][0] - TOLERANCE)) check_idx = 0;
    if (test_v.x_est < (PARALLEL_SPACE_CORNER[2][0] + TOLERANCE) && test_v.x_est > (PARALLEL_SPACE_CORNER[2][0] - TOLERANCE)) check_idx = 1;
    if (test_v.x_est < (PARALLEL_SPACE_CORNER[3][0] + TOLERANCE) && test_v.x_est > (PARALLEL_SPACE_CORNER[3][0] - TOLERANCE)) check_idx = 2;
    if (test_v.x_est < (PARALLEL_SPACE_CORNER[4][0] + TOLERANCE) && test_v.x_est > (PARALLEL_SPACE_CORNER[4][0] - TOLERANCE)) check_idx = 3;
    if (test_v.x_est < (PARALLEL_SPACE_CORNER[5][0] + TOLERANCE) && test_v.x_est > (PARALLEL_SPACE_CORNER[5][0] - TOLERANCE)) check_idx = 4;
    if (test_v.x_est < (PARALLEL_SPACE_CORNER[5][0] + PARALLEL_SPACE_LENGTH + TOLERANCE) && test_v.x_est > (PARALLEL_SPACE_CORNER[5][0] + PARALLEL_SPACE_LENGTH - TOLERANCE)) check_idx = 5;
    if (check_idx != -1) {
	    bool free_space = Radar_Handle.occupancyCheck(map_d_radar, limit_gt_radar, RES, check_idx, goal);
			cout << "check idx: " << check_idx << endl;
			if(free_space){
			 cout << "Get free space and goal location found, g_x: " << goal[0] << ", g_y: " << goal[1] << endl;
			 // build map for hybrid A* start ->
			 vector<double> ox_map;
			 vector<double> oy_map;
			 for(auto& row: map_g_radar){
				 for(auto& node: row){
					 double mapx = node->getPos()[0], mapy = node->getPos()[1];
					 if(node->isOcc()){
						 ox_map.push_back(mapx);
						 oy_map.push_back(mapy);
					 }
				 }
			 }
			 Map obstacle_map(limit_gt_radar[0], limit_gt_radar[2], limit_gt_radar[1], limit_gt_radar[3], ox_map, oy_map); // this is the obstacle map will be used in Hybrid A* path planning
			 cout << "Road extraction is done" << endl;
			 // build map for hybrid A* end <-

			 // start path planning
			 /// -----------------------------   change to IPG parameters -----------------
			 double start_x = test_v.x_est;
			 double start_y = test_v.y_est;
			 double start_yaw0 = test_v.yaw_est;
			 double start_yaw1 = test_v.yaw_est;

			 double goal_x = goal[0];
			 double goal_y = goal[1];
			 double goal_yaw0 = 0;
			 double goal_yaw1 = 0;

			 // planning
			 Path_Final* path_ = calc_hybrid_astar_path(start_x, start_y, start_yaw0, start_yaw1,
			 											goal_x, goal_y, goal_yaw0, goal_yaw1, obstacle_map);

			 path_x = path_->x;
	     path_y = path_->y;
	     path_yaw = path_->yaw;
	     path_yaw1 = path_->yaw1;
	     path_direction = path_->direction;

			 break;
			}
	     //cout<<"Time: "<<t<<" x location: "<<test_v.x_est<<" the parking space "<<check_idx<<" is free: "<<free_space<<endl;
    }
		// radar end <-
		// Particle filter estimation ends
		cout << "finish the motion step " << counter << endl;
	}
	output_lidar.close();
	output_pose.close();
	output_motion_mea.close();
	output_est_state.close();
	// record the dynamic map based on the data from last several measurements
	ofstream output ("data/dynamic_map.dat");
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
	ofstream output_gt ("data/gt_map.dat");
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

	ofstream output_radar ("data/radar_dynamic_map.dat");
	if(output_radar.is_open())
	{
	    for(auto iter: map_d_radar){
	    	for(auto node: iter){
	    		if(node->isOcc()){
	    			double o_x = node->getPos()[0];
	    			double o_y = node->getPos()[1];
			    	output_radar << o_x << " " << o_y << endl;
			    }
	    	}
	    }
	    output_radar.close();
	}

	ofstream output_path ("data/Astart_Path.dat");
	if(output_path.is_open())
	{
    	for(int i = 0; i < path_x.size(); ++i){
		    output_path << path_x[i] << " " << path_y[i] << " " << path_yaw[i] << " " << path_yaw1[i] << endl;
    	}
	    output_path.close();
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
