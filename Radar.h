//
//  Radar.h
//  Radar_Detection
//
//  Created by zhouforrest on 3/29/19.
//  Copyright © 2019 Ruilin. All rights reserved.
//

#ifndef Radar_h
#define Radar_h

#include "map.h"
#include "Parameters.hpp"

#include <vector>
#include <string>
#include <queue>
#include <cmath>
#include <unordered_set>
#include <ctime>
#include <stdlib.h>
#include <iostream>
#include <random>
#include <chrono>



using namespace std;

// radar data structure
struct radar_t {
    double range;  // measurement distance
    double alpha; //  measurement angle
};

class Radar {

private:
    double x, y, z, yaw; // radar sensor location, relative to the real end of the vehicle
    double range; // radar range in radius
    double AoS; // viewing angle: [-AoS, +AoS] in radian
    double NUM_DIR; // number of directions for searching

    vector<radar_t> measurements; // radar measurements

    default_random_engine generator;
    normal_distribution<double> distribution; // normal random generator

public:
    // constructor
    Radar(double x_in, double y_in, double z_in, double yaw_in, double range, double AoS, double NUM_DIR);

    // get Radar pose
    // return: (x,y,z,yaw) in a vector
    vector<double> getPose();

    // get Radar Specification
    // return: (range, AoS, NUM_DIR)
    vector<double> getSpec();

    // Radar Scan
    // return: a 2D map with detected obstacles from the static map with vehicles (the ground truth)
    vector<radar_t> scanRadar(vector<vector<Cell*>>& map, double res, vector<double> vehicle_pose_global);

    // Filter the Raw Data, leave only obstacles in the parking space, store obstacles on the dynamic map
    void spaceFilter(vector<radar_t>& measurements, int space_idx, vector<vector<Cell*>>& map, vector<double> limit, vector<double> vehicle_pose_global, double res, string flag);

    // Occupancy Check: check the space availability of the parking space numbered by space_idx
    bool occupancyCheck(vector<vector<Cell*>>& map, vector<double> limit, double res, int space_idx, vector<double>& goal_pos);
};

#endif /* Radar_h */
