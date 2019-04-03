//
//  Parameters.cpp
//  Radar_Detection
//
//  Created by zhouforrest on 4/1/19.
//  Copyright © 2019 Ruilin. All rights reserved.
//
#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <stdio.h>
#include <vector>
#include <cmath>

using namespace::std;

// parallel parking space dimensions
static double PARALLEL_SPACE_WIDTH = 6; // y axis
static double PARALLEL_SPACE_LENGTH = 21; // x axis
static double SPACE_SAFE_TOL = 0.3;
static vector<double> PARALLEL_SPACE_ORIENTATION = {0, 0, 0, 0, 0}; // parking space orientation
static vector<vector<double>> PARALLEL_SPACE_CORNER = {
    {486, 23.3},
    {511, 23.3},
    {536, 23.3},
    {561, 23.3},
    {586, 23.3},
    {611, 23.3}
}; // bottom left corner
static double TOLERANCE = 2.0; // Tolerance for parking space


// trailer-truck dimensions
static double TRUCK_L1 = 11.135; // from rear axes to trailer end
static double TRUCK_L2 = 4.54; // from rear axes to truck front
static double TRUCK_W = 2.06; // vehicle width

// Right Radar configuration
static double RES = 0.1;
static double R_RADAR_X = 5;
static double R_RADAR_Y = 1.2;
static double R_RADAR_Z = 0.5;
static double R_RADAR_HEADING = - M_PI / 2;
static double R_RADAR_RANGE = 10;
static double R_RADAR_AOS = M_PI / 3;
static double R_RADAR_NUMDIR = 50;

// Radar Noise
static vector<double> RADAR_NOISE = {0.1, M_PI/180, 0.0001}; // range std, angle std, covariance

#endif /* PARAMETERS_H_ */
