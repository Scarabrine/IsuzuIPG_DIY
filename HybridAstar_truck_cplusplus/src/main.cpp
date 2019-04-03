/* Copyright 2018 Weilun Peng */
// main function

#include <iostream>
#include <fstream>
#include <vector>

#include "map.hpp"
#include "pathfinder_hybrid_astar.hpp"
#include "def_all.hpp"

using namespace std;

#define MIN_X_M -6.0
#define MAX_X_M 60.0
#define MIN_Y_M -50.0
#define MAX_Y_M 15.0
#define STEP_SIZE 0.5

int main(int argc, char **argv) {
    // set start and goal configuration
    // double sx = 58.202;  // [m]
    // double sy = 7.10137;  // [m]
    // double syaw0 = 180*D2R;
    // double syaw1 = 180*D2R;

    double sx = 12.2061;  // [m]
    double sy = -14.7;  // [m]
    double syaw0 = 0.0*D2R;
    double syaw1 = 0.0*D2R;

    double gx = 7.46;  // [m]
    double gy = -8.48;  // [m]
    double gyaw0 = -90.0*D2R;
    double gyaw1 = -90.0*D2R;

    // double gx = 178.0;  // [m]
    // double gy = 2.0;  // [m]
    // double gyaw0 = 0.0*D2R;
    // double gyaw1 = 0.0*D2R;

    // double gx = 88.0;  // [m]
    // double gy = 195.0;  // [m]
    // double gyaw0 = 90.0*D2R;
    // double gyaw1 = 90.0*D2R;
    
    cout << "Start Configuration: (" << sx << ", " << sy << ", " << syaw0*R2D << ", " << syaw1*R2D << ")" << endl;
    cout << "Goal Configuration: (" << gx << ", " << gy << ", " << gyaw0*R2D << ", " << gyaw1*R2D << ")" << endl;

    ifstream infile;
    infile.open("/data/Simulators/IPG_TruckMaker/CD-TruckMakerOffice-linux-6.4/ros/ros1_ws/Autoparking/map.dat");
    vector<double> ox_map;
    vector<double> oy_map;
    double x_val, y_val;
    while (infile >> x_val >> y_val)
    {
        ox_map.emplace_back(x_val);
        oy_map.emplace_back(y_val);
    }
    
    Map map(MIN_X_M, MIN_Y_M, MAX_X_M, MAX_Y_M, ox_map, oy_map);
    //find the final path
    Path_Final* path = calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, map);
    cout << "Program Done!!" <<endl;

    // save data to a txt
    ofstream savefile;
    savefile.open ("simulation/path.dat");
    for (size_t i = 0; i < path->x.size(); i++) {
        savefile << path->x[i] << " " << path->y[i] << " " << path->yaw[i] << " " << path->yaw1[i] << " " << path->direction[i] << endl;
    }
    savefile.close();

    savefile.open ("simulation/startandgoal.dat");
    savefile << sx << " " << sy << " " << syaw0 << " " << syaw1 << " " << gx << " " << gy << " " << gyaw0 << " " << gyaw1 << endl;
    savefile.close();

    savefile.open ("simulation/map.dat");
    for (size_t i = 0; i < ox_map.size(); i++) {
        savefile << ox_map[i] << " " << oy_map[i] << endl;
    }
    savefile.close();
    
    return 0;

}