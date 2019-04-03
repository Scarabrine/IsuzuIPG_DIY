//
//  Radar.cpp
//  Radar_Detection
//
//  Created by zhouforrest on 3/29/19.
//  Copyright © 2019 Ruilin. All rights reserved.
//

#include "Radar.h"



// -------------------   Radar Class Implementation  -----------------------
// Constructor
Radar::Radar(double x_in, double y_in, double z_in, double yaw_in, double range, double AoS, double NUM_DIR):
        x(x_in),
        y(y_in),
        z(z_in),
        yaw(yaw_in),
        range(range),
        AoS(AoS),
        NUM_DIR(NUM_DIR)
{
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    generator = default_random_engine(seed);
    distribution = normal_distribution<double>(0.0, 1.0);
}


// Get Radar Pose
vector<double> Radar::getPose() {
    return {x, y, z, yaw};
}

// Get Radar Ppecification
vector<double> Radar::getSpec() {
    return {range, AoS, NUM_DIR};
}

// Radar Scan
// BFS, search a sector region with angle of view, AoS [rad], and radius, range [m]
vector<radar_t> Radar::scanRadar(vector<vector<Cell*>>& map, double res, vector<double> vehicle_pose_global) {
    cout << "scan radar starts" << endl;
    // radar position in the global coordinate
    vector<double> radar_pose_local = Radar::getPose();
    double init_x = vehicle_pose_global[0] + radar_pose_local[0];
    double init_y = vehicle_pose_global[1] + radar_pose_local[1];
    double heading = radar_pose_local[3] + vehicle_pose_global[3];

    // radar searching angle of views (in global frame)
    double MIN_THETA = heading - AoS;
    double THETA_STEP = 2 * AoS / (NUM_DIR-1);

    // map info
    int x_size = map[0].size();
    int y_size = map.size();
    double MIN_X = map[0][0]->getPos()[0];
    double MIN_Y = map[0][0]->getPos()[1];
    double MAX_X = map[y_size-1][x_size-1]->getPos()[0];
    double MAX_Y = map[y_size-1][x_size-1]->getPos()[1];

    // output measurements
    measurements.clear();


    // searching in the sector region, array by array
    for (int dir=0; dir<NUM_DIR; dir++) {
        double theta = MIN_THETA + THETA_STEP * dir;
        for (double len = res; len<=range; len += res) {
            double sx = init_x + len * cos(theta);
            double sy = init_y + len * sin(theta);
            // convert from coordidate to idex
            int idx_j = (sx - MIN_X) / res;
            int idx_i = (sy - MIN_Y) / res;

            //cout<<"------> "<<sx<<", "<<sy<<endl;

            // check out of range
            if (idx_i<0 || idx_i>=y_size || idx_j<0 || idx_j>=x_size)
                break; // search another array
            // check occupancy
            if (map[idx_i][idx_j]->isOcc()) {
                // compute measurements (w.r.t. Radar Body_fixed frame)
                radar_t measure;
                measure.range = sqrt( (sx-init_x)*(sx-init_x) + (sy-init_y)*(sy-init_y) ); // range
                measure.alpha = atan2((sy-init_y), (sx-init_x)) - heading; // angle

                // add uncertainties
                double r1 = distribution(generator);
                double r2 = distribution(generator);
                //cout<<r1<<" "<<r2<<endl;
                double d_r = RADAR_NOISE[0] * r1 + RADAR_NOISE[2] * r2;
                double d_a = RADAR_NOISE[2] * r1 + RADAR_NOISE[1] * r2;
                measure.range += d_r;
                measure.alpha += d_a;

                // store in the internal structure
                measurements.push_back(measure);

                break; // search another array
            }
        }
    }


    /*
    // queues for search
    queue<Cell*> open_set;
    int idx_j = (init_x - MIN_X) / res;
    int idx_i = (init_y - MIN_Y) / res;
    open_set.push(map[idx_i][idx_j]);

    while(!open_set.empty()) {
        Cell* cur_node = open_set.front();
        open_set.pop();

        double cur_x = cur_node->getPos()[0];
        double cur_y = cur_node->getPos()[1];

        cout<<"size: "<<open_set.size()<<"------> "<<cur_x<<", "<<cur_y<<endl;

        for (int dir=0; dir<NUM_DIR; dir++) {
            double theta = MIN_THETA + THETA_STEP * dir;
            double sx = cur_x + res * cos(theta);
            double sy = cur_y + res * sin(theta);
            // convert from coordidate to idex
            idx_j = (sx - MIN_X) / res;
            idx_i = (sy - MIN_Y) / res;
            // check repeated
            // add to the query
            if (idx_i<0 || idx_i>=y_size || idx_j<0 || idx_j>x_size)
                continue;  // out of range
            else
                open_set.push(map[idx_i][idx_j]); // inside the map

            // check occupancy
            if (map[idx_i][idx_j]->isOcc()) {
                // compute measurements (w.r.t. Radar Body_fixed frame)
                radar_t measure;
                measure.range = sqrt( (sx-init_x)*(sx-init_x) + (sy-init_y)*(sy-init_y) ); // range
                measure.alpha = atan2((sy-init_y), (sx-init_x)) - heading; // angle

                // add uncertainties

                //
            }
        }
    }*/
    cout << "scan radar ends" << endl;
    return measurements;
}

void Radar::spaceFilter(vector<radar_t>& measurements, int space_idx, vector<vector<Cell*>>& map, vector<double> limit, vector<double> vehicle_pose_global, double res, string flag) {
    // parking space location: in global frame
    cout << "sapce filter start" << endl;
    double parking_theta = PARALLEL_SPACE_ORIENTATION[space_idx];
    double corner_x = PARALLEL_SPACE_CORNER[space_idx][0];
    double corner_y = PARALLEL_SPACE_CORNER[space_idx][1];

    // radar position: in global frame
    vector<double> radar_pose_local = Radar::getPose();
    double init_x = vehicle_pose_global[0] + radar_pose_local[0];
    double init_y = vehicle_pose_global[1] + radar_pose_local[1];
    double heading = radar_pose_local[3] + vehicle_pose_global[3];

    // Map Info
    double MIN_X = limit[0];
    double MIN_Y = limit[2];

    // filtering
    // ONE TIME CHECK: once found an obstacle, mark it as occupanied on the dynmaic map

    for (auto data : measurements) {
        // obstacle location: in global frame
        double ob_x_G = init_x + data.range * cos(data.alpha+heading);
        double ob_y_G = init_y + data.range * sin(data.alpha+heading);

        // obstacle location: in the local frame of the parking space
        double ob_x_B = (ob_x_G - corner_x) * cos(parking_theta) + (ob_y_G - corner_y) * sin(parking_theta);
        double ob_y_B = -(ob_x_G - corner_x) * sin(parking_theta) + (ob_y_G - corner_y) * cos(parking_theta);

        // --------------------------- occupancy check -----------------------------------------
        if (ob_x_B >= SPACE_SAFE_TOL && ob_x_B <= (PARALLEL_SPACE_LENGTH - SPACE_SAFE_TOL) &&
            ob_y_B >= SPACE_SAFE_TOL && ob_y_B <= (PARALLEL_SPACE_WIDTH - SPACE_SAFE_TOL) ) {
            // add the new pts to the current obstacle in the parking space
            // convert from coordidate to idex
            int idx_j = (ob_x_G - MIN_X) / res;
            int idx_i = (ob_y_G - MIN_Y) / res;

            if(!(idx_j >= 0 && idx_j < map[0].size() && idx_i >= 0 && idx_i < map.size())) continue;
            if (flag == "ONE_CHECK") {
               map[idx_i][idx_j]->changeOcc(true);
            }
            else if (flag == "CONTINUE_CHECK") {
               // do something
            }

            else {
               cout<<"Undefined mode: "<<flag<<endl;
               cout<<"Available modes are: CONTINUE_CHECK, ONE_CHECK"<<endl;
               throw -1;
            }
        }
    }
    cout << "sapce filter end" << endl;

}


// return true if not occuppied, otherwise return false
bool Radar::occupancyCheck(vector<vector<Cell*>>& map, vector<double> limit, double res, int space_idx, vector<double>& goal_pos) {

    // parking space location: in global frame
    double parking_theta = PARALLEL_SPACE_ORIENTATION[space_idx];
    double corner_x = PARALLEL_SPACE_CORNER[space_idx][0];
    double corner_y = PARALLEL_SPACE_CORNER[space_idx][1];

    // Map Info
    double MIN_X = limit[0];
    double MIN_Y = limit[2];

    const double threthold = 0.05; //
    const double safety_tol = 1.5; // unit: m

    int max_block_ct = 0;
    double max_bolck_ratio = 0.0;


    for (double local_y=safety_tol; local_y<=PARALLEL_SPACE_WIDTH-safety_tol; local_y += res) {
        max_block_ct = 0;

        for (double local_x=safety_tol; local_x<=PARALLEL_SPACE_LENGTH-safety_tol; local_x += res) {
            // convert from local frame to global frame
            double global_x = local_x * cos(parking_theta) - local_y * sin(parking_theta);
            double global_y = local_x * sin(parking_theta) + local_y * cos(parking_theta);
            global_x += corner_x;
            global_y += corner_y;

            // convert from coordinates to idx
            int idx_j = (global_x - MIN_X) / res;
            int idx_i = (global_y - MIN_Y) / res;

            // check occupancy
            if (map[idx_i][idx_j]->isOcc())
                max_block_ct++;
        }

        // cout<< "!!!!!!!!!!!" <<max_block_ct<<"/"<<int(PARALLEL_SPACE_LENGTH / res)<<endl;

        // obstacle too large
        max_bolck_ratio = max_block_ct * res / (PARALLEL_SPACE_LENGTH);
        if (max_bolck_ratio > threthold)
            return false;
    }

    // compute the goal location: in global frame
    double local_goal_x = safety_tol + (PARALLEL_SPACE_LENGTH - safety_tol*2 - TRUCK_L1 - TRUCK_L2) / 2
    + TRUCK_L1;
    double local_goal_y = PARALLEL_SPACE_WIDTH / 2;

    local_goal_x += corner_x;
    local_goal_y += corner_y;
    goal_pos[0] = local_goal_x * cos(parking_theta) - local_goal_y * sin(parking_theta);
    goal_pos[1] = local_goal_x * sin(parking_theta) + local_goal_y * cos(parking_theta);

    return true;
}
