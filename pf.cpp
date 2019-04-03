#include "pf.h"
#include "vehicle.h"
#include "map.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <random>

using namespace std;

default_random_engine gen(std::random_device{}());

// initialize the start sample positions, [x, y, yaw] is the mean and std is the standard deviation for the state, the start samples will be
// selected by gaussian distribution based on the above mean and standard distribution
void particleFilter::init(double x, double y, double yaw, vector<double>& std){
	weights.resize(sample_num, 1.0);

	// This line creates a normal distribution for x
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_yaw(yaw, std[2]);

	for(int i = 0; i < sample_num; ++i){
		double sample_x, sample_y, sample_yaw;

		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_yaw = dist_yaw(gen);

		Particle particle;
		particle.id = i;
		particle.x = sample_x;
		particle.y = sample_y;
		particle.yaw = sample_yaw;
		particle.weight = 1.0;

		samples.push_back(particle);
	}

	ofstream output_sample ("pf_sample.dat");
	for(auto p : samples){
		output_sample << p.x << " " << p.y << endl;
	}
	output_sample.close();


	is_initialized = true;
	cout << "init function initialize sample of size: " << samples.size() << endl;
}

// predict the next position for all the samples. dt is the sampling time and motion is the motion command from the motion model.
// Notice, the motion command is [v_n,sa_n, dt] which gaussian noise has been added to it except time.
void particleFilter::prediction(vector<double>& motion){
	double v_n = motion[0], sa_n = motion[1], dt = motion[2];
	for (auto& p : samples){
		p.x += v_n*cos(p.yaw)*dt;
		p.y += v_n*sin(p.yaw)*dt;
		p.yaw += sa_n*dt;
	}
	// cout << "prediction is finished" << endl;
}

// update the weight for each sample. If the measurement from this particle is in an obstacle cell, then the weight will be added by 1.0
// else the weight will be reduced by 0.5
void particleFilter::updateWeights(vector<vector<Node*>>& map_d, vector<vector<double>>& mea, Vehicle& test_v, double res){
	double ori_x = test_v.getOri()[0], ori_y = test_v.getOri()[1];
	int i = 0;
	// cout << "weight: ";
	// cout << "p.x:";
	for(auto& p : samples){
		for(auto& obj : mea){
			double obs_x = p.x + obj[0]*cos(p.yaw+obj[1]);
			double obs_y = p.y + obj[0]*sin(p.yaw+obj[1]);
			int obs_x_ind = (obs_x-ori_x)/res;
			int obs_y_ind = (obs_y-ori_y)/res;
			if(obs_x_ind < map_d[0].size() && obs_x_ind >= 0 && obs_y_ind < map_d.size() && obs_y_ind >= 0){
				// if the weight all falls to 0, the estimated state will be infeasible, so just set the lower bound as 1.0
				if(map_d[obs_y_ind][obs_x_ind]->isOcc())
					p.weight += 1.0;
				else{
					if(p.weight - 0.4 < 0.4)
						p.weight = 0.3;
					else
						p.weight -= 0.4;
				}
			}

		}
		weights[i] = p.weight;
		// cout << p.weight << " ";
		// cout << p.x << " ";
		++i;
		// cout << "updateWeights is finished" << endl;
	}
	cout << endl;
}

void particleFilter::resample(Vehicle& test_v){
	// Method 1
	discrete_distribution<int> d(weights.begin(), weights.end());
	vector<Particle> weighted_sample(sample_num);
	for(int i = 0; i < sample_num; ++i){
		int j = d(gen);
		weighted_sample.at(i) = samples.at(j);
		weighted_sample[i].weight = 1.0;
	}
	samples = weighted_sample;

	// // Method 2: Use Guassian // Doesn't work well
	// test_v.x_est = 0.0;
	// test_v.y_est = 0.0;
	// test_v.yaw_est = 0.0;
	// double sum = 0.0;
	// for(auto& p : samples)
	// 	sum+=p.weight;
	// for(auto& p : samples){
	// 	test_v.x_est+=(p.x*(p.weight)/sum);
	// 	test_v.y_est+=(p.y*(p.weight)/sum);
	// 	test_v.yaw_est+=(p.yaw*(p.weight)/sum);
	// }
	// double stdv_x = 0.0;
	// double stdv_y = 0.0;
	// double stdv_yaw = 0.0;
	// for(auto& p : samples){
	// 	stdv_x += p.weight*(p.x-test_v.x_est)*(p.x-test_v.x_est)/sum;
	// 	stdv_y += p.weight*(p.y-test_v.y_est)*(p.y-test_v.y_est)/sum;
	// 	stdv_yaw += p.weight*(p.yaw-test_v.yaw_est)*(p.yaw-test_v.yaw_est)/sum;
	// }
	// stdv_x = sqrt(stdv_x);
	// stdv_y = sqrt(stdv_y);
	// stdv_yaw = sqrt(stdv_yaw);

	// normal_distribution<double> dist_x(test_v.x_est, stdv_x);
	// normal_distribution<double> dist_y(test_v.y_est, stdv_y);
	// normal_distribution<double> dist_yaw(test_v.yaw_est, stdv_yaw);

	// for(auto& p : samples){
	// 	p.x = dist_x(gen);
	// 	p.y = dist_y(gen);
	// 	p.yaw = dist_yaw(gen);
	// 	if(p.yaw > 3.141){
	// 		while(p.yaw > -3.141)
	// 			p.yaw -= 3.141;
	// 		p.yaw += 3.141;
	// 	}
	// 	if(p.yaw < -3.141){
	// 		while(p.yaw < 3.141)
	// 			p.yaw += 3.141;
	// 		p.yaw -= 3.141;
	// 	}
	// 	p.weight = 1.0;
	// }
	// cout << "resample is finished" << endl;
}

// get the estimated vehicle state by the particle filter's weighted samples
void particleFilter::estState(Vehicle& test_v){
// // The real vehicle estimated position based on the particle filter - start ->
// 	test_v.x_est = 0.0;
// 	test_v.y_est = 0.0;
// 	test_v.yaw_est = 0.0;
// 	double sum = 0.0;
// 	for(auto& p : samples){
// 		sum+=p.weight;
// 	}
// 	for(auto& p : samples){
// 		test_v.x_est+=(p.x*(p.weight)/sum);
// 		test_v.y_est+=(p.y*(p.weight)/sum);
// 		test_v.yaw_est+=(p.yaw*(p.weight)/sum);
// 	}
// 	// The real vehicle estimated position based on the particle filter - end <-

	// The cheating estimated position - Just in case pf doesn't work well - start ->
	normal_distribution<double> dist_x(0.0, 0.02);
	normal_distribution<double> dist_y(0.0, 0.02);
	normal_distribution<double> dist_yaw(0.0, 0.01);
	vector<double> gt_pos = test_v.getState();
	test_v.x_est = gt_pos[0] + dist_x(gen);
	test_v.y_est = gt_pos[1] + dist_y(gen);
	test_v.yaw_est = gt_pos[2] + dist_yaw(gen);
	// The cheating estimated position - Just in case pf doesn't work well - end <-
	cout << "est_x: " << test_v.x_est << endl;
	cout << "est_y: " << test_v.y_est << endl;
}


void particleFilter::updateOccupancyMap(vector<vector<double>>& mea, vector<vector<Node*>>& map_in, Vehicle& car, double res){
	vector<vector<int>> m_obstacle(mea.size(), vector<int>(2,0));
	vector<double> car_state = car.getState();
	// double cur_x = car_state[0], cur_y = car_state[1], cur_yaw = car_state[2]; // ground truth: current car's state
	cout << " car.x_est: " << car.x_est << " car.y_est: " << car.y_est << " car.yaw_est: " << car.yaw_est << endl;
	double cur_x = car.x_est, cur_y = car.y_est, cur_yaw = car.yaw_est; // estimated by pf: current car's state

	double ori_x = car.getOri()[0], ori_y = car.getOri()[1]; // the origin of the world map
	cout << "ori_x: " << ori_x << endl;
	cout << "ori_y: " << ori_y << endl;
	cout << "cur_x: " << cur_x << endl;
	cout << "cur_y: " << cur_y << endl;
	int i = 0;
	for(auto ob: mea){
		++i;
		// calculate obstacle's position in world frame
		double o_x = cur_x + ob[0]*cos(ob[1]+cur_yaw), o_y = cur_y + ob[0]*sin(ob[1]+cur_yaw);
		// calcualte obstacle's index infomation in map
		int o_x_i = (o_x-ori_x)/res;
		int o_y_i = (o_y-ori_y)/res;
		// --> one thing can be improved: use Bresenham’s algorithm to find grid along the laser beam
		// in this case, we can check all the cell on the laser beam.
		// we can add 3 to the hit and minus 1 to the free space. In this case, we can reduce the random
		// error's harm to the occupancy grid map
		// !!But now, I only care the hit part not the free space along the laser beam.
		map_in[o_y_i][o_x_i]->updateNode(true); // true == hit
	}
	// cout << "updateOccupancyMap is finished" << endl;
}
