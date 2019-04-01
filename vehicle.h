/*
 * vehicle.h
 *
 *  Created on: March 28, 2019
 *      Author: Xingjian Liu
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "map.h"
#include <random>

class Vehicle{
public:
	Vehicle(double x_in, double y_in, double yaw_in, double range_in){
		x = x_in;
		y = y_in;
		yaw = yaw_in;
		x_mea = x_in;
		y_mea = y_in;
		yaw_mea = yaw_in;
		range = range_in;
		mean_v = 0.0;
		stdev_v = 0.0;
		mean_sa = 0.0;
		stdev_sa = 0.0;
	}
	// motion model for the vehicle
	void move(double v, double sa, double dt);

	void move_mea(double v, double sa, double dt);

	void fixOri(std::vector<double>& origin);

	void addNoiseMotion(double mean_v_in, double stdev_v_in, double mean_sa_in, double stdev_sa_in){
		mean_v = mean_v_in;
		stdev_v = stdev_v_in;
		mean_sa = mean_sa_in;
		stdev_sa = stdev_sa_in;
	}

	void addNoiseMea(double mean_r_in, double stdev_r_in, double mean_a_in, double stdev_a_in){
		mean_r = mean_r_in;
		stdev_r = stdev_r_in;
		mean_a = mean_a_in;
		stdev_a = stdev_a_in;
	}
	
	std::vector<double> getState(){
		std::vector<double> res = {x,y,yaw};
		return res;
	}

	std::vector<double> getMeaState(){
		std::vector<double> res = {x_mea, y_mea, yaw_mea};
		return res;
	}

	std::vector<double> getOri(){
		std::vector<double> res = {ori_x, ori_y};
		return res;
	}

	// return all the obstacles can be measured by the laser scanner
	std::vector<std::vector<double>> scanMeasure(std::vector<std::vector<Node*>>& map_in, double res);


private:
	// parameter for vehicle
	double x;
	double y;
	double yaw;
	double x_mea;
	double y_mea;
	double yaw_mea;
	// parameter for scanner
	double range;
	//
	double ori_x;
	double ori_y;
	// noise for motion and measurment model
	double mean_v;
	double mean_sa;
	double stdev_v;
	double stdev_sa;
	double mean_r;
	double mean_a;
	double stdev_r;
	double stdev_a;
};

std::string coorToStr(int x, int y);

#endif /* VEHICLE_H_ */