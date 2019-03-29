/*
 * vehicle.h
 *
 *  Created on: March 28, 2019
 *      Author: Xingjian Liu
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "map.h"

class Vehicle{
public:
	Vehicle(double x_in, double y_in, double yaw_in, double range_in): x(x_in), y(y_in), yaw(yaw_in), range(range_in){}
	// motion model for the vehicle
	void move(double v, double sa, double dt);

	void fixOri(std::vector<double>& origin);
	
	std::vector<double> getState(){
		std::vector<double> res = {x,y,yaw};
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
	// parameter for scanner
	double range;
	//
	double ori_x;
	double ori_y;
};

std::string coorToStr(int x, int y);

#endif /* VEHICLE_H_ */