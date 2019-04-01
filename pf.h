/*
 * pf.h
 *
 *  Created on: March 29, 2019
 *      Author: Xingjian Liu
 */

#ifndef PF_H_
#define PF_H_

#include <vector>
#include "map.h"
#include "vehicle.h"

// structure of the particle filter is referenced from Tiffany Huang from UdayCity project

struct Particle{
	int id;
	double x;
	double y;
	double yaw;
	double weight;
};

class particleFilter{
public:
	bool is_initialized;
	// update the possibilty of a node's obstacle state --> for occupancy grid map
	void updateOccupancyMap(std::vector<std::vector<double>>& mea, std::vector<std::vector<Node*>>& map_in, Vehicle& car, double res);
	
	particleFilter():sample_num(0). is_initialized(false){}
	
	~particleFilter(){}

	void init(double x, double y, double yaw, std::vecotr<double>& weights);

	void prediction(double dt, std::vector<double>& motion);

	// std_mea. Noise measurement for range and angle
	void updateWeights(double sensor_range, std::vector<double>& std_mea);

	void resample();

	const bool initialized() const{
		return is_initialized;
	}
private: 
	int sample_num;
	std::vector<Particle> samples;
	std::vector<double> weights;
};
#endif /* PF_H_ */
