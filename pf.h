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

// structure of the particle filter adapted from Tiffany Huang's UdayCity project
// the resample and updateWeights functions are totally new

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
	void updateOccupancyMap(std::vector<std::vector<double>>& mea, std::vector<std::vector<Cell*>>& map_in, Vehicle& car, double res);

	particleFilter():sample_num(0), is_initialized(false){}

	particleFilter(int sample_num_in): sample_num(sample_num_in), is_initialized(false){}

	~particleFilter(){}

	// initialize the start sample positions, [x, y, yaw] is the mean and std is the standard deviation for the state, the start samples will be
	// selected by gaussian distribution based on the above mean and standard distribution
	void init(double x, double y, double yaw, std::vector<double>& std);

	void prediction(std::vector<double>& motion);

	// map_d is the dynamic map. mea is the noise measurement from vehicle.scanMeasure() = [range, angle]
	// the test_v is also needed because the map's origin is required to calculate the index of bostacle.
	void updateWeights(std::vector<std::vector<Cell*>>& map_d, std::vector<std::vector<double>>& mea, Vehicle& test_v, double res);

	void resample(Vehicle& test_v);

	void estState(Vehicle& test_v);

	const bool initialized() const{
		return is_initialized;
	}
private:
	int sample_num;
	std::vector<Particle> samples;
	std::vector<double> weights;
};
#endif /* PF_H_ */
