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

// update the possibilty of a node's obstacle state --> for occupancy grid map
void updateOccupancyMap(std::vector<std::vector<double>>& mea, std::vector<std::vector<Node*>>& map_in, Vehicle& car, double res);


#endif /* PF_H_ */
