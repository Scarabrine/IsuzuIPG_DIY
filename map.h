/*
 * map.h
 *
 *  Created on: March 27, 2019
 *      Author: Xingjian Liu
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <string>

class Node {
public:
	Node(float x_in, float y_in, bool occu_in): x(x_in), y(y_in), occu(occu_in){}
	void changeOcc(bool in){
		occu = in;
	}
	std::vector<double> getPos(){
		std::vector<double> res = {x,y};
		return res;
	}
	bool isOcc(){
		return occu;
	}
	void possSet(int8_t poss_in, int8_t thre_in){
		poss = poss_in;
		thre = thre_in;
		return;
	}
	void updateNode(bool hit){
		// hit +3 travel though -1
		if(hit && poss < 125) poss+=3;
		else if(!hit && poss > -126) --poss;
		if(poss >= thre)
			occu = false;
	}

private:
	float x;
	float y;
	bool occu;
	int park_num;
	int8_t poss; // the possibbily of a node being obstacle
	int8_t thre; // the thersold for a node being obstacle

};

std::vector<double> initMap(std::vector<std::vector<Node*>>& map, std::string dir, double res);
void initMap(std::vector<std::vector<Node*>>& map, std::vector<double> limit, double res, int8_t poss, int8_t thre);

#endif /* MAP_H_ */
