/*
 * map.cpp
 *
 *  Created on: Mar 27, 2019
 *      Author: Xingjian Liu
 */

#include "map.h"

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <fstream>
#include <vector>
#include <string>
#include <climits>
#include <cfloat>
#include <sstream>


using namespace std;

// Turn the IPG map.dat to the static map
vector<double> initMap(vector<vector<Node*>>& map, string dir, double res){
	vector<vector<double>> ob; // all the obstacles in the map.dat;
	// open the map data
	ifstream map_in;
	map_in.open(dir);
	string line;

	while(getline(map_in, line)){
		istringstream iss(line);
		double x, y;
		if(!(iss >> x >> y)) break;
		vector<double> temp = {x, y};
		ob.push_back(temp);
	}

	// find the size of map and left bottom points
	double x_min = DBL_MAX, y_min = DBL_MAX, x_max = DBL_MIN, y_max = DBL_MIN;
	for(auto i: ob){
		if(i[0] > x_max) x_max = i[0];
		if(i[0] < x_min) x_min = i[0];
		if(i[1] > y_max) y_max = i[1];
		if(i[1] < y_min) y_min = i[1];
	}
	int size_x = (x_max-x_min)/res+1;
	int size_y = (y_max-y_min)/res+1;
	vector<vector<Node*>> ini(size_y, vector<Node*>(size_x, new Node(0.0, 0.0, false)));
	// initilize all the static map point - only the coordinate
	for(int i = 0; i < size_x; ++i){
		for(int j = 0; j < size_y; ++j){
			double x_ini = x_min + res*i, y_ini = y_min + res*j;
			bool occ_ini = false;
			Node* node_new = new Node(x_ini, y_ini, occ_ini);
			ini[j][i] = node_new;
		}
	}
	// update all the obstale points
	for(auto i: ob){
		int x_idx = (i[0]-x_min)/res, y_idx = (i[1]-y_min)/res;
		ini[y_idx][x_idx]->changeOcc(true);
	}

	map = ini;

	vector<double> limit = {x_min, x_max, y_min, y_max};

	return limit;
}

// create a blank map for dynamic map
void initMap(vector<vector<Node*>>& map, vector<double> limit, double res, int8_t poss, int8_t thre){
	double x_min = limit[0], x_max = limit[1], y_min = limit[2], y_max = limit[3];
	int size_x = (x_max-x_min)/res+1;
	int size_y = (y_max-y_min)/res+1;
	vector<vector<Node*>> ini(size_y, vector<Node*>(size_x, new Node(0.0, 0.0, false)));
	for(int i = 0; i < size_y; ++i){
		for(int j = 0; j < size_x; ++j){
			double x_ini = x_min + res*i, y_ini = y_min + res*j;
			bool occ_ini = false;
			Node* node_new = new Node(x_ini, y_ini, occ_ini);
			ini[j][i] = node_new;
			node_new->possSet(poss, thre);
		}
	}
	map = ini;
	return;
}













