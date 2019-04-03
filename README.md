# IsuzuIPG_DIY
Include necessary sensor like radar and Lidar. Can also finish parking lot exploration

## Introduction
This is a supplementary part for the current IPG vehicle simulation software. Because IPG is not open source software, it's hard for us to develop within the software, especially when IPG misses some important sensors for autonomous driving like lidar. Also, for some other sensor like radar, the information from IPG has already been highly preprocessed - software directly gives us a bonding box for the obstacles which is more like cheating. Based on this, we decide to develop a plugin for IPG which includes lidar and radar model and hopefully, this will help on our auto-parking project (especially the localization part) in Isuzu Technical Center of America.
The final outcomes is that we can extract a map.dat from IPG and based on the map, trailer truck can localize and find a parking space in the same time. After it finds a parking space, the Hybrid A* path planner will find a path for it to do the task like parallel or reverse parking.

## How To Use
compile with g++ compiler includes c++11 library
```
g++ --std=c++11 main.cpp map.cpp vehicle.cpp pf.cpp radar.cpp -o out
```
when run the code, does
```
./out
```
when plot the results, use map_plot.py
```
python map_plot.py
```

## Subsystems
![alt text](https://github.com/Scarabrine/IsuzuIPG_DIY/blob/master/image/localization.png)

## Localization and Exploration
The localization part uses classic particle filter idea. It estimates the vehicle position by lidar measurement and odometer reading. Both inputs are subjected to customized gaussian noise. The localization is mainly in pf.cpp, which has following functions:
```
void particleFilter::init(double x, double y, double yaw, vector<double>& std)
void particleFilter::prediction(vector<double>& motion)
void particleFilter::updateWeights(vector<vector<Node*>>& map_d, vector<vector<double>>& mea, Vehicle& test_v, double res)
void particleFilter::resample(Vehicle& test_v)
void particleFilter::estState(Vehicle& test_v)
void particleFilter::updateOccupancyMap(vector<vector<double>>& mea, vector<vector<Node*>>& map_in, Vehicle& car, double res)
```
You can find an example of using these functions in the main.cpp - part simulate a car is moving based on velocity and steering command. The functions should be called in the above order, where 'init' is called only by once to generate gaussian ditributed initial particles. Then, in each simulation time step, the rest functions is called in order. 'prediction' first predicts all the particles next step's state based on the current noise motion command. Then 'updateWeights' takes the lidar measurement and based on the dynamic map gives each particle a weight which stands for the possiblity this particle being true. 'resample' does resample based on the the last step's weight. 'estState' estimates the current vehicle's states by the weighted mean value off all the particles. At last, 'updateOccupancyMap' updates the dynamic map. Only the cells hitted by laser beam for several times will be considered as obstacles (the hit number can be customized). 
The last step by 'updateOccupancyMap' corresponds to the exploration task. By this function, we can keep updating the dynamic map and includes those dynamic obstacles like parking vehicles into the map.

## Parkspace finding

## Path Planning
The path planning part is adapted from [Atsushi Sakai](https://atsushisakai.github.io/HybridAStarTrailer/)'s git repo. Hybrid A* is modification based on A* which enables the path planner to plan a feasible path for trailer truck.

## Code Structure
In this part, I will go through the structure of each function briefly
### map.cpp & map.h
'map' defines the map we use in the simulation. The map is in occupancy grid map form and each cell is represents by the class 'Node'. 'Node' contains information includes real work position (double x, double y), whether this cell is occupied (bool occu), the possiblity of this cell being a obstacle (poss) and the thresold value we consider this cell as a obstalce (thre). Only when the 'poss' exceeds 'thre' can we consider 'occu' being true.
Moreover, 'map.cpp' includes two map initialization functions. 'vector<double> initMap' generates a static map based on the .dat file from IPG. The static map will only contains the unchanged features in the parking lot. This comes from another assumption we have - to ease the computational power we spend on mapping and localzation, we will first get the offline map of the parking lot (this is a general solution for current industry). To avoid cheating, we only contain the unchanged features in this offline map like poler or lamps. 'void initMap' generates a blank dynamic map with the same size as static map. The dynamic map will contains all the features captured by lidar.
  
### vehicle.cpp & vehicle.h

### radar.cpp && radar.h

### pf.cpp and pf.h
