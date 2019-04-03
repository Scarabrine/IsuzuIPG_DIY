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
You can find an example of using these functions in the main.cpp - part simulate a car is moving based on velocity and steering command. The functions should be called in the above order, where **init** is called only by once to generate gaussian ditributed initial particles. Then, in each simulation time step, the rest functions is called in order. **prediction** first predicts all the particles next step's state based on the current noise motion command. Then **updateWeights** takes the lidar measurement and based on the dynamic map gives each particle a weight which stands for the possiblity this particle being true. **resample** does resample based on the the last step's weight. **estState** estimates the current vehicle's states by the weighted mean value off all the particles. At last, **updateOccupancyMap** updates the dynamic map. Only the cells hitted by laser beam for several times will be considered as obstacles (the hit number can be customized). 
The last step by **updateOccupancyMap** corresponds to the exploration task. By this function, we can keep updating the dynamic map and includes those dynamic obstacles like parking vehicles into the map.

## Parkspace finding

## Path Planning
The path planning part is adapted from [Atsushi Sakai](https://atsushisakai.github.io/HybridAStarTrailer/)'s git repo. Hybrid A* is modification based on A* which enables the path planner to plan a feasible path for trailer truck.

## Code Structure
In this part, I will go through the structure of each function briefly

### map.cpp & map.h
**map** defines the map we use in the simulation. The map is in occupancy grid map form and each cell is represents by the class **Node**. **Node** contains information includes real work position (double x, double y), whether this cell is occupied (bool occu), the possiblity of this cell being a obstacle (poss) and the thresold value we consider this cell as a obstalce (thre). Only when the **poss** exceeds 'thre' can we consider **occu** being true.
Moreover, **map.cpp** includes two map initialization functions. **vector<double> initMap** generates a static map based on the .dat file from IPG. The static map will only contains the unchanged features in the parking lot. This comes from another assumption we have - to ease the computational power we spend on mapping and localzation, we will first get the offline map of the parking lot (this is a general solution for current industry). To avoid cheating, we only contain the unchanged features in this offline map like poler or lamps. **void initMap** generates a blank dynamic map with the same size as static map. The dynamic map will contains all the features captured by lidar. All in all, the coordinates for each cell increases as the x and y index increase.
  
### vehicle.cpp & vehicle.h
The **vehilce.cpp** and **vehicle.h** mainly contain a class called **Vehicle**. This class includes all the parameters of the vehicle:
* **x, y, yaw** -> These three parameters correspond to the ground truth position and heading angle of vehicle.
* **x_mea, y_mea, yaw_mea** -> These three parameters correspond to the position when the measured speed and steering angle rate has noise. That's say, this state is off from ground truth mentioned above due to the measurement noise.
* **x_est, y_est, yaw_est** -> These three parameters correspond to the estimated position by particle filter, which should be more accurate than **x_mea** groups. You should use this as the vehicle position in future work because it's impossible to get ground truth position in real world.
* **ori_x, ori_y** -> The origin point of map, aka map[0][0] point. 
* **addNoiseMotion** -> This function define the noise when measure vehicle's speed and steering rate. The noise is assumed to be gaussian, so four input are velocity and steering rate's mean and standard deviation.
* **addNoiseMea** -> This function define the noise of lidar measurement. The noise is also in gaussian form. Four inputs correpond to measurement range and bearing's mean and standard deviation.
* **getMeaState** -> Returns the vehilce's position only based on odometer information, aka **x_mea, y_mea, yaw_mea**
* **scanMeasure** -> This is actually the lidar's scan model. We didn't seperate it as an independent class. The scan model is simplified but still approximates a high resolution lidar's function. The only property for lidar is **range** which is how far the lidar can see. In this case, we do a typical BFS search within a circle defined by **range**. Every obstacle which has no other between vehicle positoin and itself will be measured. The measurement is the distance and relative angle of obstacle to the current vehicle state. **!!Notice**, the relative angle is in vehicle's coordinate, **!!Not** world frame. Whenever a measurement is taken, the angle it has blocked will be added to an vector. Everytime, when we find a obstacle within the range, we will first check if it's blocked by the previous obstacles. *Hint:* if we don't want to simulate a high resolution lidar, we can use Bresenham's algorithm to only check the cells on the laser beam we are intrested in. This will reduce the computational compleixty to O(n) not O(n^2) in the old version, where n is the range of lidar.

### radar.cpp && radar.h

### pf.cpp and pf.h

## Expected Outcomes
