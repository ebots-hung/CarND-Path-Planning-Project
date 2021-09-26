# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

[//]: # (Image References)

[uml_1]: ./output/UML_PathPlanning_ClassDiagram.png "PathPlanning_ClassDiagram"
[uml_2]: ./output/UML_Spline_ClassDiagram.png "Spline_ClassDiagram"
[control_scheme]: ./output/control_scheme.png "Control Scheme"
[behavior]: ./output/behavior_planning.png "Behavior Planning"
[behavior_action]: ./output/behavior_planning_actions.png "Behavior Actions"
[trajectory]: ./output/Trajectory_Generation.png "Trajectory Generation"
[best_drive]: ./output/driving_distance_wo_incident.png "Best Drive"

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.csv
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554 (about 4.32 miles).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Path Planning Implementation

### Repository folder structure
```
CarND-Path-Planning-Project
├── CMakeLists.txt
├── cmakepatch.txt
├── data
│   └── highway_map.csv
├── install-mac.sh
├── install-ubuntu.sh
├── LICENSE
├── output
│   ├── Behavior_planning.png
│   ├── control_scheme.png
│   ├── driving_distance_wo_incident.png
│   ├── Trajectory_Generation.png
│   ├── UML_PathPlanning_ClassDiagram.png
│   └── UML_Spline_ClassDiagram.png
├── README.md
├── set_git.sh
└── src
    ├── helpers.h
    ├── json.hpp
    ├── main.cpp
    ├── pathplanning.h
    └── spline.h
```
### Classes Diagram
Path Planning class diagram:
![alttext][uml_1]

Spline Interpolation class diagram:
![alttext][uml_2]

## Implementation details
Path planning module includes of tree parts: prediction, behavior planning and trajectory generation. 

![alttext][control_scheme]

Path Planning object is intanstiated in the [`main.cpp`], with waypoint of high way as the inputs. 

### Prediction

Preduction function is a member of Path Planning class ['pathplanning.h']. The goal of the prediction function is to study the surrounding environment and check the status of all the other vehicles around the ego-vehicle. The following situations are considered:

* If there is a vehicle is driving in front of the ego-vehicle blocking the traffic and has a close distance from the ego-vehicle.
* If there is a vehicle is driving to the right of the ego-vehicle at a closed distance, then making a right lane change really unsafe.
* If there is a vehicle is driving to the left of the ego-vehicle at a closed distance, then making a left lane change really unsafe.

It is safe is distance from ego-vehicle to other vehicles greater than 45 meters, otherwise it is unsafe. 

### Behavior Planning

Behavior planning is basically getting input from prediction function and perform the coressponding actions. This function is a member of Path Planning class ['pathplanning.h']. 
![alttext][behavior]

Behavior planning function is also doing a high priority task to ensure the ego-vehicle in the middle lane. 

![alttext][behavior_action]

### Trajectory Generation

Trajectory Generation function is getting behavior planning input, ego-vehicle coordinates and the past points to calculate the desired trajectory. This function is a member of of Path Planning class ['pathplanning.h']

A smooth trajectory is calculated using the spline which contains information about previous path points of the  vehicle and some future points from the map. The actual future path points of the ego vehicle are derived from the spline. This will help in doing a smooth trajectory and avoids increasing the jerk.

In order to avoid sudden changes in the velocity, the distance between the points in the path are incrementally increased or decreased.

![alttext][trajectory]

## Results 

1. **The car is able to drive at least 4.32 miles without incident.**: The vehicle was able to drive more than 4.32 miles without any incident.

![alttext][best_drive]

2. **The car drives according to the speed limit.**: The car did not exceed the maximum allowed speed limit of 50 MPH

3. **Max Acceleration and Jerk are not Exceeded.**: The maximum acceleration of 10 m/s^2 and jerk of 10 m/s^3 were not exceeded
   
4. **Car does not have collisions.**: Within all the driving tests, the car did not have any collisions.
   
5. **The car stays in its lane, except for the time between changing lanes.**: Based on the intenteded behavior, the car was always staying in its lane, except when doing a lane change due to a vehicle which is obstructing the traffic flow in front of it, or when it is on the left or right side, and it is safe to go back to the middle lane.
   
6. **The car is able to change lanes.**: The car was able to change lanes smoothly to left or right, when there is a vehicle in the front obstructing the traffic flow, or when it is returning back to the middle lane (if it is safe).

## Reference
Project is using Cubic Spline interpolation library for smooth trajectory creation. 
https://kluge.in-chemnitz.de/opensource/spline/ 
