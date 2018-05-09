# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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


## Model Documentation
Here I start by setting the `prev_size` as the size of the list of previously generated points that were not used by the car. If there is any such point then I set the s value of the last point as the current `car_s` value. Then I iterate through the list of sensor values in the sensor fusion array for each car and Check if any car in same lane is too close to our car (within 30m infront). If so check if a lane change is possible. 
To check if a car is infront I first check if the car is in the same lane as ours by checking if the car is within the corresponding range of d values for the given lane. If so then I check if a change lane left operation is safe by first checking whether a lane exists on the left in the first place and if it does I use the `isCarWithinBuffer()` function to check if there is any car within the given buffer area of +/- 15m in the given lane. If it is safe I set the `lane` variable to the left lane. Otherwise I check if the change lane right operation is possible in a similar way. If neither is possible I alert the car to slow down(at a rate of 7m/s) and keep the current lane so as to avoid collision. Finally if no car is in proximity of current lane and the current velocity of our car is less than 49.5mph then I increase the speed of the car so that it reaches the max lane speed allowed. Based on the four states above that is Keep Lane and speed up, Keep lane and slow down, Change Lane Left and Change Lane Right I generate the trajectory for the car to follow. For this we take the starting state of the car (x,y and heading yaw) and generate 5 waypoints. The first two waypoints are the previous and current state. This is either taken as last two points from the prevous trajectory list if more than 2 points are available or the current point and based on the current point a prevoius point is estimated using model equations. The final 3 points are future predictions/estimations made at a distance of 30m, 60m and 90m from current state These waypoints also take into account the shift in lane if any. Then I iterate through the waypoints points and convert them from map coordinates to car coordinates. Finally the five waypoints are used to calculate the spline that passes through every waypoint in the waypoint list. The spline ensures a smooth and minimal jerk path for the car to travel. The delta for the lane shift here is 30meters. 
Now I add all the remaining unused points from prevoius path to the new trajectory list. The remaining points that are required is generated from the spline. For this first I get the corresponding y value of the point 30m ahead of current state(ie last s value from previous points) using the generated spline. Now I use this as target point and calculate the target distance from car to the calculated target x,y point. From this using the reference velocity and update rate(0.02s) I find the total number of divisions N to be made in the predicted trajectory to ensure a smooth and jerk free ride. Using this I find the x value for the next step and then use spline to generate the corresponding y value. Finally I convert the predicted points from vehicle coordintes back to map coordinates and add them to the new trajectory list. I repeat this process untill there are a total of 50 points for the new trajectory.
