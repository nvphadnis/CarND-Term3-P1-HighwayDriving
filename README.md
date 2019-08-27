# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program: Term 3 Project 1

## Goals
The goal of this project was to safely navigate around a virtual highway with other traffic that was driving +-10 MPH of the 50 MPH speed limit. Information provided:
- Car's localization and sensor fusion data
- Traffic cars' localization and sensor fusion data
- Sparse map list of waypoints around the highway (highway_map.csv)
- A perfect path following controller operating every 0.02 sec within a simulator using (x,y) coordinates

The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Also the car should not experience total acceleration over 10 m/s^2 and jerk (rate of change of total acceleration) that is greater than 10 m/s^3.

All localization and sensor fusion data was provided both as (x,y) coordinates in the global map and as (s,d) values known as Frenet coordinates, where "s" was the distance along the highway from a starting location and "d" was the lateral distance from the dividing lane line. The waypoints were provided as Frenet coordinates as well.

## Approach

The problem was tackled one step at a time:

1) Motion model:        Traverse a straight trajectory without violating speed, acceleration and jerk limits
2) Lane following:      Generate a smooth trajectory to maintain a given lane
3) Behavioral planning: Monitor real time traffic conditions and reactions to safely navigate around the highway

**1) Motion model:**

The simulator expects a list of (x,y) coordinates to use as a reference path in controlling the car. The car's speed is determined by the spacing between these points. Since the time interval between calculating a new set of points is the simulator frequency (0.02 sec), every new point on the list was set to be 0.02 sec into the future after the latest available point. Now the distance between successive points was calculated backwards from desired acceleration.

Maximum car speed was set at 49.5 mph ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) line 110) so as to stay within the speed limit. Maximum acceleration/deceleration was set to a conservative 5 m/s^2. This value was converted into a ***delta speed*** value between successive points ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 111-112). This ***delta speed*** added (or subtracted under braking) to a reference velocity ***ref_vel*** for the car determined its target speed for each successive point on the list. With speed and time known, distances between these points was calculated iteratively over a fixed horizon ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 247-276).

**2) Lane following:**

Lines 180-291 in [main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) contain all steps taken to generate and deliver a list of (x,y) points to the simulator. It began with selecting a starting point, the car's present position. The simulator returns a list of unused points from the previous path (***previous_path_x*** and ***previous_path_y***). The last two points on that list were used to start calculating the new trajectory. When this list was too short, as would happen when the car has just begun moving, the car's present position and orientation was used to calculate tangential points which served the same purpose. Next, evenly spaced points along the highway were added to this list and it was transformed into the car frame of reference to form a rudimentary trajectory.

A helpful resource suggested by the course instructors for creating smooth trajectories was the Cubic Spline interpolation in C++(http://kluge.in-chemnitz.de/opensource/spline/) method. The header file [spline.h](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/spline.h) was borrowed from here which helped generate a spline function which fit the trajectory points list ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 234-238).

Generating new points repeatedly opens the risk of discontinuous motion. Hence unused points from the previous path were reused by adding them to the start of the new list of points ***next_x_vals*** and ***next_y_vals*** ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 240-245). X-coordinates for new points calculated using the motion model were fed into the spline function to generate corresponding Y-coordinates. Finally, these points were transformed back to the global coordinate system and appended to the list delivered back to the simulator ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 278-291).

**3) Behavioral planning:**





