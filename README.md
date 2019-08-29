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

The simulator expects a list of (x,y) coordinates to use as a reference path in controlling the car. The car's speed is determined by the spacing between these points. Since the time interval between calculating a new set of points is the simulator frequency (0.02 sec), every new point on the list was set to be 0.02 sec into the future after the latest available point. Then the distance between successive points was calculated backwards from desired acceleration.

Maximum car speed was set at 49.5 mph ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) line 110) so as to stay within the speed limit. Maximum acceleration/deceleration was set to a conservative 5 m/s^2. This value was converted into a ***delta speed*** value between successive points ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 111-112). This ***delta speed*** added (or subtracted under braking) to a reference velocity ***ref_vel*** for the car determined its target speed for each successive point on the list. With speed and time known, distances between these points were calculated iteratively over a fixed horizon ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 247-276).

**2) Lane following:**

Lines 180-291 in [main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) contain all steps taken to generate and deliver a list of (x,y) points to the simulator. It began with selecting a starting point, the car's present position. The simulator returns a list of unused points from the previous path (***previous_path_x*** and ***previous_path_y***). The last two points on that list were used to start calculating the new trajectory. When this list was too short, as would happen when the car has just begun moving, the car's present position and orientation was used to calculate tangential points which served the same purpose. Next, evenly spaced points along the highway were added to this list and it was transformed into the car frame of reference to form a rudimentary trajectory.

A helpful resource suggested by the course instructors for creating smooth trajectories was the [Cubic Spline interpolation in C++](http://kluge.in-chemnitz.de/opensource/spline/) method. The header file [spline.h](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/spline.h) was borrowed from here which helped generate a spline function which fit the trajectory points list ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 234-238).

Generating new points repeatedly opens the risk of discontinuous motion. Hence unused points from the previous path were reused by adding them to the start of the new list of points ***next_x_vals*** and ***next_y_vals*** ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 240-245). X-coordinates for new points calculated using the motion model were fed into the spline function to generate corresponding Y-coordinates. Finally, these points were transformed back to the global coordinate system and appended to the list delivered back to the simulator ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 278-291).

**3) Behavioral planning:**

Initially a basic decision tree approach was taken based on relative distances to cars in any available lane. In cass when both lanes were available, the legally faster left lane was preferred. But in simulator runs, the car found itself going into a left lane where the next car turned out to be slower and before it could revert back to its original lane, cars behind came too close and blocked its trajectory. Thus the decision to be in any lane at any given time depended on the speed as well as relative distance to other cars.

Two functions were added in [helpers.h](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/helpers.h):
- ***laneCheck*** (lines 157-176) determined whether or not a lane was open based on relative distance to the nearest cars behind and ahead in the lane being checked. A safe distance of 20 m was determined through multiple trials. Too small a value caused the car to collide with nearby cars occasionally during lane changes while too large a value prevented the car from making lane changes and gaining time around the track.
- ***laneCost*** (lines 178-221) determined the "cost" of being in a lane based on the speed as well as relative distance to other cars. A high cost would be incurred if either the car ahead or behind were too close, the car ahead were much slower or the car behind were much faster. The condition on speed of the car ahead helped avoid the lane with a slower car ahead in case multiple lanes were available. Similarly the condition on speed of the car behind helped avoid the lane with a much faster car approaching and creating a rear collision hazard after entering that lane. All costs were modeled as exponential equations approaching very large values at one end of the spectrum. Linear equations for cost were implemented first (line 218) but this approach caused conflict within the overall cost when one part had a high value while another had a low value and the net cost was reasonable even though that lane posed a safety hazard.

![delta_s_ahead](/reaadme_files/delta_s_ahead.jpg)
![delta_s_behind](/reaadme_files/delta_s_behind.jpg)
![v_ahead](/reaadme_files/v_ahead.jpg)
![v_behind](/reaadme_files/v_behind.jpg)

When the car detected another car in front ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 131-145), it calculated the cost of every available lane i.e. the current and adjacent lanes, using the ***laneCost*** function provided the ***laneCheck*** function determined that it was open, and arrived at the lane with the least cost ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 147-172). The order of checking the left lane (if it exists) before the right lane (if it exists) was deliberate. The right lane was selected only if its cost was strictly lower than that of the left lane, thus inherently preferring the legally faster left lane in case of a tie. If the best lane to be in was the current lane, a flag was raised to decelerate (used in the motion model section of [main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) in lines 270-277) and the target speed was lowered down to the speed of the next car. If the best lane to be in was another lane, the ***current_lane*** parameter was updated accordingly and the next waypoints used to calculate the spline function were automatically placed in that lane ([main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) lines 223-226).

A "stopping distance" as a function of car speed was calculated on [main.cpp](https://github.com/nvphadnis/CarND-Term3-P1-HighwayDriving/blob/master/src/main.cpp) line 142 and used as the entry condition to the entire decision making process. Such a dynamic distance threshold prevented the car from speeding up and slowing down while following another car as their relative distance varied around the threshold.

## Test runs

A few test runs were recorded to show that the car could complete the entire highway loop of 4.32 miles while meeting all [project rubrics](https://review.udacity.com/#!/rubrics/1971/view).

[Simulator Run 1](https://www.youtube.com/watch?v=shWLmyMpJW8&list=PLnSstEtb-9_2AmZft0w4D_ptlPWoqanyP&index=1)

[Simulator Run 2](https://www.youtube.com/watch?v=2oXoEIv_3DQ&list=PLnSstEtb-9_2AmZft0w4D_ptlPWoqanyP&index=2)

[Simulator Run 3](https://www.youtube.com/watch?v=BrLgJ2R3GTk&list=PLnSstEtb-9_2AmZft0w4D_ptlPWoqanyP&index=3)


Run 3 encountered three double lane changes of which one had too much jerk.

## Potential improvements

While the [project rubrics](https://review.udacity.com/#!/rubrics/1971/view) were achieved, a few areas for improvement were identified during development:
- Double lane changes in the real world could cause safety hazards or at the very least too muc jerk. One way to avoid these effects could be to create a lateral motion model which calculated successive ***d*** values with acceleration constraints similar to the longitudinal motion model implemented in this code. These could then be used to set waypoints for the spline function.
- Sudden braking when another car enters the current lane at the last minute is a corner case which would cause excessive jerk and even a rear collision. A predictive strategy based on sensor fusion data could give more time to react safely.
- More than 3 lanes would need a lot more conditional logic. If a list were created containing IDs for adjacent lanes, costs for each lane could be calculated and the lane with the least cost could be selected with much lesser lines of code.
