#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <algorithm> // std::min_element

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  int current_lane = 1; // Initial lane (0: left lane, 1: center lane, 2: right lane)
  double ref_vel = 0; // Initial reference speed (mph)
  
  h.onMessage([&ref_vel, &current_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s, &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = (j[1]["speed"]);

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
		   
		   // ****** Drive tuning parameters ****** //
		   
		   int prev_size = previous_path_x.size();
		   double time_step = 0.02; // Code and sim execution rate (sec)
		   double max_speed = 49.5; // Speed limit (mph)
		   double ref_accel = 5; // Max acceleration/deceleration (m/s^2)
		   double max_delta_speed = ref_accel*time_step*2.24; // delta_v = ref_accel*time_step converted to mph
		   double lane_width = 4.0; // Lane width (m)
		   double horizon = 100.0; // Range over which to evaluate lane costs (m)
		   
		   
		   
		   
		   
		   // ****** Behavioral planning ****** //
		   
		   if (prev_size > 0)
		   {
			   car_s = end_path_s;
		   }
		   bool too_close = false;
		   int target_lane = current_lane;
		   double max_lane_speed = max_speed; // Used as speed limit when following a slower car (mph)
		   
		   // Find lane with least cost based on traffic conditions
		   for (int i=0; i<sensor_fusion.size(); i++)
		   {
			   // Check if another car is in the current lane
			   float d = sensor_fusion[i][6];
			   if (d > lane_width*current_lane && d < lane_width*(current_lane+1))
			   {
				   double vx = sensor_fusion[i][3];
				   double vy = sensor_fusion[i][4];
				   double next_car_speed = sqrt(pow(vx,2)+pow(vy,2));
				   double next_car_s = sensor_fusion[i][5];
				   next_car_s += ((double)prev_size*time_step*next_car_speed);
				   double stopping_s = pow(ref_vel/2.24,2)/(2*ref_accel); // Motion equation
				   
				   // Check if next car is within stopping distance
				   if (next_car_s > car_s && (next_car_s-car_s) < stopping_s)
					{
						double left_lane_cost;
						double right_lane_cost;
						double current_lane_cost = laneCost(current_lane, sensor_fusion, car_s, ref_vel, horizon);
						double best_lane_cost = current_lane_cost;
						int best_lane = current_lane;						
						
						if (current_lane > 0) // Check lane on the left
						{
							bool left_lane_open = laneCheck(current_lane-1, sensor_fusion, car_s, ref_vel);
							left_lane_cost = laneCost(current_lane-1, sensor_fusion, car_s, ref_vel, horizon);
							if (left_lane_open == true && left_lane_cost < current_lane_cost)
							{
								best_lane = current_lane-1; // Switch (provisionally) into left lane
								best_lane_cost = left_lane_cost;
							}
						}
						if (current_lane < 2) // Check lane on the right
						{
							bool right_lane_open = laneCheck(current_lane+1, sensor_fusion, car_s, ref_vel);
							right_lane_cost = laneCost(current_lane+1, sensor_fusion, car_s, ref_vel, horizon);
							if (right_lane_open == true && right_lane_cost < best_lane_cost)
							{
								best_lane = current_lane+1; // Switch into right lane
							}
							// If prior to checking the right lane, the best lane was the left lane, the car would switch into the right lane if its cost were strictly lower than the left lane cost, thus inherently preferring the legally faster left lane in case of a tie
						}
						if (best_lane == current_lane)
						{
							too_close = true; // Slow down
							max_lane_speed = next_car_speed;
						}
						else
						{
							current_lane = best_lane; // Reassign current lane going forward
						}
					}
			   }
		   }
		   		   
		   
		   
		   
		   
		   // ****** Trajectory Generation ****** //
		   
		   vector<double> ptsx;
		   vector<double> ptsy;
		   
		   // reference x, y, yaw states
		   // Reference is either where the car is now or the previous path's end point
		   double ref_x = car_x;
		   double ref_y = car_y;
		   double ref_yaw = car_yaw;
		   double ref_x_prev;
		   double ref_y_prev;
		   
		   // If previous path size is almost zero, draw a unit tangent at the current car position and determine one point in the past
		   if (prev_size < 2)
		   {
			   ref_x_prev = ref_x - cos(car_yaw);
			   ref_y_prev = ref_y - sin(car_yaw);
		   }
		   // Else use the last 2 points from the previous path
		   else
		   {
			   ref_x = previous_path_x[prev_size-1];
			   ref_y = previous_path_y[prev_size-1];
			   ref_x_prev = previous_path_x[prev_size-2];
			   ref_y_prev = previous_path_y[prev_size-2];
			   ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
		   }
		   ptsx.push_back(ref_x_prev);
		   ptsx.push_back(ref_x);
		   ptsy.push_back(ref_y_prev);
		   ptsy.push_back(ref_y);
		   
		   // Add evenly spaced points ahead of the starting reference
		   vector<double> next_wp0 = getXY(car_s+30, (2+lane_width*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		   vector<double> next_wp1 = getXY(car_s+60, (2+lane_width*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		   vector<double> next_wp2 = getXY(car_s+90, (2+lane_width*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		   
		   ptsx.push_back(next_wp0[0]);
		   ptsx.push_back(next_wp1[0]);
		   ptsx.push_back(next_wp2[0]);
		   ptsy.push_back(next_wp0[1]);
		   ptsy.push_back(next_wp1[1]);
		   ptsy.push_back(next_wp2[1]);
		   
		   // Shift all new trajectory points to the car reference frame prior to curve fitting to a smooth trajectory
		   for (int i=0; i<ptsx.size(); i++)
		   {
			   double shift_x = ptsx[i]-ref_x;
			   double shift_y = ptsy[i]-ref_y;
			   ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
			   ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
		   }
		   
		   // Create a spline
		   tk::spline s;
		   
		   // Set (x,y) points to the spline
		   s.set_points(ptsx,ptsy);
		   		   
		   // Add any available points from previous path at the start of the new path for continuity
		   for (int i=0; i<previous_path_x.size(); i++)
		   {
			   next_x_vals.push_back(previous_path_x[i]);
			   next_y_vals.push_back(previous_path_y[i]);
		   }
		   
		   // Fill up remaining trajectory points fit to spline
		   int total_path_size = 50; // Number of points
		   
		   // Calculate distance y position on 30 m ahead
           double target_x = 30.0;
           double target_y = s(target_x);
           double target_dist = sqrt(target_x*target_x + target_y*target_y);
		   double x_add_on = 0.0;
		   
		   for (int i=1; i<=total_path_size-previous_path_x.size(); i++)
		   {
			   // Accelerate or decelerate
			   double delta_speed = 0.0;
			   if (too_close == false && ref_vel < max_speed)
			   {
				   delta_speed += max_delta_speed;
			   }
			   else if (too_close == true && ref_vel > max_lane_speed)
			   {
				   delta_speed -= max_delta_speed;
			   }
			   ref_vel += delta_speed;
			   double N = target_dist/(time_step*ref_vel/2.24); // Split target x distance into equidistant points
			   double x_point = x_add_on + target_x/N;
			   double y_point = s(x_point);
			   x_add_on = x_point;

			   // Transform back to global reference frame
			   double x_ref = x_point;
			   double y_ref = y_point;
			   x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
			   y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
			   x_point += ref_x;
			   y_point += ref_y;
			   
			   next_x_vals.push_back(x_point);
			   next_y_vals.push_back(y_point);
		   }
		   
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}