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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  int lane = 1; // start lane is 1 (middle lane)
  double ref_vel = 0;
  int change_in_progress = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &ref_vel, &lane, &change_in_progress, 
               &map_waypoints_dx,&map_waypoints_dy]
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
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          /**
           * TODO: check all other cars on the same side of the road and
           *   find out the best lane and speed to drive
           */

          int prev_size = previous_path_x.size();

          // if we have previous path, change car_s to previous path's end point
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }

          double front_car_s[3] = {100, 100, 100};
          double back_car_s[3] = {100, 100, 100};
          double front_car_speed[3] = {0, 0, 0};
          double back_car_speed[3] = {0, 0, 0};

          // find closest car in each lane
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];

            for(int j = 0; j < 3; j++) {
              if(d < (2 + 4*j + 2) && d > (2 + 4*j - 2) )
              {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];

                // project s value of that car outward in time from the perspective of previous path
                check_car_s += ((double)prev_size*.02*check_speed);

                // check if car is ahead of us
                if(check_car_s > car_s)
                {
                  // save distance of closest car ahead
                  if((check_car_s-car_s) < front_car_s[j])
                  {
                    front_car_s[j] = check_car_s-car_s;
                    front_car_speed[j] = check_speed*2.237;
                  }
                }
                else
                {
                  // save distance of closest car behind
                  if((car_s-check_car_s) < back_car_s[j])
                  {
                    back_car_s[j] = car_s-check_car_s;
                    back_car_speed[j] = check_speed*2.237;
                  }
                }
              }
            }
          }


          // check if there is a car ahead and find the lane to switch to
          int orig_lane = lane;
          switch(lane)
          {
            case 0:
              // if there is more space in lane 1 or lane 2, shift to lane 1
              if(front_car_s[1] > front_car_s[0] || front_car_s[2] > front_car_s[0])
              {
                // check front and back car in lane 1 to see if we enough room to shift
                if(front_car_s[1] > 30 && back_car_s[1] > 10)
                {
                  lane = 1;
                }
              }
              break;

            case 1:
              // check if there is more space in lane 0 or lane 2
              if(front_car_s[0] > front_car_s[1] || front_car_s[2] > front_car_s[1])
              {
                // if lane 0 has more space than lane 2
                if(front_car_s[0] >= front_car_s[2] && front_car_s[0] > 30 && back_car_s[0] > 10)
                {
                  lane = 0;
                }
                else if(front_car_s[2] > front_car_s[1] && front_car_s[2] > 30 && back_car_s[2] > 10)
                {
                  lane = 2;
                }
              }
              break;

            case 2:
              // if there is more space in lane 0 or lane 1, shift to lane 1
              if(front_car_s[0] > front_car_s[2] || front_car_s[1] > front_car_s[2])
              {
                // check front and back car in lane 1 to see if we enough room to shift
                if(front_car_s[1] > 30 && back_car_s[1] > 10)
                {
                  lane = 1;
                }
              }
              break;
          }


          if(lane != orig_lane)
          {
            //if(back_car_s[lane] < 20 && back_car_speed[lane] > ref_vel)
            //{
            //  // don't change the lane
            //  lane = orig_lane;
            //}

            // don't change the lane if lane change is already in progress
            if(change_in_progress != 0)
            {
              lane = orig_lane;
            }
            else
            {
              change_in_progress = 30;
            }
          }

          if(change_in_progress)
          {
            change_in_progress--;
          }


          // calculate the target velocity
          if(front_car_s[lane] < 20 && ref_vel > front_car_speed[lane])
          {
            ref_vel -= 1.792;
          }
          else if(front_car_s[lane] < 30 && ref_vel > front_car_speed[lane])
          {
            ref_vel -= .896;
          }
          else if(front_car_s[lane] < 40 && ref_vel > front_car_speed[lane])
          {
            ref_vel -= .448;
          }
          else if(ref_vel < 49.5)
          {
            ref_vel += .448;
          }





          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // widely spaced waypoints to interoplate with spline
          vector<double> ptsx;
          vector<double> ptsy;

          // reference state - could be car's current position or previous path's end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // create path tanget to the angle of the car
          if(prev_size < 2)
          {
            // if no points in previous state, use car's current state as reference
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            // use two points that make path tangent to car
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            // redefine reference state to previous path's end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

            // use two points that make path tangent to previous path's end point
            ptsx.push_back(previous_path_x[prev_size-2]);
            ptsx.push_back(previous_path_x[prev_size-1]);

            ptsy.push_back(previous_path_y[prev_size-2]);
            ptsy.push_back(previous_path_y[prev_size-1]);
          }

          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // transformation to car's local coordinates - shift car reference angle to 0 degrees
          for (int i = 0; i < ptsx.size(); i++ )
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }

          // create a spline
          tk::spline s;

          // set above calculated widely spaced x,y anchor points to spline
          s.set_points(ptsx,ptsy);

          /* start building list of actual points to use with the planner by adding previous path's points and new calculated points */

          // first add all previous path's points for continuity
          for(int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // break up spline points so that car travel at desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          // add rest of the points for the planner
          for(int i = 1; i <= 50-previous_path_x.size(); i++)
          {
            double N = (target_dist/(.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // transform car's local coordinates back to map coordinates
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

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
