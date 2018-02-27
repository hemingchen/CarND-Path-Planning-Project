#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "car.h"
#include "constants.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if (angle > pi() / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double>
getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};

}

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
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

  // Ego car lane, started from lane=1, the middle lane
  int ego_car_lane = 1;

  // Ego car speed
  double ego_car_ref_spd = 0.0; // m/s

  h.onMessage(
      [&ego_car_ref_spd, &ego_car_lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy]
          (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
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

              // Sensor Fusion Data, a list of all other cars on the same side of the road.
              auto sensor_fusion = j[1]["sensor_fusion"];


              /*****************************************************************************
               *  Path planning start
               ****************************************************************************/
              cout << "" << endl;
              cout << "ego car status: " << "x=" << car_x << ", y=" << car_y << ", s=" << car_s << ", d=" << car_d
                   << ", yaw=" << car_yaw << ", spd=" << car_speed << endl;

              int prev_path_size = previous_path_x.size();

              // Start new path from the end of the previous cycle path, although the ego car itself
              // may not be at the end of the previous path yet
              double calibr_ego_car_s = car_s;
              if (prev_path_size > 0) {
                calibr_ego_car_s = end_path_s;
              }


              /*****************************************************************************
               *  Analyze surrounding cars' locations
               ****************************************************************************/
              bool close_to_srrndng_car_ahead = false;
              bool close_to_srrndng_car_lt = false;
              bool close_to_srrndng_car_rt = false;

              for (auto const &sf : sensor_fusion) {
                // Surrounding car
                Car srrndng_car = Car((int) sf[0], sf[1], sf[2], sf[3], sf[4], sf[5], sf[6]);
                int srrndng_car_lane = srrndng_car.lane();
                double srrndng_car_spd = srrndng_car.speed();
                double calibr_srrndng_car_s = srrndng_car.s_in_t(prev_path_size * SYSTEM_SAMPLING_DT);

                // Skip if the surrounding car is in the opposite direction
                if (srrndng_car_lane < 0) {
                  continue;
                }

                // Determine if any surrounding car is close to ego car at the end of previous path
                if (srrndng_car_lane == ego_car_lane) {
                  // Check if surrounding car ahead in the same lane is close to ego car
                  close_to_srrndng_car_ahead |=
                      calibr_srrndng_car_s > calibr_ego_car_s &&
                      calibr_srrndng_car_s - calibr_ego_car_s < CAR_SEPARATION;
                } else if (srrndng_car_lane - ego_car_lane == -1) {
                  // Check if surrounding car on the left lane is close to ego car
                  close_to_srrndng_car_lt |=
                      calibr_ego_car_s - CAR_SEPARATION < calibr_srrndng_car_s &&
                      calibr_ego_car_s + CAR_SEPARATION > calibr_srrndng_car_s;
                } else if (srrndng_car_lane - ego_car_lane == 1) {
                  // Check if surrounding car on the right lane is close to ego car
                  close_to_srrndng_car_rt |=
                      calibr_ego_car_s - CAR_SEPARATION < calibr_srrndng_car_s &&
                      calibr_ego_car_s + CAR_SEPARATION > calibr_srrndng_car_s;
                }
              }


              /*****************************************************************************
               *  Determine path planning strategy
               ****************************************************************************/
              double spd_incr = 0;
              if (close_to_srrndng_car_ahead) {
                // If there is a surrounding car ahead close to ego car
                if (!close_to_srrndng_car_lt && ego_car_lane > 0) {
                  // If there is a lane to the left of ego car and no surrounding car close, then change to left lane
                  ego_car_lane--;
                } else if (!close_to_srrndng_car_rt && ego_car_lane != 2) {
                  // If there is a lane to the right of ego car and no surrounding car close, then change to right lane
                  ego_car_lane++;
                } else {
                  // Unable to change lane, so slow down in current lane
                  spd_incr -= MAX_ACC;
                }
              } else {
                // If there is no surrounding car ahead close to ego car
                if (ego_car_lane != 1) {
                  // If the ego car is not in the center lane, drive back to center lane if possible
                  if ((ego_car_lane == 0 && !close_to_srrndng_car_rt) ||
                      (ego_car_lane == 2 && !close_to_srrndng_car_lt)) {
                    ego_car_lane = 1;
                  }
                }
                if (ego_car_ref_spd < MAX_SPEED) {
                  // Ensure ego car does not exceed speed limit
                  spd_incr += MAX_ACC;
                }
              }

              cout << "selected strategy: " << "spd_incr=" << spd_incr << ", target_lane=" << ego_car_lane << endl;


              /*****************************************************************************
               *  Generate trajectory curve
               ****************************************************************************/
              vector<double> curve_x;
              vector<double> curve_y;

              double ego_car_ref_x = car_x;
              double ego_car_ref_y = car_y;
              double ego_car_ref_yaw = deg2rad(car_yaw);
              double ego_car_ref_d = LANE_WIDTH / 2. + LANE_WIDTH * ego_car_lane;

              // Use last USE_N_PREV_WPS points if exist to generate trajectory curve
              if (prev_path_size < USE_N_PREV_WPS) {
                // Not enough previous points, use current location as starting point
                curve_x.push_back(ego_car_ref_x);
                curve_y.push_back(ego_car_ref_y);
              } else {
                // Enough previous points, use last USE_N_PREV_WPS
                for (int i = 0; i < USE_N_PREV_WPS; i++) {
                  curve_x.push_back(previous_path_x[prev_path_size - USE_N_PREV_WPS + i]);
                  curve_y.push_back(previous_path_y[prev_path_size - USE_N_PREV_WPS + i]);
                }

                // Since previous way points are used, reset ego car x, y to the second one among selected
                // previous way point (so we can use the first previous way point to recalculate the yaw angle)
                double prev_path_selected_pt_x_1st = previous_path_x[prev_path_size - USE_N_PREV_WPS];
                double prev_path_selected_pt_y_1st = previous_path_y[prev_path_size - USE_N_PREV_WPS];

                double prev_path_selected_pt_x_2nd = previous_path_x[prev_path_size - USE_N_PREV_WPS + 1];
                double prev_path_selected_pt_y_2nd = previous_path_y[prev_path_size - USE_N_PREV_WPS + 1];

                double d_x = prev_path_selected_pt_x_2nd - prev_path_selected_pt_x_1st;
                double d_y = prev_path_selected_pt_y_2nd - prev_path_selected_pt_y_1st;

                ego_car_ref_x = prev_path_selected_pt_x_2nd;
                ego_car_ref_y = prev_path_selected_pt_y_2nd;
                ego_car_ref_yaw = atan2(d_y, d_x);
              }

              // Generate 3 additional way points down the road for trajectory curve
              vector<double> next_wp0 = getXY(
                  calibr_ego_car_s + 1 * PREDICT_HORIZON_IN_DIST, ego_car_ref_d,
                  map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> next_wp1 = getXY(
                  calibr_ego_car_s + 2 * PREDICT_HORIZON_IN_DIST, ego_car_ref_d,
                  map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> next_wp2 = getXY(
                  calibr_ego_car_s + 3 * PREDICT_HORIZON_IN_DIST, ego_car_ref_d,
                  map_waypoints_s, map_waypoints_x, map_waypoints_y);

              curve_x.push_back(next_wp0[0]);
              curve_x.push_back(next_wp1[0]);
              curve_x.push_back(next_wp2[0]);

              curve_y.push_back(next_wp0[1]);
              curve_y.push_back(next_wp1[1]);
              curve_y.push_back(next_wp2[1]);

              // Translate coordinates to car coordinate system
              for (int i = 0; i < curve_x.size(); i++) {
                double normalized_x = curve_x[i] - ego_car_ref_x;
                double normalized_y = curve_y[i] - ego_car_ref_y;

                curve_x[i] =
                    normalized_x * cos(0 - ego_car_ref_yaw) - normalized_y * sin(0 - ego_car_ref_yaw);
                curve_y[i] =
                    normalized_x * sin(0 - ego_car_ref_yaw) + normalized_y * cos(0 - ego_car_ref_yaw);
              }

              // Create the spline.
              tk::spline traj_curve;
              traj_curve.set_points(curve_x, curve_y);


              /*****************************************************************************
               *  Generate car path
               ****************************************************************************/
              vector<double> ego_car_path_x;
              vector<double> ego_car_path_y;

              // Firstly, add ALL way points from previous path for continuity
              for (int i = 0; i < prev_path_size; i++) {
                ego_car_path_x.push_back(previous_path_x[i]);
                ego_car_path_y.push_back(previous_path_y[i]);
              }

              // Secondly, generate new path way points from the ego car until a distance of PREDICT_HORIZON_IN_DIST
              double pred_x_end = PREDICT_HORIZON_IN_DIST;
              double pred_y_end = traj_curve(pred_x_end);
              double pred_dist = sqrt(pred_x_end * pred_x_end + pred_y_end * pred_y_end);

              double prev_pred_x = 0;

              // Generate GEN_N_WPS way points with travelling time of SYSTEM_SAMPLING_DT between two successive points
              for (int i = 1; i < GEN_N_WPS - prev_path_size; i++) {
                // Set target speed
                ego_car_ref_spd += spd_incr;
                if (ego_car_ref_spd > MAX_SPEED) {
                  ego_car_ref_spd = MAX_SPEED;
                } else if (ego_car_ref_spd < MAX_ACC) {
                  ego_car_ref_spd = MAX_ACC;
                }

                // Generate N way points based on target speed near that way point
                double pred_x = prev_pred_x + (SYSTEM_SAMPLING_DT * ego_car_ref_spd);
                double pred_y = traj_curve(pred_x);

                prev_pred_x = pred_x;

                // Translate x, y coordinates back to global frame
                double x_coord_global_frame =
                    pred_x * cos(ego_car_ref_yaw) - pred_y * sin(ego_car_ref_yaw) + ego_car_ref_x;
                double y_coord_global_frame =
                    pred_x * sin(ego_car_ref_yaw) + pred_y * cos(ego_car_ref_yaw) + ego_car_ref_y;

                // Store the global x, y coordinates
                ego_car_path_x.push_back(x_coord_global_frame);
                ego_car_path_y.push_back(y_coord_global_frame);
              }

              cout << "generated path: ego_car_path_x range=(" << ego_car_path_x[0] << "," << ego_car_path_x[1]
                   << ") ego_car_path_y range=(" << ego_car_path_y[0] << "," << ego_car_path_y[1] << ")" << endl;


              /*****************************************************************************
               *  Path planning end
               ****************************************************************************/


              json msgJson;

              msgJson["next_x"] = ego_car_path_x;
              msgJson["next_y"] = ego_car_path_y;

              auto msg = "42[\"control\"," + msgJson.dump() + "]";

              //this_thread::sleep_for(chrono::milliseconds(1000));
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            }
          } else {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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