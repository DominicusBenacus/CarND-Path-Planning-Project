// meine
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// from andrew
// Returns lane number -1, 0, 1, 2 for specified d coord. in Frenet frame.
int DtoLaneNumber(double d) {
  if (d > 0 || d < -12)
    return -1; // outside of right-driving road.
  if (d < -8)
    return 0;
  else if (d < -4)
    return 1;
  return 2;
}

// Returns D - center of Lane for specified Lane number
double LaneNumberToD(int nLaneNumber) {
  constexpr std::array<double, 3> laneCenter = {-9.75, -6.0, -2.0};
  assert(nLaneNumber >= 0 && nLaneNumber < 3);
  return laneCenter[nLaneNumber];
}

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
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {

  double closestLen = 100000; // large number
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {

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
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
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

  // see if d value is positive or negative by comparing it to a center point

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
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading =
      atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}
// Get the Lane number based on Frenet coordinates
int getLaneFrenet(double d) {
  int lane_width = 4;
  return int(floor(d / lane_width));
}

struct object {
  int car_id;
  double distanceToEgo;
  double x_map;
  double y_map;
  double vx;
  double vy;
  double speed;
  double frenet_d;
  double frenet_s;

  object(int car_id = 0, double distanceToEgo = 0, double x_map = 0,
         double y_map = 0, double vx = 0, double vy = 0, double speed = 0,
         double frenet_d = 0, double frenet_s = 0)
      : car_id(car_id), distanceToEgo(distanceToEgo), x_map(x_map),
        y_map(y_map), vx(vx), vy(vy), speed(speed), frenet_d(frenet_d),
        frenet_s(frenet_s) {}
} closestObject;

// Get ID's of cars in a given lane
vector<int> getLaneCars(int lane, json sensor_fusion) {
  vector<int> cars_ids;

  for (int i = 0; i < sensor_fusion.size(); ++i) {
    float some_d = sensor_fusion[i][6];
    int some_lane = getLaneFrenet(some_d);

    // Check for unbounded data
    if (some_lane < 0 || some_lane > 2) {
      continue;
    }
    // Collect vehicles in the entered lane
    if (some_lane == lane) {
      cars_ids.push_back(i);
    }
  }
  return cars_ids;
}

// limit accelerartion
double limitAcceleration(double axEgo, const double axLimitPositiv,
                         const double axLimitNegativ) {
  if (axEgo > axLimitPositiv) {
    axEgo = axLimitPositiv;

  } else if (axEgo < axLimitNegativ) {
    axEgo = axLimitNegativ;
  }
  return axEgo;
}

// Calculate closest distance
object getClosestDistanceOfEnteredCarIdsPerLaneInFront(vector<int> cars_ids,
                                                       json sensor_fusion,
                                                       double check_dist,
                                                       double car_s,
                                                       object closestObject) {

  closestObject.distanceToEgo = 100000;
  for (int car_id : cars_ids) {
    closestObject.vx = sensor_fusion[car_id][3];
    closestObject.vy = sensor_fusion[car_id][4];
    closestObject.speed = sqrt(closestObject.vx * closestObject.vx +
                               closestObject.vy * closestObject.vy);
    closestObject.frenet_s = sensor_fusion[car_id][5];
    closestObject.frenet_d = sensor_fusion[car_id][6];

    double check_end_car_s =
        closestObject.frenet_s + check_dist * closestObject.speed;

    double dist_start = fabs(closestObject.frenet_s - car_s);
    if (dist_start < closestObject.distanceToEgo) {
      closestObject.distanceToEgo = dist_start;
      closestObject.car_id = car_id;
      closestObject.x_map = sensor_fusion[car_id][1];
      closestObject.y_map = sensor_fusion[car_id][2];
      closestObject.vx = sensor_fusion[car_id][3];
      closestObject.vy = sensor_fusion[car_id][4];
      closestObject.speed = sqrt(closestObject.vx * closestObject.vx +
                                 closestObject.vy * closestObject.vy);
      closestObject.frenet_s = sensor_fusion[car_id][5];
      closestObject.frenet_d = sensor_fusion[car_id][6];
    }

    double dist_end = fabs(check_end_car_s - car_s);
    if (dist_end < closestObject.distanceToEgo) {
      closestObject.distanceToEgo = dist_end;
      closestObject.car_id = car_id;
      closestObject.x_map = sensor_fusion[car_id][1];
      closestObject.y_map = sensor_fusion[car_id][2];
      closestObject.vx = sensor_fusion[car_id][3];
      closestObject.vy = sensor_fusion[car_id][4];
      closestObject.speed = sqrt(closestObject.vx * closestObject.vx +
                                 closestObject.vy * closestObject.vy);
      closestObject.frenet_s = sensor_fusion[car_id][5];
      closestObject.frenet_d = sensor_fusion[car_id][6];
    }
  }
  return closestObject;
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

  // start in lane one
  int lane = 1;
  // Have a reference velocity to target as close to the spees limit as possible
  double ref_velocity = 0; // [mph]
  const double setSpeed = 49.5 * 0.44704;

  h.onMessage([&setSpeed, &ref_velocity, &map_waypoints_x, &map_waypoints_y,
               &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,
               &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message
    // event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
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
          const double norm = 0.44704;
          car_speed = car_speed * norm;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          // of
          // the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          // A previous list of points which really helps doing a transition
          // The simulator actually telling one which this previous path was
          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          // get IDs of ego lane
          vector<int> cars_ids = getLaneCars(lane, sensor_fusion);
          // how many lane cars are available
          int numberOfCarIds = sizeof(cars_ids);

          // initialize standart values
          double axEgo = 0.0;
          double tEgo = 0.0;
          double tauGap = 0.9;
          const double distanceOfInterest = 20.0;
          const double axLimitPositiv = 3.5;
          const double axLimitNegativ = -4.5;
          const double tauGapSetSpeed = 4.5;
          const double fixDesiredDistance = 10.0;

          // get distance of closest IDs
          object closestObject;
          closestObject = getClosestDistanceOfEnteredCarIdsPerLaneInFront(
              cars_ids, sensor_fusion, 0.02 * prev_size, car_s, closestObject);

          // Print out some information
          cout << "============ OBJECT DATA =============" << endl;
          cout << "closest object ID = " << closestObject.car_id << endl;
          cout << "closest object Distance = " << closestObject.distanceToEgo
               << endl;
          cout << "closest object Velocity = " << closestObject.speed << endl;
          cout << "Ego Vehicle Velocity = " << car_speed << endl;

          if (closestObject.distanceToEgo <= distanceOfInterest) {

            double desiredDistance =
                closestObject.distanceToEgo - (car_speed * tauGap);

            if (desiredDistance <= 5.0) {
              desiredDistance = fixDesiredDistance;
            }

            // double desiredDistance = fixDesiredDistance;
            cout << "calculated desired distance = " << desiredDistance << endl;

            cout << "============ CALC AX =============" << endl;
            if ((closestObject.speed - car_speed) >= double(0.0)) {

              tEgo = 1.25;
              axEgo = (closestObject.speed - car_speed) / tEgo;
              axEgo = limitAcceleration(axEgo, axLimitPositiv, axLimitNegativ);
              cout << "object is faster - axEgo = " << axEgo << endl;
              // limit acceleration

            } else {

              tEgo = 2.0F * desiredDistance / (car_speed - closestObject.speed);
              axEgo = (closestObject.speed - car_speed) / tEgo;
              axEgo = limitAcceleration(axEgo, axLimitPositiv, axLimitNegativ);
              cout << "ego is faster - axEgo = " << axEgo << endl;
            }
            if (closestObject.distanceToEgo <= double(6.0)) {
              axEgo = -3.0;
            }

          } else {
            axEgo = (setSpeed - car_speed) / tauGapSetSpeed;
            axEgo = limitAcceleration(axEgo, axLimitPositiv, axLimitNegativ);
          }
          cout << "ax calculated after if else = " << axEgo << endl;
          // Adapt reference velocity to avoid crashes based on the
          // calculations
          // calc increment value for increase or descrease the reference
          // velocity
          cout << "============ Calc Velocity=============" << endl;
          double deltaVelocity = axEgo * 0.0448;
          cout << "calculated delta velocity = " << deltaVelocity << endl;
          ref_velocity += deltaVelocity;
          cout << "calculated ref velocity = " << ref_velocity << endl;

          // Create a list of widely spaced (x,y) waypoints, evenly spaced
          // on
          // 30m. Later we will interpolate these waypoints with a spline
          // and
          // fill it in which more points that control speed
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y and yaw states
          // either we will reference the startig point as
          // where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as a starting
          // point
          if (prev_size < 2) {
            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous paths end point as starting reference
          else {
            // Redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // use two points that make the path tangent to the previous
            // path
            // end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet Coordinates add evenly 30m spaced points ahead oth
          // the
          // starting reference [s,d,maps,mapx,mapy]
          // 2+4*lane = 6 if lane is 1
          vector<double> next_wp0 =
              getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 =
              getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 =
              getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++) {
            // shift car reference angle to 0 degree
            // doing a transformation to this local car's coordinates
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            // Makes sure that the last point (x,y) of the previous
            // path is at the origin of the car and its angle is zero degree
            // makes the math easier -- it is a shift in rotation
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // create a spline
          tk::spline s;

          // set (x,y) the created Frenet waypoints to the spline object for
          // the
          // path planning
          s.set_points(ptsx, ptsy);

          // define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up spline points so that we travel
          // at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist =
              sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0.0;

          // fill up the rest of our path planner after filling
          // it with previous points, here we will always output
          // previous_path_x is not the full previous path.
          // It is the previous path minus the already passed by car points
          // E.g. planned 50 points and the car passed 10 = previous path
          // would
          // be 40
          // So, here we will always output 50 points
          for (int i = 0; i <= 50 - previous_path_x.size(); i++) {
            double N = (target_dist /
                        (.02 * ref_velocity)); // 2.24 to convert into m/S
            double x_point = x_add_on + (target_x) / N;
            // returns the corresponding y value of the splie for given x
            // value
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal rotating it earlier --> to cartesian
            // coordinates
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;

          // // TODO: define a path made up of (x,y) points that the car
          // will
          // visit
          // // sequentially every .02 seconds
          // double dist_inc = 0.5;
          // for (int i = 0; i < 50; i++)
          // {

          //   double next_s = car_s + (i + 1) * dist_inc;
          //   double next_d = 6; // lateral position in meter

          //   vector<double> xy = getXY(next_s, next_d, map_waypoints_s,
          //                             map_waypoints_x, map_waypoints_y);
          //   next_x_vals.push_back(xy[0]);
          //   next_y_vals.push_back(xy[1]);
          //   // next_x_vals.push_back(car_x +
          //   //                       (dist_inc * i) *
          //   //                       cos(deg2rad(car_yaw)));
          //   // next_y_vals.push_back(car_y +
          //   //                       (dist_inc * i) *
          //   //                       sin(deg2rad(car_yaw)));
          // }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
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
  // program
  // doesn't compile :-(
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
