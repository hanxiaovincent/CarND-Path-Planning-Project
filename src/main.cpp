#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/Geometry"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

const double speedLimit = 49 * 0.4474; // speed limit m/s.

// lane from left to right is 0 to 2, never go to the other side of the road.
static inline double laneTod(int line)
{
  return line * 4 + 2;
}

// create a finite state machine state.
struct State
{
  // state 0 == keep current lane
  // state 1 == pass
  int state;
  // the current lane of the state.
  int target_lane;
};

// car neighbor check
struct CarNeighbors
{
  double frontCarSpeed;
  double frontCarDist;
  double leftFrontCarSpeed;
  double leftFrontCarDist;
  double leftBackCarSpeed;
  double leftBackCarDist;
  double rightFrontCarSpeed;
  double rightFrontCarDist;
  double rightBackCarSpeed;
  double rightBackCarDist;
};

// check surrounding cars information from sensor fusion.
CarNeighbors checkNeighbors(int target_lane, double car_s, int prev_size,
  const std::vector<std::vector<double>>& sensor_fusion)
{
  CarNeighbors neighbors;
  neighbors.frontCarSpeed = 0;
  neighbors.frontCarDist = 9999;
  neighbors.leftFrontCarSpeed = 0;
  neighbors.leftFrontCarDist = 9999;
  neighbors.leftBackCarSpeed = 0;
  neighbors.leftBackCarDist = -9999;
  neighbors.rightFrontCarSpeed = 0;
  neighbors.rightFrontCarDist = 9999;
  neighbors.rightBackCarSpeed = 0;
  neighbors.rightBackCarDist = -9999;

  for(int i = 0; i < sensor_fusion.size(); ++i)
  {
    double d = sensor_fusion[i][6];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx + vy*vy);
    double check_s = sensor_fusion[i][5];
    check_s += prev_size * 0.02 * check_speed;
    double dist = check_s - car_s;
    // check current lane.
    if(d < laneTod(target_lane) + 2 && d > laneTod(target_lane) - 2)
    {
      if(dist > 0)
      {
        if(dist < neighbors.frontCarDist)
        {
          neighbors.frontCarDist = dist;
          neighbors.frontCarSpeed = check_speed;
        }
      }
    }

    // check left
    if(target_lane >= 1)
    {
      if(d < laneTod(target_lane - 1) + 2 && d > laneTod(target_lane - 1) - 2)
      {
        if(dist > 0)
        {
          if(dist < neighbors.leftFrontCarDist)
          {
            neighbors.leftFrontCarDist = dist;
            neighbors.leftFrontCarSpeed = check_speed;
          }
        }
        else
        {
          if(dist > neighbors.leftBackCarDist)
          {
            neighbors.leftBackCarDist = dist;
            neighbors.leftBackCarSpeed = check_speed;
          }
        }
      }
    }

    // check right
    if(target_lane <= 1)
    {
      if(d < laneTod(target_lane + 1) + 2 && d > laneTod(target_lane + 1) - 2)
      {
        if(dist > 0)
        {
          if(dist < neighbors.rightFrontCarDist)
          {
            neighbors.rightFrontCarDist = dist;
            neighbors.rightFrontCarSpeed = check_speed;
          }
        }
        else
        {
          if(dist > neighbors.rightBackCarDist)
          {
            neighbors.rightBackCarDist = dist;
            neighbors.rightBackCarSpeed = check_speed;
          }
        }
      }
    }
  }

  return neighbors;
}

// state transition function
static State nextState(
  const State& state, const CarNeighbors& neighbors, 
  double car_d, double car_speed)
{
  double closeDistance = 25.0;
  State re = state;
  // keep current lane state, if blocked by front car, switch to pass state.
  if(state.state == 0)
  {
    if(car_speed < speedLimit * 0.95 && neighbors.frontCarDist < closeDistance)
    {
      if(neighbors.leftFrontCarDist > closeDistance && 
        neighbors.leftBackCarDist < -closeDistance * 0.5 && 
        car_speed > neighbors.leftBackCarSpeed * 0.5 && state.target_lane > 0)
      {
        re.state = 1;
        re.target_lane = state.target_lane - 1;
      }
      else if(neighbors.rightFrontCarDist > closeDistance && 
        neighbors.rightBackCarDist < -closeDistance * 0.5 && 
        car_speed > neighbors.rightBackCarSpeed * 0.5 && state.target_lane < 2)
      {
        re.state = 1;
        re.target_lane = state.target_lane + 1;
      }
    }
  }

  // if pass state finished, switch back to keep lane state.
  if(state.state == 1)
  {
    if(fabs(car_d - laneTod(state.target_lane)) < 0.1)
    {
      re.state = 0;
      re.target_lane = state.target_lane;
    }
  }
  
  return re;
}

// path generate function
static void generatePath(
  int target_lane, const CarNeighbors& neighbors,
  double car_x, double car_y, double car_yaw, double car_s, double& car_speed_ref,
  const std::vector<double>& previous_path_x, 
  const std::vector<double>& previous_path_y,
  const vector<double>& map_waypoints_x,
  const vector<double>& map_waypoints_y,
  const vector<double>& map_waypoints_s,
  std::vector<double>& next_x_vals, 
  std::vector<double>& next_y_vals)
{
  double target_d = laneTod(target_lane);
  double brakeDistance = 25.0;
  int totalPoints = 30;
  int pointsToGenerate = totalPoints - previous_path_x.size();
  double acc = 0.1;

  std::cout << "front car distance " << neighbors.frontCarDist << " speed "<< neighbors.frontCarSpeed / 0.4474 <<std::endl;
  std::cout << "left cars " << neighbors.leftFrontCarDist << ", "<< neighbors.leftBackCarDist <<std::endl;
  std::cout << "right cars " << neighbors.rightFrontCarDist << ", "<< neighbors.rightBackCarDist <<std::endl;

  // generate anchor points
  std::vector<Eigen::Vector2d> anchor_points;
  int prev_size = previous_path_x.size();

  if(prev_size < 2)
  {
    double car_yaw_rad = deg2rad(car_yaw);

    anchor_points.push_back(Eigen::Vector2d(
      car_x - cos(car_yaw_rad),
      car_y - sin(car_yaw_rad)));

    anchor_points.push_back(Eigen::Vector2d(
      car_x,
      car_y));
  }
  else
  {
    anchor_points.push_back(Eigen::Vector2d(
      previous_path_x[prev_size - 2],
      previous_path_y[prev_size - 2]));

    anchor_points.push_back(Eigen::Vector2d(
      previous_path_x[prev_size - 1],
      previous_path_y[prev_size - 1]));
  }

  // compute angle with the last two prev points;
  double ref_yaw  = atan2(
    anchor_points[1].y() - anchor_points[0].y(),
    anchor_points[1].x() - anchor_points[0].x());

  // generate three more anchor points
  double wp_interval = 30.0;
  auto wp0 = getXY(car_s + wp_interval, target_d, 
    map_waypoints_s, map_waypoints_x, map_waypoints_y);
  auto wp1 = getXY(car_s + 2 * wp_interval, target_d, 
    map_waypoints_s, map_waypoints_x, map_waypoints_y);
  auto wp2 = getXY(car_s + 3 * wp_interval, target_d, 
    map_waypoints_s, map_waypoints_x, map_waypoints_y);

  anchor_points.push_back(Eigen::Vector2d(wp0[0], wp0[1]));
  anchor_points.push_back(Eigen::Vector2d(wp1[0], wp1[1]));
  anchor_points.push_back(Eigen::Vector2d(wp2[0], wp2[1]));

  // transform from carFromWorld
  Eigen::Matrix3d worldFromCar;
  worldFromCar << cos(ref_yaw), -sin(ref_yaw), anchor_points[1].x(),
    sin(ref_yaw), cos(ref_yaw), anchor_points[1].y(),
    0, 0, 1;
  Eigen::Matrix3d carFromWorld = worldFromCar.inverse();

  std::vector<double> ptx;
  std::vector<double> pty;
  
  for(int i = 0; i < anchor_points.size(); ++i)
  {
    anchor_points[i] = 
      (carFromWorld * (anchor_points[i].homogeneous())).hnormalized();
    ptx.push_back(anchor_points[i].x());
    pty.push_back(anchor_points[i].y());
  }

  tk::spline s;
  s.set_points(ptx, pty);

  double target_x = wp_interval;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x+target_y*target_y);

  next_x_vals.insert(
    next_x_vals.end(), previous_path_x.begin(), previous_path_x.end());
  next_y_vals.insert(
    next_y_vals.end(), previous_path_y.begin(), previous_path_y.end());

  double x_spline_prev = 0;
  for(int i = 0; i < pointsToGenerate; ++i)
  {
    if(neighbors.frontCarDist > brakeDistance)
    {
      car_speed_ref = fmin(speedLimit, car_speed_ref + acc);
    }
    else
    {
      if(car_speed_ref > neighbors.frontCarSpeed * 0.5)
      {
        car_speed_ref -= acc;
      }
    }
    
    double N = target_dist / (0.02 * car_speed_ref);
    double x_spline_cur = x_spline_prev + target_x / N;
    double y_spline_cur = s(x_spline_cur);
    Eigen::Vector2d pCar(x_spline_cur, y_spline_cur);
    Eigen::Vector2d pWorld = (worldFromCar * (pCar.homogeneous())).hnormalized();
    next_x_vals.push_back(pWorld.x());
    next_y_vals.push_back(pWorld.y());
    x_spline_prev = x_spline_cur;
  }
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

  // initialization parameters.
  State state;
  state.state = 0;
  state.target_lane = 1;
  double car_speed_ref = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &state, &car_speed_ref]
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // TODO start
          int prev_size = previous_path_x.size();
          
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          // check neighbor information.
          auto neighbors = checkNeighbors(
            state.target_lane, car_s, prev_size, sensor_fusion);
          
          // state transition.
          state = nextState(state, neighbors, end_path_d, car_speed_ref);
          
          // genearte path.
          generatePath(
            state.target_lane, neighbors,
            car_x, car_y, car_yaw, car_s, car_speed_ref,
            previous_path_x, 
            previous_path_y,
            map_waypoints_x,
            map_waypoints_y,
            map_waypoints_s,
            next_x_vals, 
            next_y_vals);
          
          // TODO end
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