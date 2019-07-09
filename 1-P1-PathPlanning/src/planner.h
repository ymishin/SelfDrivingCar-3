#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <math.h>
#include "spline.h"

using std::vector;

class Planner
{

public:
  
  Planner(
    const vector<double> &map_waypoints_x,
    const vector<double> &map_waypoints_y,
    const vector<double> &map_waypoints_s,
    const vector<double> &map_waypoints_dx,
    const vector<double> &map_waypoints_dy);
  
  void CalculateTrajectory(
    const double car_x,
    const double car_y,
    const double car_s,
    const double car_d,
    const double car_yaw,
    const double car_speed,
    const double end_path_s,
    const double end_path_d,
    const std::vector<double> &previous_path_x,
    const std::vector<double> &previous_path_y,
    const std::vector<std::vector<double>> &sensor_fusion,
    std::vector<double> *next_x_vals,
    std::vector<double> *next_y_vals);

private:

  int target_lane;
  int current_lane;  
  double current_velocity;
  double max_velocity;
  double accel_decel;
  int max_path_size;
  double spline_step;
  double planning_horizon;
  double min_traffic_gap;
  double min_traffic_gap_front;
  double min_traffic_gap_change_front;
  double min_traffic_gap_change_back;

  vector<double> map_w_x;
  vector<double> map_w_y;
  vector<double> map_w_s;
  vector<double> map_w_dx;
  vector<double> map_w_dy;

};

#endif // PLANNER_H