#include "planner.h"
#include "spline.h"

#include <iostream>

////////////////////////////////////////////////////////////

//
// Helper functions related to waypoints and converting from XY to Frenet or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
  const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
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

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
  const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
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
  double proj_norm = (x_x*n_x + x_y * n_y) / (n_x*n_x + n_y * n_y);
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
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return { frenet_s,frenet_d };
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
  const vector<double> &maps_x,
  const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
    (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return { x,y };
}

////////////////////////////////////////////////////////////

Planner::Planner(
  const vector<double> &map_waypoints_x,
  const vector<double> &map_waypoints_y,
  const vector<double> &map_waypoints_s,
  const vector<double> &map_waypoints_dx,
  const vector<double> &map_waypoints_dy)
{
  this->map_w_x = map_waypoints_x;
  this->map_w_y = map_waypoints_y;
  this->map_w_s = map_waypoints_s;
  this->map_w_dx = map_waypoints_dx;
  this->map_w_dy = map_waypoints_dy;

  current_lane = 1;
  current_velocity = 0.0;
  max_velocity = 22.0; // [m/s]
  accel_decel = 0.1;
  min_traffic_gap = 30.0;
  max_path_size = 50;
  spline_step = 30.0;
  planning_horizon = 30.0;
}

void Planner::CalculateTrajectory(
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
  std::vector<double> *next_y_vals)
{

  size_t prev_n = previous_path_x.size();
  double ref_s = (prev_n > 0) ? end_path_s : car_s;

  // *** Prediction  ***
  
  double current_d = 2 + 4 * current_lane;
  
  bool too_close = false;

  for (const auto &other_car : sensor_fusion) {

    double x = other_car[1];
    double y = other_car[2];
    double vx = other_car[3];
    double vy = other_car[4];
    double s = other_car[5];
    double d = other_car[6];

    if (d > (current_d - 2) && d < (current_d + 2)) {
      double check_car_vel = sqrt(vx * vx + vy * vy);
      double check_car_s = s + prev_n * 0.02 * check_car_vel;
      if ((check_car_s > ref_s) && (check_car_s - ref_s < min_traffic_gap)) {
        too_close = true;
      }
    }
  }

  // Adjust velocity
  if (too_close) {
    current_velocity -= accel_decel;
  } else if (current_velocity < max_velocity) {
    current_velocity += accel_decel;
  }

  // *** Create points for trajectory generation (spline interpolation) ***

  vector<double> spline_px;
  vector<double> spline_py;

  double ref_x;
  double ref_y;
  double ref_x0;
  double ref_y0;
  double ref_yaw;

  if (prev_n < 2) {
    // Use car as reference
    ref_x = car_x;
    ref_y = car_y;
    ref_yaw = deg2rad(car_yaw);
    ref_x0 = ref_x - cos(ref_yaw);
    ref_y0 = ref_y - sin(ref_yaw);
  } else {
    // Use previous path as reference
    ref_x = previous_path_x[prev_n - 1];
    ref_y = previous_path_y[prev_n - 1];
    ref_x0 = previous_path_x[prev_n - 2];
    ref_y0 = previous_path_y[prev_n - 2];
    ref_yaw = atan2(ref_y - ref_y0, ref_x - ref_x0);
  }
  
  // Add reference points for spline interpolation
  spline_px.push_back(ref_x0);
  spline_py.push_back(ref_y0);
  spline_px.push_back(ref_x);
  spline_py.push_back(ref_y);

  // Add some more points ahead for spline interpolation
  double step = spline_step;
  auto p0 = getXY(ref_s + step, (2 + 4 * current_lane), map_w_s, map_w_x, map_w_y);
  auto p1 = getXY(ref_s + step * 2, (2 + 4 * current_lane), map_w_s, map_w_x, map_w_y);
  auto p2 = getXY(ref_s + step * 3, (2 + 4 * current_lane), map_w_s, map_w_x, map_w_y);

  spline_px.push_back(p0[0]);
  spline_px.push_back(p1[0]);
  spline_px.push_back(p2[0]);

  spline_py.push_back(p0[1]);
  spline_py.push_back(p1[1]);
  spline_py.push_back(p2[1]);

  // Transform (shift and rotate) to local car coordinates
  for (size_t i = 0; i < spline_px.size(); ++i) {
    double shift_x = spline_px[i] - ref_x;
    double shift_y = spline_py[i] - ref_y;
    spline_px[i] = shift_x * cos(0.0 - ref_yaw) - shift_y * sin(0.0 - ref_yaw);
    spline_py[i] = shift_x * sin(0.0 - ref_yaw) + shift_y * cos(0.0 - ref_yaw);
  }

  // *** Generate trajectory (Spline interpolation) ***

  // Create a spline
  tk::spline spline;
  spline.set_points(spline_px, spline_py);

  // Copy previous points
  next_x_vals->assign(previous_path_x.begin(), previous_path_x.end());
  next_y_vals->assign(previous_path_y.begin(), previous_path_y.end());

  // Calculate number of points to respect reference velocity
  double target_x = planning_horizon;
  double target_y = spline(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  // Perform spline interpolation and a create a trajectory
  double x_add_on = 0.0;  
  for (size_t i = 1; i < max_path_size - prev_n; ++i) {

    // Car coordinates
    double N = target_dist / (0.02 * current_velocity);
    double x_point = x_add_on + target_x / N;
    double y_point = spline(x_point);    
    x_add_on = x_point;

    // Global coordinates
    double x0 = x_point;
    double y0 = y_point;
    x_point = ref_x + x0 * cos(ref_yaw) - y0 * sin(ref_yaw);
    y_point = ref_y + x0 * sin(ref_yaw) + y0 * cos(ref_yaw);

    next_x_vals->push_back(x_point);
    next_y_vals->push_back(y_point);
  }

}