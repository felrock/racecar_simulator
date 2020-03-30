#include "f110_simulator/pose_2d.hpp"
#include "f110_simulator/scan_simulator_2d.hpp"
#include "f110_simulator/distance_transform.hpp"
#include <iostream>

using namespace racecar_simulator;

ScanSimulator2D::ScanSimulator2D(
    int num_beams_,
    double field_of_view_,
    double scan_std_dev_,
    double ray_tracing_epsilon_,
    int theta_discretization)
  : num_beams(num_beams_),
    field_of_view(field_of_view_),
    scan_std_dev(scan_std_dev_),
    ray_tracing_epsilon(ray_tracing_epsilon_),
    theta_discretization(theta_discretization)
{
  // 2*M_PI
  M_PI2 = 2.0*M_PI;

  // Initialize laser settings
  angle_increment = field_of_view/(num_beams - 1);

  // Initialize the output
  scan_output = std::vector<double>(num_beams);

  // Initialize the noise
  noise_generator = std::mt19937(std::random_device{}());
  noise_dist = std::normal_distribution<double>(0., scan_std_dev);

  // Precompute sines and cosines
  theta_index_increment = theta_discretization * angle_increment/(M_PI2);
  sines = std::vector<double>(theta_discretization + 1);
  cosines = std::vector<double>(theta_discretization + 1);
  for (int i = 0; i <= theta_discretization; i++) {
    double theta = (2 * M_PI * i)/((double) theta_discretization);
    sines[i] = std::sin(theta);
    cosines[i] = std::cos(theta);
  }
}

const std::vector<double> ScanSimulator2D::scan(const Pose2D & pose) {
  scan(pose, scan_output.data());
  return scan_output;
}

void ScanSimulator2D::scan(const Pose2D & pose, double * scan_data) {
  // Make theta discrete by mapping the range [-pi,pi] onto [0, theta_discretization)
  double theta_index =
    theta_discretization * (pose.theta - field_of_view/2.)/(M_PI2);

  // Make sure it is wrapped properly
  //theta_index = std::fmod(theta_index, theta_discretization);
  if (theta_index < 0)
  {
    theta_index += theta_discretization;
  }

  // Sweep through each beam
  for (size_t i = 0; i < num_beams; ++i) {
    // Compute the distance to the nearest point
    scan_data[i] = trace_ray(pose.x, pose.y, theta_index);

    // Add Gaussian noise to the ray trace
    //if (scan_std_dev > 0)
    scan_data[i] += noise_dist(noise_generator);

    // Increment the scan
    theta_index += theta_index_increment;

    // Make sure it stays in the range [0, theta_discretization)
    if (theta_index >= theta_discretization)
    {
      theta_index -= theta_discretization;
    }
  }
}

double ScanSimulator2D::trace_ray(double x, double y, double theta_index) const {

  int theta_index_ = std::round(theta_index);
  double s = sines[theta_index_];
  double c = cosines[theta_index_];

  // Initialize the distance to the nearest obstacle
  double distance_to_nearest = distance_transform(x, y);
  double total_distance = distance_to_nearest;

  while (distance_to_nearest > ray_tracing_epsilon)
  {
    // Move in the direction of the ray
    // by distance_to_nearest
    x += distance_to_nearest * c;
    y += distance_to_nearest * s;

    // Compute the nearest distance at that point
    distance_to_nearest = distance_transform(x, y);
    total_distance += distance_to_nearest;
  }

  return total_distance;
}

double ScanSimulator2D::distance_transform(double x, double y) const {
  // Convert the pose to a grid cell

  // Translate the state by the origin
  double x_trans = x - origin.x;
  double y_trans = y - origin.y;

  // Rotate the state into the map
  double x_rot =   x_trans * origin_c + y_trans * origin_s;
  double y_rot = - x_trans * origin_s + y_trans * origin_c;

  // Clip the state to be a cell
  if (x_rot < 0 or y_rot < 0 )
  {
    return 0;
  }
  // Clip the state to be a cell
  if (x_rot >= width_res or y_rot >= height_res)
  {
    return 0;
  }

  // Discretize the state into row and column
  // floor was removed
  int col = x_rot/resolution;
  int row = y_rot/resolution;

  int cell = row * width + col;

  return dt[cell];
}

void ScanSimulator2D::set_map(
    const std::vector<double> & map,
    size_t height_,
    size_t width_,
    double resolution_,
    const Pose2D & origin_,
    double free_threshold) {

  // Assign parameters
  height = height_;
  width = width_;
  resolution = resolution_;
  origin = origin_;
  origin_c = std::cos(origin.theta);
  origin_s = std::sin(origin.theta);

  // size times resolution
  width_res  = width*resolution;
  height_res = height*resolution;

  // Threshold the map
  dt = std::vector<double>(map.size());
  for (size_t i = 0; i < map.size(); i++) {
    if (0 <= map[i] and map[i] <= free_threshold) {
      dt[i] = 99999; // Free
    } else {
      dt[i] = 0; // Occupied
    }
  }
  DistanceTransform::distance_2d(dt, width, height, resolution);
}

// overload for changing map on the fly
void ScanSimulator2D::set_map(const std::vector<double> & map, double free_threshold) {
  for (size_t i = 0; i < map.size(); i++) {
    if (0 <= map[i] and map[i] <= free_threshold) {
      dt[i] = 99999; // Free
    } else {
      dt[i] = 0; // Occupied
    }
  }
  DistanceTransform::distance_2d(dt, width, height, resolution);
}
