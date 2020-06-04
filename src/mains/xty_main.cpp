#include <fstream>  //ifstream
#include <fusion.hpp>
#include <iostream>
#include <vector>
using namespace ser94mor::sensor_fusion;
struct lidar_target {
  double dist, angle;  // angle in degree
};
struct radar_target {
  double dist, angle, speed, course;  // angle course in degree
};

int main() {
  std::cout << "Start processing data\n";
  // parameter initialization
  CV::ProcessNoiseCovarianceMatrix cv_mtx;
  cv_mtx << 9.0, 0.0, 0.0, 9.0;

  Lidar::MeasurementCovarianceMatrix lidar_mtx;
  lidar_mtx << 0.0225, 0.0, 0.0, 0.0225;

  Radar::MeasurementCovarianceMatrix radar_mtx;
  radar_mtx << 0.09, 0.0, 0.0, 0.0, 0.0009, 0.0, 0.0, 0.0, 0.09;

  std::vector<lidar_target> lidar_targets;
  std::vector<radar_target> radar_targets;

  std::ifstream infile;
  infile.open("data/matlab_lidar_target.txt");
  int lidar_number;
  infile >> lidar_number;
  for (int i = 0; i < lidar_number; i++) {
    lidar_target temp_target;
    infile >> temp_target.dist;
    infile >> temp_target.angle;
    lidar_targets.push_back(temp_target);
  }
  std::cout << "Read " << lidar_targets.size() << " lidar targets\n";
  infile.close();

  infile.open("data/matlab_radar_target.txt");
  int radar_number;
  infile >> radar_number;
  for (int i = 0; i < radar_number; i++) {
    radar_target temp_target;
    infile >> temp_target.dist;
    infile >> temp_target.angle;
    infile >> temp_target.speed;
    infile >> temp_target.course;
    radar_targets.push_back(temp_target);
  }
  std::cout << "Read " << radar_targets.size() << " radar targets\n";
  infile.close();
}