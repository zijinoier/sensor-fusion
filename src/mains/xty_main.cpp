#include <math.h>

#include <fstream>  //ifstream
#include <fusion.hpp>
#include <iostream>
#include <vector>
#define PI 3.14159265
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

  // fusion
  EKF_CV_LIDAR_RADAR_Fusion fusion{cv_mtx, lidar_mtx, radar_mtx};

  for (int time_stamp = 0; time_stamp < lidar_number; time_stamp++) {
    Radar::MeasurementVector r_meas_vect;
    // r_meas_vect << radar_targets[time_stamp].dist *
    //                    cos(radar_targets[time_stamp].angle * PI / 180.0),
    //     radar_targets[time_stamp].dist *
    //         sin(radar_targets[time_stamp].angle * PI / 180.0),
    //     0;
    r_meas_vect << radar_targets[time_stamp].dist,
        (radar_targets[time_stamp].angle * PI / 180.0), 0;
    Radar::Measurement r_measurement{time_stamp, r_meas_vect};
    auto belief{fusion.ProcessMeasurement(r_measurement)};
    const auto& sv{belief.mu()};
    CV::ROStateVectorView state_vector_view{sv};
    // std::cout << "time: " << time_stamp << " " <<
    // radar_targets[time_stamp].dist
    //           << " " << radar_targets[time_stamp].angle << " "
    //           << state_vector_view.px() << " " << state_vector_view.py()
    //           << std::endl;

    Lidar::MeasurementVector l_meas_vect;
    l_meas_vect << lidar_targets[time_stamp].dist *
                       cos(lidar_targets[time_stamp].angle * PI / 180.0),
        lidar_targets[time_stamp].dist *
            sin(lidar_targets[time_stamp].angle * PI / 180.0);
    Lidar::Measurement l_measurement{time_stamp, l_meas_vect};
    auto l_belief{fusion.ProcessMeasurement(l_measurement)};
    const auto& l_sv{l_belief.mu()};
    CV::ROStateVectorView l_state_vector_view{l_sv};
    // std::cout << "time: " << time_stamp << " " <<
    // lidar_targets[time_stamp].dist
    //           << " " << lidar_targets[time_stamp].angle << " "
    //           << l_state_vector_view.px() << " " << l_state_vector_view.py()
    //           << std::endl;
    std::cout << time_stamp << " " << l_state_vector_view.px() << " "
              << l_state_vector_view.py() << std::endl;
  }
}