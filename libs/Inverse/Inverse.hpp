/**
 * @file Inverse.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @brief Header file for Inverse kinematics library declarations
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#pragma once

#include <vector>

class Inverse {

 public:
  Inverse();
  std::vector<double> inverse(std::vector<double> target_coordinates,
                              std::vector<double> initial_pose);
  std::vector<double> get_input_coordinates();
  void set_input_coordinates(std::vector<double> input_coords);
  std::vector<double> get_robot_pose();
  void set_robot_pose(std::vector<double> robot_pose);
  std::vector<double> get_angle_constraint();
  void set_angle_constraint(std::vector<double> angle_constraints);
  double **get_dh_params();

 private:
  std::vector<double> input_coordinates_;
  std::vector<double> output_coordinates_;
  std::vector<double> robot_pose_;
  std::vector<double> angle_constraints_;
  double **dh_params_;
};
