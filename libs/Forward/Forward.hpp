/**
 * @file Forward.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @brief Header file for Forward kinematics library declarations
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#pragma once

#include <vector>

class Forward {

 public:
  Forward();
  std::vector<double> forward(std::vector<double> joint_angles,
                              std::vector<double> tcp_position);
  std::vector<double> get_input_angles();
  void set_input_angles(std::vector<double> input_angles);
  std::vector<double> get_tcp_position();
  void set_tcp_position(std::vector<double> tcp_position);
  std::vector<double> get_coordinate_constraint();
  void set_coordinate_constraint(std::vector<double> coordinate_constraint);
  double **get_dh_params();
  std::vector<double> get_angle_constraint();
  void set_angle_constraint(std::vector<double> angle_constraint);

 private:
  std::vector<double> input_angles_;
  std::vector<double> output_angles_;
  std::vector<double> robot_tcp_position_;
  std::vector<double> coordinate_constraints_;
  double **dh_params_;
};
