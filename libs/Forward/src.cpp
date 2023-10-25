/**
 * @file src.cpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @brief Source file for Forward kinematics library definitions
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#include "Forward.hpp"

Forward::Forward() : input_angles_{}, output_angles_{}, robot_tcp_position_{},
                     coordinate_constraints_{}, dh_params_{} {}

std::vector<double> Forward::forward(
    std::vector<double> joint_angles, std::vector<double> tcp_position) {
  std::vector<double> dummy_vector {1.0};
  return dummy_vector;
}
