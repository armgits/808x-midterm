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

 #include "Inverse.hpp"

 Inverse::Inverse() : input_coordinates_{}, output_coordinates_{}, robot_pose_{},
                      angle_constraints_{}, dh_params_{} {}

std::vector<double> Inverse::inverse(
    std::vector<double> target_coordinates, std::vector<double> initial_pose) {
  std::vector<double> dummy_vector {1.0};

  return dummy_vector;
}
