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

/**
 * @brief Constructor for forward kinematics class. Initializes attributes to zero.
 *
 */
Forward::Forward() : input_angles_{}, output_angles_{}, robot_tcp_position_{},
                     coordinate_constraints_{}, dh_params_{} {}

/**
 * @brief Method to compute the forward kinematics of the arm for given joint angles.
 *        Currently returns a dummy vector. Implementation pending.
 *
 * @param joint_angles
 * @param tcp_position
 * @return std::vector<double>
 */
std::vector<double> Forward::forward(
    const std::vector<double>& joint_angles,
    const std::vector<double>& tcp_position) {
  std::vector<double> dummy_vector {1.0};
  return dummy_vector;
}
