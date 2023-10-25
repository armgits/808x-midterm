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

/**
* @brief Constructor for inverse kinematics class. Initializes attributes to zero.
*
*/
Inverse::Inverse() : input_coordinates_{}, output_coordinates_{}, robot_pose_{},
                    angle_constraints_{}, dh_params_{} {}

/**
 * @brief Method to compute the inverse kinematics for given target end-effector
 *         coordinate. Currently returns a dummy vector. Implementation pending.
 *
 * @param target_coordinates
 * @param initial_pose
 * @return std::vector<double>
 */
std::vector<double> Inverse::inverse(
    const std::vector<double>& target_coordinates,
    const std::vector<double>& initial_pose) {
  std::vector<double> dummy_vector {1.0};

  return dummy_vector;
}
