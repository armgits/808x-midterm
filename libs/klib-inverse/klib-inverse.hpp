/**
 * @file klib-inverse.hpp
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

#include "klib-datatypes.hpp"

/**
 * @brief Class for computing the inverse kinematics of an articulated arm
 *
 */
class Inverse {
 public:
  /**
   * @brief Constructor for an inverse kinematics solver object
   *
   */
  Inverse(const klib::DHParameters6R arm_description_);

  /**
   * @brief Method to compute the inverse kinematics with the solver object
   *
   * @param target_coordinates
   * @param current_pose
   * @return std::vector<double>
   */
  klib::ArmPose6R compute(const klib::Point& target_coordinates,
                          const klib::ArmPose6R& current_pose);

  /**
   * @brief Method to obtain the joint angles describing the robot pose computed
   *        by the inverse kinematics solver
   *
   * @return std::vector<double>
   */
  std::vector<double> get_robot_pose();

 private:
  /**
   * @brief Stores the target coordinates for the end-effector
   *
   */
  klib::Point input_coordinates_;

  /**
   * @brief Stores the pose computed to achieve target end-effector coordinates
   *
   */
  klib::ArmPose6R robot_pose_;

  /**
   * @brief Stores the DH Parameters that describes the arm
   *
   */
  klib::DHParameters6R dh_params_;
};
