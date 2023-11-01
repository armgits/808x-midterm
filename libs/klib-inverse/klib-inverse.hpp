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
#include <Eigen/Dense>

#include "klib-datatypes.hpp"

namespace klib {
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
  explicit Inverse(const klib::DHParameters6R arm_description_);

  /**
   * @brief Method to compute the inverse kinematics for a given end-effector
   *        pose.
   *
   * @param target_tool_pose
   * @param current_arm_pose
   * @return klib::ArmPose6R
   */
  klib::ArmPose6R Compute(const klib::Pose& target_tool_pose,
                          const klib::ArmPose6R& current_arm_pose);

  /**
   * @brief Method to obtain the joint angles describing the robot pose computed
   *        by the inverse kinematics solver
   *
   * @return std::vector<double>
   */
  klib::ArmPose6R GetSolution();

 private:
  /**
   * @brief Generates an A matrix for a given theta and DH parameters of the joint
   *
   * @param theta Angle of joint
   * @param joint Joint DH Parameters
   * @return Eigen::Matrix<double, 4, 4>
   */
  Eigen::Matrix<double, 4, 4> GenerateAMatrix(double theta,
                                             const klib::JointParameter& joint);

  /**
   * @brief Computes the Jacobian for a given arm pose using the DH parameters
   *
   * @param current_pose
   * @return Eigen::Matrix<double, 6, 6>
   */
  Eigen::Matrix<double, 6, 6> ComputeJacobian(const klib::ArmPose6R &current_arm_pose);

  /**
   * @brief Stores the target coordinates for the end-effector
   *
   */
  klib::Pose tool_pose_;

  /**
   * @brief Stores the pose computed to achieve target end-effector coordinates
   *
   */
  klib::ArmPose6R arm_pose_;

  /**
   * @brief Stores the DH Parameters that describes the arm
   *
   */
  klib::DHParameters6R dh_parameters_;
};

}
