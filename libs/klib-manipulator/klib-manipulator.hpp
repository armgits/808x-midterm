/**
 * @file Manipulator.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @author Abhimanyu Saxena (asaxena4@umd.edu)
 * @brief Header file for Manipulator library declarations
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#pragma once

#include <iostream>
#include <utility>
#include <vector>

#include "klib-datatypes.hpp"

/**
 * @brief Class for describing a manipulator
 *
 */
class Manipulator {
 public:
  /**
   * @brief Constructs a new Manipulator object
   *
   */
  Manipulator();

  /**
   * @brief Sets the DH Parameters that describes the arm
   *
   * @param params
   */
  void set_dh_params(const double params[6][4]);

  /**
   * @brief Get the dh_params object
   *
   * @return klib::DHParameters6R object
   */
  klib::DHParameters6R get_dh_params();

  /**
   * @brief Gets the current joint positions of the arm
   *
   */
  klib::ArmPose6R get_joint_angles();

  /**
   * @brief Sets the current joint positions of the arm
   *
   * @param joint_angles desired joint angles
   */
  bool set_joint_angles(const std::vector<double> &joint_angles);

  /**
   * @brief Gets the current tcp position of the arm
   *
   */
  klib::Pose get_tcp_position();

  /**
   * @brief Sets the current tcp positions of the arm
   *
   * @param tcp_position desired tcp position
   */
  bool set_tcp_position(const std::vector<double> &tcp_position);

 private:
  /**
   * @brief Stores the degrees of freedom of arm
   *
   */
  int dof_;

  /**
   * @brief Stores the DH Parameters of the arm
   *
   */
  klib::DHParameters6R dh_params_;

  /**
   * @brief Stores current joint angles of the arm
   *
   */
  klib::ArmPose6R robot_joint_angles_;

  /**
   * @brief Stores current tcp position of the arm
   *
   */
  klib::Pose robot_tcp_pose_;
};
