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
  Inverse();

  /**
   * @brief Method to compute the inverse kinematics with the solver object
   *
   * @param target_coordinates
   * @param initial_pose
   * @return std::vector<double>
   */
  std::vector<double> inverse(std::vector<double> target_coordinates,
                              std::vector<double> initial_pose);

  /**
   * @brief Method to obtain the target coordinates for the end-effector
   *
   * @return std::vector<double>
   */
  std::vector<double> get_input_coordinates();

  /**
   * @brief Method to set the target coordinates for the end-effector
   *
   * @param input_coords
   */
  void set_input_coordinates(std::vector<double> input_coords);

  /**
   * @brief Method to obtain the joint angles describing the robot pose computed
   *        by the inverse kinematics solver
   *
   * @return std::vector<double>
   */
  std::vector<double> get_robot_pose();

  /**
   * @brief Method to manually override the robot pose computed by the inverse
   *        kinematics solver
   *
   * @param robot_pose
   */
  void set_robot_pose(std::vector<double> robot_pose);

  /**
   * @brief Method to obtain the joint angles constraints set for the arm and
   *        for generating an inverse kinematics solution.
   *
   * @return std::vector<double>
   */
  std::vector<double> get_angle_constraint();

  /**
   * @brief Method to set the joint angles constraints to to generate a valid
   *        inverse kinematics solution.
   *
   * @param angle_constraints
   */
  void set_angle_constraint(std::vector<double> angle_constraints);

  /**
   * @brief Method to obtain the DH Parameters describing the arm for computing
   *        inverse kinematics.
   *
   * @return double**
   */
  double **get_dh_params();

 private:
  /**
   * @brief Stores the target coordinates for the end-effector
   *
   */
  std::vector<double> input_coordinates_;

  /**
   * @brief Stores the final coordinates achieved by the solver object
   *
   */
  std::vector<double> output_coordinates_;

  /**
   * @brief Stores the pose computed to achieve target end-effector coordinates
   *
   */
  std::vector<double> robot_pose_;

  /**
   * @brief Stores the joint angle constraints for generating a valid solution
   *
   */
  std::vector<double> angle_constraints_;

  /**
   * @brief Stores the DH Parameters that describes the arm
   *
   */
  double **dh_params_;
};
