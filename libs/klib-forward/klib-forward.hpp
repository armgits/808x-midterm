/**
 * @file klib-forward.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @author Mudit Singal (msingal@umd.edu)
 * @brief Header file for Forward kinematics library declarations
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#pragma once

#include <vector>
#include <utility>
#include <iostream>
#include "klib-datatypes.hpp"

/**
 * @brief Class for computing forward kinematics of an articulated arm
 *
 */
class Forward {
 public:
  /**
   * @brief Constructor for forward kinematics solver object
   *
   */
  Forward();

  struct angle_constraint_
  {
    double min_angle_;
    double max_angle_;
  };

  /**
   * @brief Method to compute the forward kinematics that outputs coordinates of
   *        end-effector for given joint angles
   *
   * @param joint_angles
   * @param tcp_position
   * @return std::vector<double>
   */
  std::vector<double> forward(const std::vector<double>& joint_angles,
                              const std::vector<double>& tcp_position);

  /**
   * @brief Method to obtain input angles given to forward kinematics solver
   *
   * @return std::vector<double>
   */
  std::vector<double> get_input_angles();

  /**
   * @brief Method to set the input joint angles to forward kinematics solver
   *
   * @param input_angles
   */
  void set_input_angles(const std::vector<double>& input_angles);

  /**
   * @brief Method to obtain the end-effector position calculated by forward
   *        kinematics solver
   *
   * @return std::vector<double>
   */
  std::vector<double> get_tcp_position();

  /**
   * @brief Method to manually override end-effector position computed from
   *        joint angles
   *
   * @param tcp_position
   */
  void set_tcp_position(const std::vector<double>& tcp_position);

  /**
   * @brief Obtains the constraints set to the end-effector coordinate computed
   *        by the forward kinematics solver
   *
   * @return std::vector<double>
   */
  std::vector<double> get_coordinate_constraint();

  /**
   * @brief Sets the constraints for the end-effector coordinate computed by the
   *        forward kinematics solver
   *
   * @param coordinate_constraint
   */
  void set_coordinate_constraint(
      const std::vector<double>& coordinate_constraint);

  /**
   * @brief Obtains the DH parameters set for the forward kinematics solver
   *
   * @return double**
   */
  double **get_dh_params();

  /**
   * @brief Obtains the constraints set for the joint angles
   *
   * @return std::vector<double>
   */
  std::vector<angle_constraint_> get_angle_constraint();

  /**
   * @brief Set the angle constraint object
   *
   * @param angle_constraint
   */
  void set_angle_constraint(const std::vector<angle_constraint_>& angle_constraint);

 private:
  /**
   * @brief Stores the input joint angles of the arm
   *
   */
  std::vector<double> input_angles_;

  /**
   * @brief Stores the output joint angles of the arm
   *
   */
  std::vector<double> output_angles_;

  /**
   * @brief Stores the end-effector position and orientation of the arm
   *
   */
  std::vector<double> robot_tcp_pose_;

  /**
   * @brief Stores the constraints set for the end-effector coordinates
   *
   */
  std::vector<double> coordinate_constraints_;

  /**
   * @brief Stores the constraints for minimum and maximum angles that each joint can have. Index 0 corresponds to the first joint and so on.
   * 
   */

  std::vector<angle_constraint_> angle_constraints_;

  /**
   * @brief Stores the DH Parameters that describes the arm for forward kinematics
   *
   */
  double **dh_params_;
};
