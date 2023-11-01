/**
 * @file src.cpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @author Abhimanyu Saxena (asaxena4@umd.edu)
 * @brief Source file for Manipulator library definitions
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#include "klib-datatypes.hpp"
#include "klib-manipulator.hpp"
#include <iostream>
#include <utility>
#include <vector>

/**
 * @brief Construct a new Manipulator:: Manipulator object
 *
 */
Manipulator::Manipulator()
    : dof_(6), dh_params_(), robot_joint_angles_(), robot_tcp_pose_() {}

/**
 * @brief Method definition for set_dh_params
 *
 * @param dhArray 6*4 array defining DH parameters for robot arm
 */
void Manipulator::set_dh_params(const double dhArray[6][4]) {
  dh_params_.joint1.d_length = dhArray[0][0];
  dh_params_.joint1.a_length = dhArray[0][1];
  dh_params_.joint1.alpha = dhArray[0][2];
  dh_params_.joint1.offset = dhArray[0][3];

  dh_params_.joint2.d_length = dhArray[1][0];
  dh_params_.joint2.a_length = dhArray[1][1];
  dh_params_.joint2.alpha = dhArray[1][2];
  dh_params_.joint2.offset = dhArray[1][3];

  dh_params_.joint3.d_length = dhArray[2][0];
  dh_params_.joint3.a_length = dhArray[2][1];
  dh_params_.joint3.alpha = dhArray[2][2];
  dh_params_.joint3.offset = dhArray[2][3];

  dh_params_.joint4.d_length = dhArray[3][0];
  dh_params_.joint4.a_length = dhArray[3][1];
  dh_params_.joint4.alpha = dhArray[3][2];
  dh_params_.joint4.offset = dhArray[3][3];

  dh_params_.joint5.d_length = dhArray[4][0];
  dh_params_.joint5.a_length = dhArray[4][1];
  dh_params_.joint5.alpha = dhArray[4][2];
  dh_params_.joint5.offset = dhArray[4][3];

  dh_params_.joint6.d_length = dhArray[5][0];
  dh_params_.joint6.a_length = dhArray[5][1];
  dh_params_.joint6.alpha = dhArray[5][2];
  dh_params_.joint6.offset = dhArray[5][3];
}

/**
 * @brief Method definition for get_joint_angles
 *
 * @return klib::ArmPose6R Struct representing current joint angles
 */
klib::ArmPose6R Manipulator::get_joint_angles() { return robot_joint_angles_; }

/**
 * @brief Method definition for get_tcp_position
 *
 * @return klib::Pose Struct representing current tcp position
 */
klib::Pose Manipulator::get_tcp_position() { return robot_tcp_pose_; }

/**
 * @brief Method definition for get_dh_params
 *
 * @return klib::DHParameters6R dh_param obejct
 */
klib::DHParameters6R Manipulator::get_dh_params() { return dh_params_; }

/**
 * @brief Method definition for set_joint_angles
 *
 * @param joint_angles Desired joint angles
 */
bool Manipulator::set_joint_angles(const std::vector<double> &joint_angles) {
  robot_joint_angles_.theta1 = joint_angles[0];
  robot_joint_angles_.theta2 = joint_angles[1];
  robot_joint_angles_.theta3 = joint_angles[2];
  robot_joint_angles_.theta4 = joint_angles[3];
  robot_joint_angles_.theta5 = joint_angles[4];
  robot_joint_angles_.theta6 = joint_angles[5];

  return true;
}
/**
 * @brief Method definition for set_tcp_position
 *
 * @param tcpPosition Desired tcp position
 */
bool Manipulator::set_tcp_position(const std::vector<double> &tcpPosition) {
  robot_tcp_pose_.x = tcpPosition[0];
  robot_tcp_pose_.y = tcpPosition[1];
  robot_tcp_pose_.z = tcpPosition[2];
  robot_tcp_pose_.wx = tcpPosition[3];
  robot_tcp_pose_.wy = tcpPosition[4];
  robot_tcp_pose_.wz = tcpPosition[5];

  return true;
}
