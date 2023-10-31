/**
 * @file src.cpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
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

Manipulator::Manipulator()
    : dof_(6), dh_params_(), robot_joint_angles_(), robot_tcp_pose_() {}

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
