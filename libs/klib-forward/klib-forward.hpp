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

#include <Eigen/Dense>
#include <iostream>
#include <utility>
#include <vector>

#include "klib-datatypes.hpp"
#include "klib-manipulator.hpp"

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

  struct angle_constraint_ {
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
  klib::Pose forward(const std::vector<double>& joint_angles,
                     Manipulator m_obj);
};

Eigen::Matrix4d compute_DH_matrix(double a, double alpha, double d,
                                  double theta);
