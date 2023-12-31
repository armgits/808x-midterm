/**
 * @file datatypes.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @author Mudit Singal (msingal@umd.edu)
 * @brief Header-only library for custom datatypes for kinematics and planning
 * @version 1.0
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#pragma once

#include <cmath>

namespace klib {

/**
 * @brief Struct that describes the pose of an object in the real world using
 *        position and orientation
 *
 */
struct Pose {
  float x {0.0};
  float y {0.0};
  float z {0.0};
  float wx {0.0};
  float wy {0.0};
  float wz {0.0};

  /**
   * @brief Default constructor for Pose struct, zero initializes the members
   *
   */
  Pose() {};

  /**
   * @brief Constructor for individual position and orientation values
   *
   */
  constexpr Pose(float x_i, float y_i, float z_i, float wx_i, float wy_i,
          float wz_i) : x{x_i}, y{y_i}, z{z_i}, wx{wx_i}, wy{wy_i}, wz{wz_i} {}

  /**
   * @brief Greater than comparision operator overloaded to this structure
   *
   * @param pose_compare
   * @return true
   * @return false
   */
  bool operator>(const Pose& pose_compare) {
    if (x > pose_compare.x && y > pose_compare.y && z > pose_compare.z &&
        wx > pose_compare.wx && wy > pose_compare.wy && wz > pose_compare.wz) {
      return true;
    }

    else {
      return false;
    }
  }

  /**
   * @brief Subtraction operator overloaded for finding difference between
   *        two poses.
   *
   * @param pose_subtrahend
   * @return Pose
   */
  Pose operator-(const Pose& pose_subtrahend) const {
    Pose pose_difference {
      x - pose_subtrahend.x,
      y - pose_subtrahend.y,
      z - pose_subtrahend.z,
      wx - pose_subtrahend.wx,
      wy - pose_subtrahend.wy,
      wz - pose_subtrahend.wz
    };

    return pose_difference;
  }

  /**
   * @brief Method that finds the absolute difference between two poses.
   *
   * @param pose_subtrahend
   * @return Pose
   */
  Pose absdiff(const Pose& pose_subtrahend) const {
    Pose pose_abs_difference {
      std::abs(x - pose_subtrahend.x),
      std::abs(y - pose_subtrahend.y),
      std::abs(z - pose_subtrahend.z),
      std::abs(wx - pose_subtrahend.wx),
      std::abs(wy - pose_subtrahend.wy),
      std::abs(wz - pose_subtrahend.wz)
    };

    return pose_abs_difference;
  }
};

/**
 * @brief Struct that describes the DH parameters of a joint in the arm
 *
 */
struct JointParameter {
  float offset {0.0};
  float a_length {0.0};
  float d_length {0.0};
  float alpha {0.0};
  float min_limit {0.0};
  float max_limit {0.0};

  /**
   * @brief Default constructor, zero initiazes all members
   *
   */
  JointParameter() {}

  /**
   * @brief Constructor for specific values passed during instantiation
   *
   * @param offset_i
   * @param a_length_i
   * @param d_length_i
   * @param alpha_i
   * @param min_limit_i
   * @param max_limit_i
   */
  JointParameter(
    float offset_i,
    float a_length_i,
    float d_length_i,
    float alpha_i,
    float min_limit_i,
    float max_limit_i
  )
  : offset{offset_i},
    a_length{a_length_i},
    d_length{d_length_i},
    alpha{alpha_i},
    min_limit{min_limit_i},
    max_limit{max_limit_i} {}
};

/**
 * @brief Struct that stores the DH parameters of an arm. Each member of this
 *        struct consists of DH parameters of a specific joint
 *
 */
struct DHParameters6R {
  JointParameter joint1;
  JointParameter joint2;
  JointParameter joint3;
  JointParameter joint4;
  JointParameter joint5;
  JointParameter joint6;

  /**
   * @brief Default constructor, zero initializes all the members
   *
   */
  DHParameters6R() {}

  /**
   * @brief Constructor for passing specific joint DH parameters during instantiation
   *
   * @param joint1_i
   * @param joint2_i
   * @param joint3_i
   * @param joint4_i
   * @param joint5_i
   * @param joint6_i
   */
  DHParameters6R(JointParameter joint1_i, JointParameter joint2_i,
                 JointParameter joint3_i, JointParameter joint4_i,
                 JointParameter joint5_i, JointParameter joint6_i)
      : joint1{joint2_i}, joint2{joint2_i}, joint3{joint3_i}, joint4{joint4_i},
        joint5{joint5_i}, joint6{joint6_i} {}

  /**
   * @brief Constructor for passing a copy struct during instantiation
   *
   * @param dh_parameters_i
   */
  DHParameters6R(const DHParameters6R& dh_parameters_i)
      : joint1{dh_parameters_i.joint1}, joint2{dh_parameters_i.joint2},
        joint3{dh_parameters_i.joint3}, joint4{dh_parameters_i.joint4},
        joint5{dh_parameters_i.joint5}, joint6{dh_parameters_i.joint6} {}
};

/**
 * @brief Struct that describes the pose of an arm in joint space. Angles are in radians
 *
 */
struct ArmPose6R {
  float theta1 {0.0};
  float theta2 {0.0};
  float theta3 {0.0};
  float theta4 {0.0};
  float theta5 {0.0};
  float theta6 {0.0};

  /**
   * @brief Default constructor, zero initializes the members
   *
   */
  ArmPose6R() {}

  /**
   * @brief Constructor for passing specific joint angles during instantiation
   *
   * @param theta1_i
   * @param theta2_i
   * @param theta3_i
   * @param theta4_i
   * @param theta5_i
   * @param theta6_i
   */
  ArmPose6R(float theta1_i, float theta2_i, float theta3_i, float theta4_i,
            float theta5_i, float theta6_i)
    : theta1{theta1_i}, theta2{theta2_i}, theta3{theta3_i}, theta4{theta4_i},
      theta5{theta5_i}, theta6{theta6_i} {}

  /**
   * @brief Constructor for initializing the struct using a copy struct
   *
   * @param arm_pose
   */
  ArmPose6R(const ArmPose6R& arm_pose) : theta1{arm_pose.theta1},
      theta2{arm_pose.theta2}, theta3{arm_pose.theta3}, theta4{arm_pose.theta4},
      theta5{arm_pose.theta5}, theta6{arm_pose.theta6} {}

  bool operator==(const ArmPose6R& arm_pose_compare) {
    if (theta1 == arm_pose_compare.theta1 && theta2 == arm_pose_compare.theta2 &&
        theta3 == arm_pose_compare.theta3 && theta4 == arm_pose_compare.theta4 &&
        theta5 == arm_pose_compare.theta5 && theta6 == arm_pose_compare.theta6) {
          return true;
        }

    else
      return false;
  }
};

}

