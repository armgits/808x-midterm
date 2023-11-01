/**
 * @file datatypes.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @author Mudit Singal (msingal@umd.edu)
 * @brief Header-only library for custom datatypes for kinematics and planning
 * @version 0.2
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#pragma once

#include <cmath>

namespace klib {

struct Pose {
  float x {0.0};
  float y {0.0};
  float z {0.0};
  float wx {0.0};
  float wy {0.0};
  float wz {0.0};

  Pose() {};

  constexpr Pose(float x_i, float y_i, float z_i, float wx_i, float wy_i,
                 float wz_i) {
    x = x_i;
    y = y_i;
    z = z_i;
    wx = wx_i;
    wy = wy_i;
    wz = wz_i;
  }

  bool operator>(const Pose& pose_compare) {
    if (x > pose_compare.x &&
        y > pose_compare.y &&
        z > pose_compare.z &&
        wx > pose_compare.wx &&
        wy > pose_compare.wy &&
        wz > pose_compare.wz) {
      return true;
    }

    else {
      return false;
    }
  }

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

struct JointParameter {
  float offset {0.0};
  float a_length {0.0};
  float d_length {0.0};
  float alpha {0.0};
  float min_limit {0.0};
  float max_limit {0.0};

  JointParameter() {}

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

struct DHParameters6R {
  JointParameter joint1;
  JointParameter joint2;
  JointParameter joint3;
  JointParameter joint4;
  JointParameter joint5;
  JointParameter joint6;

  DHParameters6R() {}

  DHParameters6R(
    JointParameter joint1_i,
    JointParameter joint2_i,
    JointParameter joint3_i,
    JointParameter joint4_i,
    JointParameter joint5_i,
    JointParameter joint6_i
  )
  : joint1{joint2_i},
    joint2{joint2_i},
    joint3{joint3_i},
    joint4{joint4_i},
    joint5{joint5_i},
    joint6{joint6_i} {}
};

struct ArmPose6R {
  float theta1 {0.0};
  float theta2 {0.0};
  float theta3 {0.0};
  float theta4 {0.0};
  float theta5 {0.0};
  float theta6 {0.0};

  ArmPose6R() {}

  ArmPose6R(
    float theta1_i,
    float theta2_i,
    float theta3_i,
    float theta4_i,
    float theta5_i,
    float theta6_i
  )
  : theta1{theta1_i},
    theta2{theta2_i},
    theta3{theta3_i},
    theta4{theta4_i},
    theta5{theta5_i},
    theta6{theta6_i} {}

  ArmPose6R(const ArmPose6R& arm_pose)
  : theta1{arm_pose.theta1},
    theta2{arm_pose.theta2},
    theta3{arm_pose.theta3},
    theta4{arm_pose.theta4},
    theta5{arm_pose.theta5},
    theta6{arm_pose.theta6} {}
};

}

