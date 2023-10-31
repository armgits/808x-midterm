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
};

struct DHParameters6R {
  JointParameter joint1;
  JointParameter joint2;
  JointParameter joint3;
  JointParameter joint4;
  JointParameter joint5;
  JointParameter joint6;
};

struct ArmPose6R {
  float theta1 {0.0};
  float theta2 {0.0};
  float theta3 {0.0};
  float theta4 {0.0};
  float theta5 {0.0};
  float theta6 {0.0};
};

}

