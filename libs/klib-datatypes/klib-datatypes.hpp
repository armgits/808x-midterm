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

namespace klib {

struct Point {
  float x {0.0};
  float y {0.0};
  float z {0.0};
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

