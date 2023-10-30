/**
 * @file datatypes.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @author Mudit Singal (msingal@umd.edu)
 * @brief Header-only library for custom datatypes for kinematics and planning
 * @version 0.1
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
  JointParameter joint1_parameter;
  JointParameter joint2_parameter;
  JointParameter joint3_parameter;
  JointParameter joint4_parameter;
  JointParameter joint5_parameter;
  JointParameter joint6_parameter;
};

struct ArmPose6R {
  float joint1_angle {0.0};
  float joint2_angle {0.0};
  float joint3_angle {0.0};
  float joint4_angle {0.0};
  float joint5_angle {0.0};
  float joint6_angle {0.0};
};

}

