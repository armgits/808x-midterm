/**
 * @file datatypes.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @brief Header-only library for custom datatypes for kinematics and planning
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#pragma once

namespace kplib {

struct Point {
  float x {0.0};
  float y {0.0};
  float z {0.0};
};

struct Joint {
  float offset {0.0};
  float a_length {0.0};
  float d_length {0.0};
  float alpha {0.0};
  float min_limit {0.0};
  float max_limit {0.0};
};

struct DHParameters6R {
  Joint joint1;
  Joint joint2;
  Joint joint3;
  Joint joint4;
  Joint joint5;
  Joint joint6;
};

struct ArmPose6R {
  float joint1 {0.0};
  float joint2 {0.0};
  float joint3 {0.0};
  float joint4 {0.0};
  float joint5 {0.0};
  float joint6 {0.0};
};

}

