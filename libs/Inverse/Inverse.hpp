/**
 * @file Inverse.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @brief Header file for Inverse kinematics library declarations
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#pragma once

#include <vector>

class Inverse {

 public:

 private:
  std::vector<double> input_coordinates_;
  std::vector<double> output_coordinates_;
  std::vector<double> robot_pose_;
  std::vector<double> angle_constraints_;
  double **dh_params_;
};
