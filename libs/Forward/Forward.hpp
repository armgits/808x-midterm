/**
 * @file Forward.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @brief Header file for Forward kinematics library declarations
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#pragma once

#include <vector>

class Forward {

 public:

 private:
  std::vector<double> input_angles_;
  std::vector<double> output_angles_;
  std::vector<double> robot_tcp_position_;
  std::vector<double> coordinate_constraints_;
  double **dh_params_;
};
