/**
 * @file src.cpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @author Mudit Singal (msingal@umd.edu)
 * @brief Source file for Forward kinematics library definitions
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#include "klib-forward.hpp"
#include <list>

/**
 * @brief Constructor for forward kinematics class. Initializes attributes to zero.
 *
 */
Forward::Forward() : input_angles_{}, output_angles_{}, robot_tcp_pose_{},
                     coordinate_constraints_{}, dh_params_{} {}

/**
 * @brief Method to compute the forward kinematics of the arm for given joint angles.
 *        Currently returns a dummy vector. Implementation pending.
 *
 * @param joint_angles
 * @param tcp_position
 * @return std::vector<double>
 */
std::vector<double> Forward::forward(
    const std::vector<double>& joint_angles,
    const std::vector<double>& tcp_position) {


  std::vector<double> dummy_vector {1.0};
  return dummy_vector;
}

/**
 * @brief Obtain the constraints set for the joint angles
 *
 * @return std::vector<double>
 */
std::vector<Forward::angle_constraint_> Forward::get_angle_constraint()
{
  return angle_constraints_;
}

void Forward::set_angle_constraint(const std::vector<Forward::angle_constraint_> &angle_constraint)
{
  // check validity of each joint angle constraint
  for(int i = 0; i < angle_constraint.size(); i++)
  {
    if(angle_constraint[i].min_angle_ > angle_constraint[i].max_angle_)
    {
      std::cout<< "Angle constraint invalid. Min angle should be less than max angle" << std::endl;
      return;
    }
  }
  angle_constraints_ = angle_constraint;
  std::cout<< "Angle constraints set successfully!" << std::endl;
}
