/**
 * @file test.cpp
 * @author Abhimanyu Saxena (asaxena4@umd.edu)
 * @brief Adding dummy test cases for Sprint 1
 * @version 0.1
 * @date 2023-10-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <gtest/gtest.h>

#include "klib-datatypes.hpp"
#include "klib-forward.hpp"
#include "klib-manipulator.hpp"

const double pi = 3.141592;
const double epsilon = 0.1;
double dhp[6][4] = {{0.1625, 0.0, 0.5 * pi, 0.0},  {0.0, -0.425, 0.0, 0.0},
                    {0.0, -0.3922, 0.0, 0.0},      {0.1333, 0.0, 0.5 * pi, 0.0},
                    {0.0997, 0.0, -0.5 * pi, 0.0}, {0.0996, 0.0, 0.0, 0.0}};

/**
 * @brief Test case 1 for Forward method
 *
 */
TEST(forward_kinematics_test_1, should_return_tcp_pose_1) {
  // Create an instance of manipulator class
  Manipulator manipulator;

  manipulator.set_dh_params(dhp);

  Forward f1;

  // Define the expected end effector pose for a set of known joint angles
  std::vector<double> joint_angles = {0.0,     -pi / 2, -pi / 2,
                                      -pi / 2, pi / 2,  0.0012};
  klib::Pose expected_pose{0.4919, -0.1333, 0.4879, 3.14159, 0.0, 1.572};

  // Call the forward kinematics method to get the actual end effector pose
  klib::Pose tcp_pose = f1.forward(joint_angles, manipulator);

  // Check if the actual pose is equal to the expected pose within a tolerance
  EXPECT_NEAR(tcp_pose.x, expected_pose.x, epsilon);
  EXPECT_NEAR(tcp_pose.y, expected_pose.y, epsilon);
  EXPECT_NEAR(tcp_pose.z, expected_pose.z, epsilon);
  EXPECT_NEAR(tcp_pose.wx, expected_pose.wx, epsilon);
  EXPECT_NEAR(tcp_pose.wy, expected_pose.wy, epsilon);
  EXPECT_NEAR(tcp_pose.wz, expected_pose.wz, epsilon);
}

/**
 * @brief Test case 2 for Forward method
 *
 */
TEST(forward_kinematics_test_2, should_return_tcp_pose_2) {
  // Create an instance of manipulator class
  Manipulator manipulator;

  manipulator.set_dh_params(dhp);

  Forward f1;

  // Define the expected end effector pose for a set of known joint angles
  std::vector<double> joint_angles = {pi / 2,  -pi / 2, -pi / 2,
                                      -pi / 2, pi / 2,  0.0012};
  klib::Pose expected_pose{0.1333, 0.4919, 0.4879, 0.0, 3.14159, -3.14039};

  // Call the forward kinematics method to get the actual end effector pose
  klib::Pose tcp_pose = f1.forward(joint_angles, manipulator);

  // Check if the actual pose is equal to the expected pose within a tolerance
  EXPECT_NEAR(tcp_pose.x, expected_pose.x, epsilon);
  EXPECT_NEAR(tcp_pose.y, expected_pose.y, epsilon);
  EXPECT_NEAR(tcp_pose.z, expected_pose.z, epsilon);
  EXPECT_NEAR(tcp_pose.wx, expected_pose.wx, epsilon);
  EXPECT_NEAR(tcp_pose.wy, expected_pose.wy, epsilon);
  EXPECT_NEAR(tcp_pose.wz, expected_pose.wz, epsilon);
}
