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
const double epsilon = 0.5;

/**
 * @brief Test case 1 for Forward method
 *
 */
TEST(forward_kinematics_test_1, should_return_tcp_pose_1) {
  // Create an instance of manipulator class
  Manipulator manipulator;
  Forward f1;

  // Define the expected end effector pose for a set of known joint angles
  std::vector<double> joint_angles = {0.0,     -pi / 2, -pi / 2,
                                      -pi / 2, pi / 2,  0.0012};
  klib::Pose expected_pose{0.49309, -0.13026, 0.48946, 0.05, 0.05, 0.012};

  // Call the forward kinematics method to get the actual end effector pose
  klib::Pose tcp_pose = f1.forward(joint_angles);

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
  Forward f1;

  // Define the expected end effector pose for a set of known joint angles
  std::vector<double> joint_angles = {pi / 2,  -pi / 2, -pi / 2,
                                      -pi / 2, pi / 2,  0.0012};
  klib::Pose expected_pose{0.13150, 0.492, 0.48944, 0.05, -0.002, 0.015};

  // Call the forward kinematics method to get the actual end effector pose
  klib::Pose tcp_pose = f1.forward(joint_angles);

  // Check if the actual pose is equal to the expected pose within a tolerance
  EXPECT_NEAR(tcp_pose.x, expected_pose.x, epsilon);
  EXPECT_NEAR(tcp_pose.y, expected_pose.y, epsilon);
  EXPECT_NEAR(tcp_pose.z, expected_pose.z, epsilon);
  EXPECT_NEAR(tcp_pose.wx, expected_pose.wx, epsilon);
  EXPECT_NEAR(tcp_pose.wy, expected_pose.wy, epsilon);
  EXPECT_NEAR(tcp_pose.wz, expected_pose.wz, epsilon);
}