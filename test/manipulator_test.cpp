/**
 * @file manipulator_test.cpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @brief Test cases for the Manipulator class
 * @version 1.0
 * @date 2023-11-01
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#include "gtest/gtest.h"
#include "klib-manipulator.hpp"

TEST(ManipulatorTest, SetTcpPosition) {
    Manipulator manipulator;

    std::vector<double> tcpPosition = {1.0, 2.0, 3.0, 0.1, 0.2, 0.3};

    bool result = manipulator.set_tcp_position(tcpPosition);

    ASSERT_TRUE(result);
}

TEST(ManipulatorTest, SetJointAngles) {
    Manipulator manipulator;

    std::vector<double> angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    bool result = manipulator.set_joint_angles(angles);

    ASSERT_TRUE(result);
}
