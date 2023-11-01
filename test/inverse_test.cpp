/**
 * @file inverse_test.cpp
 * @author Mudit Singal (msingal@umd.edu)
 * @brief Test cases for inverse kinematics class
 * @version 0.1
 * @date 2023-10-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <gtest/gtest.h>

#include "klib-datatypes.hpp"
#include "klib-inverse.hpp"

/**
 * @brief Test case 1 for inverse compute function. Target coordinate (1, 1, 1)
 *
 */
TEST(inverse_test, compute_method_test1) {
  klib::JointParameter joint1_param {0, 0, 0.15185, M_PI_2, 0, 2*M_PI};
  klib::JointParameter joint2_param {0, -0.24355, 0, 0, 0, 2*M_PI};
  klib::JointParameter joint3_param {0, -0.24355, 0, 0, 0, 2*M_PI};
  klib::JointParameter joint4_param {0, 0, 0.13105, M_PI_2, 0, 2*M_PI};
  klib::JointParameter joint5_param {0, 0, 0.8535, -M_PI_2, 0, 2*M_PI};
  klib::JointParameter joint6_param {0, 0, 0.0921, 0, 0, 2*M_PI};

  klib::DHParameters6R test_arm {
      joint1_param,
      joint2_param,
      joint3_param,
      joint4_param,
      joint5_param,
      joint6_param
  };

  auto test_inverse_solver {std::make_shared<klib::Inverse>(test_arm)};

  klib::Pose target_pose {1, 1, 1, 0, 0, 0};
  klib::ArmPose6R arm_pose {0, -1.5708, 1.5708, -1.5708, 1.5708, -0.0174533};

  auto result_arm_pose {test_inverse_solver->Compute(target_pose, arm_pose)};
  klib::ArmPose6R expected_arm_pose {-1.53, -5.44, 0.699, -3.61, -0.10, -0.01};

  std::cout << "Result arm pose: " << std::endl;
  std::cout << "  Joint1: " << result_arm_pose.theta1 << std::endl;
  std::cout << "  Joint2: " << result_arm_pose.theta2 << std::endl;
  std::cout << "  Joint3: " << result_arm_pose.theta3 << std::endl;
  std::cout << "  Joint4: " << result_arm_pose.theta4 << std::endl;
  std::cout << "  Joint5: " << result_arm_pose.theta5 << std::endl;
  std::cout << "  Joint6: " << result_arm_pose.theta6 << std::endl;

  ASSERT_NEAR(result_arm_pose.theta1, expected_arm_pose.theta1, 0.1);
  ASSERT_NEAR(result_arm_pose.theta2, expected_arm_pose.theta2, 0.1);
  ASSERT_NEAR(result_arm_pose.theta3, expected_arm_pose.theta3, 0.1);
  ASSERT_NEAR(result_arm_pose.theta4, expected_arm_pose.theta4, 0.1);
  ASSERT_NEAR(result_arm_pose.theta5, expected_arm_pose.theta5, 0.1);
  ASSERT_NEAR(result_arm_pose.theta6, expected_arm_pose.theta6, 0.1);
}

/**
 * @brief Test case 2 for inverse compute function. Target coordinate (0.5, 0.5, 0.5)
 *
 */
TEST(inverse_test, compute_method_test2) {
  klib::JointParameter joint1_param {0, 0, 0.15185, M_PI_2, 0, 2*M_PI};
  klib::JointParameter joint2_param {0, -0.24355, 0, 0, 0, 2*M_PI};
  klib::JointParameter joint3_param {0, -0.24355, 0, 0, 0, 2*M_PI};
  klib::JointParameter joint4_param {0, 0, 0.13105, M_PI_2, 0, 2*M_PI};
  klib::JointParameter joint5_param {0, 0, 0.8535, -M_PI_2, 0, 2*M_PI};
  klib::JointParameter joint6_param {0, 0, 0.0921, 0, 0, 2*M_PI};

  klib::DHParameters6R test_arm {
      joint1_param,
      joint2_param,
      joint3_param,
      joint4_param,
      joint5_param,
      joint6_param
  };

  auto test_inverse_solver {std::make_shared<klib::Inverse>(test_arm)};

  klib::Pose target_pose {0.5, 0.5, 0.5, 0, 0, 0};
  klib::ArmPose6R arm_pose {0, -1.5708, 1.5708, -1.5708, 1.5708, -0.0174533};

  auto result_arm_pose {test_inverse_solver->Compute(target_pose, arm_pose)};
  klib::ArmPose6R expected_arm_pose {-0.815, -2.267, 2.812, -4.717, 0.159, -0.01};

  std::cout << "Result arm pose: " << std::endl;
  std::cout << "  Joint1: " << result_arm_pose.theta1 << std::endl;
  std::cout << "  Joint2: " << result_arm_pose.theta2 << std::endl;
  std::cout << "  Joint3: " << result_arm_pose.theta3 << std::endl;
  std::cout << "  Joint4: " << result_arm_pose.theta4 << std::endl;
  std::cout << "  Joint5: " << result_arm_pose.theta5 << std::endl;
  std::cout << "  Joint6: " << result_arm_pose.theta6 << std::endl;

  ASSERT_NEAR(result_arm_pose.theta1, expected_arm_pose.theta1, 0.1);
  ASSERT_NEAR(result_arm_pose.theta2, expected_arm_pose.theta2, 0.1);
  ASSERT_NEAR(result_arm_pose.theta3, expected_arm_pose.theta3, 0.1);
  ASSERT_NEAR(result_arm_pose.theta4, expected_arm_pose.theta4, 0.1);
  ASSERT_NEAR(result_arm_pose.theta5, expected_arm_pose.theta5, 0.1);
  ASSERT_NEAR(result_arm_pose.theta6, expected_arm_pose.theta6, 0.1);
}
