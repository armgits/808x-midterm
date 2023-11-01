/**
 * @file src.cpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @brief Source file for Forward kinematics library definitions
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#include <cmath>

#include "klib-datatypes.hpp"
#include "klib-inverse.hpp"

/**
 * @brief Constructs a new Inverse kinematics solver object by initializing
 *        DH Parameters and target coordinates, solution to zero.
 *
 * @param arm_description_
 */
klib::Inverse::Inverse(const klib::DHParameters6R arm_description_)
    : dh_parameters_{arm_description_}, tool_pose_{}, arm_pose_{} {}

/**
 * @brief Generates an A matrix using a transformation matrix template with
 *        DH Parameters in it
 *
 * @param theta
 * @param joint
 * @return Eigen::Matrix<double, 4, 4>
 */
Eigen::Matrix<double, 4, 4> klib::Inverse::GenerateAMatrix(double theta,
                                            const klib::JointParameter& joint) {
  Eigen::Matrix<double, 4, 4> a_matrix {
    {cos(theta), -cos(joint.alpha)*sin(theta),
        sin(joint.alpha)*sin(theta), joint.a_length*cos(theta)},

    {sin(theta), cos(theta)*cos(joint.alpha),
        -sin(joint.alpha)*cos(theta), joint.a_length*sin(theta)},

    {0, sin(joint.alpha), cos(joint.alpha), joint.d_length},

    {0, 0, 0, 1}
  };

  return a_matrix;
}

/**
 * @brief Jacobian is computed using method 1
 *
 * @param current_arm_pose
 * @return Eigen::Matrix<double, 6, 6>
 */
Eigen::Matrix<double, 6, 6> klib::Inverse::ComputeJacobian(
                                      const klib::ArmPose6R &current_arm_pose) {
  auto A1 {GenerateAMatrix(current_arm_pose.theta1, dh_parameters_.joint1)};
  auto A2 {GenerateAMatrix(current_arm_pose.theta2, dh_parameters_.joint2)};
  auto A3 {GenerateAMatrix(current_arm_pose.theta3, dh_parameters_.joint3)};
  auto A4 {GenerateAMatrix(current_arm_pose.theta4, dh_parameters_.joint4)};
  auto A5 {GenerateAMatrix(current_arm_pose.theta5, dh_parameters_.joint5)};
  auto A6 {GenerateAMatrix(current_arm_pose.theta6, dh_parameters_.joint6)};

  Eigen::Matrix<double, 4, 4> T1 {A1};
  Eigen::Matrix<double, 4, 4> T2 {T1*A2};
  Eigen::Matrix<double, 4, 4> T3 {T2*A3};
  Eigen::Matrix<double, 4, 4> T4 {T3*A4};
  Eigen::Matrix<double, 4, 4> T5 {T4*A5};
  Eigen::Matrix<double, 4, 4> T6 {T5*A6};

  Eigen::Matrix<double, 3, 1> O0 {0, 0, 0};
  Eigen::Matrix<double, 3, 1> O1 {T1(Eigen::seqN(0, 3), Eigen::last)};
  Eigen::Matrix<double, 3, 1> O2 {T2(Eigen::seqN(0, 3), Eigen::last)};
  Eigen::Matrix<double, 3, 1> O3 {T3(Eigen::seqN(0, 3), Eigen::last)};
  Eigen::Matrix<double, 3, 1> O4 {T4(Eigen::seqN(0, 3), Eigen::last)};
  Eigen::Matrix<double, 3, 1> O5 {T5(Eigen::seqN(0, 3), Eigen::last)};
  Eigen::Matrix<double, 3, 1> O6 {T6(Eigen::seqN(0, 3), Eigen::last)};

  tool_pose_.x = O6(0);
  tool_pose_.y = O6(1);
  tool_pose_.z = O6(2);

  Eigen::Matrix<double, 3, 1> Z0 {0, 0, 1};
  Eigen::Matrix<double, 3, 1> Z1 {T1(Eigen::seqN(0, 3), Eigen::last-1)};
  Eigen::Matrix<double, 3, 1> Z2 {T2(Eigen::seqN(0, 3), Eigen::last-1)};
  Eigen::Matrix<double, 3, 1> Z3 {T3(Eigen::seqN(0, 3), Eigen::last-1)};
  Eigen::Matrix<double, 3, 1> Z4 {T4(Eigen::seqN(0, 3), Eigen::last-1)};
  Eigen::Matrix<double, 3, 1> Z5 {T5(Eigen::seqN(0, 3), Eigen::last-1)};

  Eigen::Matrix<double, 6, 1> J1;
  J1 << Z0.cross(O6 - O0), Z0;

  Eigen::Matrix<double, 6, 1> J2;
  J2 << Z1.cross(O6 - O1), Z1;

  Eigen::Matrix<double, 6, 1> J3;
  J3 << Z2.cross(O6 - O2), Z2;

  Eigen::Matrix<double, 6, 1> J4;
  J4 << Z3.cross(O6 - O3), Z3;

  Eigen::Matrix<double, 6, 1> J5;
  J5 << Z4.cross(O6 - O4), Z4;

  Eigen::Matrix<double, 6, 1> J6;
  J6 << Z5.cross(O6 - O5), Z5;

  Eigen::Matrix<double, 6, 6> J;
  J << J1, J2, J3, J4, J5, J6;

  return J;
}

/**
 * @brief Computes an inverse kinematics solution for a given end-effector
 *        coordinate. Returns the joint angles required to achieve that position.
 *
 * @param target_coordinates
 * @param current_pose
 * @return klib::ArmPose6R
 */
klib::ArmPose6R klib::Inverse::Compute(const klib::Pose &target_tool_pose,
                                      const klib::ArmPose6R &current_arm_pose) {
  klib::ArmPose6R final_arm_pose {current_arm_pose};

  int i {0};
  while (i < 100) {
    auto J_inv {ComputeJacobian(final_arm_pose).transpose()};

    auto deltaToolPose {target_tool_pose - tool_pose_};

    Eigen::Matrix<double, 6, 1> deltaToolPoseVector;
    deltaToolPoseVector << deltaToolPose.x, deltaToolPose.y, deltaToolPose.z, 0,
                           0, 0;

    Eigen::Matrix<double, 6, 1> deltaArmPose {J_inv * deltaToolPoseVector};

    final_arm_pose.theta1 += deltaArmPose(0);
    final_arm_pose.theta2 += deltaArmPose(1);
    final_arm_pose.theta3 += deltaArmPose(2);
    final_arm_pose.theta4 += deltaArmPose(3);
    final_arm_pose.theta5 += deltaArmPose(4);
    final_arm_pose.theta6 += deltaArmPose(5);

    i++;
  }

  return final_arm_pose;
}
