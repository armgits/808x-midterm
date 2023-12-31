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

#include <list>

#include "klib-forward.hpp"

/**
 * @brief Constructor for forward kinematics class. Initializes attributes to
 * zero.
 *
 */
Forward::Forward() {}

Eigen::Matrix4d compute_DH_matrix(double a, double alpha, double d,
                                  double theta) {
  Eigen::Matrix4d dh_matrix;
  dh_matrix << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),
      a * cos(theta), sin(theta), cos(theta) * cos(alpha),
      -cos(theta) * sin(alpha), a * sin(theta), 0, sin(alpha), cos(alpha), d, 0,
      0, 0, 1;  // cppcheck-suppress constStatement
  return dh_matrix;
}

/**
 * @brief Method to compute the forward kinematics of the arm for given joint
 * angles. Implemented using custom datatypes of manipulator pose and Eigen
 * library.
 *
 * @param joint_angles
 * @return klib::Pose
 */
klib::Pose Forward::forward(const std::vector<double>& joint_angles,
                            Manipulator m_obj) {
  // Instantiating Manipulator object, DH parameters object, and dh_matrix
  // object for use in the function
  klib::DHParameters6R m1_dh_params;
  Eigen::Matrix4d dh_matrix;

  // The DH parameters initialization as variables for ease in code
  // understanding
  float a1, d1, alpha1, theta1;
  float a2, d2, alpha2, theta2;
  float a3, d3, alpha3, theta3;
  float a4, d4, alpha4, theta4;
  float a5, d5, alpha5, theta5;
  float a6, d6, alpha6, theta6;

  // Pose of the tooltip as custom datatype klib::Pose
  klib::Pose fwd_tcp_pose;

  // Checking joint angle size. Returns an error if the number of joint angles
  // is not equal to 6 for 6-DOF manipulator.
  if (joint_angles.size() != 6) {
    std::cerr << "Invalid number of joint angles. Expected 6 angles."
              << std::endl;
    return klib::Pose();
  }

  // Initializing end effector pose as 4x4 identity matrix
  Eigen::Matrix4d tcp_transform = Eigen::Matrix4d::Identity();

  // Fetching the DH parameters from manipulator
  m1_dh_params = m_obj.get_dh_params();

  // Extract joint 1 DH parameters
  d1 = m1_dh_params.joint1.d_length;
  a1 = m1_dh_params.joint1.a_length;
  alpha1 = m1_dh_params.joint1.alpha;
  theta1 = m1_dh_params.joint1.offset + joint_angles[0];

  // Compute the DH matrix and find the end effector transformation matrix after
  // application of joint actuation and DH matrix
  dh_matrix = compute_DH_matrix(a1, alpha1, d1, theta1);
  tcp_transform = tcp_transform * dh_matrix;

  // Extract joint 2 DH parameters
  d2 = m1_dh_params.joint2.d_length;
  a2 = m1_dh_params.joint2.a_length;
  alpha2 = m1_dh_params.joint2.alpha;
  theta2 = m1_dh_params.joint2.offset + joint_angles[1];

  dh_matrix = compute_DH_matrix(a2, alpha2, d2, theta2);
  tcp_transform = tcp_transform * dh_matrix;

  // Extract joint 3 DH parameters
  d3 = m1_dh_params.joint3.d_length;
  a3 = m1_dh_params.joint3.a_length;
  alpha3 = m1_dh_params.joint3.alpha;
  theta3 = m1_dh_params.joint3.offset + joint_angles[2];

  dh_matrix = compute_DH_matrix(a3, alpha3, d3, theta3);
  tcp_transform = tcp_transform * dh_matrix;

  // Extract joint 4 DH parameters
  d4 = m1_dh_params.joint4.d_length;
  a4 = m1_dh_params.joint4.a_length;
  alpha4 = m1_dh_params.joint4.alpha;
  theta4 = m1_dh_params.joint4.offset + joint_angles[3];

  dh_matrix = compute_DH_matrix(a4, alpha4, d4, theta4);
  tcp_transform = tcp_transform * dh_matrix;

  // Extract joint 5 DH parameters
  d5 = m1_dh_params.joint5.d_length;
  a5 = m1_dh_params.joint5.a_length;
  alpha5 = m1_dh_params.joint5.alpha;
  theta5 = m1_dh_params.joint5.offset + joint_angles[4];

  dh_matrix = compute_DH_matrix(a5, alpha5, d5, theta5);
  tcp_transform = tcp_transform * dh_matrix;

  // Extract joint 6 DH parameters
  d6 = m1_dh_params.joint6.d_length;
  a6 = m1_dh_params.joint6.a_length;
  alpha6 = m1_dh_params.joint6.alpha;
  theta6 = m1_dh_params.joint6.offset + joint_angles[5];

  // Compute the final end effector transformation matrix after application of
  // joint actuation and DH matrix
  dh_matrix = compute_DH_matrix(a6, alpha6, d6, theta6);
  tcp_transform = tcp_transform * dh_matrix;

  // Get the tool tip position and orientation from the calculated
  // transformation matrix
  Eigen::Vector3d fwd_tcp_position = tcp_transform.block<3, 1>(0, 3);
  Eigen::Quaterniond fwd_tcp_orientation(tcp_transform.block<3, 3>(0, 0));

  // Convert orientation to Euler angles
  Eigen::Vector3d fwd_tcp_euler_angles =
      fwd_tcp_orientation.toRotationMatrix().eulerAngles(0, 1, 2);

  // Assign the tool tip pose from the computed end effector position and euler
  // angles and return it
  fwd_tcp_pose.x = fwd_tcp_position[0];
  fwd_tcp_pose.y = fwd_tcp_position[1];
  fwd_tcp_pose.z = fwd_tcp_position[2];

  fwd_tcp_pose.wx = fwd_tcp_euler_angles[0];
  fwd_tcp_pose.wy = fwd_tcp_euler_angles[1];
  fwd_tcp_pose.wz = fwd_tcp_euler_angles[2];

  return fwd_tcp_pose;
}

// int main()
// {
//   Forward mani_f1;
//   Manipulator m1;

//   double pi = 3.14159265359;
//   double dh_param_UR5e[6][4] = {
//     {0.1625, 0.0, 0.5 * pi, 0.0},  {0.0, -0.425, 0.0, 0.0},
//     {0.0, -0.3922, 0.0, 0.0},      {0.1333, 0.0, 0.5 * pi, 0.0},
//     {0.0997, 0.0, -0.5 * pi, 0.0}, {0.0996, 0.0, 0.0, 0.0}};

//   std::vector<double> demo_joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//   m1.set_dh_params(dh_param_UR5e);

//   mani_f1.forward(demo_joint_angles);

//   return 0;
// }
