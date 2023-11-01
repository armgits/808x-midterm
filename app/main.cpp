/**
 * @file main.cpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @author Mudit Singal (msingal@umd.edu)
 * @author Abhimanyu Saxena (asaxena4@umd.edu)
 * @brief Main function for functional demonstration of klib library
 * @version 1.0
 * @date 2023-10-26
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */
#include <memory>
#include <cmath>

#include "klib-datatypes.hpp"
#include "klib-manipulator.hpp"
#include "klib-forward.hpp"
#include "klib-inverse.hpp"

int main() {
  auto manipulator {std::make_unique<Manipulator>()};
  double dh_parameters[6][4] = {{0.1625, 0.0, 0.5 * M_PI, 0.0},  {0.0, -0.425, 0.0, 0.0},
                    {0.0, -0.3922, 0.0, 0.0},      {0.1333, 0.0, 0.5 * M_PI, 0.0},
                    {0.0997, 0.0, -0.5 * M_PI, 0.0}, {0.0996, 0.0, 0.0, 0.0}};

  manipulator->set_dh_params(dh_parameters);

  auto forward_solver {std::make_unique<Forward>()};

  std::vector<double> arm_pose {M_PI_2, -M_PI_2, -M_PI_2, 0, -M_PI_2, 0};
  klib::ArmPose6R input_arm_pose {M_PI_2, -M_PI_2, -M_PI_2, 0, -M_PI_2, 0};

  std::cout << "Input Joint Angles: " << std::endl;
  std::cout << "  Joint 1: " << input_arm_pose.theta1 << std::endl;
  std::cout << "  Joint 2: " << input_arm_pose.theta2 << std::endl;
  std::cout << "  Joint 3: " << input_arm_pose.theta3 << std::endl;
  std::cout << "  Joint 4: " << input_arm_pose.theta4 << std::endl;
  std::cout << "  Joint 5: " << input_arm_pose.theta5 << std::endl;
  std::cout << "  Joint 6: " << input_arm_pose.theta6 << std::endl;
  std::cout << std::endl;

  auto tcp_pose {forward_solver->forward(arm_pose, *manipulator)};

  std::cout << "Output TCP from Forward solver: " << std::endl;
  std::cout << "  x: " << tcp_pose.x << std::endl;
  std::cout << "  y: " << tcp_pose.y << std::endl;
  std::cout << "  z: " << tcp_pose.z << std::endl;
  std::cout << "  wx: " << tcp_pose.wx << std::endl;
  std::cout << "  wy: " << tcp_pose.wy << std::endl;
  std::cout << "  wz: " << tcp_pose.wz << std::endl;
  std::cout << std::endl;

  auto dh_parameters_struct {manipulator->get_dh_params()};
  auto inverse_solver {std::make_unique<klib::Inverse>(dh_parameters_struct)};

  std::cout << "Plugging the above result into Inverse solver..." << std::endl;
  std::cout << std::endl;

  auto result_arm_pose {inverse_solver->Compute(tcp_pose, input_arm_pose)};

  std::cout << "Output Joint Angles: " << std::endl;
  std::cout << "  Joint 1: " << result_arm_pose.theta1 << std::endl;
  std::cout << "  Joint 2: " << result_arm_pose.theta2 << std::endl;
  std::cout << "  Joint 3: " << result_arm_pose.theta3 << std::endl;
  std::cout << "  Joint 4: " << result_arm_pose.theta4 << std::endl;
  std::cout << "  Joint 5: " << result_arm_pose.theta5 << std::endl;
  std::cout << "  Joint 6: " << result_arm_pose.theta6 << std::endl;
  std::cout << std::endl;

  (result_arm_pose == input_arm_pose) ?
      std::cout << "Output joint angles are equal to input joint angles"
    : std::cout << "Output joint angles are not equal to input joint angles";

  std::cout << std::endl;

  return 0;
}
