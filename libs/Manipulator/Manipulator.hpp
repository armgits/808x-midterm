/**
 * @file Manipulator.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @brief Header file for Manipulator library declarations
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 Group 12
 *
 */

#pragma once

/**
 * @brief Class for describing a manipulator
 *
 */
class Manipulator {

 public:
  /**
   * @brief Constructs a new Manipulator object
   *
   */
  Manipulator(int dof);

  /**
   * @brief Sets the DH Parameters that describes the arm
   *
   * @param params
   */
  void set_dh_params(double **params);

 private:
  /**
   * @brief Stores the degrees of freedom of arm
   *
   */
  int dof_;

  /**
   * @brief Stores the DH Parameters of the arm
   *
   */
  double **dh_params_;
};
