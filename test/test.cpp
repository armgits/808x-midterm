/**
 * @file test.cpp
 * @author Abhimanyu Saxena (asaxena4@umd.edu)
 * @brief Adding dummy test cases for Sprint 1
 * @version 0.1
 * @date 2023-10-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <gtest/gtest.h>

#include "Forward.hpp"
#include "Inverse.hpp"

/**
 * @brief Dummy test case for inverse method
 *
 */
TEST(inverse_test, compute_function) {
  auto inverse_solver{std::make_unique<Inverse>()};

  auto result{inverse_solver->inverse({0.0}, {0.0})};
  std::vector<double> expected{1.0};
  EXPECT_EQ(result, expected);
}

/**
 * @brief Dummy test case for forward method
 *
 */
TEST(forward_test, compute_function) {
  auto forward_solver{std::make_unique<Forward>()};

  auto result{forward_solver->forward({0.0}, {0.0})};
  std::vector<double> expected{1.0};

  EXPECT_EQ(result, expected);
}
