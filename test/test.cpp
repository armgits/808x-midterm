#include <gtest/gtest.h>

#include "Forward.hpp"
#include "Inverse.hpp"

TEST(inverse_test, compute_function) {
  auto inverse_solver {std::make_unique<Inverse>()};

  auto result {inverse_solver->inverse({0.0}, {0.0})};
  std::vector<double> expected {1.0};
  EXPECT_EQ(result, expected);
}

TEST(forward_test, compute_function) {
  auto forward_solver {std::make_unique<Forward>()};

  auto result {forward_solver->forward({0.0}, {0.0})};
  std::vector<double> expected {1.0};

  EXPECT_EQ(result, expected);
}
