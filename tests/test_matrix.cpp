// test_matrix.cpp
//
// Unit tests for Matrix.hpp's solve_dense_linear_system: correctness on a
// well-conditioned system, and the documented behavior of each
// SingularPivotPolicy branch when the system is exactly singular. This is
// the solver shared by the Gauss-Newton normal equations (Solver.cpp) and
// the EKF/covariance solves (Simulation.cpp) -- see the code-quality note in
// README.md/the repo history on why these two call sites used to be
// independent, divergent hand-rolled solvers.

#include "adaptive_localization/Matrix.hpp"
#include "test_framework.hpp"

#include <cmath>

using namespace adaptive;

ADAPTIVE_TEST(solve_dense_linear_system_well_conditioned) {
    // 2x + y = 5, x + 3y = 10  =>  x = 1, y = 3.
    const Matrix a = {{2.0, 1.0}, {1.0, 3.0}};
    const std::vector<double> b = {5.0, 10.0};

    const auto x = solve_dense_linear_system(a, b, SingularPivotPolicy::RegularizeDiagonal);

    CHECK(x.size() == 2);
    EXPECT_NEAR(x[0], 1.0, 1e-9);
    EXPECT_NEAR(x[1], 3.0, 1e-9);
}

ADAPTIVE_TEST(solve_dense_linear_system_singular_return_zero) {
    // Row 2 is exactly 2x row 1, so this system is singular regardless of b.
    const Matrix a = {{1.0, 2.0}, {2.0, 4.0}};
    const std::vector<double> b = {3.0, 6.0};

    const auto x = solve_dense_linear_system(a, b, SingularPivotPolicy::ReturnZero);

    CHECK(x.size() == 2);
    EXPECT_NEAR(x[0], 0.0, 1e-12);
    EXPECT_NEAR(x[1], 0.0, 1e-12);
}

ADAPTIVE_TEST(solve_dense_linear_system_singular_regularize_diagonal) {
    const Matrix a = {{1.0, 2.0}, {2.0, 4.0}};
    const std::vector<double> b = {3.0, 6.0};

    const auto x = solve_dense_linear_system(a, b, SingularPivotPolicy::RegularizeDiagonal);

    // RegularizeDiagonal nudges the singular pivot by a small jitter instead
    // of returning early, so (unlike ReturnZero) it always produces a
    // finite, determinate result -- the exact value is an artifact of the
    // jitter rather than a meaningful answer, so we only check it didn't
    // propagate NaN/inf.
    CHECK(x.size() == 2);
    CHECK(std::isfinite(x[0]));
    CHECK(std::isfinite(x[1]));
}
