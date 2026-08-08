// test_gauss_newton.cpp
//
// Checks the generic damped Gauss-Newton solver (Solver.hpp) against a toy
// problem with a known closed-form minimum, exercising the finite-difference
// Jacobian fallback path (no analytic jacobian argument is supplied here --
// the scenario-specific solves in Estimators.hpp/Simulation.cpp are what
// exercise the analytic-Jacobian path, covered separately by
// test_jacobian.cpp).

#include "adaptive_localization/Solver.hpp"
#include "test_framework.hpp"

using namespace adaptive;

ADAPTIVE_TEST(gauss_newton_converges_on_toy_problem) {
    // r0(x, y) = x^2 - 4  (root at x = +-2; x0 = 1.5 > 0 converges to +2)
    // r1(x, y) = y - 3    (root at y = 3)
    // Unique minimum at (2, 3) with zero residual/cost.
    const ResidualFunction residuals = [](const std::vector<double>& state) {
        return std::vector<double>{state[0] * state[0] - 4.0, state[1] - 3.0};
    };
    const std::vector<double> x0 = {1.5, 0.0};

    const SolverResult result = gauss_newton(x0, residuals, /*max_iterations=*/80);

    CHECK(result.converged);
    CHECK(result.x.size() == 2);
    EXPECT_NEAR(result.x[0], 2.0, 1e-4);
    EXPECT_NEAR(result.x[1], 3.0, 1e-4);
    EXPECT_NEAR(result.cost, 0.0, 1e-6);
}
