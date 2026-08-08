// Solver.hpp
//
// Generic damped Gauss-Newton (Levenberg-Marquardt-style) nonlinear
// least-squares solver, independent of any particular measurement model. All
// of the scenario-specific residual/Jacobian functions in Estimators.hpp and
// Simulation.cpp are minimized by calling gauss_newton() below.

#pragma once

#include <functional>
#include <vector>

#include "adaptive_localization/Types.hpp"

namespace adaptive {

/// A function mapping a state vector to its (whitened) residual vector.
using ResidualFunction = std::function<std::vector<double>(const std::vector<double>&)>;
/// A function mapping a state vector to its residual Jacobian (rows =
/// residual index, columns = state index). If omitted when calling
/// gauss_newton, the Jacobian is instead approximated by forward finite
/// differences.
using JacobianFunction =
    std::function<std::vector<std::vector<double>>(const std::vector<double>&)>;

/**
 * Minimizes 0.5 * sum(residuals(x)^2) starting from `x0` using a damped
 * Gauss-Newton (Levenberg-Marquardt style) iteration.
 *
 * At each iteration the normal equations (J^T J + lambda*I) * delta = -J^T r
 * are formed and solved, where `lambda` is the Levenberg damping term: it is
 * shrunk after any step that reduces cost (allowing faster, more
 * Newton-like convergence) and grown after a rejected step (falling back
 * toward a smaller, more gradient-descent-like, better-conditioned step).
 * The damping term also keeps the normal equations solvable even when J^T J
 * is ill-conditioned or rank-deficient (e.g. under weak observability /
 * near-degenerate excitation).
 *
 * @param x0 initial state guess (see Estimators.hpp for scenario-specific
 *     initializers, e.g. initial_state_scenario1 or the closed-form
 *     two_view_closed_form_initial_state).
 * @param residuals whitened residual function (see ResidualFunction).
 * @param max_iterations maximum number of Gauss-Newton iterations to run.
 * @param initial_lambda starting value of the Levenberg damping term.
 * @param jacobian optional analytic Jacobian function; if not provided, a
 *     forward finite-difference approximation is used instead (one extra
 *     residual evaluation per state dimension).
 * @return the final state, cost, iteration count, and whether convergence
 *     (small step norm or small cost improvement) was reached before
 *     `max_iterations`.
 */
SolverResult gauss_newton(
    std::vector<double> x0,
    const ResidualFunction& residuals,
    int max_iterations = 80,
    double initial_lambda = 1e-3,
    const JacobianFunction& jacobian = JacobianFunction());

}  // namespace adaptive
