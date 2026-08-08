// Solver.cpp
//
// Implementation of the generic damped Gauss-Newton (Levenberg-Marquardt
// style) nonlinear least-squares solver declared in Solver.hpp. The dense
// linear solve for the normal equations is delegated to
// solve_dense_linear_system() in Matrix.hpp/Matrix.cpp (shared with the EKF
// baseline in Simulation.cpp), using SingularPivotPolicy::RegularizeDiagonal
// -- see Matrix.hpp for why this solver's near-singular fallback differs
// from the EKF's.

#include "adaptive_localization/Solver.hpp"

#include <algorithm>
#include <cmath>

#include "adaptive_localization/Matrix.hpp"

namespace adaptive {

namespace {

/// The nonlinear least-squares objective: 0.5 * sum(r_i^2). The 0.5 factor
/// matches the convention used when differentiating the cost (so the
/// gradient is exactly J^T r, without a stray factor of 2).
double residual_cost(const std::vector<double>& r) {
    double value = 0.0;
    for (double residual : r) {
        value += residual * residual;
    }
    return 0.5 * value;
}

}  // namespace

/**
 * Damped Gauss-Newton (Levenberg-Marquardt style) nonlinear least-squares
 * solve. See Solver.hpp for the full contract; this walks through the
 * per-iteration mechanics.
 */
SolverResult gauss_newton(
    std::vector<double> x,
    const ResidualFunction& residuals,
    int max_iterations,
    double initial_lambda,
    const JacobianFunction& jacobian_function) {
    double lambda = initial_lambda;
    std::vector<double> r = residuals(x);
    double cost = residual_cost(r);
    bool converged = false;
    int iter = 0;

    for (; iter < max_iterations; ++iter) {
        const std::size_t m = r.size();
        const std::size_t n = x.size();
        std::vector<std::vector<double>> jacobian;
        std::vector<std::vector<double>> jtj(n, std::vector<double>(n, 0.0));
        std::vector<double> jtr(n, 0.0);

        if (jacobian_function) {
            jacobian = jacobian_function(x);
        } else {
            // No analytic Jacobian supplied: approximate each column via a
            // one-sided (forward) finite difference. The step size scales
            // with the magnitude of the corresponding state entry
            // (1e-6 * (1 + |x_col|)) so the perturbation is neither
            // vanishingly small relative to large state values nor
            // needlessly large for near-zero ones.
            jacobian.assign(m, std::vector<double>(n, 0.0));
            for (std::size_t col = 0; col < n; ++col) {
                const double step = 1e-6 * (1.0 + std::abs(x[col]));
                std::vector<double> xp = x;
                xp[col] += step;
                const std::vector<double> rp = residuals(xp);
                for (std::size_t row = 0; row < m; ++row) {
                    jacobian[row][col] = (rp[row] - r[row]) / step;
                }
            }
        }

        // Accumulate the Gauss-Newton normal-equation pieces: J^T J (only
        // the lower triangle is computed here; the upper triangle is
        // mirrored below since J^T J is symmetric) and J^T r.
        for (std::size_t row = 0; row < m; ++row) {
            for (std::size_t col = 0; col < n; ++col) {
                jtr[col] += jacobian[row][col] * r[row];
                for (std::size_t col2 = 0; col2 <= col; ++col2) {
                    jtj[col][col2] += jacobian[row][col] * jacobian[row][col2];
                }
            }
        }

        // Mirror the lower triangle into the upper triangle, then add the
        // Levenberg damping term lambda to the diagonal. This is what turns
        // plain Gauss-Newton into damped Gauss-Newton/Levenberg-Marquardt:
        // larger lambda shrinks the step and biases it toward the (always
        // well-defined) gradient-descent direction, which is essential when
        // J^T J is ill-conditioned or singular (e.g. under weak/degenerate
        // observability).
        for (std::size_t row = 0; row < n; ++row) {
            for (std::size_t col = row + 1; col < n; ++col) {
                jtj[row][col] = jtj[col][row];
            }
            jtj[row][row] += lambda;
        }

        // Solve (J^T J + lambda*I) * delta = -J^T r for the Gauss-Newton step.
        std::vector<double> rhs(jtr.size());
        std::transform(jtr.begin(), jtr.end(), rhs.begin(), [](double v) { return -v; });
        const std::vector<double> delta =
            solve_dense_linear_system(jtj, rhs, SingularPivotPolicy::RegularizeDiagonal);

        std::vector<double> candidate = x;
        double step_norm = 0.0;
        for (std::size_t i = 0; i < candidate.size(); ++i) {
            candidate[i] += delta[i];
            step_norm += delta[i] * delta[i];
        }
        step_norm = std::sqrt(step_norm);

        // Evaluate the candidate step's actual cost (not just its predicted
        // improvement) and accept/reject it -- the core Levenberg-Marquardt
        // trust-region-like heuristic.
        const std::vector<double> candidate_r = residuals(candidate);
        const double candidate_cost = residual_cost(candidate_r);
        if (candidate_cost < cost) {
            // Step accepted: commit it, then shrink lambda (move closer to
            // plain, faster-converging Gauss-Newton) since the local
            // quadratic model is proving trustworthy.
            const double improvement = cost - candidate_cost;
            x = candidate;
            r = candidate_r;
            cost = candidate_cost;
            lambda = std::max(1e-9, lambda * 0.3);
            // Converged once the step itself is negligible or the cost is no
            // longer meaningfully improving.
            if (step_norm < 1e-8 || improvement < 1e-10) {
                converged = true;
                break;
            }
        } else {
            // Step rejected: cost got worse, so distrust the local quadratic
            // model more -- grow lambda (move closer to a smaller,
            // gradient-descent-like step) and retry from the same x on the
            // next iteration.
            lambda = std::min(1e9, lambda * 10.0);
        }
    }

    return {x, cost, std::min(iter + 1, max_iterations), converged};
}

}  // namespace adaptive
