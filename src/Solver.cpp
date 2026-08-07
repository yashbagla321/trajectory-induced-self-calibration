#include "adaptive_localization/Solver.hpp"

#include <algorithm>
#include <cmath>

namespace adaptive {

namespace {

double residual_cost(const std::vector<double>& r) {
    double value = 0.0;
    for (double residual : r) {
        value += residual * residual;
    }
    return 0.5 * value;
}

std::vector<double> solve_linear_system(std::vector<std::vector<double>> a, std::vector<double> b) {
    const std::size_t n = b.size();
    for (std::size_t col = 0; col < n; ++col) {
        std::size_t pivot = col;
        double best = std::abs(a[col][col]);
        for (std::size_t row = col + 1; row < n; ++row) {
            const double value = std::abs(a[row][col]);
            if (value > best) {
                best = value;
                pivot = row;
            }
        }
        if (best < 1e-12) {
            a[col][col] += 1e-8;
            pivot = col;
        }
        if (pivot != col) {
            std::swap(a[pivot], a[col]);
            std::swap(b[pivot], b[col]);
        }

        const double diag = a[col][col];
        for (std::size_t j = col; j < n; ++j) {
            a[col][j] /= diag;
        }
        b[col] /= diag;

        for (std::size_t row = 0; row < n; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = a[row][col];
            for (std::size_t j = col; j < n; ++j) {
                a[row][j] -= factor * a[col][j];
            }
            b[row] -= factor * b[col];
        }
    }
    return b;
}

}  // namespace

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

        for (std::size_t row = 0; row < m; ++row) {
            for (std::size_t col = 0; col < n; ++col) {
                jtr[col] += jacobian[row][col] * r[row];
                for (std::size_t col2 = 0; col2 <= col; ++col2) {
                    jtj[col][col2] += jacobian[row][col] * jacobian[row][col2];
                }
            }
        }

        for (std::size_t row = 0; row < n; ++row) {
            for (std::size_t col = row + 1; col < n; ++col) {
                jtj[row][col] = jtj[col][row];
            }
            jtj[row][row] += lambda;
        }

        std::vector<double> rhs(jtr.size());
        std::transform(jtr.begin(), jtr.end(), rhs.begin(), [](double v) { return -v; });
        const std::vector<double> delta = solve_linear_system(jtj, rhs);

        std::vector<double> candidate = x;
        double step_norm = 0.0;
        for (std::size_t i = 0; i < candidate.size(); ++i) {
            candidate[i] += delta[i];
            step_norm += delta[i] * delta[i];
        }
        step_norm = std::sqrt(step_norm);

        const std::vector<double> candidate_r = residuals(candidate);
        const double candidate_cost = residual_cost(candidate_r);
        if (candidate_cost < cost) {
            const double improvement = cost - candidate_cost;
            x = candidate;
            r = candidate_r;
            cost = candidate_cost;
            lambda = std::max(1e-9, lambda * 0.3);
            if (step_norm < 1e-8 || improvement < 1e-10) {
                converged = true;
                break;
            }
        } else {
            lambda = std::min(1e9, lambda * 10.0);
        }
    }

    return {x, cost, std::min(iter + 1, max_iterations), converged};
}

}  // namespace adaptive
