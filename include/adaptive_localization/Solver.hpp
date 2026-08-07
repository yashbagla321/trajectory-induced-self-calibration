#pragma once

#include <functional>
#include <vector>

#include "adaptive_localization/Types.hpp"

namespace adaptive {

using ResidualFunction = std::function<std::vector<double>(const std::vector<double>&)>;
using JacobianFunction =
    std::function<std::vector<std::vector<double>>(const std::vector<double>&)>;

SolverResult gauss_newton(
    std::vector<double> x0,
    const ResidualFunction& residuals,
    int max_iterations = 80,
    double initial_lambda = 1e-3,
    const JacobianFunction& jacobian = JacobianFunction());

}  // namespace adaptive
