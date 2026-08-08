// test_jacobian.cpp
//
// Checks jacobian_scenario1 (the hand-derived analytic Jacobian used to
// avoid falling back to slow finite-difference Gauss-Newton steps) against
// a central finite-difference approximation of residuals_scenario1 at an
// arbitrary state. The measurement values below are arbitrary and never
// used as truth -- the residual is (predicted - measured) / sigma, so its
// derivative with respect to the state is independent of which values were
// "measured"; only the predicted-quantity formulas (and hence their
// derivatives) matter here.

#include "adaptive_localization/Estimators.hpp"
#include "test_framework.hpp"

using namespace adaptive;

namespace {

std::vector<LocalFrameMeasurement> make_two_pose_measurements() {
    LocalFrameMeasurement first;
    first.beacon = 0;
    first.time = 0;
    first.rv = 1.0;
    first.bv_local = 0.3;
    first.rt = 1.5;
    first.bt_local = -0.2;

    LocalFrameMeasurement second;
    second.beacon = 0;
    second.time = 1;
    second.rv = 1.2;
    second.bv_local = 0.25;
    second.rt = 1.6;
    second.bt_local = -0.15;

    return {first, second};
}

}  // namespace

ADAPTIVE_TEST(jacobian_scenario1_matches_finite_difference) {
    const std::vector<Vec2> path = {Vec2(0.0, 0.0), Vec2(1.0, 0.3)};
    const auto measurements = make_two_pose_measurements();
    const Noise noise;
    const int beacon_count = 1;
    // [target.x, target.y, beacon0.x, beacon0.y, beacon0.yaw]
    const std::vector<double> state = {1.0, 2.0, 0.5, -0.3, 0.2};

    const auto analytic = jacobian_scenario1(state, beacon_count, path, measurements, noise);
    const auto base_residuals = residuals_scenario1(state, beacon_count, path, measurements, noise);

    CHECK(!analytic.empty());
    CHECK(analytic.size() == base_residuals.size());

    constexpr double h = 1e-6;
    for (std::size_t dim = 0; dim < state.size(); ++dim) {
        std::vector<double> plus = state;
        std::vector<double> minus = state;
        plus[dim] += h;
        minus[dim] -= h;
        const auto residuals_plus = residuals_scenario1(plus, beacon_count, path, measurements, noise);
        const auto residuals_minus = residuals_scenario1(minus, beacon_count, path, measurements, noise);

        for (std::size_t row = 0; row < base_residuals.size(); ++row) {
            const double finite_difference = (residuals_plus[row] - residuals_minus[row]) / (2.0 * h);
            EXPECT_NEAR(analytic[row][dim], finite_difference, 1e-4);
        }
    }
}
