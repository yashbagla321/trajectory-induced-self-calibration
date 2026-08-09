// test_closed_loop_gate.cpp
//
// Covers the closed loop's two-view initialization gate (Algorithm 1) and
// the constructive seed it relies on: the estimator must retain its prior
// while the stored window holds only one view, become active exactly when a
// second distinct view arrives, and recover the noiseless state exactly at
// that point. Also checks the measurement model's exact equivariance to a
// common global translation of (vehicle, beacon, target), which is what
// makes the spread certificate S_v depend only on relative geometry.

#include <random>
#include <vector>

#include "adaptive_localization/Config.hpp"
#include "adaptive_localization/Estimators.hpp"
#include "adaptive_localization/Math.hpp"
#include "adaptive_localization/Measurements.hpp"
#include "adaptive_localization/Simulation.hpp"
#include "adaptive_localization/World.hpp"
#include "test_framework.hpp"

using namespace adaptive;

ADAPTIVE_TEST(two_view_constructive_seed_recovers_noiseless_state) {
    const World world = make_world(1);
    const std::vector<Vec2> path{{-3.0, 2.6}, {-2.7, 2.4}};
    std::mt19937 rng(17U);
    const auto measurements =
        generate_local_frame_measurements(world, path, Noise{0.0, 0.0}, rng);

    std::vector<double> state;
    CHECK(two_view_closed_form_initial_state(1, path, measurements, state));
    EXPECT_NEAR(state[0], world.target.x, 1e-10);
    EXPECT_NEAR(state[1], world.target.y, 1e-10);
    EXPECT_NEAR(state[2], world.beacons[0].x, 1e-10);
    EXPECT_NEAR(state[3], world.beacons[0].y, 1e-10);
    EXPECT_NEAR(wrap_angle(state[4] - world.beacon_yaws[0]), 0.0, 1e-10);
}

ADAPTIVE_TEST(one_view_gate_retains_prior_until_second_view) {
    SimulationConfig config;
    config.closed_loop_steps = 2;
    config.closed_loop_noise = {0.0, 0.0};
    std::mt19937 rng(3U);
    const auto result = run_closed_loop_comparison(
        1, 1, config, rng, ClosedLoopExcitationMode::Supervised);

    CHECK(result.points.size() == 3);
    // One stored view: the gate must hold and the prior target estimate
    // must pass through the controller untouched.
    CHECK(!result.points[1].estimate_ready);
    EXPECT_NEAR(result.points[1].target_estimate.x,
                config.initial_target_estimate.x, 1e-12);
    EXPECT_NEAR(result.points[1].target_estimate.y,
                config.initial_target_estimate.y, 1e-12);
    // Two distinct views: the constructive seed activates the estimator and
    // the noiseless estimate is exact up to solver tolerance.
    CHECK(result.points[2].estimate_ready);
    CHECK(result.points[2].target_error < 1e-8);
}

ADAPTIVE_TEST(local_packets_are_invariant_to_common_translation) {
    const World world = make_world(1);
    const Vec2 q{-1.2, 0.7};
    const Vec2 shift{4.5, -2.25};
    World shifted = world;
    shifted.target = shifted.target + shift;
    shifted.beacons[0] = shifted.beacons[0] + shift;

    std::mt19937 rng_a(9U);
    std::mt19937 rng_b(9U);
    const auto a = make_local_frame_measurement(world, q, 0, 0, Noise{0.0, 0.0}, rng_a);
    const auto b =
        make_local_frame_measurement(shifted, q + shift, 0, 0, Noise{0.0, 0.0}, rng_b);

    EXPECT_NEAR(a.rv, b.rv, 1e-12);
    EXPECT_NEAR(wrap_angle(a.bv_local - b.bv_local), 0.0, 1e-12);
    EXPECT_NEAR(a.rt, b.rt, 1e-12);
    EXPECT_NEAR(wrap_angle(a.bt_local - b.bt_local), 0.0, 1e-12);
}
