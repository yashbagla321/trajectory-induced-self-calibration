// Estimators.hpp
//
// Scenario-specific pieces plugged into the generic Solver.hpp Gauss-Newton
// solver: whitened residual/Jacobian functions for the two measurement
// models (scenario 1 = uncalibrated local-frame self-calibration, scenario 2
// = calibrated global-frame baseline), state-vector layout helpers, and
// initial-state ("seed") constructors, including the non-iterative
// closed-form two-view seed used to warm-start the nonlinear solve.

#pragma once

#include <vector>

#include "adaptive_localization/Types.hpp"

namespace adaptive {

/**
 * Whitened residual vector for the scenario-1 (uncalibrated local-frame)
 * measurement model. State layout is
 * [target.x, target.y, beacon0.x, beacon0.y, beacon0.yaw, beacon1.x, ...]
 * (2 + 3*beacon_count entries). For each measurement, predicts the
 * vehicle range/bearing and (once per beacon, unless `repeat_target_packets`
 * is true) the target range/bearing from the current state, and returns
 * (predicted - measured) divided by the appropriate noise standard deviation
 * (range_sigma / bearing_sigma from `noise`) -- i.e. each residual is in
 * units of standard deviations ("whitened"), so that minimizing the sum of
 * squares is equivalent to maximizing Gaussian log-likelihood. Bearing
 * residuals are wrapped to (-pi, pi] before whitening.
 *
 * `repeat_target_packets` controls whether every measurement of a beacon
 * contributes its own target-range/bearing residual pair (true, the default,
 * matching how measurements are actually generated -- one target packet per
 * beacon per time step) or only the first sighting of each beacon does
 * (false, used where callers want to avoid double-counting a
 * once-per-beacon target reading against many per-time-step vehicle
 * readings).
 */
std::vector<double> residuals_scenario1(
    const std::vector<double>& state,
    int beacon_count,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise = Noise{},
    bool repeat_target_packets = true);

/**
 * Analytic Jacobian of residuals_scenario1 with respect to the state vector,
 * row-aligned with the residual vector residuals_scenario1 would produce for
 * the same arguments (each measurement contributes a vehicle-range row, a
 * vehicle-bearing row, and -- when the target packet is included -- a
 * target-range row and target-bearing row). Supplying this avoids the
 * generic solver falling back to (slower, approximate) finite differences.
 */
std::vector<std::vector<double>> jacobian_scenario1(
    const std::vector<double>& state,
    int beacon_count,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise = Noise{},
    bool repeat_target_packets = true);

/**
 * Whitened residual vector for the scenario-2 (calibrated global-frame
 * baseline) measurement model. State layout is just [target.x, target.y].
 * For each measurement, the beacon position is inferred from the current
 * target estimate and the (already-global-frame) bearing/range to the
 * target, then the predicted vehicle range from that inferred beacon
 * position is compared against the measured vehicle range. Residuals are
 * whitened by a fixed range noise scale of 0.05 (not the caller-supplied
 * Noise, since this baseline model only ever varies range noise via the
 * shared `Noise::range_sigma` used at measurement-generation time).
 */
std::vector<double> residuals_scenario2(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<GlobalBearingMeasurement>& measurements);

/// Builds a generic (non-data-driven) scenario-1 initial state: the given
/// `target_seed`, plus `beacon_count` beacons placed evenly around a circle
/// of radius `beacon_radius` (centered at the origin) all sharing the same
/// initial yaw guess `beacon_yaw`. Used when a closed-form seed (see
/// two_view_closed_form_initial_state) isn't available or applicable.
std::vector<double> initial_state_scenario1(
    int beacon_count,
    const Vec2& target_seed,
    double beacon_radius,
    double beacon_yaw);

/// Builds the (trivial, 2-entry) scenario-2 initial state from a target seed.
std::vector<double> initial_state_scenario2(const Vec2& target_seed);

/// Extracts per-beacon position/yaw estimates from a solved scenario-1 state
/// vector (inverse of the layout used by initial_state_scenario1 /
/// residuals_scenario1). Yaw is wrapped to (-pi, pi].
std::vector<BeaconEstimate> beacon_estimates_from_scenario1_state(
    const std::vector<double>& state,
    int beacon_count);

/**
 * Derives per-beacon position estimates for scenario 2 from a solved target
 * estimate and the raw global-bearing measurements: for each measurement,
 * infers a candidate beacon position as
 * target_estimate - (unit vector along bt_global) * rt, then averages all
 * such candidates per beacon. Scenario 2 does not estimate beacon yaw, so
 * every returned BeaconEstimate has yaw = 0.0. Beacons with no measurements
 * get a default (zero) position.
 */
std::vector<BeaconEstimate> beacon_estimates_from_scenario2_measurements(
    const Vec2& target_estimate,
    int beacon_count,
    const std::vector<GlobalBearingMeasurement>& measurements);

/**
 * Builds a non-iterative, closed-form scenario-1 initial state ("constructive
 * two-view seed") directly from data, without needing an initial guess or any
 * nonlinear solve. For each beacon, searches all pairs of time steps at
 * which that beacon has a measurement and picks the pair whose local
 * vehicle-bearing vectors differ the most (largest baseline/"spread"),
 * since a small baseline makes two-view triangulation ill-conditioned. The
 * beacon's unknown yaw is then recovered as the rotation that maps the
 * observed local displacement between those two views onto the known global
 * displacement of the robot between the same two views (comparing
 * atan2 of each); beacon position is back-solved from the first view using
 * that recovered yaw, and the target position is triangulated from every
 * measurement of that beacon using the same recovered pose, then averaged
 * across beacons.
 *
 * @return false (leaving `state` untouched) if `beacon_count` is
 *     non-positive, `path` is empty, or any beacon lacks two views with
 *     baseline spread above a small threshold (1e-6) -- i.e. the trajectory
 *     did not sufficiently excite that beacon for the closed form to be
 *     well-defined. Returns true and fills `state` (in the same layout as
 *     initial_state_scenario1) otherwise.
 */
bool two_view_closed_form_initial_state(
    int beacon_count,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    std::vector<double>& state);

}  // namespace adaptive
