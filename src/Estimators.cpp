// Estimators.cpp
//
// Implements the scenario-specific residual/Jacobian functions, state-layout
// helpers, and initial-state constructors declared in Estimators.hpp --
// including the closed-form "constructive" two-view beacon-pose initializer
// that seeds the Gauss-Newton solver without needing an iterative guess.

#include "adaptive_localization/Estimators.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace adaptive {

namespace {

/// Returns noise.range_sigma, falling back to a reasonable default (0.03)
/// when it is non-positive (treated elsewhere as "noiseless"), so whitening
/// by this value never divides by zero.
double effective_range_sigma(const Noise& noise) {
    return noise.range_sigma > 0.0 ? noise.range_sigma : 0.03;
}

/// Returns noise.bearing_sigma, falling back to a reasonable default
/// (0.006) when it is non-positive, mirroring effective_range_sigma.
double effective_bearing_sigma(const Noise& noise) {
    return noise.bearing_sigma > 0.0 ? noise.bearing_sigma : 0.006;
}

/// Reconstructs the local-frame displacement vector from a beacon to the
/// vehicle at the time this measurement was taken, from its polar
/// (range, bearing) representation.
Vec2 local_vehicle_vector(const LocalFrameMeasurement& measurement) {
    return unit_from_angle(measurement.bv_local) * measurement.rv;
}

/// Reconstructs the local-frame displacement vector from a beacon to the
/// target, from its polar (range, bearing) representation.
Vec2 local_target_vector(const LocalFrameMeasurement& measurement) {
    return unit_from_angle(measurement.bt_local) * measurement.rt;
}

/**
 * Decides whether a given measurement's target-range/bearing packet should
 * contribute residuals. When `repeat_target_packets` is true, every
 * measurement's target packet counts (matching how measurements are
 * actually generated: one target reading per beacon per time step). When
 * false, only the first measurement seen for each beacon counts -- `
 * target_seen` tracks, per beacon index, whether that beacon's target
 * packet has already been consumed, so repeated target readings from later
 * time steps of the same beacon are skipped (avoiding over-weighting a
 * quantity, the target position, that does not actually change over time).
 */
bool should_include_target_packet(
    std::vector<bool>& target_seen,
    const LocalFrameMeasurement& measurement,
    bool repeat_target_packets) {
    if (repeat_target_packets) {
        return true;
    }
    if (measurement.beacon >= target_seen.size()) {
        return false;
    }
    if (target_seen[measurement.beacon]) {
        return false;
    }
    target_seen[measurement.beacon] = true;
    return true;
}

}  // namespace

std::vector<double> residuals_scenario1(
    const std::vector<double>& state,
    int beacon_count,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise,
    bool repeat_target_packets) {
    std::vector<double> residuals;
    residuals.reserve(measurements.size() * 4);
    std::vector<bool> target_seen(static_cast<std::size_t>(std::max(0, beacon_count)), false);
    // Target position lives in the first 2 state entries; see the file-level
    // note in Estimators.hpp for the full state layout.
    const Vec2 p{state[0], state[1]};
    const double range_sigma = effective_range_sigma(noise);
    const double bearing_sigma = effective_bearing_sigma(noise);

    for (const auto& m : measurements) {
        // Beacon m.beacon's pose occupies state[base], state[base+1] (x, y)
        // and state[base+2] (yaw).
        const std::size_t base = 2 + 3 * m.beacon;
        const Vec2 x_i{state[base], state[base + 1]};
        const double yaw_i = state[base + 2];
        // Predicted vehicle/target displacement from the beacon, in the
        // GLOBAL frame, using the current state estimate.
        const Vec2 vehicle_delta = path[m.time] - x_i;
        const Vec2 target_delta = p - x_i;
        // Predicted range is just the displacement's length (floored to
        // avoid a degenerate zero-range prediction). Predicted bearing is
        // the global-frame angle to the displacement, rotated into the
        // beacon's own (currently estimated) local frame by subtracting its
        // yaw -- mirroring how the local-frame measurement was generated.
        const double rv_pred = std::max(1e-9, norm(vehicle_delta));
        const double bv_pred = wrap_angle(std::atan2(vehicle_delta.y, vehicle_delta.x) - yaw_i);

        // Whitened residuals: (prediction - measurement) / noise std dev, so
        // the returned vector is in units of standard deviations and
        // minimizing its sum of squares is equivalent to maximizing Gaussian
        // likelihood. Bearing residuals are wrapped to (-pi, pi] before
        // whitening so a bearing error near +-2*pi isn't mistaken for a huge
        // error.
        residuals.push_back((rv_pred - m.rv) / range_sigma);
        residuals.push_back(wrap_angle(bv_pred - m.bv_local) / bearing_sigma);
        if (should_include_target_packet(target_seen, m, repeat_target_packets)) {
            const double rt_pred = std::max(1e-9, norm(target_delta));
            const double bt_pred = wrap_angle(std::atan2(target_delta.y, target_delta.x) - yaw_i);
            residuals.push_back((rt_pred - m.rt) / range_sigma);
            residuals.push_back(wrap_angle(bt_pred - m.bt_local) / bearing_sigma);
        }
    }

    return residuals;
}

std::vector<std::vector<double>> jacobian_scenario1(
    const std::vector<double>& state,
    int beacon_count,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise,
    bool repeat_target_packets) {
    const std::size_t state_dim = state.size();
    std::vector<std::vector<double>> jacobian;
    jacobian.reserve(measurements.size() * 4);
    std::vector<bool> target_seen(static_cast<std::size_t>(std::max(0, beacon_count)), false);
    const Vec2 p{state[0], state[1]};
    const double range_sigma = effective_range_sigma(noise);
    const double bearing_sigma = effective_bearing_sigma(noise);

    for (const auto& m : measurements) {
        // Each measurement contributes 2 rows (vehicle range, vehicle
        // bearing) or 4 rows (also target range, target bearing) to the
        // Jacobian, in exactly the order residuals_scenario1 would emit the
        // corresponding residuals -- so this function's output rows line up
        // one-to-one with residuals_scenario1's output entries.
        const std::size_t base = 2 + 3 * m.beacon;
        const Vec2 x_i{state[base], state[base + 1]};
        const Vec2 vehicle_delta = path[m.time] - x_i;
        const double rv = std::max(1e-9, norm(vehicle_delta));
        const double rv2 = rv * rv;

        // d(range)/d(beacon position): the whitened range residual is
        // (|delta| - measured)/sigma, so its gradient w.r.t. the beacon
        // position (which delta = vehicle - beacon depends on negatively) is
        // -delta/(|delta|*sigma). Only the beacon-position columns of this
        // row are nonzero because the vehicle range does not depend on the
        // target position or on this beacon's yaw.
        std::vector<double> vehicle_range_row(state_dim, 0.0);
        vehicle_range_row[base] = -vehicle_delta.x / (rv * range_sigma);
        vehicle_range_row[base + 1] = -vehicle_delta.y / (rv * range_sigma);
        jacobian.push_back(std::move(vehicle_range_row));

        // d(bearing)/d(state): the whitened bearing residual is
        // (atan2(delta) - yaw - measured)/sigma, so its gradient w.r.t.
        // (delta.x, delta.y) is the standard d(atan2)/d(.) derivative
        // (delta.y, -delta.x)/|delta|^2, propagated through delta's
        // dependence on beacon position (with a sign flip, since delta =
        // vehicle - beacon), plus a direct -1/sigma term for the beacon-yaw
        // column since the residual subtracts yaw linearly.
        std::vector<double> vehicle_bearing_row(state_dim, 0.0);
        vehicle_bearing_row[base] = vehicle_delta.y / (rv2 * bearing_sigma);
        vehicle_bearing_row[base + 1] = -vehicle_delta.x / (rv2 * bearing_sigma);
        vehicle_bearing_row[base + 2] = -1.0 / bearing_sigma;
        jacobian.push_back(std::move(vehicle_bearing_row));

        if (should_include_target_packet(target_seen, m, repeat_target_packets)) {
            const Vec2 target_delta = p - x_i;
            const double rt = std::max(1e-9, norm(target_delta));
            const double rt2 = rt * rt;

            // Same range-derivative pattern as vehicle_range_row, but now
            // target_delta = target - beacon depends on BOTH the target
            // position (columns 0, 1) and this beacon's position (columns
            // base, base+1), with opposite signs, since moving either point
            // changes the same displacement vector in opposite ways.
            std::vector<double> target_range_row(state_dim, 0.0);
            target_range_row[0] = target_delta.x / (rt * range_sigma);
            target_range_row[1] = target_delta.y / (rt * range_sigma);
            target_range_row[base] = -target_delta.x / (rt * range_sigma);
            target_range_row[base + 1] = -target_delta.y / (rt * range_sigma);
            jacobian.push_back(std::move(target_range_row));

            // Same bearing-derivative pattern as vehicle_bearing_row, again
            // split between the target-position columns and the
            // beacon-position columns (opposite signs), plus the beacon-yaw
            // column's direct -1/sigma term.
            std::vector<double> target_bearing_row(state_dim, 0.0);
            target_bearing_row[0] = -target_delta.y / (rt2 * bearing_sigma);
            target_bearing_row[1] = target_delta.x / (rt2 * bearing_sigma);
            target_bearing_row[base] = target_delta.y / (rt2 * bearing_sigma);
            target_bearing_row[base + 1] = -target_delta.x / (rt2 * bearing_sigma);
            target_bearing_row[base + 2] = -1.0 / bearing_sigma;
            jacobian.push_back(std::move(target_bearing_row));
        }
    }
    return jacobian;
}

std::vector<double> residuals_scenario2(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<GlobalBearingMeasurement>& measurements) {
    std::vector<double> residuals;
    residuals.reserve(measurements.size());
    const Vec2 p{state[0], state[1]};

    for (const auto& m : measurements) {
        // Since the target bearing is already in the global frame, the
        // beacon's position can be inferred directly from the current
        // target estimate: beacon = target - (unit vector along
        // bt_global) * rt. The residual then compares the vehicle range
        // predicted from that inferred beacon position against the
        // measured vehicle range. Whitened by a fixed range-noise scale of
        // 0.05 (not the caller's Noise) since this baseline model is only
        // ever exercised with that scale in this codebase.
        const Vec2 beacon_est = p - unit_from_angle(m.bt_global) * m.rt;
        residuals.push_back((norm(path[m.time] - beacon_est) - m.rv) / 0.05);
    }

    return residuals;
}

/// Generic (non-data-driven) scenario-1 seed: `beacon_count` beacons are
/// placed evenly spaced around a circle of radius `beacon_radius` centered
/// at the origin, all sharing the same initial yaw guess `beacon_yaw`. This
/// is a coarse placeholder guess -- correctness relies on the Gauss-Newton
/// solver (or multistart) converging from here, unlike
/// two_view_closed_form_initial_state below which computes an exact seed
/// from data.
std::vector<double> initial_state_scenario1(
    int beacon_count,
    const Vec2& target_seed,
    double beacon_radius,
    double beacon_yaw) {
    std::vector<double> state{target_seed.x, target_seed.y};
    for (int i = 0; i < beacon_count; ++i) {
        const double angle = 2.0 * kPi * static_cast<double>(i) / std::max(1, beacon_count);
        state.push_back(beacon_radius * std::cos(angle));
        state.push_back(beacon_radius * std::sin(angle));
        state.push_back(beacon_yaw);
    }
    return state;
}

/// Trivial scenario-2 seed: just the 2D target-position guess (scenario 2
/// has no beacon-pose unknowns to seed).
std::vector<double> initial_state_scenario2(const Vec2& target_seed) {
    return {target_seed.x, target_seed.y};
}

/// Inverse of the beacon-pose portion of the scenario-1 state layout: reads
/// back `beacon_count` (position, yaw) estimates from a solved state vector,
/// wrapping yaw to (-pi, pi].
std::vector<BeaconEstimate> beacon_estimates_from_scenario1_state(
    const std::vector<double>& state,
    int beacon_count) {
    std::vector<BeaconEstimate> estimates;
    for (int i = 0; i < beacon_count; ++i) {
        const std::size_t base = 2 + 3 * static_cast<std::size_t>(i);
        estimates.push_back({{state[base], state[base + 1]}, wrap_angle(state[base + 2])});
    }
    return estimates;
}

/**
 * Derives per-beacon position estimates for scenario 2 (which never
 * jointly estimates beacon position) after the target position has been
 * solved for. For each measurement of beacon i, a candidate beacon position
 * is inferred as target_estimate - (unit vector along bt_global) * rt
 * (i.e. "walk backward from the target along the measured bearing/range");
 * these candidates are averaged across all measurements of that beacon to
 * reduce noise. A beacon with zero measurements gets a default
 * (zero-initialized) position. Beacon yaw is never estimated in scenario 2,
 * so every returned estimate has yaw = 0.0.
 */
std::vector<BeaconEstimate> beacon_estimates_from_scenario2_measurements(
    const Vec2& target_estimate,
    int beacon_count,
    const std::vector<GlobalBearingMeasurement>& measurements) {
    std::vector<Vec2> sums(static_cast<std::size_t>(beacon_count));
    std::vector<int> counts(static_cast<std::size_t>(beacon_count), 0);

    for (const auto& m : measurements) {
        sums[m.beacon] = sums[m.beacon] + (target_estimate - unit_from_angle(m.bt_global) * m.rt);
        counts[m.beacon] += 1;
    }

    std::vector<BeaconEstimate> estimates;
    for (int i = 0; i < beacon_count; ++i) {
        Vec2 position;
        if (counts[static_cast<std::size_t>(i)] > 0) {
            position = sums[static_cast<std::size_t>(i)] /
                static_cast<double>(counts[static_cast<std::size_t>(i)]);
        }
        estimates.push_back({position, 0.0});
    }
    return estimates;
}

/**
 * Non-iterative "constructive two-view seed" for scenario 1. Computes an
 * exact closed-form estimate of every beacon's pose (position + yaw) and the
 * target's position directly from data, without needing an initial guess or
 * running any Gauss-Newton iterations. Intended as a much better warm-start
 * for the nonlinear solver than the generic circular guess in
 * initial_state_scenario1, and used directly (without refinement) as the
 * scenario-4 EKF seed in Simulation.cpp.
 *
 * Per beacon, the construction is:
 *  1. Find the two-view baseline: among every pair of time steps at which
 *     this beacon observed the vehicle, pick the pair (`first`, `second`)
 *     whose LOCAL vehicle-bearing vectors differ the most (`best_spread` =
 *     the largest norm(ell_b - ell_a)). A large spread means the vehicle was
 *     seen from two substantially different aspect angles/ranges, which is
 *     what makes the two-view triangulation below well-conditioned; a small
 *     spread would make it numerically unstable (nearly parallel views), so
 *     beacons/datasets without any pair above a 1e-6 spread threshold are
 *     rejected (return false).
 *  2. Recover the beacon's unknown yaw: the vehicle's motion between the two
 *     chosen views is known exactly in the GLOBAL frame (dq = path[second] -
 *     path[first]), and the same motion is observed in the beacon's LOCAL
 *     frame as dell = ell_b - ell_a. Since dell is dq rotated into the
 *     beacon's local frame, the beacon's yaw is exactly the angle needed to
 *     rotate dell onto dq: yaw = atan2(dq) - atan2(dell).
 *  3. Back-solve beacon position: having recovered yaw, the local-frame
 *     vector ell_a (beacon -> vehicle at the first view) rotates into the
 *     global frame as rotate(ell_a, yaw), so beacon position =
 *     path[first] - rotate(ell_a, yaw).
 *  4. Triangulate the target: every measurement of this beacon gives a
 *     local-frame vector to the target, which is rotated by the just-solved
 *     yaw and added to the beacon position to get one global-frame estimate
 *     of the target; these are summed here and averaged (across all beacons
 *     and all their measurements) once the per-beacon loop finishes.
 *
 * @return false, leaving `state` untouched, if there are no beacons/path
 *     poses to work with, if any beacon lacks a sufficiently-excited view
 *     pair (spread >= 1e-6), or if no target sightings exist at all.
 *     Otherwise fills `state` in the same [target, beacon0, beacon1, ...]
 *     layout as initial_state_scenario1 and returns true.
 */
bool two_view_closed_form_initial_state(
    int beacon_count,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    std::vector<double>& state) {
    if (beacon_count <= 0 || path.empty()) {
        return false;
    }

    std::vector<BeaconEstimate> beacon_estimates(static_cast<std::size_t>(beacon_count));
    std::vector<int> target_counts(static_cast<std::size_t>(beacon_count), 0);
    Vec2 target_sum{0.0, 0.0};

    for (int beacon = 0; beacon < beacon_count; ++beacon) {
        // Step 1: brute-force search over all ordered pairs of this beacon's
        // measurements for the pair with the largest local-frame baseline
        // ("spread") between the vehicle-bearing vectors. O(k^2) in the
        // number of this beacon's measurements, which is fine since k is
        // just the number of path steps.
        const LocalFrameMeasurement* first = nullptr;
        const LocalFrameMeasurement* second = nullptr;
        double best_spread = 0.0;
        for (const auto& a : measurements) {
            if (a.beacon != static_cast<std::size_t>(beacon) || a.time >= path.size()) {
                continue;
            }
            const Vec2 ell_a = local_vehicle_vector(a);
            for (const auto& b : measurements) {
                if (b.beacon != static_cast<std::size_t>(beacon) || b.time >= path.size() || a.time == b.time) {
                    continue;
                }
                const Vec2 ell_b = local_vehicle_vector(b);
                const double spread = norm(ell_b - ell_a);
                if (spread > best_spread) {
                    best_spread = spread;
                    first = &a;
                    second = &b;
                }
            }
        }
        if (first == nullptr || second == nullptr || best_spread < 1e-6) {
            // No sufficiently-excited view pair for this beacon (e.g. it was
            // only seen once, or from nearly the same aspect angle every
            // time) -- the closed form is ill-defined, so bail out entirely
            // rather than return a poor/undefined seed.
            return false;
        }

        // Step 2: recover beacon yaw from the rotation between the known
        // global-frame displacement (dq) and the observed local-frame
        // displacement (dell) of the vehicle between the two chosen views.
        const Vec2 ell_a = local_vehicle_vector(*first);
        const Vec2 ell_b = local_vehicle_vector(*second);
        const Vec2 dq = path[second->time] - path[first->time];
        const Vec2 dell = ell_b - ell_a;
        const double yaw = wrap_angle(std::atan2(dq.y, dq.x) - std::atan2(dell.y, dell.x));
        // Step 3: back-solve beacon position from the first view using the
        // just-recovered yaw.
        const Vec2 position = path[first->time] - rotate(ell_a, yaw);
        beacon_estimates[static_cast<std::size_t>(beacon)] = {position, yaw};

        // Step 4: triangulate the target from every measurement of this
        // beacon (not just the two chosen views) using the now-known beacon
        // pose, accumulating a running sum to be averaged later across all
        // beacons.
        for (const auto& m : measurements) {
            if (m.beacon != static_cast<std::size_t>(beacon)) {
                continue;
            }
            target_sum = target_sum + position + rotate(local_target_vector(m), yaw);
            ++target_counts[static_cast<std::size_t>(beacon)];
        }
    }

    int total_target_count = 0;
    for (int count : target_counts) {
        total_target_count += count;
    }
    if (total_target_count <= 0) {
        return false;
    }

    // Average all per-measurement target estimates (across every beacon)
    // into a single target-position seed, then pack everything into the
    // standard scenario-1 state layout.
    const Vec2 target = target_sum / static_cast<double>(total_target_count);
    state.clear();
    state.push_back(target.x);
    state.push_back(target.y);
    for (const auto& estimate : beacon_estimates) {
        state.push_back(estimate.position.x);
        state.push_back(estimate.position.y);
        state.push_back(estimate.yaw);
    }
    return true;
}

/** @brief RMSE of estimated vs. true beacon positions (paired by index with
 *  `world.beacons`); returns 0.0 for an empty estimate list. See
 *  Estimators.hpp for why this lives here rather than in Simulation.cpp. */
double beacon_position_rmse(const World& world, const std::vector<BeaconEstimate>& estimates) {
    if (estimates.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (std::size_t i = 0; i < estimates.size(); ++i) {
        const Vec2 error = estimates[i].position - world.beacons[i];
        sum += dot(error, error);
    }
    return std::sqrt(sum / static_cast<double>(estimates.size()));
}

/** @brief RMSE of estimated vs. true beacon yaw (wrapped to (-pi, pi] before
 *  squaring); returns the sentinel -1.0 for an empty estimate list, matching
 *  the "not applicable" convention used by scenario 2 (no yaw estimated). */
double beacon_yaw_rmse(const World& world, const std::vector<BeaconEstimate>& estimates) {
    if (estimates.empty()) {
        return -1.0;
    }
    double sum = 0.0;
    for (std::size_t i = 0; i < estimates.size(); ++i) {
        const double error = wrap_angle(estimates[i].yaw - world.beacon_yaws[i]);
        sum += error * error;
    }
    return std::sqrt(sum / static_cast<double>(estimates.size()));
}

}  // namespace adaptive
