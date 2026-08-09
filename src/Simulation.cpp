#include "adaptive_localization/Simulation.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <tuple>
#include <utility>

#include "adaptive_localization/Estimators.hpp"
#include "adaptive_localization/Matrix.hpp"
#include "adaptive_localization/Measurements.hpp"
#include "adaptive_localization/Solver.hpp"
#include "adaptive_localization/World.hpp"

/**
 * @file Simulation.cpp
 * @brief Implements every simulation entry point declared in Simulation.hpp:
 * the continuous-time adaptive estimator (run_adaptive_localization), the
 * animated closed-loop estimate/update/control demo
 * (run_closed_loop_comparison) with its three excitation policies, the
 * discrete-time batch trial harness spanning the uncalibrated local-frame
 * self-calibration model (scenario 1), the calibrated global-frame baseline
 * (scenario 2), and two EKF variants (scenarios 3/4), plus the local
 * observability/Fisher-information ("gauge") diagnostics
 * (normal_matrix_for_local_observability, jacobi_eigenvalues,
 * local_observability_metrics) and the many Monte Carlo robustness sweeps
 * (noise, geometry, trajectory shape, initialization, measurement
 * dropout/outliers, vehicle-pose noise, and information conditioning) that
 * populate the paper's result tables. Anonymous-namespace helpers below are
 * implementation details private to this translation unit.
 */

namespace adaptive {

/**
 * @brief Computes the eigenvalues of a symmetric 5x5 matrix via the
 * classic cyclic Jacobi eigenvalue algorithm: repeatedly finds the largest
 * off-diagonal entry and zeroes it with a Givens/Jacobi rotation, for up to
 * 80 sweeps or until all off-diagonal entries are negligible (< 1e-10).
 * Eigenvalues are clamped to be non-negative (the normal matrix is PSD in
 * exact arithmetic; small negative numerical noise is floored to 0) and
 * returned sorted ascending. Used to diagonalize the normal matrix from
 * normal_matrix_for_local_observability(); sqrt(eigenvalue) gives the
 * corresponding singular value of the Jacobian.
 *
 * Declared in Simulation.hpp (rather than kept file-private) so it can be
 * exercised directly by tests/jacobi_eigenvalues_test.cpp against matrices
 * with known closed-form eigenvalues, independent of the Monte Carlo sweeps
 * that are its only other caller.
 *
 * @param a Row-major flattened symmetric 5x5 matrix (passed by value since
 *        the algorithm mutates it in place as it rotates).
 * @return The 5 eigenvalues in ascending order.
 */
std::array<double, 5> jacobi_eigenvalues(std::array<double, 25> a) {
    constexpr int n = 5;
    for (int sweep = 0; sweep < 80; ++sweep) {
        // Find the largest-magnitude off-diagonal entry (p, q); this is the
        // pair the next Jacobi rotation will annihilate.
        int p = 0;
        int q = 1;
        double max_offdiag = 0.0;
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                const double value = std::abs(a[static_cast<std::size_t>(i * n + j)]);
                if (value > max_offdiag) {
                    max_offdiag = value;
                    p = i;
                    q = j;
                }
            }
        }
        if (max_offdiag < 1e-10) {
            // Off-diagonal entries are all negligible: the matrix is
            // effectively diagonal already, so the diagonal holds the
            // eigenvalues and we can stop early.
            break;
        }

        // Rotation angle that zeroes a[p][q]/a[q][p] (standard 2x2 Jacobi
        // rotation formula), then apply the rotation to rows/columns p, q.
        const double app = a[static_cast<std::size_t>(p * n + p)];
        const double aqq = a[static_cast<std::size_t>(q * n + q)];
        const double apq = a[static_cast<std::size_t>(p * n + q)];
        const double angle = 0.5 * std::atan2(2.0 * apq, aqq - app);
        const double c = std::cos(angle);
        const double s = std::sin(angle);

        for (int k = 0; k < n; ++k) {
            if (k == p || k == q) {
                continue;
            }
            const double akp = a[static_cast<std::size_t>(k * n + p)];
            const double akq = a[static_cast<std::size_t>(k * n + q)];
            const double new_kp = c * akp - s * akq;
            const double new_kq = s * akp + c * akq;
            a[static_cast<std::size_t>(k * n + p)] = new_kp;
            a[static_cast<std::size_t>(p * n + k)] = new_kp;
            a[static_cast<std::size_t>(k * n + q)] = new_kq;
            a[static_cast<std::size_t>(q * n + k)] = new_kq;
        }

        a[static_cast<std::size_t>(p * n + p)] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a[static_cast<std::size_t>(q * n + q)] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a[static_cast<std::size_t>(p * n + q)] = 0.0;
        a[static_cast<std::size_t>(q * n + p)] = 0.0;
    }

    // After convergence (or the sweep cap), the diagonal holds the
    // eigenvalues; clamp tiny negative noise to zero and sort ascending so
    // callers can read off sigma_min/sigma_max directly from the ends.
    std::array<double, 5> values{};
    for (int i = 0; i < n; ++i) {
        values[static_cast<std::size_t>(i)] =
            std::max(0.0, a[static_cast<std::size_t>(i * n + i)]);
    }
    std::sort(values.begin(), values.end());
    return values;
}

namespace {

/**
 * @brief One noisy range/bearing observation of a single beacon under the
 * rigorous global-frame "core" adaptive model used by
 * run_adaptive_localization(): `rv` is the measured robot-to-beacon range,
 * `rt` is the measured target-to-beacon range, and `u` is the (noisy) unit
 * bearing vector from the beacon toward the target.
 */
struct CoreMeasurement {
    double rv = 0.0;
    double rt = 0.0;
    Vec2 u;
};

/**
 * @brief Local-observability/Fisher-information summary for a single
 * beacon's 5-dimensional state (target x,y + beacon x,y,yaw), computed by
 * local_observability_metrics(). `rank` counts singular values of the
 * whitened Jacobian above a relative threshold; `sigma_min` (the paper's
 * S_v) is reported only when full rank 5 is achieved; `condition_number` is
 * sigma_max/sigma_min (or a large sentinel when rank-deficient); `logdet`
 * is the log-determinant of the Fisher information matrix (information
 * score used to drive the excitation controller); `trajectory_spread` is
 * the viewing-geometry scatter measure from trajectory_spread().
 */
struct ObservabilityMetrics {
    int rank = 0;
    double sigma_min = 0.0;
    double sigma_max = 0.0;
    double condition_number = 0.0;
    double logdet = 0.0;
    double trajectory_spread = 0.0;
};

/**
 * @brief Configuration for injecting synthetic measurement imperfections in
 * generate_stressed_local_measurements(): `dropout_probability` randomly
 * discards measurements (missed detections), `outlier_probability`
 * corrupts a measurement's range/bearing by a fixed magnitude with random
 * sign (gross sensor errors), and `use_fov`/`fov_half_angle` discard
 * measurements whose local bearing falls outside a field-of-view half
 * angle. All defaults are "no stress" (pass-through).
 */
struct MeasurementStress {
    double dropout_probability = 0.0;
    double outlier_probability = 0.0;
    double outlier_range_magnitude = 0.0;
    double outlier_bearing_magnitude = 0.0;
    double fov_half_angle = 0.0;
    bool use_fov = false;
};

/**
 * @brief Generates one noisy CoreMeasurement per beacon for the
 * continuous-time adaptive model: the target-bearing-from-beacon `u` is
 * corrupted with bearing noise, and both the robot-to-beacon range `rv` and
 * target-to-beacon range `rt` are corrupted with range noise (each clamped
 * to a minimum of 0.05 to avoid degenerate zero/negative ranges).
 */
std::vector<CoreMeasurement> measure_core_model(
    const World& world,
    const Vec2& robot,
    const Noise& noise,
    std::mt19937& rng) {
    std::vector<CoreMeasurement> measurements;
    measurements.reserve(world.beacons.size());

    for (const Vec2& beacon : world.beacons) {
        const double theta = bearing(beacon, world.target) + sample_noise(noise.bearing_sigma, rng);
        CoreMeasurement m;
        m.rv = std::max(0.05, norm(robot - beacon) + sample_noise(noise.range_sigma, rng));
        m.rt = std::max(0.05, norm(world.target - beacon) + sample_noise(noise.range_sigma, rng));
        m.u = unit_from_angle(theta);
        measurements.push_back(m);
    }
    return measurements;
}

/**
 * @brief Evaluates the continuous adaptive model's cost
 * sum_i 0.5 * epsilon_i^2, where
 * epsilon_i = (r_i^v)^2 - ||robot - target_estimate + r_i^t u_i||^2
 * is the per-beacon prediction-error ("innovation") signal from the paper's
 * adaptive law. Used only for logging/diagnostics; the adaptive update
 * itself (core_adaptive_update) does not follow this cost's gradient with
 * respect to the robot.
 */
double core_cost(
    const Vec2& robot,
    const Vec2& target_estimate,
    const std::vector<CoreMeasurement>& measurements) {
    double cost = 0.0;
    for (const auto& m : measurements) {
        const Vec2 v = robot - target_estimate + m.u * m.rt;
        const double eps = m.rv * m.rv - dot(v, v);
        cost += 0.5 * eps * eps;
    }
    return cost;
}

/**
 * @brief Computes phat_dot for the continuous adaptive law:
 * phat_dot = -2 * gamma * sum_i epsilon_i * (robot - target_estimate + r_i^t u_i)
 * where epsilon_i = (r_i^v)^2 - ||robot - target_estimate + r_i^t u_i||^2.
 * Callers (run_adaptive_localization) integrate this forward with a fixed
 * Euler step rather than solving a batch least-squares problem.
 */
Vec2 core_adaptive_update(
    const Vec2& robot,
    const Vec2& target_estimate,
    const std::vector<CoreMeasurement>& measurements,
    double gamma) {
    Vec2 sum{0.0, 0.0};
    for (const auto& m : measurements) {
        const Vec2 v = robot - target_estimate + m.u * m.rt;
        const double eps = m.rv * m.rv - dot(v, v);
        sum = sum + v * eps;
    }
    return sum * (-2.0 * gamma);
}

/**
 * @brief Derives a per-beacon position estimate from the current target
 * estimate and each beacon's measured bearing-to-target/range-to-target:
 * beacon_position = target_estimate - r_i^t * u_i. Used by the continuous
 * adaptive model, which never estimates beacon yaw (yaw is always reported
 * as 0.0 since this model has no notion of a beacon-local frame).
 */
std::vector<BeaconEstimate> core_beacon_estimates(
    const Vec2& target_estimate,
    const std::vector<CoreMeasurement>& measurements) {
    std::vector<BeaconEstimate> estimates;
    estimates.reserve(measurements.size());
    for (const auto& m : measurements) {
        estimates.push_back({target_estimate - m.u * m.rt, 0.0});
    }
    return estimates;
}

// beacon_position_rmse() and beacon_yaw_rmse() previously defined here now
// live in Estimators.cpp (declared in Estimators.hpp), shared with the
// ROS 2 / Gazebo closed-loop node rather than kept private to this
// translation unit.

/**
 * @brief Simple pass/fail accuracy gate for one trial: target error and
 * beacon position error must both be under 5 cm, and beacon yaw error (if
 * reported, i.e. not the -1.0 "not applicable" sentinel) must also be under
 * 0.05 rad. Distinct from the solver's own `converged` flag (which reflects
 * whether Gauss-Newton/EKF numerically settled, not whether the settled
 * answer was accurate).
 */
bool trial_accuracy_success(
    double target_error,
    double beacon_position_error,
    double beacon_yaw_error) {
    return target_error < 0.05 &&
        beacon_position_error < 0.05 &&
        (beacon_yaw_error < 0.0 || beacon_yaw_error < 0.05);
}

/**
 * @brief Packs the ground-truth World into the scenario-1 state layout
 * [target.x, target.y, beacon0.x, beacon0.y, beacon0.yaw, ...], i.e. the
 * value the estimator's state vector would take if it recovered the truth
 * exactly. Used as the state at which noiseless observability diagnostics
 * (local_observability_metrics, local_observability_rank_and_sigma_min) are
 * evaluated.
 */
std::vector<double> true_state_scenario1(const World& world) {
    std::vector<double> state{world.target.x, world.target.y};
    for (std::size_t i = 0; i < world.beacons.size(); ++i) {
        state.push_back(world.beacons[i].x);
        state.push_back(world.beacons[i].y);
        state.push_back(world.beacon_yaws[i]);
    }
    return state;
}

/**
 * @brief Builds the 5x5 normal/Fisher-information matrix J^T J for a
 * single beacon's state (target x,y + beacon x,y,yaw) from the whitened
 * scenario-1 Jacobian (jacobian_scenario1, beacon_count fixed to 1). This
 * is the core "gauge"/local-observability building block: its eigenvalues
 * (via jacobi_eigenvalues) give the squared singular values of the
 * Jacobian, which local_observability_metrics() turns into rank,
 * sigma_min/sigma_max, condition number, and log-determinant.
 *
 * NOTE: this routine (and everything downstream of it) is scoped to the
 * single-beacon scenario-1 model: the normal matrix is fixed at 5x5 and the
 * Jacobian is evaluated with beacon_count = 1. The guard below rejects any
 * other state size rather than silently producing metrics for the wrong
 * model.
 *
 * @param state        5-element state [target.x, target.y, beacon.x,
 *        beacon.y, beacon.yaw] at which the Jacobian is linearized.
 * @param path         Known vehicle path (only entries referenced by
 *        `measurements` matter).
 * @param measurements Local-frame measurements of the single beacon.
 * @param noise        Residual whitening; defaults to the estimator-default
 *        sigmas so the open-loop conditioning studies keep their published
 *        whitening, while the closed-loop supervisor passes its own noise
 *        level explicitly.
 * @return Row-major flattened 5x5 symmetric normal matrix, or the zero
 *         matrix (rank 0 / sigma_min 0 downstream) when `state` is not the
 *         5-dimensional single-beacon model.
 */
std::array<double, 25> normal_matrix_for_local_observability(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise = Noise{}) {
    constexpr int n = 5;
    if (state.size() != 5) {
        // Wrong model dimension: return the zero matrix, which downstream
        // reads as rank 0 / sigma_min 0 ("unobservable") -- a conservative,
        // visible failure instead of a silently wrong metric.
        return {};
    }
    const auto jacobian = jacobian_scenario1(state, 1, path, measurements, noise);
    std::array<double, 25> normal{};

    // Accumulate J^T J directly (only the upper triangle is computed, then
    // mirrored, since the normal matrix is symmetric by construction).
    for (int row = 0; row < n; ++row) {
        for (int col = row; col < n; ++col) {
            double value = 0.0;
            for (const auto& jacobian_row : jacobian) {
                value += jacobian_row[static_cast<std::size_t>(row)] *
                    jacobian_row[static_cast<std::size_t>(col)];
            }
            normal[static_cast<std::size_t>(row * n + col)] = value;
            normal[static_cast<std::size_t>(col * n + row)] = value;
        }
    }
    return normal;
}

/**
 * @brief Measures viewing-geometry diversity ("excitation") for each
 * beacon: the sum of squared deviations of the local vehicle-bearing
 * vectors (range/bearing-to-vehicle expressed in the beacon's own local
 * frame) about their per-beacon mean, summed over all beacons. A large
 * spread indicates the vehicle was observed from many different relative
 * angles/ranges (good for self-calibration); a near-zero spread indicates
 * a near-stationary or collinear/degenerate viewing history.
 *
 * The supervisor's path-based certificate S_v now lives in Math.hpp as
 * adaptive::path_spread, shared with the ROS 2 / Gazebo closed-loop node.
 * This measurement-derived trajectory_spread() is retained only for the
 * open-loop conditioning diagnostics.
 *
 * @param measurements Local-frame measurements (possibly for multiple
 *        beacons; each is binned into `measurement.beacon`).
 * @param beacon_count Number of beacons to bin over; measurements
 *        referencing an out-of-range beacon index are skipped.
 * @return 0.0 if there are no measurements or beacons; otherwise the total
 *         scatter (sum over beacons of the per-beacon squared-deviation
 *         sum).
 */
double trajectory_spread(const std::vector<LocalFrameMeasurement>& measurements, int beacon_count) {
    if (measurements.empty() || beacon_count <= 0) {
        return 0.0;
    }
    std::vector<Vec2> sums(static_cast<std::size_t>(beacon_count));
    std::vector<int> counts(static_cast<std::size_t>(beacon_count), 0);
    // First pass: accumulate the mean local vehicle-bearing vector per beacon.
    for (const auto& measurement : measurements) {
        if (measurement.beacon >= sums.size()) {
            continue;
        }
        const Vec2 local_vehicle =
            unit_from_angle(measurement.bv_local) * measurement.rv;
        sums[measurement.beacon] = sums[measurement.beacon] + local_vehicle;
        counts[measurement.beacon] += 1;
    }
    std::vector<Vec2> means(static_cast<std::size_t>(beacon_count));
    for (int i = 0; i < beacon_count; ++i) {
        if (counts[static_cast<std::size_t>(i)] > 0) {
            means[static_cast<std::size_t>(i)] =
                sums[static_cast<std::size_t>(i)] / static_cast<double>(counts[static_cast<std::size_t>(i)]);
        }
    }
    // Second pass: sum squared deviations from each beacon's mean (a
    // scatter/variance measure of viewing-geometry diversity over time).
    double spread = 0.0;
    for (const auto& measurement : measurements) {
        if (measurement.beacon >= means.size()) {
            continue;
        }
        const Vec2 local_vehicle =
            unit_from_angle(measurement.bv_local) * measurement.rv;
        const Vec2 centered = local_vehicle - means[measurement.beacon];
        spread += dot(centered, centered);
    }
    return spread;
}

}  // namespace

// See Simulation.hpp for the full contract. Exported (rather than kept
// file-private) because the ROS 2 / Gazebo closed-loop node logs the same
// conditioning diagnostic the batch simulator does. The default noise
// argument lives on the header declaration.
std::pair<int, double> local_observability_rank_and_sigma_min(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise) {
    const auto metrics = [&]() {
        const auto eigenvalues = jacobi_eigenvalues(
            normal_matrix_for_local_observability(state, path, measurements, noise));
        const double largest = std::sqrt(eigenvalues.back());
        const double threshold = std::max(1e-6, largest * 1e-7);
        int rank = 0;
        double smallest_positive = 0.0;
        // eigenvalues is sorted ascending, so the first singular value that
        // clears the threshold is the smallest observed one.
        for (double eigenvalue : eigenvalues) {
            const double singular_value = std::sqrt(std::max(0.0, eigenvalue));
            if (singular_value > threshold) {
                if (rank == 0) {
                    smallest_positive = singular_value;
                }
                ++rank;
            }
        }
        return std::make_pair(rank, rank == 5 ? smallest_positive : 0.0);
    }();
    return metrics;
}

namespace {

/**
 * @brief Full local-observability/Fisher-information diagnostic for a
 * single beacon: diagonalizes the normal matrix (normal_matrix_for_local_
 * observability + jacobi_eigenvalues) and derives rank, sigma_min (S_v,
 * valid only at full rank 5), sigma_max, condition_number
 * (sigma_max/sigma_min, or the sentinel 1e12 when rank-deficient), the
 * information-theoretic logdet score (sum of log(eigenvalue), floored at
 * 1e-18 to avoid log(0) — this is the objective the information-driven
 * excitation controller greedily climbs), and trajectory_spread (viewing
 * geometry diversity).
 */
ObservabilityMetrics local_observability_metrics(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements) {
    const auto eigenvalues = jacobi_eigenvalues(
        normal_matrix_for_local_observability(state, path, measurements));
    const double largest = std::sqrt(eigenvalues.back());
    const double threshold = std::max(1e-6, largest * 1e-7);
    ObservabilityMetrics metrics;
    metrics.sigma_max = largest;
    metrics.logdet = 0.0;
    metrics.trajectory_spread = trajectory_spread(measurements, 1);
    double smallest_positive = 0.0;
    for (double eigenvalue : eigenvalues) {
        const double singular_value = std::sqrt(std::max(0.0, eigenvalue));
        metrics.logdet += std::log(std::max(eigenvalue, 1e-18));
        if (singular_value > threshold) {
            if (metrics.rank == 0) {
                smallest_positive = singular_value;
            }
            ++metrics.rank;
        }
    }
    metrics.sigma_min = metrics.rank == 5 ? smallest_positive : 0.0;
    metrics.condition_number =
        metrics.sigma_min > 0.0 ? metrics.sigma_max / metrics.sigma_min : 1e12;
    return metrics;
}

/**
 * @brief Predicts what a LocalFrameMeasurement of `beacon_estimate` would
 * look like from `robot`, given the current `target_estimate`: forward
 * range/bearing model evaluated at the estimated (not true) beacon pose.
 * Used by predicted_local_logdet_score() to synthesize a hypothetical
 * "next" measurement for a candidate robot position, without needing an
 * actual sensor reading.
 */
LocalFrameMeasurement predicted_local_frame_measurement(
    const Vec2& robot,
    const Vec2& target_estimate,
    const BeaconEstimate& beacon_estimate,
    std::size_t beacon_index,
    std::size_t time_index) {
    LocalFrameMeasurement measurement;
    measurement.beacon = beacon_index;
    measurement.time = time_index;
    measurement.rv = std::max(0.05, norm(robot - beacon_estimate.position));
    measurement.bv_local = wrap_angle(
        bearing(beacon_estimate.position, robot) - beacon_estimate.yaw);
    measurement.rt = std::max(0.05, norm(target_estimate - beacon_estimate.position));
    measurement.bt_local = wrap_angle(
        bearing(beacon_estimate.position, target_estimate) - beacon_estimate.yaw);
    return measurement;
}

/**
 * @brief Evaluates what the local-observability logdet score would become
 * if the robot took one additional hypothetical measurement from
 * `candidate_robot` (single-beacon case only). Appends a predicted
 * measurement (predicted_local_frame_measurement) to the existing history
 * and re-runs local_observability_metrics() on the augmented path. This is
 * the scalar objective information_driven_excitation() differentiates via
 * finite differences to choose an excitation direction.
 *
 * @return -1e18 (an effectively -infinity sentinel) if the single-beacon
 *         preconditions (5-element state, exactly one beacon estimate)
 *         are not met.
 */
double predicted_local_logdet_score(
    const Vec2& candidate_robot,
    const Vec2& target_estimate,
    const std::vector<double>& state,
    const std::vector<BeaconEstimate>& beacon_estimates,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements) {
    if (state.size() != 5 || beacon_estimates.size() != 1) {
        return -1e18;
    }

    std::vector<Vec2> candidate_path = path;
    candidate_path.push_back(candidate_robot);

    std::vector<LocalFrameMeasurement> candidate_measurements = measurements;
    candidate_measurements.push_back(predicted_local_frame_measurement(
        candidate_robot,
        target_estimate,
        beacon_estimates.front(),
        0,
        candidate_path.size() - 1U));

    return local_observability_metrics(state, candidate_path, candidate_measurements).logdet;
}

/**
 * @brief Computes the excitation velocity for
 * ClosedLoopExcitationMode::Information: a central finite-difference
 * gradient of predicted_local_logdet_score() with respect to the robot's
 * (x, y) position, scaled by a decaying exploration envelope so early
 * steps explore more aggressively and later steps settle down (mirroring
 * the amplitude decay of the fixed circular-swirl schedule).
 *
 * @param robot            Current robot position (gradient is evaluated
 *        around this point).
 * @param target_estimate  Current target-position estimate.
 * @param state            Current scenario-1 state (must be exactly 5
 *        elements: single-beacon case).
 * @param beacon_estimates Current beacon estimate(s) (must be exactly one).
 * @param path             Measurement history's vehicle path so far.
 * @param measurements     Measurement history so far.
 * @param config           Supplies `information_gradient_step` (finite
 *        difference step h), `exploration_amplitude`/`exploration_decay`
 *        (envelope cap), and `information_exploration_gain` (gradient
 *        scale).
 * @param elapsed_time     Physical time elapsed since the loop started
 *        (seconds), used to decay the envelope over time.
 * @return {0,0} if preconditions fail, the gradient is non-finite, or the
 *         gradient norm is negligible (< 1e-9); otherwise a vector along
 *         the ascent direction of the logdet score, magnitude-capped by the
 *         decaying envelope.
 */
Vec2 information_driven_excitation(
    const Vec2& robot,
    const Vec2& target_estimate,
    const std::vector<double>& state,
    const std::vector<BeaconEstimate>& beacon_estimates,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const SimulationConfig& config,
    double elapsed_time) {
    if (path.empty() || state.size() != 5 || beacon_estimates.size() != 1) {
        return {0.0, 0.0};
    }

    // Central finite-difference gradient of the predicted logdet score
    // w.r.t. the candidate robot position (x then y), step size h.
    const double h = std::max(1e-4, config.information_gradient_step);
    const double sx_plus = predicted_local_logdet_score(
        {robot.x + h, robot.y}, target_estimate, state, beacon_estimates, path, measurements);
    const double sx_minus = predicted_local_logdet_score(
        {robot.x - h, robot.y}, target_estimate, state, beacon_estimates, path, measurements);
    const double sy_plus = predicted_local_logdet_score(
        {robot.x, robot.y + h}, target_estimate, state, beacon_estimates, path, measurements);
    const double sy_minus = predicted_local_logdet_score(
        {robot.x, robot.y - h}, target_estimate, state, beacon_estimates, path, measurements);

    Vec2 gradient{
        (sx_plus - sx_minus) / (2.0 * h),
        (sy_plus - sy_minus) / (2.0 * h),
    };
    const double gradient_norm = norm(gradient);
    if (!std::isfinite(gradient_norm) || gradient_norm < 1e-9) {
        return {0.0, 0.0};
    }

    // Cap the excitation magnitude by a decaying envelope (same shape as
    // the fixed circular schedule) so early steps explore more and the
    // controller settles down over time even if the gradient stays large.
    const double envelope =
        config.exploration_amplitude *
        std::exp(-config.exploration_decay * elapsed_time);
    const double magnitude = std::min(
        envelope,
        config.information_exploration_gain * gradient_norm);
    return gradient * (magnitude / gradient_norm);
}

/**
 * @brief Forward measurement model used by the EKF (run_ekf_local_frame_trial):
 * given the current scenario-1 state estimate and the known robot position,
 * predicts the 4-vector [rv, bv_local, rt, bt_local] that a
 * LocalFrameMeasurement of `measurement.beacon` would report. Distinct from
 * predicted_local_frame_measurement() in that it reads the beacon
 * pose/target directly out of the flattened state vector rather than a
 * BeaconEstimate struct, since the EKF represents the whole joint state
 * (not just one beacon) as a single vector.
 */
std::vector<double> local_frame_measurement_prediction(
    const std::vector<double>& state,
    const Vec2& robot,
    const LocalFrameMeasurement& measurement) {
    const std::size_t base = 2 + 3 * measurement.beacon;
    const Vec2 target{state[0], state[1]};
    const Vec2 beacon{state[base], state[base + 1]};
    const double yaw = state[base + 2];

    const Vec2 vehicle_local = rotate(robot - beacon, -yaw);
    const Vec2 target_local = rotate(target - beacon, -yaw);
    return {
        norm(vehicle_local),
        std::atan2(vehicle_local.y, vehicle_local.x),
        norm(target_local),
        std::atan2(target_local.y, target_local.x),
    };
}

/** @brief Flattens a LocalFrameMeasurement into the same [rv, bv_local, rt,
 *  bt_local] ordering produced by local_frame_measurement_prediction(), so
 *  predicted and observed measurements can be subtracted element-wise to
 *  form the EKF innovation. */
std::vector<double> local_frame_measurement_vector(const LocalFrameMeasurement& measurement) {
    return {
        measurement.rv,
        measurement.bv_local,
        measurement.rt,
        measurement.bt_local,
    };
}

/**
 * @brief Generates local-frame measurements for every (time, beacon) pair
 * along `path`, applying the MeasurementStress pipeline on top of ordinary
 * sensor noise: field-of-view culling (discard if either local bearing
 * exceeds `fov_half_angle`), then Bernoulli dropout (discard entirely,
 * simulating a missed detection), then Bernoulli-triggered outlier
 * corruption (adds/subtracts a fixed range/bearing magnitude with a random
 * sign, simulating a gross sensor error rather than ordinary Gaussian
 * noise). The dropout/FOV/outlier decisions and signs are all drawn from
 * `rng` in a fixed order, so results are reproducible for a given seed.
 */
std::vector<LocalFrameMeasurement> generate_stressed_local_measurements(
    const World& world,
    const std::vector<Vec2>& path,
    const Noise& noise,
    const MeasurementStress& stress,
    std::mt19937& rng) {
    std::vector<LocalFrameMeasurement> measurements;
    measurements.reserve(path.size() * world.beacons.size());
    std::bernoulli_distribution dropout(stress.dropout_probability);
    std::bernoulli_distribution outlier(stress.outlier_probability);
    std::bernoulli_distribution sign(0.5);

    for (std::size_t time = 0; time < path.size(); ++time) {
        for (std::size_t beacon = 0; beacon < world.beacons.size(); ++beacon) {
            // Start from an ordinary noisy measurement, then apply stress.
            auto measurement = make_local_frame_measurement(
                world, path[time], beacon, time, noise, rng);

            // Field-of-view gate: discard if either local bearing (to the
            // vehicle or to the target) falls outside the sensor's half
            // angle.
            if (stress.use_fov &&
                (std::abs(wrap_angle(measurement.bv_local)) > stress.fov_half_angle ||
                 std::abs(wrap_angle(measurement.bt_local)) > stress.fov_half_angle)) {
                continue;
            }
            // Dropout: simulate a missed detection by discarding the
            // measurement entirely (Bernoulli draw).
            if (dropout(rng)) {
                continue;
            }
            // Outlier: corrupt range/bearing by a fixed magnitude with a
            // random sign (gross sensor error), applied with opposite signs
            // to the vehicle- and target-range/bearing terms so the
            // corruption isn't simply cancelled out downstream.
            if (outlier(rng)) {
                const double range_sign = sign(rng) ? 1.0 : -1.0;
                const double bearing_sign = sign(rng) ? 1.0 : -1.0;
                measurement.rv = std::max(0.05, measurement.rv + range_sign * stress.outlier_range_magnitude);
                measurement.rt = std::max(0.05, measurement.rt - range_sign * stress.outlier_range_magnitude);
                measurement.bv_local =
                    wrap_angle(measurement.bv_local + bearing_sign * stress.outlier_bearing_magnitude);
                measurement.bt_local =
                    wrap_angle(measurement.bt_local - bearing_sign * stress.outlier_bearing_magnitude);
            }
            measurements.push_back(measurement);
        }
    }
    return measurements;
}

/**
 * @brief Returns a copy of `path` with independent zero-mean Gaussian noise
 * (std-dev `position_sigma`) added to each x/y coordinate, modeling
 * imperfect vehicle self-localization ("known" path used by the estimator
 * differs from the true path used to generate measurements). Returns
 * `path` unchanged when `position_sigma <= 0`.
 */
std::vector<Vec2> make_noisy_path(
    const std::vector<Vec2>& path,
    double position_sigma,
    std::mt19937& rng) {
    if (position_sigma <= 0.0) {
        return path;
    }
    std::normal_distribution<double> noise(0.0, position_sigma);
    std::vector<Vec2> noisy_path;
    noisy_path.reserve(path.size());
    for (const Vec2& p : path) {
        noisy_path.push_back({p.x + noise(rng), p.y + noise(rng)});
    }
    return noisy_path;
}

/**
 * @brief Returns a copy of `path` perturbed by an accumulating random-walk
 * drift: at each step, Gaussian noise scaled by `drift_fraction` times the
 * true step length is added to a running drift offset, which is then
 * applied to that step's position. Models dead-reckoning-style drift in
 * the vehicle's self-localization estimate, as opposed to the iid noise of
 * make_noisy_path(). The first path point is left unperturbed (drift starts
 * at zero). Returns `path` unchanged when `drift_fraction <= 0` or `path`
 * is empty.
 */
std::vector<Vec2> make_drifted_path(
    const std::vector<Vec2>& path,
    double drift_fraction,
    std::mt19937& rng) {
    if (drift_fraction <= 0.0 || path.empty()) {
        return path;
    }
    std::normal_distribution<double> unit_noise(0.0, 1.0);
    std::vector<Vec2> drifted_path;
    drifted_path.reserve(path.size());
    Vec2 drift{0.0, 0.0};
    drifted_path.push_back(path.front());
    for (std::size_t k = 1; k < path.size(); ++k) {
        const double step_length = norm(path[k] - path[k - 1U]);
        const double sigma = drift_fraction * step_length;
        drift = drift + Vec2{sigma * unit_noise(rng), sigma * unit_noise(rng)};
        drifted_path.push_back(path[k] + drift);
    }
    return drifted_path;
}

/**
 * @brief Applies a Huber robust-loss reweighting to a vector of whitened
 * residuals: residuals within `delta` are left unchanged (behaving as
 * ordinary least squares), while residuals beyond `delta` are soft-clipped
 * to sqrt(delta * |residual|) (sign-preserved), so squaring this in the
 * Gauss-Newton cost reproduces the Huber loss's linear (rather than
 * quadratic) growth for large residuals. This is what "robust" estimator
 * variants use in place of huber_delta &lt;= 0 (plain least squares).
 *
 * @param residuals Whitened residual vector (modified copy is returned).
 * @param delta     Huber transition point; passing <= 0 disables robust
 *        reweighting and returns `residuals` unchanged.
 */
std::vector<double> huber_scaled_residuals(
    std::vector<double> residuals,
    double delta) {
    if (delta <= 0.0) {
        return residuals;
    }
    for (double& residual : residuals) {
        const double magnitude = std::abs(residual);
        if (magnitude > delta) {
            residual = std::copysign(std::sqrt(delta * magnitude), residual);
        }
    }
    return residuals;
}

// identity_matrix() and the dense linear solve previously defined here now
// live in Matrix.hpp/Matrix.cpp as identity_matrix() and
// solve_dense_linear_system(..., SingularPivotPolicy::ReturnZero) --
// shared with Solver.cpp's Gauss-Newton solve, which uses the same
// elimination with SingularPivotPolicy::RegularizeDiagonal instead. See
// Matrix.hpp for why the two singular-pivot behaviors differ.

/**
 * @brief Approximates the EKF's initial state covariance from the
 * whitened information matrix J^T J of the full multi-beacon scenario-1
 * Jacobian (not the single-beacon 5x5 gauge matrix), inverting it via
 * solve_dense_linear_system() one basis column at a time. A small ridge
 * (1e-6) is added to the diagonal before inversion to keep the matrix
 * numerically invertible even when some directions are weakly observed.
 * Used only when seeding the EKF from the two-view closed-form initializer
 * (two_view_closed_form_initial_state), to give the filter a covariance
 * consistent with the information actually available at seed time rather
 * than an arbitrary generic prior.
 */
std::vector<std::vector<double>> covariance_from_local_information(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise) {
    const auto jacobian = jacobian_scenario1(state, 1, path, measurements, noise);
    const int state_dim = static_cast<int>(state.size());
    Matrix normal = gram_matrix(jacobian);
    for (int i = 0; i < state_dim; ++i) {
        normal[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] += 1e-6;
    }

    std::vector<std::vector<double>> covariance(
        static_cast<std::size_t>(state_dim),
        std::vector<double>(static_cast<std::size_t>(state_dim), 0.0));
    for (int col = 0; col < state_dim; ++col) {
        std::vector<double> rhs(static_cast<std::size_t>(state_dim), 0.0);
        rhs[static_cast<std::size_t>(col)] = 1.0;
        const auto solution = solve_dense_linear_system(normal, rhs, SingularPivotPolicy::ReturnZero);
        for (int row = 0; row < state_dim; ++row) {
            covariance[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
                solution[static_cast<std::size_t>(row)];
        }
    }
    return covariance;
}

/**
 * @brief Computes the EKF measurement Jacobian H = d(prediction)/d(state)
 * for one LocalFrameMeasurement via central finite differences (step
 * 1e-6) on local_frame_measurement_prediction(), wrapping the bearing rows
 * (1 and 3, i.e. bv_local and bt_local) to avoid a spurious large
 * derivative across the +-pi discontinuity. Used in place of an analytic
 * Jacobian because the EKF only touches one beacon's block of the state at
 * a time but must still differentiate against the full state vector.
 *
 * @return A 4 x state_dim matrix (rows: rv, bv_local, rt, bt_local).
 */
std::vector<std::vector<double>> finite_difference_local_measurement_jacobian(
    const std::vector<double>& state,
    const Vec2& robot,
    const LocalFrameMeasurement& measurement) {
    const int measurement_dim = 4;
    const int state_dim = static_cast<int>(state.size());
    const double h = 1e-6;
    std::vector<std::vector<double>> jacobian(
        static_cast<std::size_t>(measurement_dim),
        std::vector<double>(static_cast<std::size_t>(state_dim), 0.0));

    for (int col = 0; col < state_dim; ++col) {
        std::vector<double> plus = state;
        std::vector<double> minus = state;
        plus[static_cast<std::size_t>(col)] += h;
        minus[static_cast<std::size_t>(col)] -= h;
        const auto z_plus = local_frame_measurement_prediction(plus, robot, measurement);
        const auto z_minus = local_frame_measurement_prediction(minus, robot, measurement);
        for (int row = 0; row < measurement_dim; ++row) {
            double diff = z_plus[static_cast<std::size_t>(row)] - z_minus[static_cast<std::size_t>(row)];
            if (row == 1 || row == 3) {
                diff = wrap_angle(diff);
            }
            jacobian[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] = diff / (2.0 * h);
        }
    }
    return jacobian;
}

/**
 * @brief Runs the EKF (Extended Kalman Filter) alternative to the batch
 * Gauss-Newton scenario-1 solve: instead of solving the whole nonlinear
 * least-squares problem at once, this ingests `measurements` one at a time
 * in generation order, updating a state estimate and covariance matrix
 * after every single measurement using a finite-difference measurement
 * Jacobian and the standard EKF (Joseph-form) gain/covariance update. A
 * small fixed process-noise diagonal (1e-5) is injected before every
 * update, treating the state as a slow random walk to keep the filter from
 * becoming overconfident/inconsistent over many updates.
 *
 * @param beacon_count              Number of beacons (only beacon_count==1
 *        supports the two-view closed-form seed).
 * @param trial                     Trial index recorded in the result.
 * @param world                     Ground-truth world.
 * @param path                      Known vehicle path.
 * @param config                    Supplies the initial seed
 *        (`initial_target_estimate`, `initial_beacon_guess_radius/yaw`).
 * @param noise                     Measurement noise model (defines the
 *        fixed per-measurement covariance `measurement_variance`).
 * @param use_two_view_initialization When true and `beacon_count == 1`,
 *        seeds the filter from two_view_closed_form_initial_state() with a
 *        covariance derived from the local information matrix
 *        (covariance_from_local_information) instead of the generic
 *        circular seed; this is scenario 4 vs. scenario 3.
 * @param rng                       Random engine used to draw measurement
 *        noise when generating `measurements`.
 * @return A TrialResult with scenario set to 3 (generic seed) or 4
 *         (two-view seed), `solver_converged` always true (the EKF has no
 *         notion of non-convergence), and `converged` set by
 *         trial_accuracy_success() against the final EKF state.
 */
TrialResult run_ekf_local_frame_trial(
    int beacon_count,
    int trial,
    const World& world,
    const std::vector<Vec2>& path,
    const SimulationConfig& config,
    const Noise& noise,
    bool use_two_view_initialization,
    std::mt19937& rng) {
    const int scenario = use_two_view_initialization ? 4 : 3;
    const auto measurements = generate_local_frame_measurements(world, path, noise, rng);
    const auto start = std::chrono::steady_clock::now();

    // Default seed: generic coarse circular beacon layout (same seed used
    // by the batch solver), with a wide diagonal covariance (25 for
    // position, 9 for yaw) reflecting how little is known a priori.
    std::vector<double> state = initial_state_scenario1(
        beacon_count,
        config.initial_target_estimate,
        config.initial_beacon_guess_radius,
        config.initial_beacon_guess_yaw);
    const int state_dim = static_cast<int>(state.size());
    std::vector<std::vector<double>> covariance = identity_matrix(state_dim);
    for (int i = 0; i < state_dim; ++i) {
        const bool is_yaw = i >= 4 && ((i - 4) % 3 == 0);
        covariance[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
            is_yaw ? 9.0 : 25.0;
    }
    // Optional constructive seed: replace the generic seed/covariance with
    // the two-view closed-form estimate and an information-derived
    // covariance (scenario 4), falling back to the generic seed above if
    // no sufficiently-excited view pair exists.
    if (use_two_view_initialization && beacon_count == 1) {
        std::vector<double> closed_form_state;
        if (two_view_closed_form_initial_state(beacon_count, path, measurements, closed_form_state)) {
            state = closed_form_state;
            covariance = covariance_from_local_information(state, path, measurements, noise);
            for (int i = 0; i < state_dim; ++i) {
                covariance[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
                    std::max(covariance[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)], 1e-4);
            }
        }
    }

    // Fixed per-measurement-component noise variance (falls back to the
    // scenario defaults 0.03/0.006 when noise sigmas are non-positive,
    // matching effective_range_sigma/effective_bearing_sigma in
    // Estimators.cpp).
    const double range_variance =
        std::pow(noise.range_sigma > 0.0 ? noise.range_sigma : 0.03, 2.0);
    const double bearing_variance =
        std::pow(noise.bearing_sigma > 0.0 ? noise.bearing_sigma : 0.006, 2.0);
    const std::array<double, 4> measurement_variance{
        range_variance, bearing_variance, range_variance, bearing_variance};

    // Sequential EKF update: process one measurement at a time in
    // generation order (i.e. time-then-beacon order from
    // generate_local_frame_measurements).
    for (const auto& measurement : measurements) {
        // Process-noise injection: treat the state as a slow random walk so
        // the filter's confidence never collapses to zero even after many
        // updates (keeps it from becoming inconsistently overconfident).
        for (int i = 0; i < state_dim; ++i) {
            covariance[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] += 1e-5;
        }

        // Innovation: observed minus predicted measurement, with the two
        // bearing components wrapped to (-pi, pi] to avoid a spurious large
        // innovation across the +-pi discontinuity.
        const Vec2& robot = path[measurement.time];
        const auto predicted = local_frame_measurement_prediction(state, robot, measurement);
        const auto observed = local_frame_measurement_vector(measurement);
        std::array<double, 4> innovation{};
        for (int row = 0; row < 4; ++row) {
            innovation[static_cast<std::size_t>(row)] =
                observed[static_cast<std::size_t>(row)] - predicted[static_cast<std::size_t>(row)];
            if (row == 1 || row == 3) {
                innovation[static_cast<std::size_t>(row)] =
                    wrap_angle(innovation[static_cast<std::size_t>(row)]);
            }
        }

        // Innovation covariance S = H P H^T + R (R = measurement_variance
        // on the diagonal).
        const auto h_matrix = finite_difference_local_measurement_jacobian(state, robot, measurement);
        Matrix innovation_covariance = sandwich(h_matrix, covariance);
        for (int row = 0; row < 4; ++row) {
            innovation_covariance[static_cast<std::size_t>(row)][static_cast<std::size_t>(row)] +=
                measurement_variance[static_cast<std::size_t>(row)];
        }

        // Kalman gain K = P H^T S^-1: first P H^T (state_dim x 4), then
        // column-by-column solve S^T k_row = e_row for each basis vector (S
        // is symmetric, so this is equivalent to solving against S
        // directly).
        const Matrix p_ht = matmul(covariance, transpose(h_matrix));
        std::vector<std::vector<double>> kalman_gain(
            static_cast<std::size_t>(state_dim), std::vector<double>(4, 0.0));
        for (int i = 0; i < state_dim; ++i) {
            for (int row = 0; row < 4; ++row) {
                std::vector<double> basis(4, 0.0);
                basis[static_cast<std::size_t>(row)] = 1.0;
                const auto column =
                    solve_dense_linear_system(innovation_covariance, basis, SingularPivotPolicy::ReturnZero);
                for (int k = 0; k < 4; ++k) {
                    kalman_gain[static_cast<std::size_t>(i)][static_cast<std::size_t>(row)] +=
                        p_ht[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)] *
                        column[static_cast<std::size_t>(k)];
                }
            }
        }

        // State update: x <- x + K * innovation.
        for (int i = 0; i < state_dim; ++i) {
            double dx = 0.0;
            for (int row = 0; row < 4; ++row) {
                dx += kalman_gain[static_cast<std::size_t>(i)][static_cast<std::size_t>(row)] *
                    innovation[static_cast<std::size_t>(row)];
            }
            state[static_cast<std::size_t>(i)] += dx;
        }

        // Covariance update in Joseph ("stabilized") form:
        //   P <- (I - K H) P (I - K H)^T + K R K^T
        // which is more numerically robust to a slightly wrong/asymmetric
        // gain than the simpler P <- (I - K H) P update.
        const Matrix kh = matmul(kalman_gain, h_matrix);
        Matrix identity_minus_kh = identity_matrix(state_dim);
        for (int i = 0; i < state_dim; ++i) {
            for (int j = 0; j < state_dim; ++j) {
                identity_minus_kh[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] -=
                    kh[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
            }
        }

        const Matrix left_product = matmul(identity_minus_kh, covariance);
        Matrix updated = matmul(left_product, transpose(identity_minus_kh));
        for (int i = 0; i < state_dim; ++i) {
            for (int j = 0; j < state_dim; ++j) {
                for (int row = 0; row < 4; ++row) {
                    updated[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] +=
                        kalman_gain[static_cast<std::size_t>(i)][static_cast<std::size_t>(row)] *
                        measurement_variance[static_cast<std::size_t>(row)] *
                        kalman_gain[static_cast<std::size_t>(j)][static_cast<std::size_t>(row)];
                }
            }
        }
        covariance = updated;
    }

    const auto stop = std::chrono::steady_clock::now();
    const Vec2 estimate{state[0], state[1]};
    const auto beacon_estimates = beacon_estimates_from_scenario1_state(state, beacon_count);
    const auto final_residuals = residuals_scenario1(state, beacon_count, path, measurements, noise);
    double cost = 0.0;
    for (double residual : final_residuals) {
        cost += 0.5 * residual * residual;
    }

    const double target_error = norm(estimate - world.target);
    const double beacon_position_error = beacon_position_rmse(world, beacon_estimates);
    const double beacon_yaw_error = beacon_yaw_rmse(world, beacon_estimates);
    const double runtime_ms = std::chrono::duration<double, std::milli>(stop - start).count();
    return {scenario, beacon_count, trial, world.target, estimate, target_error,
            beacon_position_error, beacon_yaw_error,
            cost, static_cast<int>(measurements.size()), runtime_ms, true,
            trial_accuracy_success(target_error, beacon_position_error, beacon_yaw_error)};
}

/** @brief Elapsed wall-clock time in milliseconds between two steady_clock
 *  time points; used throughout to fill each TrialResult's `runtime_ms`. */
double elapsed_ms(
    const std::chrono::steady_clock::time_point& start,
    const std::chrono::steady_clock::time_point& stop) {
    return std::chrono::duration<double, std::milli>(stop - start).count();
}

/**
 * @brief 95% confidence half-width for the sample mean of a scalar
 * quantity, given only running sums (sum of values, sum of squared values)
 * and the sample count, using the normal approximation (1.96 * standard
 * error). Returns 0.0 when fewer than 2 samples are available (sample
 * variance is undefined). Used for the CI columns on SummaryRow's mean
 * error and mean beacon RMSE fields.
 */
double mean_ci95_from_sums(double sum, double sum2, int count) {
    if (count < 2) {
        return 0.0;
    }
    const double mean = sum / static_cast<double>(count);
    const double variance =
        std::max(0.0, (sum2 - static_cast<double>(count) * mean * mean) /
                          static_cast<double>(count - 1));
    return 1.96 * std::sqrt(variance / static_cast<double>(count));
}

/**
 * @brief 95% confidence half-width for an RMSE statistic (sqrt of a mean
 * squared error), propagated from the delta-method approximation: the CI
 * on the mean squared error (derived from sum_error2, sum_error4, the sums
 * of error^2 and error^4) is halved and divided by the RMSE itself, since
 * d(sqrt(x))/dx = 1/(2*sqrt(x)). Returns 0.0 when fewer than 2 samples are
 * available or the mean squared error is non-positive.
 */
double rmse_ci95_from_sums(double sum_error2, double sum_error4, int count) {
    if (count < 2 || sum_error2 <= 0.0) {
        return 0.0;
    }
    const double mean_square = sum_error2 / static_cast<double>(count);
    const double variance_square =
        std::max(0.0, (sum_error4 - static_cast<double>(count) * mean_square * mean_square) /
                          static_cast<double>(count - 1));
    const double rmse = std::sqrt(mean_square);
    const double se_mean_square = std::sqrt(variance_square / static_cast<double>(count));
    return 1.96 * se_mean_square / (2.0 * rmse);
}

/**
 * @brief Core single-solve scenario-1 batch trial: given an already-drawn
 * set of measurements and an initial seed, optionally overrides the seed
 * with the two-view closed-form estimate, optionally runs a robust
 * "warm-start" pass, then runs the final damped Gauss-Newton solve
 * (analytic Jacobian for the plain solve, finite-difference Jacobian for
 * the robust solve since Huber-reweighted residuals have no closed-form
 * Jacobian here) and packages the result into a TrialResult. This is the
 * common implementation shared by run_local_batch_trial_with_seed(),
 * run_multistart_local_batch_trial(), and most of the robustness sweeps.
 *
 * @param trial                 Trial index recorded in the result.
 * @param beacon_count          Number of beacons.
 * @param world                 Ground-truth world (for error computation).
 * @param estimator_path        Vehicle path used by the estimator (may
 *        differ from the true path that generated `measurements`, e.g.
 *        under vehicle-localization noise).
 * @param measurements          Pre-generated (possibly stressed) local-frame
 *        measurements.
 * @param config                Supplies `batch_solver_max_iterations` /
 *        `batch_solver_initial_lambda`.
 * @param residual_noise        Noise model used to whiten residuals (should
 *        match the noise the measurements were generated with, for a
 *        correctly-scaled cost).
 * @param initial_state         Fallback seed used when
 *        `use_closed_form_seed` is false or the closed-form seed fails.
 * @param robust                If true, reweights residuals with
 *        huber_scaled_residuals(robust_delta) and first runs a shorter
 *        "warm start" solve at half the iteration budget with the analytic
 *        Jacobian and un-reweighted residuals before the final robust
 *        solve (helps the robust solve start from a decent basin).
 * @param robust_delta          Huber transition point (see
 *        huber_scaled_residuals).
 * @param use_closed_form_seed  Whether to attempt overriding `initial_state`
 *        with two_view_closed_form_initial_state().
 * @param repeat_target_packets Forwarded to residuals_scenario1/
 *        jacobian_scenario1; when false, only the first target packet per
 *        beacon is used (see "single_target_packet_batch_gn" in
 *        run_expanded_baseline_comparison).
 * @return TrialResult with scenario fixed to 1; `iterations` is the sum of
 *         the warm-start and final solve's iteration counts.
 */
TrialResult run_local_batch_trial_with_measurements(
    int trial,
    int beacon_count,
    const World& world,
    const std::vector<Vec2>& estimator_path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const SimulationConfig& config,
    const Noise& residual_noise,
    const std::vector<double>& initial_state,
    bool robust,
    double robust_delta,
    bool use_closed_form_seed = true,
    bool repeat_target_packets = true) {
    const auto start = std::chrono::steady_clock::now();
    std::vector<double> seed = initial_state;
    if (use_closed_form_seed) {
        std::vector<double> closed_form_seed;
        if (two_view_closed_form_initial_state(beacon_count, estimator_path, measurements, closed_form_seed)) {
            seed = closed_form_seed;
        }
    }
    int warm_start_iterations = 0;
    if (robust) {
        // Warm-start pass: solve the un-reweighted (ordinary least squares)
        // problem first with a shorter iteration budget, so the subsequent
        // Huber-robust solve starts near a good basin rather than from the
        // raw seed.
        const auto warm_start = gauss_newton(
            seed,
            [&](const std::vector<double>& state) {
                return residuals_scenario1(
                    state, beacon_count, estimator_path, measurements, residual_noise, repeat_target_packets);
            },
            std::max(10, config.batch_solver_max_iterations / 2),
            config.batch_solver_initial_lambda,
            [&](const std::vector<double>& state) {
                return jacobian_scenario1(
                    state, beacon_count, estimator_path, measurements, residual_noise, repeat_target_packets);
            });
        seed = warm_start.x;
        warm_start_iterations = warm_start.iterations;
    }
    const auto result = gauss_newton(
        seed,
        [&](const std::vector<double>& state) {
            auto residuals = residuals_scenario1(
                state, beacon_count, estimator_path, measurements, residual_noise, repeat_target_packets);
            return robust ? huber_scaled_residuals(std::move(residuals), robust_delta) : residuals;
        },
        config.batch_solver_max_iterations,
        config.batch_solver_initial_lambda,
        robust ? JacobianFunction() :
            JacobianFunction([&](const std::vector<double>& state) {
                return jacobian_scenario1(
                    state, beacon_count, estimator_path, measurements, residual_noise, repeat_target_packets);
            }));
    const auto stop = std::chrono::steady_clock::now();
    const Vec2 estimate{result.x[0], result.x[1]};
    const auto beacon_estimates = beacon_estimates_from_scenario1_state(result.x, beacon_count);
    const double target_error = norm(estimate - world.target);
    const double beacon_position_error = beacon_position_rmse(world, beacon_estimates);
    const double beacon_yaw_error = beacon_yaw_rmse(world, beacon_estimates);
    return {1, beacon_count, trial, world.target, estimate, target_error,
            beacon_position_error, beacon_yaw_error,
            result.cost, result.iterations + warm_start_iterations, elapsed_ms(start, stop),
            result.converged, trial_accuracy_success(target_error, beacon_position_error, beacon_yaw_error)};
}

/**
 * @brief Convenience wrapper around run_local_batch_trial_with_measurements()
 * that first draws stressed measurements from `true_path` and builds the
 * generic circular initial seed from the given target/beacon-radius/yaw
 * seed values, then solves against `estimator_path` (which may be a noisy
 * or drifted version of `true_path` — see make_noisy_path/make_drifted_path).
 * Uses the closed-form seed override by default (inherited default
 * argument of the wrapped function).
 */
TrialResult run_local_batch_trial_with_seed(
    int trial,
    int beacon_count,
    const World& world,
    const std::vector<Vec2>& true_path,
    const std::vector<Vec2>& estimator_path,
    const SimulationConfig& config,
    const Noise& noise,
    const MeasurementStress& stress,
    const Vec2& target_seed,
    double beacon_seed_radius,
    double beacon_yaw_seed,
    bool robust,
    std::mt19937& rng) {
    const auto measurements = generate_stressed_local_measurements(world, true_path, noise, stress, rng);
    const auto initial_state =
        initial_state_scenario1(beacon_count, target_seed, beacon_seed_radius, beacon_yaw_seed);
    return run_local_batch_trial_with_measurements(
        trial,
        beacon_count,
        world,
        estimator_path,
        measurements,
        config,
        noise,
        initial_state,
        robust,
        config.robust_huber_delta);
}

/**
 * @brief Multistart batch trial (see glossary concept "Multistart"): draws
 * one set of stressed measurements, then runs
 * run_local_batch_trial_with_measurements() from `multistarts` different
 * perturbed initial seeds and keeps whichever converged result has the
 * lowest final cost. Mitigates sensitivity to a poor initialization / local
 * minima in the nonlinear least-squares problem, especially under weak
 * observability (see run_poor_initialization_sweep).
 *
 * @param multistarts Number of seeds to try; clamped to at least 1. The
 *        first start (`start_index == 0`) always uses the nominal
 *        target_seed/beacon_seed_radius/beacon_yaw_seed unperturbed; later
 *        starts perturb the target seed around a circle of radius
 *        `0.4 * beacon_seed_radius` and scale/rotate the beacon
 *        radius/yaw guess.
 * @return The lowest-cost TrialResult across all starts; `runtime_ms` is
 *         overwritten with the *total* wall-clock time across all starts
 *         (not just the winning one), since multistart's cost is the sum
 *         of every attempt.
 */
TrialResult run_multistart_local_batch_trial(
    int trial,
    int beacon_count,
    const World& world,
    const std::vector<Vec2>& true_path,
    const std::vector<Vec2>& estimator_path,
    const SimulationConfig& config,
    const Noise& noise,
    const MeasurementStress& stress,
    const Vec2& target_seed,
    double beacon_seed_radius,
    double beacon_yaw_seed,
    int multistarts,
    bool robust,
    std::mt19937& rng) {
    const auto measurements = generate_stressed_local_measurements(world, true_path, noise, stress, rng);
    TrialResult best;
    best.cost = std::numeric_limits<double>::infinity();
    const int starts = std::max(1, multistarts);
    const auto multistart_start = std::chrono::steady_clock::now();
    for (int start_index = 0; start_index < starts; ++start_index) {
        // Distribute additional starts evenly around a circle: perturb the
        // target seed tangentially and vary the beacon radius/yaw guess so
        // each start probes a genuinely different basin of attraction.
        const double angle = 2.0 * kPi * static_cast<double>(start_index) / static_cast<double>(starts);
        const Vec2 shifted_target{
            start_index == 0 ? target_seed.x : target_seed.x + 0.4 * beacon_seed_radius * std::cos(angle),
            start_index == 0 ? target_seed.y : target_seed.y + 0.4 * beacon_seed_radius * std::sin(angle),
        };
        const double radius_scale =
            start_index == 0 ? 1.0 : 0.7 + 0.6 * static_cast<double>((start_index % 3)) / 2.0;
        const auto initial_state = initial_state_scenario1(
            beacon_count,
            shifted_target,
            beacon_seed_radius * radius_scale,
            start_index == 0 ? beacon_yaw_seed : beacon_yaw_seed + angle);
        auto result = run_local_batch_trial_with_measurements(
            trial,
            beacon_count,
            world,
            estimator_path,
            measurements,
            config,
            noise,
            initial_state,
            robust,
            config.robust_huber_delta);
        // Keep the best (lowest-cost) result seen so far across all starts.
        if (result.cost < best.cost) {
            best = result;
        }
    }
    const auto multistart_stop = std::chrono::steady_clock::now();
    best.runtime_ms = elapsed_ms(multistart_start, multistart_stop);
    return best;
}

/**
 * @brief Restricts `measurements` to only those whose `time` index falls
 * within the most recent `window_size` time steps (i.e. a fixed-size
 * trailing sliding window over the measurement history), used by the
 * "sliding_window_gn" estimator variant in run_expanded_baseline_comparison
 * to approximate a bounded-memory/receding-horizon batch solve instead of
 * accumulating the full measurement history. Returns `measurements`
 * unchanged if empty or `window_size <= 0`.
 */
std::vector<LocalFrameMeasurement> recent_measurement_window(
    const std::vector<LocalFrameMeasurement>& measurements,
    int window_size) {
    if (measurements.empty() || window_size <= 0) {
        return measurements;
    }
    const std::size_t last_time = measurements.back().time;
    const std::size_t first_time =
        last_time > static_cast<std::size_t>(window_size) ?
            last_time - static_cast<std::size_t>(window_size) + 1U :
            0U;
    std::vector<LocalFrameMeasurement> window;
    for (const auto& measurement : measurements) {
        if (measurement.time >= first_time) {
            window.push_back(measurement);
        }
    }
    return window;
}

/**
 * @brief Dispatches one Monte Carlo trial for a given (world, path) pair
 * across all four scenario IDs: 3/4 delegate to the EKF
 * (run_ekf_local_frame_trial, generic vs. two-view seed); 1 draws
 * local-frame measurements, seeds from the two-view closed-form estimator
 * when it succeeds (falling back to the generic circular seed otherwise),
 * and solves with damped Gauss-Newton using the analytic Jacobian; 2 (the
 * default/else branch) draws global-frame bearing measurements and solves
 * only for the 2D target position, deriving beacon estimates afterward by
 * simple averaging (beacon_estimates_from_scenario2_measurements) rather
 * than jointly optimizing them.
 */
TrialResult run_trial_with_world_path(
    int scenario,
    int beacon_count,
    int trial,
    const World& world,
    const std::vector<Vec2>& path,
    const SimulationConfig& config,
    const Noise& noise,
    std::mt19937& rng) {
    if (scenario == 3) {
        return run_ekf_local_frame_trial(beacon_count, trial, world, path, config, noise, false, rng);
    }
    if (scenario == 4) {
        return run_ekf_local_frame_trial(beacon_count, trial, world, path, config, noise, true, rng);
    }

    if (scenario == 1) {
        const auto measurements = generate_local_frame_measurements(world, path, noise, rng);
        const auto start = std::chrono::steady_clock::now();
        auto initial_state = initial_state_scenario1(
            beacon_count,
            config.initial_target_estimate,
            config.initial_beacon_guess_radius,
            config.initial_beacon_guess_yaw);
        // Prefer the constructive two-view closed-form seed when it
        // succeeds (sufficiently excited view pair found for every
        // beacon); otherwise keep the generic circular seed above.
        std::vector<double> closed_form_seed;
        if (two_view_closed_form_initial_state(beacon_count, path, measurements, closed_form_seed)) {
            initial_state = closed_form_seed;
        }
        const auto result = gauss_newton(
            initial_state,
            [&](const std::vector<double>& state) {
                return residuals_scenario1(state, beacon_count, path, measurements, noise);
            },
            config.batch_solver_max_iterations,
            config.batch_solver_initial_lambda,
            [&](const std::vector<double>& state) {
                return jacobian_scenario1(state, beacon_count, path, measurements, noise);
            });
        const auto stop = std::chrono::steady_clock::now();
        const Vec2 estimate{result.x[0], result.x[1]};
        const auto beacon_estimates = beacon_estimates_from_scenario1_state(result.x, beacon_count);
        const double target_error = norm(estimate - world.target);
        const double beacon_position_error = beacon_position_rmse(world, beacon_estimates);
        const double beacon_yaw_error = beacon_yaw_rmse(world, beacon_estimates);
        return {scenario, beacon_count, trial, world.target, estimate, target_error,
                beacon_position_error, beacon_yaw_error,
                result.cost, result.iterations, elapsed_ms(start, stop),
                result.converged, trial_accuracy_success(target_error, beacon_position_error, beacon_yaw_error)};
    }

    // Scenario 2 (calibrated global-frame baseline): only the 2D target
    // position is jointly optimized; per-beacon estimates are derived
    // afterward by simple averaging, not solved for directly.
    const auto measurements = generate_global_bearing_measurements(world, path, noise, rng);
    const auto start = std::chrono::steady_clock::now();
    const auto result = gauss_newton(
        initial_state_scenario2(config.initial_target_estimate),
        [&](const std::vector<double>& state) {
            return residuals_scenario2(state, path, measurements);
        },
        config.batch_solver_max_iterations,
        config.batch_solver_initial_lambda);
    const auto stop = std::chrono::steady_clock::now();
    const Vec2 estimate{result.x[0], result.x[1]};
    const auto beacon_estimates =
        beacon_estimates_from_scenario2_measurements(estimate, beacon_count, measurements);
    const double target_error = norm(estimate - world.target);
    const double beacon_position_error = beacon_position_rmse(world, beacon_estimates);
    return {scenario, beacon_count, trial, world.target, estimate, target_error,
            beacon_position_error, -1.0,
            result.cost, result.iterations, elapsed_ms(start, stop),
            result.converged, trial_accuracy_success(target_error, beacon_position_error, -1.0)};
}

}  // namespace

// See Simulation.hpp for the full contract. Implementation: forward-Euler
// integration of the continuous adaptive law, one step per iteration.
AdaptiveLocalizationRun run_adaptive_localization(
    const SimulationConfig& config,
    std::mt19937& rng) {
    World world = make_world(config.adaptive_beacon_count);
    Vec2 robot = config.adaptive_initial_robot;
    Vec2 target_estimate = config.adaptive_initial_target_estimate;
    std::vector<AdaptiveLocalizationPoint> points;
    points.reserve(static_cast<std::size_t>(config.adaptive_steps));

    for (int step = 0; step < config.adaptive_steps; ++step) {
        // Draw this step's noisy range/bearing measurements, then apply the
        // continuous adaptive update law phat_dot = -2*Gamma*sum_i(epsilon_i
        // * ...), integrated forward with a fixed Euler step (adaptive_dt).
        const auto measurements = measure_core_model(world, robot, config.adaptive_noise, rng);
        const Vec2 target_dot = core_adaptive_update(
            robot, target_estimate, measurements, config.adaptive_gain);
        target_estimate = target_estimate + target_dot * config.adaptive_dt;

        // Record this step's estimates/errors before advancing the robot,
        // so the logged point reflects the estimate that produced the
        // control below.
        AdaptiveLocalizationPoint point;
        point.step = step;
        point.robot = robot;
        point.target_estimate = target_estimate;
        point.beacon_estimates = core_beacon_estimates(target_estimate, measurements);
        point.target_error = norm(target_estimate - world.target);
        point.goal_error = norm(robot - world.target);
        point.cost = core_cost(robot, target_estimate, measurements);
        points.push_back(point);

        // Target-seeking control law xi_dot = -k(xi - phat): drive the
        // robot toward the current target estimate (there is no separate
        // excitation term in this rigorous global-frame model, unlike the
        // closed-loop scenario-1/2 comparison below).
        const Vec2 control = (target_estimate - robot) * config.adaptive_target_seeking_gain;
        robot = robot + control * config.adaptive_dt;
    }

    return {world, points};
}

// See Simulation.hpp: delegates to the beacon-count-explicit overload using
// config.closed_loop_beacon_count.
ClosedLoopResult run_closed_loop_comparison(
    int scenario,
    const SimulationConfig& config,
    std::mt19937& rng) {
    return run_closed_loop_comparison(scenario, config.closed_loop_beacon_count, config, rng);
}

// See Simulation.hpp: delegates to the full overload with the default
// fixed decaying-circular excitation schedule.
ClosedLoopResult run_closed_loop_comparison(
    int scenario,
    int beacon_count,
    const SimulationConfig& config,
    std::mt19937& rng) {
    return run_closed_loop_comparison(
        scenario, beacon_count, config, rng, ClosedLoopExcitationMode::Circular);
}

// Full implementation; see Simulation.hpp for the contract. Each step:
// take a measurement, re-solve the batch estimate from scratch with the
// full history so far, evaluate the excitation policy, then advance the
// robot with target-seeking control plus the chosen excitation term.
ClosedLoopResult run_closed_loop_comparison(
    int scenario,
    int beacon_count,
    const SimulationConfig& config,
    std::mt19937& rng,
    ClosedLoopExcitationMode excitation_mode) {
    const World world = make_world(beacon_count);

    std::vector<Vec2> path;
    std::vector<LocalFrameMeasurement> local_measurements;
    std::vector<GlobalBearingMeasurement> global_measurements;
    std::vector<double> state1 = initial_state_scenario1(
        beacon_count,
        config.initial_target_estimate,
        config.initial_beacon_guess_radius,
        config.initial_beacon_guess_yaw);
    std::vector<double> state2 = initial_state_scenario2(config.initial_target_estimate);
    std::vector<BeaconEstimate> beacon_estimates =
        beacon_estimates_from_scenario1_state(state1, beacon_count);
    std::vector<ClosedLoopPoint> points;
    Vec2 robot = config.initial_robot;
    Vec2 target_estimate = config.initial_target_estimate;
    double current_cost = 0.0;
    // Step index at which the current excitation "epoch" began; only
    // meaningful/updated in Supervised mode, where retriggering resets it.
    int excitation_epoch = 1;
    // Two-view gate (Algorithm 1): the scenario-1 estimator retains its
    // prior until the stored window contains two distinct views and the
    // constructive seed exists, so no gauge-dependent solution ever feeds
    // the controller. Scenario 2 needs no gauge-removing seed.
    bool estimator_initialized = scenario != 1;

    ClosedLoopPoint initial_point;
    initial_point.step = 0;
    initial_point.robot = robot;
    initial_point.target_estimate = target_estimate;
    initial_point.beacon_estimates = beacon_estimates;
    initial_point.target_error = norm(target_estimate - world.target);
    initial_point.goal_error = norm(robot - world.target);
    initial_point.beacon_position_rmse = beacon_position_rmse(world, beacon_estimates);
    initial_point.beacon_yaw_rmse =
        scenario == 1 ? beacon_yaw_rmse(world, beacon_estimates) : -1.0;
    initial_point.cost = current_cost;
    initial_point.estimate_ready = estimator_initialized;
    points.push_back(initial_point);

    for (int step = 1; step <= config.closed_loop_steps; ++step) {
        // Record the current robot position as a new path pose and take one
        // measurement of every beacon from it.
        path.push_back(robot);
        for (std::size_t i = 0; i < world.beacons.size(); ++i) {
            if (scenario == 1) {
                local_measurements.push_back(make_local_frame_measurement(
                    world, robot, i, path.size() - 1, config.closed_loop_noise, rng));
            } else {
                global_measurements.push_back(make_global_bearing_measurement(
                    world, robot, i, path.size() - 1, config.closed_loop_noise, rng));
            }
        }

        // Refine the batch estimate over the *entire* measurement history
        // accumulated so far, for scenario 1 (joint target+beacon
        // self-calibration, gated on the two-view constructive seed) or
        // scenario 2 (target-only, beacons averaged afterward).
        if (scenario == 1) {
            if (!estimator_initialized) {
                // Try the constructive two-view seed once per step until the
                // window contains two distinct views; until then the prior
                // estimate is retained untouched.
                std::vector<double> closed_form_seed;
                estimator_initialized = two_view_closed_form_initial_state(
                    beacon_count, path, local_measurements, closed_form_seed);
                if (estimator_initialized) {
                    state1 = std::move(closed_form_seed);
                }
            }
            if (estimator_initialized) {
                // Warm-started damped Gauss-Newton on residuals whitened with
                // the actual closed-loop noise sigmas, with the analytic
                // Jacobian (no finite-difference fallback).
                const auto result = gauss_newton(
                    state1,
                    [&](const std::vector<double>& state) {
                        return residuals_scenario1(
                            state, beacon_count, path, local_measurements,
                            config.closed_loop_noise);
                    },
                    config.closed_loop_solver_max_iterations,
                    config.closed_loop_solver_initial_lambda,
                    [&](const std::vector<double>& state) {
                        return jacobian_scenario1(
                            state, beacon_count, path, local_measurements,
                            config.closed_loop_noise);
                    });
                state1 = result.x;
                target_estimate = {state1[0], state1[1]};
                current_cost = result.cost;
                beacon_estimates = beacon_estimates_from_scenario1_state(state1, beacon_count);
            }
        } else {
            const auto result = gauss_newton(
                state2,
                [&](const std::vector<double>& state) {
                    return residuals_scenario2(state, path, global_measurements);
                },
                config.closed_loop_solver_max_iterations,
                config.closed_loop_solver_initial_lambda);
            state2 = result.x;
            target_estimate = {state2[0], state2[1]};
            current_cost = result.cost;
            beacon_estimates = beacon_estimates_from_scenario2_measurements(
                target_estimate, beacon_count, global_measurements);
        }

        // Accumulated spread of the stored window, computed from the known
        // measurement poses -- exactly the noiseless certificate S_v the
        // theory supervises on (see Math.hpp's path_spread). Logged every
        // step, for every excitation mode, so the paper's S_v traces plot
        // the same quantity the supervisor acts on.
        const double window_spread = path_spread(path);
        // Conditioning diagnostic (never a control trigger): sigma_min of the
        // whitened stacked Jacobian at the current estimate, whitened with
        // the actual closed-loop noise sigmas. Only defined for the 1-beacon
        // scenario-1 model this diagnostic is scoped to.
        double sigma_min_diagnostic = -1.0;
        if (scenario == 1 && beacon_count == 1) {
            sigma_min_diagnostic = local_observability_rank_and_sigma_min(
                state1, path, local_measurements, config.closed_loop_noise).second;
        }

        bool retriggered = false;
        if (excitation_mode == ClosedLoopExcitationMode::Supervised) {
            // Spread-only supervision (Algorithm 1): retrigger the excitation
            // epoch while the stored window's spread is below the accuracy-
            // driven threshold S_bar. S_v is computed from the known poses,
            // so this trigger is exactly the certificate covered by the
            // finite-acquisition proposition; conditioning (sigma_min) is
            // reported as a diagnostic but deliberately not a trigger, since
            // the acquisition guarantee does not cover a second threshold.
            if (window_spread < config.supervised_spread_threshold) {
                excitation_epoch = step;
                retriggered = true;
            }
        }

        // Decaying circular "swirl" excitation term, evaluated at the
        // physical packet time t_k = (k-1)*dt so lambda is in s^-1 and omega
        // in rad/s (matching the paper and the Gazebo node). The amplitude
        // decays exponentially since the start of the current epoch
        // (Supervised mode measures decay relative to the last retrigger;
        // Circular/Information modes measure relative to the loop start);
        // the swirl phase always advances with absolute time.
        const double current_time = static_cast<double>(step - 1) * config.closed_loop_dt;
        const double epoch_time = static_cast<double>(excitation_epoch - 1) * config.closed_loop_dt;
        const double decay_reference = excitation_mode == ClosedLoopExcitationMode::Supervised
            ? current_time - epoch_time
            : current_time;
        const double phase_reference = current_time;
        const double exploration =
            config.exploration_amplitude * std::exp(-config.exploration_decay * decay_reference);
        const Vec2 swirl{
            exploration * std::cos(config.exploration_frequency * phase_reference),
            exploration * std::sin(config.exploration_frequency * phase_reference),
        };
        // In Information mode, replace the fixed swirl with the greedy
        // logdet-gradient excitation (information_driven_excitation);
        // Circular and Supervised modes both use the swirl (Supervised only
        // differs in when the swirl's clock resets).
        Vec2 excitation = swirl;
        if (excitation_mode == ClosedLoopExcitationMode::Information) {
            excitation = information_driven_excitation(
                robot,
                target_estimate,
                state1,
                beacon_estimates,
                path,
                local_measurements,
                config,
                current_time);
        }
        // Target-seeking control plus excitation, integrated with a fixed
        // Euler step (closed_loop_dt).
        robot = robot + ((target_estimate - robot) * config.closed_loop_control_gain + excitation) *
            config.closed_loop_dt;

        ClosedLoopPoint point;
        point.step = step;
        point.robot = robot;
        point.target_estimate = target_estimate;
        point.beacon_estimates = beacon_estimates;
        point.target_error = norm(target_estimate - world.target);
        point.goal_error = norm(robot - world.target);
        point.beacon_position_rmse = beacon_position_rmse(world, beacon_estimates);
        point.beacon_yaw_rmse = scenario == 1 ? beacon_yaw_rmse(world, beacon_estimates) : -1.0;
        point.cost = current_cost;
        point.spread = window_spread;
        point.sigma_min = sigma_min_diagnostic;
        point.excitation_norm2 = dot(excitation, excitation);
        point.retriggered = retriggered;
        point.estimate_ready = estimator_initialized;
        points.push_back(point);
    }

    return {scenario, world, points, beacon_estimates, target_estimate};
}

// See Simulation.hpp. Runs the same single-beacon closed loop twice from
// paired RNG seeds, once per excitation policy, and reports final accuracy.
std::vector<ActiveExcitationComparisonRow> run_active_excitation_comparison(
    const SimulationConfig& config) {
    const auto make_row = [](const std::string& name, const ClosedLoopResult& result) {
        ActiveExcitationComparisonRow row;
        row.excitation = name;
        row.beacons = static_cast<int>(result.world.beacons.size());
        if (!result.points.empty()) {
            const auto& final_point = result.points.back();
            row.final_goal_error = final_point.goal_error;
            row.final_target_error = final_point.target_error;
            row.final_beacon_position_rmse = final_point.beacon_position_rmse;
            row.final_beacon_yaw_rmse = final_point.beacon_yaw_rmse;
            row.final_cost = final_point.cost;
        }
        return row;
    };

    // Seed base shared by both policies below so the underlying noise
    // draws are directly comparable; other sweeps use their own bases
    // purely to keep unrelated RNG streams decorrelated from each other.
    constexpr unsigned int kExcitationPolicyComparisonSeedBase = 200U;
    std::mt19937 circular_rng(config.closed_loop_seed + kExcitationPolicyComparisonSeedBase);
    const auto circular = run_closed_loop_comparison(
        1, 1, config, circular_rng, ClosedLoopExcitationMode::Circular);

    std::mt19937 information_rng(config.closed_loop_seed + kExcitationPolicyComparisonSeedBase);
    const auto information = run_closed_loop_comparison(
        1, 1, config, information_rng, ClosedLoopExcitationMode::Information);

    return {
        make_row("decaying_circular", circular),
        make_row("information_logdet", information),
    };
}

// Shared RNG seed base for every fixed-vs-supervised excitation comparison
// below (run_supervised_excitation_comparison's run_pair lambda and
// run_supervised_lambda_sweep): both policies in a pair start from the same
// seed so their noise draws are directly comparable; the value itself only
// needs to differ from other sweeps' bases to keep unrelated RNG streams
// decorrelated.
constexpr unsigned int kFixedVsSupervisedSeedBase = 300U;

// See Simulation.hpp for the full contract.
std::vector<SupervisedExcitationComparisonRow> run_supervised_excitation_comparison(
    const SimulationConfig& config) {
    const auto steps_to_threshold = [](const std::vector<ClosedLoopPoint>& points, double threshold,
                                        double (*metric)(const ClosedLoopPoint&)) {
        for (const auto& point : points) {
            if (metric(point) < threshold) {
                return point.step;
            }
        }
        return -1;
    };

    const auto make_row = [&](const std::string& name, const ClosedLoopResult& result) {
        SupervisedExcitationComparisonRow row;
        row.excitation = name;
        for (const auto& point : result.points) {
            if (point.retriggered) {
                ++row.retrigger_count;
            }
        }
        row.steps_to_goal_threshold = steps_to_threshold(
            result.points, config.supervised_goal_error_threshold,
            [](const ClosedLoopPoint& point) { return point.goal_error; });
        row.steps_to_target_threshold = steps_to_threshold(
            result.points, config.supervised_target_error_threshold,
            [](const ClosedLoopPoint& point) { return point.target_error; });
        if (!result.points.empty()) {
            const auto& final_point = result.points.back();
            row.final_goal_error = final_point.goal_error;
            row.final_target_error = final_point.target_error;
            row.final_beacon_position_rmse = final_point.beacon_position_rmse;
            row.final_beacon_yaw_rmse = final_point.beacon_yaw_rmse;
            row.final_cost = final_point.cost;
        }
        return row;
    };

    const auto run_pair = [&](const std::string& suffix, const SimulationConfig& scenario_config) {
        std::mt19937 circular_rng(scenario_config.closed_loop_seed + kFixedVsSupervisedSeedBase);
        const auto circular = run_closed_loop_comparison(
            1, 1, scenario_config, circular_rng, ClosedLoopExcitationMode::Circular);

        std::mt19937 supervised_rng(scenario_config.closed_loop_seed + kFixedVsSupervisedSeedBase);
        const auto supervised = run_closed_loop_comparison(
            1, 1, scenario_config, supervised_rng, ClosedLoopExcitationMode::Supervised);

        return std::vector<SupervisedExcitationComparisonRow>{
            make_row("decaying_circular_" + suffix, circular),
            make_row("excitation_supervised_" + suffix, supervised),
        };
    };

    // Nominal scenario: same closed-loop parameters used throughout the rest
    // of the paper (far initial pose, moderate decay). The target-seeking
    // transient alone already supplies ample trajectory diversity here.
    const auto nominal = run_pair("nominal", config);

    // Understimulated scenario: the excitation schedule decays far faster
    // than the paper's default, and the robot starts exactly at the true
    // target (matching World::make_world's fixed target), removing the
    // convergence transient that would otherwise supply diversity on its
    // own. Only the excitation schedule itself can excite the geometry.
    SimulationConfig stress_config = config;
    stress_config.exploration_decay = 2.0;
    stress_config.initial_target_estimate = {1.2, -0.75};
    stress_config.initial_robot = stress_config.initial_target_estimate;
    const auto understimulated = run_pair("understimulated", stress_config);

    std::vector<SupervisedExcitationComparisonRow> rows;
    rows.insert(rows.end(), nominal.begin(), nominal.end());
    rows.insert(rows.end(), understimulated.begin(), understimulated.end());
    return rows;
}

// See Simulation.hpp. Every run is seeded independently, so adding or
// reordering showcase runs cannot perturb any other experiment's RNG stream.
std::vector<ClosedLoopShowcaseRun> run_closed_loop_showcase(
    const SimulationConfig& config) {
    std::vector<ClosedLoopShowcaseRun> runs;
    const auto run_mode = [&runs](const std::string& name,
                                  const SimulationConfig& scenario_config,
                                  unsigned int seed,
                                  ClosedLoopExcitationMode mode) {
        std::mt19937 rng(seed);
        runs.push_back({name, run_closed_loop_comparison(1, 1, scenario_config, rng, mode)});
    };

    // Understimulated (no-transient) trio: the exact scenario and seed of
    // run_supervised_excitation_comparison's understimulated pair, plus the
    // information-gradient schedule. Both unsupervised schedules share the
    // same decaying envelope, so the trio isolates the reset rule (not the
    // excitation shape) as the difference.
    SimulationConfig stress = config;
    stress.exploration_decay = 2.0;
    stress.initial_target_estimate = {1.2, -0.75};
    stress.initial_robot = stress.initial_target_estimate;
    const unsigned int stress_seed = stress.closed_loop_seed + 300U;
    run_mode("understimulated_fixed", stress, stress_seed,
             ClosedLoopExcitationMode::Circular);
    run_mode("understimulated_information", stress, stress_seed,
             ClosedLoopExcitationMode::Information);
    run_mode("understimulated_supervised", stress, stress_seed,
             ClosedLoopExcitationMode::Supervised);

    // Nominal scenario with the excitation term removed entirely: the
    // convergence transit alone accumulates the certification spread, which
    // is why the nominal trajectory figures look nearly straight.
    SimulationConfig no_excitation = config;
    no_excitation.exploration_amplitude = 0.0;
    run_mode("nominal_no_excitation", no_excitation,
             no_excitation.closed_loop_seed + 300U,
             ClosedLoopExcitationMode::Supervised);

    // Supervised target seeking from a ring of six start positions around
    // the true target (fast decay, common wrong target prior, one distinct
    // noise seed per start): the variety counterpart of the single seeking
    // scenario, showing certification is not tied to one chosen start.
    constexpr double kRingRadius = 1.8;
    const Vec2 ring_center{1.2, -0.75};
    for (int i = 0; i < 6; ++i) {
        const double angle = static_cast<double>(i) * kPi / 3.0;
        SimulationConfig seek = config;
        seek.exploration_decay = 2.0;
        seek.initial_robot = {ring_center.x + kRingRadius * std::cos(angle),
                              ring_center.y + kRingRadius * std::sin(angle)};
        run_mode("seeking_ring_" + std::to_string(i), seek,
                 seek.closed_loop_seed + 400U + static_cast<unsigned int>(i),
                 ClosedLoopExcitationMode::Supervised);
    }
    return runs;
}

namespace {

/**
 * @brief Across-trial RMSE of a batch of scalar errors:
 * sqrt((1/M) sum e_j^2). This is the statistic the design rule
 * var(psi_hat) ~ sigma^2 / S_v predicts (a population RMS level, not a
 * per-trial bound and not a mean absolute error), so the Monte Carlo tables
 * report it directly.
 */
double batch_rmse(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    double sum2 = 0.0;
    for (double value : values) {
        sum2 += value * value;
    }
    return std::sqrt(sum2 / static_cast<double>(values.size()));
}

/**
 * @brief Percentile-bootstrap 95% confidence interval for batch_rmse():
 * resamples the trial errors with replacement B times and takes the
 * 2.5th/97.5th percentiles of the resampled RMSEs. The bootstrap RNG is
 * seeded from a fixed constant (independent of the simulation seeds), so
 * the CI is deterministic; index selection uses rng() % n, whose modulo
 * bias is negligible for n on the order of 10^2 against a 32-bit engine.
 */
std::pair<double, double> bootstrap_rmse_ci95(const std::vector<double>& values) {
    if (values.size() < 2) {
        const double point = batch_rmse(values);
        return {point, point};
    }
    constexpr int kResamples = 2000;
    constexpr unsigned int kBootstrapSeed = 987654321U;
    std::mt19937 rng(kBootstrapSeed);
    const std::size_t n = values.size();
    std::vector<double> resampled_rmse;
    resampled_rmse.reserve(kResamples);
    for (int b = 0; b < kResamples; ++b) {
        double sum2 = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            const double value = values[rng() % n];
            sum2 += value * value;
        }
        resampled_rmse.push_back(std::sqrt(sum2 / static_cast<double>(n)));
    }
    std::sort(resampled_rmse.begin(), resampled_rmse.end());
    const std::size_t lo_index = static_cast<std::size_t>(0.025 * (kResamples - 1));
    const std::size_t hi_index = static_cast<std::size_t>(0.975 * (kResamples - 1));
    return {resampled_rmse[lo_index], resampled_rmse[hi_index]};
}

/** @brief Arithmetic mean of `values` (0.0 for an empty batch). */
double batch_mean(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (double value : values) {
        sum += value;
    }
    return sum / static_cast<double>(values.size());
}

/** @brief Normal-approximation 95% CI half-width for the mean of `values`. */
double batch_mean_ci95(const std::vector<double>& values) {
    if (values.size() < 2) {
        return 0.0;
    }
    const double mean = batch_mean(values);
    double sum2 = 0.0;
    for (double value : values) {
        sum2 += (value - mean) * (value - mean);
    }
    const double variance = sum2 / static_cast<double>(values.size() - 1);
    return 1.96 * std::sqrt(variance / static_cast<double>(values.size()));
}

/**
 * @brief Collects per-trial outcomes for one excitation policy across a
 * Monte Carlo batch of closed-loop runs; the run_supervised_* aggregators
 * below turn these samples into RMSEs with bootstrap CIs and success rates.
 */
struct ClosedLoopBatchSamples {
    std::vector<double> final_goal_errors;
    std::vector<double> final_target_errors;
    std::vector<double> final_beacon_position_rmses;
    std::vector<double> final_beacon_yaw_rmses;
    // Packets needed to first cross the goal-error threshold, over the
    // trials that reached it at all: the time-to-accuracy statistic that
    // separates guaranteed excitation acquisition (supervised) from relying
    // on estimate-jitter-driven wander to self-excite (fixed schedule).
    std::vector<double> steps_to_goal;
    // Packets needed for the stored window's spread to first reach the
    // supervisor's threshold, over the trials that reached it.
    std::vector<double> packets_to_threshold;
    // Per-trial count of underexcited (retriggered) packets, and of
    // excitation episodes (maximal runs of consecutive retriggered packets).
    std::vector<double> retrigger_counts;
    std::vector<double> episode_counts;
    // Per-trial traveled path length and integrated excitation effort
    // sum_k ||u_k^exp||^2 * dt.
    std::vector<double> path_lengths;
    std::vector<double> excitation_efforts;
    int count = 0;

    void add(const ClosedLoopResult& result, double goal_threshold, double spread_threshold,
             double dt) {
        if (result.points.empty()) {
            return;
        }
        const auto& final_point = result.points.back();
        final_goal_errors.push_back(final_point.goal_error);
        final_target_errors.push_back(final_point.target_error);
        final_beacon_position_rmses.push_back(final_point.beacon_position_rmse);
        final_beacon_yaw_rmses.push_back(final_point.beacon_yaw_rmse);

        int retriggers = 0;
        int episodes = 0;
        bool previous_retriggered = false;
        double path_length = 0.0;
        double effort = 0.0;
        bool goal_recorded = false;
        bool threshold_recorded = false;
        for (std::size_t i = 0; i < result.points.size(); ++i) {
            const auto& point = result.points[i];
            if (point.retriggered) {
                ++retriggers;
                if (!previous_retriggered) {
                    ++episodes;
                }
            }
            previous_retriggered = point.retriggered;
            if (i > 0) {
                path_length += norm(point.robot - result.points[i - 1].robot);
            }
            effort += point.excitation_norm2 * dt;
            if (!goal_recorded && point.goal_error < goal_threshold) {
                steps_to_goal.push_back(static_cast<double>(point.step));
                goal_recorded = true;
            }
            if (!threshold_recorded && point.spread >= spread_threshold) {
                packets_to_threshold.push_back(static_cast<double>(point.step));
                threshold_recorded = true;
            }
        }
        retrigger_counts.push_back(static_cast<double>(retriggers));
        episode_counts.push_back(static_cast<double>(episodes));
        path_lengths.push_back(path_length);
        excitation_efforts.push_back(effort);
        ++count;
    }

    /// Fraction of trials whose final value of `values` is strictly below
    /// (or, for `or_equal`, at or below) `threshold`.
    static double success_rate(const std::vector<double>& values, double threshold,
                               bool or_equal = false) {
        if (values.empty()) {
            return 0.0;
        }
        int successes = 0;
        for (double value : values) {
            if (or_equal ? value <= threshold : value < threshold) {
                ++successes;
            }
        }
        return static_cast<double>(successes) / static_cast<double>(values.size());
    }

    double reached_rate(const std::vector<double>& reached) const {
        return count > 0 ? static_cast<double>(reached.size()) / static_cast<double>(count) : 0.0;
    }
};

// The paper's declared yaw-accuracy success criterion. Success rates count
// trials whose final beacon-yaw error is at or below this level; it is held
// fixed across the threshold ablation so success measures the same accuracy
// at every S_bar (the design rule then predicts which thresholds meet it).
constexpr double kYawSuccessThresholdRad = 0.05;

/**
 * @brief The design rule's population yaw-RMSE prediction eps_psi = sigma /
 * sqrt(S_bar) (Remark on accuracy-driven threshold selection): choosing
 * S_bar = sigma^2 / eps_psi^2 targets an across-trial yaw RMSE of eps_psi.
 * At the default S_bar = 0.16 and closed-loop range sigma = 0.02 m this
 * equals the 0.05-rad success criterion above, so the primary experiment's
 * threshold is selected to deliver exactly the accuracy the experiments
 * declare as success.
 */
double predicted_yaw_rmse(double range_sigma, double spread_threshold) {
    return range_sigma / std::sqrt(std::max(spread_threshold, 1e-12));
}

}  // namespace

// See Simulation.hpp for the full contract.
std::vector<SupervisedLambdaSweepRow> run_supervised_lambda_sweep(
    const SimulationConfig& config) {
    const std::vector<double> lambdas{0.02, 0.05, 0.10, 0.25, 0.50, 1.0, 2.0};
    const int trials = std::max(1, config.supervised_monte_carlo_trials);

    std::vector<SupervisedLambdaSweepRow> rows;
    rows.reserve(lambdas.size());
    for (double lambda : lambdas) {
        // Same no-transient scenario as the underexcited comparison: the
        // robot starts at the true target, so only the excitation schedule
        // can excite the geometry, and lambda controls how quickly the
        // fixed schedule gives up. Paired Monte Carlo: per trial, the fixed
        // and supervised policies consume the same seed, so per-trial
        // differences are attributable to the policy rather than the noise
        // realization.
        SimulationConfig sweep_config = config;
        sweep_config.exploration_decay = lambda;
        sweep_config.initial_target_estimate = {1.2, -0.75};
        sweep_config.initial_robot = sweep_config.initial_target_estimate;

        ClosedLoopBatchSamples fixed_samples;
        ClosedLoopBatchSamples supervised_samples;
        for (int trial = 0; trial < trials; ++trial) {
            const unsigned int seed = sweep_config.closed_loop_seed +
                kFixedVsSupervisedSeedBase + static_cast<unsigned int>(trial);

            std::mt19937 fixed_rng(seed);
            const auto fixed = run_closed_loop_comparison(
                1, 1, sweep_config, fixed_rng, ClosedLoopExcitationMode::Circular);
            fixed_samples.add(fixed, config.supervised_goal_error_threshold,
                              config.supervised_spread_threshold, config.closed_loop_dt);

            std::mt19937 supervised_rng(seed);
            const auto supervised = run_closed_loop_comparison(
                1, 1, sweep_config, supervised_rng, ClosedLoopExcitationMode::Supervised);
            supervised_samples.add(supervised, config.supervised_goal_error_threshold,
                                   config.supervised_spread_threshold, config.closed_loop_dt);
        }

        SupervisedLambdaSweepRow row;
        row.lambda = lambda;
        row.trials = trials;
        row.supervised_mean_retrigger_count = batch_mean(supervised_samples.retrigger_counts);

        row.fixed_target_rmse = batch_rmse(fixed_samples.final_target_errors);
        std::tie(row.fixed_target_rmse_ci_lo, row.fixed_target_rmse_ci_hi) =
            bootstrap_rmse_ci95(fixed_samples.final_target_errors);
        row.fixed_beacon_position_rmse = batch_rmse(fixed_samples.final_beacon_position_rmses);
        std::tie(row.fixed_beacon_position_rmse_ci_lo, row.fixed_beacon_position_rmse_ci_hi) =
            bootstrap_rmse_ci95(fixed_samples.final_beacon_position_rmses);
        row.fixed_beacon_yaw_rmse = batch_rmse(fixed_samples.final_beacon_yaw_rmses);
        std::tie(row.fixed_beacon_yaw_rmse_ci_lo, row.fixed_beacon_yaw_rmse_ci_hi) =
            bootstrap_rmse_ci95(fixed_samples.final_beacon_yaw_rmses);
        row.fixed_yaw_success_rate = ClosedLoopBatchSamples::success_rate(
            fixed_samples.final_beacon_yaw_rmses, kYawSuccessThresholdRad, true);

        row.supervised_target_rmse = batch_rmse(supervised_samples.final_target_errors);
        std::tie(row.supervised_target_rmse_ci_lo, row.supervised_target_rmse_ci_hi) =
            bootstrap_rmse_ci95(supervised_samples.final_target_errors);
        row.supervised_beacon_position_rmse =
            batch_rmse(supervised_samples.final_beacon_position_rmses);
        std::tie(row.supervised_beacon_position_rmse_ci_lo,
                 row.supervised_beacon_position_rmse_ci_hi) =
            bootstrap_rmse_ci95(supervised_samples.final_beacon_position_rmses);
        row.supervised_beacon_yaw_rmse = batch_rmse(supervised_samples.final_beacon_yaw_rmses);
        std::tie(row.supervised_beacon_yaw_rmse_ci_lo, row.supervised_beacon_yaw_rmse_ci_hi) =
            bootstrap_rmse_ci95(supervised_samples.final_beacon_yaw_rmses);
        row.supervised_yaw_success_rate = ClosedLoopBatchSamples::success_rate(
            supervised_samples.final_beacon_yaw_rmses, kYawSuccessThresholdRad, true);
        rows.push_back(row);
    }
    return rows;
}

// Seed base for the target-seeking comparison's Monte Carlo batch; distinct
// from the other sweeps' bases purely to keep unrelated RNG streams
// decorrelated. Both policies in a pair share the same per-trial seed.
constexpr unsigned int kSeekingComparisonSeedBase = 400U;

// See Simulation.hpp for the full contract.
std::vector<SupervisedSeekingComparisonRow> run_supervised_seeking_comparison(
    const SimulationConfig& config) {
    const int trials = std::max(1, config.supervised_monte_carlo_trials);

    // Nontrivial target-seeking scenario: the vehicle starts AT the wrong
    // initial target estimate (both at the origin, ~1.4 m from the true
    // target), so the seeking term -k(q - p_hat) is initially quiescent and
    // produces no exploratory transient of its own. Only the excitation
    // policy can generate the trajectory spread needed to calibrate the
    // relay, and without calibration the target estimate -- and hence the
    // vehicle -- never moves to the true target. Target-seeking success is
    // therefore genuinely contingent on the excitation policy here, unlike
    // the decay sweep's calibration-isolation scenario where the vehicle
    // already sits at the target. The fast decay makes the fixed schedule's
    // excitation budget inadequate unless it happens to suffice by luck.
    SimulationConfig seeking_config = config;
    seeking_config.exploration_decay = 2.0;
    seeking_config.initial_target_estimate = {0.0, 0.0};
    seeking_config.initial_robot = seeking_config.initial_target_estimate;

    ClosedLoopBatchSamples fixed_samples;
    ClosedLoopBatchSamples supervised_samples;
    for (int trial = 0; trial < trials; ++trial) {
        const unsigned int seed = seeking_config.closed_loop_seed +
            kSeekingComparisonSeedBase + static_cast<unsigned int>(trial);

        std::mt19937 fixed_rng(seed);
        const auto fixed = run_closed_loop_comparison(
            1, 1, seeking_config, fixed_rng, ClosedLoopExcitationMode::Circular);
        fixed_samples.add(fixed, config.supervised_goal_error_threshold,
                          config.supervised_spread_threshold, config.closed_loop_dt);

        std::mt19937 supervised_rng(seed);
        const auto supervised = run_closed_loop_comparison(
            1, 1, seeking_config, supervised_rng, ClosedLoopExcitationMode::Supervised);
        supervised_samples.add(supervised, config.supervised_goal_error_threshold,
                               config.supervised_spread_threshold, config.closed_loop_dt);
    }

    const auto make_row = [&](const std::string& name, const ClosedLoopBatchSamples& samples) {
        SupervisedSeekingComparisonRow row;
        row.excitation = name;
        row.trials = samples.count;
        row.mean_retrigger_count = batch_mean(samples.retrigger_counts);
        row.goal_success_rate = ClosedLoopBatchSamples::success_rate(
            samples.final_goal_errors, config.supervised_goal_error_threshold);
        row.target_success_rate = ClosedLoopBatchSamples::success_rate(
            samples.final_target_errors, config.supervised_target_error_threshold);
        row.yaw_success_rate = ClosedLoopBatchSamples::success_rate(
            samples.final_beacon_yaw_rmses, kYawSuccessThresholdRad, true);
        row.goal_reached_rate = samples.reached_rate(samples.steps_to_goal);
        row.steps_to_goal_mean =
            samples.steps_to_goal.empty() ? -1.0 : batch_mean(samples.steps_to_goal);
        row.steps_to_goal_ci95 = batch_mean_ci95(samples.steps_to_goal);
        row.final_goal_rmse = batch_rmse(samples.final_goal_errors);
        std::tie(row.final_goal_rmse_ci_lo, row.final_goal_rmse_ci_hi) =
            bootstrap_rmse_ci95(samples.final_goal_errors);
        row.final_target_rmse = batch_rmse(samples.final_target_errors);
        std::tie(row.final_target_rmse_ci_lo, row.final_target_rmse_ci_hi) =
            bootstrap_rmse_ci95(samples.final_target_errors);
        row.final_beacon_position_rmse = batch_rmse(samples.final_beacon_position_rmses);
        std::tie(row.final_beacon_position_rmse_ci_lo, row.final_beacon_position_rmse_ci_hi) =
            bootstrap_rmse_ci95(samples.final_beacon_position_rmses);
        row.final_beacon_yaw_rmse = batch_rmse(samples.final_beacon_yaw_rmses);
        std::tie(row.final_beacon_yaw_rmse_ci_lo, row.final_beacon_yaw_rmse_ci_hi) =
            bootstrap_rmse_ci95(samples.final_beacon_yaw_rmses);
        return row;
    };

    return {
        make_row("decaying_circular", fixed_samples),
        make_row("excitation_supervised", supervised_samples),
    };
}

// Seed base for the spread-threshold ablation's Monte Carlo batch; distinct
// from the other supervised sweeps' bases purely to keep unrelated RNG
// streams decorrelated.
constexpr unsigned int kThresholdAblationSeedBase = 500U;

// See Simulation.hpp for the full contract.
std::vector<SupervisedThresholdAblationRow> run_supervised_threshold_ablation(
    const SimulationConfig& config) {
    // Candidate thresholds bracketing the primary S_bar = 0.16 setting: the
    // loose 0.05 (design-rule accuracy 0.089 rad), the accuracy-matched
    // 0.16 (0.05 rad, the declared success criterion), an intermediate 1.0
    // (0.02 rad), and the paper's high-precision budget example 9.04
    // (0.0067 rad). The trial success criterion stays fixed at 0.05 rad
    // throughout, so the sweep shows what each threshold buys -- and costs
    // -- at the same declared accuracy.
    const std::vector<double> thresholds{0.05, 0.16, 1.0, 9.04};
    const int trials = std::max(1, config.supervised_monte_carlo_trials);

    std::vector<SupervisedThresholdAblationRow> rows;
    rows.reserve(thresholds.size());
    for (double threshold : thresholds) {
        // Understimulated scenario with the fast decay: the robot starts at
        // the true target with a correct initial estimate, so essentially
        // all excitation is supervisor-commanded and the per-threshold cost
        // columns (path length, effort, underexcited packets) measure the
        // supervisor's own demand rather than seeking transients.
        SimulationConfig ablation_config = config;
        ablation_config.exploration_decay = 2.0;
        ablation_config.initial_target_estimate = {1.2, -0.75};
        ablation_config.initial_robot = ablation_config.initial_target_estimate;
        ablation_config.supervised_spread_threshold = threshold;

        ClosedLoopBatchSamples samples;
        for (int trial = 0; trial < trials; ++trial) {
            const unsigned int seed = ablation_config.closed_loop_seed +
                kThresholdAblationSeedBase + static_cast<unsigned int>(trial);
            std::mt19937 rng(seed);
            const auto result = run_closed_loop_comparison(
                1, 1, ablation_config, rng, ClosedLoopExcitationMode::Supervised);
            samples.add(result, config.supervised_goal_error_threshold, threshold,
                        config.closed_loop_dt);
        }

        SupervisedThresholdAblationRow row;
        row.spread_threshold = threshold;
        row.trials = trials;
        row.predicted_yaw_rmse =
            predicted_yaw_rmse(config.closed_loop_noise.range_sigma, threshold);
        row.yaw_rmse = batch_rmse(samples.final_beacon_yaw_rmses);
        std::tie(row.yaw_rmse_ci_lo, row.yaw_rmse_ci_hi) =
            bootstrap_rmse_ci95(samples.final_beacon_yaw_rmses);
        row.yaw_success_rate = ClosedLoopBatchSamples::success_rate(
            samples.final_beacon_yaw_rmses, kYawSuccessThresholdRad, true);
        row.threshold_reached_rate = samples.reached_rate(samples.packets_to_threshold);
        row.packets_to_threshold_mean =
            samples.packets_to_threshold.empty() ? -1.0 : batch_mean(samples.packets_to_threshold);
        row.packets_to_threshold_ci95 = batch_mean_ci95(samples.packets_to_threshold);
        row.mean_retrigger_count = batch_mean(samples.retrigger_counts);
        row.mean_episode_count = batch_mean(samples.episode_counts);
        row.mean_path_length = batch_mean(samples.path_lengths);
        row.mean_excitation_effort = batch_mean(samples.excitation_efforts);
        row.target_rmse = batch_rmse(samples.final_target_errors);
        std::tie(row.target_rmse_ci_lo, row.target_rmse_ci_hi) =
            bootstrap_rmse_ci95(samples.final_target_errors);
        row.beacon_position_rmse = batch_rmse(samples.final_beacon_position_rmses);
        std::tie(row.beacon_position_rmse_ci_lo, row.beacon_position_rmse_ci_hi) =
            bootstrap_rmse_ci95(samples.final_beacon_position_rmses);
        rows.push_back(row);
    }
    return rows;
}

// See Simulation.hpp for the full contract.
TrialResult run_trial(
    int scenario,
    int beacon_count,
    int trial,
    const SimulationConfig& config,
    std::mt19937& rng) {
    const World world = make_world(beacon_count);
    const std::vector<Vec2> path = make_vehicle_path(config.monte_carlo_path_steps);
    return run_trial_with_world_path(
        scenario, beacon_count, trial, world, path, config, config.monte_carlo_noise, rng);
}

// See Simulation.hpp for the full contract: one shared RNG stream is
// advanced across every (scenario, beacon_count, trial) combination in
// nested-loop order, so results are reproducible for a fixed seed but
// individual trials are not independently re-seeded.
std::vector<TrialResult> run_monte_carlo(const SimulationConfig& config) {
    std::mt19937 rng(config.monte_carlo_seed);
    std::vector<TrialResult> trials;
    trials.reserve(
        static_cast<std::size_t>(config.monte_carlo_trials_per_case) *
        config.monte_carlo_scenarios.size() *
        config.monte_carlo_beacon_counts.size());

    for (int scenario : config.monte_carlo_scenarios) {
        for (int beacon_count : config.monte_carlo_beacon_counts) {
            for (int trial = 0; trial < config.monte_carlo_trials_per_case; ++trial) {
                trials.push_back(run_trial(scenario, beacon_count, trial, config, rng));
            }
        }
    }
    return trials;
}

// See Simulation.hpp for the full contract. Implementation note: this
// accumulates running sums (rather than storing per-trial values) so the
// mean/RMSE/CI95 statistics below can all be derived in a single pass over
// `trials`; yaw statistics are averaged only over the subset of trials that
// report a valid (non-negative) beacon_yaw_rmse.
SummaryRow summarize(int scenario, int beacon_count, const std::vector<TrialResult>& trials) {
    SummaryRow row;
    row.scenario = scenario;
    row.beacons = beacon_count;
    double sum_error = 0.0;
    double sum_error2 = 0.0;
    double sum_error4 = 0.0;
    double sum_dx = 0.0;
    double sum_dy = 0.0;
    double sum_cost = 0.0;
    double sum_iterations = 0.0;
    double sum_runtime_ms = 0.0;
    double sum_beacon_position_rmse = 0.0;
    double sum_beacon_position_rmse2 = 0.0;
    double sum_beacon_yaw_rmse = 0.0;
    double sum_beacon_yaw_rmse2 = 0.0;
    int converged = 0;
    int count = 0;
    int yaw_count = 0;

    // Filter to the requested (scenario, beacon_count) pair and accumulate
    // running sums (including squared/4th-power sums needed for the CI95
    // computations below).
    for (const auto& trial : trials) {
        if (trial.scenario != scenario || trial.beacons != beacon_count) {
            continue;
        }
        const double dx = trial.estimate.x - trial.truth.x;
        const double dy = trial.estimate.y - trial.truth.y;
        sum_error += trial.error;
        sum_error2 += trial.error * trial.error;
        sum_error4 += trial.error * trial.error * trial.error * trial.error;
        sum_dx += dx;
        sum_dy += dy;
        sum_cost += trial.cost;
        sum_iterations += static_cast<double>(trial.iterations);
        sum_runtime_ms += trial.runtime_ms;
        sum_beacon_position_rmse += trial.beacon_position_rmse;
        sum_beacon_position_rmse2 += trial.beacon_position_rmse * trial.beacon_position_rmse;
        if (trial.beacon_yaw_rmse >= 0.0) {
            sum_beacon_yaw_rmse += trial.beacon_yaw_rmse;
            sum_beacon_yaw_rmse2 += trial.beacon_yaw_rmse * trial.beacon_yaw_rmse;
            ++yaw_count;
        }
        converged += trial.converged ? 1 : 0;
        ++count;
    }

    if (count > 0) {
        row.mean_error = sum_error / count;
        row.rmse = std::sqrt(sum_error2 / count);
        row.rmse_ci95 = rmse_ci95_from_sums(sum_error2, sum_error4, count);
        row.mean_error_ci95 = mean_ci95_from_sums(sum_error, sum_error2, count);
        row.bias_x = sum_dx / count;
        row.bias_y = sum_dy / count;
        row.mean_cost = sum_cost / count;
        row.mean_iterations = sum_iterations / count;
        row.mean_runtime_ms = sum_runtime_ms / count;
        row.convergence_rate = static_cast<double>(converged) / count;
        row.mean_beacon_position_rmse = sum_beacon_position_rmse / count;
        row.mean_beacon_position_rmse_ci95 =
            mean_ci95_from_sums(sum_beacon_position_rmse, sum_beacon_position_rmse2, count);
        row.mean_beacon_yaw_rmse =
            yaw_count > 0 ? sum_beacon_yaw_rmse / static_cast<double>(yaw_count) : -1.0;
        row.mean_beacon_yaw_rmse_ci95 =
            yaw_count > 0
                ? mean_ci95_from_sums(sum_beacon_yaw_rmse, sum_beacon_yaw_rmse2, yaw_count)
                : -1.0;
    }
    return row;
}

/**
 * @brief Factors the "run N trials, then reduce them via summarize()"
 * pattern shared by every Monte Carlo sweep below (noise, geometry,
 * trajectory, near-degenerate-trajectory, intermittent-measurement,
 * outlier, and vehicle-pose-noise). `run_one_trial(trial)` is called for
 * each trial index in [0, trial_count) and must return one TrialResult;
 * callers close over whatever per-sweep state (world, path, noise, rng,
 * measurement stress, ...) that call needs. This is a mechanical
 * extraction of the repeated loop-and-reduce boilerplate, not a behavioral
 * change: it preserves each sweep's exact original trial order and RNG
 * draw sequence.
 */
template <typename TrialRunner>
SummaryRow run_trials_and_summarize(
    int scenario, int beacon_count, int trial_count, TrialRunner&& run_one_trial) {
    std::vector<TrialResult> trials;
    trials.reserve(static_cast<std::size_t>(trial_count));
    for (int trial = 0; trial < trial_count; ++trial) {
        trials.push_back(run_one_trial(trial));
    }
    return summarize(scenario, beacon_count, trials);
}

// See Simulation.hpp for the full contract. Each (range_sigma, bearing_sigma)
// combination gets its own RNG, seeded deterministically from the noise
// values themselves so the sweep is reproducible without a shared stream.
std::vector<NoiseRobustnessRow> run_noise_robustness_sweep(const SimulationConfig& config) {
    const std::vector<double> range_sigmas{0.0, 0.01, 0.03, 0.06, 0.09};
    const std::vector<double> bearing_sigmas{0.0, 0.002, 0.006, 0.012, 0.018};
    const std::vector<int> beacon_counts{1, 2};
    std::vector<NoiseRobustnessRow> rows;
    rows.reserve(
        range_sigmas.size() * bearing_sigmas.size() *
        beacon_counts.size() * config.monte_carlo_scenarios.size());

    for (double range_sigma : range_sigmas) {
        for (double bearing_sigma : bearing_sigmas) {
            SimulationConfig sweep_config = config;
            sweep_config.monte_carlo_noise.range_sigma = range_sigma;
            sweep_config.monte_carlo_noise.bearing_sigma = bearing_sigma;
            const unsigned int seed_offset =
                static_cast<unsigned int>(100000.0 * range_sigma + 1000000.0 * bearing_sigma + 17.0);
            std::mt19937 rng(config.monte_carlo_seed + seed_offset);

            for (int scenario : config.monte_carlo_scenarios) {
                for (int beacon_count : beacon_counts) {
                    const SummaryRow summary = run_trials_and_summarize(
                        scenario, beacon_count, config.monte_carlo_trials_per_case,
                        [&](int trial) { return run_trial(scenario, beacon_count, trial, sweep_config, rng); });
                    rows.push_back({
                        scenario,
                        beacon_count,
                        range_sigma,
                        bearing_sigma,
                        summary.rmse,
                        summary.mean_beacon_position_rmse,
                        summary.mean_beacon_yaw_rmse,
                        summary.mean_cost,
                        summary.mean_iterations,
                        summary.mean_runtime_ms,
                        summary.convergence_rate,
                    });
                }
            }
        }
    }

    return rows;
}

// See Simulation.hpp for the full contract.
std::vector<GeometrySweepRow> run_geometry_sweep(const SimulationConfig& config) {
    const std::vector<double> separations{0.3, 0.6, 1.0, 1.6, 2.4, 3.2, 4.0};
    const auto path = make_vehicle_path(config.monte_carlo_path_steps, "excited");
    std::vector<GeometrySweepRow> rows;
    rows.reserve(separations.size());

    // Seed base for this sweep's RNG stream; offset from other sweeps'
    // bases below purely to keep their streams decorrelated, not for any
    // numerical significance.
    constexpr unsigned int kGeometrySweepSeedBase = 2000U;
    for (std::size_t i = 0; i < separations.size(); ++i) {
        const World world = make_world_with_beacon_separation(separations[i]);
        std::mt19937 rng(config.monte_carlo_seed + kGeometrySweepSeedBase + static_cast<unsigned int>(i));
        const SummaryRow summary = run_trials_and_summarize(
            1, 2, config.monte_carlo_trials_per_case,
            [&](int trial) {
                return run_trial_with_world_path(
                    1, 2, trial, world, path, config, config.monte_carlo_noise, rng);
            });
        rows.push_back({
            2,
            separations[i],
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }

    return rows;
}

// See Simulation.hpp for the full contract.
std::vector<TrajectorySweepRow> run_trajectory_sweep(const SimulationConfig& config) {
    const std::vector<std::string> trajectories{
        "stationary", "line", "circle", "figure_eight", "excited"};
    const World world = make_world(1);
    std::vector<TrajectorySweepRow> rows;
    rows.reserve(trajectories.size());

    constexpr unsigned int kTrajectorySweepRankSeedBase = 3000U;
    constexpr unsigned int kTrajectorySweepTrialSeedBase = 4000U;
    for (std::size_t i = 0; i < trajectories.size(); ++i) {
        const auto path = make_vehicle_path(config.monte_carlo_path_steps, trajectories[i]);
        // Noiseless observability diagnostic at the ground-truth state,
        // computed independently of (and before) the noisy Monte Carlo
        // trials below, so rank/sigma_min reflect trajectory geometry alone.
        std::mt19937 rank_rng(config.monte_carlo_seed + kTrajectorySweepRankSeedBase + static_cast<unsigned int>(i));
        const auto rank_measurements = generate_local_frame_measurements(world, path, Noise{0.0, 0.0}, rank_rng);
        const auto rank_and_sigma =
            local_observability_rank_and_sigma_min(true_state_scenario1(world), path, rank_measurements);

        std::mt19937 rng(config.monte_carlo_seed + kTrajectorySweepTrialSeedBase + static_cast<unsigned int>(i));
        const SummaryRow summary = run_trials_and_summarize(
            1, 1, config.monte_carlo_trials_per_case,
            [&](int trial) {
                return run_trial_with_world_path(
                    1, 1, trial, world, path, config, config.monte_carlo_noise, rng);
            });
        rows.push_back({
            trajectories[i],
            1,
            rank_and_sigma.first,
            rank_and_sigma.second,
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }

    return rows;
}

// See Simulation.hpp for the full contract.
std::vector<InitialPoseRobustnessRow> run_initial_pose_robustness_sweep(const SimulationConfig& config) {
    const std::vector<Vec2> starts{
        config.initial_robot,
        {-3.2, -2.4},
        {3.2, 2.4},
        {-3.4, 2.2},
        {3.4, -2.2},
        {0.0, 3.0},
    };

    std::vector<InitialPoseRobustnessRow> rows;
    rows.reserve(starts.size());
    constexpr unsigned int kInitialPoseRobustnessSeedBase = 500U;
    for (std::size_t i = 0; i < starts.size(); ++i) {
        SimulationConfig sweep_config = config;
        sweep_config.initial_robot = starts[i];
        std::mt19937 rng(config.closed_loop_seed + kInitialPoseRobustnessSeedBase + static_cast<unsigned int>(i));
        const ClosedLoopResult result = run_closed_loop_comparison(1, sweep_config, rng);
        const ClosedLoopPoint& final_point = result.points.back();
        rows.push_back({
            1,
            static_cast<int>(i),
            starts[i],
            final_point.goal_error,
            final_point.target_error,
            final_point.beacon_position_rmse,
            final_point.beacon_yaw_rmse,
        });
    }

    return rows;
}

// See Simulation.hpp for the full contract.
std::vector<MinimalBeaconExcitationRow> run_minimal_beacon_excitation_study(
    const SimulationConfig& config) {
    const World world = make_world(1);
    const auto full_path = make_vehicle_path(config.monte_carlo_path_steps);
    const std::vector<std::pair<const char*, int>> cases{
        {"single_pose_no_excitation", 1},
        {"two_distinct_poses", 2},
        {"full_excited_trajectory", config.monte_carlo_path_steps},
    };

    std::vector<MinimalBeaconExcitationRow> rows;
    rows.reserve(cases.size());

    constexpr unsigned int kMinimalBeaconExcitationSeedBase = 700U;
    for (std::size_t case_index = 0; case_index < cases.size(); ++case_index) {
        // Build each case's path from the same underlying full trajectory:
        // a single fixed pose, two poses well separated in time (roughly a
        // third of the trajectory apart), or the entire trajectory.
        const int poses = std::max(1, cases[case_index].second);
        std::vector<Vec2> path;
        path.reserve(static_cast<std::size_t>(poses));
        if (poses == 1) {
            path.push_back(full_path.front());
        } else if (poses == 2) {
            path.push_back(full_path.front());
            path.push_back(full_path[full_path.size() / 3U]);
        } else {
            path = full_path;
        }

        std::mt19937 measurement_rng(
            config.monte_carlo_seed + kMinimalBeaconExcitationSeedBase + static_cast<unsigned int>(case_index));
        const auto measurements = generate_local_frame_measurements(world, path, Noise{0.0, 0.0}, measurement_rng);
        const auto truth = true_state_scenario1(world);
        const auto metrics = local_observability_metrics(truth, path, measurements);

        auto seed = initial_state_scenario1(
            1,
            config.initial_target_estimate,
            config.initial_beacon_guess_radius,
            config.initial_beacon_guess_yaw);
        std::vector<double> closed_form_seed;
        if (two_view_closed_form_initial_state(1, path, measurements, closed_form_seed)) {
            seed = closed_form_seed;
        }
        const auto result = gauss_newton(
            seed,
            [&](const std::vector<double>& state) {
                return residuals_scenario1(state, 1, path, measurements);
            },
            config.batch_solver_max_iterations,
            config.batch_solver_initial_lambda,
            [&](const std::vector<double>& state) {
                return jacobian_scenario1(state, 1, path, measurements);
            });
        const Vec2 estimate{result.x[0], result.x[1]};
        const auto beacon_estimates = beacon_estimates_from_scenario1_state(result.x, 1);

        rows.push_back({
            cases[case_index].first,
            1,
            static_cast<int>(path.size()),
            metrics.rank,
            metrics.trajectory_spread,
            metrics.sigma_min,
            norm(estimate - world.target),
            beacon_position_rmse(world, beacon_estimates),
            beacon_yaw_rmse(world, beacon_estimates),
            result.cost,
            result.iterations,
            result.converged,
        });
    }

    return rows;
}

// See Simulation.hpp for the full contract. Six hand-picked cases probe
// increasingly poor seeds: nominal, then offset target seed at two
// magnitudes, a poor beacon-radius guess, a poor yaw guess, and finally a
// bad seed rescued by multistart (config.multistart_count starts instead
// of 1).
std::vector<PoorInitializationSweepRow> run_poor_initialization_sweep(
    const SimulationConfig& config) {
    const World world = make_world(1);
    const auto path = make_vehicle_path(config.monte_carlo_path_steps, "excited");
    const MeasurementStress stress;
    const std::vector<std::tuple<const char*, double, double, double, int>> cases{
        {"nominal_seed", 0.0, 2.0, 0.0, 1},
        {"target_seed_1m", 1.0, 2.0, 0.0, 1},
        {"target_seed_2m", 2.0, 2.0, 0.0, 1},
        {"poor_beacon_radius", 1.0, 0.7, 0.0, 1},
        {"poor_yaw_seed", 1.0, 2.0, 1.57, 1},
        {"poor_seed_multistart", 2.0, 2.8, 2.4, std::max(2, config.multistart_count)},
    };

    std::vector<PoorInitializationSweepRow> rows;
    rows.reserve(cases.size());
    constexpr unsigned int kPoorInitializationSweepSeedBase = 9000U;
    for (std::size_t case_index = 0; case_index < cases.size(); ++case_index) {
        const auto& [name, target_offset, beacon_radius, yaw_seed, multistarts] = cases[case_index];
        std::mt19937 rng(
            config.monte_carlo_seed + kPoorInitializationSweepSeedBase + static_cast<unsigned int>(case_index));
        std::vector<TrialResult> trials;
        trials.reserve(static_cast<std::size_t>(config.expanded_trials_per_case));
        for (int trial = 0; trial < config.expanded_trials_per_case; ++trial) {
            // Rotate the offset target seed around the true target per
            // trial, so the case's fixed offset magnitude is tested from
            // many different directions rather than just one.
            const double angle = 2.0 * kPi * static_cast<double>(trial) /
                static_cast<double>(std::max(1, config.expanded_trials_per_case));
            const Vec2 target_seed{
                world.target.x + target_offset * std::cos(angle),
                world.target.y + target_offset * std::sin(angle),
            };
            trials.push_back(run_multistart_local_batch_trial(
                trial,
                1,
                world,
                path,
                path,
                config,
                config.monte_carlo_noise,
                stress,
                target_seed,
                beacon_radius,
                yaw_seed,
                multistarts,
                false,
                rng));
        }
        const auto summary = summarize(1, 1, trials);
        rows.push_back({
            name,
            target_offset,
            beacon_radius,
            yaw_seed,
            multistarts,
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }
    return rows;
}

// See Simulation.hpp for the full contract.
std::vector<TrajectorySweepRow> run_near_degenerate_trajectory_sweep(
    const SimulationConfig& config) {
    const std::vector<std::string> trajectories{
        "stationary",
        "short_line",
        "repeated_viewpoints",
        "low_curvature_arc",
        "collinear_pass",
        "line",
        "excited_figure_eight",
    };
    const World world = make_world(1);
    std::vector<TrajectorySweepRow> rows;
    rows.reserve(trajectories.size());

    constexpr unsigned int kNearDegenerateTrajectorySweepRankSeedBase = 10000U;
    constexpr unsigned int kNearDegenerateTrajectorySweepTrialSeedBase = 11000U;
    for (std::size_t i = 0; i < trajectories.size(); ++i) {
        const auto path = make_vehicle_path(config.monte_carlo_path_steps, trajectories[i]);
        std::mt19937 rank_rng(
            config.monte_carlo_seed + kNearDegenerateTrajectorySweepRankSeedBase + static_cast<unsigned int>(i));
        const auto rank_measurements = generate_local_frame_measurements(world, path, Noise{0.0, 0.0}, rank_rng);
        const auto metrics = local_observability_metrics(true_state_scenario1(world), path, rank_measurements);

        std::mt19937 rng(
            config.monte_carlo_seed + kNearDegenerateTrajectorySweepTrialSeedBase + static_cast<unsigned int>(i));
        const auto summary = run_trials_and_summarize(
            1, 1, config.expanded_trials_per_case,
            [&](int trial) {
                return run_trial_with_world_path(
                    1, 1, trial, world, path, config, config.monte_carlo_noise, rng);
            });
        rows.push_back({
            trajectories[i],
            1,
            metrics.rank,
            metrics.sigma_min,
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }
    return rows;
}

// See Simulation.hpp for the full contract.
std::vector<IntermittentMeasurementSweepRow> run_intermittent_measurement_sweep(
    const SimulationConfig& config) {
    const World world = make_world(1);
    const auto path = make_vehicle_path(config.monte_carlo_path_steps, "excited");
    const std::vector<double> dropouts{
        0.0,
        0.25 * config.dropout_probability_max,
        0.50 * config.dropout_probability_max,
        0.75 * config.dropout_probability_max,
        config.dropout_probability_max,
    };

    std::vector<IntermittentMeasurementSweepRow> rows;
    rows.reserve(dropouts.size());
    constexpr unsigned int kIntermittentMeasurementSweepSeedBase = 12000U;
    for (std::size_t i = 0; i < dropouts.size(); ++i) {
        MeasurementStress stress;
        stress.dropout_probability = dropouts[i];
        std::mt19937 rng(
            config.monte_carlo_seed + kIntermittentMeasurementSweepSeedBase + static_cast<unsigned int>(i));
        double measurement_count = 0.0;
        const auto summary = run_trials_and_summarize(
            1, 1, config.expanded_trials_per_case,
            [&](int trial) {
                const auto measurements = generate_stressed_local_measurements(
                    world, path, config.monte_carlo_noise, stress, rng);
                measurement_count += static_cast<double>(measurements.size());
                const auto initial_state = initial_state_scenario1(
                    1,
                    config.initial_target_estimate,
                    config.initial_beacon_guess_radius,
                    config.initial_beacon_guess_yaw);
                return run_local_batch_trial_with_measurements(
                    trial, 1, world, path, measurements, config, config.monte_carlo_noise,
                    initial_state, false, 0.0);
            });
        rows.push_back({
            dropouts[i],
            measurement_count / static_cast<double>(std::max(1, config.expanded_trials_per_case)),
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }
    return rows;
}

// See Simulation.hpp for the full contract.
std::vector<OutlierRobustnessSweepRow> run_outlier_robustness_sweep(
    const SimulationConfig& config) {
    const World world = make_world(1);
    const auto path = make_vehicle_path(config.monte_carlo_path_steps, "excited");
    const std::vector<double> probabilities{0.0, 0.05, 0.10, 0.20};
    std::vector<OutlierRobustnessSweepRow> rows;
    rows.reserve(probabilities.size() * 2U);

    // Not routed through run_trials_and_summarize() above: each trial here
    // draws one set of stressed measurements and evaluates it under *two*
    // estimators (vanilla and Huber-robust), so the loop produces two
    // TrialResult vectors per iteration rather than the "one draw, one
    // result" shape that helper assumes.
    constexpr unsigned int kOutlierRobustnessSweepSeedBase = 13000U;
    for (std::size_t i = 0; i < probabilities.size(); ++i) {
        MeasurementStress stress;
        stress.outlier_probability = probabilities[i];
        stress.outlier_range_magnitude = config.outlier_range_magnitude;
        stress.outlier_bearing_magnitude = config.outlier_bearing_magnitude;
        std::mt19937 rng(config.monte_carlo_seed + kOutlierRobustnessSweepSeedBase + static_cast<unsigned int>(i));
        std::vector<TrialResult> vanilla_trials;
        std::vector<TrialResult> robust_trials;
        vanilla_trials.reserve(static_cast<std::size_t>(config.expanded_trials_per_case));
        robust_trials.reserve(static_cast<std::size_t>(config.expanded_trials_per_case));

        // Draw one set of stressed measurements per trial, then solve it
        // with both the vanilla (quadratic-loss) and Huber-robust
        // estimators, so the two rows at each probability level are
        // directly comparable (same outlier draws, only the loss differs).
        for (int trial = 0; trial < config.expanded_trials_per_case; ++trial) {
            const auto measurements = generate_stressed_local_measurements(
                world, path, config.monte_carlo_noise, stress, rng);
            const auto initial_state = initial_state_scenario1(
                1,
                config.initial_target_estimate,
                config.initial_beacon_guess_radius,
                config.initial_beacon_guess_yaw);
            vanilla_trials.push_back(run_local_batch_trial_with_measurements(
                trial, 1, world, path, measurements, config, config.monte_carlo_noise,
                initial_state, false, 0.0));
            robust_trials.push_back(run_local_batch_trial_with_measurements(
                trial, 1, world, path, measurements, config, config.monte_carlo_noise,
                initial_state, true, config.robust_huber_delta));
        }

        const auto vanilla = summarize(1, 1, vanilla_trials);
        const auto robust = summarize(1, 1, robust_trials);
        rows.push_back({
            "batch_gn",
            probabilities[i],
            config.outlier_range_magnitude,
            config.outlier_bearing_magnitude,
            vanilla.rmse,
            vanilla.mean_beacon_position_rmse,
            vanilla.mean_beacon_yaw_rmse,
            vanilla.mean_cost,
            vanilla.mean_iterations,
            vanilla.mean_runtime_ms,
            vanilla.convergence_rate,
        });
        rows.push_back({
            "robust_huber_batch_gn",
            probabilities[i],
            config.outlier_range_magnitude,
            config.outlier_bearing_magnitude,
            robust.rmse,
            robust.mean_beacon_position_rmse,
            robust.mean_beacon_yaw_rmse,
            robust.mean_cost,
            robust.mean_iterations,
            robust.mean_runtime_ms,
            robust.convergence_rate,
        });
    }
    return rows;
}

// See Simulation.hpp for the full contract. Runs two separate sub-sweeps:
// an iid-noise sweep over `sigmas` (this loop), followed by a single
// random-walk-drift case (the block below it).
std::vector<VehicleLocalizationNoiseSweepRow> run_vehicle_localization_noise_sweep(
    const SimulationConfig& config) {
    const World world = make_world(1);
    const auto true_path = make_vehicle_path(config.monte_carlo_path_steps, "excited");
    const MeasurementStress stress;
    const std::vector<double> sigmas{
        0.0,
        0.25 * config.vehicle_pose_noise_max,
        0.50 * config.vehicle_pose_noise_max,
        0.75 * config.vehicle_pose_noise_max,
        config.vehicle_pose_noise_max,
    };

    std::vector<VehicleLocalizationNoiseSweepRow> rows;
    rows.reserve(sigmas.size() + 1U);
    constexpr unsigned int kVehicleLocalizationNoiseSweepSeedBase = 14000U;
    constexpr unsigned int kVehicleLocalizationDriftSeedBase = 14500U;
    for (std::size_t i = 0; i < sigmas.size(); ++i) {
        std::mt19937 rng(
            config.monte_carlo_seed + kVehicleLocalizationNoiseSweepSeedBase + static_cast<unsigned int>(i));
        const auto summary = run_trials_and_summarize(
            1, 1, config.expanded_trials_per_case,
            [&](int trial) {
                // Measurements are generated from the true path, but the
                // estimator only sees a noisy version of it
                // (make_noisy_path), modeling imperfect vehicle
                // self-localization.
                const auto estimator_path = make_noisy_path(true_path, sigmas[i], rng);
                return run_local_batch_trial_with_seed(
                    trial,
                    1,
                    world,
                    true_path,
                    estimator_path,
                    config,
                    config.monte_carlo_noise,
                    stress,
                    config.initial_target_estimate,
                    config.initial_beacon_guess_radius,
                    config.initial_beacon_guess_yaw,
                    false,
                    rng);
            });
        rows.push_back({
            "iid_position_noise",
            sigmas[i],
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }
    // Second sub-sweep: a single random-walk-drift case (accumulating
    // dead-reckoning-style error, distinct from the iid noise above).
    {
        constexpr double drift_fraction = 0.005;
        std::mt19937 rng(config.monte_carlo_seed + kVehicleLocalizationDriftSeedBase);
        const auto summary = run_trials_and_summarize(
            1, 1, config.expanded_trials_per_case,
            [&](int trial) {
                const auto estimator_path = make_drifted_path(true_path, drift_fraction, rng);
                return run_local_batch_trial_with_seed(
                    trial,
                    1,
                    world,
                    true_path,
                    estimator_path,
                    config,
                    config.monte_carlo_noise,
                    stress,
                    config.initial_target_estimate,
                    config.initial_beacon_guess_radius,
                    config.initial_beacon_guess_yaw,
                    false,
                    rng);
            });
        rows.push_back({
            "random_walk_drift",
            drift_fraction,
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }
    return rows;
}

// See Simulation.hpp for the full contract.
std::vector<InformationConditioningRow> run_information_conditioning_sweep(
    const SimulationConfig& config) {
    const std::vector<std::string> trajectories{
        "stationary",
        "short_line",
        "repeated_viewpoints",
        "low_curvature_arc",
        "collinear_pass",
        "line",
        "circle",
        "figure_eight",
        "excited",
        "excited_figure_eight",
    };
    const World world = make_world(1);
    std::vector<InformationConditioningRow> rows;
    rows.reserve(trajectories.size());
    constexpr unsigned int kInformationConditioningSweepSeedBase = 15000U;
    for (std::size_t i = 0; i < trajectories.size(); ++i) {
        const auto path = make_vehicle_path(config.monte_carlo_path_steps, trajectories[i]);
        std::mt19937 rng(
            config.monte_carlo_seed + kInformationConditioningSweepSeedBase + static_cast<unsigned int>(i));
        const auto measurements = generate_local_frame_measurements(world, path, Noise{0.0, 0.0}, rng);
        const auto metrics = local_observability_metrics(true_state_scenario1(world), path, measurements);
        rows.push_back({
            trajectories[i],
            1,
            static_cast<int>(measurements.size()),
            metrics.rank,
            metrics.trajectory_spread,
            metrics.sigma_min,
            metrics.sigma_max,
            metrics.condition_number,
            metrics.logdet,
        });
    }
    return rows;
}

// See Simulation.hpp for the full contract. `estimator` is a string tag
// dispatched via a chain of comparisons below to the appropriate helper:
// EKF variants delegate straight to run_trial_with_world_path (scenario 3
// or 4); every other tag draws one common set of (unstressed) measurements
// and then routes to run_local_batch_trial_with_measurements() or
// run_multistart_local_batch_trial() with estimator-specific arguments
// (robust reweighting, a trailing sliding window, multistart, or a single
// target packet per beacon).
std::vector<ExpandedBaselineSummaryRow> run_expanded_baseline_comparison(
    const SimulationConfig& config) {
    const World world = make_world(1);
    const auto path = make_vehicle_path(config.monte_carlo_path_steps, "excited");
    const MeasurementStress no_stress;
    std::vector<ExpandedBaselineSummaryRow> rows;
    const std::vector<std::string> estimators{
        "batch_gn",
        "robust_huber_batch_gn",
        "multistart_batch_gn",
        "sliding_window_gn",
        "two_view_initialized_ekf",
        "naive_ekf",
        "single_target_packet_batch_gn",
    };
    rows.reserve(estimators.size());

    constexpr unsigned int kExpandedBaselineComparisonSeedBase = 16000U;
    for (const auto& estimator : estimators) {
        std::mt19937 rng(config.monte_carlo_seed + kExpandedBaselineComparisonSeedBase +
            static_cast<unsigned int>(rows.size() * 101U));
        std::vector<TrialResult> trials;
        trials.reserve(static_cast<std::size_t>(config.expanded_trials_per_case));
        for (int trial = 0; trial < config.expanded_trials_per_case; ++trial) {
            // EKF variants bypass the batch-measurement path entirely and
            // delegate straight to the EKF trial runner.
            if (estimator == "naive_ekf") {
                trials.push_back(run_trial_with_world_path(
                    3, 1, trial, world, path, config, config.monte_carlo_noise, rng));
                continue;
            }
            if (estimator == "two_view_initialized_ekf") {
                trials.push_back(run_trial_with_world_path(
                    4, 1, trial, world, path, config, config.monte_carlo_noise, rng));
                continue;
            }

            // All remaining (batch Gauss-Newton) variants share one set of
            // unstressed measurements and the generic circular seed, only
            // differing in which solver options/helper they route to below.
            const auto measurements = generate_stressed_local_measurements(
                world, path, config.monte_carlo_noise, no_stress, rng);
            const auto initial_state = initial_state_scenario1(
                1,
                config.initial_target_estimate,
                config.initial_beacon_guess_radius,
                config.initial_beacon_guess_yaw);
            if (estimator == "robust_huber_batch_gn") {
                trials.push_back(run_local_batch_trial_with_measurements(
                    trial, 1, world, path, measurements, config, config.monte_carlo_noise,
                    initial_state, true, config.robust_huber_delta));
            } else if (estimator == "single_target_packet_batch_gn") {
                trials.push_back(run_local_batch_trial_with_measurements(
                    trial, 1, world, path, measurements, config, config.monte_carlo_noise,
                    initial_state, false, 0.0, true, false));
            } else if (estimator == "multistart_batch_gn") {
                trials.push_back(run_multistart_local_batch_trial(
                    trial,
                    1,
                    world,
                    path,
                    path,
                    config,
                    config.monte_carlo_noise,
                    no_stress,
                    config.initial_target_estimate,
                    config.initial_beacon_guess_radius,
                    config.initial_beacon_guess_yaw,
                    config.multistart_count,
                    false,
                    rng));
            } else if (estimator == "sliding_window_gn") {
                const auto window = recent_measurement_window(measurements, config.sliding_window_size);
                trials.push_back(run_local_batch_trial_with_measurements(
                    trial, 1, world, path, window, config, config.monte_carlo_noise,
                    initial_state, false, 0.0));
            } else {
                trials.push_back(run_local_batch_trial_with_measurements(
                    trial, 1, world, path, measurements, config, config.monte_carlo_noise,
                    initial_state, false, 0.0));
            }
        }
        const int summary_scenario =
            estimator == "naive_ekf" ? 3 : estimator == "two_view_initialized_ekf" ? 4 : 1;
        const auto summary = summarize(summary_scenario, 1, trials);
        rows.push_back({
            estimator == "single_target_packet_batch_gn" ?
                "single_target_packet" : "gaussian_no_outliers",
            estimator,
            1,
            summary.rmse,
            summary.rmse_ci95,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_position_rmse_ci95,
            summary.mean_beacon_yaw_rmse,
            summary.mean_beacon_yaw_rmse_ci95,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }
    return rows;
}

}  // namespace adaptive
