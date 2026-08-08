// Types.hpp
//
// Plain-data (POD-style) structs shared across the whole codebase: the
// ground-truth World description, measurement records for the two estimation
// scenarios (uncalibrated local-frame vs. calibrated global-frame), solver
// outputs, and the many CSV "row" structs that carry aggregated Monte Carlo
// results out of Simulation.cpp and into Output.cpp's writers. None of these
// types have behavior beyond default field values -- they exist purely to
// give named shape to data passed between modules.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "adaptive_localization/Math.hpp"

namespace adaptive {

/**
 * Ground-truth scenario description used to generate synthetic measurements.
 * `target` is the hidden point the robot/beacons are ultimately trying to
 * localize. `beacons` are fixed anchor positions, and `beacon_yaws[i]` is
 * beacon i's own local-frame orientation offset relative to the global
 * frame. In the uncalibrated ("scenario 1") measurement model, these beacon
 * yaws are treated as UNKNOWN by the estimator and must be recovered jointly
 * with beacon position and target position -- this is the "self-calibration"
 * problem the paper addresses. `beacons` and `beacon_yaws` are always the
 * same length, indexed by beacon id.
 */
struct World {
    Vec2 target;
    std::vector<Vec2> beacons;
    std::vector<double> beacon_yaws;
};

/// Standard deviations of the synthetic Gaussian measurement noise applied
/// when generating range/bearing measurements. A value of 0.0 for either
/// field means "noiseless" for that channel.
struct Noise {
    double range_sigma = 0.03;
    double bearing_sigma = 0.006;
};

/**
 * A single "scenario 1" (uncalibrated local-frame) measurement packet from
 * one beacon at one time step: the beacon reports the range/bearing to the
 * vehicle (`rv`, `bv_local`) and to the hidden target (`rt`, `bt_local`),
 * both expressed in the beacon's own local/body frame (whose rotation
 * relative to the global frame is unknown to the estimator). `beacon` and
 * `time` index which beacon and which pose along the known vehicle path this
 * packet corresponds to.
 */
struct LocalFrameMeasurement {
    std::size_t beacon = 0;
    std::size_t time = 0;
    double rv = 0.0;
    double bv_local = 0.0;
    double rt = 0.0;
    double bt_local = 0.0;
};

/**
 * A single "scenario 2" (calibrated global-frame baseline) measurement
 * packet: range to the vehicle (`rv`), range to the target (`rt`), and the
 * bearing to the target expressed directly in the known GLOBAL frame
 * (`bt_global`) -- i.e. beacon orientation is already known/irrelevant here,
 * unlike LocalFrameMeasurement. Used as an "if calibration were already
 * solved" baseline for comparison against scenario 1.
 */
struct GlobalBearingMeasurement {
    std::size_t beacon = 0;
    std::size_t time = 0;
    double rv = 0.0;
    double rt = 0.0;
    double bt_global = 0.0;
};

/// Output of a nonlinear least-squares solve (see Solver.hpp's
/// gauss_newton): the final state vector `x`, the final whitened
/// sum-of-squared-residuals cost, the number of iterations actually run, and
/// whether the solver's convergence criteria (small step / small cost
/// improvement) were met before hitting `max_iterations`.
struct SolverResult {
    std::vector<double> x;
    double cost = 0.0;
    int iterations = 0;
    bool converged = false;
};

/// An estimated beacon pose: 2D position plus yaw (local-frame orientation
/// offset relative to the global frame). Used both as a piece of an
/// estimator's state and as a per-beacon summary of estimation error.
struct BeaconEstimate {
    Vec2 position;
    double yaw = 0.0;
};

/**
 * One time step of a simulated online (closed-loop) estimate-update-control
 * run: the true robot position (`robot`), the current target-position
 * estimate, the current per-beacon pose estimates, and error/cost metrics at
 * that step. `beacon_yaw_rmse` uses -1.0 as a sentinel meaning "not
 * applicable" (e.g. under scenario 2, where beacon yaw is not estimated).
 * `retriggered` marks steps where the excitation-supervised controller (see
 * Simulation.hpp's ClosedLoopExcitationMode::Supervised) restarted its
 * exploration epoch because observability had degraded below a threshold.
 */
struct ClosedLoopPoint {
    int step = 0;
    Vec2 robot;
    Vec2 target_estimate;
    std::vector<BeaconEstimate> beacon_estimates;
    double target_error = 0.0;
    double goal_error = 0.0;
    double beacon_position_rmse = 0.0;
    double beacon_yaw_rmse = -1.0;
    double cost = 0.0;
    bool retriggered = false;
};

/// Full trajectory of a closed-loop run: which `scenario` model was used,
/// the ground-truth `world`, the per-step history (`points`), the final
/// per-beacon pose estimates, and the final target-position estimate.
/// Written out by Output.cpp's CSV/SVG/HTML writers for visualization.
struct ClosedLoopResult {
    int scenario = 0;
    World world;
    std::vector<ClosedLoopPoint> points;
    std::vector<BeaconEstimate> beacon_estimates;
    Vec2 final_target_estimate;
};

/// One row of run_active_excitation_comparison's output: final accuracy/cost
/// metrics for a single closed-loop run under a named excitation strategy
/// (`excitation`, e.g. a fixed decaying swirl vs. an information-driven
/// steering law) at a given beacon count.
struct ActiveExcitationComparisonRow {
    std::string excitation;
    int beacons = 1;
    double final_goal_error = 0.0;
    double final_target_error = 0.0;
    double final_beacon_position_rmse = 0.0;
    double final_beacon_yaw_rmse = 0.0;
    double final_cost = 0.0;
};

/// One row of run_supervised_excitation_comparison's output: compares a
/// named excitation strategy's time-to-threshold behavior (steps needed to
/// bring goal/target error below configured thresholds -- see
/// SimulationConfig::supervised_goal_error_threshold /
/// supervised_target_error_threshold) and how many times its excitation
/// epoch was retriggered, against final accuracy/cost.
struct SupervisedExcitationComparisonRow {
    std::string excitation;
    int retrigger_count = 0;
    int steps_to_goal_threshold = -1;
    int steps_to_target_threshold = -1;
    double final_goal_error = 0.0;
    double final_target_error = 0.0;
    double final_beacon_position_rmse = 0.0;
    double final_beacon_yaw_rmse = 0.0;
    double final_cost = 0.0;
};

/// One row of run_supervised_lambda_sweep's output: at a given fixed-schedule
/// decay rate `lambda`, compares the fixed decaying-swirl excitation
/// schedule ("fixed_*" fields) against the excitation-supervised controller
/// ("supervised_*" fields), including how many times the supervised
/// controller retriggered its excitation epoch.
struct SupervisedLambdaSweepRow {
    double lambda = 0.0;
    int supervised_retrigger_count = 0;
    double fixed_final_target_error = 0.0;
    double fixed_final_beacon_position_rmse = 0.0;
    double fixed_final_beacon_yaw_rmse = 0.0;
    double supervised_final_target_error = 0.0;
    double supervised_final_beacon_position_rmse = 0.0;
    double supervised_final_beacon_yaw_rmse = 0.0;
};

/// Outcome of a single batch Monte Carlo trial (one noisy dataset, one
/// solve): which `scenario`/`beacons` configuration and `trial` index it
/// belongs to, the true vs. estimated target position, resulting errors,
/// final solver cost/iteration count/wall-clock runtime, and two distinct
/// convergence notions -- `solver_converged` reflects the Gauss-Newton
/// solver's own stopping criteria, while `converged` additionally requires
/// the trial to meet the accuracy thresholds checked by
/// trial_accuracy_success in Simulation.cpp. `beacon_yaw_rmse` is -1.0 when
/// beacon yaw is not estimated (e.g. scenario 2).
struct TrialResult {
    int scenario = 0;
    int beacons = 0;
    int trial = 0;
    Vec2 truth;
    Vec2 estimate;
    double error = 0.0;
    double beacon_position_rmse = 0.0;
    double beacon_yaw_rmse = -1.0;
    double cost = 0.0;
    int iterations = 0;
    double runtime_ms = 0.0;
    bool solver_converged = false;
    bool converged = false;
};

/// Aggregate statistics over a batch of TrialResult rows sharing a
/// scenario/beacon-count pair (see Simulation.hpp's `summarize`): RMSE and
/// mean error with 95% confidence intervals (`*_ci95`), per-axis bias,
/// fraction of trials that converged, and mean cost/iterations/runtime.
/// `mean_beacon_yaw_rmse`/`mean_beacon_yaw_rmse_ci95` are -1.0 when yaw is
/// not estimated for this scenario.
struct SummaryRow {
    int scenario = 0;
    int beacons = 0;
    double rmse = 0.0;
    double rmse_ci95 = 0.0;
    double mean_error = 0.0;
    double mean_error_ci95 = 0.0;
    double bias_x = 0.0;
    double bias_y = 0.0;
    double convergence_rate = 0.0;
    double mean_cost = 0.0;
    double mean_iterations = 0.0;
    double mean_runtime_ms = 0.0;
    double mean_beacon_position_rmse = 0.0;
    double mean_beacon_position_rmse_ci95 = 0.0;
    double mean_beacon_yaw_rmse = -1.0;
    double mean_beacon_yaw_rmse_ci95 = -1.0;
};

/// One row of run_noise_robustness_sweep's output: aggregated batch-trial
/// accuracy/cost/convergence at a given (`range_sigma`, `bearing_sigma`)
/// measurement-noise level, for a given scenario/beacon-count pair.
struct NoiseRobustnessRow {
    int scenario = 0;
    int beacons = 0;
    double range_sigma = 0.0;
    double bearing_sigma = 0.0;
    double target_rmse = 0.0;
    double beacon_position_rmse = 0.0;
    double beacon_yaw_rmse = -1.0;
    double mean_cost = 0.0;
    double mean_iterations = 0.0;
    double mean_runtime_ms = 0.0;
    double convergence_rate = 0.0;
};

/// One row of run_geometry_sweep's output: aggregated accuracy/cost as a
/// function of the physical separation between two beacons
/// (`beacon_separation`), holding beacon count fixed at 2.
struct GeometrySweepRow {
    int beacons = 2;
    double beacon_separation = 0.0;
    double target_rmse = 0.0;
    double beacon_position_rmse = 0.0;
    double beacon_yaw_rmse = 0.0;
    double mean_cost = 0.0;
    double mean_iterations = 0.0;
    double mean_runtime_ms = 0.0;
    double convergence_rate = 0.0;
};

/// One row of run_trajectory_sweep / run_near_degenerate_trajectory_sweep's
/// output: aggregated accuracy alongside observability diagnostics
/// (`observability_rank`, `smallest_singular_value` -- the paper's S_v) for
/// a named vehicle trajectory shape (see World.hpp's make_vehicle_path),
/// letting the reader correlate excitation quality with estimation error.
struct TrajectorySweepRow {
    std::string trajectory;
    int beacons = 1;
    int observability_rank = 0;
    double smallest_singular_value = 0.0;
    double target_rmse = 0.0;
    double beacon_position_rmse = 0.0;
    double beacon_yaw_rmse = 0.0;
    double mean_cost = 0.0;
    double mean_iterations = 0.0;
    double mean_runtime_ms = 0.0;
    double convergence_rate = 0.0;
};

/// One row of run_initial_pose_robustness_sweep's output: final closed-loop
/// accuracy for one trial starting from a particular `initial_robot` pose,
/// used to check sensitivity of the uncalibrated closed-loop estimator to
/// where the robot begins.
struct InitialPoseRobustnessRow {
    int scenario = 0;
    int trial = 0;
    Vec2 initial_robot;
    double final_goal_error = 0.0;
    double final_target_error = 0.0;
    double final_beacon_position_rmse = 0.0;
    double final_beacon_yaw_rmse = -1.0;
};

/// One row of run_minimal_beacon_excitation_study's output: demonstrates the
/// one-beacon local-frame observability result for a named dataset case
/// (`case_name`, e.g. single pose vs. two distinct poses vs. full
/// trajectory) with a given number of observed `poses`, reporting the
/// resulting observability rank/S_v alongside final estimation accuracy and
/// solver behavior.
struct MinimalBeaconExcitationRow {
    std::string case_name;
    int beacons = 1;
    int poses = 0;
    int observability_rank = 0;
    double trajectory_spread = 0.0;
    double smallest_singular_value = 0.0;
    double target_error = 0.0;
    double beacon_position_error = 0.0;
    double beacon_yaw_error = 0.0;
    double cost = 0.0;
    int iterations = 0;
    bool converged = false;
};

/// One row of run_poor_initialization_sweep's output: aggregated accuracy
/// when the Gauss-Newton solver is seeded from a deliberately poor initial
/// guess (`target_seed_offset`, `beacon_seed_radius`, `beacon_yaw_seed`
/// describe how far the seed is from truth), optionally using multiple
/// random restarts (`multistarts`, see Simulation.cpp's
/// run_multistart_local_batch_trial) to recover from local minima.
struct PoorInitializationSweepRow {
    std::string case_name;
    double target_seed_offset = 0.0;
    double beacon_seed_radius = 0.0;
    double beacon_yaw_seed = 0.0;
    int multistarts = 1;
    double target_rmse = 0.0;
    double beacon_position_rmse = 0.0;
    double beacon_yaw_rmse = 0.0;
    double mean_cost = 0.0;
    double mean_iterations = 0.0;
    double mean_runtime_ms = 0.0;
    double convergence_rate = 0.0;
};

/// One row of run_intermittent_measurement_sweep's output: aggregated
/// accuracy when measurements are randomly dropped with probability
/// `dropout_probability` (see Simulation.cpp's MeasurementStress /
/// generate_stressed_local_measurements), reporting the resulting mean
/// number of surviving measurements per trial and the effect on accuracy.
struct IntermittentMeasurementSweepRow {
    double dropout_probability = 0.0;
    double mean_measurements = 0.0;
    double target_rmse = 0.0;
    double beacon_position_rmse = 0.0;
    double beacon_yaw_rmse = 0.0;
    double mean_cost = 0.0;
    double mean_iterations = 0.0;
    double mean_runtime_ms = 0.0;
    double convergence_rate = 0.0;
};

/// One row of run_outlier_robustness_sweep's output: aggregated accuracy for
/// a named `estimator` variant (e.g. standard least squares vs. a
/// Huber-robust cost) when a fraction of measurements (`outlier_probability`)
/// are corrupted by a fixed range/bearing offset
/// (`outlier_range_magnitude`/`outlier_bearing_magnitude`), used to compare
/// robustness of different loss functions to gross sensor errors.
struct OutlierRobustnessSweepRow {
    std::string estimator;
    double outlier_probability = 0.0;
    double outlier_range_magnitude = 0.0;
    double outlier_bearing_magnitude = 0.0;
    double target_rmse = 0.0;
    double beacon_position_rmse = 0.0;
    double beacon_yaw_rmse = 0.0;
    double mean_cost = 0.0;
    double mean_iterations = 0.0;
    double mean_runtime_ms = 0.0;
    double convergence_rate = 0.0;
};

/// One row of run_vehicle_localization_noise_sweep's output: aggregated
/// accuracy for a named case as the standard deviation of the (otherwise
/// assumed-known) vehicle path position, `vehicle_position_sigma`, is
/// increased -- i.e. how sensitive the self-calibration estimate is to
/// errors in the robot's own localization.
struct VehicleLocalizationNoiseSweepRow {
    std::string case_name;
    double vehicle_position_sigma = 0.0;
    double target_rmse = 0.0;
    double beacon_position_rmse = 0.0;
    double beacon_yaw_rmse = 0.0;
    double mean_cost = 0.0;
    double mean_iterations = 0.0;
    double mean_runtime_ms = 0.0;
    double convergence_rate = 0.0;
};

/// One row of run_information_conditioning_sweep's output: pure
/// observability/conditioning diagnostics (no estimation error) for a named
/// `trajectory` shape with a given number of `observations` -- the rank,
/// smallest/largest singular values (S_v and its counterpart) of the local
/// observability normal matrix (see Simulation.cpp's
/// local_observability_metrics), their ratio (`condition_number`), and the
/// log-determinant of the information matrix (`logdet_information`, an
/// information-theoretic summary used as the objective for
/// information-driven excitation).
struct InformationConditioningRow {
    std::string trajectory;
    int beacons = 1;
    int observations = 0;
    int observability_rank = 0;
    double trajectory_spread = 0.0;
    double smallest_singular_value = 0.0;
    double largest_singular_value = 0.0;
    double condition_number = 0.0;
    double logdet_information = 0.0;
};

/// One row of run_expanded_baseline_comparison's output: aggregated accuracy
/// (with 95% confidence intervals) for a named `case_name`/`estimator`
/// combination -- the broadest robustness comparison table, spanning the
/// various stress conditions (noise, dropout, outliers, vehicle-pose noise,
/// poor initialization) exercised by the other sweeps above.
struct ExpandedBaselineSummaryRow {
    std::string case_name;
    std::string estimator;
    int beacons = 1;
    double target_rmse = 0.0;
    double target_rmse_ci95 = 0.0;
    double beacon_position_rmse = 0.0;
    double beacon_position_rmse_ci95 = 0.0;
    double beacon_yaw_rmse = 0.0;
    double beacon_yaw_rmse_ci95 = 0.0;
    double mean_cost = 0.0;
    double mean_iterations = 0.0;
    double mean_runtime_ms = 0.0;
    double convergence_rate = 0.0;
};

/// One time step of the continuous-time adaptive-localization run (see
/// Simulation.hpp's run_adaptive_localization): the true robot position,
/// current target-position estimate, per-beacon estimates implied by that
/// target estimate, and the resulting error/cost at that step. Distinct from
/// ClosedLoopPoint in that it is produced by integrating the paper's
/// continuous adaptive law rather than by repeated discrete Gauss-Newton
/// solves.
struct AdaptiveLocalizationPoint {
    int step = 0;
    Vec2 robot;
    Vec2 target_estimate;
    std::vector<BeaconEstimate> beacon_estimates;
    double target_error = 0.0;
    double goal_error = 0.0;
    double cost = 0.0;
};

/// Full trajectory of a run_adaptive_localization simulation: the
/// ground-truth `world` and the per-step history (`points`).
struct AdaptiveLocalizationRun {
    World world;
    std::vector<AdaptiveLocalizationPoint> points;
};

}  // namespace adaptive
