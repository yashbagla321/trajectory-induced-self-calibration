#pragma once

#include <array>
#include <random>
#include <utility>
#include <vector>

#include "adaptive_localization/Config.hpp"
#include "adaptive_localization/Types.hpp"

/**
 * @file Simulation.hpp
 * @brief Public entry points for the trajectory-induced self-calibration
 * simulator: the continuous-time adaptive estimator, the animated
 * closed-loop (online estimate/update/control) demos, the batch Monte Carlo
 * trial harness (scenario 1 uncalibrated local-frame self-calibration vs.
 * scenario 2 calibrated global-frame baseline, plus EKF variants 3/4), and
 * the family of robustness sweeps (noise, geometry, trajectory shape,
 * initialization, dropout/outlier stress, vehicle-pose noise, and
 * information-conditioning) used to generate the paper's result tables.
 * All heavy lifting lives in the corresponding Simulation.cpp; this header
 * only exposes the functions/rows needed by the CLI driver and Output.cpp.
 */

namespace adaptive {

/**
 * @brief Eigenvalues (ascending) of a symmetric 5x5 matrix via the cyclic
 * Jacobi algorithm; see Simulation.cpp for the full algorithm description.
 * Exposed here (rather than kept file-private) so it is directly testable
 * against matrices with known closed-form eigenvalues; its only production
 * caller remains within Simulation.cpp's local-observability diagnostics.
 *
 * @param a Row-major flattened symmetric 5x5 matrix.
 * @return The 5 eigenvalues in ascending order.
 */
std::array<double, 5> jacobi_eigenvalues(std::array<double, 25> a);

/**
 * @brief Selects the excitation policy used by the animated closed-loop
 * simulation (run_closed_loop_comparison) to keep the robot's trajectory
 * informative enough for the joint target/beacon self-calibration solve.
 */
enum class ClosedLoopExcitationMode {
    /** Fixed, decaying-amplitude circular "swirl" added on top of the
     *  target-seeking control law; the default excitation schedule used
     *  throughout most of the paper's closed-loop results. */
    Circular,
    /** Greedy excitation driven by the finite-difference gradient of the
     *  predicted local-observability log-determinant score (see
     *  information_driven_excitation() in Simulation.cpp) instead of a
     *  fixed schedule. */
    Information,
    /** Excitation-supervised mode (Algorithm 1 in the CDC closed-loop
     *  paper): the decaying schedule's epoch is retriggered while the
     *  stored window's trajectory spread S_v (computed from the known
     *  measurement poses, see Math.hpp's path_spread) is below the
     *  configured threshold, rather than decaying unconditionally.
     *  Conditioning (sigma_min) is logged per step as a diagnostic but is
     *  deliberately never a trigger: the finite-acquisition guarantee
     *  covers only the spread threshold. */
    Supervised,
};

/**
 * @brief Runs the continuous-time adaptive-localization core simulation
 * from the paper's rigorous global-frame model. At every step this
 * integrates forward in time:
 *   epsilon_i = (r_i^v)^2 - ||xi - phat + r_i^t u_i||^2
 *   phat_dot  = -2 Gamma sum_i epsilon_i (xi - phat + r_i^t u_i)
 *   xi_dot    = -k(xi - phat)
 * where phat is the target-position estimate, xi is the robot position, and
 * epsilon_i is the per-beacon prediction-error ("innovation") signal. This
 * is a gradient-like adaptive law integrated with a fixed Euler step,
 * contrasting with the discrete-time Gauss-Newton/EKF estimators used by
 * the batch trial and closed-loop functions below.
 *
 * @param config Simulation configuration; see the `adaptive_*` fields of
 *        SimulationConfig for step count, gains, and integration step size.
 * @param rng    Random engine used to draw per-step measurement noise.
 * @return The world (truth) and the full time series of robot/target/beacon
 *         estimates and errors recorded at every step.
 */
AdaptiveLocalizationRun run_adaptive_localization(
    const SimulationConfig& config,
    std::mt19937& rng);

/**
 * @brief Runs the online estimate-update-control comparison shown in the
 * HTML viewer, using the fixed decaying-circular excitation schedule and
 * the beacon count from `config.closed_loop_beacon_count`.
 *
 * @param scenario Measurement/estimation model: 1 = uncalibrated
 *        local-frame self-calibration (batch Gauss-Newton re-solved every
 *        step), 2 = calibrated global-frame baseline.
 * @param config   Simulation configuration (closed-loop step count, gains,
 *        noise, solver settings).
 * @param rng      Random engine used to draw per-step measurement noise.
 * @return The simulated world, the per-step trajectory of estimates/errors,
 *         and the final beacon/target estimates.
 */
ClosedLoopResult run_closed_loop_comparison(
    int scenario,
    const SimulationConfig& config,
    std::mt19937& rng);

/**
 * @brief Overload of run_closed_loop_comparison() that lets the caller pick
 * the beacon count explicitly instead of using
 * `config.closed_loop_beacon_count`. Uses the fixed decaying-circular
 * excitation schedule (ClosedLoopExcitationMode::Circular).
 */
ClosedLoopResult run_closed_loop_comparison(
    int scenario,
    int beacon_count,
    const SimulationConfig& config,
    std::mt19937& rng);

/**
 * @brief Full form of run_closed_loop_comparison() that also lets the
 * caller select the excitation policy (see ClosedLoopExcitationMode). This
 * is the implementation the other two overloads delegate to.
 *
 * @param scenario        1 = uncalibrated local-frame model, 2 = calibrated
 *        global-frame baseline.
 * @param beacon_count    Number of beacons to place in the simulated world.
 * @param config          Simulation configuration.
 * @param rng             Random engine used to draw per-step measurement
 *        noise.
 * @param excitation_mode Which excitation policy drives the robot's
 *        exploratory motion on top of target-seeking control.
 */
ClosedLoopResult run_closed_loop_comparison(
    int scenario,
    int beacon_count,
    const SimulationConfig& config,
    std::mt19937& rng,
    ClosedLoopExcitationMode excitation_mode);

/**
 * @brief Compares the fixed decaying-circular excitation schedule against
 * the information-driven (predicted logdet gradient) excitation policy for
 * a single-beacon closed loop, reporting final accuracy/cost for each. Both
 * runs share the same RNG seed so differences are attributable to the
 * excitation policy rather than noise draws.
 */
std::vector<ActiveExcitationComparisonRow> run_active_excitation_comparison(
    const SimulationConfig& config);

/**
 * @brief Runs the fixed-schedule vs. excitation-supervised closed-loop
 * comparison (Algorithm 1 in the CDC closed-loop paper): packets to reach
 * the goal/target error thresholds, final accuracy, and how many times the
 * supervised controller retriggered its excitation epoch, under two scenarios: a nominal case
 * (default configuration) and an "understimulated" case (fast decay, robot
 * initialized exactly at the true target so there is no convergence
 * transient to supply excitation on its own). Both scenarios use paired RNG
 * seeds between the fixed and supervised runs.
 *
 * @return One SupervisedExcitationComparisonRow per {schedule, scenario}
 *         combination (four rows total).
 */
std::vector<SupervisedExcitationComparisonRow> run_supervised_excitation_comparison(
    const SimulationConfig& config);

/**
 * @brief Sweeps the fixed schedule's decay rate lambda in the no-transient
 * scenario (vehicle initialized at the true target), comparing fixed vs
 * supervised final calibration accuracy at each value over a paired Monte
 * Carlo batch (`config.supervised_monte_carlo_trials` trials per lambda,
 * fixed and supervised sharing the same seed per trial). Establishes
 * robustness to mismatch in the unknown time-to-adequate-excitation rather
 * than superiority in one hand-selected failure case.
 *
 * @return One SupervisedLambdaSweepRow per swept lambda, carrying
 *         across-trial RMSEs with bootstrap 95% CIs, yaw success rates, and
 *         the supervised controller's mean retrigger count.
 */
std::vector<SupervisedLambdaSweepRow> run_supervised_lambda_sweep(
    const SimulationConfig& config);

/**
 * @brief Paired Monte Carlo comparison in the nontrivial target-seeking
 * scenario: the vehicle starts at the wrong initial target estimate
 * (seeking term initially quiescent), so target-seeking success depends on
 * the excitation policy generating enough spread to calibrate.
 *
 * @return One aggregate SupervisedSeekingComparisonRow per policy (fixed
 *         decaying-circular vs excitation-supervised).
 */
std::vector<SupervisedSeekingComparisonRow> run_supervised_seeking_comparison(
    const SimulationConfig& config);

/**
 * @brief Monte Carlo ablation over the supervisor's spread threshold S_bar
 * in the understimulated scenario (supervised policy only), holding the
 * trial success criterion fixed at the paper's declared 0.05-rad yaw
 * accuracy. Each row pairs the design rule's predicted yaw RMSE
 * sigma / sqrt(S_bar) with the measured across-trial RMSE and the
 * excitation cost of reaching that threshold (packets, underexcited
 * packets, episodes, path length, integrated effort).
 */
std::vector<SupervisedThresholdAblationRow> run_supervised_threshold_ablation(
    const SimulationConfig& config);

/**
 * @brief Rank and smallest singular value of the whitened stacked Jacobian
 * of the single-beacon scenario-1 model at `state`, whitened with `noise`.
 * This is the conditioning diagnostic the closed-loop experiments log
 * alongside the spread certificate (it is deliberately never a control
 * trigger). The rank counts singular values above a relative threshold
 * `max(1e-6, sigma_max*1e-7)`; when rank < 5, sigma_min is reported as 0.0
 * rather than a meaningless near-zero value, since a gauge-degenerate
 * direction exists (e.g. from a stationary or collinear trajectory) and no
 * singular value meaningfully bounds the estimator's conditioning. Returns
 * rank 0 / sigma_min 0 for any state that is not the 5-dimensional
 * single-beacon model. Exported so the ROS 2 / Gazebo closed-loop node logs
 * the same quantity as the batch simulator.
 *
 * @return {rank, sigma_min}; sigma_min is 0.0 whenever rank != 5.
 */
std::pair<int, double> local_observability_rank_and_sigma_min(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise = Noise{});

/**
 * @brief Runs one Monte Carlo trial: builds a fresh world and vehicle path,
 * draws noisy measurements with `rng`, and solves scenario 1 (uncalibrated
 * local-frame self-calibration, closed-form two-view seed + damped
 * Gauss-Newton), scenario 2 (calibrated global-frame baseline), or the EKF
 * variants (scenario 3 generic seed / 4 two-view seed) depending on
 * `scenario`.
 *
 * @param scenario     1, 2, 3, or 4 (see run_trial_with_world_path in
 *        Simulation.cpp for the full scenario dispatch).
 * @param beacon_count Number of beacons in the generated world.
 * @param trial        Trial index, recorded in the returned TrialResult for
 *        bookkeeping (does not affect the RNG draw itself).
 * @param config       Simulation configuration (path length, solver
 *        settings, noise defaults come from `config.monte_carlo_noise`).
 * @param rng          Random engine advanced by this call's noise draws.
 */
TrialResult run_trial(
    int scenario,
    int beacon_count,
    int trial,
    const SimulationConfig& config,
    std::mt19937& rng);

/**
 * @brief Runs `config.monte_carlo_trials_per_case` trials of run_trial()
 * for every combination of `config.monte_carlo_scenarios` x
 * `config.monte_carlo_beacon_counts`, sharing a single RNG stream seeded
 * from `config.monte_carlo_seed` across the whole sweep (so results are
 * reproducible but trials are not independently re-seeded).
 */
std::vector<TrialResult> run_monte_carlo(const SimulationConfig& config);

/**
 * @brief Filters `trials` down to the given {scenario, beacon_count} pair
 * and aggregates them into a single SummaryRow: target-position RMSE (with
 * 95% CI), mean signed bias per axis, convergence rate, mean solver
 * cost/iterations/runtime, and mean beacon position/yaw RMSE (yaw RMSE is
 * averaged only over trials that report it, i.e. scenario 1/3/4).
 *
 * @param scenario     Scenario ID to filter on.
 * @param beacon_count Beacon count to filter on.
 * @param trials       Pool of trial results to filter and aggregate; only
 *        entries matching (scenario, beacon_count) contribute.
 * @return A SummaryRow with all-zero statistics if no trial matches.
 */
SummaryRow summarize(int scenario, int beacon_count, const std::vector<TrialResult>& trials);

/**
 * @brief Sweeps range and bearing measurement noise (Cartesian product of a
 * small grid of sigmas) for every configured scenario at beacon counts
 * {1, 2}, running a full Monte Carlo batch at each noise level and
 * beacon count via run_trial()/summarize().
 */
std::vector<NoiseRobustnessRow> run_noise_robustness_sweep(const SimulationConfig& config);

/**
 * @brief Sweeps the physical separation between two beacons (fixed
 * excitation trajectory, scenario 1) and reports how estimation accuracy
 * degrades as the beacons move closer together / farther apart.
 */
std::vector<GeometrySweepRow> run_geometry_sweep(const SimulationConfig& config);

/**
 * @brief Runs the single-beacon scenario-1 Monte Carlo batch over a fixed
 * set of named vehicle trajectories (stationary, line, circle, figure
 * eight, excited), reporting local observability rank/sigma_min for the
 * noiseless geometry alongside the noisy-trial RMSE statistics, so the
 * observability diagnostics can be cross-referenced against actual
 * estimation accuracy for each trajectory shape.
 */
std::vector<TrajectorySweepRow> run_trajectory_sweep(const SimulationConfig& config);

/**
 * @brief Runs the scenario-1 closed-loop comparison (run_closed_loop_comparison)
 * from a fixed set of initial robot positions (including the configured
 * default) and reports each run's final goal/target error and beacon
 * position/yaw RMSE, showing sensitivity (or lack thereof) to the initial
 * pose of the closed-loop transient.
 */
std::vector<InitialPoseRobustnessRow> run_initial_pose_robustness_sweep(const SimulationConfig& config);

/**
 * @brief For a single beacon, compares three noiseless-measurement
 * datasets of increasing viewing diversity ("single_pose_no_excitation",
 * "two_distinct_poses", "full_excited_trajectory"), reporting local
 * observability rank/sigma_min/trajectory_spread alongside the batch
 * Gauss-Newton solve's target/beacon errors for each, to demonstrate that a
 * single vehicle pose under-observes the 5-dimensional single-beacon state
 * (target x,y + beacon x,y,yaw) while two well-separated poses (or a full
 * trajectory) recover full rank.
 */
std::vector<MinimalBeaconExcitationRow> run_minimal_beacon_excitation_study(
    const SimulationConfig& config);

/**
 * @brief Sweeps a fixed set of deliberately poor initial-seed cases
 * (target-seed offset, beacon-radius seed, beacon-yaw seed, and a
 * multistart case) for the single-beacon scenario-1 batch solve, showing
 * how far the damped Gauss-Newton solve tolerates a bad initialization
 * before multistart (run_multistart_local_batch_trial) is needed to recover
 * the global optimum.
 */
std::vector<PoorInitializationSweepRow> run_poor_initialization_sweep(
    const SimulationConfig& config);

/**
 * @brief Same reporting as run_trajectory_sweep() (local observability
 * rank/sigma_min plus Monte Carlo RMSE) but over a set of trajectories
 * chosen to probe near-degenerate/weakly-excited geometries (stationary,
 * short line, repeated viewpoints, low-curvature arc, collinear pass) next
 * to well-excited baselines (line, excited figure eight).
 */
std::vector<TrajectorySweepRow> run_near_degenerate_trajectory_sweep(
    const SimulationConfig& config);

/**
 * @brief Sweeps measurement-dropout probability (Bernoulli-discarded
 * measurements, see MeasurementStress in Simulation.cpp) from 0 up to
 * `config.dropout_probability_max`, running single-beacon scenario-1 batch
 * trials at each level and reporting mean surviving measurement count
 * alongside RMSE/convergence statistics.
 */
std::vector<IntermittentMeasurementSweepRow> run_intermittent_measurement_sweep(
    const SimulationConfig& config);

/**
 * @brief Sweeps gross-outlier probability (see MeasurementStress) and, for
 * each level, runs both a plain (vanilla) batch Gauss-Newton estimator and a
 * Huber-robust-loss variant (huber_scaled_residuals) on the *same* stressed
 * measurement draws, so the two rows per probability level are directly
 * comparable.
 */
std::vector<OutlierRobustnessSweepRow> run_outlier_robustness_sweep(
    const SimulationConfig& config);

/**
 * @brief Sweeps how much iid Gaussian noise (and, in one additional case, a
 * random-walk drift) is applied to the vehicle's *own* self-localization
 * estimate (the "known" path used by the estimator) versus the true path
 * used to generate measurements, reporting how estimation accuracy degrades
 * as the assumption of a perfectly known trajectory is relaxed.
 */
std::vector<VehicleLocalizationNoiseSweepRow> run_vehicle_localization_noise_sweep(
    const SimulationConfig& config);

/**
 * @brief Computes local-observability/Fisher-information diagnostics
 * (rank, trajectory_spread, sigma_min, sigma_max, condition_number, logdet)
 * for a single beacon across a broad set of named trajectories, from
 * degenerate (stationary, collinear) to well-excited (excited_figure_eight),
 * using noiseless measurements so the metrics reflect trajectory geometry
 * alone rather than sensor noise.
 */
std::vector<InformationConditioningRow> run_information_conditioning_sweep(
    const SimulationConfig& config);

/**
 * @brief Runs an expanded head-to-head comparison of estimator variants
 * (plain batch Gauss-Newton, Huber-robust batch, multistart batch, a
 * sliding-window batch that only sees the most recent measurements, the EKF
 * with a naive seed, the EKF with the two-view closed-form seed, and a
 * batch variant that only ingests one target packet per beacon) on the same
 * single-beacon excited trajectory, reporting RMSE with 95% CI, mean
 * cost/iterations/runtime, and convergence rate for each.
 */
std::vector<ExpandedBaselineSummaryRow> run_expanded_baseline_comparison(
    const SimulationConfig& config);

}  // namespace adaptive
