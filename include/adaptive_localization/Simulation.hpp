#pragma once

#include <random>
#include <vector>

#include "adaptive_localization/Config.hpp"
#include "adaptive_localization/Types.hpp"

namespace adaptive {

enum class ClosedLoopExcitationMode {
    Circular,
    Information,
    Supervised,
};

// Runs the core adaptive-localization simulation from the paper:
//   epsilon_i = (r_i^v)^2 - ||xi - phat + r_i^t u_i||^2
//   phat_dot = -2 Gamma sum_i epsilon_i (xi - phat + r_i^t u_i)
//   xi_dot = -k(xi - phat)
AdaptiveLocalizationRun run_adaptive_localization(
    const SimulationConfig& config,
    std::mt19937& rng);

// Runs the online estimate-update-control comparison shown in the HTML viewer.
ClosedLoopResult run_closed_loop_comparison(
    int scenario,
    const SimulationConfig& config,
    std::mt19937& rng);

ClosedLoopResult run_closed_loop_comparison(
    int scenario,
    int beacon_count,
    const SimulationConfig& config,
    std::mt19937& rng);

ClosedLoopResult run_closed_loop_comparison(
    int scenario,
    int beacon_count,
    const SimulationConfig& config,
    std::mt19937& rng,
    ClosedLoopExcitationMode excitation_mode);

std::vector<ActiveExcitationComparisonRow> run_active_excitation_comparison(
    const SimulationConfig& config);

// Compares the fixed decaying-swirl schedule against the excitation-supervised
// controller of Algorithm 1 (CDC closed-loop paper): packets to reach the
// goal/target error thresholds, final accuracy, and how many times the
// supervised controller retriggered its excitation epoch.
std::vector<SupervisedExcitationComparisonRow> run_supervised_excitation_comparison(
    const SimulationConfig& config);

// Sweeps the fixed schedule's decay rate lambda in the no-transient scenario
// (vehicle initialized at the true target), comparing fixed vs supervised
// final calibration accuracy at each value. Establishes robustness to
// mismatch in the unknown time-to-adequate-excitation rather than
// superiority in one hand-selected failure case.
std::vector<SupervisedLambdaSweepRow> run_supervised_lambda_sweep(
    const SimulationConfig& config);

// Runs one batch localization trial for a scenario/beacon-count pair.
TrialResult run_trial(
    int scenario,
    int beacon_count,
    int trial,
    const SimulationConfig& config,
    std::mt19937& rng);

// Runs all configured Monte Carlo scenario/beacon-count combinations.
std::vector<TrialResult> run_monte_carlo(const SimulationConfig& config);

// Reduces trial-level results to RMSE, bias, convergence rate, and mean cost.
SummaryRow summarize(int scenario, int beacon_count, const std::vector<TrialResult>& trials);

// Sweeps measurement noise for both scenario models at the two-beacon setting.
std::vector<NoiseRobustnessRow> run_noise_robustness_sweep(const SimulationConfig& config);

std::vector<GeometrySweepRow> run_geometry_sweep(const SimulationConfig& config);

std::vector<TrajectorySweepRow> run_trajectory_sweep(const SimulationConfig& config);

// Sweeps initial robot pose for the uncalibrated closed-loop hidden-target model.
std::vector<InitialPoseRobustnessRow> run_initial_pose_robustness_sweep(const SimulationConfig& config);

// Demonstrates the one-beacon local-frame observability result by comparing
// single-pose, two-distinct-pose, and full-trajectory one-beacon datasets.
std::vector<MinimalBeaconExcitationRow> run_minimal_beacon_excitation_study(
    const SimulationConfig& config);

std::vector<PoorInitializationSweepRow> run_poor_initialization_sweep(
    const SimulationConfig& config);

std::vector<TrajectorySweepRow> run_near_degenerate_trajectory_sweep(
    const SimulationConfig& config);

std::vector<IntermittentMeasurementSweepRow> run_intermittent_measurement_sweep(
    const SimulationConfig& config);

std::vector<OutlierRobustnessSweepRow> run_outlier_robustness_sweep(
    const SimulationConfig& config);

std::vector<VehicleLocalizationNoiseSweepRow> run_vehicle_localization_noise_sweep(
    const SimulationConfig& config);

std::vector<InformationConditioningRow> run_information_conditioning_sweep(
    const SimulationConfig& config);

std::vector<ExpandedBaselineSummaryRow> run_expanded_baseline_comparison(
    const SimulationConfig& config);

}  // namespace adaptive
