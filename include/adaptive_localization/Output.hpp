// Output.hpp
//
// Serialization layer: writes every Monte Carlo/sweep result-row vector (see
// Types.hpp) to CSV, writes closed-loop and adaptive-localization run
// trajectories to CSV/SVG, and assembles a self-contained HTML viewer for
// visually inspecting closed-loop runs. All functions here are pure I/O --
// they format and write data that Simulation.cpp/main.cpp already computed.

#pragma once

#include <filesystem>
#include <vector>

#include "adaptive_localization/Types.hpp"

namespace adaptive {

/// Writes aggregated Monte Carlo summary statistics (see SummaryRow) to a
/// CSV file at `path`, one row per scenario/beacon-count combination.
void write_summary_csv(const std::filesystem::path& path, const std::vector<SummaryRow>& rows);
/// Writes per-trial Monte Carlo results (see TrialResult) to a CSV file.
void write_trial_csv(const std::filesystem::path& path, const std::vector<TrialResult>& trials);
/// Writes the measurement-noise robustness sweep results (see
/// NoiseRobustnessRow) to a CSV file.
void write_noise_robustness_csv(const std::filesystem::path& path, const std::vector<NoiseRobustnessRow>& rows);
/// Writes the beacon-separation geometry sweep results (see
/// GeometrySweepRow) to a CSV file.
void write_geometry_sweep_csv(const std::filesystem::path& path, const std::vector<GeometrySweepRow>& rows);
/// Writes the named-trajectory sweep results (see TrajectorySweepRow,
/// including observability rank/S_v diagnostics) to a CSV file.
void write_trajectory_sweep_csv(const std::filesystem::path& path, const std::vector<TrajectorySweepRow>& rows);
/// Writes the initial-robot-pose robustness sweep results (see
/// InitialPoseRobustnessRow) to a CSV file.
void write_initial_pose_robustness_csv(
    const std::filesystem::path& path,
    const std::vector<InitialPoseRobustnessRow>& rows);
/// Writes the one-beacon minimal-excitation study results (see
/// MinimalBeaconExcitationRow) to a CSV file.
void write_minimal_beacon_excitation_csv(
    const std::filesystem::path& path,
    const std::vector<MinimalBeaconExcitationRow>& rows);
/// Writes the poor-initial-guess / multistart sweep results (see
/// PoorInitializationSweepRow) to a CSV file.
void write_poor_initialization_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<PoorInitializationSweepRow>& rows);
/// Writes the measurement-dropout sweep results (see
/// IntermittentMeasurementSweepRow) to a CSV file.
void write_intermittent_measurement_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<IntermittentMeasurementSweepRow>& rows);
/// Writes the outlier-robustness sweep results (see
/// OutlierRobustnessSweepRow) to a CSV file.
void write_outlier_robustness_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<OutlierRobustnessSweepRow>& rows);
/// Writes the vehicle-self-localization-noise sweep results (see
/// VehicleLocalizationNoiseSweepRow) to a CSV file.
void write_vehicle_localization_noise_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<VehicleLocalizationNoiseSweepRow>& rows);
/// Writes the pure observability/conditioning diagnostics (see
/// InformationConditioningRow) to a CSV file.
void write_information_conditioning_csv(
    const std::filesystem::path& path,
    const std::vector<InformationConditioningRow>& rows);
/// Writes the broad multi-condition baseline comparison results (see
/// ExpandedBaselineSummaryRow) to a CSV file.
void write_expanded_baseline_summary_csv(
    const std::filesystem::path& path,
    const std::vector<ExpandedBaselineSummaryRow>& rows);
/// Writes the fixed-vs-active excitation strategy comparison results (see
/// ActiveExcitationComparisonRow) to a CSV file.
void write_active_excitation_comparison_csv(
    const std::filesystem::path& path,
    const std::vector<ActiveExcitationComparisonRow>& rows);
/// Writes the fixed-vs-excitation-supervised controller comparison results
/// (see SupervisedExcitationComparisonRow) to a CSV file.
void write_supervised_excitation_comparison_csv(
    const std::filesystem::path& path,
    const std::vector<SupervisedExcitationComparisonRow>& rows);
/// Writes the fixed-schedule decay-rate Monte Carlo sweep results (see
/// SupervisedLambdaSweepRow) to a CSV file.
void write_supervised_lambda_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<SupervisedLambdaSweepRow>& rows);
/// Writes the nontrivial target-seeking Monte Carlo comparison results (see
/// SupervisedSeekingComparisonRow) to a CSV file.
void write_supervised_seeking_comparison_csv(
    const std::filesystem::path& path,
    const std::vector<SupervisedSeekingComparisonRow>& rows);
/// Writes the supervisor spread-threshold Monte Carlo ablation results (see
/// SupervisedThresholdAblationRow) to a CSV file.
void write_supervised_threshold_ablation_csv(
    const std::filesystem::path& path,
    const std::vector<SupervisedThresholdAblationRow>& rows);
/// Convenience entry point that runs and writes a small fixed set of example
/// CSVs into `output_dir`, useful for smoke-testing the output pipeline.
void write_example_csvs(const std::filesystem::path& output_dir);
/// Writes the per-step trajectory/error history of one closed-loop run (see
/// ClosedLoopResult::points) to a CSV file.
void write_closed_loop_csv(const std::filesystem::path& path, const ClosedLoopResult& result);
/// Writes the final per-beacon position/yaw estimates (and their errors
/// against ground truth) of one closed-loop run to a CSV file.
void write_beacon_estimate_csv(const std::filesystem::path& path, const ClosedLoopResult& result);
/// Renders the robot path, target, and beacon ground-truth/estimate
/// positions of one closed-loop run as a standalone SVG scatter/line plot.
void write_svg_plot(const std::filesystem::path& path, const ClosedLoopResult& result);
/// Renders target-error and goal-error over time for one closed-loop run as
/// a standalone SVG line chart.
void write_error_curve_svg(const std::filesystem::path& path, const ClosedLoopResult& result);
/// Renders beacon-position (and, when available, beacon-yaw) RMSE over time
/// for one closed-loop run as a standalone SVG line chart.
void write_beacon_error_svg(const std::filesystem::path& path, const ClosedLoopResult& result);
/// Renders the noise-robustness sweep results as a standalone SVG chart of
/// error vs. noise level.
void write_noise_robustness_svg(const std::filesystem::path& path, const std::vector<NoiseRobustnessRow>& rows);
/// Assembles a single self-contained HTML file at `path` that lets a reader
/// interactively step through/compare several closed-loop runs (a
/// single-beacon local-frame run, scenario-1, and scenario-2) alongside the
/// continuous-time adaptive-localization run, for visual inspection without
/// needing any external plotting tool.
void write_html_viewer(
    const std::filesystem::path& path,
    const ClosedLoopResult& local_single_beacon,
    const ClosedLoopResult& scenario1,
    const ClosedLoopResult& scenario2,
    const AdaptiveLocalizationRun& adaptive_localization);

/// Writes the per-step trajectory/error history of an
/// AdaptiveLocalizationRun (the continuous-time adaptive estimator) to a
/// CSV file.
void write_adaptive_localization_csv(
    const std::filesystem::path& path,
    const AdaptiveLocalizationRun& result);

}  // namespace adaptive
