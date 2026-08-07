#pragma once

#include <filesystem>
#include <vector>

#include "adaptive_localization/Types.hpp"

namespace adaptive {

void write_summary_csv(const std::filesystem::path& path, const std::vector<SummaryRow>& rows);
void write_trial_csv(const std::filesystem::path& path, const std::vector<TrialResult>& trials);
void write_noise_robustness_csv(const std::filesystem::path& path, const std::vector<NoiseRobustnessRow>& rows);
void write_geometry_sweep_csv(const std::filesystem::path& path, const std::vector<GeometrySweepRow>& rows);
void write_trajectory_sweep_csv(const std::filesystem::path& path, const std::vector<TrajectorySweepRow>& rows);
void write_initial_pose_robustness_csv(
    const std::filesystem::path& path,
    const std::vector<InitialPoseRobustnessRow>& rows);
void write_minimal_beacon_excitation_csv(
    const std::filesystem::path& path,
    const std::vector<MinimalBeaconExcitationRow>& rows);
void write_poor_initialization_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<PoorInitializationSweepRow>& rows);
void write_intermittent_measurement_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<IntermittentMeasurementSweepRow>& rows);
void write_outlier_robustness_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<OutlierRobustnessSweepRow>& rows);
void write_vehicle_localization_noise_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<VehicleLocalizationNoiseSweepRow>& rows);
void write_information_conditioning_csv(
    const std::filesystem::path& path,
    const std::vector<InformationConditioningRow>& rows);
void write_expanded_baseline_summary_csv(
    const std::filesystem::path& path,
    const std::vector<ExpandedBaselineSummaryRow>& rows);
void write_active_excitation_comparison_csv(
    const std::filesystem::path& path,
    const std::vector<ActiveExcitationComparisonRow>& rows);
void write_supervised_excitation_comparison_csv(
    const std::filesystem::path& path,
    const std::vector<SupervisedExcitationComparisonRow>& rows);
void write_supervised_lambda_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<SupervisedLambdaSweepRow>& rows);
void write_example_csvs(const std::filesystem::path& output_dir);
void write_closed_loop_csv(const std::filesystem::path& path, const ClosedLoopResult& result);
void write_beacon_estimate_csv(const std::filesystem::path& path, const ClosedLoopResult& result);
void write_svg_plot(const std::filesystem::path& path, const ClosedLoopResult& result);
void write_error_curve_svg(const std::filesystem::path& path, const ClosedLoopResult& result);
void write_beacon_error_svg(const std::filesystem::path& path, const ClosedLoopResult& result);
void write_noise_robustness_svg(const std::filesystem::path& path, const std::vector<NoiseRobustnessRow>& rows);
void write_html_viewer(
    const std::filesystem::path& path,
    const ClosedLoopResult& local_single_beacon,
    const ClosedLoopResult& scenario1,
    const ClosedLoopResult& scenario2,
    const AdaptiveLocalizationRun& adaptive_localization);

void write_adaptive_localization_csv(
    const std::filesystem::path& path,
    const AdaptiveLocalizationRun& result);

}  // namespace adaptive
