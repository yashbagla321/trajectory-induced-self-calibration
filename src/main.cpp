#include <filesystem>
#include <iomanip>
#include <iostream>
#include <exception>
#include <vector>

#include "adaptive_localization/Config.hpp"
#include "adaptive_localization/Output.hpp"
#include "adaptive_localization/Simulation.hpp"

// This entry point is trimmed to the sweeps and tables cited by the
// paper "Trajectory-Induced Self-Calibration for Hidden-Target Localization
// Through an Unknown-Pose Range-Bearing Relay". The estimator core
// (include/, src/*.cpp other than this file) is shared verbatim with the
// companion excitation-supervised-closed-loop repository, which runs the
// closed-loop supervision study this file does not exercise.

namespace {

std::vector<adaptive::SummaryRow> make_summaries(
    const adaptive::SimulationConfig& config,
    const std::vector<adaptive::TrialResult>& trials) {
    std::vector<adaptive::SummaryRow> summaries;
    for (int scenario : config.monte_carlo_scenarios) {
        for (int beacon_count : config.monte_carlo_beacon_counts) {
            summaries.push_back(adaptive::summarize(scenario, beacon_count, trials));
        }
    }
    return summaries;
}

const char* scenario_name(int scenario) {
    switch (scenario) {
        case 1:
            return "Uncalibrated local-frame cooperative model";
        case 2:
            return "Calibrated global-frame baseline";
        case 3:
            return "Naive EKF local-frame baseline";
        case 4:
            return "Two-view initialized EKF local-frame baseline";
        default:
            return "Unknown scenario";
    }
}

void write_all_outputs(
    const std::filesystem::path& output_dir,
    const std::vector<adaptive::TrialResult>& trials,
    const std::vector<adaptive::SummaryRow>& summaries,
    const std::vector<adaptive::NoiseRobustnessRow>& noise_rows,
    const std::vector<adaptive::GeometrySweepRow>& geometry_rows,
    const std::vector<adaptive::TrajectorySweepRow>& trajectory_rows,
    const std::vector<adaptive::InitialPoseRobustnessRow>& initial_pose_rows,
    const std::vector<adaptive::MinimalBeaconExcitationRow>& minimal_beacon_rows,
    const std::vector<adaptive::PoorInitializationSweepRow>& poor_initialization_rows,
    const std::vector<adaptive::TrajectorySweepRow>& near_degenerate_rows,
    const std::vector<adaptive::IntermittentMeasurementSweepRow>& intermittent_rows,
    const std::vector<adaptive::OutlierRobustnessSweepRow>& outlier_rows,
    const std::vector<adaptive::VehicleLocalizationNoiseSweepRow>& vehicle_noise_rows,
    const std::vector<adaptive::InformationConditioningRow>& information_rows,
    const std::vector<adaptive::ExpandedBaselineSummaryRow>& expanded_baseline_rows) {
    adaptive::write_summary_csv(output_dir / "monte_carlo_summary.csv", summaries);
    adaptive::write_trial_csv(output_dir / "monte_carlo_trials.csv", trials);
    adaptive::write_noise_robustness_csv(output_dir / "noise_robustness.csv", noise_rows);
    adaptive::write_geometry_sweep_csv(output_dir / "geometry_sweep.csv", geometry_rows);
    adaptive::write_trajectory_sweep_csv(output_dir / "trajectory_sweep.csv", trajectory_rows);
    adaptive::write_initial_pose_robustness_csv(output_dir / "initial_pose_robustness.csv", initial_pose_rows);
    adaptive::write_minimal_beacon_excitation_csv(
        output_dir / "minimal_beacon_excitation.csv", minimal_beacon_rows);
    adaptive::write_poor_initialization_sweep_csv(
        output_dir / "poor_initialization_sweep.csv", poor_initialization_rows);
    adaptive::write_trajectory_sweep_csv(
        output_dir / "near_degenerate_trajectory_sweep.csv", near_degenerate_rows);
    adaptive::write_intermittent_measurement_sweep_csv(
        output_dir / "intermittent_measurement_sweep.csv", intermittent_rows);
    adaptive::write_outlier_robustness_sweep_csv(
        output_dir / "outlier_robustness_sweep.csv", outlier_rows);
    adaptive::write_vehicle_localization_noise_sweep_csv(
        output_dir / "vehicle_localization_noise_sweep.csv", vehicle_noise_rows);
    adaptive::write_information_conditioning_csv(
        output_dir / "information_conditioning.csv", information_rows);
    adaptive::write_expanded_baseline_summary_csv(
        output_dir / "expanded_baseline_summary.csv", expanded_baseline_rows);
    adaptive::write_example_csvs(output_dir);
}

void print_run_summary(
    const adaptive::SimulationConfig& config,
    const std::filesystem::path& config_path,
    const std::vector<adaptive::SummaryRow>& summaries) {
    std::cout << "Identifiability simulation complete.\n";
    std::cout << "Config: " << config_path.string() << "\n";
    std::cout << "Output directory: " << config.output_dir.string() << "\n";
    std::cout << "Monte Carlo scenarios: " << adaptive::join_ints(config.monte_carlo_scenarios) << "\n";
    std::cout << "Monte Carlo beacon counts: " << adaptive::join_ints(config.monte_carlo_beacon_counts) << "\n\n";
    std::cout << "Wrote " << (config.output_dir / "monte_carlo_summary.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "monte_carlo_trials.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "noise_robustness.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "geometry_sweep.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "trajectory_sweep.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "initial_pose_robustness.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "minimal_beacon_excitation.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "poor_initialization_sweep.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "near_degenerate_trajectory_sweep.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "intermittent_measurement_sweep.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "outlier_robustness_sweep.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "vehicle_localization_noise_sweep.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "information_conditioning.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "expanded_baseline_summary.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "example_trajectory.csv").string() << "\n\n";

    std::cout << "model, beacons, target_rmse, target_rmse_ci95, beacon_position_rmse, beacon_position_ci95, beacon_yaw_rmse, beacon_yaw_ci95, mean_cost, mean_iterations, mean_runtime_ms, success_rate\n";
    for (const auto& row : summaries) {
        std::cout << scenario_name(row.scenario) << ", "
                  << row.beacons << ", "
                  << std::fixed << std::setprecision(6) << row.rmse << ", "
                  << row.rmse_ci95 << ", "
                  << row.mean_beacon_position_rmse << ", ";
        std::cout << row.mean_beacon_position_rmse_ci95 << ", ";
        if (row.mean_beacon_yaw_rmse >= 0.0) {
            std::cout << row.mean_beacon_yaw_rmse;
        } else {
            std::cout << "n/a";
        }
        std::cout << ", ";
        if (row.mean_beacon_yaw_rmse_ci95 >= 0.0) {
            std::cout << row.mean_beacon_yaw_rmse_ci95;
        } else {
            std::cout << "n/a";
        }
        std::cout << ", "
                  << row.mean_cost << ", "
                  << row.mean_iterations << ", "
                  << row.mean_runtime_ms << ", "
                  << row.convergence_rate << '\n';
    }
}

}  // namespace

int run_main(int argc, char** argv) {
    const std::filesystem::path config_path = argc > 1 ? argv[1] : "config/simulation.ini";
    adaptive::write_default_config_if_missing(config_path);
    const auto config = adaptive::load_config(config_path);
    std::filesystem::create_directories(config.output_dir);

    const auto trials = adaptive::run_monte_carlo(config);
    const auto summaries = make_summaries(config, trials);
    const auto noise_rows = adaptive::run_noise_robustness_sweep(config);
    const auto geometry_rows = adaptive::run_geometry_sweep(config);
    const auto trajectory_rows = adaptive::run_trajectory_sweep(config);
    const auto initial_pose_rows = adaptive::run_initial_pose_robustness_sweep(config);
    const auto minimal_beacon_rows = adaptive::run_minimal_beacon_excitation_study(config);
    const auto poor_initialization_rows = adaptive::run_poor_initialization_sweep(config);
    const auto near_degenerate_rows = adaptive::run_near_degenerate_trajectory_sweep(config);
    const auto intermittent_rows = adaptive::run_intermittent_measurement_sweep(config);
    const auto outlier_rows = adaptive::run_outlier_robustness_sweep(config);
    const auto vehicle_noise_rows = adaptive::run_vehicle_localization_noise_sweep(config);
    const auto information_rows = adaptive::run_information_conditioning_sweep(config);
    const auto expanded_baseline_rows = adaptive::run_expanded_baseline_comparison(config);

    write_all_outputs(
        config.output_dir,
        trials,
        summaries,
        noise_rows,
        geometry_rows,
        trajectory_rows,
        initial_pose_rows,
        minimal_beacon_rows,
        poor_initialization_rows,
        near_degenerate_rows,
        intermittent_rows,
        outlier_rows,
        vehicle_noise_rows,
        information_rows,
        expanded_baseline_rows);
    print_run_summary(config, config_path, summaries);
    return 0;
}

int main(int argc, char** argv) {
    try {
        return run_main(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << "Simulation failed: " << error.what() << '\n';
        return 1;
    }
}
