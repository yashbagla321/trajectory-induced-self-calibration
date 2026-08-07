#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "adaptive_localization/Math.hpp"

namespace adaptive {

struct World {
    Vec2 target;
    std::vector<Vec2> beacons;
    std::vector<double> beacon_yaws;
};

struct Noise {
    double range_sigma = 0.03;
    double bearing_sigma = 0.006;
};

struct LocalFrameMeasurement {
    std::size_t beacon = 0;
    std::size_t time = 0;
    double rv = 0.0;
    double bv_local = 0.0;
    double rt = 0.0;
    double bt_local = 0.0;
};

struct GlobalBearingMeasurement {
    std::size_t beacon = 0;
    std::size_t time = 0;
    double rv = 0.0;
    double rt = 0.0;
    double bt_global = 0.0;
};

struct SolverResult {
    std::vector<double> x;
    double cost = 0.0;
    int iterations = 0;
    bool converged = false;
};

struct BeaconEstimate {
    Vec2 position;
    double yaw = 0.0;
};

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

struct ClosedLoopResult {
    int scenario = 0;
    World world;
    std::vector<ClosedLoopPoint> points;
    std::vector<BeaconEstimate> beacon_estimates;
    Vec2 final_target_estimate;
};

struct ActiveExcitationComparisonRow {
    std::string excitation;
    int beacons = 1;
    double final_goal_error = 0.0;
    double final_target_error = 0.0;
    double final_beacon_position_rmse = 0.0;
    double final_beacon_yaw_rmse = 0.0;
    double final_cost = 0.0;
};

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

struct InitialPoseRobustnessRow {
    int scenario = 0;
    int trial = 0;
    Vec2 initial_robot;
    double final_goal_error = 0.0;
    double final_target_error = 0.0;
    double final_beacon_position_rmse = 0.0;
    double final_beacon_yaw_rmse = -1.0;
};

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

struct AdaptiveLocalizationPoint {
    int step = 0;
    Vec2 robot;
    Vec2 target_estimate;
    std::vector<BeaconEstimate> beacon_estimates;
    double target_error = 0.0;
    double goal_error = 0.0;
    double cost = 0.0;
};

struct AdaptiveLocalizationRun {
    World world;
    std::vector<AdaptiveLocalizationPoint> points;
};

}  // namespace adaptive
