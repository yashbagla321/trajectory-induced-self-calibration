#include "adaptive_localization/Output.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <string>

#include "adaptive_localization/World.hpp"

namespace adaptive {

namespace {

struct SvgTransform {
    double min_x = -4.0;
    double max_x = 4.0;
    double min_y = -3.5;
    double max_y = 3.5;
    double width = 1200.0;
    double height = 950.0;
    double margin = 150.0;

    double sx(double x) const { return margin + (x - min_x) / (max_x - min_x) * (width - 2.0 * margin); }
    double sy(double y) const { return height - margin - (y - min_y) / (max_y - min_y) * (height - 2.0 * margin); }
};

void write_optional_metric(std::ostream& out, double value) {
    if (value < 0.0) {
        return;
    }
    out << value;
}

void write_json_optional_metric(std::ostream& out, double value) {
    if (value < 0.0) {
        out << "null";
        return;
    }
    out << value;
}

void write_closed_loop_json(std::ostream& out, const ClosedLoopResult& result) {
    out << "{";
    out << "\"scenario\":" << result.scenario << ",";
    out << "\"target\":{\"x\":" << result.world.target.x << ",\"y\":" << result.world.target.y << "},";
    out << "\"beacons\":[";
    for (std::size_t i = 0; i < result.world.beacons.size(); ++i) {
        if (i > 0) out << ",";
        out << "{\"x\":" << result.world.beacons[i].x
            << ",\"y\":" << result.world.beacons[i].y
            << ",\"yaw\":" << result.world.beacon_yaws[i] << "}";
    }
    out << "],\"points\":[";
    for (std::size_t k = 0; k < result.points.size(); ++k) {
        if (k > 0) out << ",";
        const auto& p = result.points[k];
        out << "{\"step\":" << p.step
            << ",\"robot\":{\"x\":" << p.robot.x << ",\"y\":" << p.robot.y << "}"
            << ",\"targetEstimate\":{\"x\":" << p.target_estimate.x << ",\"y\":" << p.target_estimate.y << "}"
            << ",\"targetError\":" << p.target_error
            << ",\"goalError\":" << p.goal_error
            << ",\"beaconPositionRmse\":" << p.beacon_position_rmse
            << ",\"beaconYawRmse\":";
        write_json_optional_metric(out, p.beacon_yaw_rmse);
        out << ""
            << ",\"cost\":" << p.cost
            << ",\"beaconEstimates\":[";
        for (std::size_t i = 0; i < p.beacon_estimates.size(); ++i) {
            if (i > 0) out << ",";
            out << "{\"x\":" << p.beacon_estimates[i].position.x
                << ",\"y\":" << p.beacon_estimates[i].position.y
                << ",\"yaw\":" << p.beacon_estimates[i].yaw << "}";
        }
        out << "]}";
    }
    out << "]}";
}

void write_adaptive_localization_json(std::ostream& out, const AdaptiveLocalizationRun& result) {
    out << "{";
    out << "\"scenario\":3,";
    out << "\"target\":{\"x\":" << result.world.target.x << ",\"y\":" << result.world.target.y << "},";
    out << "\"beacons\":[";
    for (std::size_t i = 0; i < result.world.beacons.size(); ++i) {
        if (i > 0) out << ",";
        out << "{\"x\":" << result.world.beacons[i].x
            << ",\"y\":" << result.world.beacons[i].y
            << ",\"yaw\":" << result.world.beacon_yaws[i] << "}";
    }
    out << "],\"points\":[";
    for (std::size_t k = 0; k < result.points.size(); ++k) {
        if (k > 0) out << ",";
        const auto& p = result.points[k];
        out << "{\"step\":" << p.step
            << ",\"robot\":{\"x\":" << p.robot.x << ",\"y\":" << p.robot.y << "}"
            << ",\"targetEstimate\":{\"x\":" << p.target_estimate.x << ",\"y\":" << p.target_estimate.y << "}"
            << ",\"targetError\":" << p.target_error
            << ",\"goalError\":" << p.goal_error
            << ",\"cost\":" << p.cost
            << ",\"beaconEstimates\":[";
        for (std::size_t i = 0; i < p.beacon_estimates.size(); ++i) {
            if (i > 0) out << ",";
            out << "{\"x\":" << p.beacon_estimates[i].position.x
                << ",\"y\":" << p.beacon_estimates[i].position.y
                << ",\"yaw\":" << p.beacon_estimates[i].yaw << "}";
        }
        out << "]}";
    }
    out << "]}";
}

}  // namespace

void write_summary_csv(const std::filesystem::path& path, const std::vector<SummaryRow>& rows) {
    std::ofstream out(path);
    out << "scenario,beacons,rmse,rmse_ci95,mean_error,mean_error_ci95,bias_x,bias_y,"
           "success_rate,mean_cost,mean_iterations,mean_runtime_ms,"
           "mean_beacon_position_rmse,mean_beacon_position_rmse_ci95,"
           "mean_beacon_yaw_rmse,mean_beacon_yaw_rmse_ci95\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.scenario << ',' << row.beacons << ',' << row.rmse << ',' << row.rmse_ci95 << ','
            << row.mean_error << ',' << row.mean_error_ci95 << ','
            << row.bias_x << ',' << row.bias_y << ',' << row.convergence_rate << ',' << row.mean_cost << ','
            << row.mean_iterations << ',' << row.mean_runtime_ms << ','
            << row.mean_beacon_position_rmse << ',' << row.mean_beacon_position_rmse_ci95 << ',';
        write_optional_metric(out, row.mean_beacon_yaw_rmse);
        out << ',';
        write_optional_metric(out, row.mean_beacon_yaw_rmse_ci95);
        out << '\n';
    }
}

void write_trial_csv(const std::filesystem::path& path, const std::vector<TrialResult>& trials) {
    std::ofstream out(path);
    out << "scenario,beacons,trial,target_x,target_y,estimate_x,estimate_y,error,"
           "beacon_position_rmse,beacon_yaw_rmse,cost,iterations,runtime_ms,solver_converged,success\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& t : trials) {
        out << t.scenario << ',' << t.beacons << ',' << t.trial << ',' << t.truth.x << ',' << t.truth.y << ','
            << t.estimate.x << ',' << t.estimate.y << ',' << t.error << ','
            << t.beacon_position_rmse << ',';
        write_optional_metric(out, t.beacon_yaw_rmse);
        out << ',' << t.cost << ',' << t.iterations << ',' << t.runtime_ms << ','
            << (t.solver_converged ? 1 : 0) << ',' << (t.converged ? 1 : 0) << '\n';
    }
}

void write_noise_robustness_csv(const std::filesystem::path& path, const std::vector<NoiseRobustnessRow>& rows) {
    std::ofstream out(path);
    out << "scenario,beacons,range_sigma,bearing_sigma,target_rmse,beacon_position_rmse,"
           "beacon_yaw_rmse,mean_cost,mean_iterations,mean_runtime_ms,success_rate\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.scenario << ',' << row.beacons << ','
            << row.range_sigma << ',' << row.bearing_sigma << ','
            << row.target_rmse << ',' << row.beacon_position_rmse << ',';
        write_optional_metric(out, row.beacon_yaw_rmse);
        out << ',' << row.mean_cost << ',' << row.mean_iterations << ','
            << row.mean_runtime_ms << ',' << row.convergence_rate << '\n';
    }
}

void write_geometry_sweep_csv(const std::filesystem::path& path, const std::vector<GeometrySweepRow>& rows) {
    std::ofstream out(path);
    out << "beacons,beacon_separation,target_rmse,beacon_position_rmse,beacon_yaw_rmse,"
           "mean_cost,mean_iterations,mean_runtime_ms,success_rate\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.beacons << ',' << row.beacon_separation << ','
            << row.target_rmse << ',' << row.beacon_position_rmse << ','
            << row.beacon_yaw_rmse << ',' << row.mean_cost << ','
            << row.mean_iterations << ',' << row.mean_runtime_ms << ','
            << row.convergence_rate << '\n';
    }
}

void write_trajectory_sweep_csv(const std::filesystem::path& path, const std::vector<TrajectorySweepRow>& rows) {
    std::ofstream out(path);
    out << "trajectory,beacons,observability_rank,smallest_singular_value,target_rmse,"
           "beacon_position_rmse,beacon_yaw_rmse,mean_cost,mean_iterations,mean_runtime_ms,success_rate\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.trajectory << ',' << row.beacons << ',' << row.observability_rank << ','
            << row.smallest_singular_value << ',' << row.target_rmse << ','
            << row.beacon_position_rmse << ',' << row.beacon_yaw_rmse << ','
            << row.mean_cost << ',' << row.mean_iterations << ',' << row.mean_runtime_ms << ','
            << row.convergence_rate << '\n';
    }
}

void write_initial_pose_robustness_csv(
    const std::filesystem::path& path,
    const std::vector<InitialPoseRobustnessRow>& rows) {
    std::ofstream out(path);
    out << "scenario,trial,initial_robot_x,initial_robot_y,final_goal_error,final_target_error,"
           "final_beacon_position_rmse,final_beacon_yaw_rmse\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.scenario << ',' << row.trial << ','
            << row.initial_robot.x << ',' << row.initial_robot.y << ','
            << row.final_goal_error << ',' << row.final_target_error << ','
            << row.final_beacon_position_rmse << ',';
        write_optional_metric(out, row.final_beacon_yaw_rmse);
        out << '\n';
    }
}

void write_minimal_beacon_excitation_csv(
    const std::filesystem::path& path,
    const std::vector<MinimalBeaconExcitationRow>& rows) {
    std::ofstream out(path);
    out << "case,beacons,poses,observability_rank,smallest_singular_value,target_error,"
           "trajectory_spread,beacon_position_error,beacon_yaw_error,cost,iterations,solver_converged\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.case_name << ',' << row.beacons << ',' << row.poses << ','
            << row.observability_rank << ',' << row.smallest_singular_value << ','
            << row.target_error << ',' << row.trajectory_spread << ',' << row.beacon_position_error << ','
            << row.beacon_yaw_error << ',' << row.cost << ',' << row.iterations << ','
            << (row.converged ? 1 : 0) << '\n';
    }
}

void write_poor_initialization_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<PoorInitializationSweepRow>& rows) {
    std::ofstream out(path);
    out << "case,target_seed_offset,beacon_seed_radius,beacon_yaw_seed,multistarts,"
           "target_rmse,beacon_position_rmse,beacon_yaw_rmse,mean_cost,mean_iterations,"
           "mean_runtime_ms,success_rate\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.case_name << ',' << row.target_seed_offset << ','
            << row.beacon_seed_radius << ',' << row.beacon_yaw_seed << ','
            << row.multistarts << ',' << row.target_rmse << ','
            << row.beacon_position_rmse << ',' << row.beacon_yaw_rmse << ','
            << row.mean_cost << ',' << row.mean_iterations << ','
            << row.mean_runtime_ms << ',' << row.convergence_rate << '\n';
    }
}

void write_intermittent_measurement_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<IntermittentMeasurementSweepRow>& rows) {
    std::ofstream out(path);
    out << "dropout_probability,mean_measurements,target_rmse,beacon_position_rmse,"
           "beacon_yaw_rmse,mean_cost,mean_iterations,mean_runtime_ms,success_rate\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.dropout_probability << ',' << row.mean_measurements << ','
            << row.target_rmse << ',' << row.beacon_position_rmse << ','
            << row.beacon_yaw_rmse << ',' << row.mean_cost << ','
            << row.mean_iterations << ',' << row.mean_runtime_ms << ','
            << row.convergence_rate << '\n';
    }
}

void write_outlier_robustness_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<OutlierRobustnessSweepRow>& rows) {
    std::ofstream out(path);
    out << "estimator,outlier_probability,outlier_range_magnitude,outlier_bearing_magnitude,"
           "target_rmse,beacon_position_rmse,beacon_yaw_rmse,mean_cost,mean_iterations,"
           "mean_runtime_ms,success_rate\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.estimator << ',' << row.outlier_probability << ','
            << row.outlier_range_magnitude << ',' << row.outlier_bearing_magnitude << ','
            << row.target_rmse << ',' << row.beacon_position_rmse << ','
            << row.beacon_yaw_rmse << ',' << row.mean_cost << ','
            << row.mean_iterations << ',' << row.mean_runtime_ms << ','
            << row.convergence_rate << '\n';
    }
}

void write_vehicle_localization_noise_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<VehicleLocalizationNoiseSweepRow>& rows) {
    std::ofstream out(path);
    out << "case,vehicle_position_sigma,target_rmse,beacon_position_rmse,beacon_yaw_rmse,"
           "mean_cost,mean_iterations,mean_runtime_ms,success_rate\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.case_name << ',' << row.vehicle_position_sigma << ',' << row.target_rmse << ','
            << row.beacon_position_rmse << ',' << row.beacon_yaw_rmse << ','
            << row.mean_cost << ',' << row.mean_iterations << ','
            << row.mean_runtime_ms << ',' << row.convergence_rate << '\n';
    }
}

void write_information_conditioning_csv(
    const std::filesystem::path& path,
    const std::vector<InformationConditioningRow>& rows) {
    std::ofstream out(path);
    out << "trajectory,beacons,observations,observability_rank,smallest_singular_value,"
           "trajectory_spread,largest_singular_value,condition_number,logdet_information\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.trajectory << ',' << row.beacons << ',' << row.observations << ','
            << row.observability_rank << ',' << row.smallest_singular_value << ','
            << row.trajectory_spread << ','
            << row.largest_singular_value << ',' << row.condition_number << ','
            << row.logdet_information << '\n';
    }
}

void write_expanded_baseline_summary_csv(
    const std::filesystem::path& path,
    const std::vector<ExpandedBaselineSummaryRow>& rows) {
    std::ofstream out(path);
    out << "case,estimator,beacons,target_rmse,target_rmse_ci95,beacon_position_rmse,"
           "beacon_position_rmse_ci95,beacon_yaw_rmse,beacon_yaw_rmse_ci95,"
           "mean_cost,mean_iterations,mean_runtime_ms,success_rate\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.case_name << ',' << row.estimator << ',' << row.beacons << ','
            << row.target_rmse << ',' << row.target_rmse_ci95 << ','
            << row.beacon_position_rmse << ',' << row.beacon_position_rmse_ci95 << ','
            << row.beacon_yaw_rmse << ',' << row.beacon_yaw_rmse_ci95 << ','
            << row.mean_cost << ','
            << row.mean_iterations << ',' << row.mean_runtime_ms << ','
            << row.convergence_rate << '\n';
    }
}

void write_active_excitation_comparison_csv(
    const std::filesystem::path& path,
    const std::vector<ActiveExcitationComparisonRow>& rows) {
    std::ofstream out(path);
    out << "excitation,beacons,final_goal_error_m,final_target_error_m,"
           "final_beacon_position_rmse_m,final_beacon_yaw_rmse_rad,final_cost\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.excitation << ',' << row.beacons << ','
            << row.final_goal_error << ',' << row.final_target_error << ','
            << row.final_beacon_position_rmse << ',' << row.final_beacon_yaw_rmse << ','
            << row.final_cost << '\n';
    }
}

void write_supervised_excitation_comparison_csv(
    const std::filesystem::path& path,
    const std::vector<SupervisedExcitationComparisonRow>& rows) {
    std::ofstream out(path);
    out << "excitation,retrigger_count,steps_to_goal_threshold,steps_to_target_threshold,"
           "final_goal_error_m,final_target_error_m,final_beacon_position_rmse_m,"
           "final_beacon_yaw_rmse_rad,final_cost\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.excitation << ',' << row.retrigger_count << ','
            << row.steps_to_goal_threshold << ',' << row.steps_to_target_threshold << ','
            << row.final_goal_error << ',' << row.final_target_error << ','
            << row.final_beacon_position_rmse << ',' << row.final_beacon_yaw_rmse << ','
            << row.final_cost << '\n';
    }
}

void write_supervised_lambda_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<SupervisedLambdaSweepRow>& rows) {
    std::ofstream out(path);
    out << "lambda,supervised_retrigger_count,"
           "fixed_final_target_error_m,fixed_final_beacon_position_rmse_m,"
           "fixed_final_beacon_yaw_rmse_rad,"
           "supervised_final_target_error_m,supervised_final_beacon_position_rmse_m,"
           "supervised_final_beacon_yaw_rmse_rad\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        out << row.lambda << ',' << row.supervised_retrigger_count << ','
            << row.fixed_final_target_error << ','
            << row.fixed_final_beacon_position_rmse << ','
            << row.fixed_final_beacon_yaw_rmse << ','
            << row.supervised_final_target_error << ','
            << row.supervised_final_beacon_position_rmse << ','
            << row.supervised_final_beacon_yaw_rmse << '\n';
    }
}

void write_example_csvs(const std::filesystem::path& output_dir) {
    const World world = make_world(2);
    const auto path = make_vehicle_path(80);
    std::ofstream trajectory(output_dir / "example_trajectory.csv");
    trajectory << "time,y_x,y_y,target_x,target_y,beacon_0_x,beacon_0_y,beacon_1_x,beacon_1_y\n";
    trajectory << std::fixed << std::setprecision(8);
    for (std::size_t k = 0; k < path.size(); ++k) {
        trajectory << k << ',' << path[k].x << ',' << path[k].y << ',' << world.target.x << ','
                   << world.target.y << ',' << world.beacons[0].x << ',' << world.beacons[0].y << ','
                   << world.beacons[1].x << ',' << world.beacons[1].y << '\n';
    }
}

void write_closed_loop_csv(const std::filesystem::path& path, const ClosedLoopResult& result) {
    std::ofstream out(path);
    out << "step,robot_x,robot_y,target_estimate_x,target_estimate_y,target_error,goal_error,"
           "beacon_position_rmse,beacon_yaw_rmse,cost,retriggered\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& point : result.points) {
        out << point.step << ',' << point.robot.x << ',' << point.robot.y << ','
            << point.target_estimate.x << ',' << point.target_estimate.y << ','
            << point.target_error << ',' << point.goal_error << ','
            << point.beacon_position_rmse << ',';
        write_optional_metric(out, point.beacon_yaw_rmse);
        out << ',' << point.cost << ',' << (point.retriggered ? 1 : 0) << '\n';
    }
}

void write_beacon_estimate_csv(const std::filesystem::path& path, const ClosedLoopResult& result) {
    std::ofstream out(path);
    out << "scenario,beacon,true_x,true_y,estimate_x,estimate_y,true_yaw,estimate_yaw,"
           "final_vehicle_range,final_vehicle_bearing,final_target_range,final_target_bearing\n";
    out << std::fixed << std::setprecision(8);
    const Vec2 final_robot = result.points.back().robot;
    for (std::size_t i = 0; i < result.world.beacons.size(); ++i) {
        const Vec2 true_beacon = result.world.beacons[i];
        const Vec2 estimated_beacon = result.beacon_estimates[i].position;
        out << result.scenario << ',' << i << ',' << true_beacon.x << ',' << true_beacon.y << ','
            << estimated_beacon.x << ',' << estimated_beacon.y << ',' << result.world.beacon_yaws[i] << ','
            << result.beacon_estimates[i].yaw << ',' << norm(final_robot - true_beacon) << ','
            << bearing(true_beacon, final_robot) << ',' << norm(result.world.target - true_beacon) << ','
            << bearing(true_beacon, result.world.target) << '\n';
    }
}

void write_svg_plot(const std::filesystem::path& path, const ClosedLoopResult& result) {
    SvgTransform t;
    const char* beacon_colors[] = {"#7c3aed", "#0891b2", "#db2777", "#65a30d", "#ea580c", "#4f46e5"};
    const std::string title =
        result.scenario == 1
            ? "Local-frame model, " + std::to_string(result.world.beacons.size()) + " beacon(s)"
            : "Calibrated baseline, " + std::to_string(result.world.beacons.size()) + " beacon(s)";
    std::ofstream out(path);
    out << std::fixed << std::setprecision(3);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << t.width << "\" height=\"" << t.height
        << "\" viewBox=\"0 0 " << t.width << ' ' << t.height << "\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"#fbfbf8\"/>\n";
    out << "<text x=\"70\" y=\"45\" font-family=\"Arial\" font-size=\"38\" font-weight=\"bold\" fill=\"#1f2937\">"
        << title << ": hidden-target seeking</text>\n";
    out << "<line x1=\"" << t.margin << "\" y1=\"" << t.height - t.margin
        << "\" x2=\"" << t.width - t.margin << "\" y2=\"" << t.height - t.margin
        << "\" stroke=\"#9ca3af\"/>\n";
    out << "<line x1=\"" << t.margin << "\" y1=\"" << t.margin << "\" x2=\"" << t.margin
        << "\" y2=\"" << t.height - t.margin << "\" stroke=\"#9ca3af\"/>\n";
    out << "<text x=\"" << t.width / 2.0 - 72.0 << "\" y=\"" << t.height - 28.0
        << "\" font-family=\"Arial\" font-size=\"28\" font-weight=\"bold\" fill=\"#374151\">x position</text>\n";
    out << "<text x=\"38\" y=\"" << t.height / 2.0 + 72.0
        << "\" font-family=\"Arial\" font-size=\"28\" font-weight=\"bold\" fill=\"#374151\" transform=\"rotate(-90 38 "
        << t.height / 2.0 + 72.0 << ")\">y position</text>\n";
    out << "<polyline fill=\"none\" stroke=\"#2563eb\" stroke-width=\"3\" points=\"";
    for (const auto& point : result.points) {
        out << t.sx(point.robot.x) << ',' << t.sy(point.robot.y) << ' ';
    }
    out << "\"/>\n";
    out << "<circle cx=\"" << t.sx(result.points.front().robot.x)
        << "\" cy=\"" << t.sy(result.points.front().robot.y)
        << "\" r=\"6\" fill=\"#111827\"/>\n";
    out << "<text x=\"" << t.sx(result.points.front().robot.x) + 10
        << "\" y=\"" << t.sy(result.points.front().robot.y) - 8
        << "\" font-family=\"Arial\" font-size=\"24\" font-weight=\"bold\" fill=\"#111827\">q(0)</text>\n";
    const Vec2 final_robot = result.points.back().robot;
    out << "<circle cx=\"" << t.sx(final_robot.x) << "\" cy=\"" << t.sy(final_robot.y)
        << "\" r=\"7\" fill=\"#2563eb\" stroke=\"#ffffff\" stroke-width=\"2\"/>\n";
    for (std::size_t k = 0; k < result.points.size(); ++k) {
        const Vec2 phat = result.points[k].target_estimate;
        const double opacity = 0.30 + 0.60 * static_cast<double>(k) /
            static_cast<double>(std::max<std::size_t>(1, result.points.size() - 1));
        out << "<circle cx=\"" << t.sx(phat.x) << "\" cy=\"" << t.sy(phat.y)
            << "\" r=\"3.5\" fill=\"#16a34a\" fill-opacity=\"" << opacity << "\"/>\n";
    }
    out << "<circle cx=\"" << t.sx(result.points.front().target_estimate.x)
        << "\" cy=\"" << t.sy(result.points.front().target_estimate.y)
        << "\" r=\"5\" fill=\"#111827\"/>\n";
    out << "<text x=\"" << t.sx(result.points.front().target_estimate.x) + 10
        << "\" y=\"" << t.sy(result.points.front().target_estimate.y) - 8
        << "\" font-family=\"Arial\" font-size=\"24\" font-weight=\"bold\" fill=\"#111827\">p_hat(0)</text>\n";
    out << "<path d=\"M " << t.sx(result.world.target.x) - 8 << ' ' << t.sy(result.world.target.y)
        << " L " << t.sx(result.world.target.x) + 8 << ' ' << t.sy(result.world.target.y)
        << " M " << t.sx(result.world.target.x) << ' ' << t.sy(result.world.target.y) - 8
        << " L " << t.sx(result.world.target.x) << ' ' << t.sy(result.world.target.y) + 8
        << "\" stroke=\"#dc2626\" stroke-width=\"3\"/>\n";
    out << "<circle cx=\"" << t.sx(result.final_target_estimate.x)
        << "\" cy=\"" << t.sy(result.final_target_estimate.y)
        << "\" r=\"6\" fill=\"none\" stroke=\"#16a34a\" stroke-width=\"3\"/>\n";
    out << "<text x=\"70\" y=\"85\" font-family=\"Arial\" font-size=\"26\" fill=\"#2563eb\">Vehicle trajectory - q(t)</text>\n";
    out << "<text x=\"410\" y=\"85\" font-family=\"Arial\" font-size=\"26\" fill=\"#111827\">Initial conditions - q(0), p_hat(0)</text>\n";
    out << "<text x=\"70\" y=\"120\" font-family=\"Arial\" font-size=\"26\" fill=\"#16a34a\">Target Estimates - p_hat(t)</text>\n";
    out << "<text x=\"410\" y=\"120\" font-family=\"Arial\" font-size=\"26\" fill=\"#7c3aed\">Beacon History - x_hat_i(t)</text>\n";
    for (std::size_t i = 0; i < result.world.beacons.size(); ++i) {
        const Vec2 b = result.world.beacons[i];
        const Vec2 bhat = result.beacon_estimates[i].position;
        out << "<rect x=\"" << t.sx(b.x) - 6 << "\" y=\"" << t.sy(b.y) - 6
            << "\" width=\"12\" height=\"12\" fill=\"#f97316\"/>\n";
        out << "<text x=\"" << t.sx(b.x) + 12 << "\" y=\"" << t.sy(b.y) + 24
            << "\" font-family=\"Arial\" font-size=\"24\" fill=\"#9a3412\">x_" << i << "</text>\n";
        for (std::size_t k = 0; k < result.points.size(); ++k) {
            if (i >= result.points[k].beacon_estimates.size()) {
                continue;
            }
            const Vec2 history = result.points[k].beacon_estimates[i].position;
            const double opacity = 0.25 + 0.60 * static_cast<double>(k) /
                static_cast<double>(std::max<std::size_t>(1, result.points.size() - 1));
            out << "<circle cx=\"" << t.sx(history.x) << "\" cy=\"" << t.sy(history.y)
                << "\" r=\"3\" fill=\"" << beacon_colors[i % 6]
                << "\" fill-opacity=\"" << opacity << "\"/>\n";
        }
        out << "<path d=\"M " << t.sx(bhat.x) - 7 << ' ' << t.sy(bhat.y) - 7
            << " L " << t.sx(bhat.x) + 7 << ' ' << t.sy(bhat.y) + 7
            << " M " << t.sx(bhat.x) + 7 << ' ' << t.sy(bhat.y) - 7
            << " L " << t.sx(bhat.x) - 7 << ' ' << t.sy(bhat.y) + 7
            << "\" stroke=\"" << beacon_colors[i % 6] << "\" stroke-width=\"3\"/>\n";
        out << "<text x=\"" << t.sx(bhat.x) + 12 << "\" y=\"" << t.sy(bhat.y) - 22
            << "\" font-family=\"Arial\" font-size=\"24\" fill=\"" << beacon_colors[i % 6]
            << "\">xhat_" << i << "</text>\n";
    }
    out << "<text x=\"" << t.sx(result.world.target.x) + 12 << "\" y=\"" << t.sy(result.world.target.y) + 28
        << "\" font-family=\"Arial\" font-size=\"24\" fill=\"#dc2626\">p</text>\n";
    out << "<text x=\"" << t.sx(result.final_target_estimate.x) + 10
        << "\" y=\"" << t.sy(result.final_target_estimate.y) - 28
        << "\" font-family=\"Arial\" font-size=\"24\" fill=\"#16a34a\">p_hat(t_f)</text>\n";
    out << "</svg>\n";
}

void write_error_curve_svg(const std::filesystem::path& path, const ClosedLoopResult& result) {
    const double width = 1200.0;
    const double height = 1050.0;
    const double margin = 160.0;
    std::size_t plot_count = result.points.size();
    while (plot_count > 0 && result.points[plot_count - 1].step > 60) {
        --plot_count;
    }
    plot_count = std::max<std::size_t>(2, plot_count);
    double max_error = 0.01;
    for (std::size_t i = 0; i < plot_count; ++i) {
        max_error = std::max(max_error, std::max(result.points[i].goal_error, result.points[i].target_error));
    }
    const auto sx = [&](std::size_t i) {
        return margin + static_cast<double>(i) / static_cast<double>(plot_count - 1) *
            (width - 2.0 * margin);
    };
    const auto sy = [&](double value) {
        return height - margin - value / max_error * (height - 2.0 * margin);
    };

    std::ofstream out(path);
    out << std::fixed << std::setprecision(3);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\"" << height
        << "\" viewBox=\"0 0 " << width << ' ' << height << "\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"#fffefa\"/>\n";
    out << "<text x=\"75\" y=\"45\" font-family=\"Arial\" font-size=\"40\" font-weight=\"bold\" fill=\"#1f2937\">Closed-loop error convergence, first 60 steps</text>\n";
    out << "<text x=\"75\" y=\"85\" font-family=\"Arial\" font-size=\"26\" fill=\"#2563eb\">Vehicle-to-target error - ||q(t)-p||</text>\n";
    out << "<text x=\"75\" y=\"118\" font-family=\"Arial\" font-size=\"26\" fill=\"#16a34a\">Target-estimate error - ||p_hat(t)-p||</text>\n";
    out << "<line x1=\"" << margin << "\" y1=\"" << height - margin << "\" x2=\"" << width - margin
        << "\" y2=\"" << height - margin << "\" stroke=\"#9ca3af\"/>\n";
    out << "<line x1=\"" << margin << "\" y1=\"" << margin << "\" x2=\"" << margin
        << "\" y2=\"" << height - margin << "\" stroke=\"#9ca3af\"/>\n";
    for (int tick = 0; tick <= 4; ++tick) {
        const double value = max_error * static_cast<double>(tick) / 4.0;
        const double y = sy(value);
        out << "<line x1=\"" << margin << "\" y1=\"" << y << "\" x2=\"" << width - margin
            << "\" y2=\"" << y << "\" stroke=\"#e5e1d8\" stroke-width=\"0.8\"/>\n";
        out << "<text x=\"" << margin - 14 << "\" y=\"" << y + 7
            << "\" font-family=\"Arial\" font-size=\"22\" fill=\"#6b7280\" text-anchor=\"end\">"
            << value << "</text>\n";
    }
    for (int tick = 0; tick <= 10; ++tick) {
        const std::size_t step = static_cast<std::size_t>(
            std::round(static_cast<double>(tick) * static_cast<double>(plot_count - 1) / 10.0));
        const double x = sx(step);
        out << "<line x1=\"" << x << "\" y1=\"" << margin << "\" x2=\"" << x
            << "\" y2=\"" << height - margin << "\" stroke=\"#eee9df\" stroke-width=\"0.7\"/>\n";
        out << "<text x=\"" << x << "\" y=\"" << height - margin + 34.0
            << "\" font-family=\"Arial\" font-size=\"20\" fill=\"#6b7280\" text-anchor=\"middle\">"
            << result.points[step].step << "</text>\n";
    }
    out << "<text x=\"" << width / 2.0 - 120.0 << "\" y=\"" << height - 28.0
        << "\" font-family=\"Arial\" font-size=\"28\" font-weight=\"bold\" fill=\"#374151\">measurement step k</text>\n";
    out << "<text x=\"38\" y=\"" << height / 2.0 + 65.0
        << "\" font-family=\"Arial\" font-size=\"28\" font-weight=\"bold\" fill=\"#374151\" transform=\"rotate(-90 38 "
        << height / 2.0 + 65.0 << ")\">error norm</text>\n";
    const auto polyline = [&](const char* key, const char* color) {
        out << "<polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\"3\" points=\"";
        for (std::size_t i = 0; i < plot_count; ++i) {
            const double value = key[0] == 'g' ? result.points[i].goal_error : result.points[i].target_error;
            out << sx(i) << ',' << sy(value) << ' ';
        }
        out << "\"/>\n";
    };
    polyline("goal", "#2563eb");
    polyline("target", "#16a34a");
    out << "</svg>\n";
}

void write_beacon_error_svg(const std::filesystem::path& path, const ClosedLoopResult& result) {
    const double width = 1200.0;
    const double height = 1050.0;
    const double margin = 160.0;
    std::size_t plot_count = result.points.size();
    while (plot_count > 0 && result.points[plot_count - 1].step > 30) {
        --plot_count;
    }
    plot_count = std::max<std::size_t>(2, plot_count);
    double max_error = 0.01;
    for (std::size_t i = 0; i < plot_count; ++i) {
        max_error = std::max(max_error, result.points[i].beacon_position_rmse);
        if (result.points[i].beacon_yaw_rmse >= 0.0) {
            max_error = std::max(max_error, result.points[i].beacon_yaw_rmse);
        }
    }
    const auto sx = [&](std::size_t i) {
        return margin + static_cast<double>(i) / static_cast<double>(plot_count - 1) *
            (width - 2.0 * margin);
    };
    const auto sy = [&](double value) {
        return height - margin - value / max_error * (height - 2.0 * margin);
    };

    std::ofstream out(path);
    out << std::fixed << std::setprecision(3);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\"" << height
        << "\" viewBox=\"0 0 " << width << ' ' << height << "\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"#fffefa\"/>\n";
    out << "<text x=\"75\" y=\"45\" font-family=\"Arial\" font-size=\"40\" font-weight=\"bold\" fill=\"#1f2937\">Beacon estimate errors, first 30 steps</text>\n";
    out << "<text x=\"75\" y=\"85\" font-family=\"Arial\" font-size=\"26\" fill=\"#7c3aed\">Beacon position RMSE - ||x_hat_i(t)-x_i||</text>\n";
    out << "<text x=\"75\" y=\"118\" font-family=\"Arial\" font-size=\"26\" fill=\"#0891b2\">Beacon yaw RMSE - ||psi_hat_i(t)-psi_i||</text>\n";
    out << "<line x1=\"" << margin << "\" y1=\"" << height - margin << "\" x2=\"" << width - margin
        << "\" y2=\"" << height - margin << "\" stroke=\"#9ca3af\"/>\n";
    out << "<line x1=\"" << margin << "\" y1=\"" << margin << "\" x2=\"" << margin
        << "\" y2=\"" << height - margin << "\" stroke=\"#9ca3af\"/>\n";
    for (int tick = 0; tick <= 4; ++tick) {
        const double value = max_error * static_cast<double>(tick) / 4.0;
        const double y = sy(value);
        out << "<line x1=\"" << margin << "\" y1=\"" << y << "\" x2=\"" << width - margin
            << "\" y2=\"" << y << "\" stroke=\"#e5e1d8\" stroke-width=\"0.8\"/>\n";
        out << "<text x=\"" << margin - 14 << "\" y=\"" << y + 7
            << "\" font-family=\"Arial\" font-size=\"22\" fill=\"#6b7280\" text-anchor=\"end\">"
            << value << "</text>\n";
    }
    for (int tick = 0; tick <= 10; ++tick) {
        const std::size_t step = static_cast<std::size_t>(
            std::round(static_cast<double>(tick) * static_cast<double>(plot_count - 1) / 10.0));
        const double x = sx(step);
        out << "<line x1=\"" << x << "\" y1=\"" << margin << "\" x2=\"" << x
            << "\" y2=\"" << height - margin << "\" stroke=\"#eee9df\" stroke-width=\"0.7\"/>\n";
        out << "<text x=\"" << x << "\" y=\"" << height - margin + 34.0
            << "\" font-family=\"Arial\" font-size=\"20\" fill=\"#6b7280\" text-anchor=\"middle\">"
            << result.points[step].step << "</text>\n";
    }
    out << "<text x=\"" << width / 2.0 - 120.0 << "\" y=\"" << height - 28.0
        << "\" font-family=\"Arial\" font-size=\"28\" font-weight=\"bold\" fill=\"#374151\">measurement step k</text>\n";
    out << "<text x=\"38\" y=\"" << height / 2.0 + 65.0
        << "\" font-family=\"Arial\" font-size=\"28\" font-weight=\"bold\" fill=\"#374151\" transform=\"rotate(-90 38 "
        << height / 2.0 + 65.0 << ")\">error metric</text>\n";
    out << "<polyline fill=\"none\" stroke=\"#7c3aed\" stroke-width=\"3\" points=\"";
    for (std::size_t i = 0; i < plot_count; ++i) {
        out << sx(i) << ',' << sy(result.points[i].beacon_position_rmse) << ' ';
    }
    out << "\"/>\n";
    out << "<polyline fill=\"none\" stroke=\"#0891b2\" stroke-width=\"3\" points=\"";
    for (std::size_t i = 0; i < plot_count; ++i) {
        const double yaw = result.points[i].beacon_yaw_rmse >= 0.0 ? result.points[i].beacon_yaw_rmse : 0.0;
        out << sx(i) << ',' << sy(yaw) << ' ';
    }
    out << "\"/>\n";
    out << "</svg>\n";
}

void write_noise_robustness_svg(const std::filesystem::path& path, const std::vector<NoiseRobustnessRow>& rows) {
    const double width = 1200.0;
    const double height = 650.0;
    const double margin = 110.0;
    const double bearing_slice = 0.006;
    double max_sigma = 0.001;
    double max_rmse = 0.001;
    for (const auto& row : rows) {
        if (std::abs(row.bearing_sigma - bearing_slice) < 1e-9 &&
            (row.scenario == 1 || (row.scenario == 2 && row.beacons == 2))) {
            max_sigma = std::max(max_sigma, row.range_sigma);
            max_rmse = std::max(max_rmse, row.target_rmse);
        }
    }
    const auto sx = [&](double sigma) {
        return margin + sigma / max_sigma * (width - 2.0 * margin);
    };
    const auto sy = [&](double value) {
        return height - margin - value / max_rmse * (height - 2.0 * margin);
    };

    std::ofstream out(path);
    out << std::fixed << std::setprecision(3);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\"" << height
        << "\" viewBox=\"0 0 " << width << ' ' << height << "\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"#fffefa\"/>\n";
    out << "<text x=\"70\" y=\"43\" font-family=\"Arial\" font-size=\"28\" font-weight=\"bold\" fill=\"#1f2937\">Noise robustness: target RMSE at bearing sigma = 0.006</text>\n";
    out << "<line x1=\"" << margin << "\" y1=\"" << height - margin << "\" x2=\"" << width - margin
        << "\" y2=\"" << height - margin << "\" stroke=\"#9ca3af\"/>\n";
    out << "<line x1=\"" << margin << "\" y1=\"" << margin << "\" x2=\"" << margin
        << "\" y2=\"" << height - margin << "\" stroke=\"#9ca3af\"/>\n";
    for (int tick = 0; tick <= 5; ++tick) {
        const double sigma = max_sigma * static_cast<double>(tick) / 5.0;
        const double x = sx(sigma);
        out << "<line x1=\"" << x << "\" y1=\"" << margin << "\" x2=\"" << x
            << "\" y2=\"" << height - margin << "\" stroke=\"#eee9df\" stroke-width=\"0.7\"/>\n";
        out << "<text x=\"" << x << "\" y=\"" << height - margin + 34.0
            << "\" font-family=\"Arial\" font-size=\"20\" fill=\"#6b7280\" text-anchor=\"middle\">"
            << sigma << "</text>\n";
    }
    for (int tick = 0; tick <= 5; ++tick) {
        const double rmse = max_rmse * static_cast<double>(tick) / 5.0;
        const double y = sy(rmse);
        out << "<line x1=\"" << margin << "\" y1=\"" << y << "\" x2=\"" << width - margin
            << "\" y2=\"" << y << "\" stroke=\"#e5e1d8\" stroke-width=\"0.8\"/>\n";
        out << "<text x=\"" << margin - 14 << "\" y=\"" << y + 7
            << "\" font-family=\"Arial\" font-size=\"20\" fill=\"#6b7280\" text-anchor=\"end\">"
            << rmse << "</text>\n";
    }
    out << "<text x=\"" << width / 2.0 - 115.0 << "\" y=\"" << height - 28.0
        << "\" font-family=\"Arial\" font-size=\"28\" font-weight=\"bold\" fill=\"#374151\">range noise sigma</text>\n";
    out << "<text x=\"38\" y=\"" << height / 2.0 + 65.0
        << "\" font-family=\"Arial\" font-size=\"28\" font-weight=\"bold\" fill=\"#374151\" transform=\"rotate(-90 38 "
        << height / 2.0 + 65.0 << ")\">target RMSE</text>\n";
    const auto curve = [&](int scenario, int beacons, const char* color) {
        out << "<polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\"3\" points=\"";
        for (const auto& row : rows) {
            if (row.scenario == scenario && row.beacons == beacons &&
                std::abs(row.bearing_sigma - bearing_slice) < 1e-9) {
                out << sx(row.range_sigma) << ',' << sy(row.target_rmse) << ' ';
            }
        }
        out << "\"/>\n";
        for (const auto& row : rows) {
            if (row.scenario == scenario && row.beacons == beacons &&
                std::abs(row.bearing_sigma - bearing_slice) < 1e-9) {
                out << "<circle cx=\"" << sx(row.range_sigma) << "\" cy=\"" << sy(row.target_rmse)
                    << "\" r=\"4\" fill=\"" << color << "\"/>\n";
            }
        }
    };
    curve(1, 1, "#7c3aed");
    curve(1, 2, "#0891b2");
    curve(2, 2, "#16a34a");
    out << "<text x=\"70\" y=\"85\" font-family=\"Arial\" font-size=\"24\" fill=\"#7c3aed\">local, 1 beacon</text>\n";
    out << "<text x=\"290\" y=\"85\" font-family=\"Arial\" font-size=\"24\" fill=\"#0891b2\">local, 2 beacons</text>\n";
    out << "<text x=\"520\" y=\"85\" font-family=\"Arial\" font-size=\"24\" fill=\"#16a34a\">calibrated, 2 beacons</text>\n";
    out << "</svg>\n";
}

void write_html_viewer(
    const std::filesystem::path& path,
    const ClosedLoopResult& local_single_beacon,
    const ClosedLoopResult& scenario1,
    const ClosedLoopResult& scenario2,
    const AdaptiveLocalizationRun& adaptive_localization) {
    std::ofstream out(path);
    out << std::setprecision(10);
    out << R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Adaptive Localization Simulation Viewer</title>
<style>
body{margin:0;font-family:Arial,Helvetica,sans-serif;background:#f6f5f0;color:#202124}
header{padding:18px 24px 10px;border-bottom:1px solid #d8d4ca;background:#fbfaf7}
h1{margin:0 0 6px;font-size:22px}.subtitle{color:#5f6368;font-size:13px}
.layout{display:grid;grid-template-columns:minmax(900px,1fr) 340px;gap:16px;padding:16px}
.panel{background:#fffefa;border:1px solid #d8d4ca;border-radius:8px;overflow:hidden}
.toolbar{display:flex;align-items:center;gap:10px;padding:12px;border-bottom:1px solid #e5e1d8;flex-wrap:wrap}
button,select{border:1px solid #b9b3a7;background:#fff;border-radius:6px;min-height:34px;padding:0 10px;font-size:14px}
input[type=range]{flex:1 1 240px}canvas{display:block;width:100%;background:#fbfaf7}
.plot-grid{display:grid;grid-template-columns:1fr 1fr;gap:14px;padding:14px;background:#f1efe7}
.plot-window{border:2px solid #2f343b;background:#fffefa;border-radius:6px;overflow:hidden;box-shadow:0 1px 0 rgba(0,0,0,.05)}
.plot-window canvas{aspect-ratio:1/1;height:auto}
#scene,#errors,#targetScatter,#beaconScatter{width:100%}
.metrics{padding:14px;display:grid;gap:10px}.metric{border-bottom:1px solid #ebe7dd;padding-bottom:10px}
.metric label{display:block;color:#5f6368;font-size:12px;margin-bottom:3px}.metric strong{font-size:18px}
table{width:100%;border-collapse:collapse;font-size:12px}th,td{text-align:right;padding:5px 4px;border-bottom:1px solid #ebe7dd}
th:first-child,td:first-child{text-align:left}.legend{display:grid;gap:7px;padding:0 14px 14px;font-size:12px;color:#4b5563}
.swatch{display:inline-block;width:18px;height:3px;margin-right:7px;vertical-align:middle}
@media(max-width:1200px){.layout{grid-template-columns:1fr}.plot-grid{grid-template-columns:1fr}}
</style>
</head>
<body>
<header><h1>Adaptive Localization Simulation Viewer</h1>
<div class="subtitle">Animated closed-loop trajectory q(t), target p, target estimate p_hat(t), beacon estimates x_hat_i(t), and range-bearing geometry.</div></header>
<main class="layout">
<section class="panel"><div class="toolbar">
<select id="scenario"><option value="0">Local-frame unknown-pose model, 1 beacon</option>
<option value="1">Local-frame unknown-pose model, 2 beacons</option>
<option value="2">Calibrated global-frame baseline, 2 beacons</option>
<option value="3">Adaptive localization core: adaptive law + target-seeking controller</option></select>
<button id="play">Play</button><button id="prev">Prev</button><button id="next">Next</button>
<input id="step" type="range" min="0" value="0"><span id="stepLabel"></span></div>
<div class="plot-grid">
<div class="plot-window"><canvas id="scene"></canvas></div>
<div class="plot-window"><canvas id="errors"></canvas></div>
<div class="plot-window"><canvas id="targetScatter"></canvas></div>
<div class="plot-window"><canvas id="beaconScatter"></canvas></div>
</div></section>
<aside class="panel"><div class="metrics">
<div class="metric"><label>Vehicle q(t)</label><strong id="robotMetric"></strong></div>
<div class="metric"><label>Target estimate p_hat(t)</label><strong id="targetMetric"></strong></div>
<div class="metric"><label>||q(t)-p||</label><strong id="goalErrorMetric"></strong></div>
<div class="metric"><label>||p_hat(t)-p||</label><strong id="targetErrorMetric"></strong></div>
<div class="metric"><label>Beacon position RMSE ||x_hat_i-x_i||</label><strong id="beaconPositionMetric"></strong></div>
<div class="metric"><label>Beacon yaw RMSE ||psi_hat_i-psi_i||</label><strong id="beaconYawMetric"></strong></div>
<div class="metric"><label>Least-squares cost J</label><strong id="costMetric"></strong></div>
<div><label style="color:#5f6368;font-size:12px;">Beacon estimates and current measurement geometry</label>
<table id="beaconTable"><thead><tr><th>i</th><th>xhat_i,x</th><th>xhat_i,y</th><th>r_i^v</th><th>beta_i^v</th><th>r_i^t</th><th>beta_i^t</th></tr></thead><tbody></tbody></table></div>
</div><div class="legend">
<div><span class="swatch" style="background:#2563eb"></span>Vehicle trajectory - q(t)</div>
<div><span class="swatch" style="background:#2563eb"></span>Current vehicle position - q</div>
<div><span class="swatch" style="background:#dc2626"></span>Target - p</div>
<div><span class="swatch" style="background:#16a34a"></span>Target Estimates - p_hat(t)</div>
<div><span class="swatch" style="background:#6b7280"></span>Range-bearing Links</div>
<div><span class="swatch" style="background:#f97316"></span>True Beacon Positions - x_i</div>
<div><span class="swatch" style="background:#7c3aed"></span><span class="swatch" style="background:#0891b2"></span><span class="swatch" style="background:#db2777"></span>Beacon History - x_hat_i(t)</div>
</div></aside></main><script>
const DATA=[)HTML";
    write_closed_loop_json(out, local_single_beacon);
    out << ",";
    write_closed_loop_json(out, scenario1);
    out << ",";
    write_closed_loop_json(out, scenario2);
    out << ",";
    write_adaptive_localization_json(out, adaptive_localization);
    out << R"HTML(];
const scene=document.getElementById('scene'),errors=document.getElementById('errors'),targetScatter=document.getElementById('targetScatter'),beaconScatter=document.getElementById('beaconScatter'),scenarioSelect=document.getElementById('scenario');
const stepInput=document.getElementById('step'),playButton=document.getElementById('play');let scenarioIndex=0,stepIndex=0,timer=null;
const fmt=(v,d=3)=>Number(v).toFixed(d);
const fmtMaybe=(v,d=5)=>v==null?'n/a':fmt(v,d);
const beaconColors=['#7c3aed','#0891b2','#db2777','#65a30d','#ea580c','#4f46e5'];
function resize(c){const r=c.getBoundingClientRect(),q=window.devicePixelRatio||1;c.width=Math.round(r.width*q);c.height=Math.round(r.height*q);const x=c.getContext('2d');x.setTransform(q,0,0,q,0,0);return{x,w:r.width,h:r.height};}
function bounds(d){const xs=[d.target.x],ys=[d.target.y];d.beacons.forEach(b=>{xs.push(b.x);ys.push(b.y)});d.points.forEach(p=>{xs.push(p.robot.x,p.targetEstimate.x);ys.push(p.robot.y,p.targetEstimate.y);p.beaconEstimates.forEach(b=>{xs.push(b.x);ys.push(b.y)})});return{minX:Math.min(...xs)-.7,maxX:Math.max(...xs)+.7,minY:Math.min(...ys)-.7,maxY:Math.max(...ys)+.7};}
function targetEstimateBounds(d){const xs=[d.target.x],ys=[d.target.y];d.points.forEach(p=>{xs.push(p.targetEstimate.x);ys.push(p.targetEstimate.y)});const pad=.45;return{minX:Math.min(...xs)-pad,maxX:Math.max(...xs)+pad,minY:Math.min(...ys)-pad,maxY:Math.max(...ys)+pad};}
function beaconBounds(d){const xs=[],ys=[];d.beacons.forEach(b=>{xs.push(b.x);ys.push(b.y)});d.points.forEach(p=>p.beaconEstimates.forEach(b=>{xs.push(b.x);ys.push(b.y)}));const pad=.45;return{minX:Math.min(...xs)-pad,maxX:Math.max(...xs)+pad,minY:Math.min(...ys)-pad,maxY:Math.max(...ys)+pad};}
function transform(w,h,b){const ml=62,mr=24,mt=36,mb=50,span=Math.max(b.maxX-b.minX,b.maxY-b.minY),maxX=b.minX+span,maxY=b.minY+span,s=Math.min((w-ml-mr)/span,(h-mt-mb)/span);return{x:v=>ml+(v-b.minX)*s,y:v=>h-mb-(v-b.minY)*s,ml,mr,mt,mb,maxX,maxY};}
function circle(c,t,p,r,f,s=null,l=1){c.beginPath();c.arc(t.x(p.x),t.y(p.y),r,0,Math.PI*2);c.fillStyle=f;c.fill();if(s){c.strokeStyle=s;c.lineWidth=l;c.stroke();}}
function cross(c,t,p,col,z=8){const x=t.x(p.x),y=t.y(p.y);c.strokeStyle=col;c.lineWidth=3;c.beginPath();c.moveTo(x-z,y-z);c.lineTo(x+z,y+z);c.moveTo(x+z,y-z);c.lineTo(x-z,y+z);c.stroke();}
function legendLine(c,x,y,col,text){c.strokeStyle=col;c.lineWidth=3;c.beginPath();c.moveTo(x,y);c.lineTo(x+24,y);c.stroke();c.fillStyle='#374151';c.font='12px Arial';c.fillText(text,x+32,y+4);}
function legendDot(c,x,y,col,text){c.fillStyle=col;c.beginPath();c.arc(x+12,y,5,0,Math.PI*2);c.fill();c.fillStyle='#374151';c.font='12px Arial';c.fillText(text,x+32,y+4);}
function legendSquare(c,x,y,col,text){c.fillStyle=col;c.fillRect(x+7,y-5,10,10);c.fillStyle='#374151';c.font='12px Arial';c.fillText(text,x+32,y+4);}
function legendBeaconPalette(c,x,y,text){beaconColors.slice(0,4).forEach((col,i)=>{c.fillStyle=col;c.beginPath();c.arc(x+6+i*8,y,4,0,Math.PI*2);c.fill();c.strokeStyle='#111827';c.lineWidth=.5;c.stroke()});c.fillStyle='#374151';c.font='12px Arial';c.fillText(text,x+42,y+4);}
function labelWorldAxes(c,t,r,b,title,xLabel='x position',yLabel='y position'){c.fillStyle='#1f2937';c.font='bold 16px Arial';c.fillText(title,18,24);c.strokeStyle='#9ca3af';c.lineWidth=1.2;c.beginPath();c.moveTo(t.ml,t.mt);c.lineTo(t.ml,r.h-t.mb);c.lineTo(r.w-t.mr,r.h-t.mb);c.stroke();c.fillStyle='#374151';c.font='bold 14px Arial';c.fillText(xLabel,r.w/2-34,r.h-12);c.save();c.translate(18,r.h/2+38);c.rotate(-Math.PI/2);c.fillText(yLabel,0,0);c.restore();c.font='11px Arial';c.textAlign='center';for(let x=Math.ceil(b.minX);x<=Math.floor(t.maxX);x++){const px=t.x(x);if(px>t.ml-1&&px<r.w-t.mr+1){c.strokeStyle='#e5e1d8';c.lineWidth=1;c.beginPath();c.moveTo(px,t.mt);c.lineTo(px,r.h-t.mb);c.stroke();c.fillStyle='#6b7280';c.fillText(String(x),px,r.h-t.mb+16)}}c.textAlign='right';for(let y=Math.ceil(b.minY);y<=Math.floor(t.maxY);y++){const py=t.y(y);if(py>t.mt-1&&py<r.h-t.mb+1){c.strokeStyle='#e5e1d8';c.lineWidth=1;c.beginPath();c.moveTo(t.ml,py);c.lineTo(r.w-t.mr,py);c.stroke();c.fillStyle='#6b7280';c.fillText(String(y),t.ml-8,py+4)}}c.textAlign='left';}
function drawPointLabel(c,text,x,y,occupied,r){c.font='12px Arial';const w=c.measureText(text).width+10,h=18,choices=[[12,-16],[12,16],[-w-12,-16],[-w-12,16],[12,-34],[-w-12,34]];let rect=null;for(const o of choices){const q={x:Math.min(Math.max(x+o[0],42),r.w-w-8),y:Math.min(Math.max(y+o[1],28),r.h-h-24),w,h};if(!occupied.some(a=>q.x<a.x+a.w+4&&q.x+q.w+4>a.x&&q.y<a.y+a.h+4&&q.y+q.h+4>a.y)){rect=q;break}}if(!rect)rect={x:Math.min(Math.max(x+12,42),r.w-w-8),y:Math.min(Math.max(y+16,28),r.h-h-24),w,h};occupied.push(rect);c.fillStyle='rgba(255,255,255,.88)';c.strokeStyle='rgba(209,213,219,.9)';c.lineWidth=1;c.fillRect(rect.x,rect.y,rect.w,rect.h);c.strokeRect(rect.x,rect.y,rect.w,rect.h);c.fillStyle='#374151';c.fillText(text,rect.x+5,rect.y+13);}
function drawSceneLegend(c,r){const x=Math.max(18,r.w-292),y=22;c.fillStyle='rgba(255,255,255,.86)';c.strokeStyle='#e5e7eb';c.lineWidth=1;c.fillRect(x-10,y-14,282,124);c.strokeRect(x-10,y-14,282,124);legendLine(c,x,y,'#2563eb','Vehicle trajectory - q(t)');legendDot(c,x,y+18,'#2563eb','Current vehicle position - q');legendDot(c,x,y+36,'#16a34a','Target Estimates - p_hat(t)');legendLine(c,x,y+54,'#6b7280','Range-bearing Links');legendSquare(c,x,y+72,'#f97316','True Beacon Positions - x_i');legendBeaconPalette(c,x,y+90,'Beacon History - x_hat_i(t)');}
function drawTargetLegend(c,r){const x=Math.max(18,r.w-260),y=22;c.fillStyle='rgba(255,255,255,.88)';c.strokeStyle='#e5e7eb';c.lineWidth=1;c.fillRect(x-10,y-14,250,86);c.strokeRect(x-10,y-14,250,86);legendDot(c,x,y,'#16a34a','Target Estimates - p_hat(t)');legendLine(c,x,y+18,'#16a34a','p_hat(t) evolution');c.strokeStyle='#dc2626';c.lineWidth=3;c.beginPath();c.moveTo(x+5,y+31);c.lineTo(x+19,y+45);c.moveTo(x+19,y+31);c.lineTo(x+5,y+45);c.stroke();c.fillStyle='#374151';c.font='12px Arial';c.fillText('Target - p',x+32,y+40);legendDot(c,x,y+58,'#111827','Initial Estimate - p_hat(0)');}
function drawBeaconLegend(c,r){const x=Math.max(18,r.w-272),y=22;c.fillStyle='rgba(255,255,255,.88)';c.strokeStyle='#e5e7eb';c.lineWidth=1;c.fillRect(x-10,y-14,262,68);c.strokeRect(x-10,y-14,262,68);legendSquare(c,x,y,'#f97316','True Beacon Positions - x_i');legendBeaconPalette(c,x,y+18,'Beacon History - x_hat_i(t)');beaconColors.slice(0,4).forEach((col,i)=>{c.strokeStyle=col;c.lineWidth=2.2;c.beginPath();const px=x+5+i*8;c.moveTo(px,y+29);c.lineTo(px+7,y+43);c.moveTo(px+7,y+29);c.lineTo(px,y+43);c.stroke()});c.fillStyle='#374151';c.font='12px Arial';c.fillText('Current Beacon Estimates - x_hat_i(t)',x+42,y+40);}
function angleError(a,b){return Math.abs(Math.atan2(Math.sin(a-b),Math.cos(a-b)));}
function beaconPositionError(d,p,bi){const e=p.beaconEstimates[bi],b=d.beacons[bi];return e&&b?Math.hypot(e.x-b.x,e.y-b.y):null;}
function beaconYawError(d,p,bi){const e=p.beaconEstimates[bi],b=d.beacons[bi];return e&&b&&e.yaw!=null&&b.yaw!=null?angleError(e.yaw,b.yaw):null;}
function panelSeriesMax(series){let m=.001;series.forEach(s=>s.values.forEach(v=>{if(v!=null&&Number.isFinite(v))m=Math.max(m,v)}));return m*1.08;}
function drawMiniPanel(c,panel,title,yLabel,series,d,plotSteps){const max=panelSeriesMax(series),steps=plotSteps,mx=42,my=34,mb=34,mr=14,x0=panel.x+mx,y0=panel.y+my,w=panel.w-mx-mr,h=panel.h-my-mb,sx=i=>x0+(steps<=1?0:i/(steps-1)*w),sy=v=>y0+h-v/max*h;c.fillStyle='#fffefa';c.fillRect(panel.x,panel.y,panel.w,panel.h);c.strokeStyle='#d1d5db';c.lineWidth=1;c.strokeRect(panel.x+.5,panel.y+.5,panel.w-1,panel.h-1);c.fillStyle='#111827';c.font='bold 13px Arial';c.fillText(title,panel.x+10,panel.y+18);c.strokeStyle='#9ca3af';c.beginPath();c.moveTo(x0,y0);c.lineTo(x0,y0+h);c.lineTo(x0+w,y0+h);c.stroke();c.font='10px Arial';c.fillStyle='#6b7280';c.textAlign='right';for(let i=0;i<=3;i++){const v=max*(1-i/3),y=y0+i/3*h;c.strokeStyle='#eee9df';c.beginPath();c.moveTo(x0,y);c.lineTo(x0+w,y);c.stroke();c.fillStyle='#6b7280';c.fillText(fmt(v,3),x0-6,y+3)}c.textAlign='center';for(let i=0;i<=4;i++){const idx=Math.round(i*(steps-1)/4),x=sx(idx);c.strokeStyle='#f1eee6';c.beginPath();c.moveTo(x,y0);c.lineTo(x,y0+h);c.stroke();c.fillStyle='#6b7280';c.fillText(String(idx),x,y0+h+14)}c.save();c.translate(panel.x+12,y0+h/2+22);c.rotate(-Math.PI/2);c.fillStyle='#4b5563';c.font='bold 10px Arial';c.textAlign='center';c.fillText(yLabel,0,0);c.restore();c.textAlign='left';series.forEach(s=>{c.strokeStyle=s.color;c.lineWidth=2;c.beginPath();let started=false;s.values.slice(0,plotSteps).forEach((v,i)=>{if(v==null||!Number.isFinite(v))return;started?c.lineTo(sx(i),sy(v)):c.moveTo(sx(i),sy(v));started=true});c.stroke()});const current=Math.min(stepIndex,plotSteps-1),cx=sx(current);c.strokeStyle='#111827';c.setLineDash([4,4]);c.beginPath();c.moveTo(cx,y0);c.lineTo(cx,y0+h);c.stroke();c.setLineDash([]);let lx=panel.x+10,ly=panel.y+panel.h-10;series.forEach(s=>{c.strokeStyle=s.color;c.lineWidth=2;c.beginPath();c.moveTo(lx,ly-4);c.lineTo(lx+18,ly-4);c.stroke();c.fillStyle='#374151';c.font='10px Arial';c.fillText(s.label,lx+23,ly);lx+=Math.min(120,c.measureText(s.label).width+48);if(lx>panel.x+panel.w-100){lx=panel.x+10;ly-=14}});}
function drawBeaconEstimateHistory(c,t,d){for(let bi=0;bi<d.beacons.length;bi++){const col=beaconColors[bi%beaconColors.length];for(let k=0;k<=stepIndex;k++){const b=d.points[k].beaconEstimates[bi];if(!b)continue;c.globalAlpha=.48+.45*(k/Math.max(1,stepIndex));c.fillStyle=col;c.beginPath();c.arc(t.x(b.x),t.y(b.y),4.2,0,Math.PI*2);c.fill();c.globalAlpha=.65;c.strokeStyle='#111827';c.lineWidth=.7;c.stroke()}}c.globalAlpha=1;}
function drawTargetEstimateHistory(c,t,d,connect=false){if(connect){c.strokeStyle='rgba(22,163,74,.75)';c.lineWidth=2;c.beginPath();for(let k=0;k<=stepIndex;k++){const q=d.points[k].targetEstimate;k?c.lineTo(t.x(q.x),t.y(q.y)):c.moveTo(t.x(q.x),t.y(q.y))}c.stroke()}for(let k=0;k<=stepIndex;k++){const q=d.points[k].targetEstimate;c.globalAlpha=.35+.55*(k/Math.max(1,stepIndex));c.fillStyle='#16a34a';c.beginPath();c.arc(t.x(q.x),t.y(q.y),3.5,0,Math.PI*2);c.fill()}c.globalAlpha=1;}
function drawScene(){const d=DATA[scenarioIndex],p=d.points[stepIndex],r=resize(scene),c=r.x,b=bounds(d),t=transform(r.w,r.h,b);c.clearRect(0,0,r.w,r.h);c.fillStyle='#fbfaf7';c.fillRect(0,0,r.w,r.h);labelWorldAxes(c,t,r,b,'Workspace: q(t), p, p_hat(t), x_i, x_hat_i(t)');c.strokeStyle='#2563eb';c.lineWidth=3;c.beginPath();for(let i=0;i<=stepIndex;i++){const q=d.points[i].robot;i?c.lineTo(t.x(q.x),t.y(q.y)):c.moveTo(t.x(q.x),t.y(q.y))}c.stroke();drawTargetEstimateHistory(c,t,d);drawBeaconEstimateHistory(c,t,d);circle(c,t,d.points[0].robot,5,'#111827');circle(c,t,p.robot,8,'#2563eb','#fff',2);cross(c,t,d.target,'#dc2626',9);circle(c,t,p.targetEstimate,8,'rgba(22,163,74,.13)','#16a34a',3);const labels=[];drawPointLabel(c,'q(t)',t.x(p.robot.x),t.y(p.robot.y),labels,r);drawPointLabel(c,'p',t.x(d.target.x),t.y(d.target.y),labels,r);drawPointLabel(c,'p_hat(t)',t.x(p.targetEstimate.x),t.y(p.targetEstimate.y),labels,r);d.beacons.forEach((b,i)=>{c.fillStyle='#f97316';c.fillRect(t.x(b.x)-6,t.y(b.y)-6,12,12);c.fillStyle='#9a3412';c.font='12px Arial';c.fillText('x_'+i,t.x(b.x)+9,t.y(b.y)+4)});p.beaconEstimates.forEach((b,i)=>{const col=beaconColors[i%beaconColors.length];c.setLineDash([6,5]);c.strokeStyle='#6b7280';c.lineWidth=1.6;c.beginPath();c.moveTo(t.x(p.robot.x),t.y(p.robot.y));c.lineTo(t.x(b.x),t.y(b.y));c.lineTo(t.x(p.targetEstimate.x),t.y(p.targetEstimate.y));c.stroke();c.setLineDash([]);cross(c,t,b,col,8);c.fillStyle=col;c.font='12px Arial';c.fillText('xhat_'+i,t.x(b.x)+9,t.y(b.y)-7)});drawSceneLegend(c,r);c.fillStyle='#111827';c.font='13px Arial';c.fillText('step '+p.step+' / '+(d.points.length-1),18,40);}
function drawErrors(){const d=DATA[scenarioIndex],r=resize(errors),c=r.x,plotSteps=Math.min(60,d.points.length);c.clearRect(0,0,r.w,r.h);c.fillStyle='#f4f0e6';c.fillRect(0,0,r.w,r.h);c.fillStyle='#111827';c.font='bold 16px Arial';c.fillText('Closed-loop parameter and tracking error convergence',16,24);c.fillStyle='#4b5563';c.font='11px Arial';c.fillText('First 60 measurement steps: ||q-p||, ||p_hat-p||, ||xhat_i-x_i||, and |psihat_i-psi_i| error traces',16,40);const gap=10,top=50,pw=(r.w-3*gap)/2,ph=(r.h-top-2*gap)/2,panels=[{x:gap,y:top,w:pw,h:ph},{x:2*gap+pw,y:top,w:pw,h:ph},{x:gap,y:top+gap+ph,w:pw,h:ph},{x:2*gap+pw,y:top+gap+ph,w:pw,h:ph}];const robot=[{label:'||q-p||',color:'#2563eb',values:d.points.map(p=>p.goalError)}];const target=[{label:'||p_hat-p||',color:'#16a34a',values:d.points.map(p=>p.targetError)}];const pos=[];const yaw=[];for(let bi=0;bi<d.beacons.length;bi++){const col=beaconColors[bi%beaconColors.length];pos.push({label:'B'+(bi+1)+' ||xhat_i-x_i||',color:col,values:d.points.map(p=>beaconPositionError(d,p,bi))});yaw.push({label:'B'+(bi+1)+' |psihat_i-psi_i|',color:col,values:d.points.map(p=>beaconYawError(d,p,bi))});}if(!pos.length)pos.push({label:'n/a',color:'#9ca3af',values:d.points.map(()=>null)});if(!yaw.length)yaw.push({label:'n/a',color:'#9ca3af',values:d.points.map(()=>null)});drawMiniPanel(c,panels[0],'Vehicle to target','distance',robot,d,plotSteps);drawMiniPanel(c,panels[1],'Target estimate parameter','distance',target,d,plotSteps);drawMiniPanel(c,panels[2],'Beacon position parameters','distance',pos,d,plotSteps);drawMiniPanel(c,panels[3],'Yaw/self-calibration parameter','radians',yaw,d,plotSteps);}
function drawTargetScatter(){const d=DATA[scenarioIndex],r=resize(targetScatter),c=r.x,b=targetEstimateBounds(d),t=transform(r.w,r.h,b);c.clearRect(0,0,r.w,r.h);c.fillStyle='#fffefa';c.fillRect(0,0,r.w,r.h);labelWorldAxes(c,t,r,b,'Target Estimates - p_hat(t)','p_hat_x','p_hat_y');drawTargetEstimateHistory(c,t,d,true);cross(c,t,d.target,'#dc2626',9);circle(c,t,d.points[0].targetEstimate,5,'#111827');circle(c,t,d.points[stepIndex].targetEstimate,8,'rgba(22,163,74,.13)','#16a34a',3);const labels=[];drawPointLabel(c,'p',t.x(d.target.x),t.y(d.target.y),labels,r);drawPointLabel(c,'p_hat(0)',t.x(d.points[0].targetEstimate.x),t.y(d.points[0].targetEstimate.y),labels,r);drawPointLabel(c,'p_hat(t)',t.x(d.points[stepIndex].targetEstimate.x),t.y(d.points[stepIndex].targetEstimate.y),labels,r);drawTargetLegend(c,r);}
function drawBeaconScatter(){const d=DATA[scenarioIndex],r=resize(beaconScatter),c=r.x,b=beaconBounds(d),t=transform(r.w,r.h,b);c.clearRect(0,0,r.w,r.h);c.fillStyle='#fffefa';c.fillRect(0,0,r.w,r.h);labelWorldAxes(c,t,r,b,'Beacon estimate scatter: x_i and x_hat_i(k)','x position','y position');drawBeaconLegend(c,r);d.beacons.forEach((b,i)=>{c.fillStyle='#f97316';c.fillRect(t.x(b.x)-6,t.y(b.y)-6,12,12);c.fillStyle='#9a3412';c.fillText('x_'+i,t.x(b.x)+10,t.y(b.y)+4)});for(let bi=0;bi<d.beacons.length;bi++){const col=beaconColors[bi%beaconColors.length];for(let k=0;k<=stepIndex;k++){const b=d.points[k].beaconEstimates[bi];if(!b)continue;c.globalAlpha=.5+.42*(k/Math.max(1,stepIndex));c.fillStyle=col;c.beginPath();c.arc(t.x(b.x),t.y(b.y),3.2,0,Math.PI*2);c.fill();c.globalAlpha=.7;c.strokeStyle='#111827';c.lineWidth=.6;c.stroke()}c.globalAlpha=1;const cur=d.points[stepIndex].beaconEstimates[bi];if(cur){cross(c,t,cur,col,8);c.fillStyle=col;c.fillText('xhat_'+bi,t.x(cur.x)+10,t.y(cur.y)-7)}}c.globalAlpha=1;}
function metrics(){const d=DATA[scenarioIndex],p=d.points[stepIndex];stepInput.max=d.points.length-1;stepInput.value=stepIndex;document.getElementById('stepLabel').textContent=p.step+' / '+(d.points.length-1);document.getElementById('robotMetric').textContent='('+fmt(p.robot.x)+', '+fmt(p.robot.y)+')';document.getElementById('targetMetric').textContent='('+fmt(p.targetEstimate.x)+', '+fmt(p.targetEstimate.y)+')';document.getElementById('goalErrorMetric').textContent=fmt(p.goalError,5);document.getElementById('targetErrorMetric').textContent=fmt(p.targetError,5);document.getElementById('beaconPositionMetric').textContent=fmtMaybe(p.beaconPositionRmse,5);document.getElementById('beaconYawMetric').textContent=fmtMaybe(p.beaconYawRmse,5);document.getElementById('costMetric').textContent=fmt(p.cost,4);const body=document.querySelector('#beaconTable tbody');body.innerHTML='';p.beaconEstimates.forEach((b,i)=>{const rv=Math.hypot(p.robot.x-b.x,p.robot.y-b.y),bv=Math.atan2(p.robot.y-b.y,p.robot.x-b.x),rt=Math.hypot(p.targetEstimate.x-b.x,p.targetEstimate.y-b.y),bt=Math.atan2(p.targetEstimate.y-b.y,p.targetEstimate.x-b.x),tr=document.createElement('tr');tr.innerHTML='<td>B'+i+'</td><td>'+fmt(b.x)+'</td><td>'+fmt(b.y)+'</td><td>'+fmt(rv)+'</td><td>'+fmt(bv)+'</td><td>'+fmt(rt)+'</td><td>'+fmt(bt)+'</td>';body.appendChild(tr)});}
function render(){drawScene();drawErrors();drawTargetScatter();drawBeaconScatter();metrics()}function stop(){if(timer){clearInterval(timer);timer=null}playButton.textContent='Play'}function toggle(){if(timer){stop();return}playButton.textContent='Pause';timer=setInterval(()=>{stepIndex=Math.min(DATA[scenarioIndex].points.length-1,stepIndex+1);render();if(stepIndex>=DATA[scenarioIndex].points.length-1)stop()},85)}
scenarioSelect.onchange=e=>{scenarioIndex=Number(e.target.value);stepIndex=0;stop();render()};stepInput.oninput=e=>{stepIndex=Number(e.target.value);render()};playButton.onclick=toggle;document.getElementById('prev').onclick=()=>{stepIndex=Math.max(0,stepIndex-1);render()};document.getElementById('next').onclick=()=>{stepIndex=Math.min(DATA[scenarioIndex].points.length-1,stepIndex+1);render()};window.onresize=render;render();
</script></body></html>)HTML";
}

void write_adaptive_localization_csv(
    const std::filesystem::path& path,
    const AdaptiveLocalizationRun& result) {
    std::ofstream out(path);
    out << "step,robot_x,robot_y,target_estimate_x,target_estimate_y,target_error,goal_error,cost\n";
    out << std::fixed << std::setprecision(8);
    for (const auto& point : result.points) {
        out << point.step << ','
            << point.robot.x << ','
            << point.robot.y << ','
            << point.target_estimate.x << ','
            << point.target_estimate.y << ','
            << point.target_error << ','
            << point.goal_error << ','
            << point.cost << '\n';
    }
}

}  // namespace adaptive
