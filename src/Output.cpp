/**
 * @file Output.cpp
 * @brief Serialization of simulation results for offline inspection and plotting.
 *
 * This file implements every `write_*` entry point declared in Output.hpp: CSV writers for
 * the various Monte Carlo sweep/summary row structs (see Types.hpp), CSV/SVG writers for
 * closed-loop trajectories and adaptive-localization runs, and a single self-contained HTML
 * viewer (embedding per-step JSON plus inline CSS/JS) that animates a run. The resulting
 * files are consumed downstream by the paper's figures and by Python plotting scripts; no
 * simulation logic lives here, only formatting.
 */

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

/**
 * @brief Fixed world-to-SVG-pixel affine transform used by write_svg_plot().
 *
 * Maps a hard-coded world-space window (x in [min_x, max_x], y in [min_y, max_y]) onto an
 * SVG canvas of size width x height with a border of `margin` pixels on every side. The
 * window is not auto-fit to the data; it is a fixed viewport chosen to frame the scenarios
 * this file plots. sx()/sy() perform the linear mapping; sy() additionally flips the y axis
 * because SVG y grows downward while world y grows upward.
 */
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

/**
 * @brief Write a metric to a CSV stream, leaving the field blank if it is a "not applicable" sentinel.
 *
 * Several row structs (see Types.hpp) use a negative value (typically -1.0) for
 * beacon_yaw_rmse-style fields to mean "beacon yaw was not estimated/not applicable for this
 * scenario or estimator" (e.g. the calibrated baseline in scenario 2 does not estimate beacon
 * yaw at all). This helper centralizes that convention for CSV output: negative values are
 * skipped entirely (the CSV cell is left empty between the surrounding commas), non-negative
 * values are streamed normally.
 */
void write_optional_metric(std::ostream& out, double value) {
    if (value < 0.0) {
        return;
    }
    out << value;
}

/**
 * @brief JSON counterpart of write_optional_metric(): emits the literal `null` for negative
 * (not-applicable) sentinel values instead of leaving the field blank, since JSON has no
 * concept of an omitted scalar in this inline-array-building style. Non-negative values are
 * streamed normally. Used when embedding ClosedLoopResult data into the HTML viewer's JS
 * `DATA` array, where the front-end code checks for `null` (see `fmtMaybe` in
 * write_html_viewer()).
 */
void write_json_optional_metric(std::ostream& out, double value) {
    if (value < 0.0) {
        out << "null";
        return;
    }
    out << value;
}

/**
 * @brief Shared boilerplate for every write_*_csv row-writer below: opens `path`, writes the
 * literal `header` (expected to end in its own "\n"), switches the stream to fixed notation at
 * 8 decimal places (the convention every one of these writers already used), then calls
 * `write_row(out, row)` once per element of `rows` in row order, appending a newline after each
 * call. `write_row` is responsible only for a row's own comma-separated fields (including any
 * write_optional_metric() calls for "not applicable" sentinel columns) -- the file open,
 * header, precision setup, and per-row newline are handled once here instead of being repeated
 * verbatim by every caller below.
 */
template <typename Row, typename RowWriter>
void write_csv(
    const std::filesystem::path& path,
    const char* header,
    const std::vector<Row>& rows,
    RowWriter write_row) {
    std::ofstream out(path);
    out << header;
    out << std::fixed << std::setprecision(8);
    for (const auto& row : rows) {
        write_row(out, row);
        out << '\n';
    }
}

/**
 * @brief Serialize a ClosedLoopResult to a single JSON object literal on `out`.
 *
 * Emits, in order: `scenario` (int), `target` ({x,y}), `beacons` (array of {x,y,yaw} in
 * world.beacons/world.beacon_yaws order), and `points` (one object per ClosedLoopPoint with
 * step, robot, targetEstimate, targetError, goalError, beaconPositionRmse, beaconYawRmse
 * (null when not applicable, via write_json_optional_metric), cost, and the per-step
 * beaconEstimates array of {x,y,yaw}). This is not a standalone file format: it produces a
 * bare JSON value with no trailing newline, intended to be embedded directly inside a
 * JavaScript array literal by write_html_viewer(). No whitespace/pretty-printing is added.
 */
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

/**
 * @brief Serialize an AdaptiveLocalizationRun to a single JSON object literal on `out`.
 *
 * Same shape and purpose as write_closed_loop_json(), for the adaptive-localization run that
 * is displayed as the 4th scenario in the HTML viewer. `scenario` is hard-coded to 3 (the
 * viewer's scenario index for "adaptive law + target-seeking controller"), since
 * AdaptiveLocalizationRun itself carries no scenario id. AdaptiveLocalizationPoint has no
 * beacon_position_rmse/beacon_yaw_rmse/retriggered fields, so unlike write_closed_loop_json()
 * this only emits step, robot, targetEstimate, targetError, goalError, cost, and
 * beaconEstimates per point.
 */
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

/**
 * @brief Write per-(scenario, beacon-count) aggregate accuracy statistics to CSV.
 *
 * One row per SummaryRow: scenario id, beacon count, target-position RMSE and its 95% CI,
 * mean signed error and its 95% CI, per-axis bias (bias_x, bias_y), convergence rate (written
 * under the CSV header `success_rate`), mean solver cost/iterations/runtime, and mean beacon
 * position RMSE (with CI). The trailing mean_beacon_yaw_rmse / mean_beacon_yaw_rmse_ci95
 * columns use write_optional_metric() so rows where beacon yaw is not estimated (sentinel
 * -1.0, see Types.hpp) are written as empty CSV cells rather than -1. Values use fixed
 * notation at 8 decimal places; the header row is written first.
 */
void write_summary_csv(const std::filesystem::path& path, const std::vector<SummaryRow>& rows) {
    write_csv(
        path,
        "scenario,beacons,rmse,rmse_ci95,mean_error,mean_error_ci95,bias_x,bias_y,"
        "success_rate,mean_cost,mean_iterations,mean_runtime_ms,"
        "mean_beacon_position_rmse,mean_beacon_position_rmse_ci95,"
        "mean_beacon_yaw_rmse,mean_beacon_yaw_rmse_ci95\n",
        rows,
        [](std::ostream& out, const SummaryRow& row) {
            out << row.scenario << ',' << row.beacons << ',' << row.rmse << ',' << row.rmse_ci95 << ','
                << row.mean_error << ',' << row.mean_error_ci95 << ','
                << row.bias_x << ',' << row.bias_y << ',' << row.convergence_rate << ',' << row.mean_cost << ','
                << row.mean_iterations << ',' << row.mean_runtime_ms << ','
                << row.mean_beacon_position_rmse << ',' << row.mean_beacon_position_rmse_ci95 << ',';
            write_optional_metric(out, row.mean_beacon_yaw_rmse);
            out << ',';
            write_optional_metric(out, row.mean_beacon_yaw_rmse_ci95);
        });
}

/**
 * @brief Write one row per individual Monte Carlo trial to CSV.
 *
 * One row per TrialResult: scenario, beacon count, trial index, true target position
 * (target_x/target_y, from `truth`), estimated target position (estimate_x/estimate_y),
 * target error, beacon position RMSE, beacon yaw RMSE (blank via write_optional_metric() when
 * the -1.0 "not applicable" sentinel is set), solver cost, iterations, runtime in ms, and two
 * boolean flags emitted as 0/1: solver_converged (raw solver convergence flag) and `success`
 * (CSV header name for TrialResult::converged, the trial-level success criterion). Fixed
 * notation, 8 decimal places.
 */
void write_trial_csv(const std::filesystem::path& path, const std::vector<TrialResult>& trials) {
    write_csv(
        path,
        "scenario,beacons,trial,target_x,target_y,estimate_x,estimate_y,error,"
        "beacon_position_rmse,beacon_yaw_rmse,cost,iterations,runtime_ms,solver_converged,success\n",
        trials,
        [](std::ostream& out, const TrialResult& t) {
            out << t.scenario << ',' << t.beacons << ',' << t.trial << ',' << t.truth.x << ',' << t.truth.y << ','
                << t.estimate.x << ',' << t.estimate.y << ',' << t.error << ','
                << t.beacon_position_rmse << ',';
            write_optional_metric(out, t.beacon_yaw_rmse);
            out << ',' << t.cost << ',' << t.iterations << ',' << t.runtime_ms << ','
                << (t.solver_converged ? 1 : 0) << ',' << (t.converged ? 1 : 0);
        });
}

/**
 * @brief Write the noise-robustness sweep (target/beacon accuracy vs. measurement noise) to CSV.
 *
 * One row per NoiseRobustnessRow: scenario, beacon count, the range and bearing noise sigmas
 * used for that trial batch, target RMSE, beacon position RMSE, beacon yaw RMSE (blank via
 * write_optional_metric() when not applicable), mean cost/iterations/runtime, and convergence
 * rate (CSV header `success_rate`). This CSV is also the data source consumed by
 * write_noise_robustness_svg() to plot target RMSE vs. range sigma.
 */
void write_noise_robustness_csv(const std::filesystem::path& path, const std::vector<NoiseRobustnessRow>& rows) {
    write_csv(
        path,
        "scenario,beacons,range_sigma,bearing_sigma,target_rmse,beacon_position_rmse,"
        "beacon_yaw_rmse,mean_cost,mean_iterations,mean_runtime_ms,success_rate\n",
        rows,
        [](std::ostream& out, const NoiseRobustnessRow& row) {
            out << row.scenario << ',' << row.beacons << ','
                << row.range_sigma << ',' << row.bearing_sigma << ','
                << row.target_rmse << ',' << row.beacon_position_rmse << ',';
            write_optional_metric(out, row.beacon_yaw_rmse);
            out << ',' << row.mean_cost << ',' << row.mean_iterations << ','
                << row.mean_runtime_ms << ',' << row.convergence_rate;
        });
}

/**
 * @brief Write the beacon-geometry sweep (accuracy vs. beacon separation) to CSV.
 *
 * One row per GeometrySweepRow: beacon count, beacon separation distance, target RMSE, beacon
 * position RMSE, beacon yaw RMSE, mean cost/iterations/runtime, and convergence rate (CSV
 * header `success_rate`). Unlike several other sweep rows, GeometrySweepRow::beacon_yaw_rmse
 * has no -1.0 sentinel convention here, so it is written directly with `<<` rather than via
 * write_optional_metric().
 */
void write_geometry_sweep_csv(const std::filesystem::path& path, const std::vector<GeometrySweepRow>& rows) {
    write_csv(
        path,
        "beacons,beacon_separation,target_rmse,beacon_position_rmse,beacon_yaw_rmse,"
        "mean_cost,mean_iterations,mean_runtime_ms,success_rate\n",
        rows,
        [](std::ostream& out, const GeometrySweepRow& row) {
            out << row.beacons << ',' << row.beacon_separation << ','
                << row.target_rmse << ',' << row.beacon_position_rmse << ','
                << row.beacon_yaw_rmse << ',' << row.mean_cost << ','
                << row.mean_iterations << ',' << row.mean_runtime_ms << ','
                << row.convergence_rate;
        });
}

/**
 * @brief Write the vehicle-trajectory sweep (accuracy/observability vs. trajectory shape) to CSV.
 *
 * One row per TrajectorySweepRow: trajectory name/label, beacon count, observability rank and
 * smallest singular value of the observability/information matrix, target RMSE, beacon
 * position RMSE, beacon yaw RMSE, mean cost/iterations/runtime, and convergence rate (CSV
 * header `success_rate`).
 */
void write_trajectory_sweep_csv(const std::filesystem::path& path, const std::vector<TrajectorySweepRow>& rows) {
    write_csv(
        path,
        "trajectory,beacons,observability_rank,smallest_singular_value,target_rmse,"
        "beacon_position_rmse,beacon_yaw_rmse,mean_cost,mean_iterations,mean_runtime_ms,success_rate\n",
        rows,
        [](std::ostream& out, const TrajectorySweepRow& row) {
            out << row.trajectory << ',' << row.beacons << ',' << row.observability_rank << ','
                << row.smallest_singular_value << ',' << row.target_rmse << ','
                << row.beacon_position_rmse << ',' << row.beacon_yaw_rmse << ','
                << row.mean_cost << ',' << row.mean_iterations << ',' << row.mean_runtime_ms << ','
                << row.convergence_rate;
        });
}

/**
 * @brief Write the initial-robot-pose robustness sweep to CSV.
 *
 * One row per InitialPoseRobustnessRow: scenario, trial index, the robot's initial position
 * (initial_robot_x/y), and final-step accuracy metrics: goal error, target error, beacon
 * position RMSE, and beacon yaw RMSE (blank via write_optional_metric() when the -1.0
 * sentinel indicates yaw is not estimated for that scenario).
 */
void write_initial_pose_robustness_csv(
    const std::filesystem::path& path,
    const std::vector<InitialPoseRobustnessRow>& rows) {
    write_csv(
        path,
        "scenario,trial,initial_robot_x,initial_robot_y,final_goal_error,final_target_error,"
        "final_beacon_position_rmse,final_beacon_yaw_rmse\n",
        rows,
        [](std::ostream& out, const InitialPoseRobustnessRow& row) {
            out << row.scenario << ',' << row.trial << ','
                << row.initial_robot.x << ',' << row.initial_robot.y << ','
                << row.final_goal_error << ',' << row.final_target_error << ','
                << row.final_beacon_position_rmse << ',';
            write_optional_metric(out, row.final_beacon_yaw_rmse);
        });
}

/**
 * @brief Write the minimal-beacon-excitation case study results to CSV.
 *
 * One row per MinimalBeaconExcitationRow: case name (CSV header `case`), beacon count, pose
 * count, observability rank, smallest singular value, target error, trajectory spread,
 * beacon position error, beacon yaw error, solver cost/iterations, and converged flag emitted
 * as 0/1 under the CSV header `solver_converged`. Note the CSV column order
 * (..., target_error, trajectory_spread, beacon_position_error, ...) differs slightly from
 * the struct's declaration order in Types.hpp (trajectory_spread is declared before
 * smallest_singular_value there); this writer's own header row is the authoritative column
 * order for the emitted file.
 */
void write_minimal_beacon_excitation_csv(
    const std::filesystem::path& path,
    const std::vector<MinimalBeaconExcitationRow>& rows) {
    write_csv(
        path,
        "case,beacons,poses,observability_rank,smallest_singular_value,target_error,"
        "trajectory_spread,beacon_position_error,beacon_yaw_error,cost,iterations,solver_converged\n",
        rows,
        [](std::ostream& out, const MinimalBeaconExcitationRow& row) {
            out << row.case_name << ',' << row.beacons << ',' << row.poses << ','
                << row.observability_rank << ',' << row.smallest_singular_value << ','
                << row.target_error << ',' << row.trajectory_spread << ',' << row.beacon_position_error << ','
                << row.beacon_yaw_error << ',' << row.cost << ',' << row.iterations << ','
                << (row.converged ? 1 : 0);
        });
}

/**
 * @brief Write the poor-initialization / multistart robustness sweep to CSV.
 *
 * One row per PoorInitializationSweepRow: case name (CSV header `case`), the seed
 * perturbation parameters (target_seed_offset, beacon_seed_radius, beacon_yaw_seed), the
 * multistart count, target RMSE, beacon position RMSE, beacon yaw RMSE, mean
 * cost/iterations/runtime, and convergence rate (CSV header `success_rate`).
 */
void write_poor_initialization_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<PoorInitializationSweepRow>& rows) {
    write_csv(
        path,
        "case,target_seed_offset,beacon_seed_radius,beacon_yaw_seed,multistarts,"
        "target_rmse,beacon_position_rmse,beacon_yaw_rmse,mean_cost,mean_iterations,"
        "mean_runtime_ms,success_rate\n",
        rows,
        [](std::ostream& out, const PoorInitializationSweepRow& row) {
            out << row.case_name << ',' << row.target_seed_offset << ','
                << row.beacon_seed_radius << ',' << row.beacon_yaw_seed << ','
                << row.multistarts << ',' << row.target_rmse << ','
                << row.beacon_position_rmse << ',' << row.beacon_yaw_rmse << ','
                << row.mean_cost << ',' << row.mean_iterations << ','
                << row.mean_runtime_ms << ',' << row.convergence_rate;
        });
}

/**
 * @brief Write the intermittent-measurement (dropout) robustness sweep to CSV.
 *
 * One row per IntermittentMeasurementSweepRow: dropout probability, the resulting mean
 * number of measurements actually received, target RMSE, beacon position RMSE, beacon yaw
 * RMSE, mean cost/iterations/runtime, and convergence rate (CSV header `success_rate`).
 */
void write_intermittent_measurement_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<IntermittentMeasurementSweepRow>& rows) {
    write_csv(
        path,
        "dropout_probability,mean_measurements,target_rmse,beacon_position_rmse,"
        "beacon_yaw_rmse,mean_cost,mean_iterations,mean_runtime_ms,success_rate\n",
        rows,
        [](std::ostream& out, const IntermittentMeasurementSweepRow& row) {
            out << row.dropout_probability << ',' << row.mean_measurements << ','
                << row.target_rmse << ',' << row.beacon_position_rmse << ','
                << row.beacon_yaw_rmse << ',' << row.mean_cost << ','
                << row.mean_iterations << ',' << row.mean_runtime_ms << ','
                << row.convergence_rate;
        });
}

/**
 * @brief Write the outlier-measurement robustness sweep to CSV.
 *
 * One row per OutlierRobustnessSweepRow: estimator name, outlier occurrence probability, the
 * outlier magnitude injected into range and bearing measurements, target RMSE, beacon
 * position RMSE, beacon yaw RMSE, mean cost/iterations/runtime, and convergence rate (CSV
 * header `success_rate`). Used to compare robust vs. non-robust estimators under outliers.
 */
void write_outlier_robustness_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<OutlierRobustnessSweepRow>& rows) {
    write_csv(
        path,
        "estimator,outlier_probability,outlier_range_magnitude,outlier_bearing_magnitude,"
        "target_rmse,beacon_position_rmse,beacon_yaw_rmse,mean_cost,mean_iterations,"
        "mean_runtime_ms,success_rate\n",
        rows,
        [](std::ostream& out, const OutlierRobustnessSweepRow& row) {
            out << row.estimator << ',' << row.outlier_probability << ','
                << row.outlier_range_magnitude << ',' << row.outlier_bearing_magnitude << ','
                << row.target_rmse << ',' << row.beacon_position_rmse << ','
                << row.beacon_yaw_rmse << ',' << row.mean_cost << ','
                << row.mean_iterations << ',' << row.mean_runtime_ms << ','
                << row.convergence_rate;
        });
}

/**
 * @brief Write the vehicle self-localization noise sweep to CSV.
 *
 * One row per VehicleLocalizationNoiseSweepRow: case name (CSV header `case`), the injected
 * vehicle position noise sigma, target RMSE, beacon position RMSE, beacon yaw RMSE, mean
 * cost/iterations/runtime, and convergence rate (CSV header `success_rate`). This models the
 * effect of imperfect vehicle self-localization (as opposed to range/bearing measurement
 * noise) on target/beacon estimation accuracy.
 */
void write_vehicle_localization_noise_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<VehicleLocalizationNoiseSweepRow>& rows) {
    write_csv(
        path,
        "case,vehicle_position_sigma,target_rmse,beacon_position_rmse,beacon_yaw_rmse,"
        "mean_cost,mean_iterations,mean_runtime_ms,success_rate\n",
        rows,
        [](std::ostream& out, const VehicleLocalizationNoiseSweepRow& row) {
            out << row.case_name << ',' << row.vehicle_position_sigma << ',' << row.target_rmse << ','
                << row.beacon_position_rmse << ',' << row.beacon_yaw_rmse << ','
                << row.mean_cost << ',' << row.mean_iterations << ','
                << row.mean_runtime_ms << ',' << row.convergence_rate;
        });
}

/**
 * @brief Write the Fisher-information/observability conditioning study to CSV.
 *
 * One row per InformationConditioningRow: trajectory name, beacon count, observation count,
 * observability rank, smallest and largest singular values of the information matrix,
 * trajectory spread, condition number, and log-determinant of the information matrix. Note
 * the CSV column order places trajectory_spread after largest/smallest singular values
 * (`...,smallest_singular_value,trajectory_spread,largest_singular_value,...`), which differs
 * from the struct's declaration order in Types.hpp; the header row below is authoritative for
 * the emitted file. No convergence/cost columns are included, since this row characterizes
 * problem conditioning rather than solver outcomes.
 */
void write_information_conditioning_csv(
    const std::filesystem::path& path,
    const std::vector<InformationConditioningRow>& rows) {
    write_csv(
        path,
        "trajectory,beacons,observations,observability_rank,smallest_singular_value,"
        "trajectory_spread,largest_singular_value,condition_number,logdet_information\n",
        rows,
        [](std::ostream& out, const InformationConditioningRow& row) {
            out << row.trajectory << ',' << row.beacons << ',' << row.observations << ','
                << row.observability_rank << ',' << row.smallest_singular_value << ','
                << row.trajectory_spread << ','
                << row.largest_singular_value << ',' << row.condition_number << ','
                << row.logdet_information;
        });
}

/**
 * @brief Write the expanded baseline-comparison summary (multiple estimators/cases) to CSV.
 *
 * One row per ExpandedBaselineSummaryRow: case name, estimator name, beacon count, target
 * RMSE and its 95% CI, beacon position RMSE and its 95% CI, beacon yaw RMSE and its 95% CI,
 * mean cost/iterations/runtime, and convergence rate (CSV header `success_rate`). Unlike
 * SummaryRow, this struct's beacon_yaw_rmse/CI fields have no -1.0 sentinel here, so they are
 * written directly rather than via write_optional_metric().
 */
void write_expanded_baseline_summary_csv(
    const std::filesystem::path& path,
    const std::vector<ExpandedBaselineSummaryRow>& rows) {
    write_csv(
        path,
        "case,estimator,beacons,target_rmse,target_rmse_ci95,beacon_position_rmse,"
        "beacon_position_rmse_ci95,beacon_yaw_rmse,beacon_yaw_rmse_ci95,"
        "mean_cost,mean_iterations,mean_runtime_ms,success_rate\n",
        rows,
        [](std::ostream& out, const ExpandedBaselineSummaryRow& row) {
            out << row.case_name << ',' << row.estimator << ',' << row.beacons << ','
                << row.target_rmse << ',' << row.target_rmse_ci95 << ','
                << row.beacon_position_rmse << ',' << row.beacon_position_rmse_ci95 << ','
                << row.beacon_yaw_rmse << ',' << row.beacon_yaw_rmse_ci95 << ','
                << row.mean_cost << ','
                << row.mean_iterations << ',' << row.mean_runtime_ms << ','
                << row.convergence_rate;
        });
}

/**
 * @brief Write the active-excitation strategy comparison to CSV.
 *
 * One row per ActiveExcitationComparisonRow: excitation strategy name, beacon count, and
 * final-step accuracy metrics: goal error, target error, beacon position RMSE, beacon yaw
 * RMSE, and cost. Column headers carry explicit units (`_m` for the distance-based error
 * metrics, `_rad` for the yaw RMSE) since this row has no -1.0 "not applicable" sentinel
 * convention and all fields are always meaningful.
 */
void write_active_excitation_comparison_csv(
    const std::filesystem::path& path,
    const std::vector<ActiveExcitationComparisonRow>& rows) {
    write_csv(
        path,
        "excitation,beacons,final_goal_error_m,final_target_error_m,"
        "final_beacon_position_rmse_m,final_beacon_yaw_rmse_rad,final_cost\n",
        rows,
        [](std::ostream& out, const ActiveExcitationComparisonRow& row) {
            out << row.excitation << ',' << row.beacons << ','
                << row.final_goal_error << ',' << row.final_target_error << ','
                << row.final_beacon_position_rmse << ',' << row.final_beacon_yaw_rmse << ','
                << row.final_cost;
        });
}

/**
 * @brief Write the supervised-excitation strategy comparison to CSV.
 *
 * One row per SupervisedExcitationComparisonRow: excitation strategy name, how many times the
 * supervisor retriggered active excitation, how many steps it took to first cross the goal-
 * and target-error thresholds (-1 if the threshold was never reached, written as-is, not via
 * write_optional_metric()), and final-step accuracy metrics (goal error, target error, beacon
 * position RMSE, beacon yaw RMSE, cost), with `_m`/`_rad` unit suffixes on the distance/angle
 * columns as in write_active_excitation_comparison_csv().
 */
void write_supervised_excitation_comparison_csv(
    const std::filesystem::path& path,
    const std::vector<SupervisedExcitationComparisonRow>& rows) {
    write_csv(
        path,
        "excitation,retrigger_count,steps_to_goal_threshold,steps_to_target_threshold,"
        "final_goal_error_m,final_target_error_m,final_beacon_position_rmse_m,"
        "final_beacon_yaw_rmse_rad,final_cost\n",
        rows,
        [](std::ostream& out, const SupervisedExcitationComparisonRow& row) {
            out << row.excitation << ',' << row.retrigger_count << ','
                << row.steps_to_goal_threshold << ',' << row.steps_to_target_threshold << ','
                << row.final_goal_error << ',' << row.final_target_error << ','
                << row.final_beacon_position_rmse << ',' << row.final_beacon_yaw_rmse << ','
                << row.final_cost;
        });
}

/**
 * @brief Write the fixed-schedule decay-rate (lambda) Monte Carlo sweep to CSV.
 *
 * One row per SupervisedLambdaSweepRow: the decay rate lambda, the paired Monte Carlo batch
 * size (`trials`), the supervised controller's mean retrigger count at that lambda, and
 * across-trial RMSE metrics for both the fixed baseline and the supervised controller side by
 * side: target RMSE, beacon position RMSE, and beacon yaw RMSE, each with its
 * percentile-bootstrap 95% CI ([ci_lo, ci_hi] columns) and a yaw success rate (fraction of
 * trials at or below the declared 0.05-rad criterion), with `_m`/`_rad` unit suffixes. This
 * lets the two policies be compared directly at each lambda value.
 */
void write_supervised_lambda_sweep_csv(
    const std::filesystem::path& path,
    const std::vector<SupervisedLambdaSweepRow>& rows) {
    write_csv(
        path,
        "lambda,trials,supervised_mean_retrigger_count,"
        "fixed_target_rmse_m,fixed_target_rmse_ci_lo_m,fixed_target_rmse_ci_hi_m,"
        "fixed_beacon_position_rmse_m,fixed_beacon_position_rmse_ci_lo_m,"
        "fixed_beacon_position_rmse_ci_hi_m,"
        "fixed_beacon_yaw_rmse_rad,fixed_beacon_yaw_rmse_ci_lo_rad,"
        "fixed_beacon_yaw_rmse_ci_hi_rad,fixed_yaw_success_rate,"
        "supervised_target_rmse_m,supervised_target_rmse_ci_lo_m,"
        "supervised_target_rmse_ci_hi_m,"
        "supervised_beacon_position_rmse_m,supervised_beacon_position_rmse_ci_lo_m,"
        "supervised_beacon_position_rmse_ci_hi_m,"
        "supervised_beacon_yaw_rmse_rad,supervised_beacon_yaw_rmse_ci_lo_rad,"
        "supervised_beacon_yaw_rmse_ci_hi_rad,supervised_yaw_success_rate\n",
        rows,
        [](std::ostream& out, const SupervisedLambdaSweepRow& row) {
            out << row.lambda << ',' << row.trials << ','
                << row.supervised_mean_retrigger_count << ','
                << row.fixed_target_rmse << ',' << row.fixed_target_rmse_ci_lo << ','
                << row.fixed_target_rmse_ci_hi << ','
                << row.fixed_beacon_position_rmse << ',' << row.fixed_beacon_position_rmse_ci_lo << ','
                << row.fixed_beacon_position_rmse_ci_hi << ','
                << row.fixed_beacon_yaw_rmse << ',' << row.fixed_beacon_yaw_rmse_ci_lo << ','
                << row.fixed_beacon_yaw_rmse_ci_hi << ',' << row.fixed_yaw_success_rate << ','
                << row.supervised_target_rmse << ',' << row.supervised_target_rmse_ci_lo << ','
                << row.supervised_target_rmse_ci_hi << ','
                << row.supervised_beacon_position_rmse << ','
                << row.supervised_beacon_position_rmse_ci_lo << ','
                << row.supervised_beacon_position_rmse_ci_hi << ','
                << row.supervised_beacon_yaw_rmse << ',' << row.supervised_beacon_yaw_rmse_ci_lo << ','
                << row.supervised_beacon_yaw_rmse_ci_hi << ','
                << row.supervised_yaw_success_rate;
        });
}

/**
 * @brief Write the nontrivial target-seeking Monte Carlo comparison to CSV.
 *
 * One row per SupervisedSeekingComparisonRow (one per excitation policy): policy name, paired
 * Monte Carlo batch size (`trials`), mean retrigger count, the goal/target/yaw success rates,
 * the goal-reached rate with the mean packets-to-goal and its normal-approximation 95% CI
 * among trials that reached it, and across-trial final RMSEs (goal, target, beacon position,
 * beacon yaw) each with a percentile-bootstrap 95% CI ([ci_lo, ci_hi] columns), with
 * `_m`/`_rad` unit suffixes on the distance/angle columns.
 */
void write_supervised_seeking_comparison_csv(
    const std::filesystem::path& path,
    const std::vector<SupervisedSeekingComparisonRow>& rows) {
    write_csv(
        path,
        "excitation,trials,mean_retrigger_count,"
        "goal_success_rate,target_success_rate,yaw_success_rate,"
        "goal_reached_rate,steps_to_goal_mean,steps_to_goal_ci95,"
        "final_goal_rmse_m,final_goal_rmse_ci_lo_m,final_goal_rmse_ci_hi_m,"
        "final_target_rmse_m,final_target_rmse_ci_lo_m,final_target_rmse_ci_hi_m,"
        "final_beacon_position_rmse_m,final_beacon_position_rmse_ci_lo_m,"
        "final_beacon_position_rmse_ci_hi_m,"
        "final_beacon_yaw_rmse_rad,final_beacon_yaw_rmse_ci_lo_rad,"
        "final_beacon_yaw_rmse_ci_hi_rad\n",
        rows,
        [](std::ostream& out, const SupervisedSeekingComparisonRow& row) {
            out << row.excitation << ',' << row.trials << ',' << row.mean_retrigger_count << ','
                << row.goal_success_rate << ',' << row.target_success_rate << ','
                << row.yaw_success_rate << ','
                << row.goal_reached_rate << ',' << row.steps_to_goal_mean << ','
                << row.steps_to_goal_ci95 << ','
                << row.final_goal_rmse << ',' << row.final_goal_rmse_ci_lo << ','
                << row.final_goal_rmse_ci_hi << ','
                << row.final_target_rmse << ',' << row.final_target_rmse_ci_lo << ','
                << row.final_target_rmse_ci_hi << ','
                << row.final_beacon_position_rmse << ','
                << row.final_beacon_position_rmse_ci_lo << ','
                << row.final_beacon_position_rmse_ci_hi << ','
                << row.final_beacon_yaw_rmse << ',' << row.final_beacon_yaw_rmse_ci_lo << ','
                << row.final_beacon_yaw_rmse_ci_hi;
        });
}

/**
 * @brief Write the supervisor spread-threshold (S_bar) Monte Carlo ablation to CSV.
 *
 * One row per SupervisedThresholdAblationRow: the candidate spread threshold, Monte Carlo
 * batch size (`trials`), the design rule's predicted yaw RMSE sigma/sqrt(S_bar) next to the
 * measured across-trial yaw RMSE with its percentile-bootstrap 95% CI and the (fixed,
 * 0.05-rad) yaw success rate, the threshold-reached rate with the mean packets-to-threshold
 * and its normal-approximation 95% CI, the excitation cost columns (mean retrigger/episode
 * counts, mean traveled path length, mean integrated excitation effort), and across-trial
 * target/beacon-position RMSEs with bootstrap CIs, with `_m`/`_rad` unit suffixes.
 */
void write_supervised_threshold_ablation_csv(
    const std::filesystem::path& path,
    const std::vector<SupervisedThresholdAblationRow>& rows) {
    write_csv(
        path,
        "spread_threshold,trials,predicted_yaw_rmse_rad,"
        "yaw_rmse_rad,yaw_rmse_ci_lo_rad,yaw_rmse_ci_hi_rad,yaw_success_rate,"
        "threshold_reached_rate,packets_to_threshold_mean,packets_to_threshold_ci95,"
        "mean_retrigger_count,mean_episode_count,mean_path_length_m,"
        "mean_excitation_effort,"
        "target_rmse_m,target_rmse_ci_lo_m,target_rmse_ci_hi_m,"
        "beacon_position_rmse_m,beacon_position_rmse_ci_lo_m,"
        "beacon_position_rmse_ci_hi_m\n",
        rows,
        [](std::ostream& out, const SupervisedThresholdAblationRow& row) {
            out << row.spread_threshold << ',' << row.trials << ',' << row.predicted_yaw_rmse << ','
                << row.yaw_rmse << ',' << row.yaw_rmse_ci_lo << ',' << row.yaw_rmse_ci_hi << ','
                << row.yaw_success_rate << ','
                << row.threshold_reached_rate << ',' << row.packets_to_threshold_mean << ','
                << row.packets_to_threshold_ci95 << ','
                << row.mean_retrigger_count << ',' << row.mean_episode_count << ','
                << row.mean_path_length << ',' << row.mean_excitation_effort << ','
                << row.target_rmse << ',' << row.target_rmse_ci_lo << ',' << row.target_rmse_ci_hi << ','
                << row.beacon_position_rmse << ',' << row.beacon_position_rmse_ci_lo << ','
                << row.beacon_position_rmse_ci_hi;
        });
}

/**
 * @brief Write a small illustrative example trajectory/world CSV for documentation purposes.
 *
 * Builds a fixed 2-beacon world (make_world(2)) and an 80-step vehicle path
 * (make_vehicle_path(80)), then writes "example_trajectory.csv" into `output_dir` with one
 * row per time step: time index, vehicle position (y_x, y_y), the (constant, repeated every
 * row) target position, and the two beacons' (constant, repeated every row) positions. This
 * is a standalone illustrative dataset, not derived from any solver run; it exists to give
 * downstream tooling/docs a minimal concrete example of the trajectory/world geometry.
 */
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

/**
 * @brief Write the per-step time history of a closed-loop run to CSV.
 *
 * One row per ClosedLoopPoint in result.points, in step order: step index, robot position,
 * target-estimate position, target error, goal error, beacon position RMSE, beacon yaw RMSE
 * (blank via write_optional_metric() when the -1.0 sentinel indicates yaw is not estimated,
 * e.g. for the calibrated baseline scenario), least-squares cost, the supervision/diagnostic
 * columns (`spread`, `sigma_min`, `excitation_norm2` -- see below), whether active
 * excitation was retriggered at that step (0/1), and whether the two-view constructive
 * seed had initialized the estimator by that step (`estimate_ready`, 0/1). This is the CSV
 * counterpart of the per-run time series also embedded as JSON by write_closed_loop_json()
 * for the HTML viewer.
 */
void write_closed_loop_csv(const std::filesystem::path& path, const ClosedLoopResult& result) {
    write_csv(
        path,
        "step,robot_x,robot_y,target_estimate_x,target_estimate_y,target_error,goal_error,"
        "beacon_position_rmse,beacon_yaw_rmse,cost,spread,sigma_min,excitation_norm2,"
        "retriggered,estimate_ready\n",
        result.points,
        [](std::ostream& out, const ClosedLoopPoint& point) {
            out << point.step << ',' << point.robot.x << ',' << point.robot.y << ','
                << point.target_estimate.x << ',' << point.target_estimate.y << ','
                << point.target_error << ',' << point.goal_error << ','
                << point.beacon_position_rmse << ',';
            write_optional_metric(out, point.beacon_yaw_rmse);
            // `spread` is the path-based S_v of the stored window at this step
            // (the exact quantity the excitation supervisor triggers on);
            // `sigma_min` is the whitened-Jacobian conditioning diagnostic, blank
            // when not applicable (scenario 2 / multi-beacon); `excitation_norm2`
            // is ||u_exp||^2 at this step (sum over steps times dt gives the
            // integrated excitation effort).
            out << ',' << point.cost << ',' << point.spread << ',';
            write_optional_metric(out, point.sigma_min);
            out << ',' << point.excitation_norm2 << ',' << (point.retriggered ? 1 : 0) << ','
                << (point.estimate_ready ? 1 : 0);
        });
}

/**
 * @brief Write final per-beacon estimate accuracy plus true final measurement geometry to CSV.
 *
 * One row per beacon in result.world.beacons: scenario id, beacon index, true beacon position
 * and yaw, final estimated beacon position and yaw (from result.beacon_estimates, i.e. the
 * converged/final estimate, not the per-step history), and four derived ground-truth
 * measurement-geometry quantities computed from the *true* (not estimated) beacon, robot, and
 * target positions: final_vehicle_range/bearing (range and bearing from the true beacon to
 * the robot's final position, `result.points.back().robot`) and final_target_range/bearing
 * (range and bearing from the true beacon to the true target). These last four columns
 * describe the actual sensing geometry at the end of the run, independent of estimation
 * error, and are useful for checking observability/geometry rather than accuracy.
 */
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

/**
 * @brief Render a static top-down SVG plot of one closed-loop run's full spatial geometry.
 *
 * Draws, on the fixed SvgTransform world window: the robot trajectory q(t) as a polyline with
 * markers at the start (q(0)) and end; the evolving target-estimate trace p_hat(t) as a
 * sequence of green dots whose opacity ramps from ~0.30 to ~0.90 over the run (earlier
 * estimates are more transparent, so the most recent estimate stands out); the true target p
 * as a red crosshair and the final target estimate as a green ring; each true beacon as an
 * orange square with an "x_i" label; each beacon's per-step estimate history as a small dot
 * trail in a per-beacon color (cycled through `beacon_colors` via `i % 6`, matching the color
 * cycle also used in the HTML viewer's JS `beaconColors`); and each beacon's final estimate as
 * an "x" mark ("xhat_i"). The title switches between "Local-frame model" (scenario 1) and
 * "Calibrated baseline" (any other scenario, i.e. scenario 2). All world coordinates are
 * mapped to pixel space via SvgTransform::sx()/sy(). Coordinate values in the SVG use fixed
 * notation at 3 decimal places.
 */
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
        // Fade earlier target-estimate dots so the trail reads chronologically: opacity
        // ramps linearly from 0.30 (k=0) to 0.90 (last point) as k advances.
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
            // Defensive bounds check: a beacon may not yet have an estimate recorded at an
            // early step (e.g. before it enters observability), so skip rather than index
            // out of range.
            if (i >= result.points[k].beacon_estimates.size()) {
                continue;
            }
            const Vec2 history = result.points[k].beacon_estimates[i].position;
            // Same fade-in-over-time treatment as the target-estimate trail above, but
            // starting slightly more transparent (0.25 instead of 0.30).
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

/**
 * @brief Render an SVG line chart of vehicle-to-target error and target-estimate error vs. step.
 *
 * Plots ||q(t)-p|| (goal_error, blue) and ||p_hat(t)-p|| (target_error, green) as two
 * polylines over the first 60 measurement steps (by point `step` value, not point count).
 * The y axis is auto-scaled to the data: `max_error` is the largest goal/target error seen in
 * the plotted window (with a floor of 0.01 to avoid a degenerate/zero-height axis), and the
 * x axis is a simple linear index-to-pixel mapping over the plotted points, with gridlines/
 * axis labels drawn at 4 evenly spaced y-ticks and 10 evenly spaced x-ticks (labeled with the
 * underlying `step` value, since step and array index can differ once retriggering/dropped
 * points are involved). SVG values use fixed notation at 3 decimal places.
 */
void write_error_curve_svg(const std::filesystem::path& path, const ClosedLoopResult& result) {
    const double width = 1200.0;
    const double height = 1050.0;
    const double margin = 160.0;
    // Restrict the plot to steps with `step <= 60`: walk back from the end of `points` while
    // the trailing point's step number exceeds 60, shrinking plot_count accordingly.
    std::size_t plot_count = result.points.size();
    while (plot_count > 0 && result.points[plot_count - 1].step > 60) {
        --plot_count;
    }
    // Always plot at least 2 points so the polylines/index math below stay well-defined.
    plot_count = std::max<std::size_t>(2, plot_count);
    // Auto-scale the y axis to the larger of the two error curves over the plotted window,
    // with a small floor (0.01) so the axis never collapses to zero height.
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

/**
 * @brief Render an SVG line chart of beacon position RMSE and beacon yaw RMSE vs. step.
 *
 * Same layout/scaling approach as write_error_curve_svg(), but restricted to the first 30
 * steps (by `step` value) and plotting beacon_position_rmse (purple) and beacon_yaw_rmse
 * (teal). The y-axis auto-scale (`max_error`) and the plotted yaw curve both special-case the
 * -1.0 "yaw not applicable" sentinel (see Types.hpp / write_optional_metric()): points with
 * beacon_yaw_rmse < 0 are excluded from the max_error computation, and when actually plotting
 * the yaw polyline, such points are substituted with 0.0 rather than the sentinel value so the
 * curve does not spike to a nonsensical negative-turned-large value. This means that for
 * scenarios where yaw is never estimated (all points sentinel), the yaw curve degenerates to a
 * flat line at 0.
 */
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
        // Skip the -1.0 "not applicable" sentinel when computing the axis scale, otherwise it
        // would be (wrongly) ignored anyway since -1.0 < the position RMSE floor, but this
        // guard keeps the intent explicit and matches the substitution used below.
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
        // Sentinel substitution: draw "not applicable" yaw-RMSE points at 0 rather than at
        // the raw -1.0 sentinel value, which would otherwise plot off-chart / below the axis.
        const double yaw = result.points[i].beacon_yaw_rmse >= 0.0 ? result.points[i].beacon_yaw_rmse : 0.0;
        out << sx(i) << ',' << sy(yaw) << ' ';
    }
    out << "\"/>\n";
    out << "</svg>\n";
}

/**
 * @brief Render an SVG line chart of target RMSE vs. range noise sigma, for a fixed bearing
 * noise "slice", overlaying three curves from the noise-robustness sweep (see
 * write_noise_robustness_csv() / NoiseRobustnessRow): scenario 1 with 1 beacon (purple),
 * scenario 1 with 2 beacons (teal), and scenario 2 (calibrated baseline) with 2 beacons
 * (green).
 *
 * Only rows whose bearing_sigma matches `bearing_slice` (0.006, compared with a 1e-9
 * tolerance for floating-point equality) and whose (scenario, beacons) combination is one of
 * the three plotted series are considered at all -- both for computing the axis bounds
 * (max_sigma, max_rmse, each floored at 0.001) and for drawing the actual curves. Rows are
 * assumed to already be sorted by range_sigma; points are simply connected in row order.
 */
void write_noise_robustness_svg(const std::filesystem::path& path, const std::vector<NoiseRobustnessRow>& rows) {
    const double width = 1200.0;
    const double height = 650.0;
    const double margin = 110.0;
    const double bearing_slice = 0.006;
    double max_sigma = 0.001;
    double max_rmse = 0.001;
    // Axis bounds are derived only from the rows that will actually be plotted below: the
    // fixed bearing-sigma slice, restricted to the three (scenario, beacon-count) series of
    // interest.
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
    // Draws one filtered curve (polyline + per-point markers) for a given (scenario,
    // beacons) pair, restricted to the fixed bearing-sigma slice, in whatever row order the
    // matching rows appear in `rows`.
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

/**
 * @brief Write a single self-contained, offline-viewable HTML page that animates all four
 * simulation scenarios (interactive canvas-based viewer; no server or external assets).
 *
 * The four scenarios, selectable from a dropdown, correspond to the four arguments in order:
 * `local_single_beacon` (local-frame unknown-pose model, 1 beacon), `scenario1` (local-frame
 * unknown-pose model, 2 beacons), `scenario2` (calibrated global-frame baseline, 2 beacons),
 * and `adaptive_localization` (the adaptive law + target-seeking controller run). The
 * function emits one large HTML document via raw string literals: a `<style>` block, a static
 * DOM layout (toolbar with scenario selector/play/prev/next/step-slider, a 2x2 canvas grid for
 * the scene/error/target-scatter/beacon-scatter plots, and a metrics/legend sidebar), followed
 * by a `<script>` block. Inside that script, the four runs are serialized to a JS array
 * literal `DATA` by calling write_closed_loop_json() for the first three (ClosedLoopResult)
 * arguments and write_adaptive_localization_json() for the last (AdaptiveLocalizationRun)
 * argument, each separated by a literal comma written directly to `out`. The remainder of the
 * script (embedded verbatim as a raw string, not generated per-call) implements: per-frame
 * canvas drawing helpers (circle/cross/legend swatches), auto-fit view-bounds computation
 * (bounds()/targetEstimateBounds()/beaconBounds(), each padding the data's bounding box),
 * an affine world-to-canvas transform (transform(), analogous in purpose to
 * SvgTransform::sx()/sy() but computed per-frame and per-canvas from the padded bounds, and
 * square-aspect via a shared `span`), animated per-step rendering of the trajectory/estimate
 * history with fading opacity for older samples, a metrics panel, and a beacon table showing
 * live-computed vehicle/target range-bearing to each beacon estimate. `out` uses a numeric
 * precision of 10 for the embedded JSON (set once, for the whole HTML document); the ostream
 * is otherwise fed pre-formatted markup/script text.
 */
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
    // Build the JS `DATA` array literal by streaming each scenario's JSON object in turn,
    // hand-inserting the separating commas; this must stay in the same order as the
    // `<select id="scenario">` options above (index 0..3) since the front-end indexes
    // straight into DATA by scenarioIndex.
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

/**
 * @brief Write the per-step time history of an adaptive-localization run to CSV.
 *
 * One row per AdaptiveLocalizationPoint in result.points, in step order: step index, robot
 * position, target-estimate position, target error, goal error, and least-squares cost.
 * AdaptiveLocalizationPoint has no beacon_position_rmse/beacon_yaw_rmse/retriggered fields
 * (unlike ClosedLoopPoint), so this CSV has fewer columns than write_closed_loop_csv() and
 * never needs write_optional_metric(). Fixed notation, 8 decimal places.
 */
void write_adaptive_localization_csv(
    const std::filesystem::path& path,
    const AdaptiveLocalizationRun& result) {
    write_csv(
        path,
        "step,robot_x,robot_y,target_estimate_x,target_estimate_y,target_error,goal_error,cost\n",
        result.points,
        [](std::ostream& out, const AdaptiveLocalizationPoint& point) {
            out << point.step << ','
                << point.robot.x << ','
                << point.robot.y << ','
                << point.target_estimate.x << ','
                << point.target_estimate.y << ','
                << point.target_error << ','
                << point.goal_error << ','
                << point.cost;
        });
}

}  // namespace adaptive
