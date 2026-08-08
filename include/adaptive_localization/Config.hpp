#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "adaptive_localization/Types.hpp"

namespace adaptive {

// Runtime settings loaded from config/simulation.ini.
//
// Keep this struct boring on purpose: values are plain data, modules receive
// only the settings they need, and the simulator remains dependency-free.
struct SimulationConfig {
    // Directory where CSV, SVG, and HTML outputs are written.
    std::filesystem::path output_dir = "results";

    // Number of noisy Monte Carlo trials per scenario/beacon-count pair.
    int monte_carlo_trials_per_case = 100;

    // Number of known vehicle poses used in each batch Monte Carlo trial.
    int monte_carlo_path_steps = 80;

    // Beacon counts to compare in Monte Carlo runs.
    std::vector<int> monte_carlo_beacon_counts{1, 2};

    // Scenario IDs to compare. ID 1 is the uncalibrated local-frame cooperative
    // model; ID 2 is the calibrated global-frame baseline.
    std::vector<int> monte_carlo_scenarios{1, 2};

    // Deterministic random seeds for repeatable experiments.
    unsigned int monte_carlo_seed = 42;
    unsigned int closed_loop_seed = 7;

    // Measurement noise for batch Monte Carlo runs.
    Noise monte_carlo_noise{0.03, 0.006};

    // Number of beacons used in the animated closed-loop examples.
    int closed_loop_beacon_count = 2;

    // Number of online measurement/control updates in the animated run.
    int closed_loop_steps = 120;

    // Closed-loop controller integration step size.
    double closed_loop_dt = 0.08;

    // Proportional gain driving robot position toward the current target estimate.
    double closed_loop_control_gain = 1.2;

    // Decaying exploratory swirl that keeps early measurements informative.
    double exploration_amplitude = 0.25;
    double exploration_decay = 0.035;
    double exploration_frequency = 0.45;
    double information_exploration_gain = 0.35;
    double information_gradient_step = 0.08;

    // Excitation-supervised mode (Algorithm 1 in the CDC closed-loop paper):
    // the excitation epoch resets whenever the stored window falls below
    // either threshold, in place of the fixed decaying-swirl schedule.
    double supervised_spread_threshold = 0.05;
    double supervised_sigma_min_threshold = 0.05;
    double supervised_goal_error_threshold = 0.05;
    double supervised_target_error_threshold = 0.01;

    // Initial robot position used by closed-loop visualizations.
    Vec2 initial_robot{-3.0, 2.6};

    // Shared estimator seed used by the closed-loop examples and batch trials.
    // Scenario 1 uses this target seed plus a coarse circular beacon layout.
    // Scenario 2 uses the target seed; its beacon estimates are implied by
    // the target estimate and global range-bearing measurements.
    Vec2 initial_target_estimate{0.0, 0.0};
    double initial_beacon_guess_radius = 2.0;
    double initial_beacon_guess_yaw = 0.0;

    // Measurement noise for the animated closed-loop examples.
    Noise closed_loop_noise{0.02, 0.004};

    // Damped Gauss-Newton controls for batch and closed-loop solves.
    int batch_solver_max_iterations = 80;
    double batch_solver_initial_lambda = 1e-3;
    int closed_loop_solver_max_iterations = 25;
    double closed_loop_solver_initial_lambda = 1e-2;

    // Expanded robustness experiments. These defaults keep the suite useful
    // without making the normal command prohibitively slow on a laptop.
    int expanded_trials_per_case = 50;
    double dropout_probability_max = 0.6;
    double outlier_range_magnitude = 0.75;
    double outlier_bearing_magnitude = 0.35;
    double robust_huber_delta = 3.0;
    double vehicle_pose_noise_max = 0.4;
    int multistart_count = 6;
    int sliding_window_size = 12;

    // Adaptive localization core simulation for the rigorous global-frame model.
    // This uses epsilon_i = (r_i^v)^2 - ||q - phat + r_i^t u_i||^2 and
    // phat_dot = -2 Gamma sum_i epsilon_i (q - phat + r_i^t u_i).
    int adaptive_steps = 1000;
    int adaptive_beacon_count = 2;
    double adaptive_dt = 0.01;
    double adaptive_gain = 0.1;
    double adaptive_target_seeking_gain = 1.2;
    Vec2 adaptive_initial_robot{-3.0, 2.6};
    Vec2 adaptive_initial_target_estimate{0.0, 0.0};
    Noise adaptive_noise{0.0, 0.0};
};

/**
 * Loads a config file. Missing files are allowed and simply return defaults.
 * Unknown keys are ignored so local experiment notes do not break a run.
 * @param path path to an INI-style `key = value` config file (see
 *     config/simulation.ini for the canonical example).
 * @return a SimulationConfig with any recognized keys applied on top of the
 *     struct's default field values.
 */
SimulationConfig load_config(const std::filesystem::path& path);

/**
 * Writes a fully commented default config file if it does not already
 * exist (a no-op if a file is already present at `path`), so that a fresh
 * checkout gets an editable, self-documenting config/simulation.ini.
 */
void write_default_config_if_missing(const std::filesystem::path& path);

/// Joins a list of integers into a single comma-separated string (e.g. for
/// writing SimulationConfig::monte_carlo_beacon_counts /
/// monte_carlo_scenarios back out to a config file).
std::string join_ints(const std::vector<int>& values);

}  // namespace adaptive
