#include "adaptive_localization/Config.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace adaptive {

namespace {

std::string trim(std::string text) {
    const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char c) {
        return std::isspace(c) != 0;
    });
    const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char c) {
        return std::isspace(c) != 0;
    }).base();
    if (first >= last) {
        return {};
    }
    return {first, last};
}

std::vector<int> parse_int_list(const std::string& value) {
    std::vector<int> result;
    std::stringstream stream(value);
    std::string item;
    while (std::getline(stream, item, ',')) {
        item = trim(item);
        if (!item.empty()) {
            result.push_back(std::stoi(item));
        }
    }
    return result;
}

double get_double(
    const std::unordered_map<std::string, std::string>& values,
    const std::string& key,
    double fallback) {
    const auto it = values.find(key);
    return it == values.end() ? fallback : std::stod(it->second);
}

int get_int(
    const std::unordered_map<std::string, std::string>& values,
    const std::string& key,
    int fallback) {
    const auto it = values.find(key);
    return it == values.end() ? fallback : std::stoi(it->second);
}

unsigned int get_uint(
    const std::unordered_map<std::string, std::string>& values,
    const std::string& key,
    unsigned int fallback) {
    const auto it = values.find(key);
    return it == values.end() ? fallback : static_cast<unsigned int>(std::stoul(it->second));
}

std::vector<int> get_int_list(
    const std::unordered_map<std::string, std::string>& values,
    const std::string& key,
    const std::vector<int>& fallback) {
    const auto it = values.find(key);
    if (it == values.end()) {
        return fallback;
    }
    const auto parsed = parse_int_list(it->second);
    return parsed.empty() ? fallback : parsed;
}

}  // namespace

std::string join_ints(const std::vector<int>& values) {
    std::ostringstream out;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            out << ",";
        }
        out << values[i];
    }
    return out.str();
}

SimulationConfig load_config(const std::filesystem::path& path) {
    SimulationConfig config;
    std::ifstream in(path);
    if (!in) {
        return config;
    }

    std::unordered_map<std::string, std::string> values;
    std::string line;
    while (std::getline(in, line)) {
        const auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }
        const auto equals_pos = line.find('=');
        if (equals_pos == std::string::npos) {
            continue;
        }
        const std::string key = trim(line.substr(0, equals_pos));
        const std::string value = trim(line.substr(equals_pos + 1));
        if (!key.empty() && !value.empty()) {
            values[key] = value;
        }
    }

    if (const auto it = values.find("output_dir"); it != values.end()) {
        config.output_dir = it->second;
    }
    config.monte_carlo_trials_per_case = get_int(values, "monte_carlo_trials_per_case", config.monte_carlo_trials_per_case);
    config.monte_carlo_path_steps = get_int(values, "monte_carlo_path_steps", config.monte_carlo_path_steps);
    config.monte_carlo_beacon_counts = get_int_list(values, "monte_carlo_beacon_counts", config.monte_carlo_beacon_counts);
    config.monte_carlo_scenarios = get_int_list(values, "monte_carlo_scenarios", config.monte_carlo_scenarios);
    config.monte_carlo_seed = get_uint(values, "monte_carlo_seed", config.monte_carlo_seed);
    config.closed_loop_seed = get_uint(values, "closed_loop_seed", config.closed_loop_seed);
    config.monte_carlo_noise.range_sigma = get_double(values, "monte_carlo_range_sigma", config.monte_carlo_noise.range_sigma);
    config.monte_carlo_noise.bearing_sigma = get_double(values, "monte_carlo_bearing_sigma", config.monte_carlo_noise.bearing_sigma);
    config.closed_loop_beacon_count = get_int(values, "closed_loop_beacon_count", config.closed_loop_beacon_count);
    config.closed_loop_steps = get_int(values, "closed_loop_steps", config.closed_loop_steps);
    config.closed_loop_dt = get_double(values, "closed_loop_dt", config.closed_loop_dt);
    config.closed_loop_control_gain = get_double(values, "closed_loop_control_gain", config.closed_loop_control_gain);
    config.exploration_amplitude = get_double(values, "exploration_amplitude", config.exploration_amplitude);
    config.exploration_decay = get_double(values, "exploration_decay", config.exploration_decay);
    config.exploration_frequency = get_double(values, "exploration_frequency", config.exploration_frequency);
    config.information_exploration_gain =
        get_double(values, "information_exploration_gain", config.information_exploration_gain);
    config.information_gradient_step =
        get_double(values, "information_gradient_step", config.information_gradient_step);
    config.supervised_spread_threshold =
        get_double(values, "supervised_spread_threshold", config.supervised_spread_threshold);
    config.supervised_sigma_min_threshold =
        get_double(values, "supervised_sigma_min_threshold", config.supervised_sigma_min_threshold);
    config.supervised_goal_error_threshold =
        get_double(values, "supervised_goal_error_threshold", config.supervised_goal_error_threshold);
    config.supervised_target_error_threshold =
        get_double(values, "supervised_target_error_threshold", config.supervised_target_error_threshold);
    config.initial_robot.x = get_double(values, "initial_robot_x", config.initial_robot.x);
    config.initial_robot.y = get_double(values, "initial_robot_y", config.initial_robot.y);
    config.initial_target_estimate.x =
        get_double(values, "initial_target_estimate_x", config.initial_target_estimate.x);
    config.initial_target_estimate.y =
        get_double(values, "initial_target_estimate_y", config.initial_target_estimate.y);
    config.initial_beacon_guess_radius =
        get_double(values, "initial_beacon_guess_radius", config.initial_beacon_guess_radius);
    config.initial_beacon_guess_yaw =
        get_double(values, "initial_beacon_guess_yaw", config.initial_beacon_guess_yaw);
    config.closed_loop_noise.range_sigma = get_double(values, "closed_loop_range_sigma", config.closed_loop_noise.range_sigma);
    config.closed_loop_noise.bearing_sigma = get_double(values, "closed_loop_bearing_sigma", config.closed_loop_noise.bearing_sigma);
    config.batch_solver_max_iterations = get_int(values, "batch_solver_max_iterations", config.batch_solver_max_iterations);
    config.batch_solver_initial_lambda = get_double(values, "batch_solver_initial_lambda", config.batch_solver_initial_lambda);
    config.closed_loop_solver_max_iterations = get_int(values, "closed_loop_solver_max_iterations", config.closed_loop_solver_max_iterations);
    config.closed_loop_solver_initial_lambda = get_double(values, "closed_loop_solver_initial_lambda", config.closed_loop_solver_initial_lambda);
    config.expanded_trials_per_case = get_int(values, "expanded_trials_per_case", config.expanded_trials_per_case);
    config.dropout_probability_max =
        get_double(values, "dropout_probability_max", config.dropout_probability_max);
    config.outlier_range_magnitude =
        get_double(values, "outlier_range_magnitude", config.outlier_range_magnitude);
    config.outlier_bearing_magnitude =
        get_double(values, "outlier_bearing_magnitude", config.outlier_bearing_magnitude);
    config.robust_huber_delta =
        get_double(values, "robust_huber_delta", config.robust_huber_delta);
    config.vehicle_pose_noise_max =
        get_double(values, "vehicle_pose_noise_max", config.vehicle_pose_noise_max);
    config.multistart_count = get_int(values, "multistart_count", config.multistart_count);
    config.sliding_window_size = get_int(values, "sliding_window_size", config.sliding_window_size);
    config.adaptive_steps = get_int(values, "adaptive_steps", config.adaptive_steps);
    config.adaptive_beacon_count = get_int(values, "adaptive_beacon_count", config.adaptive_beacon_count);
    config.adaptive_dt = get_double(values, "adaptive_dt", config.adaptive_dt);
    config.adaptive_gain = get_double(values, "adaptive_gain", config.adaptive_gain);
    config.adaptive_target_seeking_gain =
        get_double(values, "adaptive_target_seeking_gain", config.adaptive_target_seeking_gain);
    config.adaptive_initial_robot.x =
        get_double(values, "adaptive_initial_robot_x", config.adaptive_initial_robot.x);
    config.adaptive_initial_robot.y =
        get_double(values, "adaptive_initial_robot_y", config.adaptive_initial_robot.y);
    config.adaptive_initial_target_estimate.x =
        get_double(values, "adaptive_initial_target_estimate_x", config.adaptive_initial_target_estimate.x);
    config.adaptive_initial_target_estimate.y =
        get_double(values, "adaptive_initial_target_estimate_y", config.adaptive_initial_target_estimate.y);
    config.adaptive_noise.range_sigma =
        get_double(values, "adaptive_range_sigma", config.adaptive_noise.range_sigma);
    config.adaptive_noise.bearing_sigma =
        get_double(values, "adaptive_bearing_sigma", config.adaptive_noise.bearing_sigma);
    return config;
}

void write_default_config_if_missing(const std::filesystem::path& path) {
    if (std::filesystem::exists(path)) {
        return;
    }
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream out(path);
    out << R"CFG(# Adaptive localization simulation configuration.
# Lines beginning with # are comments. Values use: key = value
# Lists use comma-separated integers, for example: 1,2

# Directory where generated CSV, SVG, and HTML files are written.
output_dir = results

# Number of noisy Monte Carlo trials for each scenario/beacon-count pair.
monte_carlo_trials_per_case = 100

# Number of known robot poses used in each batch Monte Carlo trial.
monte_carlo_path_steps = 80

# Beacon counts to compare in the Monte Carlo summary.
monte_carlo_beacon_counts = 1,2

# Scenario IDs to compare.
# 1: uncalibrated local-frame cooperative model with unknown beacon yaw.
# 2: calibrated global-frame baseline with global target bearing and vehicle range.
# 3: EKF local-frame baseline with the same unknown beacon pose/yaw state.
monte_carlo_scenarios = 1,2,3

# Random seeds make the noisy experiments repeatable.
monte_carlo_seed = 42
closed_loop_seed = 7

# Standard deviation of range and bearing noise for Monte Carlo experiments.
# Range is in workspace distance units; bearing is in radians.
monte_carlo_range_sigma = 0.03
monte_carlo_bearing_sigma = 0.006

# Number of beacons used in the animated closed-loop examples.
closed_loop_beacon_count = 2

# Number of online measurement/update/control steps in the animation.
closed_loop_steps = 120

# Controller integration time step.
closed_loop_dt = 0.08

# Proportional gain that moves the robot toward the current target estimate.
closed_loop_control_gain = 1.2

# Decaying exploratory swirl added to the controller.
# This helps early measurements excite the geometry before station keeping.
exploration_amplitude = 0.25
exploration_decay = 0.035
exploration_frequency = 0.45

# Information-driven excitation settings. The controller uses finite
# differences of the regularized local observability log determinant.
information_exploration_gain = 0.35
information_gradient_step = 0.08

# Excitation-supervised mode. The excitation epoch resets whenever the stored
# window's trajectory spread or local-observability sigma_min falls below
# these thresholds, in place of the fixed decaying-swirl schedule. The
# threshold_m settings define "converged" for the packets-to-threshold
# comparison against the fixed schedule.
supervised_spread_threshold = 0.05
supervised_sigma_min_threshold = 0.05
supervised_goal_error_threshold = 0.05
supervised_target_error_threshold = 0.01

# Initial robot position for the closed-loop visualization.
initial_robot_x = -3.0
initial_robot_y = 2.6

# Shared estimator initial conditions used wherever the model permits.
# Scenario 1 starts with this target estimate and a coarse circular layout of
# beacon guesses. Scenario 2 starts from the same target estimate, but beacon
# locations are recovered from its global-frame measurements rather than stored
# as independent optimizer states.
initial_target_estimate_x = 0.0
initial_target_estimate_y = 0.0
initial_beacon_guess_radius = 2.0
initial_beacon_guess_yaw = 0.0

# Standard deviation of range and bearing noise for closed-loop animation.
closed_loop_range_sigma = 0.02
closed_loop_bearing_sigma = 0.004

# Damped Gauss-Newton settings for batch Monte Carlo estimation.
batch_solver_max_iterations = 80
batch_solver_initial_lambda = 0.001

# Damped Gauss-Newton settings for each online closed-loop update.
closed_loop_solver_max_iterations = 25
closed_loop_solver_initial_lambda = 0.01

# Expanded robustness suite.
expanded_trials_per_case = 50
dropout_probability_max = 0.6
outlier_range_magnitude = 0.75
outlier_bearing_magnitude = 0.35
robust_huber_delta = 3.0
vehicle_pose_noise_max = 0.4
multistart_count = 6
sliding_window_size = 12

# Adaptive localization core simulation.
# This follows the rigorous global-frame equations directly:
# epsilon_i = (r_i^v)^2 - ||q - phat + r_i^t u_i||^2
# phat_dot = -2 Gamma sum_i epsilon_i (q - phat + r_i^t u_i)
# u = -k(q - phat)
adaptive_steps = 1000
adaptive_beacon_count = 2
adaptive_dt = 0.01
adaptive_gain = 0.1
adaptive_target_seeking_gain = 1.2
adaptive_initial_robot_x = -3.0
adaptive_initial_robot_y = 2.6
adaptive_initial_target_estimate_x = 0.0
adaptive_initial_target_estimate_y = 0.0

# Noise for the adaptive localization core. Keep zero for the clean theorem-style
# convergence demonstration, then increase for robustness figures.
adaptive_range_sigma = 0.0
adaptive_bearing_sigma = 0.0
)CFG";
}

}  // namespace adaptive
