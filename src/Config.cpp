/**
 * @file Config.cpp
 * @brief Loading and default-generation for the plain-text simulation config.
 *
 * This module parses the INI-style `config/simulation.ini` file into an
 * in-memory SimulationConfig struct (see Config.hpp), falling back to the
 * struct's compiled-in defaults whenever the file is missing or a given key
 * is absent, and silently ignoring any key it does not recognize. It also
 * writes a fresh, fully-commented default config file when none exists yet.
 */

#include "adaptive_localization/Config.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace adaptive {

namespace {

/**
 * @brief Removes leading and trailing whitespace from a string.
 *
 * Whitespace is determined by std::isspace (the argument is cast to
 * unsigned char first to avoid undefined behavior for negative char values
 * on platforms where char is signed). Used to clean up INI keys, values,
 * and comma-separated list items before they are stored or parsed as
 * numbers.
 *
 * @param text The string to trim, taken by value so it can be sliced in place.
 * @return A copy containing @p text with leading/trailing whitespace removed.
 *         If @p text is empty or entirely whitespace, an empty string is
 *         returned.
 */
std::string trim(std::string text) {
    // Scan forward from the start for the first non-whitespace character.
    const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char c) {
        return std::isspace(c) != 0;
    });
    // Scan backward from the end for the first non-whitespace character, then
    // convert the reverse iterator back to a forward one with .base() so it
    // can be used as the exclusive end of the trimmed range below.
    const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char c) {
        return std::isspace(c) != 0;
    }).base();
    if (first >= last) {
        // The string was empty or entirely whitespace.
        return {};
    }
    return {first, last};
}

/**
 * @brief Parses a comma-separated list of integers, e.g. "1, 2,3".
 *
 * The value is fed through a std::stringstream and split with
 * std::getline(..., ',') so each comma-delimited segment becomes one item.
 * Every item is trimmed of surrounding whitespace before conversion; an item
 * that trims to empty (e.g. a trailing comma, or "1,,2") is skipped rather
 * than being parsed as a value, so it does not produce a spurious 0 entry.
 *
 * @param value The raw config value string for a list-valued key.
 * @return The parsed integers in their original order; empty if @p value
 *         contains no non-empty items.
 * @throws std::invalid_argument or std::out_of_range if a non-empty item is
 *         not a valid integer (propagated from std::stoi; not caught here).
 */
std::vector<int> parse_int_list(const std::string& value) {
    std::vector<int> result;
    std::stringstream stream(value);
    std::string item;
    // Split on commas; each iteration handles one list item.
    while (std::getline(stream, item, ',')) {
        item = trim(item);
        if (!item.empty()) {
            result.push_back(std::stoi(item));
        }
    }
    return result;
}

/**
 * @brief Looks up a key in the parsed key/value map and parses it as a double.
 *
 * This is the workhorse used by load_config for every floating-point
 * SimulationConfig field: since the caller always passes the field's current
 * (default) value as @p fallback, a key that is absent from the file simply
 * leaves that field unchanged.
 *
 * @param values Parsed "key = value" pairs read from the config file.
 * @param key The config key to look up (e.g. "closed_loop_dt").
 * @param fallback Value returned when @p key is not present in @p values.
 * @return The parsed double, or @p fallback if the key was not found.
 * @throws std::invalid_argument or std::out_of_range if the stored string is
 *         not a valid double (propagated from std::stod; not caught here, so
 *         a malformed numeric value aborts load_config rather than being
 *         silently ignored).
 */
double get_double(
    const std::unordered_map<std::string, std::string>& values,
    const std::string& key,
    double fallback) {
    const auto it = values.find(key);
    return it == values.end() ? fallback : std::stod(it->second);
}

/**
 * @brief Looks up a key in the parsed key/value map and parses it as an int.
 *
 * Same missing-key/fallback behavior as get_double(), used for the integer
 * SimulationConfig fields (trial counts, step counts, iteration limits, ...).
 *
 * @param values Parsed "key = value" pairs read from the config file.
 * @param key The config key to look up.
 * @param fallback Value returned when @p key is not present in @p values.
 * @return The parsed int, or @p fallback if the key was not found.
 * @throws std::invalid_argument or std::out_of_range if the stored string is
 *         not a valid int (propagated from std::stoi; not caught here).
 */
int get_int(
    const std::unordered_map<std::string, std::string>& values,
    const std::string& key,
    int fallback) {
    const auto it = values.find(key);
    return it == values.end() ? fallback : std::stoi(it->second);
}

/**
 * @brief Looks up a key in the parsed key/value map and parses it as an
 *        unsigned int (used for the Monte Carlo / closed-loop random seeds).
 *
 * The value is parsed with std::stoul (unsigned long) and then narrowed with
 * static_cast to unsigned int, matching SimulationConfig::monte_carlo_seed
 * and SimulationConfig::closed_loop_seed.
 *
 * @param values Parsed "key = value" pairs read from the config file.
 * @param key The config key to look up.
 * @param fallback Value returned when @p key is not present in @p values.
 * @return The parsed unsigned int, or @p fallback if the key was not found.
 * @throws std::invalid_argument or std::out_of_range if the stored string is
 *         not a valid unsigned long (propagated from std::stoul; not caught
 *         here).
 */
unsigned int get_uint(
    const std::unordered_map<std::string, std::string>& values,
    const std::string& key,
    unsigned int fallback) {
    const auto it = values.find(key);
    return it == values.end() ? fallback : static_cast<unsigned int>(std::stoul(it->second));
}

/**
 * @brief Looks up a key and parses it as a comma-separated list of integers.
 *
 * Used for the beacon-count and scenario-ID lists. If the key is absent, or
 * present but parses to an empty list (e.g. a blank value or one made only
 * of commas), @p fallback is returned so a stray/blank override in the file
 * cannot silently clear out the default list.
 *
 * @param values Parsed "key = value" pairs read from the config file.
 * @param key The config key to look up.
 * @param fallback Value returned when @p key is absent or parses to empty.
 * @return The parsed list, or @p fallback.
 * @throws std::invalid_argument or std::out_of_range if a non-empty item in
 *         the value cannot be parsed as an integer (propagated from
 *         parse_int_list()/std::stoi).
 */
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

/**
 * @brief Serializes a list of integers back into "a,b,c" form.
 *
 * This is the inverse of parse_int_list()'s comma-separated format (no
 * spaces are inserted around the commas). Intended for logging/echoing the
 * effective beacon-count or scenario list that was loaded, not for writing
 * the config file itself (write_default_config_if_missing() embeds its list
 * defaults as literal text instead).
 *
 * @param values Integers to join, in order.
 * @return A comma-separated string; empty if @p values is empty.
 */
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

/**
 * @brief Loads simulation settings from a plain-text INI-style config file.
 *
 * Parsing pass: the file is read line by line. On each line, anything from
 * the first '#' onward is treated as a comment and stripped (so both
 * whole-line and trailing "key = value  # note" comments work); the
 * remaining text is then split on the first '=' into a key and a value, both
 * trimmed of surrounding whitespace. A line with no '=' at all — including
 * blank lines and lines that were pure comments — is skipped outright. A
 * line whose key or value trims to empty after stripping is likewise
 * discarded rather than stored, so "key =" (no value) or "= value" (no key)
 * lines have no effect. Surviving "key = value" pairs are collected into an
 * unordered_map keyed by the trimmed key, with later duplicate keys in the
 * file overwriting earlier ones.
 *
 * Dispatch pass: each recognized SimulationConfig field is then read out of
 * that map via get_double()/get_int()/get_uint()/get_int_list(), passing the
 * field's already-default-initialized value as the fallback. This means a
 * key that never appears in the file leaves the corresponding field at its
 * compiled-in default, and a key that appears in the file but is not one of
 * these recognized names is simply never looked up — it stays in the map,
 * unused, so unrecognized/experimental keys do not break parsing.
 *
 * Missing files are allowed: if the ifstream fails to open @p path (file
 * does not exist, unreadable, etc.), this function returns a
 * default-constructed SimulationConfig immediately, before the parsing pass
 * runs.
 *
 * @param path Path to the INI-style config file (typically config/simulation.ini).
 * @return A SimulationConfig with every recognized field overridden by the
 *         corresponding value found in the file, and left at its compiled-in
 *         default everywhere else (including when the file is missing).
 * @throws std::invalid_argument or std::out_of_range if a recognized
 *         numeric key's value cannot be parsed as the expected int/unsigned
 *         int/double (propagated from the get_*() helpers' std::stoi /
 *         std::stod / std::stoul calls; malformed numeric values are not
 *         caught and will abort loading rather than falling back silently).
 */
SimulationConfig load_config(const std::filesystem::path& path) {
    SimulationConfig config;
    std::ifstream in(path);
    if (!in) {
        // No file (or it could not be opened): return the compiled-in defaults.
        return config;
    }

    std::unordered_map<std::string, std::string> values;
    std::string line;
    while (std::getline(in, line)) {
        // Strip anything from the first '#' onward as a comment. This also
        // turns a whole-line comment into an empty string.
        const auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }
        // Lines without '=' (blank lines, lines that were pure comments,
        // or any other malformed line) carry no key/value pair; skip them.
        const auto equals_pos = line.find('=');
        if (equals_pos == std::string::npos) {
            continue;
        }
        // Split on the first '=' into key/value and trim whitespace from each.
        const std::string key = trim(line.substr(0, equals_pos));
        const std::string value = trim(line.substr(equals_pos + 1));
        // Only keep pairs where both sides are non-empty; e.g. "key =" or
        // "= value" contribute nothing.
        if (!key.empty() && !value.empty()) {
            values[key] = value;
        }
    }

    // Dispatch each recognized key into its SimulationConfig field. The
    // current (default) field value is always passed as the fallback, so an
    // absent key is a no-op. output_dir is a filesystem::path, so it is
    // assigned directly from the string rather than via a get_* helper.
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

/**
 * @brief Writes a fully-commented default config file if one does not
 *        already exist at @p path.
 *
 * If @p path already exists, this function returns immediately and does not
 * touch it (a hand-edited or previously generated config is never
 * overwritten). Otherwise it creates any missing parent directories and
 * writes a single large raw string literal (delimited with R"CFG(...)CFG" so
 * embedded '#', quotes, and parentheses in the file content need no
 * escaping) to a fresh file. That literal is the canonical default
 * `simulation.ini`: it contains, for every SimulationConfig field, one or
 * more explanatory '#' comment lines followed by a "key = value" line whose
 * value matches the field's compiled-in default in Config.hpp, using the
 * same comment syntax and comma-separated list syntax that load_config()
 * understands. This is the file a fresh checkout's config/simulation.ini is
 * generated from the first time the simulator runs.
 *
 * @param path Destination path for the default config file (typically
 *             config/simulation.ini).
 */
void write_default_config_if_missing(const std::filesystem::path& path) {
    if (std::filesystem::exists(path)) {
        // Never clobber an existing config file, even if it is stale.
        return;
    }
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream out(path);
    // The template below is emitted verbatim; it mirrors every field's
    // default from SimulationConfig (Config.hpp) with a human-readable
    // comment above it.
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
