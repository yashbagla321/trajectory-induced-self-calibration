#include "adaptive_localization/Simulation.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <tuple>
#include <utility>

#include "adaptive_localization/Estimators.hpp"
#include "adaptive_localization/Measurements.hpp"
#include "adaptive_localization/Solver.hpp"
#include "adaptive_localization/World.hpp"

namespace adaptive {

namespace {

struct CoreMeasurement {
    double rv = 0.0;
    double rt = 0.0;
    Vec2 u;
};

struct ObservabilityMetrics {
    int rank = 0;
    double sigma_min = 0.0;
    double sigma_max = 0.0;
    double condition_number = 0.0;
    double logdet = 0.0;
    double trajectory_spread = 0.0;
};

struct MeasurementStress {
    double dropout_probability = 0.0;
    double outlier_probability = 0.0;
    double outlier_range_magnitude = 0.0;
    double outlier_bearing_magnitude = 0.0;
    double fov_half_angle = 0.0;
    bool use_fov = false;
};

double sample_noise(double sigma, std::mt19937& rng) {
    if (sigma <= 0.0) {
        return 0.0;
    }
    std::normal_distribution<double> distribution(0.0, sigma);
    return distribution(rng);
}

std::vector<CoreMeasurement> measure_core_model(
    const World& world,
    const Vec2& robot,
    const Noise& noise,
    std::mt19937& rng) {
    std::vector<CoreMeasurement> measurements;
    measurements.reserve(world.beacons.size());

    for (const Vec2& beacon : world.beacons) {
        const double theta = bearing(beacon, world.target) + sample_noise(noise.bearing_sigma, rng);
        CoreMeasurement m;
        m.rv = std::max(0.05, norm(robot - beacon) + sample_noise(noise.range_sigma, rng));
        m.rt = std::max(0.05, norm(world.target - beacon) + sample_noise(noise.range_sigma, rng));
        m.u = unit_from_angle(theta);
        measurements.push_back(m);
    }
    return measurements;
}

double core_cost(
    const Vec2& robot,
    const Vec2& target_estimate,
    const std::vector<CoreMeasurement>& measurements) {
    double cost = 0.0;
    for (const auto& m : measurements) {
        const Vec2 v = robot - target_estimate + m.u * m.rt;
        const double eps = m.rv * m.rv - dot(v, v);
        cost += 0.5 * eps * eps;
    }
    return cost;
}

Vec2 core_adaptive_update(
    const Vec2& robot,
    const Vec2& target_estimate,
    const std::vector<CoreMeasurement>& measurements,
    double gamma) {
    Vec2 sum{0.0, 0.0};
    for (const auto& m : measurements) {
        const Vec2 v = robot - target_estimate + m.u * m.rt;
        const double eps = m.rv * m.rv - dot(v, v);
        sum = sum + v * eps;
    }
    return sum * (-2.0 * gamma);
}

std::vector<BeaconEstimate> core_beacon_estimates(
    const Vec2& target_estimate,
    const std::vector<CoreMeasurement>& measurements) {
    std::vector<BeaconEstimate> estimates;
    estimates.reserve(measurements.size());
    for (const auto& m : measurements) {
        estimates.push_back({target_estimate - m.u * m.rt, 0.0});
    }
    return estimates;
}

double beacon_position_rmse(const World& world, const std::vector<BeaconEstimate>& estimates) {
    if (estimates.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (std::size_t i = 0; i < estimates.size(); ++i) {
        const Vec2 error = estimates[i].position - world.beacons[i];
        sum += dot(error, error);
    }
    return std::sqrt(sum / static_cast<double>(estimates.size()));
}

double beacon_yaw_rmse(const World& world, const std::vector<BeaconEstimate>& estimates) {
    if (estimates.empty()) {
        return -1.0;
    }
    double sum = 0.0;
    for (std::size_t i = 0; i < estimates.size(); ++i) {
        const double error = wrap_angle(estimates[i].yaw - world.beacon_yaws[i]);
        sum += error * error;
    }
    return std::sqrt(sum / static_cast<double>(estimates.size()));
}

bool trial_accuracy_success(
    double target_error,
    double beacon_position_error,
    double beacon_yaw_error) {
    return target_error < 0.05 &&
        beacon_position_error < 0.05 &&
        (beacon_yaw_error < 0.0 || beacon_yaw_error < 0.05);
}

std::vector<double> true_state_scenario1(const World& world) {
    std::vector<double> state{world.target.x, world.target.y};
    for (std::size_t i = 0; i < world.beacons.size(); ++i) {
        state.push_back(world.beacons[i].x);
        state.push_back(world.beacons[i].y);
        state.push_back(world.beacon_yaws[i]);
    }
    return state;
}

std::array<double, 25> normal_matrix_for_local_observability(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements) {
    constexpr int n = 5;
    const auto jacobian = jacobian_scenario1(state, 1, path, measurements);
    std::array<double, 25> normal{};

    for (int row = 0; row < n; ++row) {
        for (int col = row; col < n; ++col) {
            double value = 0.0;
            for (const auto& jacobian_row : jacobian) {
                value += jacobian_row[static_cast<std::size_t>(row)] *
                    jacobian_row[static_cast<std::size_t>(col)];
            }
            normal[static_cast<std::size_t>(row * n + col)] = value;
            normal[static_cast<std::size_t>(col * n + row)] = value;
        }
    }
    return normal;
}

double trajectory_spread(const std::vector<LocalFrameMeasurement>& measurements, int beacon_count) {
    if (measurements.empty() || beacon_count <= 0) {
        return 0.0;
    }
    std::vector<Vec2> sums(static_cast<std::size_t>(beacon_count));
    std::vector<int> counts(static_cast<std::size_t>(beacon_count), 0);
    for (const auto& measurement : measurements) {
        if (measurement.beacon >= sums.size()) {
            continue;
        }
        const Vec2 local_vehicle =
            unit_from_angle(measurement.bv_local) * measurement.rv;
        sums[measurement.beacon] = sums[measurement.beacon] + local_vehicle;
        counts[measurement.beacon] += 1;
    }
    std::vector<Vec2> means(static_cast<std::size_t>(beacon_count));
    for (int i = 0; i < beacon_count; ++i) {
        if (counts[static_cast<std::size_t>(i)] > 0) {
            means[static_cast<std::size_t>(i)] =
                sums[static_cast<std::size_t>(i)] / static_cast<double>(counts[static_cast<std::size_t>(i)]);
        }
    }
    double spread = 0.0;
    for (const auto& measurement : measurements) {
        if (measurement.beacon >= means.size()) {
            continue;
        }
        const Vec2 local_vehicle =
            unit_from_angle(measurement.bv_local) * measurement.rv;
        const Vec2 centered = local_vehicle - means[measurement.beacon];
        spread += dot(centered, centered);
    }
    return spread;
}

std::array<double, 5> jacobi_eigenvalues(std::array<double, 25> a) {
    constexpr int n = 5;
    for (int sweep = 0; sweep < 80; ++sweep) {
        int p = 0;
        int q = 1;
        double max_offdiag = 0.0;
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                const double value = std::abs(a[static_cast<std::size_t>(i * n + j)]);
                if (value > max_offdiag) {
                    max_offdiag = value;
                    p = i;
                    q = j;
                }
            }
        }
        if (max_offdiag < 1e-10) {
            break;
        }

        const double app = a[static_cast<std::size_t>(p * n + p)];
        const double aqq = a[static_cast<std::size_t>(q * n + q)];
        const double apq = a[static_cast<std::size_t>(p * n + q)];
        const double angle = 0.5 * std::atan2(2.0 * apq, aqq - app);
        const double c = std::cos(angle);
        const double s = std::sin(angle);

        for (int k = 0; k < n; ++k) {
            if (k == p || k == q) {
                continue;
            }
            const double akp = a[static_cast<std::size_t>(k * n + p)];
            const double akq = a[static_cast<std::size_t>(k * n + q)];
            const double new_kp = c * akp - s * akq;
            const double new_kq = s * akp + c * akq;
            a[static_cast<std::size_t>(k * n + p)] = new_kp;
            a[static_cast<std::size_t>(p * n + k)] = new_kp;
            a[static_cast<std::size_t>(k * n + q)] = new_kq;
            a[static_cast<std::size_t>(q * n + k)] = new_kq;
        }

        a[static_cast<std::size_t>(p * n + p)] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a[static_cast<std::size_t>(q * n + q)] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a[static_cast<std::size_t>(p * n + q)] = 0.0;
        a[static_cast<std::size_t>(q * n + p)] = 0.0;
    }

    std::array<double, 5> values{};
    for (int i = 0; i < n; ++i) {
        values[static_cast<std::size_t>(i)] =
            std::max(0.0, a[static_cast<std::size_t>(i * n + i)]);
    }
    std::sort(values.begin(), values.end());
    return values;
}

std::pair<int, double> local_observability_rank_and_sigma_min(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements) {
    const auto metrics = [&]() {
        const auto eigenvalues = jacobi_eigenvalues(
            normal_matrix_for_local_observability(state, path, measurements));
        const double largest = std::sqrt(eigenvalues.back());
        const double threshold = std::max(1e-6, largest * 1e-7);
        int rank = 0;
        double smallest_positive = 0.0;
        for (double eigenvalue : eigenvalues) {
            const double singular_value = std::sqrt(std::max(0.0, eigenvalue));
            if (singular_value > threshold) {
                if (rank == 0) {
                    smallest_positive = singular_value;
                }
                ++rank;
            }
        }
        return std::make_pair(rank, rank == 5 ? smallest_positive : 0.0);
    }();
    return metrics;
}

ObservabilityMetrics local_observability_metrics(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements) {
    const auto eigenvalues = jacobi_eigenvalues(
        normal_matrix_for_local_observability(state, path, measurements));
    const double largest = std::sqrt(eigenvalues.back());
    const double threshold = std::max(1e-6, largest * 1e-7);
    ObservabilityMetrics metrics;
    metrics.sigma_max = largest;
    metrics.logdet = 0.0;
    metrics.trajectory_spread = trajectory_spread(measurements, 1);
    double smallest_positive = 0.0;
    for (double eigenvalue : eigenvalues) {
        const double singular_value = std::sqrt(std::max(0.0, eigenvalue));
        metrics.logdet += std::log(std::max(eigenvalue, 1e-18));
        if (singular_value > threshold) {
            if (metrics.rank == 0) {
                smallest_positive = singular_value;
            }
            ++metrics.rank;
        }
    }
    metrics.sigma_min = metrics.rank == 5 ? smallest_positive : 0.0;
    metrics.condition_number =
        metrics.sigma_min > 0.0 ? metrics.sigma_max / metrics.sigma_min : 1e12;
    return metrics;
}

LocalFrameMeasurement predicted_local_frame_measurement(
    const Vec2& robot,
    const Vec2& target_estimate,
    const BeaconEstimate& beacon_estimate,
    std::size_t beacon_index,
    std::size_t time_index) {
    LocalFrameMeasurement measurement;
    measurement.beacon = beacon_index;
    measurement.time = time_index;
    measurement.rv = std::max(0.05, norm(robot - beacon_estimate.position));
    measurement.bv_local = wrap_angle(
        bearing(beacon_estimate.position, robot) - beacon_estimate.yaw);
    measurement.rt = std::max(0.05, norm(target_estimate - beacon_estimate.position));
    measurement.bt_local = wrap_angle(
        bearing(beacon_estimate.position, target_estimate) - beacon_estimate.yaw);
    return measurement;
}

double predicted_local_logdet_score(
    const Vec2& candidate_robot,
    const Vec2& target_estimate,
    const std::vector<double>& state,
    const std::vector<BeaconEstimate>& beacon_estimates,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements) {
    if (state.size() != 5 || beacon_estimates.size() != 1) {
        return -1e18;
    }

    std::vector<Vec2> candidate_path = path;
    candidate_path.push_back(candidate_robot);

    std::vector<LocalFrameMeasurement> candidate_measurements = measurements;
    candidate_measurements.push_back(predicted_local_frame_measurement(
        candidate_robot,
        target_estimate,
        beacon_estimates.front(),
        0,
        candidate_path.size() - 1U));

    return local_observability_metrics(state, candidate_path, candidate_measurements).logdet;
}

Vec2 information_driven_excitation(
    const Vec2& robot,
    const Vec2& target_estimate,
    const std::vector<double>& state,
    const std::vector<BeaconEstimate>& beacon_estimates,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const SimulationConfig& config,
    int step) {
    if (path.empty() || state.size() != 5 || beacon_estimates.size() != 1) {
        return {0.0, 0.0};
    }

    const double h = std::max(1e-4, config.information_gradient_step);
    const double sx_plus = predicted_local_logdet_score(
        {robot.x + h, robot.y}, target_estimate, state, beacon_estimates, path, measurements);
    const double sx_minus = predicted_local_logdet_score(
        {robot.x - h, robot.y}, target_estimate, state, beacon_estimates, path, measurements);
    const double sy_plus = predicted_local_logdet_score(
        {robot.x, robot.y + h}, target_estimate, state, beacon_estimates, path, measurements);
    const double sy_minus = predicted_local_logdet_score(
        {robot.x, robot.y - h}, target_estimate, state, beacon_estimates, path, measurements);

    Vec2 gradient{
        (sx_plus - sx_minus) / (2.0 * h),
        (sy_plus - sy_minus) / (2.0 * h),
    };
    const double gradient_norm = norm(gradient);
    if (!std::isfinite(gradient_norm) || gradient_norm < 1e-9) {
        return {0.0, 0.0};
    }

    const double envelope =
        config.exploration_amplitude *
        std::exp(-config.exploration_decay * static_cast<double>(step - 1));
    const double magnitude = std::min(
        envelope,
        config.information_exploration_gain * gradient_norm);
    return gradient * (magnitude / gradient_norm);
}

std::vector<double> local_frame_measurement_prediction(
    const std::vector<double>& state,
    const Vec2& robot,
    const LocalFrameMeasurement& measurement) {
    const std::size_t base = 2 + 3 * measurement.beacon;
    const Vec2 target{state[0], state[1]};
    const Vec2 beacon{state[base], state[base + 1]};
    const double yaw = state[base + 2];

    const Vec2 vehicle_local = rotate(robot - beacon, -yaw);
    const Vec2 target_local = rotate(target - beacon, -yaw);
    return {
        norm(vehicle_local),
        std::atan2(vehicle_local.y, vehicle_local.x),
        norm(target_local),
        std::atan2(target_local.y, target_local.x),
    };
}

std::vector<double> local_frame_measurement_vector(const LocalFrameMeasurement& measurement) {
    return {
        measurement.rv,
        measurement.bv_local,
        measurement.rt,
        measurement.bt_local,
    };
}

std::vector<LocalFrameMeasurement> generate_stressed_local_measurements(
    const World& world,
    const std::vector<Vec2>& path,
    const Noise& noise,
    const MeasurementStress& stress,
    std::mt19937& rng) {
    std::vector<LocalFrameMeasurement> measurements;
    measurements.reserve(path.size() * world.beacons.size());
    std::bernoulli_distribution dropout(stress.dropout_probability);
    std::bernoulli_distribution outlier(stress.outlier_probability);
    std::bernoulli_distribution sign(0.5);

    for (std::size_t time = 0; time < path.size(); ++time) {
        for (std::size_t beacon = 0; beacon < world.beacons.size(); ++beacon) {
            auto measurement = make_local_frame_measurement(
                world, path[time], beacon, time, noise, rng);

            if (stress.use_fov &&
                (std::abs(wrap_angle(measurement.bv_local)) > stress.fov_half_angle ||
                 std::abs(wrap_angle(measurement.bt_local)) > stress.fov_half_angle)) {
                continue;
            }
            if (dropout(rng)) {
                continue;
            }
            if (outlier(rng)) {
                const double range_sign = sign(rng) ? 1.0 : -1.0;
                const double bearing_sign = sign(rng) ? 1.0 : -1.0;
                measurement.rv = std::max(0.05, measurement.rv + range_sign * stress.outlier_range_magnitude);
                measurement.rt = std::max(0.05, measurement.rt - range_sign * stress.outlier_range_magnitude);
                measurement.bv_local =
                    wrap_angle(measurement.bv_local + bearing_sign * stress.outlier_bearing_magnitude);
                measurement.bt_local =
                    wrap_angle(measurement.bt_local - bearing_sign * stress.outlier_bearing_magnitude);
            }
            measurements.push_back(measurement);
        }
    }
    return measurements;
}

std::vector<Vec2> make_noisy_path(
    const std::vector<Vec2>& path,
    double position_sigma,
    std::mt19937& rng) {
    if (position_sigma <= 0.0) {
        return path;
    }
    std::normal_distribution<double> noise(0.0, position_sigma);
    std::vector<Vec2> noisy_path;
    noisy_path.reserve(path.size());
    for (const Vec2& p : path) {
        noisy_path.push_back({p.x + noise(rng), p.y + noise(rng)});
    }
    return noisy_path;
}

std::vector<Vec2> make_drifted_path(
    const std::vector<Vec2>& path,
    double drift_fraction,
    std::mt19937& rng) {
    if (drift_fraction <= 0.0 || path.empty()) {
        return path;
    }
    std::normal_distribution<double> unit_noise(0.0, 1.0);
    std::vector<Vec2> drifted_path;
    drifted_path.reserve(path.size());
    Vec2 drift{0.0, 0.0};
    drifted_path.push_back(path.front());
    for (std::size_t k = 1; k < path.size(); ++k) {
        const double step_length = norm(path[k] - path[k - 1U]);
        const double sigma = drift_fraction * step_length;
        drift = drift + Vec2{sigma * unit_noise(rng), sigma * unit_noise(rng)};
        drifted_path.push_back(path[k] + drift);
    }
    return drifted_path;
}

std::vector<double> huber_scaled_residuals(
    std::vector<double> residuals,
    double delta) {
    if (delta <= 0.0) {
        return residuals;
    }
    for (double& residual : residuals) {
        const double magnitude = std::abs(residual);
        if (magnitude > delta) {
            residual = std::copysign(std::sqrt(delta * magnitude), residual);
        }
    }
    return residuals;
}

std::vector<std::vector<double>> identity_matrix(int n, double scale = 1.0) {
    std::vector<std::vector<double>> matrix(
        static_cast<std::size_t>(n), std::vector<double>(static_cast<std::size_t>(n), 0.0));
    for (int i = 0; i < n; ++i) {
        matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] = scale;
    }
    return matrix;
}

std::vector<double> solve_dense_system(
    std::vector<std::vector<double>> a,
    std::vector<double> b) {
    const int n = static_cast<int>(b.size());
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        double pivot_value = std::abs(a[static_cast<std::size_t>(col)][static_cast<std::size_t>(col)]);
        for (int row = col + 1; row < n; ++row) {
            const double value = std::abs(a[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)]);
            if (value > pivot_value) {
                pivot = row;
                pivot_value = value;
            }
        }
        if (pivot_value < 1e-12) {
            return std::vector<double>(static_cast<std::size_t>(n), 0.0);
        }
        if (pivot != col) {
            std::swap(a[static_cast<std::size_t>(pivot)], a[static_cast<std::size_t>(col)]);
            std::swap(b[static_cast<std::size_t>(pivot)], b[static_cast<std::size_t>(col)]);
        }

        const double diagonal = a[static_cast<std::size_t>(col)][static_cast<std::size_t>(col)];
        for (int j = col; j < n; ++j) {
            a[static_cast<std::size_t>(col)][static_cast<std::size_t>(j)] /= diagonal;
        }
        b[static_cast<std::size_t>(col)] /= diagonal;

        for (int row = 0; row < n; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = a[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
            for (int j = col; j < n; ++j) {
                a[static_cast<std::size_t>(row)][static_cast<std::size_t>(j)] -=
                    factor * a[static_cast<std::size_t>(col)][static_cast<std::size_t>(j)];
            }
            b[static_cast<std::size_t>(row)] -= factor * b[static_cast<std::size_t>(col)];
        }
    }
    return b;
}

std::vector<std::vector<double>> covariance_from_local_information(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise) {
    const auto jacobian = jacobian_scenario1(state, 1, path, measurements, noise);
    const int state_dim = static_cast<int>(state.size());
    std::vector<std::vector<double>> normal(
        static_cast<std::size_t>(state_dim),
        std::vector<double>(static_cast<std::size_t>(state_dim), 0.0));
    for (const auto& row : jacobian) {
        for (int i = 0; i < state_dim; ++i) {
            for (int j = 0; j < state_dim; ++j) {
                normal[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] +=
                    row[static_cast<std::size_t>(i)] * row[static_cast<std::size_t>(j)];
            }
        }
    }
    for (int i = 0; i < state_dim; ++i) {
        normal[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] += 1e-6;
    }

    std::vector<std::vector<double>> covariance(
        static_cast<std::size_t>(state_dim),
        std::vector<double>(static_cast<std::size_t>(state_dim), 0.0));
    for (int col = 0; col < state_dim; ++col) {
        std::vector<double> rhs(static_cast<std::size_t>(state_dim), 0.0);
        rhs[static_cast<std::size_t>(col)] = 1.0;
        const auto solution = solve_dense_system(normal, rhs);
        for (int row = 0; row < state_dim; ++row) {
            covariance[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
                solution[static_cast<std::size_t>(row)];
        }
    }
    return covariance;
}

std::vector<std::vector<double>> finite_difference_local_measurement_jacobian(
    const std::vector<double>& state,
    const Vec2& robot,
    const LocalFrameMeasurement& measurement) {
    const int measurement_dim = 4;
    const int state_dim = static_cast<int>(state.size());
    const double h = 1e-6;
    std::vector<std::vector<double>> jacobian(
        static_cast<std::size_t>(measurement_dim),
        std::vector<double>(static_cast<std::size_t>(state_dim), 0.0));

    for (int col = 0; col < state_dim; ++col) {
        std::vector<double> plus = state;
        std::vector<double> minus = state;
        plus[static_cast<std::size_t>(col)] += h;
        minus[static_cast<std::size_t>(col)] -= h;
        const auto z_plus = local_frame_measurement_prediction(plus, robot, measurement);
        const auto z_minus = local_frame_measurement_prediction(minus, robot, measurement);
        for (int row = 0; row < measurement_dim; ++row) {
            double diff = z_plus[static_cast<std::size_t>(row)] - z_minus[static_cast<std::size_t>(row)];
            if (row == 1 || row == 3) {
                diff = wrap_angle(diff);
            }
            jacobian[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] = diff / (2.0 * h);
        }
    }
    return jacobian;
}

TrialResult run_ekf_local_frame_trial(
    int beacon_count,
    int trial,
    const World& world,
    const std::vector<Vec2>& path,
    const SimulationConfig& config,
    const Noise& noise,
    bool use_two_view_initialization,
    std::mt19937& rng) {
    const int scenario = use_two_view_initialization ? 4 : 3;
    const auto measurements = generate_local_frame_measurements(world, path, noise, rng);
    const auto start = std::chrono::steady_clock::now();

    std::vector<double> state = initial_state_scenario1(
        beacon_count,
        config.initial_target_estimate,
        config.initial_beacon_guess_radius,
        config.initial_beacon_guess_yaw);
    const int state_dim = static_cast<int>(state.size());
    std::vector<std::vector<double>> covariance = identity_matrix(state_dim);
    for (int i = 0; i < state_dim; ++i) {
        const bool is_yaw = i >= 4 && ((i - 4) % 3 == 0);
        covariance[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
            is_yaw ? 9.0 : 25.0;
    }
    if (use_two_view_initialization && beacon_count == 1) {
        std::vector<double> closed_form_state;
        if (two_view_closed_form_initial_state(beacon_count, path, measurements, closed_form_state)) {
            state = closed_form_state;
            covariance = covariance_from_local_information(state, path, measurements, noise);
            for (int i = 0; i < state_dim; ++i) {
                covariance[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
                    std::max(covariance[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)], 1e-4);
            }
        }
    }

    const double range_variance =
        std::pow(noise.range_sigma > 0.0 ? noise.range_sigma : 0.03, 2.0);
    const double bearing_variance =
        std::pow(noise.bearing_sigma > 0.0 ? noise.bearing_sigma : 0.006, 2.0);
    const std::array<double, 4> measurement_variance{
        range_variance, bearing_variance, range_variance, bearing_variance};

    for (const auto& measurement : measurements) {
        for (int i = 0; i < state_dim; ++i) {
            covariance[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] += 1e-5;
        }

        const Vec2& robot = path[measurement.time];
        const auto predicted = local_frame_measurement_prediction(state, robot, measurement);
        const auto observed = local_frame_measurement_vector(measurement);
        std::array<double, 4> innovation{};
        for (int row = 0; row < 4; ++row) {
            innovation[static_cast<std::size_t>(row)] =
                observed[static_cast<std::size_t>(row)] - predicted[static_cast<std::size_t>(row)];
            if (row == 1 || row == 3) {
                innovation[static_cast<std::size_t>(row)] =
                    wrap_angle(innovation[static_cast<std::size_t>(row)]);
            }
        }

        const auto h_matrix = finite_difference_local_measurement_jacobian(state, robot, measurement);
        std::vector<std::vector<double>> innovation_covariance(
            4, std::vector<double>(4, 0.0));
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                double value = 0.0;
                for (int i = 0; i < state_dim; ++i) {
                    for (int j = 0; j < state_dim; ++j) {
                        value += h_matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(i)] *
                            covariance[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                            h_matrix[static_cast<std::size_t>(col)][static_cast<std::size_t>(j)];
                    }
                }
                innovation_covariance[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] = value;
            }
            innovation_covariance[static_cast<std::size_t>(row)][static_cast<std::size_t>(row)] +=
                measurement_variance[static_cast<std::size_t>(row)];
        }

        std::vector<std::vector<double>> kalman_gain(
            static_cast<std::size_t>(state_dim), std::vector<double>(4, 0.0));
        for (int i = 0; i < state_dim; ++i) {
            std::vector<double> p_ht(4, 0.0);
            for (int row = 0; row < 4; ++row) {
                for (int j = 0; j < state_dim; ++j) {
                    p_ht[static_cast<std::size_t>(row)] +=
                        covariance[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                        h_matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(j)];
                }
            }
            for (int row = 0; row < 4; ++row) {
                std::vector<double> basis(4, 0.0);
                basis[static_cast<std::size_t>(row)] = 1.0;
                const auto column = solve_dense_system(innovation_covariance, basis);
                for (int k = 0; k < 4; ++k) {
                    kalman_gain[static_cast<std::size_t>(i)][static_cast<std::size_t>(row)] +=
                        p_ht[static_cast<std::size_t>(k)] * column[static_cast<std::size_t>(k)];
                }
            }
        }

        for (int i = 0; i < state_dim; ++i) {
            double dx = 0.0;
            for (int row = 0; row < 4; ++row) {
                dx += kalman_gain[static_cast<std::size_t>(i)][static_cast<std::size_t>(row)] *
                    innovation[static_cast<std::size_t>(row)];
            }
            state[static_cast<std::size_t>(i)] += dx;
        }

        std::vector<std::vector<double>> kh(
            static_cast<std::size_t>(state_dim),
            std::vector<double>(static_cast<std::size_t>(state_dim), 0.0));
        for (int i = 0; i < state_dim; ++i) {
            for (int j = 0; j < state_dim; ++j) {
                for (int row = 0; row < 4; ++row) {
                    kh[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] +=
                        kalman_gain[static_cast<std::size_t>(i)][static_cast<std::size_t>(row)] *
                        h_matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(j)];
                }
            }
        }

        std::vector<std::vector<double>> identity_minus_kh =
            identity_matrix(state_dim);
        for (int i = 0; i < state_dim; ++i) {
            for (int j = 0; j < state_dim; ++j) {
                identity_minus_kh[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] -=
                    kh[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
            }
        }

        std::vector<std::vector<double>> left_product(
            static_cast<std::size_t>(state_dim),
            std::vector<double>(static_cast<std::size_t>(state_dim), 0.0));
        for (int i = 0; i < state_dim; ++i) {
            for (int j = 0; j < state_dim; ++j) {
                for (int k = 0; k < state_dim; ++k) {
                    left_product[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] +=
                        identity_minus_kh[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)] *
                        covariance[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)];
                }
            }
        }

        std::vector<std::vector<double>> updated(
            static_cast<std::size_t>(state_dim),
            std::vector<double>(static_cast<std::size_t>(state_dim), 0.0));
        for (int i = 0; i < state_dim; ++i) {
            for (int j = 0; j < state_dim; ++j) {
                for (int k = 0; k < state_dim; ++k) {
                    updated[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] +=
                        left_product[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)] *
                        identity_minus_kh[static_cast<std::size_t>(j)][static_cast<std::size_t>(k)];
                }
                for (int row = 0; row < 4; ++row) {
                    updated[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] +=
                        kalman_gain[static_cast<std::size_t>(i)][static_cast<std::size_t>(row)] *
                        measurement_variance[static_cast<std::size_t>(row)] *
                        kalman_gain[static_cast<std::size_t>(j)][static_cast<std::size_t>(row)];
                }
            }
        }
        covariance = updated;
    }

    const auto stop = std::chrono::steady_clock::now();
    const Vec2 estimate{state[0], state[1]};
    const auto beacon_estimates = beacon_estimates_from_scenario1_state(state, beacon_count);
    const auto final_residuals = residuals_scenario1(state, beacon_count, path, measurements, noise);
    double cost = 0.0;
    for (double residual : final_residuals) {
        cost += 0.5 * residual * residual;
    }

    const double target_error = norm(estimate - world.target);
    const double beacon_position_error = beacon_position_rmse(world, beacon_estimates);
    const double beacon_yaw_error = beacon_yaw_rmse(world, beacon_estimates);
    const double runtime_ms = std::chrono::duration<double, std::milli>(stop - start).count();
    return {scenario, beacon_count, trial, world.target, estimate, target_error,
            beacon_position_error, beacon_yaw_error,
            cost, static_cast<int>(measurements.size()), runtime_ms, true,
            trial_accuracy_success(target_error, beacon_position_error, beacon_yaw_error)};
}

double elapsed_ms(
    const std::chrono::steady_clock::time_point& start,
    const std::chrono::steady_clock::time_point& stop) {
    return std::chrono::duration<double, std::milli>(stop - start).count();
}

double mean_ci95_from_sums(double sum, double sum2, int count) {
    if (count < 2) {
        return 0.0;
    }
    const double mean = sum / static_cast<double>(count);
    const double variance =
        std::max(0.0, (sum2 - static_cast<double>(count) * mean * mean) /
                          static_cast<double>(count - 1));
    return 1.96 * std::sqrt(variance / static_cast<double>(count));
}

double rmse_ci95_from_sums(double sum_error2, double sum_error4, int count) {
    if (count < 2 || sum_error2 <= 0.0) {
        return 0.0;
    }
    const double mean_square = sum_error2 / static_cast<double>(count);
    const double variance_square =
        std::max(0.0, (sum_error4 - static_cast<double>(count) * mean_square * mean_square) /
                          static_cast<double>(count - 1));
    const double rmse = std::sqrt(mean_square);
    const double se_mean_square = std::sqrt(variance_square / static_cast<double>(count));
    return 1.96 * se_mean_square / (2.0 * rmse);
}

TrialResult run_local_batch_trial_with_measurements(
    int trial,
    int beacon_count,
    const World& world,
    const std::vector<Vec2>& estimator_path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const SimulationConfig& config,
    const Noise& residual_noise,
    const std::vector<double>& initial_state,
    bool robust,
    double robust_delta,
    bool use_closed_form_seed = true,
    bool repeat_target_packets = true) {
    const auto start = std::chrono::steady_clock::now();
    std::vector<double> seed = initial_state;
    if (use_closed_form_seed) {
        std::vector<double> closed_form_seed;
        if (two_view_closed_form_initial_state(beacon_count, estimator_path, measurements, closed_form_seed)) {
            seed = closed_form_seed;
        }
    }
    int warm_start_iterations = 0;
    if (robust) {
        const auto warm_start = gauss_newton(
            seed,
            [&](const std::vector<double>& state) {
                return residuals_scenario1(
                    state, beacon_count, estimator_path, measurements, residual_noise, repeat_target_packets);
            },
            std::max(10, config.batch_solver_max_iterations / 2),
            config.batch_solver_initial_lambda,
            [&](const std::vector<double>& state) {
                return jacobian_scenario1(
                    state, beacon_count, estimator_path, measurements, residual_noise, repeat_target_packets);
            });
        seed = warm_start.x;
        warm_start_iterations = warm_start.iterations;
    }
    const auto result = gauss_newton(
        seed,
        [&](const std::vector<double>& state) {
            auto residuals = residuals_scenario1(
                state, beacon_count, estimator_path, measurements, residual_noise, repeat_target_packets);
            return robust ? huber_scaled_residuals(std::move(residuals), robust_delta) : residuals;
        },
        config.batch_solver_max_iterations,
        config.batch_solver_initial_lambda,
        robust ? JacobianFunction() :
            JacobianFunction([&](const std::vector<double>& state) {
                return jacobian_scenario1(
                    state, beacon_count, estimator_path, measurements, residual_noise, repeat_target_packets);
            }));
    const auto stop = std::chrono::steady_clock::now();
    const Vec2 estimate{result.x[0], result.x[1]};
    const auto beacon_estimates = beacon_estimates_from_scenario1_state(result.x, beacon_count);
    const double target_error = norm(estimate - world.target);
    const double beacon_position_error = beacon_position_rmse(world, beacon_estimates);
    const double beacon_yaw_error = beacon_yaw_rmse(world, beacon_estimates);
    return {1, beacon_count, trial, world.target, estimate, target_error,
            beacon_position_error, beacon_yaw_error,
            result.cost, result.iterations + warm_start_iterations, elapsed_ms(start, stop),
            result.converged, trial_accuracy_success(target_error, beacon_position_error, beacon_yaw_error)};
}

TrialResult run_local_batch_trial_with_seed(
    int trial,
    int beacon_count,
    const World& world,
    const std::vector<Vec2>& true_path,
    const std::vector<Vec2>& estimator_path,
    const SimulationConfig& config,
    const Noise& noise,
    const MeasurementStress& stress,
    const Vec2& target_seed,
    double beacon_seed_radius,
    double beacon_yaw_seed,
    bool robust,
    std::mt19937& rng) {
    const auto measurements = generate_stressed_local_measurements(world, true_path, noise, stress, rng);
    const auto initial_state =
        initial_state_scenario1(beacon_count, target_seed, beacon_seed_radius, beacon_yaw_seed);
    return run_local_batch_trial_with_measurements(
        trial,
        beacon_count,
        world,
        estimator_path,
        measurements,
        config,
        noise,
        initial_state,
        robust,
        config.robust_huber_delta);
}

TrialResult run_multistart_local_batch_trial(
    int trial,
    int beacon_count,
    const World& world,
    const std::vector<Vec2>& true_path,
    const std::vector<Vec2>& estimator_path,
    const SimulationConfig& config,
    const Noise& noise,
    const MeasurementStress& stress,
    const Vec2& target_seed,
    double beacon_seed_radius,
    double beacon_yaw_seed,
    int multistarts,
    bool robust,
    std::mt19937& rng) {
    const auto measurements = generate_stressed_local_measurements(world, true_path, noise, stress, rng);
    TrialResult best;
    best.cost = std::numeric_limits<double>::infinity();
    const int starts = std::max(1, multistarts);
    const auto multistart_start = std::chrono::steady_clock::now();
    for (int start_index = 0; start_index < starts; ++start_index) {
        const double angle = 2.0 * kPi * static_cast<double>(start_index) / static_cast<double>(starts);
        const Vec2 shifted_target{
            start_index == 0 ? target_seed.x : target_seed.x + 0.4 * beacon_seed_radius * std::cos(angle),
            start_index == 0 ? target_seed.y : target_seed.y + 0.4 * beacon_seed_radius * std::sin(angle),
        };
        const double radius_scale =
            start_index == 0 ? 1.0 : 0.7 + 0.6 * static_cast<double>((start_index % 3)) / 2.0;
        const auto initial_state = initial_state_scenario1(
            beacon_count,
            shifted_target,
            beacon_seed_radius * radius_scale,
            start_index == 0 ? beacon_yaw_seed : beacon_yaw_seed + angle);
        auto result = run_local_batch_trial_with_measurements(
            trial,
            beacon_count,
            world,
            estimator_path,
            measurements,
            config,
            noise,
            initial_state,
            robust,
            config.robust_huber_delta);
        if (result.cost < best.cost) {
            best = result;
        }
    }
    const auto multistart_stop = std::chrono::steady_clock::now();
    best.runtime_ms = elapsed_ms(multistart_start, multistart_stop);
    return best;
}

std::vector<LocalFrameMeasurement> recent_measurement_window(
    const std::vector<LocalFrameMeasurement>& measurements,
    int window_size) {
    if (measurements.empty() || window_size <= 0) {
        return measurements;
    }
    const std::size_t last_time = measurements.back().time;
    const std::size_t first_time =
        last_time > static_cast<std::size_t>(window_size) ?
            last_time - static_cast<std::size_t>(window_size) + 1U :
            0U;
    std::vector<LocalFrameMeasurement> window;
    for (const auto& measurement : measurements) {
        if (measurement.time >= first_time) {
            window.push_back(measurement);
        }
    }
    return window;
}

TrialResult run_trial_with_world_path(
    int scenario,
    int beacon_count,
    int trial,
    const World& world,
    const std::vector<Vec2>& path,
    const SimulationConfig& config,
    const Noise& noise,
    std::mt19937& rng) {
    if (scenario == 3) {
        return run_ekf_local_frame_trial(beacon_count, trial, world, path, config, noise, false, rng);
    }
    if (scenario == 4) {
        return run_ekf_local_frame_trial(beacon_count, trial, world, path, config, noise, true, rng);
    }

    if (scenario == 1) {
        const auto measurements = generate_local_frame_measurements(world, path, noise, rng);
        const auto start = std::chrono::steady_clock::now();
        auto initial_state = initial_state_scenario1(
            beacon_count,
            config.initial_target_estimate,
            config.initial_beacon_guess_radius,
            config.initial_beacon_guess_yaw);
        std::vector<double> closed_form_seed;
        if (two_view_closed_form_initial_state(beacon_count, path, measurements, closed_form_seed)) {
            initial_state = closed_form_seed;
        }
        const auto result = gauss_newton(
            initial_state,
            [&](const std::vector<double>& state) {
                return residuals_scenario1(state, beacon_count, path, measurements, noise);
            },
            config.batch_solver_max_iterations,
            config.batch_solver_initial_lambda,
            [&](const std::vector<double>& state) {
                return jacobian_scenario1(state, beacon_count, path, measurements, noise);
            });
        const auto stop = std::chrono::steady_clock::now();
        const Vec2 estimate{result.x[0], result.x[1]};
        const auto beacon_estimates = beacon_estimates_from_scenario1_state(result.x, beacon_count);
        const double target_error = norm(estimate - world.target);
        const double beacon_position_error = beacon_position_rmse(world, beacon_estimates);
        const double beacon_yaw_error = beacon_yaw_rmse(world, beacon_estimates);
        return {scenario, beacon_count, trial, world.target, estimate, target_error,
                beacon_position_error, beacon_yaw_error,
                result.cost, result.iterations, elapsed_ms(start, stop),
                result.converged, trial_accuracy_success(target_error, beacon_position_error, beacon_yaw_error)};
    }

    const auto measurements = generate_global_bearing_measurements(world, path, noise, rng);
    const auto start = std::chrono::steady_clock::now();
    const auto result = gauss_newton(
        initial_state_scenario2(config.initial_target_estimate),
        [&](const std::vector<double>& state) {
            return residuals_scenario2(state, path, measurements);
        },
        config.batch_solver_max_iterations,
        config.batch_solver_initial_lambda);
    const auto stop = std::chrono::steady_clock::now();
    const Vec2 estimate{result.x[0], result.x[1]};
    const auto beacon_estimates =
        beacon_estimates_from_scenario2_measurements(estimate, beacon_count, measurements);
    const double target_error = norm(estimate - world.target);
    const double beacon_position_error = beacon_position_rmse(world, beacon_estimates);
    return {scenario, beacon_count, trial, world.target, estimate, target_error,
            beacon_position_error, -1.0,
            result.cost, result.iterations, elapsed_ms(start, stop),
            result.converged, trial_accuracy_success(target_error, beacon_position_error, -1.0)};
}

}  // namespace

AdaptiveLocalizationRun run_adaptive_localization(
    const SimulationConfig& config,
    std::mt19937& rng) {
    World world = make_world(config.adaptive_beacon_count);
    Vec2 robot = config.adaptive_initial_robot;
    Vec2 target_estimate = config.adaptive_initial_target_estimate;
    std::vector<AdaptiveLocalizationPoint> points;
    points.reserve(static_cast<std::size_t>(config.adaptive_steps));

    for (int step = 0; step < config.adaptive_steps; ++step) {
        const auto measurements = measure_core_model(world, robot, config.adaptive_noise, rng);
        const Vec2 target_dot = core_adaptive_update(
            robot, target_estimate, measurements, config.adaptive_gain);
        target_estimate = target_estimate + target_dot * config.adaptive_dt;

        AdaptiveLocalizationPoint point;
        point.step = step;
        point.robot = robot;
        point.target_estimate = target_estimate;
        point.beacon_estimates = core_beacon_estimates(target_estimate, measurements);
        point.target_error = norm(target_estimate - world.target);
        point.goal_error = norm(robot - world.target);
        point.cost = core_cost(robot, target_estimate, measurements);
        points.push_back(point);

        const Vec2 control = (target_estimate - robot) * config.adaptive_target_seeking_gain;
        robot = robot + control * config.adaptive_dt;
    }

    return {world, points};
}

ClosedLoopResult run_closed_loop_comparison(
    int scenario,
    const SimulationConfig& config,
    std::mt19937& rng) {
    return run_closed_loop_comparison(scenario, config.closed_loop_beacon_count, config, rng);
}

ClosedLoopResult run_closed_loop_comparison(
    int scenario,
    int beacon_count,
    const SimulationConfig& config,
    std::mt19937& rng) {
    return run_closed_loop_comparison(
        scenario, beacon_count, config, rng, ClosedLoopExcitationMode::Circular);
}

ClosedLoopResult run_closed_loop_comparison(
    int scenario,
    int beacon_count,
    const SimulationConfig& config,
    std::mt19937& rng,
    ClosedLoopExcitationMode excitation_mode) {
    const World world = make_world(beacon_count);

    std::vector<Vec2> path;
    std::vector<LocalFrameMeasurement> local_measurements;
    std::vector<GlobalBearingMeasurement> global_measurements;
    std::vector<double> state1 = initial_state_scenario1(
        beacon_count,
        config.initial_target_estimate,
        config.initial_beacon_guess_radius,
        config.initial_beacon_guess_yaw);
    std::vector<double> state2 = initial_state_scenario2(config.initial_target_estimate);
    std::vector<BeaconEstimate> beacon_estimates =
        beacon_estimates_from_scenario1_state(state1, beacon_count);
    std::vector<ClosedLoopPoint> points;
    Vec2 robot = config.initial_robot;
    Vec2 target_estimate = config.initial_target_estimate;
    double current_cost = 0.0;
    int excitation_epoch = 1;

    ClosedLoopPoint initial_point;
    initial_point.step = 0;
    initial_point.robot = robot;
    initial_point.target_estimate = target_estimate;
    initial_point.beacon_estimates = beacon_estimates;
    initial_point.target_error = norm(target_estimate - world.target);
    initial_point.goal_error = norm(robot - world.target);
    initial_point.beacon_position_rmse = beacon_position_rmse(world, beacon_estimates);
    initial_point.beacon_yaw_rmse =
        scenario == 1 ? beacon_yaw_rmse(world, beacon_estimates) : -1.0;
    initial_point.cost = current_cost;
    points.push_back(initial_point);

    for (int step = 1; step <= config.closed_loop_steps; ++step) {
        path.push_back(robot);
        for (std::size_t i = 0; i < world.beacons.size(); ++i) {
            if (scenario == 1) {
                local_measurements.push_back(make_local_frame_measurement(
                    world, robot, i, path.size() - 1, config.closed_loop_noise, rng));
            } else {
                global_measurements.push_back(make_global_bearing_measurement(
                    world, robot, i, path.size() - 1, config.closed_loop_noise, rng));
            }
        }

        if (scenario == 1) {
            const auto result = gauss_newton(
                state1,
                [&](const std::vector<double>& state) {
                    return residuals_scenario1(state, beacon_count, path, local_measurements);
                },
                config.closed_loop_solver_max_iterations,
                config.closed_loop_solver_initial_lambda);
            state1 = result.x;
            target_estimate = {state1[0], state1[1]};
            current_cost = result.cost;
            beacon_estimates = beacon_estimates_from_scenario1_state(state1, beacon_count);
        } else {
            const auto result = gauss_newton(
                state2,
                [&](const std::vector<double>& state) {
                    return residuals_scenario2(state, path, global_measurements);
                },
                config.closed_loop_solver_max_iterations,
                config.closed_loop_solver_initial_lambda);
            state2 = result.x;
            target_estimate = {state2[0], state2[1]};
            current_cost = result.cost;
            beacon_estimates = beacon_estimates_from_scenario2_measurements(
                target_estimate, beacon_count, global_measurements);
        }

        bool retriggered = false;
        if (excitation_mode == ClosedLoopExcitationMode::Supervised) {
            const auto observability =
                local_observability_rank_and_sigma_min(state1, path, local_measurements);
            const double spread = trajectory_spread(local_measurements, beacon_count);
            const bool under_excited = spread < config.supervised_spread_threshold;
            const bool under_conditioned =
                observability.first < 5 || observability.second < config.supervised_sigma_min_threshold;
            if (under_excited || under_conditioned) {
                excitation_epoch = step;
                retriggered = true;
            }
        }

        const double decay_reference = excitation_mode == ClosedLoopExcitationMode::Supervised
            ? static_cast<double>(step - excitation_epoch)
            : static_cast<double>(step - 1);
        const double phase_reference = excitation_mode == ClosedLoopExcitationMode::Supervised
            ? static_cast<double>(step)
            : static_cast<double>(step - 1);
        const double exploration =
            config.exploration_amplitude * std::exp(-config.exploration_decay * decay_reference);
        const Vec2 swirl{
            exploration * std::cos(config.exploration_frequency * phase_reference),
            exploration * std::sin(config.exploration_frequency * phase_reference),
        };
        Vec2 excitation = swirl;
        if (excitation_mode == ClosedLoopExcitationMode::Information) {
            excitation = information_driven_excitation(
                robot,
                target_estimate,
                state1,
                beacon_estimates,
                path,
                local_measurements,
                config,
                step);
        }
        robot = robot + ((target_estimate - robot) * config.closed_loop_control_gain + excitation) *
            config.closed_loop_dt;

        ClosedLoopPoint point;
        point.step = step;
        point.robot = robot;
        point.target_estimate = target_estimate;
        point.beacon_estimates = beacon_estimates;
        point.target_error = norm(target_estimate - world.target);
        point.goal_error = norm(robot - world.target);
        point.beacon_position_rmse = beacon_position_rmse(world, beacon_estimates);
        point.beacon_yaw_rmse = scenario == 1 ? beacon_yaw_rmse(world, beacon_estimates) : -1.0;
        point.cost = current_cost;
        point.retriggered = retriggered;
        points.push_back(point);
    }

    return {scenario, world, points, beacon_estimates, target_estimate};
}

std::vector<ActiveExcitationComparisonRow> run_active_excitation_comparison(
    const SimulationConfig& config) {
    const auto make_row = [](const std::string& name, const ClosedLoopResult& result) {
        ActiveExcitationComparisonRow row;
        row.excitation = name;
        row.beacons = static_cast<int>(result.world.beacons.size());
        if (!result.points.empty()) {
            const auto& final_point = result.points.back();
            row.final_goal_error = final_point.goal_error;
            row.final_target_error = final_point.target_error;
            row.final_beacon_position_rmse = final_point.beacon_position_rmse;
            row.final_beacon_yaw_rmse = final_point.beacon_yaw_rmse;
            row.final_cost = final_point.cost;
        }
        return row;
    };

    std::mt19937 circular_rng(config.closed_loop_seed + 200U);
    const auto circular = run_closed_loop_comparison(
        1, 1, config, circular_rng, ClosedLoopExcitationMode::Circular);

    std::mt19937 information_rng(config.closed_loop_seed + 200U);
    const auto information = run_closed_loop_comparison(
        1, 1, config, information_rng, ClosedLoopExcitationMode::Information);

    return {
        make_row("decaying_circular", circular),
        make_row("information_logdet", information),
    };
}

std::vector<SupervisedExcitationComparisonRow> run_supervised_excitation_comparison(
    const SimulationConfig& config) {
    const auto steps_to_threshold = [](const std::vector<ClosedLoopPoint>& points, double threshold,
                                        double (*metric)(const ClosedLoopPoint&)) {
        for (const auto& point : points) {
            if (metric(point) < threshold) {
                return point.step;
            }
        }
        return -1;
    };

    const auto make_row = [&](const std::string& name, const ClosedLoopResult& result) {
        SupervisedExcitationComparisonRow row;
        row.excitation = name;
        for (const auto& point : result.points) {
            if (point.retriggered) {
                ++row.retrigger_count;
            }
        }
        row.steps_to_goal_threshold = steps_to_threshold(
            result.points, config.supervised_goal_error_threshold,
            [](const ClosedLoopPoint& point) { return point.goal_error; });
        row.steps_to_target_threshold = steps_to_threshold(
            result.points, config.supervised_target_error_threshold,
            [](const ClosedLoopPoint& point) { return point.target_error; });
        if (!result.points.empty()) {
            const auto& final_point = result.points.back();
            row.final_goal_error = final_point.goal_error;
            row.final_target_error = final_point.target_error;
            row.final_beacon_position_rmse = final_point.beacon_position_rmse;
            row.final_beacon_yaw_rmse = final_point.beacon_yaw_rmse;
            row.final_cost = final_point.cost;
        }
        return row;
    };

    const auto run_pair = [&](const std::string& suffix, const SimulationConfig& scenario_config) {
        std::mt19937 circular_rng(scenario_config.closed_loop_seed + 300U);
        const auto circular = run_closed_loop_comparison(
            1, 1, scenario_config, circular_rng, ClosedLoopExcitationMode::Circular);

        std::mt19937 supervised_rng(scenario_config.closed_loop_seed + 300U);
        const auto supervised = run_closed_loop_comparison(
            1, 1, scenario_config, supervised_rng, ClosedLoopExcitationMode::Supervised);

        return std::vector<SupervisedExcitationComparisonRow>{
            make_row("decaying_circular_" + suffix, circular),
            make_row("excitation_supervised_" + suffix, supervised),
        };
    };

    // Nominal scenario: same closed-loop parameters used throughout the rest
    // of the paper (far initial pose, moderate decay). The target-seeking
    // transient alone already supplies ample trajectory diversity here.
    const auto nominal = run_pair("nominal", config);

    // Understimulated scenario: the excitation schedule decays far faster
    // than the ACC 2027 default, and the robot starts exactly at the true
    // target (matching World::make_world's fixed target), removing the
    // convergence transient that would otherwise supply diversity on its
    // own. Only the excitation schedule itself can excite the geometry.
    SimulationConfig stress_config = config;
    stress_config.exploration_decay = 2.0;
    stress_config.initial_target_estimate = {1.2, -0.75};
    stress_config.initial_robot = stress_config.initial_target_estimate;
    const auto understimulated = run_pair("understimulated", stress_config);

    std::vector<SupervisedExcitationComparisonRow> rows;
    rows.insert(rows.end(), nominal.begin(), nominal.end());
    rows.insert(rows.end(), understimulated.begin(), understimulated.end());
    return rows;
}

std::vector<SupervisedLambdaSweepRow> run_supervised_lambda_sweep(
    const SimulationConfig& config) {
    const std::vector<double> lambdas{0.02, 0.05, 0.10, 0.25, 0.50, 1.0, 2.0};

    std::vector<SupervisedLambdaSweepRow> rows;
    rows.reserve(lambdas.size());
    for (double lambda : lambdas) {
        // Same no-transient scenario as the underexcited comparison: the
        // robot starts at the true target, so only the excitation schedule
        // can excite the geometry, and lambda controls how quickly the
        // fixed schedule gives up.
        SimulationConfig sweep_config = config;
        sweep_config.exploration_decay = lambda;
        sweep_config.initial_target_estimate = {1.2, -0.75};
        sweep_config.initial_robot = sweep_config.initial_target_estimate;

        std::mt19937 fixed_rng(sweep_config.closed_loop_seed + 300U);
        const auto fixed = run_closed_loop_comparison(
            1, 1, sweep_config, fixed_rng, ClosedLoopExcitationMode::Circular);

        std::mt19937 supervised_rng(sweep_config.closed_loop_seed + 300U);
        const auto supervised = run_closed_loop_comparison(
            1, 1, sweep_config, supervised_rng, ClosedLoopExcitationMode::Supervised);

        SupervisedLambdaSweepRow row;
        row.lambda = lambda;
        for (const auto& point : supervised.points) {
            if (point.retriggered) {
                ++row.supervised_retrigger_count;
            }
        }
        if (!fixed.points.empty()) {
            const auto& fixed_final = fixed.points.back();
            row.fixed_final_target_error = fixed_final.target_error;
            row.fixed_final_beacon_position_rmse = fixed_final.beacon_position_rmse;
            row.fixed_final_beacon_yaw_rmse = fixed_final.beacon_yaw_rmse;
        }
        if (!supervised.points.empty()) {
            const auto& supervised_final = supervised.points.back();
            row.supervised_final_target_error = supervised_final.target_error;
            row.supervised_final_beacon_position_rmse = supervised_final.beacon_position_rmse;
            row.supervised_final_beacon_yaw_rmse = supervised_final.beacon_yaw_rmse;
        }
        rows.push_back(row);
    }
    return rows;
}

TrialResult run_trial(
    int scenario,
    int beacon_count,
    int trial,
    const SimulationConfig& config,
    std::mt19937& rng) {
    const World world = make_world(beacon_count);
    const std::vector<Vec2> path = make_vehicle_path(config.monte_carlo_path_steps);
    return run_trial_with_world_path(
        scenario, beacon_count, trial, world, path, config, config.monte_carlo_noise, rng);
}

std::vector<TrialResult> run_monte_carlo(const SimulationConfig& config) {
    std::mt19937 rng(config.monte_carlo_seed);
    std::vector<TrialResult> trials;
    trials.reserve(
        static_cast<std::size_t>(config.monte_carlo_trials_per_case) *
        config.monte_carlo_scenarios.size() *
        config.monte_carlo_beacon_counts.size());

    for (int scenario : config.monte_carlo_scenarios) {
        for (int beacon_count : config.monte_carlo_beacon_counts) {
            for (int trial = 0; trial < config.monte_carlo_trials_per_case; ++trial) {
                trials.push_back(run_trial(scenario, beacon_count, trial, config, rng));
            }
        }
    }
    return trials;
}

SummaryRow summarize(int scenario, int beacon_count, const std::vector<TrialResult>& trials) {
    SummaryRow row;
    row.scenario = scenario;
    row.beacons = beacon_count;
    double sum_error = 0.0;
    double sum_error2 = 0.0;
    double sum_error4 = 0.0;
    double sum_dx = 0.0;
    double sum_dy = 0.0;
    double sum_cost = 0.0;
    double sum_iterations = 0.0;
    double sum_runtime_ms = 0.0;
    double sum_beacon_position_rmse = 0.0;
    double sum_beacon_position_rmse2 = 0.0;
    double sum_beacon_yaw_rmse = 0.0;
    double sum_beacon_yaw_rmse2 = 0.0;
    int converged = 0;
    int count = 0;
    int yaw_count = 0;

    for (const auto& trial : trials) {
        if (trial.scenario != scenario || trial.beacons != beacon_count) {
            continue;
        }
        const double dx = trial.estimate.x - trial.truth.x;
        const double dy = trial.estimate.y - trial.truth.y;
        sum_error += trial.error;
        sum_error2 += trial.error * trial.error;
        sum_error4 += trial.error * trial.error * trial.error * trial.error;
        sum_dx += dx;
        sum_dy += dy;
        sum_cost += trial.cost;
        sum_iterations += static_cast<double>(trial.iterations);
        sum_runtime_ms += trial.runtime_ms;
        sum_beacon_position_rmse += trial.beacon_position_rmse;
        sum_beacon_position_rmse2 += trial.beacon_position_rmse * trial.beacon_position_rmse;
        if (trial.beacon_yaw_rmse >= 0.0) {
            sum_beacon_yaw_rmse += trial.beacon_yaw_rmse;
            sum_beacon_yaw_rmse2 += trial.beacon_yaw_rmse * trial.beacon_yaw_rmse;
            ++yaw_count;
        }
        converged += trial.converged ? 1 : 0;
        ++count;
    }

    if (count > 0) {
        row.mean_error = sum_error / count;
        row.rmse = std::sqrt(sum_error2 / count);
        row.rmse_ci95 = rmse_ci95_from_sums(sum_error2, sum_error4, count);
        row.mean_error_ci95 = mean_ci95_from_sums(sum_error, sum_error2, count);
        row.bias_x = sum_dx / count;
        row.bias_y = sum_dy / count;
        row.mean_cost = sum_cost / count;
        row.mean_iterations = sum_iterations / count;
        row.mean_runtime_ms = sum_runtime_ms / count;
        row.convergence_rate = static_cast<double>(converged) / count;
        row.mean_beacon_position_rmse = sum_beacon_position_rmse / count;
        row.mean_beacon_position_rmse_ci95 =
            mean_ci95_from_sums(sum_beacon_position_rmse, sum_beacon_position_rmse2, count);
        row.mean_beacon_yaw_rmse =
            yaw_count > 0 ? sum_beacon_yaw_rmse / static_cast<double>(yaw_count) : -1.0;
        row.mean_beacon_yaw_rmse_ci95 =
            yaw_count > 0
                ? mean_ci95_from_sums(sum_beacon_yaw_rmse, sum_beacon_yaw_rmse2, yaw_count)
                : -1.0;
    }
    return row;
}

std::vector<NoiseRobustnessRow> run_noise_robustness_sweep(const SimulationConfig& config) {
    const std::vector<double> range_sigmas{0.0, 0.01, 0.03, 0.06, 0.09};
    const std::vector<double> bearing_sigmas{0.0, 0.002, 0.006, 0.012, 0.018};
    const std::vector<int> beacon_counts{1, 2};
    std::vector<NoiseRobustnessRow> rows;
    rows.reserve(
        range_sigmas.size() * bearing_sigmas.size() *
        beacon_counts.size() * config.monte_carlo_scenarios.size());

    for (double range_sigma : range_sigmas) {
        for (double bearing_sigma : bearing_sigmas) {
            SimulationConfig sweep_config = config;
            sweep_config.monte_carlo_noise.range_sigma = range_sigma;
            sweep_config.monte_carlo_noise.bearing_sigma = bearing_sigma;
            const unsigned int seed_offset =
                static_cast<unsigned int>(100000.0 * range_sigma + 1000000.0 * bearing_sigma + 17.0);
            std::mt19937 rng(config.monte_carlo_seed + seed_offset);

            for (int scenario : config.monte_carlo_scenarios) {
                for (int beacon_count : beacon_counts) {
                    std::vector<TrialResult> trials;
                    trials.reserve(static_cast<std::size_t>(config.monte_carlo_trials_per_case));
                    for (int trial = 0; trial < config.monte_carlo_trials_per_case; ++trial) {
                        trials.push_back(run_trial(scenario, beacon_count, trial, sweep_config, rng));
                    }

                    const SummaryRow summary = summarize(scenario, beacon_count, trials);
                    rows.push_back({
                        scenario,
                        beacon_count,
                        range_sigma,
                        bearing_sigma,
                        summary.rmse,
                        summary.mean_beacon_position_rmse,
                        summary.mean_beacon_yaw_rmse,
                        summary.mean_cost,
                        summary.mean_iterations,
                        summary.mean_runtime_ms,
                        summary.convergence_rate,
                    });
                }
            }
        }
    }

    return rows;
}

std::vector<GeometrySweepRow> run_geometry_sweep(const SimulationConfig& config) {
    const std::vector<double> separations{0.3, 0.6, 1.0, 1.6, 2.4, 3.2, 4.0};
    const auto path = make_vehicle_path(config.monte_carlo_path_steps, "excited");
    std::vector<GeometrySweepRow> rows;
    rows.reserve(separations.size());

    for (std::size_t i = 0; i < separations.size(); ++i) {
        const World world = make_world_with_beacon_separation(separations[i]);
        std::mt19937 rng(config.monte_carlo_seed + 2000U + static_cast<unsigned int>(i));
        std::vector<TrialResult> trials;
        trials.reserve(static_cast<std::size_t>(config.monte_carlo_trials_per_case));
        for (int trial = 0; trial < config.monte_carlo_trials_per_case; ++trial) {
            trials.push_back(run_trial_with_world_path(
                1, 2, trial, world, path, config, config.monte_carlo_noise, rng));
        }
        const SummaryRow summary = summarize(1, 2, trials);
        rows.push_back({
            2,
            separations[i],
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }

    return rows;
}

std::vector<TrajectorySweepRow> run_trajectory_sweep(const SimulationConfig& config) {
    const std::vector<std::string> trajectories{
        "stationary", "line", "circle", "figure_eight", "excited"};
    const World world = make_world(1);
    std::vector<TrajectorySweepRow> rows;
    rows.reserve(trajectories.size());

    for (std::size_t i = 0; i < trajectories.size(); ++i) {
        const auto path = make_vehicle_path(config.monte_carlo_path_steps, trajectories[i]);
        std::mt19937 rank_rng(config.monte_carlo_seed + 3000U + static_cast<unsigned int>(i));
        const auto rank_measurements = generate_local_frame_measurements(world, path, Noise{0.0, 0.0}, rank_rng);
        const auto rank_and_sigma =
            local_observability_rank_and_sigma_min(true_state_scenario1(world), path, rank_measurements);

        std::mt19937 rng(config.monte_carlo_seed + 4000U + static_cast<unsigned int>(i));
        std::vector<TrialResult> trials;
        trials.reserve(static_cast<std::size_t>(config.monte_carlo_trials_per_case));
        for (int trial = 0; trial < config.monte_carlo_trials_per_case; ++trial) {
            trials.push_back(run_trial_with_world_path(
                1, 1, trial, world, path, config, config.monte_carlo_noise, rng));
        }
        const SummaryRow summary = summarize(1, 1, trials);
        rows.push_back({
            trajectories[i],
            1,
            rank_and_sigma.first,
            rank_and_sigma.second,
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }

    return rows;
}

std::vector<InitialPoseRobustnessRow> run_initial_pose_robustness_sweep(const SimulationConfig& config) {
    const std::vector<Vec2> starts{
        config.initial_robot,
        {-3.2, -2.4},
        {3.2, 2.4},
        {-3.4, 2.2},
        {3.4, -2.2},
        {0.0, 3.0},
    };

    std::vector<InitialPoseRobustnessRow> rows;
    rows.reserve(starts.size());
    for (std::size_t i = 0; i < starts.size(); ++i) {
        SimulationConfig sweep_config = config;
        sweep_config.initial_robot = starts[i];
        std::mt19937 rng(config.closed_loop_seed + 500U + static_cast<unsigned int>(i));
        const ClosedLoopResult result = run_closed_loop_comparison(1, sweep_config, rng);
        const ClosedLoopPoint& final_point = result.points.back();
        rows.push_back({
            1,
            static_cast<int>(i),
            starts[i],
            final_point.goal_error,
            final_point.target_error,
            final_point.beacon_position_rmse,
            final_point.beacon_yaw_rmse,
        });
    }

    return rows;
}

std::vector<MinimalBeaconExcitationRow> run_minimal_beacon_excitation_study(
    const SimulationConfig& config) {
    const World world = make_world(1);
    const auto full_path = make_vehicle_path(config.monte_carlo_path_steps);
    const std::vector<std::pair<const char*, int>> cases{
        {"single_pose_no_excitation", 1},
        {"two_distinct_poses", 2},
        {"full_excited_trajectory", config.monte_carlo_path_steps},
    };

    std::vector<MinimalBeaconExcitationRow> rows;
    rows.reserve(cases.size());

    for (std::size_t case_index = 0; case_index < cases.size(); ++case_index) {
        const int poses = std::max(1, cases[case_index].second);
        std::vector<Vec2> path;
        path.reserve(static_cast<std::size_t>(poses));
        if (poses == 1) {
            path.push_back(full_path.front());
        } else if (poses == 2) {
            path.push_back(full_path.front());
            path.push_back(full_path[full_path.size() / 3U]);
        } else {
            path = full_path;
        }

        std::mt19937 measurement_rng(
            config.monte_carlo_seed + 700U + static_cast<unsigned int>(case_index));
        const auto measurements = generate_local_frame_measurements(world, path, Noise{0.0, 0.0}, measurement_rng);
        const auto truth = true_state_scenario1(world);
        const auto metrics = local_observability_metrics(truth, path, measurements);

        auto seed = initial_state_scenario1(
            1,
            config.initial_target_estimate,
            config.initial_beacon_guess_radius,
            config.initial_beacon_guess_yaw);
        std::vector<double> closed_form_seed;
        if (two_view_closed_form_initial_state(1, path, measurements, closed_form_seed)) {
            seed = closed_form_seed;
        }
        const auto result = gauss_newton(
            seed,
            [&](const std::vector<double>& state) {
                return residuals_scenario1(state, 1, path, measurements);
            },
            config.batch_solver_max_iterations,
            config.batch_solver_initial_lambda,
            [&](const std::vector<double>& state) {
                return jacobian_scenario1(state, 1, path, measurements);
            });
        const Vec2 estimate{result.x[0], result.x[1]};
        const auto beacon_estimates = beacon_estimates_from_scenario1_state(result.x, 1);

        rows.push_back({
            cases[case_index].first,
            1,
            static_cast<int>(path.size()),
            metrics.rank,
            metrics.trajectory_spread,
            metrics.sigma_min,
            norm(estimate - world.target),
            beacon_position_rmse(world, beacon_estimates),
            beacon_yaw_rmse(world, beacon_estimates),
            result.cost,
            result.iterations,
            result.converged,
        });
    }

    return rows;
}

std::vector<PoorInitializationSweepRow> run_poor_initialization_sweep(
    const SimulationConfig& config) {
    const World world = make_world(1);
    const auto path = make_vehicle_path(config.monte_carlo_path_steps, "excited");
    const MeasurementStress stress;
    const std::vector<std::tuple<const char*, double, double, double, int>> cases{
        {"nominal_seed", 0.0, 2.0, 0.0, 1},
        {"target_seed_1m", 1.0, 2.0, 0.0, 1},
        {"target_seed_2m", 2.0, 2.0, 0.0, 1},
        {"poor_beacon_radius", 1.0, 0.7, 0.0, 1},
        {"poor_yaw_seed", 1.0, 2.0, 1.57, 1},
        {"poor_seed_multistart", 2.0, 2.8, 2.4, std::max(2, config.multistart_count)},
    };

    std::vector<PoorInitializationSweepRow> rows;
    rows.reserve(cases.size());
    for (std::size_t case_index = 0; case_index < cases.size(); ++case_index) {
        const auto& [name, target_offset, beacon_radius, yaw_seed, multistarts] = cases[case_index];
        std::mt19937 rng(config.monte_carlo_seed + 9000U + static_cast<unsigned int>(case_index));
        std::vector<TrialResult> trials;
        trials.reserve(static_cast<std::size_t>(config.expanded_trials_per_case));
        for (int trial = 0; trial < config.expanded_trials_per_case; ++trial) {
            const double angle = 2.0 * kPi * static_cast<double>(trial) /
                static_cast<double>(std::max(1, config.expanded_trials_per_case));
            const Vec2 target_seed{
                world.target.x + target_offset * std::cos(angle),
                world.target.y + target_offset * std::sin(angle),
            };
            trials.push_back(run_multistart_local_batch_trial(
                trial,
                1,
                world,
                path,
                path,
                config,
                config.monte_carlo_noise,
                stress,
                target_seed,
                beacon_radius,
                yaw_seed,
                multistarts,
                false,
                rng));
        }
        const auto summary = summarize(1, 1, trials);
        rows.push_back({
            name,
            target_offset,
            beacon_radius,
            yaw_seed,
            multistarts,
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }
    return rows;
}

std::vector<TrajectorySweepRow> run_near_degenerate_trajectory_sweep(
    const SimulationConfig& config) {
    const std::vector<std::string> trajectories{
        "stationary",
        "short_line",
        "repeated_viewpoints",
        "low_curvature_arc",
        "collinear_pass",
        "line",
        "excited_figure_eight",
    };
    const World world = make_world(1);
    std::vector<TrajectorySweepRow> rows;
    rows.reserve(trajectories.size());

    for (std::size_t i = 0; i < trajectories.size(); ++i) {
        const auto path = make_vehicle_path(config.monte_carlo_path_steps, trajectories[i]);
        std::mt19937 rank_rng(config.monte_carlo_seed + 10000U + static_cast<unsigned int>(i));
        const auto rank_measurements = generate_local_frame_measurements(world, path, Noise{0.0, 0.0}, rank_rng);
        const auto metrics = local_observability_metrics(true_state_scenario1(world), path, rank_measurements);

        std::mt19937 rng(config.monte_carlo_seed + 11000U + static_cast<unsigned int>(i));
        std::vector<TrialResult> trials;
        trials.reserve(static_cast<std::size_t>(config.expanded_trials_per_case));
        for (int trial = 0; trial < config.expanded_trials_per_case; ++trial) {
            trials.push_back(run_trial_with_world_path(
                1, 1, trial, world, path, config, config.monte_carlo_noise, rng));
        }
        const auto summary = summarize(1, 1, trials);
        rows.push_back({
            trajectories[i],
            1,
            metrics.rank,
            metrics.sigma_min,
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }
    return rows;
}

std::vector<IntermittentMeasurementSweepRow> run_intermittent_measurement_sweep(
    const SimulationConfig& config) {
    const World world = make_world(1);
    const auto path = make_vehicle_path(config.monte_carlo_path_steps, "excited");
    const std::vector<double> dropouts{
        0.0,
        0.25 * config.dropout_probability_max,
        0.50 * config.dropout_probability_max,
        0.75 * config.dropout_probability_max,
        config.dropout_probability_max,
    };

    std::vector<IntermittentMeasurementSweepRow> rows;
    rows.reserve(dropouts.size());
    for (std::size_t i = 0; i < dropouts.size(); ++i) {
        MeasurementStress stress;
        stress.dropout_probability = dropouts[i];
        std::mt19937 rng(config.monte_carlo_seed + 12000U + static_cast<unsigned int>(i));
        std::vector<TrialResult> trials;
        double measurement_count = 0.0;
        trials.reserve(static_cast<std::size_t>(config.expanded_trials_per_case));
        for (int trial = 0; trial < config.expanded_trials_per_case; ++trial) {
            const auto measurements = generate_stressed_local_measurements(
                world, path, config.monte_carlo_noise, stress, rng);
            measurement_count += static_cast<double>(measurements.size());
            const auto initial_state = initial_state_scenario1(
                1,
                config.initial_target_estimate,
                config.initial_beacon_guess_radius,
                config.initial_beacon_guess_yaw);
            trials.push_back(run_local_batch_trial_with_measurements(
                trial, 1, world, path, measurements, config, config.monte_carlo_noise,
                initial_state, false, 0.0));
        }
        const auto summary = summarize(1, 1, trials);
        rows.push_back({
            dropouts[i],
            measurement_count / static_cast<double>(std::max(1, config.expanded_trials_per_case)),
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }
    return rows;
}

std::vector<OutlierRobustnessSweepRow> run_outlier_robustness_sweep(
    const SimulationConfig& config) {
    const World world = make_world(1);
    const auto path = make_vehicle_path(config.monte_carlo_path_steps, "excited");
    const std::vector<double> probabilities{0.0, 0.05, 0.10, 0.20};
    std::vector<OutlierRobustnessSweepRow> rows;
    rows.reserve(probabilities.size() * 2U);

    for (std::size_t i = 0; i < probabilities.size(); ++i) {
        MeasurementStress stress;
        stress.outlier_probability = probabilities[i];
        stress.outlier_range_magnitude = config.outlier_range_magnitude;
        stress.outlier_bearing_magnitude = config.outlier_bearing_magnitude;
        std::mt19937 rng(config.monte_carlo_seed + 13000U + static_cast<unsigned int>(i));
        std::vector<TrialResult> vanilla_trials;
        std::vector<TrialResult> robust_trials;
        vanilla_trials.reserve(static_cast<std::size_t>(config.expanded_trials_per_case));
        robust_trials.reserve(static_cast<std::size_t>(config.expanded_trials_per_case));

        for (int trial = 0; trial < config.expanded_trials_per_case; ++trial) {
            const auto measurements = generate_stressed_local_measurements(
                world, path, config.monte_carlo_noise, stress, rng);
            const auto initial_state = initial_state_scenario1(
                1,
                config.initial_target_estimate,
                config.initial_beacon_guess_radius,
                config.initial_beacon_guess_yaw);
            vanilla_trials.push_back(run_local_batch_trial_with_measurements(
                trial, 1, world, path, measurements, config, config.monte_carlo_noise,
                initial_state, false, 0.0));
            robust_trials.push_back(run_local_batch_trial_with_measurements(
                trial, 1, world, path, measurements, config, config.monte_carlo_noise,
                initial_state, true, config.robust_huber_delta));
        }

        const auto vanilla = summarize(1, 1, vanilla_trials);
        const auto robust = summarize(1, 1, robust_trials);
        rows.push_back({
            "batch_gn",
            probabilities[i],
            config.outlier_range_magnitude,
            config.outlier_bearing_magnitude,
            vanilla.rmse,
            vanilla.mean_beacon_position_rmse,
            vanilla.mean_beacon_yaw_rmse,
            vanilla.mean_cost,
            vanilla.mean_iterations,
            vanilla.mean_runtime_ms,
            vanilla.convergence_rate,
        });
        rows.push_back({
            "robust_huber_batch_gn",
            probabilities[i],
            config.outlier_range_magnitude,
            config.outlier_bearing_magnitude,
            robust.rmse,
            robust.mean_beacon_position_rmse,
            robust.mean_beacon_yaw_rmse,
            robust.mean_cost,
            robust.mean_iterations,
            robust.mean_runtime_ms,
            robust.convergence_rate,
        });
    }
    return rows;
}

std::vector<VehicleLocalizationNoiseSweepRow> run_vehicle_localization_noise_sweep(
    const SimulationConfig& config) {
    const World world = make_world(1);
    const auto true_path = make_vehicle_path(config.monte_carlo_path_steps, "excited");
    const MeasurementStress stress;
    const std::vector<double> sigmas{
        0.0,
        0.25 * config.vehicle_pose_noise_max,
        0.50 * config.vehicle_pose_noise_max,
        0.75 * config.vehicle_pose_noise_max,
        config.vehicle_pose_noise_max,
    };

    std::vector<VehicleLocalizationNoiseSweepRow> rows;
    rows.reserve(sigmas.size() + 1U);
    for (std::size_t i = 0; i < sigmas.size(); ++i) {
        std::mt19937 rng(config.monte_carlo_seed + 14000U + static_cast<unsigned int>(i));
        std::vector<TrialResult> trials;
        trials.reserve(static_cast<std::size_t>(config.expanded_trials_per_case));
        for (int trial = 0; trial < config.expanded_trials_per_case; ++trial) {
            const auto estimator_path = make_noisy_path(true_path, sigmas[i], rng);
            trials.push_back(run_local_batch_trial_with_seed(
                trial,
                1,
                world,
                true_path,
                estimator_path,
                config,
                config.monte_carlo_noise,
                stress,
                config.initial_target_estimate,
                config.initial_beacon_guess_radius,
                config.initial_beacon_guess_yaw,
                false,
                rng));
        }
        const auto summary = summarize(1, 1, trials);
        rows.push_back({
            "iid_position_noise",
            sigmas[i],
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }
    {
        constexpr double drift_fraction = 0.005;
        std::mt19937 rng(config.monte_carlo_seed + 14500U);
        std::vector<TrialResult> trials;
        trials.reserve(static_cast<std::size_t>(config.expanded_trials_per_case));
        for (int trial = 0; trial < config.expanded_trials_per_case; ++trial) {
            const auto estimator_path = make_drifted_path(true_path, drift_fraction, rng);
            trials.push_back(run_local_batch_trial_with_seed(
                trial,
                1,
                world,
                true_path,
                estimator_path,
                config,
                config.monte_carlo_noise,
                stress,
                config.initial_target_estimate,
                config.initial_beacon_guess_radius,
                config.initial_beacon_guess_yaw,
                false,
                rng));
        }
        const auto summary = summarize(1, 1, trials);
        rows.push_back({
            "random_walk_drift",
            drift_fraction,
            summary.rmse,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_yaw_rmse,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }
    return rows;
}

std::vector<InformationConditioningRow> run_information_conditioning_sweep(
    const SimulationConfig& config) {
    const std::vector<std::string> trajectories{
        "stationary",
        "short_line",
        "repeated_viewpoints",
        "low_curvature_arc",
        "collinear_pass",
        "line",
        "circle",
        "figure_eight",
        "excited",
        "excited_figure_eight",
    };
    const World world = make_world(1);
    std::vector<InformationConditioningRow> rows;
    rows.reserve(trajectories.size());
    for (std::size_t i = 0; i < trajectories.size(); ++i) {
        const auto path = make_vehicle_path(config.monte_carlo_path_steps, trajectories[i]);
        std::mt19937 rng(config.monte_carlo_seed + 15000U + static_cast<unsigned int>(i));
        const auto measurements = generate_local_frame_measurements(world, path, Noise{0.0, 0.0}, rng);
        const auto metrics = local_observability_metrics(true_state_scenario1(world), path, measurements);
        rows.push_back({
            trajectories[i],
            1,
            static_cast<int>(measurements.size()),
            metrics.rank,
            metrics.trajectory_spread,
            metrics.sigma_min,
            metrics.sigma_max,
            metrics.condition_number,
            metrics.logdet,
        });
    }
    return rows;
}

std::vector<ExpandedBaselineSummaryRow> run_expanded_baseline_comparison(
    const SimulationConfig& config) {
    const World world = make_world(1);
    const auto path = make_vehicle_path(config.monte_carlo_path_steps, "excited");
    const MeasurementStress no_stress;
    std::vector<ExpandedBaselineSummaryRow> rows;
    const std::vector<std::string> estimators{
        "batch_gn",
        "robust_huber_batch_gn",
        "multistart_batch_gn",
        "sliding_window_gn",
        "two_view_initialized_ekf",
        "naive_ekf",
        "single_target_packet_batch_gn",
    };
    rows.reserve(estimators.size());

    for (const auto& estimator : estimators) {
        std::mt19937 rng(config.monte_carlo_seed + 16000U +
            static_cast<unsigned int>(rows.size() * 101U));
        std::vector<TrialResult> trials;
        trials.reserve(static_cast<std::size_t>(config.expanded_trials_per_case));
        for (int trial = 0; trial < config.expanded_trials_per_case; ++trial) {
            if (estimator == "naive_ekf") {
                trials.push_back(run_trial_with_world_path(
                    3, 1, trial, world, path, config, config.monte_carlo_noise, rng));
                continue;
            }
            if (estimator == "two_view_initialized_ekf") {
                trials.push_back(run_trial_with_world_path(
                    4, 1, trial, world, path, config, config.monte_carlo_noise, rng));
                continue;
            }

            const auto measurements = generate_stressed_local_measurements(
                world, path, config.monte_carlo_noise, no_stress, rng);
            const auto initial_state = initial_state_scenario1(
                1,
                config.initial_target_estimate,
                config.initial_beacon_guess_radius,
                config.initial_beacon_guess_yaw);
            if (estimator == "robust_huber_batch_gn") {
                trials.push_back(run_local_batch_trial_with_measurements(
                    trial, 1, world, path, measurements, config, config.monte_carlo_noise,
                    initial_state, true, config.robust_huber_delta));
            } else if (estimator == "single_target_packet_batch_gn") {
                trials.push_back(run_local_batch_trial_with_measurements(
                    trial, 1, world, path, measurements, config, config.monte_carlo_noise,
                    initial_state, false, 0.0, true, false));
            } else if (estimator == "multistart_batch_gn") {
                trials.push_back(run_multistart_local_batch_trial(
                    trial,
                    1,
                    world,
                    path,
                    path,
                    config,
                    config.monte_carlo_noise,
                    no_stress,
                    config.initial_target_estimate,
                    config.initial_beacon_guess_radius,
                    config.initial_beacon_guess_yaw,
                    config.multistart_count,
                    false,
                    rng));
            } else if (estimator == "sliding_window_gn") {
                const auto window = recent_measurement_window(measurements, config.sliding_window_size);
                trials.push_back(run_local_batch_trial_with_measurements(
                    trial, 1, world, path, window, config, config.monte_carlo_noise,
                    initial_state, false, 0.0));
            } else {
                trials.push_back(run_local_batch_trial_with_measurements(
                    trial, 1, world, path, measurements, config, config.monte_carlo_noise,
                    initial_state, false, 0.0));
            }
        }
        const int summary_scenario =
            estimator == "naive_ekf" ? 3 : estimator == "two_view_initialized_ekf" ? 4 : 1;
        const auto summary = summarize(summary_scenario, 1, trials);
        rows.push_back({
            estimator == "single_target_packet_batch_gn" ?
                "single_target_packet" : "gaussian_no_outliers",
            estimator,
            1,
            summary.rmse,
            summary.rmse_ci95,
            summary.mean_beacon_position_rmse,
            summary.mean_beacon_position_rmse_ci95,
            summary.mean_beacon_yaw_rmse,
            summary.mean_beacon_yaw_rmse_ci95,
            summary.mean_cost,
            summary.mean_iterations,
            summary.mean_runtime_ms,
            summary.convergence_rate,
        });
    }
    return rows;
}

}  // namespace adaptive
