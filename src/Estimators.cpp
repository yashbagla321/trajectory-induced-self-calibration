#include "adaptive_localization/Estimators.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace adaptive {

namespace {

double effective_range_sigma(const Noise& noise) {
    return noise.range_sigma > 0.0 ? noise.range_sigma : 0.03;
}

double effective_bearing_sigma(const Noise& noise) {
    return noise.bearing_sigma > 0.0 ? noise.bearing_sigma : 0.006;
}

Vec2 local_vehicle_vector(const LocalFrameMeasurement& measurement) {
    return unit_from_angle(measurement.bv_local) * measurement.rv;
}

Vec2 local_target_vector(const LocalFrameMeasurement& measurement) {
    return unit_from_angle(measurement.bt_local) * measurement.rt;
}

bool should_include_target_packet(
    std::vector<bool>& target_seen,
    const LocalFrameMeasurement& measurement,
    bool repeat_target_packets) {
    if (repeat_target_packets) {
        return true;
    }
    if (measurement.beacon >= target_seen.size()) {
        return false;
    }
    if (target_seen[measurement.beacon]) {
        return false;
    }
    target_seen[measurement.beacon] = true;
    return true;
}

}  // namespace

std::vector<double> residuals_scenario1(
    const std::vector<double>& state,
    int beacon_count,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise,
    bool repeat_target_packets) {
    std::vector<double> residuals;
    residuals.reserve(measurements.size() * 4);
    std::vector<bool> target_seen(static_cast<std::size_t>(std::max(0, beacon_count)), false);
    const Vec2 p{state[0], state[1]};
    const double range_sigma = effective_range_sigma(noise);
    const double bearing_sigma = effective_bearing_sigma(noise);

    for (const auto& m : measurements) {
        const std::size_t base = 2 + 3 * m.beacon;
        const Vec2 x_i{state[base], state[base + 1]};
        const double yaw_i = state[base + 2];
        const Vec2 vehicle_delta = path[m.time] - x_i;
        const Vec2 target_delta = p - x_i;
        const double rv_pred = std::max(1e-9, norm(vehicle_delta));
        const double bv_pred = wrap_angle(std::atan2(vehicle_delta.y, vehicle_delta.x) - yaw_i);

        residuals.push_back((rv_pred - m.rv) / range_sigma);
        residuals.push_back(wrap_angle(bv_pred - m.bv_local) / bearing_sigma);
        if (should_include_target_packet(target_seen, m, repeat_target_packets)) {
            const double rt_pred = std::max(1e-9, norm(target_delta));
            const double bt_pred = wrap_angle(std::atan2(target_delta.y, target_delta.x) - yaw_i);
            residuals.push_back((rt_pred - m.rt) / range_sigma);
            residuals.push_back(wrap_angle(bt_pred - m.bt_local) / bearing_sigma);
        }
    }

    return residuals;
}

std::vector<std::vector<double>> jacobian_scenario1(
    const std::vector<double>& state,
    int beacon_count,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise,
    bool repeat_target_packets) {
    const std::size_t state_dim = state.size();
    std::vector<std::vector<double>> jacobian;
    jacobian.reserve(measurements.size() * 4);
    std::vector<bool> target_seen(static_cast<std::size_t>(std::max(0, beacon_count)), false);
    const Vec2 p{state[0], state[1]};
    const double range_sigma = effective_range_sigma(noise);
    const double bearing_sigma = effective_bearing_sigma(noise);

    for (const auto& m : measurements) {
        const std::size_t base = 2 + 3 * m.beacon;
        const Vec2 x_i{state[base], state[base + 1]};
        const Vec2 vehicle_delta = path[m.time] - x_i;
        const double rv = std::max(1e-9, norm(vehicle_delta));
        const double rv2 = rv * rv;

        std::vector<double> vehicle_range_row(state_dim, 0.0);
        vehicle_range_row[base] = -vehicle_delta.x / (rv * range_sigma);
        vehicle_range_row[base + 1] = -vehicle_delta.y / (rv * range_sigma);
        jacobian.push_back(std::move(vehicle_range_row));

        std::vector<double> vehicle_bearing_row(state_dim, 0.0);
        vehicle_bearing_row[base] = vehicle_delta.y / (rv2 * bearing_sigma);
        vehicle_bearing_row[base + 1] = -vehicle_delta.x / (rv2 * bearing_sigma);
        vehicle_bearing_row[base + 2] = -1.0 / bearing_sigma;
        jacobian.push_back(std::move(vehicle_bearing_row));

        if (should_include_target_packet(target_seen, m, repeat_target_packets)) {
            const Vec2 target_delta = p - x_i;
            const double rt = std::max(1e-9, norm(target_delta));
            const double rt2 = rt * rt;

            std::vector<double> target_range_row(state_dim, 0.0);
            target_range_row[0] = target_delta.x / (rt * range_sigma);
            target_range_row[1] = target_delta.y / (rt * range_sigma);
            target_range_row[base] = -target_delta.x / (rt * range_sigma);
            target_range_row[base + 1] = -target_delta.y / (rt * range_sigma);
            jacobian.push_back(std::move(target_range_row));

            std::vector<double> target_bearing_row(state_dim, 0.0);
            target_bearing_row[0] = -target_delta.y / (rt2 * bearing_sigma);
            target_bearing_row[1] = target_delta.x / (rt2 * bearing_sigma);
            target_bearing_row[base] = target_delta.y / (rt2 * bearing_sigma);
            target_bearing_row[base + 1] = -target_delta.x / (rt2 * bearing_sigma);
            target_bearing_row[base + 2] = -1.0 / bearing_sigma;
            jacobian.push_back(std::move(target_bearing_row));
        }
    }
    return jacobian;
}

std::vector<double> residuals_scenario2(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<GlobalBearingMeasurement>& measurements) {
    std::vector<double> residuals;
    residuals.reserve(measurements.size());
    const Vec2 p{state[0], state[1]};

    for (const auto& m : measurements) {
        const Vec2 beacon_est = p - unit_from_angle(m.bt_global) * m.rt;
        residuals.push_back((norm(path[m.time] - beacon_est) - m.rv) / 0.05);
    }

    return residuals;
}

std::vector<double> initial_state_scenario1(
    int beacon_count,
    const Vec2& target_seed,
    double beacon_radius,
    double beacon_yaw) {
    std::vector<double> state{target_seed.x, target_seed.y};
    for (int i = 0; i < beacon_count; ++i) {
        const double angle = 2.0 * kPi * static_cast<double>(i) / std::max(1, beacon_count);
        state.push_back(beacon_radius * std::cos(angle));
        state.push_back(beacon_radius * std::sin(angle));
        state.push_back(beacon_yaw);
    }
    return state;
}

std::vector<double> initial_state_scenario2(const Vec2& target_seed) {
    return {target_seed.x, target_seed.y};
}

std::vector<BeaconEstimate> beacon_estimates_from_scenario1_state(
    const std::vector<double>& state,
    int beacon_count) {
    std::vector<BeaconEstimate> estimates;
    for (int i = 0; i < beacon_count; ++i) {
        const std::size_t base = 2 + 3 * static_cast<std::size_t>(i);
        estimates.push_back({{state[base], state[base + 1]}, wrap_angle(state[base + 2])});
    }
    return estimates;
}

std::vector<BeaconEstimate> beacon_estimates_from_scenario2_measurements(
    const Vec2& target_estimate,
    int beacon_count,
    const std::vector<GlobalBearingMeasurement>& measurements) {
    std::vector<Vec2> sums(static_cast<std::size_t>(beacon_count));
    std::vector<int> counts(static_cast<std::size_t>(beacon_count), 0);

    for (const auto& m : measurements) {
        sums[m.beacon] = sums[m.beacon] + (target_estimate - unit_from_angle(m.bt_global) * m.rt);
        counts[m.beacon] += 1;
    }

    std::vector<BeaconEstimate> estimates;
    for (int i = 0; i < beacon_count; ++i) {
        Vec2 position;
        if (counts[static_cast<std::size_t>(i)] > 0) {
            position = sums[static_cast<std::size_t>(i)] /
                static_cast<double>(counts[static_cast<std::size_t>(i)]);
        }
        estimates.push_back({position, 0.0});
    }
    return estimates;
}

bool two_view_closed_form_initial_state(
    int beacon_count,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    std::vector<double>& state) {
    if (beacon_count <= 0 || path.empty()) {
        return false;
    }

    std::vector<BeaconEstimate> beacon_estimates(static_cast<std::size_t>(beacon_count));
    std::vector<int> target_counts(static_cast<std::size_t>(beacon_count), 0);
    Vec2 target_sum{0.0, 0.0};

    for (int beacon = 0; beacon < beacon_count; ++beacon) {
        const LocalFrameMeasurement* first = nullptr;
        const LocalFrameMeasurement* second = nullptr;
        double best_spread = 0.0;
        for (const auto& a : measurements) {
            if (a.beacon != static_cast<std::size_t>(beacon) || a.time >= path.size()) {
                continue;
            }
            const Vec2 ell_a = local_vehicle_vector(a);
            for (const auto& b : measurements) {
                if (b.beacon != static_cast<std::size_t>(beacon) || b.time >= path.size() || a.time == b.time) {
                    continue;
                }
                const Vec2 ell_b = local_vehicle_vector(b);
                const double spread = norm(ell_b - ell_a);
                if (spread > best_spread) {
                    best_spread = spread;
                    first = &a;
                    second = &b;
                }
            }
        }
        if (first == nullptr || second == nullptr || best_spread < 1e-6) {
            return false;
        }

        const Vec2 ell_a = local_vehicle_vector(*first);
        const Vec2 ell_b = local_vehicle_vector(*second);
        const Vec2 dq = path[second->time] - path[first->time];
        const Vec2 dell = ell_b - ell_a;
        const double yaw = wrap_angle(std::atan2(dq.y, dq.x) - std::atan2(dell.y, dell.x));
        const Vec2 position = path[first->time] - rotate(ell_a, yaw);
        beacon_estimates[static_cast<std::size_t>(beacon)] = {position, yaw};

        for (const auto& m : measurements) {
            if (m.beacon != static_cast<std::size_t>(beacon)) {
                continue;
            }
            target_sum = target_sum + position + rotate(local_target_vector(m), yaw);
            ++target_counts[static_cast<std::size_t>(beacon)];
        }
    }

    int total_target_count = 0;
    for (int count : target_counts) {
        total_target_count += count;
    }
    if (total_target_count <= 0) {
        return false;
    }

    const Vec2 target = target_sum / static_cast<double>(total_target_count);
    state.clear();
    state.push_back(target.x);
    state.push_back(target.y);
    for (const auto& estimate : beacon_estimates) {
        state.push_back(estimate.position.x);
        state.push_back(estimate.position.y);
        state.push_back(estimate.yaw);
    }
    return true;
}

}  // namespace adaptive
