#pragma once

#include <vector>

#include "adaptive_localization/Types.hpp"

namespace adaptive {

std::vector<double> residuals_scenario1(
    const std::vector<double>& state,
    int beacon_count,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise = Noise{},
    bool repeat_target_packets = true);

std::vector<std::vector<double>> jacobian_scenario1(
    const std::vector<double>& state,
    int beacon_count,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    const Noise& noise = Noise{},
    bool repeat_target_packets = true);

std::vector<double> residuals_scenario2(
    const std::vector<double>& state,
    const std::vector<Vec2>& path,
    const std::vector<GlobalBearingMeasurement>& measurements);

std::vector<double> initial_state_scenario1(
    int beacon_count,
    const Vec2& target_seed,
    double beacon_radius,
    double beacon_yaw);
std::vector<double> initial_state_scenario2(const Vec2& target_seed);

std::vector<BeaconEstimate> beacon_estimates_from_scenario1_state(
    const std::vector<double>& state,
    int beacon_count);

std::vector<BeaconEstimate> beacon_estimates_from_scenario2_measurements(
    const Vec2& target_estimate,
    int beacon_count,
    const std::vector<GlobalBearingMeasurement>& measurements);

bool two_view_closed_form_initial_state(
    int beacon_count,
    const std::vector<Vec2>& path,
    const std::vector<LocalFrameMeasurement>& measurements,
    std::vector<double>& state);

}  // namespace adaptive
