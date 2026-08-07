#pragma once

#include <random>
#include <vector>

#include "adaptive_localization/Types.hpp"

namespace adaptive {

LocalFrameMeasurement make_local_frame_measurement(
    const World& world,
    const Vec2& robot,
    std::size_t beacon_index,
    std::size_t time_index,
    const Noise& noise,
    std::mt19937& rng);

GlobalBearingMeasurement make_global_bearing_measurement(
    const World& world,
    const Vec2& robot,
    std::size_t beacon_index,
    std::size_t time_index,
    const Noise& noise,
    std::mt19937& rng);

std::vector<LocalFrameMeasurement> generate_local_frame_measurements(
    const World& world,
    const std::vector<Vec2>& path,
    const Noise& noise,
    std::mt19937& rng);

std::vector<GlobalBearingMeasurement> generate_global_bearing_measurements(
    const World& world,
    const std::vector<Vec2>& path,
    const Noise& noise,
    std::mt19937& rng);

}  // namespace adaptive
