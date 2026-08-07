#include "adaptive_localization/Measurements.hpp"

#include <algorithm>

namespace adaptive {

namespace {

double sample_noise(double sigma, std::mt19937& rng) {
    if (sigma <= 0.0) {
        return 0.0;
    }
    std::normal_distribution<double> distribution(0.0, sigma);
    return distribution(rng);
}

}  // namespace

LocalFrameMeasurement make_local_frame_measurement(
    const World& world,
    const Vec2& robot,
    std::size_t beacon_index,
    std::size_t time_index,
    const Noise& noise,
    std::mt19937& rng) {
    const Vec2 beacon = world.beacons[beacon_index];
    const double yaw = world.beacon_yaws[beacon_index];
    LocalFrameMeasurement m;
    m.beacon = beacon_index;
    m.time = time_index;
    m.rv = std::max(0.05, norm(robot - beacon) + sample_noise(noise.range_sigma, rng));
    m.bv_local = wrap_angle(bearing(beacon, robot) - yaw + sample_noise(noise.bearing_sigma, rng));
    m.rt = std::max(0.05, norm(world.target - beacon) + sample_noise(noise.range_sigma, rng));
    m.bt_local = wrap_angle(bearing(beacon, world.target) - yaw + sample_noise(noise.bearing_sigma, rng));
    return m;
}

GlobalBearingMeasurement make_global_bearing_measurement(
    const World& world,
    const Vec2& robot,
    std::size_t beacon_index,
    std::size_t time_index,
    const Noise& noise,
    std::mt19937& rng) {
    const Vec2 beacon = world.beacons[beacon_index];
    GlobalBearingMeasurement m;
    m.beacon = beacon_index;
    m.time = time_index;
    m.rv = std::max(0.05, norm(robot - beacon) + sample_noise(noise.range_sigma, rng));
    m.rt = std::max(0.05, norm(world.target - beacon) + sample_noise(noise.range_sigma, rng));
    m.bt_global = wrap_angle(bearing(beacon, world.target) + sample_noise(noise.bearing_sigma, rng));
    return m;
}

std::vector<LocalFrameMeasurement> generate_local_frame_measurements(
    const World& world,
    const std::vector<Vec2>& path,
    const Noise& noise,
    std::mt19937& rng) {
    std::vector<LocalFrameMeasurement> measurements;
    for (std::size_t i = 0; i < world.beacons.size(); ++i) {
        for (std::size_t k = 0; k < path.size(); ++k) {
            measurements.push_back(make_local_frame_measurement(world, path[k], i, k, noise, rng));
        }
    }
    return measurements;
}

std::vector<GlobalBearingMeasurement> generate_global_bearing_measurements(
    const World& world,
    const std::vector<Vec2>& path,
    const Noise& noise,
    std::mt19937& rng) {
    std::vector<GlobalBearingMeasurement> measurements;
    for (std::size_t i = 0; i < world.beacons.size(); ++i) {
        for (std::size_t k = 0; k < path.size(); ++k) {
            measurements.push_back(make_global_bearing_measurement(world, path[k], i, k, noise, rng));
        }
    }
    return measurements;
}

}  // namespace adaptive
