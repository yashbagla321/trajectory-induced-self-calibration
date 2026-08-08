// Measurements.cpp
//
// Concrete synthetic sensor model: turns ground-truth World geometry plus a
// known vehicle path into noisy LocalFrameMeasurement / GlobalBearingMeasurement
// records, adding independent zero-mean Gaussian noise to every range and
// bearing channel per the caller-supplied Noise standard deviations.

#include "adaptive_localization/Measurements.hpp"

#include <algorithm>

namespace adaptive {

namespace {

/// Draws one sample of zero-mean Gaussian noise with standard deviation
/// `sigma`. Returns exactly 0.0 (no distribution constructed) when
/// `sigma <= 0.0`, which is how the rest of the codebase represents a
/// noiseless channel.
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
    // True range/bearing to the vehicle and to the target, each perturbed by
    // independent Gaussian noise. Bearings are expressed relative to the
    // beacon's own (unknown-to-the-estimator) local frame by subtracting its
    // yaw before wrapping to (-pi, pi]. Ranges are floored at 0.05 so noise
    // can never produce a non-physical zero/negative range.
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
    // Same range model as the local-frame measurement, but the bearing to
    // the target is reported directly in the global frame (no yaw
    // subtraction) -- this is what makes scenario 2 the "already calibrated"
    // baseline.
    m.rv = std::max(0.05, norm(robot - beacon) + sample_noise(noise.range_sigma, rng));
    m.rt = std::max(0.05, norm(world.target - beacon) + sample_noise(noise.range_sigma, rng));
    m.bt_global = wrap_angle(bearing(beacon, world.target) + sample_noise(noise.bearing_sigma, rng));
    return m;
}

/// Generates one LocalFrameMeasurement for every (beacon, path-step) pair,
/// i.e. every beacon observes the vehicle at every point along the path.
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

/// Generates one GlobalBearingMeasurement for every (beacon, path-step)
/// pair, mirroring generate_local_frame_measurements for scenario 2.
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
