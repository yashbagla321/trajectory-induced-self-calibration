// Measurements.hpp
//
// Synthetic measurement generation: given a ground-truth World and a known
// vehicle path, produces noisy LocalFrameMeasurement (scenario 1,
// uncalibrated local-frame) or GlobalBearingMeasurement (scenario 2,
// calibrated global-frame) records, corrupted by the Gaussian noise model in
// Noise. This is the "sensor simulator" used to feed synthetic data into the
// estimators in Estimators.hpp/Simulation.hpp.

#pragma once

#include <random>
#include <vector>

#include "adaptive_localization/Types.hpp"

namespace adaptive {

/**
 * Generates one scenario-1 (uncalibrated local-frame) measurement packet
 * from beacon `beacon_index` while the robot is at `robot` (recorded under
 * `time_index`). Computes the true range/bearing from the beacon to the
 * robot and to the world's target, expresses the bearings in the beacon's
 * own local frame (subtracting its unknown yaw, world.beacon_yaws), and adds
 * independent Gaussian noise per `noise`. Ranges are floored at 0.05 to avoid
 * degenerate zero/negative ranges.
 */
LocalFrameMeasurement make_local_frame_measurement(
    const World& world,
    const Vec2& robot,
    std::size_t beacon_index,
    std::size_t time_index,
    const Noise& noise,
    std::mt19937& rng);

/**
 * Generates one scenario-2 (calibrated global-frame) measurement packet from
 * beacon `beacon_index` while the robot is at `robot`. Like
 * make_local_frame_measurement, but the bearing to the target is reported
 * directly in the global frame (no beacon-yaw subtraction), matching the
 * "already calibrated" baseline model.
 */
GlobalBearingMeasurement make_global_bearing_measurement(
    const World& world,
    const Vec2& robot,
    std::size_t beacon_index,
    std::size_t time_index,
    const Noise& noise,
    std::mt19937& rng);

/// Generates a full scenario-1 dataset: one LocalFrameMeasurement per
/// (beacon, path-step) pair, in beacon-major, then time-minor order.
std::vector<LocalFrameMeasurement> generate_local_frame_measurements(
    const World& world,
    const std::vector<Vec2>& path,
    const Noise& noise,
    std::mt19937& rng);

/// Generates a full scenario-2 dataset: one GlobalBearingMeasurement per
/// (beacon, path-step) pair, in beacon-major, then time-minor order.
std::vector<GlobalBearingMeasurement> generate_global_bearing_measurements(
    const World& world,
    const std::vector<Vec2>& path,
    const Noise& noise,
    std::mt19937& rng);

}  // namespace adaptive
