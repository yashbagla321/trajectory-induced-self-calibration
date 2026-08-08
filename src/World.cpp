// World.cpp
//
// Concrete definitions of the fixed ground-truth World scenarios and the
// family of named synthetic vehicle trajectories used to vary excitation
// conditions across the Monte Carlo sweeps. All numeric constants here are
// hand-picked scenario geometry, not derived from any configuration.

#include "adaptive_localization/World.hpp"

#include <cmath>
#include <algorithm>

namespace adaptive {

std::vector<Vec2> make_vehicle_path(int steps) {
    return make_vehicle_path(steps, "excited");
}

/**
 * Generates a `steps`-sample path for the named `trajectory` shape. `s` is
 * the normalized progress along the path in [0, 1] and `t = 2*pi*s` is the
 * corresponding angle for the periodic shapes, so `circle`/`figure_eight`/etc.
 * always complete exactly one period over the full path regardless of
 * `steps`. Each branch below hand-codes a shape with deliberately different
 * excitation/observability properties for the self-calibration problem:
 *  - "stationary": the robot never moves -- a maximally degenerate case
 *    (zero trajectory spread, beacon yaw/position are unobservable).
 *  - "short_line": a short straight segment -- limited but nonzero
 *    parallax.
 *  - "repeated_viewpoints": alternates between exactly two fixed points
 *    every other step -- revisits the same two viewpoints repeatedly
 *    instead of sweeping out new geometry.
 *  - "low_curvature_arc": a broad, gently curved sweep.
 *  - "collinear_pass": a straight horizontal pass -- all viewpoints lie on
 *    one line, which is the classic degenerate case for bearing-only
 *    two-view triangulation.
 *  - "line": a longer straight diagonal traverse.
 *  - "circle": a closed circular loop.
 *  - "figure_eight" / "excited_figure_eight": a self-crossing figure-eight,
 *    with the "excited" variant adding higher-frequency harmonics for
 *    richer excitation (more varied aspect angles/curvature over time).
 *  - default ("excited"): a multi-frequency Lissajous-like closed curve
 *    combining two harmonics on each axis, used as the generic
 *    well-excited trajectory throughout the batch Monte Carlo trials.
 */
std::vector<Vec2> make_vehicle_path(int steps, const std::string& trajectory) {
    std::vector<Vec2> path;
    path.reserve(static_cast<std::size_t>(steps));
    for (int k = 0; k < steps; ++k) {
        const double denom = static_cast<double>(std::max(1, steps - 1));
        const double s = static_cast<double>(k) / denom;
        const double t = 2.0 * kPi * s;
        if (trajectory == "stationary") {
            path.emplace_back(-2.2, 1.8);
        } else if (trajectory == "short_line") {
            path.emplace_back(-0.35 + 0.70 * s, 0.95 - 0.20 * s);
        } else if (trajectory == "repeated_viewpoints") {
            // Alternate between two fixed points (k even vs. odd) instead of
            // continuously sweeping -- deliberately revisits the same two
            // viewpoints rather than adding new geometric diversity.
            const double phase = (k % 2 == 0) ? 0.0 : 1.0;
            path.emplace_back(-1.4 + 0.45 * phase, 1.15 - 0.10 * phase);
        } else if (trajectory == "low_curvature_arc") {
            path.emplace_back(-2.2 + 4.4 * s, 0.85 + 0.22 * std::sin(0.5 * t));
        } else if (trajectory == "collinear_pass") {
            // Constant y: every point lies on the same line, which is the
            // classic near-degenerate case for two-view bearing geometry.
            path.emplace_back(-2.8 + 5.6 * s, -1.35);
        } else if (trajectory == "line") {
            path.emplace_back(-2.8 + 5.6 * s, 1.4 - 2.8 * s);
        } else if (trajectory == "circle") {
            path.emplace_back(2.1 * std::cos(t), 2.1 * std::sin(t));
        } else if (trajectory == "figure_eight") {
            path.emplace_back(2.4 * std::sin(t), 1.6 * std::sin(t) * std::cos(t));
        } else if (trajectory == "excited_figure_eight") {
            // Figure-eight base shape plus higher-frequency (3x, 2x) terms
            // to add extra curvature/aspect-angle variation for stronger
            // excitation.
            path.emplace_back(
                2.2 * std::sin(t) + 0.45 * std::sin(3.0 * t),
                1.35 * std::sin(t) * std::cos(t) + 0.35 * std::cos(2.0 * t));
        } else {
            // Default "excited" trajectory: a closed Lissajous-like curve
            // mixing a fundamental and second-harmonic term on each axis.
            path.emplace_back(
                2.2 * std::cos(t) + 0.5 * std::cos(2.0 * t),
                1.6 * std::sin(t) + 0.35 * std::sin(3.0 * t));
        }
    }
    return path;
}

/**
 * Builds the standard ground-truth scenario: a fixed target position and up
 * to 4 hand-placed beacons (each with its own hand-picked, otherwise
 * arbitrary, unknown yaw), taking only the first `beacon_count` entries from
 * the fixed lists below.
 */
World make_world(int beacon_count) {
    World world;
    world.target = {1.2, -0.75};
    const std::vector<Vec2> all_beacons = {
        {-2.2, -1.4},
        {2.4, 1.7},
        {-1.2, 2.3},
        {2.0, -2.1},
    };
    const std::vector<double> all_yaws = {0.75, -1.15, 2.25, -2.65};

    for (int i = 0; i < beacon_count; ++i) {
        world.beacons.push_back(all_beacons.at(static_cast<std::size_t>(i)));
        world.beacon_yaws.push_back(all_yaws.at(static_cast<std::size_t>(i)));
    }
    return world;
}

/**
 * Builds a 2-beacon scenario with the same fixed target as make_world, but
 * with the two beacons placed symmetrically on either side of the target
 * (offset by +-1.2 in y) at a horizontal `separation` apart, for the
 * beacon-geometry sweep (run_geometry_sweep) that studies how estimation
 * accuracy depends on beacon spacing while holding everything else fixed.
 */
World make_world_with_beacon_separation(double separation) {
    World world;
    world.target = {1.2, -0.75};
    const double half = 0.5 * separation;
    world.beacons.push_back({world.target.x - half, world.target.y - 1.2});
    world.beacons.push_back({world.target.x + half, world.target.y + 1.2});
    world.beacon_yaws.push_back(0.75);
    world.beacon_yaws.push_back(-1.15);
    return world;
}

}  // namespace adaptive
