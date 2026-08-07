#include "adaptive_localization/World.hpp"

#include <cmath>
#include <algorithm>

namespace adaptive {

std::vector<Vec2> make_vehicle_path(int steps) {
    return make_vehicle_path(steps, "excited");
}

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
            const double phase = (k % 2 == 0) ? 0.0 : 1.0;
            path.emplace_back(-1.4 + 0.45 * phase, 1.15 - 0.10 * phase);
        } else if (trajectory == "low_curvature_arc") {
            path.emplace_back(-2.2 + 4.4 * s, 0.85 + 0.22 * std::sin(0.5 * t));
        } else if (trajectory == "collinear_pass") {
            path.emplace_back(-2.8 + 5.6 * s, -1.35);
        } else if (trajectory == "line") {
            path.emplace_back(-2.8 + 5.6 * s, 1.4 - 2.8 * s);
        } else if (trajectory == "circle") {
            path.emplace_back(2.1 * std::cos(t), 2.1 * std::sin(t));
        } else if (trajectory == "figure_eight") {
            path.emplace_back(2.4 * std::sin(t), 1.6 * std::sin(t) * std::cos(t));
        } else if (trajectory == "excited_figure_eight") {
            path.emplace_back(
                2.2 * std::sin(t) + 0.45 * std::sin(3.0 * t),
                1.35 * std::sin(t) * std::cos(t) + 0.35 * std::cos(2.0 * t));
        } else {
            path.emplace_back(
                2.2 * std::cos(t) + 0.5 * std::cos(2.0 * t),
                1.6 * std::sin(t) + 0.35 * std::sin(3.0 * t));
        }
    }
    return path;
}

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
