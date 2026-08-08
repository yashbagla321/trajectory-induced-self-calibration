// World.hpp
//
// Factories for the fixed ground-truth scenarios (target + beacon positions
// and yaws) and for the family of named synthetic vehicle trajectories used
// throughout the Monte Carlo sweeps to vary excitation/observability
// conditions.

#pragma once

#include <string>
#include <vector>

#include "adaptive_localization/Types.hpp"

namespace adaptive {

/// Builds the standard ground-truth World with a fixed target position and
/// up to 4 predefined beacon positions/yaws, using the first `beacon_count`
/// of them (beacon_count is expected to be in [0, 4]; larger values would
/// index past the fixed lists).
World make_world(int beacon_count);

/// Builds a 2-beacon World whose two beacons are placed symmetrically about
/// the fixed target position at the given `separation` distance apart, for
/// use by run_geometry_sweep to study how estimation accuracy depends on
/// beacon spacing.
World make_world_with_beacon_separation(double separation);

/// Equivalent to make_vehicle_path(steps, "excited") -- the default
/// trajectory shape used when no explicit name is given.
std::vector<Vec2> make_vehicle_path(int steps);

/// Generates a `steps`-point synthetic vehicle path parameterized by a named
/// `trajectory` shape (e.g. "stationary", "short_line",
/// "repeated_viewpoints", "low_curvature_arc", "collinear_pass", "line",
/// "circle", "figure_eight", "excited_figure_eight", or the default
/// "excited" multi-frequency Lissajous-like curve). These names are used by
/// the trajectory/near-degenerate sweeps in Simulation.hpp to compare
/// estimation accuracy and observability metrics (rank, S_v) across
/// trajectories that provide varying amounts of excitation, from
/// deliberately degenerate (stationary, collinear, repeated viewpoints) to
/// richly excited.
std::vector<Vec2> make_vehicle_path(int steps, const std::string& trajectory);

}  // namespace adaptive
