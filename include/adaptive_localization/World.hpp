#pragma once

#include <string>
#include <vector>

#include "adaptive_localization/Types.hpp"

namespace adaptive {

World make_world(int beacon_count);
World make_world_with_beacon_separation(double separation);
std::vector<Vec2> make_vehicle_path(int steps);
std::vector<Vec2> make_vehicle_path(int steps, const std::string& trajectory);

}  // namespace adaptive
