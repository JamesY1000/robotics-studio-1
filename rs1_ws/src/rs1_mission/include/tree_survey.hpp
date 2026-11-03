#pragma once

#include <cmath>
#include <vector>
#include "rs1_interfaces/msg/tree_survey.hpp"

namespace rs1_mission
{

// A tree only counts once we have a real diameter for its index in this map
inline bool validMeasurements(std::size_t tree_count,
    const std::vector<rs1_interfaces::msg::TreeMeasurement> &measurements)
{
    std::vector<bool> seen(tree_count, false);
    for (const auto &m : measurements)
    {
        if (m.tree_index >= tree_count || seen[m.tree_index] ||
            !std::isfinite(m.dbh_m) || m.dbh_m <= 0.0) return false;
        seen[m.tree_index] = true;
    }
    return true;
}

// Unfinished views belong to an unmeasured tree in this same survey
inline bool validSurveyProgress(std::size_t tree_count,
    const std::vector<rs1_interfaces::msg::TreeMeasurement> &measurements,
    const std::vector<rs1_interfaces::msg::PendingTreeMeasurement> &pending_views)
{
    if (!validMeasurements(tree_count, measurements)) return false;
    std::vector<bool> seen(tree_count, false);
    for (const auto &m : measurements) seen[m.tree_index] = true;
    const auto finite = [](const geometry_msgs::msg::Point &p) {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    };
    for (const auto &view : pending_views)
    {
        if (view.tree_index >= tree_count || seen[view.tree_index] ||
            !std::isfinite(view.dbh_m) || view.dbh_m < 0.45 ||
            !finite(view.trunk) || !finite(view.viewpoint) ||
            std::hypot(view.trunk.x-view.viewpoint.x, view.trunk.y-view.viewpoint.y) < 0.01)
            return false;
        seen[view.tree_index] = true;
    }
    return true;
}

}
