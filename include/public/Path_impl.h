#pragma once

#include "tools/Path.h"

void amp::unwrapPath(Path2D& path, const Eigen::Vector2d& wrapped_lower_bounds, const Eigen::Vector2d& wrapped_upper_bounds) {
    unwrapWaypoints<Eigen::Vector2d>(path.waypoints, wrapped_lower_bounds, wrapped_upper_bounds);
}
void amp::unwrapPath(Path& path, const Eigen::VectorXd& wrapped_lower_bounds, const Eigen::VectorXd& wrapped_upper_bounds) {
    unwrapWaypoints<Eigen::VectorXd>(path.waypoints, wrapped_lower_bounds, wrapped_upper_bounds);
}

template <class WAYPOINT_T>
void amp::unwrapWaypoints(std::vector<WAYPOINT_T>& waypoints, const WAYPOINT_T& wrapped_lower_bounds, const WAYPOINT_T& wrapped_upper_bounds) {
    auto it_src = waypoints.cbegin();
    auto it_dst = std::next(waypoints.begin(), 1);
    WAYPOINT_T scale = wrapped_upper_bounds - wrapped_lower_bounds;
    for (; it_dst != waypoints.end();) {
        const WAYPOINT_T& src = *it_src;
        WAYPOINT_T& dst = *it_dst;
        for (uint32_t i = 0; i < it_src->size(); ++i) {
            if (dst[i] < wrapped_lower_bounds[i] || dst[i] > wrapped_upper_bounds[i])
                WARN("Value: " << dst[i] << " is outside the bounds [" << wrapped_lower_bounds[i] << ", " << wrapped_upper_bounds[i] << "] for dimension " << i);
            double n = std::round((src[i] - dst[i]) / (scale[i]));
            dst[i] = dst[i] + n * scale[i];
        }

        ++it_src;
        ++it_dst;
    }
}