#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/Serializer.h"

namespace amp {

struct Path2D {
    Path2D() = default;
    std::vector<Eigen::Vector2d> waypoints;

    void serialize(Serializer& szr) const;
    void deserialize(const Deserializer& dszr);

    /// @brief Print the object
    /// @param heading Log what type of object is being printed
    void print(const std::string& heading = "Path2D") const;
};

}