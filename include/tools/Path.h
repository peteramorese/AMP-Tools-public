#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/Serializer.h"

namespace amp {

/// @brief 2-dimensional path
struct Path2D {
    Path2D() = default;
    std::vector<Eigen::Vector2d> waypoints;

    /// @brief Get the path length
    /// @return Total path length
    double length() const;

    void serialize(Serializer& szr) const;
    void deserialize(const Deserializer& dszr);

    /// @brief Print the object
    /// @param heading Log what type of object is being printed
    void print(const std::string& heading = "Path2D") const;
};

/// @brief N-dimensional path
struct Path {
    Path() = default;
    std::vector<std::vector<double>> waypoints;

    /// @brief Print the object
    /// @param heading Log what type of object is being printed
    void print(const std::string& heading = "Path") const;
};


}