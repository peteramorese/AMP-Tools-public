#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/Serializer.h"

namespace amp {

/// @brief 2-dimensional path
struct Path2D {
    Path2D() = default;
    std::vector<Eigen::Vector2d> waypoints; //ZACK please explain what this std::vector<eigen shit is 

    /// @brief `true` if a solution was found, `false` otherwise
    bool valid;

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
    std::vector<Eigen::VectorXd> waypoints;

    /// @brief `true` if a solution was found, `false` otherwise
    bool valid;

    /// @brief Print the object
    /// @param heading Log what type of object is being printed
    void print(const std::string& heading = "Path") const;
};

struct MultiAgentPath2D {
    MultiAgentPath2D() = default;
    MultiAgentPath2D(uint32_t n_agents) : agent_paths(n_agents) {}
    std::vector<Path2D> agent_paths;

    /// @brief `true` if a solution was found, `false` otherwise
    bool valid;

    inline std::size_t numAgents() const {return agent_paths.size();}
};

/// @brief Given a path that has waypoints wrapped within a hyper-cube from `wrapped_lower_bounds` to `wrapped_upper_bounds`, unwrap the waypoints to
/// extend outside of the hypercube and remove discontinuities. Uses L1-norm between consecutive waypoints
/// @param path Path object (either 2D or ND) that will be edited
/// @param wrapped_lower_bounds Lower bounds of for each dimension that the waypoints were wrapped above
/// @param wrapped_upper_bounds Upper bounds of for each dimension that the waypoints were wrapped below
static void unwrapPath(Path2D& path, const Eigen::Vector2d& wrapped_lower_bounds, const Eigen::Vector2d& wrapped_upper_bounds);
static void unwrapPath(Path& path, const Eigen::VectorXd& wrapped_lower_bounds, const Eigen::VectorXd& wrapped_upper_bounds);

template <class WAYPOINT_T>
static void unwrapWaypoints(std::vector<WAYPOINT_T>& waypoints, const WAYPOINT_T& wrapped_lower_bounds, const WAYPOINT_T& wrapped_upper_bounds);

}


#include "public/Path_impl.h"