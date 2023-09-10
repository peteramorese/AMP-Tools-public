#pragma once

#include <Eigen/Core>

#include "tools/Obstacle.h" 
#include "tools/Path.h" 
#include "tools/Serializer.h"

namespace amp { 


/// @brief 2D workspace with rectangular bounds and obstacles
struct Environment2D {
    double x_min = 0.0; 
    double x_max = 10.0; 
    double y_min = 0.0; 
    double y_max = 10.0; 
    std::vector<Obstacle2D> obstacles;

    void serialize(Serializer& szr) const;
    void deserialize(const Deserializer& dszr);

    /// @brief Print the object
    /// @param heading Log what type of object is being printed
    void print(const std::string& heading = "Environment2D") const;
};

/// @brief Environment with initial state and goal state
struct Problem2D : Environment2D {
    Eigen::Vector2d q_init;
    Eigen::Vector2d q_goal;

    void serialize(Serializer& szr) const;
    void deserialize(const Deserializer& dszr);

    /// @brief Print the object
    /// @param heading Log what type of object is being printed
    void print(const std::string& heading = "Problem2D") const;
};

/// @brief Properties that dictate how the environment is generated
struct Random2DEnvironmentSpecification {
    /// @brief Initial state
    Eigen::Vector2d q_init = Eigen::Vector2d(1.0, 1.0);
    /// @brief Goal state
    Eigen::Vector2d q_goal = Eigen::Vector2d(9.0, 9.0);

    /// @brief Left margin of workspace
    double x_min = 0.0; 
    /// @brief Right margin of workspace
    double x_max = 10.0; 
    /// @brief Lower margin of workspace
    double y_min = 0.0; 
    /// @brief Upper margin of workspace
    double y_max = 10.0; 
    /// @brief Number of (possibly-overlapping) convex obstacles
    uint32_t n_obstacles = 20; 

    /// @brief Max number of vertices in an obstalce before convex hull. Increase for more circular obstacles
    uint32_t max_obstacle_vertices = 7;
    /// @brief Largest obstacle region. Increase for larger convex obstacles
    double max_obstacle_region_radius = 2.0;
    /// @brief Smallest obstacle region. Decrease for smaller obstacles and more narrow paths
    double min_obstacle_region_radius = 0.3;
    /// @brief Size of guaranteed path clearance. Calculation is exact
    double path_clearance = 0.5;
    /// @brief Number of random waypoints along goal path. Increasing this will open up larger paths to goal
    uint32_t n_waypoints = 3;
    /// @brief Decrease to get more narrow paths at the expense of computation
    double d_sep = 0.05;

    void serialize(Serializer& szr) const;
    void deserialize(const Deserializer& dszr);
};

class EnvironmentTools {
    public:
        /// @brief Generate a random environment according the the specification
        /// @param spec Specification that dictates how the environment is generated
        /// @return Problem struct with the same init/goal states in spec
        static Problem2D generateRandom(const Random2DEnvironmentSpecification& spec);

        /// @brief Generate a random environment according the the specification
        /// @param spec Specification that dictates how the environment is generated
        /// @param seed Seed the generator to produce the same environment for a given seed
        /// @return Problem struct with the same init/goal states in spec
        static Problem2D generateRandom(const Random2DEnvironmentSpecification& spec, uint32_t seed);
};

}