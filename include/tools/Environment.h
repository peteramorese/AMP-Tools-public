#pragma once

#include <Eigen/Core>

#include "tools/Obstacle.h" 
#include "tools/Path.h" 
#include "tools/Serializer.h"
#include "tools/LinkManipulator.h"

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
    /// @brief Mobile robot: location of reference point on robot
    /// Manipulator: end effector location
    Eigen::Vector2d q_init;
    /// @brief Mobile robot: location of reference point on robot
    /// Manipulator: end effector location
    Eigen::Vector2d q_goal;

    void serialize(Serializer& szr) const;
    void deserialize(const Deserializer& dszr);

    /// @brief Print the object
    /// @param heading Log what type of object is being printed
    void print(const std::string& heading = "Problem2D") const;
};

struct CircularAgentProperties {
    /// @brief Radius of the circular agent
    double radius = 1.0;

    /// @brief Initial location
    Eigen::Vector2d q_init;
    /// @brief Goal location
    Eigen::Vector2d q_goal;
};

/// @brief Environment with a set of agent properties
struct MultiAgentProblem2D : Environment2D {
    /// @brief Properties of each agent
    std::vector<CircularAgentProperties> agent_properties;

    /// @brief Number of agents is given by the number of agent properties
    /// @return Number of circular agents
    inline std::size_t numAgents() const {return agent_properties.size();}
};

struct KinodynamicProblem2D : Environment2D {
    /// @brief State bounds
    std::vector<std::pair<double, double>> q_bounds;
    /// @brief Control input bounds
    std::vector<std::pair<double, double>> u_bounds;
    /// @brief Control duration bounds
    std::pair<double, double> dt_bounds = {0.0, 0.5};  
    /// @brief True if the dimension is Cartesian, false if it's polar
    std::vector<bool> isDimCartesian;
    /// @brief Initial state
    Eigen::VectorXd q_init;
    /// @brief Goal state
    std::vector<std::pair<double, double>> q_goal;
    /// @brief Dynamics function
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamics;
    /// @brief true if point agent, false if polygon agent
    bool isPointAgent = true;
    /// @brief Dimensions of the rectangular agent
    std::pair<double, double> dimensions = {0.0, 0.0};
    
    void serialize(Serializer& szr) const;
    void deserialize(const Deserializer& dszr);
};

/// @brief Properties that dictate how the environment is generated
struct Random2DEnvironmentSpecification {
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
    /// @brief Minimum separation between an initial state and goal state as a fraction of the diagonal distance of the workspace
    double min_diagonal_fraction_goal_sep = 0.6;

    void serialize(Serializer& szr) const;
    void deserialize(const Deserializer& dszr);
};

struct Random2DManipulatorEnvironmentSpecification {
    /// @brief Number of (possibly-overlapping) convex obstacles
    uint32_t n_obstacles = 20; 

    /// @brief Max number of vertices in an obstalce before convex hull. Increase for more circular obstacles
    uint32_t max_obstacle_vertices = 7;
    /// @brief Largest obstacle region. Increase for larger convex obstacles
    double max_obstacle_region_radius = 2.0;
    /// @brief Smallest obstacle region. Decrease for smaller obstacles and more narrow paths
    double min_obstacle_region_radius = 0.3;
    /// @brief Number of random waypoints along goal path. Increasing this will open up larger paths to goal
    uint32_t n_waypoints = 2;
};

struct RandomCircularAgentsSpecification {
    /// @brief Number of agents
    uint32_t n_agents = 3;

    /// @brief Minimum agent radius
    double min_agent_radius = 0.1;

    /// @brief Maximum agent radius
    double max_agent_radius = 0.3;
};

class EnvironmentTools {
    public:
        /// @brief Generate a random environment for a point agent according the the specification
        /// @param spec Specification that dictates how the environment is generated
        /// @param seed Seed the generator to produce the same environment for a given seed (seed = 0 does not reseed the generator)
        /// @return Problem struct
        static Problem2D generateRandomPointAgentProblem(const Random2DEnvironmentSpecification& spec, uint32_t seed = 0u);

        /// @brief Generate a random environment for a manipulator agent according the the specification
        /// @param spec Specification that dictates how the environment is generated
        /// @param manipulator The manipulator to generate the problem around
        /// @param seed Seed the generator to produce the same environment for a given seed (seed = 0 does not reseed the generator)
        /// @return Problem struct
        static Problem2D generateRandomManipulatorProblem(const Random2DManipulatorEnvironmentSpecification& spec, const LinkManipulator2D& manipulator, uint32_t seed = 0u);

        /// @brief Generate a random environment for multiple circular agents according the the specification
        /// @param spec Specification that dictates how the environment is generated
        /// @param ma_spec Specification that dictates the circular agents are generated
        /// @param seed Seed the generator to produce the same environment for a given seed (seed = 0 does not reseed the generator)
        /// @return Problem struct
        static MultiAgentProblem2D generateRandomMultiAgentProblem(const Random2DEnvironmentSpecification& spec, const RandomCircularAgentsSpecification& ma_spec, uint32_t seed = 0u);
};

}