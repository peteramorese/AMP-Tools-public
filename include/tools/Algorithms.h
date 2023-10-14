#pragma once

#include <Eigen/Core>

#include "tools/Environment.h" 
#include "tools/Path.h" 
#include "tools/Obstacle.h" 
#include "tools/LinkManipulator.h" 

namespace amp { 

class PointMotionPlanner2D {
    public:
        // Type of planning agent. 
        typedef Eigen::Vector2d AgentType;
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Motion planning problem
        /// @return Path solution of the point agent
        virtual amp::Path2D plan(const amp::Problem2D& problem) = 0;

        virtual ~PointMotionPlanner2D() {}
};

class GeometricMotionPlanner2D {
    public:
        /// @brief Type of planning agent. NOTE: The origin the frame that the polygon vertices are defined in
        /// is the 'relative' point for each waypoint in the solution Path2D
        typedef Polygon AgentType;
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Motion planning problem
        /// @return Path solution of the geometric agent
        virtual amp::Path2D plan(const AgentType& geometric_agent, const amp::Problem2D& problem) = 0;

        virtual ~GeometricMotionPlanner2D() {}
};

class LinkManipulatorMotionPlanner2D {
    public:
        // Type of planning agent.
        typedef LinkManipulator2D AgentType;
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method (2-link manipulator)
        /// @param problem Motion planning problem
        /// @return State-trajectory solution to the problem
        virtual amp::ManipulatorTrajectory2Link plan(const AgentType& link_manipulator_agent, const amp::Problem2D& problem) = 0;

        virtual ~LinkManipulatorMotionPlanner2D() {}
};
}