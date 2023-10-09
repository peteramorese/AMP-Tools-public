#pragma once

#include <Eigen/Core>

#include "tools/Environment.h" 
#include "tools/Path.h" 

namespace amp { 

template <class AGENT_T>
class MotionPlanner2D {
    public:
        // Type of planning agent (Point, Polygon, etc.)
        typedef AGENT_T AgentType;
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Motion planning problem
        /// @return Path solution of the point agent
        virtual amp::Path2D plan(const amp::Problem2D& problem) = 0;

        virtual ~MotionPlanner2D() {}
};

using PointAgent2D = Eigen::Vector2d;
using GeometricAgent2D = Polygon;
}