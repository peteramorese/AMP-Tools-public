#pragma once

#include <Eigen/Core>

#include "tools/Environment.h" 
#include "tools/Path.h" 
#include "tools/Obstacle.h" 
#include "tools/LinkManipulator.h" 
#include "tools/Graph.h" 
#include "tools/AgentTypes.h" 

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

class MultiAgentCircleMotionPlanner2D {
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Multi-agent motion planning problem
        /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) = 0;

        virtual ~MultiAgentCircleMotionPlanner2D() {}
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

class AStar {
    public:
        /// @brief Result struct containing the node path and path cost returned by A*
        struct GraphSearchResult {
            /// @brief Set to `true` if path was found, `false` if no path exists
            bool success = false;

            /// @brief Sequence of nodes where `node_path.front()` must contain init node, and `node_path.back()` must contain the goal node
            std::list<amp::Node> node_path;

            /// @brief Path cost (must equal sum of edge weights along node_path)
            double path_cost;
        };
    public:
        /// @brief Find the shortest path from an init node to a goal node on a graph, while using a heuristic.
        /// @param problem Search problem containing init/goal nodes and graph
        /// @param heuristic Heuristic function that maps each node to a "cost-to-go"
        /// @return The optimal node path and path cost (if the heuristic is admissible)
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) = 0;

        virtual ~AStar() {}
};

}