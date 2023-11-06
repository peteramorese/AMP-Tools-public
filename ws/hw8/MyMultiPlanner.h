#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW8.h"
#include <Eigen/LU>
#include "HelpfulClass.h"

using Node = uint32_t;

class MyCentralizedMultiAgentRRT : public amp::CentralizedMultiAgentRRT {
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Multi-agent motion planning problem
        /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override{
            MyGoalBiasRRTND RRT;
            RRT.getN() = 50000;
            return RRT.plan(problem);
        };
};

class MyDecentralizedMultiAgentRRT : public amp::DecentralizedMultiAgentRRT {
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Multi-agent motion planning problem
        /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override{
            amp::MultiAgentPath2D PathMA2D;
            Eigen::VectorXd init(2*problem.numAgents());
            Eigen::VectorXd goal(2*problem.numAgents());
            for(int j = 0; j < 2*problem.numAgents(); j += 2){
                init(j) = problem.agent_properties[j/2].q_init(0);
                init(j + 1) = problem.agent_properties[j/2].q_init(1);
                goal(j) = problem.agent_properties[j/2].q_goal(0);
                goal(j + 1) = problem.agent_properties[j/2].q_goal(1);
            }
            for(int j = 0; j < 2*problem.numAgents(); j += 2){
                amp::Path2D tempPath;
                Eigen::Vector2d tempVec(init(j),init(j+1));
                tempPath.waypoints.push_back(tempVec);
                tempVec << goal(j),goal(j+1);
                tempPath.waypoints.push_back(tempVec);
                PathMA2D.agent_paths.push_back(tempPath);
            }
            return PathMA2D;
        };
};

