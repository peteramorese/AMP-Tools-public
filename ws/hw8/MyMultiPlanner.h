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
            MyGoalBiasRRTND RRT;
            for(int j = 0; j < problem.numAgents(); j++){
                RRT.plan(problem, PathMA2D, j);
            }
            return PathMA2D;
        };
};

