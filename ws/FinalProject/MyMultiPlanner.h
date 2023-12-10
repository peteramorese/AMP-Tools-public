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
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        void updateGraph(){
            
        }
    int& getT(){return time;};
    int& getN(){return numIterations;};
    private:
        int time = 0;
        int numIterations = 20000;
};

class MyDecentralizedMultiAgentRRT : public amp::DecentralizedMultiAgentRRT {
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Multi-agent motion planning problem
        /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override{
            amp::MultiAgentPath2D PathMA2D;
            MyGoalBiasRRTND RRT;
            RRT.getN() = 50000;
            RRT.getS() = 0.5;
            // RRT.getN() = 7500;
            // RRT.getS() = 0.5;
            // RRT.getG() = 0.05;
            // RRT.getE() = 0.25;
            auto start = std::chrono::high_resolution_clock::now();
            for(int j = 0; j < problem.numAgents(); j++){
                RRT.plan(problem, PathMA2D, j);
            }
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
            time = duration.count();
            return PathMA2D;
        };
    int& getT(){return time;};
    private:
        int time = 0;
};

