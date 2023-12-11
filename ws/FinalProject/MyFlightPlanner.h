#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW8.h"
#include <Eigen/LU>
#include "HelpfulClass.h"

using Node = uint32_t;

struct UASProblem : public amp::MultiAgentProblem2D {

    //Function for GA positions given time

    std::vector<Eigen::Vector2d> initGA; //vector of initial ground agent positions
    inline int numGA() const {return initGA.size();};
};

class MyFlightPlanner{
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Multi-agent motion planning problem
        /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
        amp::MultiAgentPath2D plan(const UASProblem& problem);

    int& getT(){return time;};
    int& getN(){return numIterations;};
    private:
        int time = 0;
        int numIterations = 20000;
};

