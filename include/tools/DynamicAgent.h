#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/Serializer.h"

namespace amp {

struct AgentDimensions {
    double length;
    double width;
};

class DynamicAgent {
    public:
        /// @brief Propagate the state forward according to dynamics model
        /// @param state Initial state (updated in place)  
        /// @param control Control input 
        /// @param dt Control duration
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) = 0;
        AgentDimensions agent_dim;
        virtual ~DynamicAgent() {}

};

}

