#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/Serializer.h"

namespace amp {

class StatePropagator {
    public:
        /// @brief Propagate the state forward according to dynamics model
        /// @param state Initial state  
        /// @param control Control input 
        /// @param dt Control duration
        /// @return `true` if the state was successfully propagated, `false` otherwise
        virtual bool propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) = 0;

        virtual ~StatePropagator() {}

};

}

