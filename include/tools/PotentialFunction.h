#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/Serializer.h"

namespace amp {

class PotentialFunction2D {
    public:
        /******* User Implemented Methods ********/

        /// @brief Calling operator. Get the value of the potential function at the coordinate `q`. Override this method
        /// to implement your potential function
        /// @param q Coordinate in 2D space to evaluate the potential function value at
        /// @return The potential function value (height)
        virtual double operator()(const Eigen::Vector2d& q) const = 0;

        /*****************************************/
};

}