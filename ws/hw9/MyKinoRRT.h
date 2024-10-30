#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

class MyStatePropagator : public amp::StatePropagator {
    public:
        virtual bool propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem) override;
};     