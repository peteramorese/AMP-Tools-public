#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

class MyPRM : public amp::PRM2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
};
