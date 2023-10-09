#pragma once

#include "AMPCore.h"
#include "hw/HW5.h"
#include <Eigen/LU>

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyGDAlgorithm : public amp::GDAlgorithm {
    public:
        
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        
    private:
        // Add any member variables here...
        
};