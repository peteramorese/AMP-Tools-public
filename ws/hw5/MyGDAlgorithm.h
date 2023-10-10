#pragma once

#include "AMPCore.h"
#include "hw/HW5.h"
#include <Eigen/LU>

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyGDAlgorithm : public amp::GDAlgorithm {
    public:
        
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Split up workspace into regions in front of vertices and in front of edges.
        /*
              v     edge
                 __________ vertex region     
               /           |
         edge /            | edge region
             /             |
            |______________|
           v      edge      v 
        */
        // Allows you to check distance by either using just the vertex or have to use normal vecs to edge

        // Don't need to create vector field plots

        //benchmarks only really worth extra credit 
        
    private:
        // Add any member variables here...
        
};