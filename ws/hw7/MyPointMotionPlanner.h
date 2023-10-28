#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"
#include <Eigen/LU>
#include "HelpfulClass.h"




// /// @brief Derive this class and implement your algorithm in the `plan` method. 
// class MyPRM : public HW7::PointMotionPlanner2D {
//     public:
//         /// @brief Solve a motion planning problem. Create a derived class and override this method
//         //virtual amp::Path2D plan(const amp::Problem2D& problem) = 0;

//         virtual ~PRM() {}
// };
class MyPRM : public amp::PointMotionPlanner2D {
    public:
        /// @brief Solve a motion planning problem. Create a derived class and override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override{
            // TODO:
            //1. Sample numSamples times to fill vector of Eigen samples (use rand() % x_max + x_min)
            //1.1. Check that each sample is valid before adding to vector (point polygon check)
            //2. Connect every sample within r radius with straight line (make graph?)
            //2.1. Check that each edge doesn't collide (line polygon check)
            //3. Search graph from start to goal, return node nums, convert to appropriate Eigen Vectors
            // amp::Graph graph();
            // Eigen::Vector2d temp;
            // for(int j = 0; j < n; j++){
            //     temp(0) = 
            // }


            return amp::Path2D();
        }

        amp::Path planND(const amp::Problem2D& problem, const amp::ConfigurationSpace& checker){
            // TODO:
            //1. Sample numSamples times to fill vector of Eigen samples (use rand() % x_max + x_min)
            //1.1. Check that each sample is valid before adding to vector (point polygon check)
            //2. Connect every sample within r radius with straight line (make graph?)
            //2.1. Check that each edge doesn't collide (line polygon check)
            //3. Search graph from start to goal, return node nums, convert to appropriate Eigen Vectors
            return amp::Path();
        }



        // virtual ~PRM() {}
        int& getN(){return numSamples;};
        double& getR(){return radius;};
    private:
        int numSamples = 100;
        double radius = 1;
};

// /// @brief Derive this class and implement your algorithm in the `plan` method. 
// class MyGoalBiasRRT : public HW7::PointMotionPlanner2D {
//     public:
//         /// @brief Solve a motion planning problem. Create a derived class and override this method
//         //virtual amp::Path2D plan(const amp::Problem2D& problem) = 0;

//         virtual ~GoalBiasRRT() {}
// };

class MyGoalBiasRRT : public amp::PointMotionPlanner2D {
    public:
        /// @brief Solve a motion planning problem. Create a derived class and override this method
        amp::Path2D plan(const amp::Problem2D& problem){

            return amp::Path2D();
        }

        // virtual ~GoalBiasRRT() {}
};