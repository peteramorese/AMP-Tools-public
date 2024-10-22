#pragma once
#include "AMPCore.h"


class DistanceMetric {
    public:
        double distance(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs);
};

// class MyPointCollisionChecker : public amp::ConfigurationSpace2D {
//     public:
//         MyPointCollisionChecker(double x_min, double x_max, double y_min, double y_max) 
//         : amp::ConfigurationSpace2D(x_min, x_max, y_min, y_max) {} //call base class constructor
//         bool inCollision(double x0, double x1) const override;
    
//         };

class MyPointCollisionChecker : public amp::ConfigurationSpace {
    public:
        MyPointCollisionChecker(const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds)
            : ConfigurationSpace(lower_bounds, upper_bounds) {}

        // Implement the pure virtual method
        virtual bool inCollision(const Eigen::VectorXd& cspace_state) const override;
        bool inCollision(amp::Problem2D problem, const Eigen::VectorXd& cspace_state) const;
        bool inCollision(amp::Problem2D problem, const Eigen::VectorXd& startpoint, const Eigen::VectorXd& endpoint) const;
        void set_problem(amp::Problem2D& problem)  {
            myproblem = problem;
        }
        amp::Problem2D myproblem;

    private:
        
};