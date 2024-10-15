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
};