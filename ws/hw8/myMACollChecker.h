#pragma once
#include "AMPCore.h"

class MyMACollChecker : public amp::ConfigurationSpace {
    public:
        MyMACollChecker(const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds)
            : ConfigurationSpace(lower_bounds, upper_bounds) {}

        // Implement the pure virtual method
        virtual bool inCollision(const Eigen::VectorXd& cspace_state) const override;
        bool inCollision(amp::MultiAgentProblem2D problem, const Eigen::VectorXd& cspace_state) const;
        bool inCollision(amp::MultiAgentProblem2D problem, const Eigen::VectorXd& startpoint, const Eigen::VectorXd& endpoint) const;
        void set_problem(amp::MultiAgentProblem2D& problem)  {
            myproblem = problem;
        }
        bool diskCollision(Eigen::Vector2d p1, Eigen::Vector2d p2) const;
        amp::MultiAgentProblem2D myproblem; //change this to multiagentproblem
        double robot_radius = 0.5;
        double cautious_radius = 1.2;
        void set_robot_radius(double radius) {robot_radius = radius;}

    private:
        
        
};