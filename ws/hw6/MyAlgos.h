#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <Eigen/LU>

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override {
            return std::make_unique<MyGridCSpace>(1, 1, 0.0, 1.0, 0.0, 1.0);
        }

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override {
            return amp::Path2D();
        }

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
            return amp::Path2D();
        }
};

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // Default ctor
        MyManipWFAlgo()
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {}

        // You can have custom ctor params for all of these classes
        MyManipWFAlgo(const std::string& beep) 
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {LOG("construcing... " << beep);}

        // This is just to get grade to work, you DO NOT need to override this method
        // virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
        //     return amp::ManipulatorTrajectory2Link();
        // }
        
        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
            return amp::Path2D();
        }
};

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override {
            return GraphSearchResult();
        }
};