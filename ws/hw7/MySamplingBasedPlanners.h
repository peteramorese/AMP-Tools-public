#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"



class MyRRT : public amp::GoalBiasRRT2D {
    private: 
        std::shared_ptr<amp::Graph<double>> mygraph;
        std::map<amp::Node, Eigen::Vector2d> mynodes;
        amp::Problem2D myproblem;
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        std::map<amp::Node, Eigen::Vector2d> get_nodes() { return mynodes; }
        std::shared_ptr<amp::Graph<double>> get_graph() { return mygraph; }
        void set_problem(const amp::Problem2D& problem) {
            myproblem = problem;
        }
};



// class MyPointCollisionChecker : public amp::GridCSpace2D {
//     public:
//         MyPointCollisionChecker(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
//             : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) // Call base class constructor
//         {}

//         // Override this method for determining which cell a continuous point belongs to
//         virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;
//         std::vector<std::pair<std::size_t, std::size_t>> getGridNeighbors(int i, int j) const override;

// };


