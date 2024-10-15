#pragma once
#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw7helpers.h"

namespace amp {
    
class GenericPRM {
    public:
        // amp::Path plan(const Eigen::VectorXd& init_state, 
        //         const Eigen::VectorXd& goal_state, 
        //         const amp::ConfigurationSpace& collision_checker,
        //         const DistanceMetric& metric);

        amp::Path plan(const Eigen::VectorXd& init_state, 
                const Eigen::VectorXd& goal_state, 
                const MyPointCollisionChecker& collision_checker,
                const DistanceMetric& metric);
};

class MyPRM2D : public amp::PRM2D, public GenericPRM {
    public:

        virtual amp::Path2D plan(const amp::Problem2D& problem) override {
            Eigen::VectorXd lower = Eigen::Vector2d{problem.x_min, problem.y_min};
            Eigen::VectorXd upper = Eigen::Vector2d{problem.x_max, problem.y_max};
            // MyPointCollisionChecker cspace = MyPointCollisionChecker(problem.x_min, problem.x_max, problem.y_min, problem.y_max);
            MyPointCollisionChecker cspace = MyPointCollisionChecker(lower, upper);

         
            // amp::ConfigurationSpace MPCC = amp::ConfigurationSpace(lower, upper);

            const DistanceMetric& metric = DistanceMetric();

            // Call the generic planner
            amp::Path path_nd = GenericPRM::plan(problem.q_init, problem.q_goal, cspace, metric);

            // Convert the ND path to a 2D path and return it...
            amp::Path2D path_2d;
            for (const auto& waypoint : path_nd.waypoints) {
                path_2d.waypoints.push_back(waypoint.head<2>());
            }
            return path_2d;
        }


};

}






