#pragma once
#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw7helpers.h"

namespace amp {
    
class GenericPRM {
    private:

        std::shared_ptr<amp::Graph<double>> mygraph;
        std::map<amp::Node, Eigen::Vector2d> mynodes;
        amp::Problem2D myproblem;
        

        // Add other arguments if needed
    public:
        std::map<amp::Node, Eigen::Vector2d> get_nodes();
        std::shared_ptr<amp::Graph<double>> get_graph();
        

        amp::Path plan(const Eigen::VectorXd& init_state, 
                const Eigen::VectorXd& goal_state, 
                const MyPointCollisionChecker& collision_checker,
                const DistanceMetric& metric);
        int myN;
        double myr;

    protected:
        void set_problem(const amp::Problem2D& problem) {
            myproblem = problem;
        }
};

class MyPRM2D : public amp::PRM2D, public GenericPRM {
    public:

        virtual amp::Path2D plan(const amp::Problem2D& problem) override {
            Eigen::VectorXd lower = Eigen::Vector2d{problem.x_min, problem.y_min};
            Eigen::VectorXd upper = Eigen::Vector2d{problem.x_max, problem.y_max};
            MyPointCollisionChecker cspace = MyPointCollisionChecker(lower, upper);
            const DistanceMetric& metric = DistanceMetric();

            set_problem(problem);

            // Call the generic planner
            amp::Path path_nd = GenericPRM::plan(problem.q_init, problem.q_goal, cspace, metric);

            // Convert the ND path to a 2D path and return it...
            if (path_nd.waypoints.size() == 0) {
                amp::Path2D path_2d;
                path_2d.waypoints.push_back(problem.q_goal);
                return path_2d;
            }
            amp::Path2D path_2d;
            // std::cout << "path_nd length: " << path_nd.waypoints.size() << "" << "\n";
            for (const auto& waypoint : path_nd.waypoints) {
                path_2d.waypoints.push_back(waypoint.head<2>());
                // std::cout << "waypoint: " << waypoint << "\n";
            }

            


            //std::cout << "path length: " << path_2d.length() << "" << "\n";
            return path_2d;
        }


};

}






