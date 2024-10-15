
#include "GenericPRM.h"
#include "hw7helpers.h"


std::shared_ptr<amp::Graph<Eigen::VectorXd>> makeGraph(int N, double r){
    std::shared_ptr<amp::Graph<Eigen::VectorXd>> random_graph;
 
    return random_graph;
    
}

namespace amp{

    amp::Path GenericPRM::plan(const Eigen::VectorXd& init_state, 
                const Eigen::VectorXd& goal_state, 
                const MyPointCollisionChecker& collision_checker,
                const DistanceMetric& metric){
            // Implement the sampling-based planner using
            // only these arguments and other hyper parameters ...
            //HYPERPARAMETERS:
            int N = 10;

            // 1. Create a data structure to store the graph

            std::shared_ptr<amp::Graph<Eigen::VectorXd>> random_graph = makeGraph(N, 2);
            NEW_LINE;
            random_graph->print("Random Graph");
            // x_min = problem.x_min;
            // x_max = problem.x_max;

            // 2. Sample N points in the C-space
            // 3. Check if each point is in collision with the environment
            // 4. Add the N points to the graph
            // 5. For each point, generate K nearest neighbors
            // 6. Check if each neighbor is in collision with the environment
            // 7. Add the valid neighbors to the graph
            // 8. Create a distance metric 
            // 9. Find the shortest path from the start to the goal using the graph

            std::cout << "HELLO WORLD" << std::endl;
            return amp::Path();
        
}   
}
