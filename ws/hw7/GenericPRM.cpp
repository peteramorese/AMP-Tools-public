
#include "GenericPRM.h"
#include "hw7helpers.h"
#include <stack>
#include "AStar.h"
#include "hw/HW6.h"
#include "tools/Algorithms.h" 

/**
 * @brief Generates a random vector of length n, where each element is chosen
 * uniformly at random from the corresponding interval [lower_bounds[i],
 * upper_bounds[i]]
 * @param lower_bounds The lower bounds of the intervals
 * @param upper_bounds The upper bounds of the intervals
 * @return A random vector of length n, where each element is within the
 * corresponding interval
 */
Eigen::VectorXd generateRandomNVector(const std::vector<double> &lower_bounds, const std::vector<double> &upper_bounds)
{
    if (lower_bounds.size() != upper_bounds.size())
    {
        throw std::invalid_argument("Erororororororrr");
    }

    int n = lower_bounds.size();
    Eigen::VectorXd random_vector(n);
    std::random_device rd;
    std::mt19937 gen(rd());
    for (int i = 0; i < n; ++i)
    {
        std::uniform_real_distribution<float> dist(lower_bounds[i], upper_bounds[i]);
        random_vector[i] = dist(gen);
    }

    return random_vector;
}

/**
 * @brief Converts a std::vector of doubles to an Eigen::VectorXd
 * @param v The input vector
 * @return The equivalent Eigen::VectorXd
 */
Eigen::VectorXd vectorToEigen(const std::vector<double> &v)
{
    Eigen::VectorXd result(v.size());
    Eigen::VectorXd::Map(&result[0], v.size()) = Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
    return result;
}

std::vector<double> eigenToStdVector(const Eigen::VectorXd& vec){
    std::vector<double> std_vec;
    std_vec.resize(vec.size());
    Eigen::VectorXd::Map(&std_vec[0], vec.size()) = vec;
    return std_vec;
}

std::map<amp::Node, Eigen::Vector2d> amp::GenericPRM::get_nodes(){
    return mynodes;
}

std::shared_ptr<amp::Graph<double>> amp::GenericPRM::get_graph(){
    return mygraph;
}

/**
 * @brief Create a random graph with N nodes, such that each node is connected
 * to all other nodes within a distance of r in the configuration space cspace.
 * @param N The number of nodes in the graph
 * @param r The maximum distance between connected nodes
 * @param cspace The configuration space checker
 * @return A shared pointer to the graph
 */


//TODO THURSDAY
//only connect safe nodes
//implement A* to search for the goal
std::pair<std::shared_ptr<amp::Graph<double>>, std::map<amp::Node, Eigen::VectorXd>> makeGraph(int N, double r, const MyPointCollisionChecker& cspace, const Eigen::VectorXd& init_state, 
                const Eigen::VectorXd& goal_state){
    // std::shared_ptr<amp::Graph<double>> random_graph;
    // std::vector<Eigen::VectorXd> nodes;
    std::shared_ptr<amp::Graph<double>> random_graph = std::make_shared<amp::Graph<double>>();    
    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::vector<Eigen::Vector2d> points;

    const Eigen::VectorXd& lowerbounds = cspace.lowerBounds();
    const Eigen::VectorXd& upperbounds = cspace.upperBounds();
    std::vector<double> lowervector = eigenToStdVector(lowerbounds);
    std::vector<double> uppervector = eigenToStdVector(upperbounds);
    const int dim = lowerbounds.size();
    

    // Generate random nodes
    
    points.push_back(goal_state);
    points.push_back(init_state);
    for (int i = 0; i < N; i++){
        Eigen::VectorXd point = generateRandomNVector(lowervector, uppervector);
        points.push_back(point);
        std::cout << "point: " << points[i].transpose() << "\n";
    }

    for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map

    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;
    for (amp::Node i = 0; i < points.size(); ++i)
        for (amp::Node j = i + 1; j < points.size(); ++j)
            edges.emplace_back(i, j, (points[i] - points[j]).norm());
    
    for (const auto& [from, to, weight] : edges) {
        if (weight < r){
            //TODO: make sure the path is collision-free
            random_graph->connect(from, to, weight); // Connect the edges in the graph
            std::cout << "connected: " << from << "," << to << "\n";
        }
        
    }
    random_graph->print("RANDOM GRAPH");

    // return std::make_pair(random_graph, nodes);
    return std::make_pair(random_graph, nodes);
    //return random_graph;
    
}

namespace amp{

    amp::Path GenericPRM::plan(const Eigen::VectorXd& init_state, 
                const Eigen::VectorXd& goal_state, 
                const MyPointCollisionChecker& collision_checker,
                const DistanceMetric& metric){
            // Implement the sampling-based planner using
            // only these arguments and other hyper parameters ...
            //HYPERPARAMETERS:
            int N = 300;
            double r = 2;

           

            // 1. Create a data structure to store the graph
            
            
            auto [random_graph, nodes] = makeGraph(N, r, collision_checker, init_state, goal_state);
            //olde version:            
            //std::shared_ptr<amp::Graph<double>> random_graph = makeGraph(N, r, collision_checker, init_state, goal_state);
            amp::Path path;
            path.waypoints.push_back(init_state);
            double tol = 2;
            // 5. Perform a graph search to find a path from init_state to goal_state
            amp::Node init_node = random_graph->nodes()[1]; //maybe try printing these
            amp::Node goal_node = random_graph->nodes()[0]; 
            std::stack<amp::Node> stack;
            std::unordered_map<amp::Node, amp::Node> came_from;
            stack.push(init_node);
            came_from[init_node] = init_node;
            while (!stack.empty()) {
                amp::Node current = stack.top();
                stack.pop();
                double dist_to_goal = (nodes[current] -nodes[goal_node]).norm();
                // std::cout << "dist_to_goal: " << dist_to_goal << "\n";
                if (current == goal_node || dist_to_goal < tol) {
                    
                    while (current != init_node) {
                        path.waypoints.push_back(nodes[current]);

                        std::cout << "current: " << nodes[current] << "\n";
                        current = came_from[current];
                    }
                    path.waypoints.push_back(nodes[init_node]); //where hadi's is fucking up
                    std::reverse(path.waypoints.begin(), path.waypoints.end());
                    return path;
                }
                for (const auto& neighbor : random_graph->children(current)) {
                    if (came_from.find(neighbor) == came_from.end()) {
                        // std::cout << "neighbor: " << nodes[neighbor] << "\n";
                        stack.push(neighbor);
                        came_from[neighbor] = current;
                        std::cout << "stack size: " << stack.size() << "\n";
                        
                    }
                }
            }

            //NEXT TODO as of thurs night: use A*
            //need to make a serach heuristic
            LookupSearchHeuristic heuristic = HW6::getEx3Heuristic(); //this is just a placeholder tbh
            MyAStarAlgo algo;
            // amp::Node init_node = random_graph->nodes()[1]; //maybe try printing these
            // amp::Node goal_node = random_graph->nodes()[0]; 
            //make a new heuristi

           
            amp::ShortestPathProblem spp = amp::ShortestPathProblem();
            spp.graph = random_graph;
            spp.init_node = init_node;
            spp.goal_node = goal_node;
            // MyAStarAlgo::GraphSearchResult result = algo.search(spp, heuristic, nodes);            
            // if (result.success) {
            //     std::cout << "Path found" << std::endl;
            //     for (const auto& node : result.node_path) {
            //         path.waypoints.push_back(nodes[node]);
            //     }
            // } else {
            //     std::cout << "Path not found" << std::endl;
            // }
            
            
            // auto [random_graph, nodes] = makeGraph(N, r, collision_checker);
            //Visualizer::makeFigure(problem, amp::Path(), *random_graph, nodes);
            NEW_LINE;
            std::cout << "Path: ";
            for (const auto& waypoint : path.waypoints) {
                // std::cout << waypoint.transpose() << " ";
            }
            for (const auto& node : nodes) {
                mynodes[node.first] = node.second.head<2>();
            }
            mygraph = random_graph;
            return path;
        
}   
}
