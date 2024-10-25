
#include "GenericPRM.h"
// #include "../hw7/hw7helpers.h"
#include <stack>
#include "hw/HW6.h"
#include "tools/Algorithms.h" 
// #include "myMACollChecker.h"

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
    // std::vector<double> lower_bounds_copy = {lower_bounds[0], -3};
    // std::vector<double> upper_bounds_copy = {upper_bounds[0], 3};
    // int n = lower_bounds_copy.size();
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

std::map<amp::Node, Eigen::VectorXd> amp::GenericPRM::get_nodes(){
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
std::pair<std::shared_ptr<amp::Graph<double>>, std::map<amp::Node, Eigen::VectorXd>> makeGraph(int N, double r, const MyMACollChecker& cspace, const Eigen::VectorXd& init_state, 
                const Eigen::VectorXd& goal_state, const amp::MultiAgentProblem2D& myproblem){
    // std::shared_ptr<amp::Graph<double>> random_graph;
    // std::vector<Eigen::VectorXd> nodes;
    std::shared_ptr<amp::Graph<double>> random_graph = std::make_shared<amp::Graph<double>>();    
    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::vector<Eigen::VectorXd> points;

    const Eigen::VectorXd& lowerbounds = cspace.lowerBounds();
    const Eigen::VectorXd& upperbounds = cspace.upperBounds();
    std::vector<double> lowervector = eigenToStdVector(lowerbounds);
    std::vector<double> uppervector = eigenToStdVector(upperbounds);
    const int dim = lowerbounds.size();
    

    // Generate random nodes
    
    points.push_back(goal_state);
    points.push_back(init_state);
    for (int i = 2; i < N; i++){
        Eigen::VectorXd point = generateRandomNVector(lowervector, uppervector);
        if (cspace.inCollision(myproblem, point)) {
            // std::cout << "collision at point: " << point.transpose() << "\n";
            continue;
        }
        points.push_back(point);
        // std::cout << "point: " << points[i].transpose() << "\n";
    }
    std::cout << "finished generating random points" << "\n";

    for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map

    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;
    for (amp::Node i = 0; i < points.size(); ++i)
        for (amp::Node j = i + 1; j < points.size(); ++j)
            edges.emplace_back(i, j, (points[i] - points[j]).norm());
    
    for (const auto& [from, to, weight] : edges) {
        if (weight < r){
            //TODO: make sure the path is collision-free
            if (!cspace.inCollision(myproblem, points[from], points[to])) {
                random_graph->connect(from, to, weight); // Connect the edges in the graph
                // std::cout << "connected: " << from << "," << to << "\n";
            }
            
        }
        
    }
    // random_graph->print("RANDOM GRAPH");

    // return std::make_pair(random_graph, nodes);
    return std::make_pair(random_graph, nodes);
    //return random_graph;
    
}

namespace amp{

    amp::Path GenericPRM::plan(const Eigen::VectorXd& init_state, 
                const Eigen::VectorXd& goal_state, 
                const MyMACollChecker& collision_checker){
            // Implement the sampling-based planner using
            // only these arguments and other hyper parameters ...
            //HYPERPARAMETERS:
            int N = 100;
            double r = 1;



           

            // 1. Create a data structure to store the graph
            
            
            auto [random_graph, nodes] = makeGraph(N, r, collision_checker, init_state, goal_state, myproblem);
            std::cout << "graph is done"   << "\n";
            //olde version:            
            //std::shared_ptr<amp::Graph<double>> random_graph = makeGraph(N, r, collision_checker, init_state, goal_state);
            amp::Path path;

            //path.waypoints.push_back(init_state);
            double tol = .5;
            // 5. Perform a graph search to find a path from init_state to goal_state
            amp::Node init_node = random_graph->nodes()[1]; //maybe try printing these
            amp::Node goal_node = random_graph->nodes()[0]; 
            std::stack<amp::Node> stack;
            std::unordered_map<amp::Node, amp::Node> came_from;
            stack.push(init_node);
            came_from[init_node] = init_node;
            double plength = 0;
            std::cout << "graph is created, starting breadth-first search" << "\n";
            // std::cout << "is reversible: "  << random_graph->isReversible() << "\n";
            while (!stack.empty()) {   
                amp::Node current = stack.top();
                // std::cout << "current: " << nodes[current] << "\n";
                stack.pop();
                double dist_to_goal = (nodes[current] -nodes[goal_node]).norm();
                // std::cout << "dist_to_goal: " << dist_to_goal << "\n";
                if (current == goal_node || dist_to_goal < tol) {
                    path.waypoints.push_back(nodes[goal_node]);
                    while (current != init_node) {
                        if (current == goal_node) {
                            plength = plength + (nodes[current] - nodes[came_from[current]]).norm();
                            current = came_from[current];
                            
                            continue;
                        }
                        path.waypoints.push_back(nodes[current]);

                        // std::cout << "pushing waypt: " << nodes[current] << "\n";
                        plength = plength + (nodes[current] - nodes[came_from[current]]).norm();
                        current = came_from[current];
                    }
                    path.waypoints.push_back(nodes[init_node]); //where hadi's is fucking up
                    std::reverse(path.waypoints.begin(), path.waypoints.end());
                    break;
                    // return path;
                }
                for (const auto& neighbor : random_graph->children(current)) {
                    // std::cout << "child: " << nodes[neighbor] << "\n";
                    if (came_from.find(neighbor) == came_from.end()) {
                        // std::cout << "pushing it" << "\n";
                        stack.push(neighbor);
                        came_from[neighbor] = current;
                        // std::cout << "stack size: " << stack.size() << "\n";
                        
                    }
                }
                for (const auto& neighbor : random_graph->parents(current)) {
                    // std::cout << "parent: " << nodes[neighbor] << "\n";
                    if (came_from.find(neighbor) == came_from.end()) {
                        // std::cout << "pushing it" << "\n";
                        stack.push(neighbor);
                        came_from[neighbor] = current;
                        // std::cout << "stack size: " << stack.size() << "\n";
                        
                    }
                }
                // std::cout << "stack size: " << stack.size() << "\n";
           
            }
            bool smoothing = true;
            if (smoothing && path.waypoints.size() > 2) {
                for (int i = 1; i < path.waypoints.size() - 1; ++i) {
                    Eigen::VectorXd p0 = path.waypoints[i-1];
                    Eigen::VectorXd p1 = path.waypoints[i];
                    Eigen::VectorXd p2 = path.waypoints[i+1];
                    if (!collision_checker.inCollision(myproblem, p0, p2)) {
                        path.waypoints.erase(path.waypoints.begin() + i);
                        --i;
                    }
                }
            }
            
            
            for (const auto& node : nodes) {
                mynodes[node.first] = node.second.head<2>();
            }
            mygraph = random_graph;
            // std::cout << "Path length: " << plength << "\n";
            return path;
        
   
}
}
