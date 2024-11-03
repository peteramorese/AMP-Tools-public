
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
    // std::cout << "random vector: " << random_vector.transpose() << "\n";
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


//want to return true if all of the agents' positions are within tol of the goal
bool agentTol(Eigen::VectorXd sample, Eigen::VectorXd goal, double tol){
    int numAgents = goal.size()/2;
    for (int i = 0; i < numAgents; i++){
        Eigen::Vector2d agentState(sample[i * 2], sample[(i * 2) + 1]);
        Eigen::Vector2d agentGoal(goal[i * 2], goal[(i * 2) + 1]);
        if ((agentState - agentGoal).norm() > tol){
            return false;
        }

    }
    return true;
}

bool agentAtGoal(Eigen::VectorXd sample, Eigen::VectorXd goal, int agent, double tol = 0.25){
    int numAgents = goal.size()/2;
    Eigen::Vector2d agentState(sample[agent * 2], sample[(agent * 2) + 1]);
    Eigen::Vector2d agentGoal(goal[agent * 2], goal[(agent * 2) + 1]);
    if ((agentState - agentGoal).norm() < tol){
        return true;
    }
    return false;
}

Eigen::VectorXd adjustSample(Eigen::VectorXd sample, Eigen::VectorXd fixed_goals){
    int numAgents = fixed_goals.size()/2;
    for (int i = 0; i < numAgents; i++){
        Eigen::Vector2d agentState(sample[i * 2], sample[(i * 2) + 1]);
        Eigen::Vector2d agentGoal(fixed_goals[i * 2], fixed_goals[(i * 2) + 1]);
        if (!(agentGoal[0] == -1 && agentGoal[1] == -1)){
            sample[i * 2] = agentGoal[0];
            sample[(i * 2) + 1] = agentGoal[1];
        }
    }

    return sample;

}
/**
 * @brief Create a random graph with N nodes, such that each node is connected
 * to all other nodes within a distance of r in the configuration space cspace.
 * @param N The number of nodes in the graph
 * @param r The maximum distance between connected nodes
 * @param cspace The configuration space checker
 * @return A shared pointer to the graph
 */


std::pair<std::shared_ptr<amp::Graph<double>>, std::map<amp::Node, Eigen::VectorXd>> makeRRTGraph(int N, double step_size, const MyMACollChecker& cspace, const Eigen::VectorXd& init_state, 
     const Eigen::VectorXd& goal_state, const amp::MultiAgentProblem2D& myproblem){
    // std::shared_ptr<amp::Graph<double>> random_graph;
    // std::vector<Eigen::VectorXd> nodes;
    std::shared_ptr<amp::Graph<double>> random_graph = std::make_shared<amp::Graph<double>>();    
    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::vector<Eigen::VectorXd> points;
    Eigen::VectorXd fixed_goals = Eigen::VectorXd::Constant(myproblem.numAgents() * 2, -1);

    const Eigen::VectorXd& lowerbounds = cspace.lowerBounds();
    const Eigen::VectorXd& upperbounds = cspace.upperBounds();
    std::vector<double> lowervector = eigenToStdVector(lowerbounds);
    std::vector<double> uppervector = eigenToStdVector(upperbounds);
    const int dim = lowerbounds.size();

    points.push_back(init_state);
    nodes[0] = init_state;
    int goalBiasCtr = 0;
    int ctr = 0;
    int pointsctr = 1;
    int failoutcount = 0;
    for (int i = 1; i < N; i++){
       

        // if (i % 500 == 1){
        //     std::cout << "iteration: " << i << "\n";
        // }
        // std::cout << "iteration: " << i << "\n";
        Eigen::VectorXd sample = generateRandomNVector(lowervector, uppervector);

        
        if (goalBiasCtr % 20 == 0){ // with a 0.5 chance
            // std::cout << "goal bias" << "\n";
            sample = goal_state;
        }
        // std::cout << "sample: " << sample.transpose() << "\n";
        // else{
        //     int failcount = 0;
        // while (cspace.inCollision(myproblem, sample)) {
        //     failcount++;
        //     if (failcount > 50) {
        //         sample = goal_state;
        //         // N = N - 1;
        //         std::cout << "can't find a valid sample; collision at " << sample.transpose() << "\n";
        //         break;
        //     }
        // // for (int j = 0; j < 10; j++){
        //     // std::cout << "collision at point: " << sample.transpose() << "\n";
        //     sample = generateRandomNVector(lowervector, uppervector);
        //     // std::cout << "can't find a valid sample" << "\n";
        // }
        // }
        // std::cout << "sample: " << sample.transpose() << "\n";
        goalBiasCtr = goalBiasCtr + 1;
        Eigen::VectorXd closest_point;
        int index_counter = 0;
        int closest_index = 0;
        double min_distance = std::numeric_limits<double>::max(); // initialize the minimum distance to positive infinity
        for (const auto& point : points) { // iterate over all points that have been sampled so far
            double distance = (sample - point).norm();
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = point;
                closest_index = index_counter;
            }
            index_counter ++;
        }
        // double stepsize = 0.5; //used to be 0.2
        //FIS THIS SO ITS NOT MIDPOINT AND SO IT COLL CHECKS
        Eigen::VectorXd mid_point = closest_point + (sample - closest_point).normalized() * step_size*1.05;
        double cautious_radius = (closest_point - init_state).norm() < 0.5 ? 0.79 : 2.2;
        // std::cout << "midpoint: " << mid_point.transpose() << "closest point: " << closest_point.transpose() << "sample: " << sample.transpose() << "\n";

        for (int j = 0; j < myproblem.numAgents(); j++){
            if (agentAtGoal(closest_point, goal_state, j)) {
                // std::cout << "fixing agent " << j << " at goal" << "\n";
                mid_point[j*2] = closest_point[j*2];
                mid_point[j*2+1] = closest_point[j*2+1];
                break;
            }
        }

        if (cspace.inCollision(myproblem, mid_point, closest_point) ) {
            if (failoutcount == 1000){ //make 10,000
                std::cout << "hacky adding  " << mid_point.transpose() << "\n";
                 sample = goal_state;
                 mid_point = closest_point + (sample - closest_point).normalized() * 2*step_size;
                points.push_back(mid_point);
                nodes[i] = mid_point;
        
                double distance_to_sample = (mid_point - closest_point).norm();
                // std::cout << "distance to sample: " << distance_to_sample << "\n";
                 // random_graph->connect(closest_index, pointsctr, distance_to_sample); // Connect the edges in the graph
                random_graph->connect(closest_index, i, distance_to_sample); // Connect the edges in the graph
                pointsctr = pointsctr + 1;
                failoutcount++;
                ctr++;
                continue;

                
            }
            
            failoutcount++;
            if (failoutcount > 15000) { //was 50,000
                points.push_back(goal_state);
                nodes[i] = goal_state;
                double distance_to_sample = (goal_state - mid_point).norm();
                // random_graph->connect(pointsctr, pointsctr+1, distance_to_sample);
                random_graph->connect(i-1, i, distance_to_sample);
                ctr++;
                std::cout << "fails out and adds goal" << "\n";
                break;

            }
            else{
                i = i - 1; //make sure we don't increment i
            }
            continue;
        }
        if (agentTol(mid_point, goal_state, .25)) {
            // std::cout << "found goal" << "\n";
            mid_point = goal_state;
            if (cspace.inCollision(myproblem, mid_point, closest_point)) {
                i = i - 1; //make sure we don't increment i
                continue;
            }   
            points.push_back(mid_point);
            nodes[i] = mid_point;
            double distance_to_sample = (mid_point - closest_point).norm();
            random_graph->connect(i, closest_index, distance_to_sample); // Connect the edges in the graph
            points.push_back(goal_state);
            
            nodes[i + 1] = goal_state;
            distance_to_sample = (goal_state - mid_point).norm();
            // random_graph->connect(pointsctr, pointsctr+1, distance_to_sample);
            random_graph->connect(i, i+1, distance_to_sample);
            ctr++;
            std::cout << "finishes adding goal" << "\n";
            break;
        }
        failoutcount = 0;
        points.push_back(mid_point);
        nodes[i] = mid_point;
        
        double distance_to_sample = (mid_point - closest_point).norm();
        // std::cout << "distance to sample: " << distance_to_sample << "\n";
        // random_graph->connect(closest_index, pointsctr, distance_to_sample); // Connect the edges in the graph
        random_graph->connect(closest_index, i, distance_to_sample); // Connect the edges in the graph
        pointsctr = pointsctr + 1;
        ctr++;
        // std::cout << "ctr: " << ctr << "\n";
        // if (i == N-1 && (points.back() - goal_state).norm() > 0.2) {
        //     std::cout << i << " " << (points.back() - goal_state).norm() << "\n";
        //     N = N + 1000;
        //     std::cout << "increasing N" << "\n";
        // }

    }
    std::cout << "make graph ctr: " << ctr << "\n";
    if ((points.back() - goal_state).norm() > 0.1) {
        Eigen::VectorXd closest_point;
        int index_counter = 0;
        int closest_index = 0;
        double min_distance = std::numeric_limits<double>::max(); // initialize the minimum distance to positive infinity
        for (const auto& point : points) { // iterate over all points that have been sampled so far
            double distance = (point - goal_state).norm();
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = point;
                closest_index = index_counter;
            }
            index_counter ++;
        }
        double distance_to_sample = (goal_state - closest_point).norm();
        random_graph->connect(points.size(), closest_index, distance_to_sample); // Connect the edges in the graph
        points.push_back(goal_state);
        nodes[points.size() - 1] = goal_state;
    }
    return std::make_pair(random_graph, nodes);
}
std::pair<std::shared_ptr<amp::Graph<double>>, std::map<amp::Node, Eigen::VectorXd>> makeGraph(int N, double r, const MyMACollChecker& cspace, const Eigen::VectorXd& init_state, 
                const Eigen::VectorXd& goal_state, const amp::MultiAgentProblem2D& myproblem){
    // std::shared_ptr<amp::Graph<double>> random_graph;
    // std::vector<Eigen::VectorXd> nodes;
    std::shared_ptr<amp::Graph<double>> random_graph = std::make_shared<amp::Graph<double>>();    
    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::vector<Eigen::VectorXd> points;

    const Eigen::VectorXd& lowerbounds = cspace.lowerBounds();
    const Eigen::VectorXd& upperbounds = cspace.upperBounds();
    std::cout << "lowerbounds size: " << lowerbounds.size() << "\n";
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
            i = i - 1;
            continue;
        }
        points.push_back(point);
        // std::cout << "point: " << points[i].transpose() << "\n";
    }
    // std::cout << "finished generating random points. size: " << points.size() << "\n" << "\n";


    for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map

    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;
    for (amp::Node i = 0; i < points.size(); ++i)
        for (amp::Node j = i + 1; j < points.size(); ++j)
            edges.emplace_back(i, j, (points[i] - points[j]).norm());
    
    int count = 0;
    for (const auto& [from, to, weight] : edges) {
        if (weight < r) {
            //TODO: make sure the path is collision-free
            if (!cspace.inCollision(myproblem, points[from], points[to])) {
                count++;
                // std::cout << "connected: " << from << "," << to << "\n";
                random_graph->connect(from, to, weight); // Connect the edges in the graph
                // std::cout << "connected: " << nodes[from] << "," << nodes[to] << "\n"; // << from << "," << to << "\n";
            }
            
        }
        
    }
    std::cout << "connected " << count << " edges" << "\n";
    // random_graph->print("RANDOM GRAPH");

    // return std::make_pair(random_graph, nodes);
    return std::make_pair(random_graph, nodes);
    //return random_graph;
    
}

namespace amp{
    //PLANNING FOR CENTRAL PLANNER
    amp::Path GenericPRM::plan(const Eigen::VectorXd& init_state, 
                const Eigen::VectorXd& goal_state, 
                const MyMACollChecker& collision_checker){
            // Implement the sampling-based planner using
            // only these arguments and other hyper parameters ...
            //HYPERPARAMETERS:
            int N = 3000*numAgents;
            double r = 4+(numAgents)*1.5;

            r = std::max(.5, collision_checker.robot_radius / 2);
            // r = 3;
            // N = 8000*numAgents;
            N = myN;
            // N = 50;
            double min_radius = std::numeric_limits<double>::max();
            for (int i = 0; i < numAgents; ++i) {
                double radius = collision_checker.get_agent_radius(i);
                min_radius = std::min(min_radius, radius);
            }
            r = min_radius;
            r = myr;

            // std::cout << "STEP SIZE: " << r << "\n";

            // 1. Create a data structure to store the graph
            
            // auto [random_graph, nodes] = makeGraph(N, r, collision_checker, init_state, goal_state, myproblem);
            auto [random_graph, nodes] = makeRRTGraph(N, r, collision_checker, init_state, goal_state, myproblem);
            treeSize = nodes.size();
            // std::cout << "graph is done"   << "\n";
            //olde version:            
            //std::shared_ptr<amp::Graph<double>> random_graph = makeGraph(N, r, collision_checker, init_state, goal_state);
            amp::Path path;

            //path.waypoints.push_back(init_state);
            
            // std::cout << "nodes in the graph: ";
            for (const auto& node : random_graph->nodes()) {
                // std::cout << nodes[node].transpose() << "\n";
            }
            std::cout << "\n";
            
            
            // 5. Perform a graph search to find a path from init_state to goal_state
            amp::Node init_node = 0; //maybe try printing these
            amp::Node goal_node = nodes.size() - 1;  //not sure if this will work
            // std::cout << "init state: " << nodes[init_node] << "\n";
            // std::cout << "goal state: " << nodes[goal_node] << "\n";
            
            std::stack<amp::Node> stack;
            std::unordered_map<amp::Node, amp::Node> came_from;
            stack.push(init_node);
            came_from[init_node] = init_node;
            double plength = 0;
            // std::cout << "graph is created, starting breadth-first search" << "\n";
            // std::cout << "is reversible: "  << random_graph->isReversible() << "\n";
            double tol = 1;
            while (!stack.empty()) {   
                amp::Node current = stack.top();
                // std::cout << "current: " << nodes[current] << "\n";
                stack.pop();
                double dist_to_goal = (nodes[current]-nodes[goal_node]).norm();
                // std::cout << "dist_to_goal: " << dist_to_goal << "\n";
                if (current == goal_node || agentTol(nodes[current], nodes[goal_node], tol)) {
                    // std::cout << "found goal near " << nodes[current] << "\n";
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
                    // std::cout << "Path1: ";
                    // for (const auto& waypoint : path.waypoints) {
                    //     std::cout << waypoint.transpose() << " ";
                    //     }
                    // break;
                    // return path;
                }
                // std::cout << "number of children: " << random_graph->children(current).size() << "\n";
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
            bool smoothing = false;
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
            
            
            // for (const auto& node : nodes) {
            //     mynodes[node.first] = node.second.head<2>();
            // }
            // std::cout << "Path2: ";
            // for (const auto& waypoint : path.waypoints) {
            //     std::cout << waypoint.transpose() << " ";
            // }
            std::cout << "\n";
            mygraph = random_graph;
            // std::cout << "Path length: " << plength << "\n";
            return path;
        
   
}


amp::Path2D MyRRTDecent::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    return path;
}

std::pair<std::shared_ptr<amp::Graph<double>>, std::map<amp::Node, Eigen::VectorXd>> makeDecentRRTGraph(int N, double r, MyMACollChecker& cspace, int agentid, const amp::MultiAgentProblem2D& myproblem){
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

    //get the initial state and goal state for the agent in question
    Eigen::Vector2d init_state = myproblem.agent_properties[agentid].q_init;
    Eigen::Vector2d goal_state = myproblem.agent_properties[agentid].q_goal;
    // std::cout << "Initial State: " << init_state.transpose() << std::endl;
    // std::cout << "Goal State: " << goal_state.transpose() << std::endl;
    points.push_back(init_state);
    nodes[0] = init_state;
    int goalBiasCtr = 0;
    int failoutcount = 0;
    // std::cout << "STEP SIZE" << r << std::endl;
    for (int i = 1; i < N; i++){
        // std::cout << "iteration: " << i << "\n";
        Eigen::VectorXd sample = generateRandomNVector(lowervector, uppervector);
        if (goalBiasCtr % 20 == 0){ // with a 0.5 chance
            sample = goal_state;
        }
        goalBiasCtr = goalBiasCtr + 1;
        Eigen::Vector2d closest_point;
        int index_counter = 0;
        int closest_index = 0;
        double min_distance = std::numeric_limits<double>::max(); // initialize the minimum distance to positive infinity
        for (const auto& point : points) { // iterate over all points that have been sampled so far
            double distance = (sample - point).norm();
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = point;
                closest_index = index_counter;
            }
            index_counter ++;
        }
        double stepsize = r; //used to be 0.2
        //FIS THIS SO ITS NOT MIDPOINT AND SO IT COLL CHECKS
        double cautious_radius = (closest_point - init_state).norm() < 0.5 ? .75 : 1.5;
        
        Eigen::Vector2d mid_point = closest_point + (sample - closest_point).normalized() * stepsize;
        if (cspace.inCollision(myproblem, mid_point, closest_point) ) {
            if (failoutcount == 10000){
                std::cout << "hackily adding  " << mid_point.transpose() << "\n";
                 sample = goal_state;
                 mid_point = closest_point + (sample - closest_point).normalized() * stepsize;
                points.push_back(mid_point);
                nodes[i] = mid_point;
        
                double distance_to_sample = (mid_point - closest_point).norm();
                // std::cout << "distance to sample: " << distance_to_sample << "\n";
                 // random_graph->connect(closest_index, pointsctr, distance_to_sample); // Connect the edges in the graph
                random_graph->connect(closest_index, i, distance_to_sample); // Connect the edges in the graph
                failoutcount++;
                continue;

                
            }
            
            failoutcount++;
            if (failoutcount > 50000) {
                points.push_back(goal_state);
                nodes[i] = goal_state;
                double distance_to_sample = (goal_state - mid_point).norm();
                // random_graph->connect(pointsctr, pointsctr+1, distance_to_sample);
                random_graph->connect(i-1, i, distance_to_sample);
                std::cout << "fails out and adds goal" << "\n";
                break;

            }
            else{
                i = i - 1; //make sure we don't increment i
            }
            continue;
        }
        if ((mid_point - goal_state).norm() < 0.25) {
            // std::cout << "found goal" << "\n";
            mid_point = goal_state;
            if (cspace.inCollision(myproblem, mid_point, closest_point)) {
                i = i - 1; //make sure we don't increment i
                continue;
            }   
            points.push_back(mid_point);
            nodes[i] = mid_point;
            double distance_to_sample = (mid_point - closest_point).norm();
            random_graph->connect(i, closest_index, distance_to_sample); // Connect the edges in the graph
            std::cout << "finishes adding goal" << "\n";
            break;
        }
        
        points.push_back(mid_point);
        failoutcount = 0;
        nodes[i] = mid_point;
        double distance_to_sample = (mid_point - sample).norm();
        random_graph->connect(i, closest_index, distance_to_sample); // Connect the edges in the graph

    }
    // std::cout << "first node: " << nodes[0].transpose() << "\n";
    
    if (points.back() != goal_state) {
        std::cout << "artifically adding goal " << goal_state.transpose() << "\n";
        Eigen::VectorXd last_node = points.back();
        double min_distance_to_goal = std::numeric_limits<double>::max();
        int closest_index_to_goal = -1;

        for (amp::Node i = 0; i < points.size(); ++i) {
            double distance = (goal_state - points[i]).norm();
            if (distance < min_distance_to_goal) {
                min_distance_to_goal = distance;
                closest_index_to_goal = i;
            }
        }

    points.push_back(goal_state);
    nodes[points.size() - 1] = goal_state;
    random_graph->connect(points.size() - 1, closest_index_to_goal, min_distance_to_goal);
}
    
    return std::make_pair(random_graph, nodes);
    //return random_graph;
    
}


amp::Path2D MyRRTDecent::plan(int agent, const amp::MultiAgentProblem2D& problem,  MyMACollChecker& cspace) {
    amp::Path2D path;
    int N = 30000;
    myproblem = problem;
    double r = cspace.get_agent_radius(agent); //consider fixing this
    cspace.set_cautious_radius(1.1);
    
    // MyMACollChecker cspace = MyMACollChecker(lower, upper);

    auto [random_graph, nodes] = amp::makeDecentRRTGraph(N, r, cspace, agent, myproblem);
    std::cout << "made graph " << "\n";
    // std::cout << "init state: " << problem.q_init.transpose() << "\n";
    // std::cout << "goal state: " << problem.q_goal.transpose() << "\n";
    
    double tol = .7;
    // 5. Perform a graph search to find a path from init_state to goal_state
    amp::Node init_node = random_graph->nodes()[0]; //maybe try printing these
    amp::Node goal_node = random_graph->nodes().back();  //not sure if this will work

    std::cout << "init_node: " << nodes[init_node].transpose() << "\n";
    std::cout << "goal_node: " << nodes[goal_node].transpose() << "\n";

    // std::cout << "starting breadth-first search" << "\n";
    std::stack<amp::Node> stack;
    std::unordered_map<amp::Node, amp::Node> came_from;
    stack.push(init_node);
    came_from[init_node] = init_node;
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
                    current = came_from[current];
                    continue;
                }
                path.waypoints.push_back(nodes[current]);

                // std::cout << "pushing waypt: " << nodes[current] << "\n";
                current = came_from[current];
            }
            path.waypoints.push_back(nodes[init_node]); //where hadi's is fucking up
            // std::cout << "path length: " << path.length() << "\n";
            // std::cout << "init node: " << nodes[init_node] << "\n";
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
    
    }
    // std::cout << "finished breadth-first search" << "\n";
    if (path.waypoints.size() > 0 && (path.waypoints.back() - nodes[goal_node]).norm() > tol) {
        path.waypoints.push_back(nodes[goal_node]);
    }
    

    for (const auto& node : nodes) {
        mynodes[node.first] = node.second.head<2>();
    }
    mygraph = random_graph;

    return path;
}







} //for namespace


