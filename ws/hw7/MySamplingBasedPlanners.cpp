# include "MySamplingBasedPlanners.h"
// #include "GenericPRM.h"
#include "hw7helpers.h"
#include <stack>



Eigen::VectorXd vectorToEigenRRT(const std::vector<double> &v)
{
    Eigen::VectorXd result(v.size());
    Eigen::VectorXd::Map(&result[0], v.size()) = Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
    return result;
}


std::vector<double> eigenToStdVectorRRT(const Eigen::VectorXd& vec){
    std::vector<double> std_vec;
    std_vec.resize(vec.size());
    Eigen::VectorXd::Map(&std_vec[0], vec.size()) = vec;
    return std_vec;
}

Eigen::VectorXd generateRandomNVectorRRT(const std::vector<double> &lower_bounds, const std::vector<double> &upper_bounds)
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


std::pair<std::shared_ptr<amp::Graph<double>>, std::map<amp::Node, Eigen::VectorXd>> makeRRTGraph(int N, double r, const MyPointCollisionChecker& cspace, const Eigen::VectorXd& init_state, 
     const Eigen::VectorXd& goal_state, const amp::Problem2D& myproblem){
    // std::shared_ptr<amp::Graph<double>> random_graph;
    // std::vector<Eigen::VectorXd> nodes;
    std::shared_ptr<amp::Graph<double>> random_graph = std::make_shared<amp::Graph<double>>();    
    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::vector<Eigen::Vector2d> points;

    const Eigen::VectorXd& lowerbounds = cspace.lowerBounds();
    const Eigen::VectorXd& upperbounds = cspace.upperBounds();
    std::vector<double> lowervector = eigenToStdVectorRRT(lowerbounds);
    std::vector<double> uppervector = eigenToStdVectorRRT(upperbounds);
    const int dim = lowerbounds.size();


    //NOTES FOR MONDAY:
    // I'm sure I'm very close; it's 
    points.push_back(init_state);
    nodes[0] = init_state;
    int goalBiasCtr = 0;
    for (int i = 1; i < N; i++){
        // std::cout << "iteration: " << i << "\n";
        Eigen::VectorXd sample = generateRandomNVectorRRT(lowervector, uppervector);
        while (cspace.inCollision(myproblem, sample)) {
            // std::cout << "collision at point: " << point.transpose() << "\n";
            sample = generateRandomNVectorRRT(lowervector, uppervector);
            //std::cout << "can't find a valid sample" << "\n";
        }
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
        double stepsize = 0.5; //used to be 0.2
        //FIS THIS SO ITS NOT MIDPOINT AND SO IT COLL CHECKS
        Eigen::Vector2d mid_point = closest_point + (sample - closest_point) * stepsize;
        if (cspace.inCollision(myproblem, mid_point, closest_point)) {
            i = i - 1; //make sure we don't increment i
            continue;
        }
        if ((mid_point - goal_state).norm() < 0.25) {
            // std::cout << "found goal" << "\n";
            mid_point = goal_state;
            if (cspace.inCollision(myproblem, mid_point, sample)) {
                i = i - 1; //make sure we don't increment i
                continue;
            }   
            points.push_back(mid_point);
            nodes[i] = mid_point;
            double distance_to_sample = (mid_point - sample).norm();
            random_graph->connect(i, closest_index, distance_to_sample); // Connect the edges in the graph
            // std::cout << "finishes adding goal" << "\n";
            break;
        }
        
        points.push_back(mid_point);
        nodes[i] = mid_point;
        double distance_to_sample = (mid_point - sample).norm();
        random_graph->connect(i, closest_index, distance_to_sample); // Connect the edges in the graph

    }
    // std::cout << "first node: " << nodes[0].transpose() << "\n";
    
    
    return std::make_pair(random_graph, nodes);
    //return random_graph;
    
}

// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    int N = 5000;
    myproblem = problem;
    double r = .5;
    Eigen::VectorXd lower = Eigen::Vector2d{problem.x_min, problem.y_min};
    Eigen::VectorXd upper = Eigen::Vector2d{problem.x_max, problem.y_max};
    MyPointCollisionChecker cspace = MyPointCollisionChecker(lower, upper);
    const DistanceMetric& metric = DistanceMetric();

    auto [random_graph, nodes] = makeRRTGraph(N, r, cspace, problem.q_init, problem.q_goal, myproblem);
    // std::cout << "init state: " << problem.q_init.transpose() << "\n";
    // std::cout << "goal state: " << problem.q_goal.transpose() << "\n";
    
    double tol = .7;
    // 5. Perform a graph search to find a path from init_state to goal_state
    amp::Node init_node = random_graph->nodes()[0]; //maybe try printing these
    amp::Node goal_node = random_graph->nodes().back();  //not sure if this will work



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


    for (const auto& node : nodes) {
        mynodes[node.first] = node.second.head<2>();
    }
    mygraph = random_graph;

    return path;
}

