#include "MyAStar.h"
#include <queue>

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    bool dijkstra = false;
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object

    // TODO: Implement the search method for the A* algorithm
    // 1. Initialize the open list with the initial node
    // 2. While the open list is not empty
    //   2.1. Get the node with the lowest cost from the open list  
    //   2.2. If the current node is the goal node

   
   
    // 2.3. Add the node to the closed list
    // 2.4. Expand the current node by adding all of its neighbors to the open list
    // 2.5. Assign the cost of the path from the initial node to each neighbor as the cost of the neighbor
    // 2.6. Assign the heuristic value of each neighbor as the heuristic value of the neighbor
    // 2.7. If the neighbor is already in the open list, update its cost and heuristic value if the new path is shorter
    
    std::priority_queue<std::pair<double, amp::Node>, std::vector<std::pair<double, amp::Node>>, std::greater<std::pair<double, amp::Node>>> open_list;
    open_list.push(std::make_pair(0.0, problem.init_node));
    //std::map<amp::Node, double, amp::NodeHash> open_map;
    std::unordered_map<double, amp::Node> open_map;
    std::unordered_map<amp::Node, amp::Node> mommies;
    open_map[problem.init_node] = 0.0;
    mommies[problem.init_node] = problem.init_node;
    std::queue<amp::Node> closed_list;
    int numiters = 0;
    while (!open_list.empty()) {
        numiters = numiters + 1;
        amp::Node current_node = open_list.top().second;
        double current_cost = open_list.top().first;
        open_list.pop();
        open_map.erase(current_node);

        if (current_node == problem.goal_node) {
            result.success = true;
            result.path_cost = current_cost;
            break;
        }
        
        //std::cout << "pushing" <<current_node << std::endl;
        closed_list.push(current_node);
        
        int child = 0;
        for (const amp::Node& neighbor : problem.graph->children(current_node)) {
            const auto& outgoing_edges = problem.graph->outgoingEdges(current_node);

            double neighbor_cost = current_cost + outgoing_edges[child];
            //numiters ++;

            if (dijkstra) {
                double neighbor_heuristic = neighbor_cost;
            }
std::cout << "heuristic: " << heuristic(neighbor) << std::endl;
            double neighbor_heuristic = neighbor_cost + heuristic(neighbor);

            
            bool in_open_list = false;
            std::priority_queue<std::pair<double, amp::Node>, std::vector<std::pair<double, amp::Node>>, std::greater<std::pair<double, amp::Node>>> open_copy = open_list;

            
            for (const auto& node : open_map) {
                if (node.second == neighbor) {
                    in_open_list = true;
                    if (node.first > neighbor_heuristic) {
                        open_list.pop();
                        open_map.erase(node.first);
                        open_list.push(std::make_pair(neighbor_heuristic, neighbor));
                        open_map[neighbor_heuristic] = neighbor;
                        mommies[neighbor] = current_node;
                    }
                    break;
                }
            }
            
            if (!in_open_list) {
                open_list.push(std::make_pair(neighbor_heuristic, neighbor));
                open_map[neighbor_heuristic] = neighbor;
                mommies[neighbor] = current_node;
            }
        child++;
        }
    }
   
    // std::priority_queue<std::pair<double, amp::Node>, std::vector<std::pair<double, amp::Node>>, std::greater<std::pair<double, amp::Node>>> open_list;
    // open_list.push(std::make_pair(0.0, problem.init_node));
    // std::queue<amp::Node> closed_list;
    
    result.node_path.push_front(problem.goal_node);
    amp::Node current_node = result.node_path.front();
    while (current_node != problem.init_node) {
        result.node_path.push_front(mommies[current_node]);
        current_node = mommies[current_node];
    }
    
    
    std::cout << numiters << " iterations" << std::endl;
    result.print();
    return result;
}
