#include "MyAStar.h"

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object
    result.node_path.push_back(problem.init_node);
    result.node_path.push_back(problem.goal_node);
    result.path_cost += 1.0;

    result.print();
    return result;
}
