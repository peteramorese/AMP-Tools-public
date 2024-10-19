#include "AStar.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "hw7helpers.h"

struct MyNode
{
    int id;
    float g = 0;                      // Cost from start to this node
    float h = 0;                      // Heuristic estimate of cost from this node to the goal
    float f() const { return g + h; } // Total estimated cost
    std::vector<int> parent;          // To reconstruct the path

    bool operator<(const MyNode &other) const
    {
        return f() > other.f();
    }
};

//this one is dummy
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem &problem, const amp::SearchHeuristic &heuristic){
    std::cout << "shouldn't get here" << "\n";
    GraphSearchResult result = {false, {}, 0.0};
    return result;
}
// Implement the search method for the A* algorithm
//I need to be able to check the actual cspace coordinates of each node. 
//so I'll need the list of points that the nodes are associated with, In other words I'll need the map that maps node IDs to the points. 
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem &problem, const amp::SearchHeuristic &heuristic, std::map<amp::Node, Eigen::VectorXd>& node_list)
{
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0};

    DistanceMetric myMetric = DistanceMetric();

    std::priority_queue<MyNode> openList;

    std::unordered_map<int, MyNode> openSet;

    std::unordered_map<int, MyNode> closedSet;

    std::cout<<"gets here 1" << std::endl;
    MyNode start;
    start.id = problem.init_node;
    std::cout<< "START: " << start.id << std::endl;
    start.g = 0.0;
    start.h = myMetric.distance(node_list[problem.init_node], node_list[problem.goal_node]);
    std::cout << "INIT COORDS: " << node_list[problem.init_node].transpose() << std::endl;
    
    openList.push(start);
    openSet[start.id] = start;

    bool goalFound = false;
    int counter = 0;

    // std::cout<<"gets here 2" << std::endl;
    while (!openList.empty())
    
    {
        // std::cout<<"gets here 3" << std::endl;
        MyNode nBest = openList.top();
        openList.pop();
        // std::cout<<"gets here 4" << std::endl;
        if (closedSet.find(nBest.id) != closedSet.end())
        {
            continue;
        }

        if (openSet.find(nBest.id) != openSet.end() && nBest.g > openSet[nBest.id].g)
        {
            continue;
        }
        // std::cout<<"gets here 5" << std::endl;
        openSet.erase(nBest.id);
        closedSet[nBest.id] = nBest;

        openSet.erase(nBest.id);
        closedSet[nBest.id] = nBest;

        if (nBest.id == problem.goal_node)
        {
            goalFound = true;
            break;
        }
        // std::cout<<"gets here 6" << std::endl;
        auto children = problem.graph->children(nBest.id);
        int i = 0;
        for (const auto &child_id : children)
        {
            // std::cout<<"gets here 7" << std::endl;
            double tentative_g = nBest.g + problem.graph->outgoingEdges(nBest.id)[i];
            i++;

            if (closedSet.find(child_id) != closedSet.end())
            {
                if (tentative_g >= closedSet[child_id].g)
                    continue;
                else
                    closedSet.erase(child_id);
            }
            // std::cout<<"gets here 8" << std::endl;
            if (openSet.find(child_id) == openSet.end() || tentative_g < openSet[child_id].g)
            {

                MyNode child;
                // std::cout<<"gets here 9" << std::endl;
                child.id = child_id;
                child.g = tentative_g;
                std::cout << "child.g" << child.g << std::endl;
                // child.h = heuristic(child.id);
                
                child.h =myMetric.distance(node_list[child_id], node_list[problem.goal_node]);
                std::cout << "Child: " << node_list[child_id].transpose() << ", Goal: " << node_list[problem.goal_node].transpose() << ", Distance: " << child.h << std::endl;
                
                child.parent = nBest.parent;
                
                child.parent.push_back(nBest.id);

                openSet[child_id] = child;
                // std::cout<<"gets here 9.5" << std::endl;
                openList.push(child);
            }
            // std::cout<<"gets here 10" << std::endl;
        }

        counter++;
    }

    if (goalFound)
    {
        result.success = true;
        int itr = 0;

        std::vector<int> path = closedSet[problem.goal_node].parent;
        path.push_back(problem.goal_node);
        for (int node : path)
        {
            std::cout << node << " ";
            result.node_path.push_back(node);
            auto chilren = problem.graph->children(node);

            if (itr < path.size() - 1)
            {
                auto it = std::find(chilren.begin(), chilren.end(), path[itr + 1]);
                int index = std::distance(chilren.begin(), it);
                result.path_cost += problem.graph->outgoingEdges(node)[index];
                itr += 1;
            }
        }
        std::cout << "i found a path woohoo" << std::endl;
        std::cout << std::endl;
    }

    else
    {
        result.success = false;
        std::cout << "No path found" << std::endl;
    }

    result.print();

    std::cout << "Number of iterations: " << counter << std::endl;
    return result;
}