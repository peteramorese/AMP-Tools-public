#include "AMPCore.h"
#include "MyAStar.h"
using namespace amp;

MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
        init = problem.init_node;
        goal = problem.goal_node;
        cout << "Init->goal: " << init << " --> " << goal <<"\n";
        ANode currNode, node;
        currNode.ind = init;
        std::list<ANode> queue = {currNode};
        std::map<uint32_t, ANode> processed;
        std::map<uint32_t, ANode> openList;
        openList[init] = currNode;
        // problem.graph->print();
        int steps = 0;
        vector<uint32_t> children;
        vector<double> edges;
        uint32_t index;
        bool success;
        while(true) {
            queue.sort(compareCost);
            currNode = queue.back();
            queue.pop_back();
            openList.erase(currNode.ind);
            // if (processed.count(currNode.ind) > 0) {
            //     if (currNode.cost < processed[currNode.ind].cost) {
            //         cout << currNode.ind << " <-- found better path \n";
            //     }
            //     continue;
            // }
            processed[currNode.ind] = currNode;
            // cout << currNode.ind << " <- Current Node\n";
            children = problem.graph->children(currNode.ind);
            edges = problem.graph->outgoingEdges(currNode.ind);
            for (int i = 0; i < edges.size(); ++i) {
                index = children[i];
                if (processed.count(index) == 0) {
                    double h = (isDijkstra) ? 0 : heuristic(index);
                    node = {index, currNode.ind, currNode.cost + edges[i], currNode.cost + edges[i] + h};
                    if (openList.count(index) == 0) {
                        queue.push_back(node);
                        openList[index] = node;
                        // cout << "Added node " << index << " with cost " << currNode.cost + edges[i] << "\n";
                    } else {
                        // cout << "Node already in list " << index << " with cost " << openList[index].cost << "\nComparing to new path cost: " << currNode.cost + edges[i] << "\n";
                        if (openList[index].cost > currNode.cost + edges[i]) {
                            queue.remove_if([index](const ANode& aNode) {
                                return aNode.ind == index;
                            });   
                            // openList.erase(index);
                            queue.push_back(node);
                            openList[index] = node;
                        }
                    }
                }
            }
            if (queue.empty()) {
                success = false;
                break;
            }
            if (currNode.ind == goal) {
                success = true;
                break;
            }
            steps++;
            if (steps > 10000) break;
        }
        std::list<uint32_t> path;
        index = currNode.ind;
        while (index != init ) {
            path.push_front(index);
            index = processed[index].parent;
            // cout << "Added node " << index << " to path\n";
        }
        path.push_front(init);
        GraphSearchResult result = {success, path, currNode.cost};
        cout << "\nValid path: " << success << "\nPath length: " << path.size() 
             << "\nPath cost: " << result.path_cost << "\nIterations: " << steps << "\n";
        return result;
}

bool MyAStarAlgo::compareCost(const ANode& a, const ANode& b) {
    return a.priority > b.priority;
}