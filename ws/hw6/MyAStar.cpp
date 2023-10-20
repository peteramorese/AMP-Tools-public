#include "AMPCore.h"
#include "MyAStar.h"
using namespace amp;

MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
        init = problem.init_node;
        goal = problem.goal_node;
        cout << "Init->goal: " << init << " --> " << goal <<"\n";
        ANode currNode;
        currNode.ind = init;
        std::list<ANode> queue = {currNode};
        std::map<uint32_t, ANode> processed;
        problem.graph->print();
        int steps = 0;
        vector<uint32_t> childern;
        vector<double> edges;
        bool success;
        while(true) {
            queue.sort(compareCost);
            currNode = queue.back();
            queue.pop_back();
            if (processed.count(currNode.ind) > 0) {
                if (currNode.cost < processed[currNode.ind].cost) {
                    cout << currNode.ind << " <-- found better path \n";
                }
                continue;
            }
            processed[currNode.ind] = currNode;
            cout << currNode.ind << " <- Current Node\n";
            childern = problem.graph->children(currNode.ind);
            edges = problem.graph->outgoingEdges(currNode.ind);
            for (int i = 0; i < edges.size(); ++i) {
                queue.push_back({childern[i], currNode.ind, currNode.cost + edges[i], 
                                 currNode.cost + edges[i] + heuristic(childern[i])});
                cout << "Added node " << childern[i] << " with cost " << currNode.cost + edges[i] << "\n";
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
            if (steps > 100) break;
        }
        std::list<uint32_t> path;
        uint32_t node = currNode.ind;
        while (node != init ) {
            path.push_front(node);
            node = processed[node].parent;
            cout << node << " added to path\n";
        }
        GraphSearchResult result = {success, path, currNode.cost};
        cout << "\nValid path: " << success << "\nPath length: " << path.size() 
             << "\nPath cost: " << result.path_cost << "\nIterations: " << steps << "\n";
        return result;
}

bool MyAStarAlgo::compareCost(const ANode& a, const ANode& b) {
    return a.priority > b.priority;
}