#pragma once
#include "AMPCore.h"
#include "hw/HW6.h"
#include "CSpaceConstructor.h"
using namespace amp;

struct ANode {
    uint32_t ind;
    uint32_t parent;
    double cost;
    double priority;
};

class MyAStarAlgo : public amp::AStar {
    public:
        MyAStarAlgo(bool isDijkstra) 
        : isDijkstra(isDijkstra) {}
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
        GraphSearchResult search2(uint32_t init, const std::set<uint32_t> goal, const std::map<uint32_t, std::vector<uint32_t>>& neighborMap, const std::map<uint32_t, std::vector<double>>& weightMap);      
        static bool compareCost(const ANode& a, const ANode& b);
    private:
        Node init;
        Node goal;
        bool isDijkstra;
};