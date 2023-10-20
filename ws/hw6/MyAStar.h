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
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
        static bool compareCost(const ANode& a, const ANode& b);
    private:
        Node init;
        Node goal;
};