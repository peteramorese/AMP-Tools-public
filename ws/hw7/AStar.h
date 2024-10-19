#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW6.h"
#include "hw7helpers.h"

class MyAStarAlgo : public amp::AStar {
    public:
    // i have a few options here. 
    //i can either create a proper heuristic thing that gets the distance. but idt that will work
    //i can also revert back to the original function call and then create another unique function for the three arguments
    //i can also add the cspace to the problem and then use that to get the cspace coordinates
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
        GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic, std::map<amp::Node, Eigen::VectorXd>& node_list);
};