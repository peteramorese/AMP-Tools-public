#include "MyFlightPlanner.h"


using Node = uint32_t;

amp::MultiAgentPath2D MyFlightPlanner::plan(const UASProblem& problem){
    MyGoalBiasRRTND RRT;
    // Centralized Planner parameters
    RRT.getN() = 50000;
    RRT.getS() = 0.5;
    RRT.getG() = 0.15;
    checkPath c;

    const auto noLOS = std::mem_fn(&checkPath::lineCollision2D);

    auto start = std::chrono::high_resolution_clock::now();

    // initialize LOS graph with empty edge sets for each Ground Agent and UAS
    std::vector<std::set<int>> losGraph;
    for(int j = 0; j < (problem.numGA() + problem.numAgents()); j++){
        std::set<int> temp;
        for(int k = 0; k < (problem.numGA() + problem.numAgents()); k++){
            Eigen::Vector2d posj;
            Eigen::Vector2d posk;
            if(j < problem.numGA()){
                posj = problem.initGA[j];
            }
            else{
                posj = problem.agent_properties[j].q_init;
            }
            if(k < problem.numGA()){
                posk = problem.initGA[k];
            }
            else{
                posk = problem.agent_properties[k].q_init;
            }
            if(!noLOS(c,posj,posk,problem)){
                temp.insert(k);
            }
        }
        losGraph.push_back(temp);
    }

    amp::MultiAgentPath2D soln = RRT.plan(problem);












    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
    time = duration.count();
    numIterations = RRT.getN();
    return soln;
}

