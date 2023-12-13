#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW8.h"
#include <Eigen/LU>
#include "HelpfulClass.h"

using Node = uint32_t;

struct UASProblem : public amp::MultiAgentProblem2D {

    //Function for GA positions given time
    std::vector<Eigen::Vector2d> initGA; //vector of initial ground agent positions
    std::vector<Eigen::Vector2d> finalGA; //vector of final ground agent positions
    amp::MultiAgentPath2D GApaths;
    std::vector<int> endGAt; //vector of times (number of steps) for each GA to reach finalGA
    int maxTime = 1; //Largest element in endGAt
    inline int numGA() const {return initGA.size();};
    //Function that returns position of a GA at time t (straight line)
    Eigen::Vector2d GApos(int time, int idxGA) const {
        if(time <= endGAt[idxGA]){
            return initGA[idxGA]*(1 - time/endGAt[idxGA]) + finalGA[idxGA]*(time/endGAt[idxGA]);
        }
        else{
            return finalGA[idxGA];
        }
    };
    UASProblem(uint32_t n_GA = 3, uint32_t n_Obs = 10, double min_Obs = 1.0, double max_Obs = 2.0){

        amp::Random2DEnvironmentSpecification eSpec;
        eSpec.n_obstacles = n_Obs;
        eSpec.min_obstacle_region_radius = min_Obs;
        eSpec.max_obstacle_region_radius = max_Obs;
        amp::RandomCircularAgentsSpecification cSpec;
        cSpec.n_agents = n_GA;
        amp::EnvironmentTools envGen;
        amp::MultiAgentProblem2D randGen = envGen.generateRandomMultiAgentProblem(eSpec, cSpec);
        this->obstacles = randGen.obstacles;
        this->agent_properties = randGen.agent_properties;

        amp::MultiAgentPath2D tempPaths;
        MyGoalBiasRRTND RRT;
        RRT.getN() = 50000;
        RRT.getS() = 0.5;
        for(int j = 0; j < this->numAgents(); j++){
            RRT.plan(randGen, tempPaths, j);
            endGAt.push_back(tempPaths.agent_paths[j].waypoints.size());
        }
        GApaths = tempPaths;


    };
};

class MyFlightPlanner : public MyGoalBiasRRTND{
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Multi-agent motion planning problem
        /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
        amp::MultiAgentPath2D plan(const UASProblem& problem);
};

class FlightChecker : public checkPath{
    public:
        bool inLOS(const Eigen::Vector2d state0, const Eigen::Vector2d state1, const amp::Environment2D& obs){
            return !lineCollision2D(state0, state1, obs);
        }

        void makeLOS(const UASProblem& problem){
            for(int j = 0; j < (problem.numGA() + problem.numAgents()); j++){
                std::set<int> temp;
                for(int k = 0; k < (problem.numGA() + problem.numAgents()); k++){
                    if(j != k){
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
                        if(inLOS(posj,posk,problem)){
                            temp.insert(k);
                        }
                    }
                }
                losGraph.push_back(temp);
            }
        }

        void updateLOS(Eigen::VectorXd state, const UASProblem& problem, int time){
            // Checks LOS between all ground agents and UAS, and ipdates connections stored in losGraph
            for(int j = 0; j < (problem.numGA() + problem.numAgents()); j++){
                std::set<int> temp;
                for(int k = 0; k < (problem.numGA() + problem.numAgents()); k++){
                    if(j != k){
                        Eigen::Vector2d posj;
                        Eigen::Vector2d posk;
                        if(j < problem.numGA()){
                            posj = problem.GApos(time, j);
                        }
                        else{
                            posj(0) = state((j - problem.numGA())*3);
                            posj(1) = state((j - problem.numGA())*3 + 1);
                        }
                        if(k < problem.numGA()){
                            posk = problem.GApos(time, k);
                        }
                        else{
                            posk(0) = state((k - problem.numGA())*3);
                            posk(1) = state((k - problem.numGA())*3 + 1);
                        }
                        if(inLOS(posj,posk,problem)){
                            temp.insert(k);
                        }
                    }
                }
                losGraph.push_back(temp);
            }

        }

        bool checkLOS(int numGA){
            std::set<int> openSet;
            std::set<int> closedSet;
            openSet.insert(0);
            while(!openSet.empty()){
                //add top element of open set to closed set, and erase from open set
                int top = *openSet.begin();
                closedSet.insert(top);
                openSet.erase(openSet.begin());
                //Add children of top to open set if not in closed set
                for (std::set<int>::iterator it=losGraph[top].begin(); it!=losGraph[top].end(); ++it){
                    if(closedSet.find(*it) == closedSet.end()){
                        openSet.insert(*it);
                    }
                }
            }
            // check if closed set has numGA ground agents connected
            auto it = next(closedSet.begin(), numGA - 1);
            return *it == (numGA - 1);
        }


    private:
        std::vector<std::set<int>> losGraph;

};

