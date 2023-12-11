#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW8.h"
#include <Eigen/LU>
#include "HelpfulClass.h"

using Node = uint32_t;

struct UASProblem : public amp::MultiAgentProblem2D {

    //Function for GA positions given time

    const std::vector<Eigen::Vector2d> initGA; //vector of initial ground agent positions
    const std::vector<Eigen::Vector2d> finalGA; //vector of final ground agent positions
    const std::vector<int> endGAt; //vector of times (number of steps) for each GA to reach finalGA
    const int maxTime; //Largest element in endGAt
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

        void updateLOS( Eigen::VectorXd state, const UASProblem& problem, int time){

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
                    if(inLOS(posj,posk,problem)){
                        temp.insert(k);
                    }
                }
                losGraph.push_back(temp);
            }


        }

        std::vector<std::set<int>> losGraph;

};

