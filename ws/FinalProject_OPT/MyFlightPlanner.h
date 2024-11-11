#pragma once

#include "AMPCore.h"
// #include "hw/HW7.h"
#include "hw/HW8.h"
#include <Eigen/LU>
#include "HelpfulClass.h"
// #include <algorithm>

using Node = uint32_t;

struct UASProblem : public amp::MultiAgentProblem2D {

    amp::MultiAgentPath2D GApaths; //xy position of ground agents
    int maxTime = 1; //Largest element in endGAt
    int numGA = 1;
    int numUAV = 1;
    double radUAV = 0.2; //radius of disks representing UAVs
    double losLim = 3.0; //range of LOS (signal strength)
    double connectRadius = 2.0; //range that target UAV waypoints can connect to
    double vMin = 0.5;
    double vMax = 0.75;
    double wMin = -2;
    double wMax = 2;

    //Constructor that generates and solves an environment using Decentralized RRT for the Ground Agend positions
    UASProblem(uint32_t n_GA = 3, uint32_t n_UAV = 2, uint32_t n_Obs = 10, double min_Obs = 1.0,
     double max_Obs = 2.0, double size_UAV = 0.2, double los_dist = 3.0, double conRad = 2.0);
    
    void changeNumUAV(int n_UAV);
};


class FlightChecker : public checkPath{
    // Extends from helper class used for path collision checking
    public:
        bool inLOS(const Eigen::Vector2d& state0, const Eigen::Vector2d& state1, const amp::Environment2D& obs){
            return !lineCollision2D(state0, state1, obs);
        };

        void makeLOS(const UASProblem& problem);

        void updateLOS(Eigen::VectorXd& state, const UASProblem& problem, int time);

        bool checkLOS(int numGA);

        void printLOS();

        inline const std::vector<std::set<int>>& getlosGraph() const {return losGraph;};

    private:
        std::vector<std::set<int>> losGraph;

};

class MyFlightPlanner : public MyGoalBiasRRTND{
    public:
        amp::MultiAgentPath2D plan(UASProblem& problem);

        void makeFlightPlan(int minUAV, int maxUAV, int runs, UASProblem& problem);

        bool propState(Eigen::VectorXd& lastState, Eigen::VectorXd& tempInit, const UASProblem& problem, FlightChecker& c);

        bool success = false;

        bool kino = false;
};
