#include "AMPCore.h"
// #include "hw/HW8.h"
#include "MyFlightPlanner.h"
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());
    bool makeBoxPlot = false;
    UASProblem prob(3,3,30,0.25,4.0,0.1,5,0.75); //UASProblem(n_GA, n_UAV, n_Obs, min_Obs, max_Obs, size_UAV, los_dist, conRad)
    // UASProblem prob(3,3,20,0.75,4.0,0.1,7,0.75);
    MyFlightPlanner fPlanner;
    fPlanner.getN() = 200; //Max number of attempts for finding the next centralized planner state
    fPlanner.makeFlightPlan(1,7,6,prob); //(min number of UAVs,max number of UAVs, max attempts to solve with n UAVs, problem)
    // fPlanner.kino = true;
    // fPlanner.makeFlightPlan(5,8,prob);


        if(makeBoxPlot){
            std::list<std::vector<double>> pathTimes;
            std::list<std::vector<double>> pathLens;
            std::vector<double> tempT;
            // std::vector<double> tempL;
            std::vector<int> NR = {1,2,3,4,5};
            std::vector<std::string> NRLabels = {"1","2","3","4","5"};
            std::vector<double> solns(NR.size(),0);
            for(int n = 0; n < NR.size(); n++){
                MyFlightPlanner fPlanner;
                fPlanner.getN() = 200;
                for(int i = 0; i < 50; i++){
                    UASProblem prob(3,3,50,0.25,4.0,0.1,5,0.75);
                    fPlanner.makeFlightPlan(NR[n],NR[n],1,prob);
                    // fPlanner.kino = true;
                    if(fPlanner.success){
                        tempT.push_back(fPlanner.getT());
                        solns[n] ++;
                    }
                    
                }
                pathTimes.push_back(tempT);
                // pathLens.push_back(tempL);
            }
            Visualizer::makeBoxPlot(pathTimes, NRLabels, std::string("Runtimes"),std::string("Workspace"),std::string("Runtime (ms)"));
            // Visualizer::makeBoxPlot(pathLens, NRLabels,std::string("RRT Path lengths"), std::string("Workspace"),std::string("Path Length"));
            Visualizer::makeBarGraph(solns, NRLabels,std::string("Num Solutions"), std::string("Num UAVs"),std::string("#solutions"));
        }

    Visualizer::showFigures();
}