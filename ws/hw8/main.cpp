#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiPlanner.h"
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // MyCentralizedMultiAgentRRT cenRRT;
    // amp::MultiAgentProblem2D prob = HW8::getWorkspace1(2);
    // amp::MultiAgentPath2D path = cenRRT.plan(prob);
    // std::vector<std::vector<Eigen::Vector2d>> collision_states;
    // HW8::check(path, prob, collision_states);
    // Visualizer::makeFigure(prob,path, collision_states);

    // MyDecentralizedMultiAgentRRT decRRT;
    // prob = HW8::getWorkspace1(3);
    // do{
    //     path = decRRT.plan(prob);
    //     collision_states.clear();
    // }while(HW8::check(path, prob, collision_states));
    // HW8::check(path, prob, collision_states);
    // Visualizer::makeFigure(prob,path, collision_states);
    // Visualizer::makeFigure(prob,path);

    // {
    //     std::list<std::vector<double>> pathTimes;
    //     std::list<std::vector<double>> pathLens;
    //     std::vector<double> tempT;
    //     std::vector<double> tempL;
    //     std::vector<int> NR = {2,3,4,5,6};
    //     std::vector<std::string> NRLabels = {"2","3","4","5","6"};
    //     for(int n = 0; n < NR.size(); n++){
    //         LOG("numAgents: " << NR[n]);
    //         MyCentralizedMultiAgentRRT cenRRT;
    //         bool plt = true;
    //         int suc = 0;
    //         for(int i = 0; i < 100; i++){
    //             amp::MultiAgentPath2D path = cenRRT.plan(HW8::getWorkspace1(NR[n]));
    //             // if(HW8::check(path,HW8::getWorkspace1(NR[n]),false)){
    //                 suc++;
    //                 tempT.push_back(cenRRT.getT());
    //                 tempL.push_back(cenRRT.getN());
    //             // }
    //             if(HW8::check(path,HW8::getWorkspace1(NR[n]),false) && plt){
    //                 Visualizer::makeFigure(HW8::getWorkspace1(NR[n]),path);
    //                 plt = false;
    //             }
                
    //         }
    //         LOG("numAgents: " << NR[n] << "num success: " << suc);
    //         double sum = accumulate(tempT.begin(), tempT.end(), 0);
    //         double mean = sum / tempT.size();
    //         double sumL = accumulate(tempL.begin(), tempL.end(), 0);
    //         double meanL = sumL / tempL.size();
    //         LOG("numAgents: " << NR[n] << " avg T: " << mean << " avg N: " << meanL << "num success: " << suc);
    //         pathTimes.push_back(tempT);
    //         pathLens.push_back(tempL);
    //     }
    //     Visualizer::makeBoxPlot(pathTimes, NRLabels, std::string("Centralized Planner Runtimes"),std::string("Number of Agents"),std::string("Runtime (ms)"));
    //     Visualizer::makeBoxPlot(pathLens, NRLabels,std::string("Centralized Planner Tree Sizes"), std::string("Number of Agents"),std::string("Tree Size"));
    // }

    // {
    //     std::list<std::vector<double>> pathTimes;
    //     std::vector<double> tempT;
    //     std::vector<int> NR = {2,3,4,5,6};
    //     std::vector<std::string> NRLabels = {"2","3","4","5","6"};
    //     for(int n = 0; n < NR.size(); n++){
    //         // LOG("numAgents: " << NR[n]);
    //         MyDecentralizedMultiAgentRRT cenRRT;
    //         bool plt = true;
    //         int suc = 0;
    //         for(int i = 0; i < 100; i++){
    //             amp::MultiAgentPath2D path = cenRRT.plan(HW8::getWorkspace1(NR[n]));
    //             if(HW8::check(path,HW8::getWorkspace1(NR[n]),false)){
    //                 suc++;
    //                 tempT.push_back(cenRRT.getT());
    //             }
    //             // if(HW8::check(path,HW8::getWorkspace1(NR[n]),false) && plt){
    //             //     Visualizer::makeFigure(HW8::getWorkspace1(NR[n]),path);
    //             //     plt = false;
    //             // }
                
    //         }
    //         double sum = accumulate(tempT.begin(), tempT.end(), 0);
    //         double mean = sum / tempT.size();
    //         LOG("numAgents: " << NR[n] << " avg T: " << mean << " num successes: " << suc);
    //         pathTimes.push_back(tempT);
    //     }
    //     Visualizer::makeBoxPlot(pathTimes, NRLabels, std::string("Decentralized Planner Runtimes"),std::string("Number of Agents"),std::string("Runtime (ms)"));
    // }

    // Visualizer::showFigures();
    MyCentralizedMultiAgentRRT cenRRT;
    MyDecentralizedMultiAgentRRT decRRT;
    HW8::grade(cenRRT, decRRT, "collin.hudson@colorado.edu", argc, argv);
    return 0;
}