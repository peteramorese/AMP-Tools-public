// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"
#include "MyRRT.h"

using namespace amp;

void problem1() {
    MultiAgentProblem2D problem = HW8::getWorkspace1(4);
    MyCentralPlanner planner(7500, 0.5, 0.05);
    MultiAgentPath2D path = planner.plan(problem);
    Visualizer::makeFigure(problem, path);
    Visualizer::showFigures();
    // HW8::generateAndCheck(planner);
}

void problem2() {
    MyDecentralPlanner planner(5000, 0.5, 0.05);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    MultiAgentProblem2D problem = HW8::getWorkspace1(4);
    MultiAgentPath2D path = planner.plan(problem);
    bool isValid = HW8::check(path, problem, collision_states);

    // MultiAgentPath2D path;
    // MultiAgentProblem2D problem;
    // bool isValid = HW8::generateAndCheck(planner, path, problem, collision_states);


    Visualizer::makeFigure(problem, path, collision_states);
    Visualizer::showFigures();
}

void testing() {
    // std::vector<int> params = {1, 2, 3, 4, 5, 6};
    // std::list<std::vector<double>> treeSizeSet; 
    // std::list<std::vector<double>> compTimeSet; 
    // double pathLength, startTime;
    // bool success;
    // amp::Timer timer("t");
    // for (int m : params) {
    //     MultiAgentProblem2D problem = HW8::getWorkspace1(m);
    //     std::vector<double> treeSizes;
    //     std::vector<double> compTimes;
    //     int successCounter = 0;
    //     for (int i=0; i < 100; ++i) {
    //         startTime = timer.now(TimeUnit::ms);
    //         MyCentralPlanner algo(7500, 0.5, 0.05);
    //         MultiAgentProblem2D path = algo.plan(problem);
    //         compTimes.push_back(timer.now(TimeUnit::ms) - startTime);
    //         if (path.waypoints.size() == 0) {
    //             pathLength = 0;
    //         }
    //         else {
    //             successCounter++;
    //             pathLengths.push_back(path.length());
    //         }
    //         // compTimes.push_back(amp::Profiler::getTotalProfile("t",  amp::TimeUnit::ms));
    //     }
    //     pathLengthSet.push_back(pathLengths);
    //     successSet.push_back(successCounter);
    //     compTimeSet.push_back(compTimes);
    // }
}

int main(int argc, char** argv) {
    // problem1();
    problem2();

    // Visualizer::showFigures();
    // HW8::grade<MyCentralPlanner, MyDecentralPlanner>("yusif.razzaq@colorado.edu", argc, argv, std::make_tuple(7500, 1, 0.05), std::make_tuple(1000, 1, 0.05));
    return 0;
}