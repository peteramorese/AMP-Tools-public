// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"
#include "MyRRT.h"

using namespace amp;

void problem1() {
    MultiAgentProblem2D problem = HW8::getWorkspace1(3);
    MyCentralPlanner planner(10000, 0.5, 0.05);
    MultiAgentPath2D path = planner.plan(problem);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;
    bool isValid = HW8::check(path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);
    Visualizer::showFigures();
    // HW8::generateAndCheck(planner);
}

void problem2() {
    MyDecentralPlanner planner(5000, 0.25, 0.05);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;
    MultiAgentProblem2D problem;
    // MultiAgentPath2D path = planner.plan(problem);
    amp::Timer timer("t");
    startTime = timer.now(TimeUnit::ms);

    MultiAgentPath2D path;
    bool isValid = HW8::generateAndCheck(planner, path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);
    Visualizer::showFigures();
}

void testing() {
    std::vector<int> params = {3, 4, 5, 6};
    std::list<std::vector<double>> treeSizeSet; 
    std::list<std::vector<double>> compTimeSet; 
    double pathLength, startTime;
    bool success;
    amp::Timer timer("t");
    for (int m : params) {
        MultiAgentProblem2D problem = HW8::getWorkspace1(m);
        std::vector<double> treeSizes;
        std::vector<double> compTimes;
        for (int i=0; i < 100; ++i) {
            startTime = timer.now(TimeUnit::ms);
            MyCentralPlanner algo(10000, 0.75, 0.05);
            MultiAgentPath2D path = algo.plan(problem);
            if (path.valid) {
                compTimes.push_back(timer.now(TimeUnit::ms) - startTime);
                treeSizes.push_back(algo.treeSize);
            }
        }
        treeSizeSet.push_back(treeSizes);
        compTimeSet.push_back(compTimes);
        double sum1 = std::accumulate(treeSizes.begin(), treeSizes.end(), 0.0);  // Sum of all elements
        double mean1 = sum1 / treeSizes.size(); 
        double sum2 = std::accumulate(compTimes.begin(), compTimes.end(), 0.0);  // Sum of all elements
        double mean2 = sum2 / compTimes.size(); 
        cout << "treeSize + time (ms): " << mean1 << ", " << mean2 << "\n";
    }
    // std::vector<std::string> labels = {"m=1 ", "m=2 "};
    std::vector<std::string> labels = {"m=3 ", "m=4 ", "m=5 ", "m=6 "};
    std::string xlabel = "Number of Agents";
    Visualizer::makeBoxPlot(treeSizeSet, labels, "Tree Size", xlabel, "Nodes in Tree");
    Visualizer::makeBoxPlot(compTimeSet, labels, "Computation Time Benchmark", xlabel, "Computation Time (ms)");
    // Visualizer::makeBarGraph(successSet, labels, "Valid Path Benchmark", xlabel, "Number of Successfull Runs (100 runs)");
    Visualizer::showFigures();
}

int main(int argc, char** argv) {
    // problem1();
    // problem2();
    testing();
    // Visualizer::showFigures();
    // HW8::grade<MyCentralPlanner, MyDecentralPlanner>("yusif.razzaq@colorado.edu", argc, argv, std::make_tuple(10000, 0.5, 0.05), std::make_tuple(5000, 0.10, 0.05));
    return 0;
}