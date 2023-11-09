// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "MyPRM.h"

using namespace amp;

void problem1a() {
    // Problem2D problem = HW5::getWorkspace1();
    // Problem2D problem = HW2::getWorkspace1();
    Problem2D problem = HW2::getWorkspace2();
    // std::vector<std::pair<int, double>> params = {{200, 0.5}, {200, 1}, {200, 1.5}, {200, 2}, {500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}};
    std::vector<std::pair<int, double>> params = {{200, 1}, {200, 2}, {500, 1}, {500, 2}, {1000, 1}, {1000, 2}};
    std::list<std::vector<double>> pathLengthSet; 
    std::list<std::vector<double>> compTimeSet; 
    std::vector<double> successSet; 
    double pathLength, startTime;
    bool success;
    amp::Timer timer("t");
    for (const std::pair<int, double>& param : params) {
        std::vector<double> pathLengths;
        std::vector<double> compTimes;
        int successCounter = 0;
        for (int i=0; i < 100; ++i) {
            startTime = timer.now(TimeUnit::ms);
            MyPRM algo(param.first, param.second, false);
            Path2D path = algo.plan(problem);
            compTimes.push_back(timer.now(TimeUnit::ms) - startTime);
            if (path.waypoints.size() == 0) {
                pathLength = 0;
            }
            else {
                successCounter++;
                pathLengths.push_back(path.length());
            }
            // compTimes.push_back(amp::Profiler::getTotalProfile("t",  amp::TimeUnit::ms));
        }
        pathLengthSet.push_back(pathLengths);
        successSet.push_back(successCounter);
        compTimeSet.push_back(compTimes);
    }
    // std::vector<std::string> labels = {"(200, 0.5) ", "(200, 1) ", "(200, 1.5)", "(200, 2)", "(500, 0.5)", "(500, 1) ", "(500, 1.5) ", "(500, 2) "};
    std::vector<std::string> labels = {"(200, 1) ", "(200, 2) ", "(500, 1)", "(500, 2)", "(1000, 1)", "(1000, 2) "};
    std::string xlabel = "(n, r)";
    Visualizer::makeBoxPlot(pathLengthSet, labels, "Path Length", xlabel, "Path Length");
    Visualizer::makeBoxPlot(compTimeSet, labels, "Computation Time Benchmark", xlabel, "Computation Time (ms)");
    Visualizer::makeBarGraph(successSet, labels, "Valid Path Benchmark", xlabel, "Number of Successfull Runs (100 runs)");

}

void problem1b() {
    Path2D path;
    Problem2D problem = HW2::getWorkspace1();
    // Problem2D problem = HW2::getWorkspace2();

    MyPRM algo(500, 2, false);
    path = algo.plan(problem);
    if (path.waypoints.size() !=0) {
        Visualizer::makeFigure(problem, path);

        Visualizer::makeFigure(problem, *algo.getGraph(), algo.getPoints());
        
        cout << "Path length: " << path.length() << "\n";
        smoothPath(path, problem.obstacles);
        cout << "Path length: " << path.length() << "\n";
        Visualizer::makeFigure(problem, path);
    } else {
        cout << "no path found\n";
    }
}

void problem2() {
    Problem2D problem = HW5::getWorkspace1();
    // Problem2D problem = HW2::getWorkspace1();
    // Problem2D problem = HW2::getWorkspace2();
    MyRRT algo(500, 1, false);
    Path2D path = algo.plan(problem);
    cout << "Path length: " << path.length() << "\n";
    Visualizer::makeFigure(problem, path);
    Visualizer::makeFigure(problem, *algo.getGraph(), algo.getPoints());
    smoothPath(path, problem.obstacles);
    cout << "Path length: " << path.length() << "\n";
    Visualizer::makeFigure(problem, path);
    

}


int main(int argc, char** argv) {
    HW7::hint();
    // problem1a();
    // problem1b();
    problem2();
    Visualizer::showFigures();
    // HW7::grade<MyPRM, MyRRT>("yusif.razzaq@colorado.edu", argc, argv, std::make_tuple(500, 2, true), std::make_tuple(1000, 2, true));
    return 0;
}