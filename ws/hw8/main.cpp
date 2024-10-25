// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"

using namespace amp;

void timer_example() {
    double startTime;
    amp::Timer timer("timer");
    for (int i=0; i < 5; ++i) {
        startTime = timer.now(TimeUnit::ms);  
        std::cout << "Press any key to continue...\n";
        std::cin.get();
        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
    }
    timer.stop();
    std::cout << "Total time elapsed: " << Profiler::getTotalProfile("timer") << std::endl;
}

int main(int argc, char** argv) {
    // Run timer example (useful for benchmarking)
    //timer_example();

    // Initialize Workspace 1 with 3 agents
    amp::RNG::seed(amp::RNG::randiUnbounded());
    MultiAgentProblem2D problem = HW8::getWorkspace1(4);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    // Solve using a centralized approach
    MyCentralPlanner central_planner;
    MultiAgentPath2D path = central_planner.plan(problem);
    std::cout << "Path waypoints: ";
    for (int i=0; i < path.agent_paths.size(); ++i) {
        std::cout << "Agent " << i << ": ";
        for (int j=0; j < path.agent_paths[i].waypoints.size(); ++j){
            std::cout << path.agent_paths[i].waypoints[j] << " ";
        }
    }
    std::cout << std::endl;
    bool isValid = HW8::check(path, problem, collision_states);
    Visualizer::makeFigure(problem, path, collision_states);

    // Solve using a decentralized approach
    // MyDecentralPlanner decentral_planner;
    // collision_states = {{}};
    // HW8::generateAndCheck(decentral_planner, path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);

    // Visualize and grade methods
    Visualizer::showFigures();
    // HW8::grade<MyCentralPlanner, MyDecentralPlanner>("shaya.naimi@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}