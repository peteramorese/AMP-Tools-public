// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
// #include "MyMultiAgentPlanners.h"
#include "MyRRT.h"

using namespace amp;

void problem1() {
    Problem2D problem = HW2::getWorkspace1();
    vector<pair<double, double>> stateLimits = {{problem.x_min, problem.x_max}, {problem.y_min, problem.y_max}, 
                                                {0, 2*M_PI}, {-1/6, 1/2}, {{-M_PI/6, M_PI/6}}};
    vector<pair<double, double>> controlLimits = {{-1/6, 1/2}, {-M_PI/6, M_PI/6}};
    MyKinoChecker kinoChecker(problem, stateLimits, 0.5, 1);
    KinoRRT RRTplanner(1000, 0.5, 0.05, controlLimits);
    amp::Path path = RRTplanner.plan(problem.q_init, problem.q_goal, kinoChecker);
    if (path.waypoints.size() != 0) {
        Path2D path2d;
        for (const VectorXd point : path.waypoints) path2d.waypoints.push_back({point(0), point(1)});
        Visualizer::makeFigure(problem, path2d);
    }
    // MyCentralPlanner planner(10000, 0.5, 0.05);
    // MultiAgentPath2D path = planner.plan(problem);
    // std::vector<std::vector<Eigen::Vector2d>> collision_states;
    // bool isValid = HW8::check(path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);
    // HW8::generateAndCheck(planner);
}
// void problem2() {
//     MyDecentralPlanner planner(5000, 0.25, 0.05);
//     std::vector<std::vector<Eigen::Vector2d>> collision_states;
//     MultiAgentProblem2D problem = HW8::getWorkspace1(1);
//     amp::Timer timer("t");
//     double startTime = timer.now(TimeUnit::ms);
//     MultiAgentPath2D path = planner.plan(problem);
//     cout << timer.now(TimeUnit::ms) - startTime << "\n";
// }


int main(int argc, char** argv) {
    // problem1();
    problem1();
    Visualizer::showFigures();
    return 0;
}