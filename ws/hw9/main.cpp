// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
// #include "MyMultiAgentPlanners.h"
#include "KinoRRT.h"
#include <cmath>

using namespace amp;

void problem1() {
    Problem2D problem = HW2::getWorkspace1();
    vector<pair<double, double>> stateLimits = {{problem.x_min, problem.x_max}, {problem.y_min, problem.y_max}, 
                                                {0, 2.0*M_PI}, {-1/6.0, 1/2.0}, {-M_PI/6.0, M_PI/6.0}};
    vector<pair<double, double>> controlLimits = {{-1.0/6.0, 1.0/2.0}, {-M_PI/6.0, M_PI/6.0}};

    Eigen::VectorXd initState(5);
    initState << problem.q_init(0), problem.q_init(1), 1.57, 0.0, 0.0;
    MyKinoChecker kinoChecker(problem, stateLimits, 0.25, 0.5);
    KinoRRT RRTplanner(5000, 0.5, 0.05, controlLimits);
    amp::Path path = RRTplanner.plan(initState, problem.q_goal, kinoChecker);
    if (path.waypoints.size() != 0) {
        Path2D path2d;
        for (const VectorXd point : path.waypoints) path2d.waypoints.push_back({point(0), point(1)});
        Visualizer::makeFigure(problem, path2d);
    }
}

int main(int argc, char** argv) {
    // problem1();
    problem1();
    Visualizer::showFigures();
    return 0;
}