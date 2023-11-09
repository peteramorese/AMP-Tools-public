#include "AMPCore.h"
#include <random>
#include "MyMultiAgentPlanners.h"
#include "MyRRT.h"

using namespace amp;
using std::vector, std::cout, Eigen::VectorXd, Eigen::Vector2d, std::pair, std::size_t;
using Regions = vector<vector<Edge>>;

MultiAgentPath2D MyCentralPlanner::plan(const MultiAgentProblem2D& problem) {
    MultiAgentPath2D path;

    int i = 0;
    int m = problem.numAgents();
    VectorXd init(2*m), goal(2*m);
    vector<std::pair<double, double>> limits;
    for (const CircularAgentProperties& agent : problem.agent_properties) {
        init(2*i) = agent.q_init(0);
        init(2*i + 1) = agent.q_init(1);
        goal(2*i) = agent.q_goal(0);
        goal(2*i + 1) = agent.q_goal(1);
        limits.push_back({problem.y_min, problem.y_max});
        limits.push_back({problem.y_min, problem.y_max});
        i++;
    }
    MyCentralChecker collisionChecker(problem);
    MyGenericRRT RRTplanner(n, r, p, limits);
    Path statePath = RRTplanner.plan(init, goal, collisionChecker);
    for (int i = 0; i < m; i++) path.agent_paths.push_back(Path2D());
    for (const VectorXd& state : statePath.waypoints) {
        for (int i = 0; i < m; i++) path.agent_paths[i].waypoints.push_back({state(2*i), state(2*i+1)});
    }
    cout << "State Path Size: "<<statePath.waypoints.size() << std::endl;
    return path;
}


// bool checkRobotOverlap(const VectorXd state, const VectorXd radii) {
//     int m = radii.size();
//     double norm;
//     Vector2d center1, center2;
//     for (int i = 0; i < m; i++) {
//         for (int j = i + 1; j < m; j++) { 
//             center1 = {state(2*i), state(2*i + 1)};
//             center2 = {state(2*j), state(2*j + 1)};
//             norm = (center1 - center2).norm();
//             if (norm < (radii(i) + radii(j))) return true;
//         }
//     }
//     return false;
// }