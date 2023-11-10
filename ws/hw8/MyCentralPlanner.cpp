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
    path.valid = true;
    // for (int i = 0; i < m; i++) path.agent_paths.push_back(Path2D());
    // return path;
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
    MyCentralChecker collisionChecker(problem, true);
    MyGenericRRT RRTplanner(n, r+0.2*m, p, 0.5+0.25*m, limits);
    Path statePath = RRTplanner.plan(init, goal, collisionChecker);
    treeSize = RRTplanner.treeSize;
    if (statePath.waypoints.size() == 0) path.valid = false;
    for (int i = 0; i < m; i++) path.agent_paths.push_back(Path2D());
    for (const VectorXd& state : statePath.waypoints) for (int i = 0; i < m; i++) path.agent_paths[i].waypoints.push_back({state(2*i), state(2*i+1)});
    // cout << "State Path Size: "<< statePath.waypoints.size() << std::endl;
    // Visualizer::makeFigure(problem, path);
    return path;
}

MultiAgentPath2D MyDecentralPlanner::plan(const MultiAgentProblem2D& problem) {
    MultiAgentPath2D path;
    path.valid = true;
    int i = 0;
    int m = problem.numAgents();
    bool goodRun = true;
    VectorXd init(2*m), goal(2*m);
    vector<std::pair<double, double>> limits;
    limits.push_back({problem.y_min, problem.y_max});
    limits.push_back({problem.y_min, problem.y_max});
    MyCentralChecker collisionChecker(problem, false);
    Path agentPath;
    for (const CircularAgentProperties& agent : problem.agent_properties) {
        if (path.valid) {
            MyGenericRRT RRTplanner(n, r, p, 1.0, limits);
            agentPath = RRTplanner.plan(agent.q_init, agent.q_goal, collisionChecker);
            if (agentPath.waypoints.size() == 0) path.valid = false;
        }
        Path2D path2d;
        for (const VectorXd point : agentPath.waypoints) path2d.waypoints.push_back(point);
        path.agent_paths.push_back(path2d);
        collisionChecker.addPath(agentPath);
    }

    // Visualizer::makeFigure(problem, path);
    return path;
}