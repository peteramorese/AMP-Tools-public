#include "AMPCore.h"
#include <random>
#include "MyMultiAgentPlanners.h"

using namespace amp;
using std::vector, std::cout, Eigen::VectorXd, Eigen::Vector2d, std::pair, std::size_t;
using Regions = vector<vector<double>>;

MultiAgentPath2D MyCentralPlanner::plan(const MultiAgentProblem2D& problem) {
    MultiAgentPath2D path;
    cout << problem.agent_properties[0].radius << "\n";
    cout << "Bing\n";
    return path;
}
void MyCentralPlanner::init(const MultiAgentProblem2D& problem) {
    m = problem.numAgents();
    int i = 0;
    for (const CircularAgentProperties& agent : problem.agent_properties) {
        radii.push_back(agent.radius);
        init(2*i) = agent.q_init(0);
        init(2*i + 1) = agent.q_init(1);
        goal(2*i) = agent.q_goal(0);
        goal(2*i + 1) = agent.q_goal(1);
        i++;
    }
}

bool checkRobotOverlap(const VectorXd state, const VectorXd radii) {
    int m = radii.size();
    double norm;
    Vector2d center1, center2;
    for (int i = 0; i < agents; i++) {
        for (int j = i + 1; j < agents; j++) { 
            center1 = {state(2*i), state(2*i + 1)};
            center2 = {state(2*j), state(2*j + 1)};
            norm = (center1 - center2).norm();
            if (norm < (radii(i) + radii(j))) return true;
        }
    }
    return false;
}

bool checkObstacleOverlap(const VectorXd state, const MultiAgentProblem2D& problem) {
    vector<Regions> polygons = findRegions(findEdges(problem));
    int m = problem.numAgents();
    Vector2d center;
    for (int i = 0; i < agents; i++) {
        for (const Regions& polygon : polygons) {
            center = {state(2*i), state(2*i + 1)};
            if (findClosestDistance(center, polygon) < radii(i)) return true;
        }
    }
    return false;
}

double findClosestDistance(const Vector2d state, const Regions& polyRegions) {
    bool left;
    int ind = 0;
    for (const vector<Edge>& region : polyRegions) {
        bool allPass = true;
        if (region.size() == 2) left = false;
        else left = true;
        ind++;
        for (const Edge& edge : region) {
            if (!checkLine(state, edge, left)) {
                allPass = false; 
                break;
            }
        }
        if (allPass) {
            Vector2d closestPoint;
            if (region.size() == 2) closestPoint = region[0].points.first;
            else closestPoint = closestPointOnLine(state, region[1]);
            return (state - closestPoint).norm();
        }
    }
    cout << "\nCRASHED\n";
    return 0;
}
