#include "AMPCore.h"
#include "MyCollisionCheckers.h"
#include <random>
#include <set>
#include <Eigen/Dense>
#include "HelpfulClass.h"
#include <boost/geometry.hpp>

using std::vector, std::pair, Eigen::VectorXd;

bool MyCentralChecker::inCollision(const Eigen::VectorXd& state) const {
    if (radius !=0) return inCollisionSingle(state);
    if (central) return inCollisionCentral(state);
    else return inCollisionDecentral(state);
};

bool MyCentralChecker::inCollisionSingle(const Eigen::VectorXd& state) const {
    if (isPointInCollision(state, obstacles)) return true;
    for (const vector<vector<Edge>>& polyRegions : regions) {
        if (findClosestDistance(state, polyRegions) < radius) return true;
    }
    return false;
};

bool MyCentralChecker::inCollisionCentral(const Eigen::VectorXd& state) const {
    if (checkRobotOverlap(state, radii)) return true;
    for (int i = 0; i < problem.numAgents(); i++) {
        if (isPointInCollision({state(2*i), state(2*i+1)}, problem.obstacles)) return true;
        for (const vector<vector<Edge>>& polyRegions : regions) {
            if (findClosestDistance({state(2*i), state(2*i+1)}, polyRegions) < radii[i]) return true;
        }
    }
    return false;
}

bool MyCentralChecker::inCollisionDecentral(const Eigen::VectorXd& state) const {
    if (isPointInCollision(state, problem.obstacles)) return true;
    for (const vector<vector<Edge>>& polyRegions : regions) {
        if (findClosestDistance(state, polyRegions) < radii[computedPaths.size()]) return true;
    }
    if (checkWithPrior(state)) return true;
    if (avoidGoals(state)) return true;
    return false;
};

bool MyCentralChecker::checkWithPrior(const Eigen::Vector2d& state) const { 
    // cout << "\nNearest Start\n" << nearest.second << "\n";
    int steps = 50;
    int robotInd = computedPaths.size();
    int k = nearest.first;
    int m = 0;
    Eigen::Vector2d stateStart = nearest.second;
    Eigen::Vector2d prePathStart, prePathEnd, preStep, stateStep;
    // cout << "Checking k0\n";
    if (k==0 && checkK0condtion(state)) return true;
    // cout << "Done checking k0\n";

    for (const amp::Path& path : computedPaths) {
        if (k >= (path.waypoints.size() - 1)) {
            prePathStart = path.waypoints[path.waypoints.size() - 1];
            // cout << "Goal robot at \n" << prePathStart << "\n";
            preStep = {0, 0};
        } else {
            prePathStart = path.waypoints[k];
            prePathEnd = path.waypoints[k+1];
            preStep = (prePathEnd - prePathStart) / steps;
        }
        stateStep = (state - stateStart) / steps;
        VectorXd combinedState(4);
        // cout << "\nPrev Start\n" << prePathStart << "\nPrev End\n" << prePathEnd << "\nCurr Start\n" << stateStart << "\nCurr End\n" << stateEnd << "\n";
        for (int i = 0; i < steps; i++) {
            if (k >= (path.waypoints.size() - 1)) {
                // cout << "check overlap with \n" << prePathStart << "\n";
            }
            combinedState << prePathStart(0), prePathStart(1), stateStart(0), stateStart(1);
            if (checkRobotOverlap(combinedState, { radii[m] , radii[robotInd] })) return true;  
            prePathStart += preStep; 
            stateStart += stateStep; 
        }
        m++;
    }
    return false;
}

bool MyCentralChecker::checkK0condtion(const Eigen::Vector2d& state) const { 
    int steps = 50;
    Vector2d stateStart = nearest.second;
    Vector2d stateStep = (state - stateStart) / steps;
    int robotInd = computedPaths.size();
    int m = radii.size();
    VectorXd combinedState(2*m), combinedStateStep(2*m);
    // for (const CircularAgentProperties& agent : problem.agent_properties) {
    for (int i = 0; i < m; i++) {
        if (i == robotInd) {
            combinedState(2*i) = state(0);
            combinedState(2*i + 1) = state(1);      
            combinedStateStep(2*i) = stateStep(0);
            combinedStateStep(2*i + 1) = stateStep(1);         
        } else {
            combinedState(2*i) = problem.agent_properties[i].q_init(0);
            combinedState(2*i + 1) = problem.agent_properties[i].q_init(1);
            combinedStateStep(2*i) = 0;
            combinedStateStep(2*i + 1) = 0;
        }
    }
    vector<double> bigRadii = radii;
    for (double& element : bigRadii) element *= 1.5;
    for (int i = 0; i < 50; i++) {
        if (checkRobotOverlap(combinedState, bigRadii)) return true;
        stateStart += combinedStateStep; 
    }
    return false;
}

bool MyCentralChecker::avoidGoals(const Eigen::Vector2d& state) const { 
    int steps = 50;
    Vector2d stateStart = nearest.second;
    Vector2d stateStep = (state - stateStart) / steps;
    int robotInd = computedPaths.size();
    int m = radii.size();
    VectorXd combinedState(2*m), combinedStateStep(2*m);
    // for (const CircularAgentProperties& agent : problem.agent_properties) {
    for (int i = 0; i < m; i++) {
        if (i == robotInd) {
            combinedState(2*i) = state(0);
            combinedState(2*i + 1) = state(1);      
            combinedStateStep(2*i) = stateStep(0);
            combinedStateStep(2*i + 1) = stateStep(1);         
        } else {
            combinedState(2*i) = problem.agent_properties[i].q_goal(0);
            combinedState(2*i + 1) = problem.agent_properties[i].q_goal(1);
            combinedStateStep(2*i) = 0;
            combinedStateStep(2*i + 1) = 0;
        }
    }
    vector<double> bigRadii = radii;
    for (double& element : bigRadii) element *= 1;
    for (int i = 0; i < 50; i++) {
        if (checkRobotOverlap(combinedState, bigRadii)) return true;
        stateStart += combinedStateStep; 
    }
    return false;
}

void MyCentralChecker::addPath(const amp::Path& path) {
    computedPaths.push_back(path);
}

pair<VectorXd, VectorXd> MyCentralChecker::getLimits() {
    return limits;
}

bool MyKinoChecker::isValid(const vector<double>& state) const {
    for (int i = 0; i < state.size() - 2; ++i) {
        if (i == 2) continue;
        if (state[i] < stateLimits[i].first || state[i] > stateLimits[i].second) return false;
    }
    // if (isPointInCollision({state[0], state[1]}, ampObstacles)) return false;
    if (inCollisionRectangle(state)) return false;
    return true;
}


bool MyKinoChecker::inCollisionRectangle(const vector<double>& state) const {
    double cx = state[0];
    double cy = state[1];
    double theta = state[2];
    const double TR_x = cx + ((w / 2) * cos(theta)) - ((l / 2) * sin(theta));
    const double TR_y = cy + ((w / 2) * sin(theta)) + ((l / 2) * cos(theta));
    std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
    // TOP LEFT VERTEX:
    const double TL_x = cx - ((w / 2) * cos(theta)) - ((l / 2) * sin(theta));
    const double TL_y = cy - ((w / 2) * sin(theta)) + ((l / 2) * cos(theta));
    std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
    // BOTTOM LEFT VERTEX:
    const double BL_x = cx - ((w / 2) * cos(theta)) + ((l / 2) * sin(theta));
    const double BL_y = cy - ((w / 2) * sin(theta)) - ((l / 2) * cos(theta));
    std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
    // BOTTOM RIGHT VERTEX:
    const double BR_x = cx + ((w / 2) * cos(theta)) + ((l / 2) * sin(theta));
    const double BR_y = cy + ((w / 2) * sin(theta)) - ((l / 2) * cos(theta));
    std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);

    // convert to string for easy initializataion
    std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
    polygon agent;
    boost::geometry::read_wkt(points, agent);
    for (polygon obs: obstacles) {
        if (! boost::geometry::disjoint(agent, obs)) return true;
    }
    return false;
}

vector<std::pair<double, double>> MyKinoChecker::getLimits() {
    return stateLimits;
}