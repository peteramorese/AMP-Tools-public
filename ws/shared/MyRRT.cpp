#include "AMPCore.h"
#include "MyRRT.h"
#include <random>
#include <Eigen/Dense>
#include "HelpfulClass.h"

using namespace amp;
using std::vector, Eigen::VectorXd, Eigen::Vector2d, std::pair, std::size_t;

amp::Path MyGenericRRT::plan(const VectorXd& init_state, const VectorXd& goal_state, MyCentralChecker& collision_checker) {
    Path path;
    VectorXd qRand, nearest;
    points[0] = init_state;
    int m = init_state.size() / 2;
    int ind = 1;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0, 1);    
    bool success = false;
    int breaker = 0;
    while (points.size() < n) {
        double goalBias = dist(gen);
        if (goalBias > (1 - p)) qRand = goal_state;
        else qRand = getRandomPoint();
        pair<int, VectorXd> nearest = findNearest(qRand, collision_checker);
        breaker++;
        if (breaker > 2*n) break; 

        if (nearest.first != -1) {
            parents[ind] = nearest.first;
            points[ind] = nearest.second;
            ind++;
            if ((nearest.second - goal_state).norm() < 1) {
                cout << "Goal found in "<< ind << " steps\n";
                success = true;
                break;
            }
        }

    }
    ind--;
    int node = ind;
    if (success) {
        while (node != 0) {
            path.waypoints.insert(path.waypoints.begin(), points[node]);
            node = parents[node];
        }
        path.waypoints.insert(path.waypoints.begin(), init_state);
        path.waypoints.push_back(goal_state);
    } 
    treeSize = points.size();
    // if (smooth) smoothPath(path, problem.obstacles);
    return path;
}

pair<int, VectorXd> MyGenericRRT::findNearest(const VectorXd& point, MyCentralChecker& collision_checker) {
    VectorXd nearest = points[0];
    int ind = 0;
    for (int i = 1; i < points.size(); i++) {
        if ((point - points[i]).norm() < (point - nearest).norm()) {
            nearest = points[i];
            ind = i;
        }
    }
    // cout << "\n Nearest Point: \n" << nearest << "\n";
    // cout << "Q Rand Sample: \n" << point << "\n";
    // cout << "Ind = " << ind << "\n";
    int k = findStepsToRoot(ind);
    cout << "K = " << k << "\n\n";
    collision_checker.nearest = {k, nearest};
    VectorXd step = (point - nearest) / (point - nearest).norm() * r ;
    VectorXd endpoint = nearest;

    int i = 0;
    while ((point - nearest).norm() > (endpoint - nearest).norm()) {
        endpoint += step;
        if (collision_checker.inCollision(endpoint)) { 
            if (i == 0) ind = -1;
            break;
        }
        i++;
    }
    endpoint -= step;  
    return {ind, endpoint};
}

VectorXd MyGenericRRT::getRandomPoint() {
    int dim = limits.size();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::pair<double, double> limit;
    VectorXd randomPoint(dim);
    for (int i = 0; i < dim; i++) {
        std::uniform_real_distribution<double> dist(limits[i].first, limits[i].second);
        randomPoint(i) = dist(gen);
    }
    return randomPoint;
}

int MyGenericRRT::findStepsToRoot(int node) { 
    int k = 0;
    while (node != 0) {
        node = parents[node];
        k++;
    }
    return k;
}

bool MyCentralChecker::inCollision(const Eigen::VectorXd& state) const {
    if (central) return inCollisionCentral(state);
    else return inCollisionDecentral(state);
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
    return false;
};

bool MyCentralChecker::checkWithPrior(const Eigen::Vector2d& state) const { 
    // cout << "\nNearest Start\n" << nearest.second << "\n";
    int steps = 50;
    int robotInd = computedPaths.size();
    int k = nearest.first;
    int m = 0;
    Vector2d stateStart = nearest.second;
    Vector2d prePathStart, prePathEnd, preStep, stateStep;
    if (k==0) if (checkK0condtion(state)) return true;
    for (const Path& path : computedPaths) {
        if (k >= (path.waypoints.size() - 1)) {
            prePathStart = path.waypoints[path.waypoints.size() - 1];
            cout << "Goal robot at \n" << prePathStart << "\n";
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

void MyCentralChecker::addPath(const Path& path) {
    computedPaths.push_back(path);
}
