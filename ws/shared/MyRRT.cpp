#include "AMPCore.h"
#include "MyRRT.h"
#include <random>
#include <Eigen/Dense>
#include "HelpfulClass.h"

using namespace amp;
using std::vector, Eigen::VectorXd, Eigen::Vector2d, std::pair, std::size_t;

amp::Path MyGenericRRT::plan(const VectorXd& init_state, const VectorXd& goal_state, const MyCentralChecker& collision_checker) {
    Path path;
    VectorXd qRand, nearest;
    points[0] = init_state;
    std::map<int, int> parents;
    int ind = 1;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0, 1);    
    bool success = false;
    while (points.size() < n) {
        double goalBias = dist(gen);
        if (goalBias > (1 - p)) qRand = goal_state;
        else qRand = getRandomPoint();
        pair<int, VectorXd> nearest = findNearest(qRand, collision_checker);
        if (nearest.first != -1) {
            points[ind] = nearest.second;
            parents[ind] = nearest.first;
            // cout << "Point added sample: \n" << nearest.second << std::endl;
            // path.waypoints.push_back(nearest.second);
            ind++;
            if ((nearest.second - goal_state).norm() < 3) {
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
    
    // if (smooth) smoothPath(path, problem.obstacles);
    return path;
}

pair<int, VectorXd> MyGenericRRT::findNearest(const VectorXd& point, const MyCentralChecker& collision_checker) {
    VectorXd nearest = points[0];
    int ind = 0;
    for (int i = 1; i < points.size(); i++) {
        if ((point - points[i]).norm() < (point - nearest).norm()) {
            nearest = points[i];
            ind = i;
        }
    }
    // cout << "Nearest: \n"<< nearest << std::endl;
    VectorXd step = (point - nearest) / (point - nearest).norm() * r;
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

bool MyCentralChecker::inCollision(const Eigen::VectorXd& state) const {
    if (checkRobotOverlap(state, radii)) return true;
    for (int i = 0; i < problem.numAgents(); i++) {
        if (isPointInCollision({state(2*i), state(2*i+1)}, problem.obstacles)) return true;
        for (const vector<vector<Edge>>& polyRegions : regions) {
            if (findClosestDistance({state(2*i), state(2*i+1)}, polyRegions) < radii[i]) return true;
        }
    }
    return false;

};
