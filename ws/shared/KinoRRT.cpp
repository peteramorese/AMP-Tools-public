#include "AMPCore.h"
#include "KinoRRT.h"
#include <random>
#include <Eigen/Dense>
#include "HelpfulClass.h"

using namespace amp;
using std::vector, Eigen::VectorXd, Eigen::Vector2d, std::pair, std::size_t;

amp::Path KinoRRT::plan(const VectorXd& init_state, const VectorXd& goal_state, MyCentralChecker& collision_checker) {
    Path path;
    path.valid = false;
    VectorXd qRand, uRand, nearest;
    points[0] = init_state;
    int m = init_state.size() / 2;
    int ind = 1;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0, 1);    
    bool success = false;
    while (points.size() < n) {
        double goalBias = dist(gen);
        if (goalBias > (1 - p)) qRand = goal_state;
        else qRand = getRandomPoint(collision_checker.getLimits());
        pair<int, VectorXd> nearest = findNearest(qRand, collision_checker);
        VectorXd x_near = nearest.second;
        VectorXd x_best = x_near;
        bool validPath = false;
        for (int i = 0; i < 5; ++i) {
            uRand = getRandomPoint(controlLimits());
            VectorXd x_new = propagateState(nearest.second, uRand, 1);
            if (distanceMetric(qRand, x_new) < distanceMetric(qRand, x_best)) {
                x_best = x_new;
                validPath = true;
            }
        }
        if (validPath) {
            parents[ind] = nearest.first;
            points[ind] = x_best;           
            ind++;
            if (distanceMetric(x_best, goal_state) < eps) {
                cout << "Goal found in "<< ind << " steps\n";
                path.valid = true;
                break;
            }
        }

    }
    ind--;
    int node = ind;
    if (path.valid) {
        while (node != 0) {
            path.waypoints.insert(path.waypoints.begin(), points[node]);
            node = parents[node];
        }
        path.waypoints.insert(path.waypoints.begin(), init_state);
        path.waypoints.push_back(goal_state);
    } else cout << "Failed to find path\n";
    return path;
}

pair<int, VectorXd> KinoRRT::findNearest(const VectorXd& point, MyCentralChecker& collision_checker) {
    VectorXd nearest = points[0];
    int ind = 0;
    for (int i = 1; i < points.size(); i++) {
        if (distanceMetric(point, points[i]) < distanceMetric(point, nearest)) {
            nearest = points[i];
            ind = i;
        }
    }
    return {ind, nearest};
}

VectorXd KinoRRT::getRandomPoint(const vector<pair<double, double>>& limits) {
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

double KinoRRT::distanceMetric(const VectorXd& state1, const VectorXd& state2) {
    return sqrt(pow(state1[0] - state2[0], 2) + pow(state1[1] - state2[1], 2));
}

VectorXd KinoRRT::propagateState(const VectorXd& x_start, const VectorXd& u, double deltaT) {
    return x_start;
}