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
    int breakerr = 0;
    while (points.size() < n) {
        double goalBias = dist(gen);
        if (goalBias > (1 - p)) qRand = goal_state;
        else qRand = getRandomPoint(collision_checker.getLimits());
        pair<int, VectorXd> nearest = findNearest(qRand, collision_checker);
        breaker++;
        if (nearest.second == init_state) breakerr++;
        else breakerr = 0;
        if (breaker > 2*n) break; 
        if (breakerr > 20) break; 

        if (nearest.first != -1) {
            parents[ind] = nearest.first;
            points[ind] = nearest.second;
            ind++;
            if ((nearest.second - goal_state).norm() < eps) {
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
    } else { cout << "Failed to find path\n";}
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
    // cout << "K = " << k << "\n";
    collision_checker.nearest = {k, nearest};
    VectorXd step = (point - nearest) / (point - nearest).norm() * r ;
    if (collision_checker.central) step = step / 50;
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

VectorXd MyGenericRRT::getRandomPoint(const pair<VectorXd, VectorXd>& limits) {
    VectorXd lower = limits.first;
    VectorXd upper = limits.second;
    int dim = lower.size();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::pair<double, double> limit;
    VectorXd randomPoint(dim);
    for (int i = 0; i < dim; i++) {
        std::uniform_real_distribution<double> dist(lower(i), upper(i));
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

