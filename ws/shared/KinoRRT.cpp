#include "AMPCore.h"
#include "KinoRRT.h"
#include <random>
#include <Eigen/Dense>
#include "HelpfulClass.h"
#include <boost/numeric/odeint.hpp>

using namespace amp;
using std::vector, Eigen::VectorXd, Eigen::Vector2d, std::pair, std::size_t;

amp::Path KinoRRT::plan(const VectorXd& init_state, const vector<VectorXd>& goal_region, MyKinoChecker& collision_checker, int maxSamples, int& attemps, vector<int> workspaceIndicies) {
    Path path;
    int workspaceDim = workspaceIndicies.size(); 
    int m = controlLimits.size(); 
    path.valid = false;
    VectorXd qRand, uRand, nearest;
    points[0] = init_state;
    // cout << "Starting Node: \n" << init_state << std::endl;
    int ind = 1;
    int samples = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0, 1);    
    bool success = false;
    VectorXd u_best(m+1);
    while (samples < maxSamples) {
        samples++;
        double goalBias = dist(gen);
        qRand = getRandomPoint(collision_checker.getLimits());
        if (goalBias > (1 - p)) {
            VectorXd wRand = sampleFromRegion(goal_region);
            int i = 0;
            for (int index : workspaceIndicies){
                qRand(index) = wRand(i);
                i++;
            }
        }
        // cout << "q Point: \n" << wRand << std::endl;

        // cout << "Sampled Point: \n" << qRand << std::endl;

        pair<int, VectorXd> nearest = findNearest(qRand);
        VectorXd x_near = nearest.second;
        VectorXd x_best = x_near;
        bool validPath = false;
        double duration;
        // if (samples > 10000)
        //     cout << "Sampled Tree\n" << qRand << std::endl;
        for (int i = 0; i < 5; ++i) {
            // cout << "Sampling Control\n";
            uRand = getRandomPoint(controlLimits);
            VectorXd x_new = propagateState(x_near, uRand, duration, collision_checker);
            if (distanceMetric(qRand, x_new) < distanceMetric(qRand, x_best)) {
                if (samples > 10000)
                    // cout << "Extending Tree\n" << x_new << std::endl;
                x_best = x_new;
                u_best << uRand(0), uRand(1), uRand(2), uRand(3), duration;
                validPath = true;
            }
        }
        if (validPath) {
            parents[ind] = nearest.first;
            points[ind] = x_best;    
            controls[ind] = u_best;
            Eigen::VectorXd wBest(workspaceDim);  
            int i = 0;
            for (int index : workspaceIndicies) {
                wBest(i) = x_best(index);
                i++;
            }
            ind++;
            if (isPointInsideRegion(wBest, goal_region)) {
                cout << "Goal found in "<< samples << " samples\n";
                path.valid = true;
                break;
            }

        }
    }
    ind--;
    int node = ind;
    if (path.valid) {
        while (node != 0) {
            VectorXd vector1 = points[node];
            VectorXd vector2 = controls[node];
            vector1.conservativeResize(vector1.size() + vector2.size());
            vector1.tail(vector2.size()) = vector2;
            path.waypoints.insert(path.waypoints.begin(), vector1);
            node = parents[node];
        }
        path.waypoints.insert(path.waypoints.begin(), init_state);
        // path.waypoints.push_back(goal_state);
    } else cout << "Failed to find path in " << samples << " samples\n";
    attemps += samples;
    return path;
}

amp::Path KinoRRT::plan(const VectorXd& init_state, const VectorXd& goal_state, MyKinoChecker& collision_checker) {
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
    VectorXd u_best(3);
    while (points.size() < n) {
        double goalBias = dist(gen);
        if (goalBias > (1 - p)) qRand = goal_state;
        else qRand = getRandomPoint(collision_checker.getLimits());
        pair<int, VectorXd> nearest = findNearest(qRand);
        VectorXd x_near = nearest.second;
        VectorXd x_best = x_near;
        bool validPath = false;
        double duration;
        for (int i = 0; i < 5; ++i) {
            uRand = getRandomPoint(controlLimits);
            VectorXd x_new = propagateState(x_near, uRand, duration, collision_checker);
            if (distanceMetric(qRand, x_new) < distanceMetric(qRand, x_best)) {
                x_best = x_new;
                u_best << uRand(0), uRand(1), duration;
                validPath = true;
            }
        }
        if (validPath) {
            parents[ind] = nearest.first;
            points[ind] = x_best;    
            controls[ind] = u_best;    
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
            VectorXd vector1 = points[node];
            VectorXd vector2 = controls[node];
            vector1.conservativeResize(vector1.size() + vector2.size());
            vector1.tail(vector2.size()) = vector2;
            path.waypoints.insert(path.waypoints.begin(), vector1);
            node = parents[node];
        }
        path.waypoints.insert(path.waypoints.begin(), init_state);
        // path.waypoints.push_back(goal_state);
    } else cout << "Failed to find path\n";
    return path;
}

pair<int, VectorXd> KinoRRT::findNearest(const VectorXd& point) {
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
    int dim = state1.size();
    if (dim == 2) {
        return sqrt(pow(state1(0) - state2(0), 2) + pow(state1(1) - state2(1), 2));
    } else {
        return sqrt(pow(state1(0) - state2(0), 2) + pow(state1(1) - state2(1), 2) + pow(state1(2) - state2(2), 2));
    }
}

VectorXd KinoRRT::propagateState(const VectorXd& x_start, const VectorXd& u, double& duration, MyKinoChecker& collision_checker) {
    // cout << "\nStart\n" << x_start;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.05, 1.5);
    std::vector<double> state = convertEigenToStd(x_start);
    int m = u.size();
    double dt = 0.05;
    auto dynamicsFunc = (dynamicsModel == QUADROTOR) ? quadrotorDynamics : carDynamics;
    for (int i = 0; i < m; i++) 
        state.push_back(u(i));
    boost::numeric::odeint::runge_kutta_dopri5<std::vector<double>> stepper;
    duration = 0.0;
    while (duration < dis(gen)) {
        stepper.do_step(dynamicsFunc, state, duration, dt);
        if (!collision_checker.isValid(state, m)) return x_start;
        duration += dt;
    }
    for (int i = 0; i < m; i++) 
        state.pop_back();
    return convertStdToEigen(state);
}
