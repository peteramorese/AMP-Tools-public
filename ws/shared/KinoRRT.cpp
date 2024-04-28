#include "AMPCore.h"
#include "KinoRRT.h"
#include <random>
#include <Eigen/Dense>
#include "HelpfulClass.h"
#include <boost/numeric/odeint.hpp>

using namespace amp;
using std::vector, Eigen::VectorXd, Eigen::Vector2d, std::pair, std::size_t;

amp::Path KinoRRT::plan(const VectorXd& init_state, const vector<Vector2d>& goal_region, MyKinoChecker& collision_checker, double& attemps) {
    Path path;
    path.valid = false;
    VectorXd qRand, uRand, nearest;
    points[0] = init_state;
    // cout << "Starting Node: \n" << init_state << std::endl;
    int m = init_state.size() / 2;
    int ind = 1;
    int samples = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0, 1);    
    bool success = false;
    VectorXd u_best(3);
    while (samples < n) {
        samples++;
        double goalBias = dist(gen);
        qRand = getRandomPoint(collision_checker.getLimits());
        if (goalBias > (1 - p)) {
            Vector2d dRand = sampleFromRegion(goal_region);
            qRand(0) = dRand.x();
            qRand(1) = dRand.y();
        }
        pair<int, VectorXd> nearest = findNearest(qRand);
        VectorXd x_near = nearest.second;
        VectorXd x_best = x_near;
        bool validPath = false;
        double duration;
        for (int i = 0; i < 5; ++i) {
            // cout << "Sampling Control\n";
            uRand = getRandomPoint(controlLimits);
            VectorXd x_new = propagateState(x_near, uRand, duration, collision_checker);
            if (distanceMetric(qRand, x_new) < distanceMetric(qRand, x_best)) {
                // cout << "Extending Tree\n";
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
            if (isPointInsidePolygon(x_best, goal_region)) {
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
    return sqrt(pow(state1(0) - state2(0), 2) + pow(state1(1) - state2(1), 2));
}

// Euler integration method
void integrateEuler(const std::function<void(const VectorXd&, const VectorXd&, VectorXd&)>& dynamics,
                    VectorXd& state,
                    const VectorXd& control,
                    double dt) {
    VectorXd state_dot(state.size());
    dynamics(state, control, state_dot);
    state += state_dot * dt;
}

void dynamics(const std::vector<double>& state, std::vector<double>& state_dot, const double /* time */) {
    double theta = state[2];
    double v = state[3];
    double phi = state[4];
    state_dot[0] = v * cos(theta);
    state_dot[1] = v * sin(theta);
    state_dot[2] = (v / 0.5) * tan(phi);
    state_dot[3] = state[5];
    state_dot[4] = state[6];
    state_dot[5] = 0;
    state_dot[6] = 0;
}

VectorXd KinoRRT::propagateState(const VectorXd& x_start, const VectorXd& u, double& duration, MyKinoChecker& collision_checker) {
    // cout << "\nStart\n" << x_start;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.5, 1.5);
    double dt = 0.1;
    std::vector<double> state = convertEigenToStd(x_start);
    state.push_back(u(0));
    state.push_back(u(1));
    boost::numeric::odeint::runge_kutta_dopri5<std::vector<double>> stepper;
    duration = 0.0;
    while (duration < dis(gen)) {
        // integrateEuler(dynamics, state, u, dt);
        stepper.do_step(dynamics, state, duration, dt);
        if (!collision_checker.isValid(state)) return x_start;
        // Do something with the updated state
        duration += dt;
    }
    state.pop_back();
    state.pop_back();
    return convertStdToEigen(state);
}
