#pragma once
#include "AMPCore.h"
#include "HelpfulClass.h"
#include "MyCollisionCheckers.h"

using namespace amp;
using std::vector, Eigen::VectorXd, std::pair;

enum DynamicsModel {
    CAR,
    QUADROTOR
};

class KinoRRT {
    public:
        KinoRRT(int n, double r, double p, vector<pair<double, double>> controlLimits, DynamicsModel dynamicsModel)
        : n(n), r(r), p(p), controlLimits(controlLimits), dynamicsModel(dynamicsModel) {}
        amp::Path plan(const VectorXd& init_state, const VectorXd& goal_state, MyKinoChecker& collision_checker); 
        amp::Path plan(const VectorXd& init_state, const vector<Eigen::VectorXd>& goal_region, MyKinoChecker& collision_checker, int maxSamples, int& attemps, vector<int> workspaceIndicies); 
        VectorXd getRandomPoint(const vector<pair<double, double>>& limits);
        VectorXd propagateState(const VectorXd& x_start, const VectorXd& u, double& duration, MyKinoChecker& collision_checker);
        double distanceMetric(const VectorXd& state1, const VectorXd& state2);
        pair<int, Eigen::VectorXd> findNearest(const Eigen::VectorXd& point);
        void clearRRT() {
            points.clear();
            parents.clear();
            controls.clear();
        }
    private:
        int n;
        double r, p;
        double eps = 0.5;
        std::map<uint32_t, VectorXd> points;
        std::map<uint32_t, uint32_t> parents;
        std::map<uint32_t, VectorXd> controls;
        vector<pair<double, double>> controlLimits;
        DynamicsModel dynamicsModel;
        static void carDynamics(const std::vector<double>& state, std::vector<double>& state_dot, const double /* time */) {
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

        static void quadrotorDynamics(const std::vector<double>& state, std::vector<double>& state_dot, const double /* time */) {
            double k1 = -0.0104;
            double k2 = 0.04167;
            state_dot[0] = state[1];
            state_dot[1] = k1 * state[1] - k2 * state[6] + k2 * state[8];
            state_dot[2] = state[3];
            state_dot[3] = k1 * state[3] - k2 * state[7] + k2 * state[9];
            state_dot[4] = state[5];
            state_dot[5] = 2 * k1 * state[5] + 0.4 * (state[6] + state[7] + state[8] + state[9]);
            state_dot[6] = 0;
            state_dot[7] = 0;
            state_dot[8] = 0;
            state_dot[9] = 0;
        }
};