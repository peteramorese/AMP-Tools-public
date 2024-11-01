#include "MyKinoRRT.h"

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
};

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    amp::KinoPath path;
    Eigen::VectorXd state = problem.q_init;
    path.waypoints.push_back(state);
    for (int i = 0; i < 10; i++) {
        Eigen::VectorXd control = Eigen::VectorXd::Random(problem.q_init.size());
        agent.propagate(state, control, 1.0);
        path.waypoints.push_back(state);
        path.controls.push_back(control);
        path.durations.push_back(1.0);
    }
    path.valid = true;
    return path;
}
