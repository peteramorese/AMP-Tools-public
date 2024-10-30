#include "MyKinoRRT.h"

bool MyStatePropagator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
    return true;
};

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem) {
    amp::KinoPath path;
    path.waypoints.push_back(problem.q_init);
    for (int i = 0; i < 10; i++) {
        Eigen::VectorXd waypoint = Eigen::VectorXd::Random(problem.q_init.size());
        path.waypoints.push_back(waypoint + problem.q_init);
        path.controls.push_back(Eigen::VectorXd::Random(problem.q_init.size()));
        path.durations.push_back(amp::RNG::randf(0, 1.0));
    }
    return path;
}
