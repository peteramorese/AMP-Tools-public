#include "hw7helpers.h"
#include "AMPCore.h"

double DistanceMetric::distance(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) {
    return (lhs - rhs).norm();
}

bool MyPointCollisionChecker::inCollision(const Eigen::VectorXd& cspace_state) const {
    return false;
}

