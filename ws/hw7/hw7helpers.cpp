#include "hw7helpers.h"
#include "AMPCore.h"

/// @brief Compute the Euclidean distance between two states in configuration space
/// @param lhs First state
/// @param rhs Second state
/// @return Euclidean distance between the two states
double DistanceMetric::distance(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) {
    return (lhs - rhs).norm();
}

/// @brief Check if a state in configuration space is colliding with an obstacle
/// @param cspace_state N-dimensional C-space state
/// @return `true` if the the state is in collision, `false` if it is not
bool MyPointCollisionChecker::inCollision(const Eigen::VectorXd& cspace_state) const {
    return false;
}

