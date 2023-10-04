#include "MyLinkManipulator.h"
#include <Eigen/Dense>

using std::vector, std::string, std::cout, Eigen::Vector2d;

Eigen::Matrix<double, 3, 3> getTMatrix(double length, double theta) {
    Eigen::Matrix<double, 3, 3> T; // 3x2 matrix

    // Fill the matrices A and B with values (example values)
    T << cos(theta), -sin(theta), length,
         sin(theta), cos(theta), 0,
         0, 0, 1;

    return T;
}

MyLinkManipulator::MyLinkManipulator() : LinkManipulator2D() {}

MyLinkManipulator::MyLinkManipulator(const std::vector<double>& link_lengths) 
    : LinkManipulator2D(uint32_t) {}


Vector2d MyLinkManipulator::getJointLocation(const ManipulatorState& state, uint32_t joint_index) const  {
    Vector2d jointLocation;
    if (joint_index == 0) {
        Eigen::Matrix<double, 3, 3> T1 = getTMatrix(link_lengths(0))
        jointLocation = {0.0, 0.0};
    } 
    else if (joint_index == 1) {
        jointLocation = {1.0, 0.0};
    } 
    else {
        jointLocation = {1.0, 2.0};
    }
    cout << jointLocation(0) << ", " << jointLocation(1) << "\n";
    return jointLocation;
};

ManipulatorState MyLinkManipulator::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    ManipulatorState state;
    return state;
};
