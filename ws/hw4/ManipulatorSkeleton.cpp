#include "ManipulatorSkeleton.h"


MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)
    std::vector<Eigen::Vector2d> joint_positions = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(1.0, 1.0)};
    return joint_positions[joint_index];
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here

    amp::ManipulatorState joint_angles;
    joint_angles.setZero();
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {

        return joint_angles;
    } else if (nLinks() == 3) {

        return joint_angles;
    } else {

        return joint_angles;
    }

    return joint_angles;
}