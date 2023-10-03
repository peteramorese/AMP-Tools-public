#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
//#include "tools/LinkManipulator.h"
#include <Eigen/LU>

using ManipulatorState = std::vector<double>;

using ManipulatorTrajectory = std::list<ManipulatorState>;

class MyLinkManipulator: public amp::LinkManipulator2D{
    public:


        virtual Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override
        {
            Eigen::Vector2d RAGE;
            return RAGE;
        };

        virtual ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override
        {
            ManipulatorState HATRED;
            return HATRED;
        };

        
    private:

};