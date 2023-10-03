#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <Eigen/LU>

using ManipulatorState = std::vector<double>;

using ManipulatorTrajectory = std::list<ManipulatorState>;

class MyLinkManipulator: public amp::LinkManipulator2D{
    public:

        virtual Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override
        {
            Eigen::Vector2d peace;                 
            return peace;
        };

        virtual ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override
        {
            ManipulatorState calm;
            return calm;
        };

        void printLinkLengths(){
            for(int j = 0; j < m_link_lengths.size(); j++){
                std::cout << "link number: " << j << " link length: " << m_link_lengths[j] << std::endl;
            }
        }

        
    private:

};