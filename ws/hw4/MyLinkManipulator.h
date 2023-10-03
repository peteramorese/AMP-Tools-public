#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <Eigen/LU>

using ManipulatorState = std::vector<double>;

using ManipulatorTrajectory = std::list<ManipulatorState>;

class MyLinkManipulator: public amp::LinkManipulator2D{
    public:
        /// @brief Get the location of the nth joint using the current link attributes using Forward Kinematics
        /// @param state Joint angle state (radians). Must have size() == nLinks()
        /// @param joint_index Joint index in order of base to end effector 
        /// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
        /// @return Joint coordinate
        virtual Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override
        {
            Eigen::Vector3d  location(getBaseLocation()[0], getBaseLocation()[1], 1);
            if(joint_index > 0){
                // for joint 1, use length 0 and angle[0]
                for(int j = 0; j < joint_index; j++){
                    location = getR3(state[j], getLinkLengths()[j])*location;                    
                }
            }
            return location.head<2>();
        };
        /// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
        /// @param end_effector_location End effector coordinate
        /// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()
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
        /// @brief Get rotation matrix for R3 rotation for given angle
        /// @param x joint angle relative to last joint
        /// @param len length of last joint
        /// @return rotation matrix
        Eigen::Matrix3d getR3(double x, double len = 0) const{
            Eigen::Matrix3d R3;
            R3 <<   cos(x), -sin(x), len,
                    sin(x), cos(x),  0.0, 
                    0.0,    0.0,     1.0;
            return R3;
        }

        
    private:

};