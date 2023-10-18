#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <Eigen/LU>
#include <cassert>

// using ManipulatorState = std::vector<double>;

// using ManipulatorTrajectory = std::list<ManipulatorState>;
using ManipulatorState = Eigen::VectorXd;
/// @brief For the specific 2-link case, use Eigen::Vector2d to make it consistent with other 2D planning problems
using ManipulatorState2Link = Eigen::Vector2d;

/// @brief List of manipulator states in chronological order
using ManipulatorTrajectory = amp::Path;
/// @brief For the specific 2-link case, use Path2D to make it consistent with other 2D planning problems
using ManipulatorTrajectory2Link = amp::Path2D;

class MyLinkManipulator: public amp::LinkManipulator2D{
    public:
        MyLinkManipulator(){
            std::vector<double> links(2, 1.0);
            getBaseLocation() = Eigen::Vector2d(0.0, 0.0);
            getLinkLengths() = links;
        }
        MyLinkManipulator(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths){
            getBaseLocation() = base_location;
            getLinkLengths() = link_lengths;
        }
        /// @brief Get the location of the nth joint using the current link attributes using Forward Kinematics
        /// @param state Joint angle state (radians). Must have size() == nLinks()
        /// @param joint_index Joint index in order of base to end effector 
        /// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
        /// @return Joint coordinate
        virtual Eigen::Vector2d getJointLocation(const ManipulatorState& stateE, uint32_t joint_index) const override
        {
           std::vector<double> state(stateE.data(), stateE.data() + stateE.size());
            assert(state.size() == nLinks());
            Eigen::Vector3d  location(getBaseLocation()[0], getBaseLocation()[1], 1);
            if(joint_index > 0){
                location = getR3(0, getLinkLengths()[joint_index - 1])*location;
                for(int j = joint_index - 1; j > 0; j--){
                    location = getR3(state[j], getLinkLengths()[j-1])*location;   
                }
                // for joint 1, use length 0 and angle[0]
                location = getR3(state[0], 0)*location;
            }
            return location.head<2>();
        };
        /// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
        /// @param end_effector_location End effector coordinate
        /// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()
        virtual ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override
        {
            std::vector<double> state;
            Eigen::Vector2d diffVec = (end_effector_location - getBaseLocation());
            switch(nLinks()){
                case 1:
                    {
                        //check that end effector is on circle of radius linklength from base location
                        if(diffVec.norm() == getLinkLengths()[0]){
                            state.push_back(atan2(diffVec[1],diffVec[0]));
                        }
                        else{
                            std::cout << "1-link arm cannot reach position " << end_effector_location << std::endl;
                        }
                    }
                    break;
                case 2:
                    {
                        //check that end effector is within reachable band of 2-link arm
                        if(diffVec.norm() >= abs(getLinkLengths()[1] - getLinkLengths()[0])){
                            double theta2 = acos((std::pow(end_effector_location.norm(),2) - (std::pow(getLinkLengths()[0],2) + std::pow(getLinkLengths()[1],2)))
                                /(2*getLinkLengths()[0]*getLinkLengths()[1]));
                            double theta1 = acos((end_effector_location[0]*(getLinkLengths()[0] + getLinkLengths()[1]*cos(theta2))
                                + end_effector_location[1]*getLinkLengths()[1]*sin(theta2))/std::pow(end_effector_location.norm(),2));
                            state.push_back(theta1);
                            state.push_back(theta2);
                        }
                        else{
                            std::cout << "2-link arm cannot reach position " << end_effector_location << std::endl;
                        }
                    }
                    break;
                case 3:
                    {
                        double theta3 = atan2(diffVec[1],diffVec[0]);
                        Eigen::Vector2d circVec(getLinkLengths()[2]*cos(theta3),getLinkLengths()[2]*sin(theta3));
                        Eigen::Vector2d circlePoint = end_effector_location - circVec;
                        double a = 2*diffVec[0];
                        double b = 2*diffVec[1];
                        //outer circ
                        double c1 = std::pow(getLinkLengths()[0] + getLinkLengths()[1],2) - std::pow(getBaseLocation()[0],2)
                        - std::pow(getBaseLocation()[1],2) - std::pow(getLinkLengths()[2],2) + std::pow(end_effector_location[0],2)
                        + std::pow(end_effector_location[1],2);
                        //inner circ (a1 != a2)
                        double c2 = std::pow(getLinkLengths()[0] - getLinkLengths()[1],2) - std::pow(getBaseLocation()[0],2)
                        - std::pow(getBaseLocation()[1],2) - std::pow(getLinkLengths()[2],2) + std::pow(end_effector_location[0],2)
                        + std::pow(end_effector_location[1],2);
                        double discriminant1 = std::pow(getLinkLengths()[0] + getLinkLengths()[1],2)*(std::pow(a,2) + std::pow(b,2)) - std::pow(c1,2);
                        double discriminant2 = std::pow(getLinkLengths()[0] - getLinkLengths()[1],2)*(std::pow(a,2) + std::pow(b,2)) - std::pow(c2,2);
                        if(discriminant1 >= 0){
                            // std::cout << "disc1" << std::endl;
                            double rad = sqrt(discriminant1);
                            circlePoint[0] = (a*c1 + b*rad)/(std::pow(a,2) + std::pow(b,2));
                            circlePoint[1] = (b*c1 - a*rad)/(std::pow(a,2) + std::pow(b,2));
                        }
                        else if(discriminant2 >= 0){
                            // std::cout << "disc2" << std::endl;
                            double rad = sqrt(discriminant2);
                            circlePoint[0] = (a*c2 + b*rad)/(std::pow(a,2) + std::pow(b,2));
                            circlePoint[1] = (b*c2 - a*rad)/(std::pow(a,2) + std::pow(b,2));
                        }
                        // std::cout << "circlePoint " << circlePoint << std::endl;
                        theta3 = atan2(end_effector_location[1] - circlePoint[1],end_effector_location[0] - circlePoint[0]);
                        double inputC = (std::pow(circlePoint.norm(),2) - (std::pow(getLinkLengths()[0],2) + std::pow(getLinkLengths()[1],2)))
                            /(2*getLinkLengths()[0]*getLinkLengths()[1]);
                        
                        double check = 1-std::pow(inputC,2);
                        if(check < 1e-10){
                            check = 0;
                        }
                        double inputS = sqrt(check);
                        double theta2 = atan2(inputS,inputC);
                        inputC = (circlePoint[0]*(getLinkLengths()[0] + getLinkLengths()[1]*cos(theta2))
                            + circlePoint[1]*getLinkLengths()[1]*sin(theta2))/std::pow(circlePoint.norm(),2);
                        inputS = (circlePoint[1]*(getLinkLengths()[0] + getLinkLengths()[1]*cos(theta2))
                            - circlePoint[0]*getLinkLengths()[1]*sin(theta2))/std::pow(circlePoint.norm(),2);
                        double theta1 = atan2(inputS,inputC);
                        state.push_back(theta1);
                        state.push_back(theta2);
                        state.push_back(theta3-theta2-theta1);
                    }
                    break;
                default:
                    std::cout << "Too many links for inverse kinematics :) " << std::endl;
            }
            double* ptr = &state[0];
            Eigen::Map<Eigen::VectorXd> stateE(ptr, state.size());
            return stateE;
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